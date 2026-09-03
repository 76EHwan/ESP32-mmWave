#!/usr/bin/env python3
"""git 브랜치 파서.

두 가지 입력 경로를 지원한다.

  1. 저장소에서 직접 수집 (기본)
     `git for-each-ref` 를 구분자와 함께 돌린다. 필드 안에 구분자가 들어갈 일이
     없으므로 파싱이 모호해지지 않는다. 가능하면 이 쪽을 쓴다.

  2. `git branch -vv` 텍스트 파싱 (--stdin)
     이미 떠 있는 출력이나 로그에 남은 출력을 그대로 넣고 싶을 때 쓴다.
     사람이 읽으라고 만든 형식이라 커밋 제목이 대괄호로 시작하면 업스트림
     표기와 헷갈릴 수 있다. 그래서 대괄호 안이 업스트림처럼 생겼을 때만
     (공백 없는 refname + ahead/behind/gone) 업스트림으로 본다.

사용 예:
    python tools/git_branch_parser.py                 # 로컬 브랜치 표로 출력
    python tools/git_branch_parser.py -a --json       # 원격 포함 JSON
    python tools/git_branch_parser.py --gone          # 업스트림이 사라진 것만
    python tools/git_branch_parser.py --stale 30      # 30일 이상 방치된 것만
    git branch -vv | python tools/git_branch_parser.py --stdin
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Iterable, List, Optional, Tuple

# for-each-ref 필드 구분자. refname 이나 커밋 제목에 나올 일이 없는 제어문자를 쓴다.
SEP = "\x1f"
REC = "\x1e"


@dataclass
class Branch:
    """브랜치 하나."""

    name: str                        # 로컬은 "feature/x", 원격은 "origin/main"
    sha: str = ""                    # 커밋 해시 (짧은 형태)
    subject: str = ""                # 커밋 제목
    is_current: bool = False         # 현재 체크아웃된 브랜치인가
    is_remote: bool = False          # 원격 추적 브랜치인가
    is_worktree: bool = False        # 다른 워크트리가 물고 있는가 (git branch 의 + 마커)
    is_detached: bool = False        # detached HEAD 줄인가
    upstream: Optional[str] = None   # 업스트림 refname
    ahead: int = 0
    behind: int = 0
    gone: bool = False               # 업스트림이 삭제됨
    committed: Optional[str] = None  # 마지막 커밋 시각 (ISO8601, for-each-ref 경로만)
    author: Optional[str] = None     # 마지막 커밋 작성자 (for-each-ref 경로만)
    alias_of: Optional[str] = None   # "origin/HEAD -> origin/master" 의 대상

    @property
    def age_days(self) -> Optional[float]:
        """마지막 커밋으로부터 지난 날짜. 시각 정보가 없으면 None."""
        if not self.committed:
            return None
        try:
            when = datetime.fromisoformat(self.committed)
        except ValueError:
            return None
        if when.tzinfo is None:
            when = when.replace(tzinfo=timezone.utc)
        return (datetime.now(timezone.utc) - when).total_seconds() / 86400.0


# ---------------------------------------------------------------------------
# 1. 저장소에서 직접 수집
# ---------------------------------------------------------------------------

_FOR_EACH_REF_FORMAT = SEP.join(
    [
        "%(refname)",
        "%(objectname:short)",
        "%(upstream)",
        "%(upstream:track,nobracket)",
        "%(committerdate:iso-strict)",
        "%(authorname)",
        "%(symref)",
        "%(contents:subject)",
    ]
) + REC


def _run_git(repo: str, args: List[str]) -> str:
    proc = subprocess.run(
        ["git", "-C", repo] + args,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if proc.returncode != 0:
        raise RuntimeError((proc.stderr or proc.stdout).strip())
    return proc.stdout


def _current_branch(repo: str) -> Optional[str]:
    """현재 브랜치 이름. detached HEAD 면 None."""
    try:
        name = _run_git(repo, ["symbolic-ref", "--quiet", "--short", "HEAD"]).strip()
    except RuntimeError:
        return None
    return name or None


def _worktree_branches(repo: str) -> set:
    """다른 워크트리가 체크아웃 중인 브랜치 이름들."""
    out = _run_git(repo, ["worktree", "list", "--porcelain"])
    names = set()
    for line in out.splitlines():
        if line.startswith("branch "):
            names.add(line[len("branch "):].strip().replace("refs/heads/", "", 1))
    return names


def _parse_track(track: str) -> Tuple[int, int, bool]:
    """upstream:track 문자열을 (ahead, behind, gone) 으로."""
    if not track:
        return 0, 0, False
    if "gone" in track:
        return 0, 0, True
    ahead = behind = 0
    m = re.search(r"ahead (\d+)", track)
    if m:
        ahead = int(m.group(1))
    m = re.search(r"behind (\d+)", track)
    if m:
        behind = int(m.group(1))
    return ahead, behind, False


def collect(repo: str = ".", include_remote: bool = False) -> List[Branch]:
    """저장소에서 브랜치 목록을 모은다."""
    patterns = ["refs/heads"]
    if include_remote:
        patterns.append("refs/remotes")

    out = _run_git(repo, ["for-each-ref", "--format=" + _FOR_EACH_REF_FORMAT] + patterns)
    current = _current_branch(repo)
    try:
        in_worktree = _worktree_branches(repo)
    except RuntimeError:
        in_worktree = set()

    branches: List[Branch] = []
    for record in out.split(REC):
        record = record.strip("\n")
        if not record:
            continue
        parts = record.split(SEP)
        if len(parts) < 8:
            continue
        refname, sha, upstream, track, cdate, author, symref, subject = parts[:8]

        is_remote = refname.startswith("refs/remotes/")
        if is_remote:
            name = refname[len("refs/remotes/"):]
        else:
            name = refname[len("refs/heads/"):]

        ahead, behind, gone = _parse_track(track)
        alias = None
        if symref:
            # origin/HEAD 처럼 다른 ref 를 가리키는 심볼릭 ref
            alias = symref.replace("refs/remotes/", "", 1).replace("refs/heads/", "", 1)

        branches.append(
            Branch(
                name=name,
                sha=sha,
                subject=subject,
                is_current=(not is_remote and name == current),
                is_remote=is_remote,
                is_worktree=(not is_remote and name in in_worktree and name != current),
                upstream=upstream.replace("refs/remotes/", "", 1) or None,
                ahead=ahead,
                behind=behind,
                gone=gone,
                committed=cdate or None,
                author=author or None,
                alias_of=alias,
            )
        )
    return branches


# ---------------------------------------------------------------------------
# 2. git branch -vv 텍스트 파싱
# ---------------------------------------------------------------------------

# "* main", "  feature/x", "+ wt-branch" 의 앞부분
_HEAD_RE = re.compile(r"^(?P<mark>[*+ ])\s+(?P<rest>.*)$")
# detached HEAD 줄
_DETACHED_RE = re.compile(r"^\((?:HEAD detached (?:at|from)|no branch)")
# "origin/HEAD -> origin/master"
_ALIAS_RE = re.compile(r"^(?P<name>\S+)\s+->\s+(?P<target>\S+)$")
# 업스트림 대괄호. refname 은 공백을 못 가지므로 공백 없는 토큰만 받는다.
# 커밋 제목이 "[WIP] ..." 처럼 시작해도 뒤 형식이 안 맞으면 업스트림으로 보지 않는다.
_UPSTREAM_RE = re.compile(
    r"^\[(?P<up>[^\s\[\]:]+)"
    r"(?::\s*(?P<track>gone|ahead \d+(?:, behind \d+)?|behind \d+))?\]\s*"
)


def parse_branch_v(text: str) -> List[Branch]:
    """git branch -v / -vv / -a 출력 텍스트를 파싱한다."""
    branches: List[Branch] = []

    for raw in text.splitlines():
        line = raw.lstrip("﻿").rstrip()   # BOM 붙은 파일도 받아 준다
        if not line.strip():
            continue

        m = _HEAD_RE.match(line)
        if m:
            mark, rest = m.group("mark"), m.group("rest")
        else:
            # 마커 열 없이 붙여 넣은 경우도 받아 준다
            mark, rest = " ", line.strip()

        if _DETACHED_RE.match(rest) and ")" in rest:
            # "* (HEAD detached at abc1234) abc1234 subject"
            close = rest.index(")")
            tail = rest[close + 1:].strip()
            sha, _, subject = tail.partition(" ")
            branches.append(
                Branch(
                    name=rest[: close + 1],
                    sha=sha,
                    subject=subject.strip(),
                    is_current=(mark == "*"),
                    is_detached=True,
                )
            )
            continue

        alias = _ALIAS_RE.match(rest)
        if alias:
            # "remotes/origin/HEAD -> origin/master"
            name = alias.group("name")
            branches.append(
                Branch(
                    name=name.replace("remotes/", "", 1),
                    is_remote=name.startswith("remotes/") or "/" in name,
                    alias_of=alias.group("target"),
                )
            )
            continue

        name, _, tail = rest.partition(" ")
        tail = tail.strip()
        sha, _, tail = tail.partition(" ")
        tail = tail.strip()

        upstream = None
        ahead = behind = 0
        gone = False
        um = _UPSTREAM_RE.match(tail)
        # 커밋 제목이 "[WIP] ..." 처럼 시작하는 경우와 구분한다.
        # 업스트림이라면 ahead/behind/gone 이 붙어 있거나, 최소한 "원격/브랜치"
        # 형태로 슬래시를 포함한다.
        if um and not (um.group("track") or "/" in um.group("up")):
            um = None
        if um:
            upstream = um.group("up")
            ahead, behind, gone = _parse_track(um.group("track") or "")
            tail = tail[um.end():]

        is_remote = name.startswith("remotes/")
        branches.append(
            Branch(
                name=name.replace("remotes/", "", 1) if is_remote else name,
                sha=sha,
                subject=tail.strip(),
                is_current=(mark == "*"),
                is_remote=is_remote,
                is_worktree=(mark == "+"),
                upstream=upstream,
                ahead=ahead,
                behind=behind,
                gone=gone,
            )
        )

    return branches


# ---------------------------------------------------------------------------
# 출력
# ---------------------------------------------------------------------------

def format_table(branches: Iterable[Branch]) -> str:
    rows = []
    for b in branches:
        flag = "*" if b.is_current else ("+" if b.is_worktree else " ")
        if b.alias_of:
            track = "-> " + b.alias_of
        elif b.gone:
            track = "gone"
        elif b.ahead or b.behind:
            track = " ".join(
                p
                for p in (
                    f"+{b.ahead}" if b.ahead else "",
                    f"-{b.behind}" if b.behind else "",
                )
                if p
            )
        else:
            track = ""
        age = b.age_days
        rows.append(
            [
                flag,
                b.name,
                b.sha,
                b.upstream or "",
                track,
                f"{age:.0f}d" if age is not None else "",
                b.subject,
            ]
        )
    if not rows:
        return "(브랜치 없음)"

    header = ["", "BRANCH", "SHA", "UPSTREAM", "TRACK", "AGE", "SUBJECT"]
    widths = [max(len(str(r[i])) for r in [header] + rows) for i in range(len(header))]
    out = []
    for r in [header] + rows:
        # 마지막 열(SUBJECT)은 폭을 맞추지 않는다
        cells = [str(c).ljust(widths[i]) for i, c in enumerate(r[:-1])]
        out.append(" ".join(cells + [str(r[-1])]).rstrip())
    return "\n".join(out)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="git 브랜치 파서")
    ap.add_argument("-C", "--repo", default=".", help="대상 저장소 경로 (기본: 현재 폴더)")
    ap.add_argument("-a", "--all", action="store_true", help="원격 추적 브랜치도 포함")
    ap.add_argument("--stdin", action="store_true", help="git branch -vv 출력을 표준입력에서 읽는다")
    ap.add_argument("--json", action="store_true", help="JSON 으로 출력")
    ap.add_argument("--gone", action="store_true", help="업스트림이 사라진 브랜치만")
    ap.add_argument("--merged", action="store_true", help="HEAD 에 이미 병합된 브랜치만")
    ap.add_argument("--stale", type=float, metavar="DAYS", help="마지막 커밋이 DAYS 일보다 오래된 것만")
    ap.add_argument(
        "--sort",
        choices=["name", "age", "ahead", "behind"],
        default="name",
        help="정렬 기준 (기본: name)",
    )
    args = ap.parse_args(argv)

    try:
        if args.stdin:
            branches = parse_branch_v(sys.stdin.read())
        else:
            branches = collect(args.repo, include_remote=args.all)

        if args.gone:
            branches = [b for b in branches if b.gone]

        if args.merged:
            if args.stdin:
                print("--merged 는 저장소가 필요해 --stdin 과 함께 쓸 수 없습니다.", file=sys.stderr)
                return 2
            merged = {
                line[2:].strip()
                for line in _run_git(args.repo, ["branch", "--merged", "HEAD"]).splitlines()
                if line.strip()
            }
            branches = [b for b in branches if b.name in merged and not b.is_current]
    except RuntimeError as e:
        print(f"git 오류: {e}", file=sys.stderr)
        return 1
    except FileNotFoundError:
        print("git 실행 파일을 찾을 수 없습니다.", file=sys.stderr)
        return 1

    if args.stale is not None:
        branches = [b for b in branches if (b.age_days or 0) >= args.stale]

    keys = {
        "name": lambda b: b.name,
        "age": lambda b: -(b.age_days or 0),
        "ahead": lambda b: -b.ahead,
        "behind": lambda b: -b.behind,
    }
    branches.sort(key=keys[args.sort])

    if args.json:
        print(json.dumps([asdict(b) for b in branches], ensure_ascii=False, indent=2))
    else:
        print(format_table(branches))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
