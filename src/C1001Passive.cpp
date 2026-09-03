#include "C1001Passive.h"

// ---------------------------------------------------------------------------
//  (con,cmd) 매핑
//
//  아래 ID 는 실제 모듈(G60SM1SY / R60A) 이 능동 보고로 올린 프레임을 그대로
//  받아 적은 것이다. 보고는 0x0X, 질의 응답은 0x8X 를 쓰므로 둘 다 넣는다.
// ---------------------------------------------------------------------------

struct FieldMap {
    uint8_t con;
    uint8_t cmd;
    uint8_t width;                        // 1 = 1바이트, 2 = 상위/하위 결합
    C1001Passive::Field C1001Passive::*f;
};

static const FieldMap MAP[] = {
    // 재실 관련 (con 0x80)
    {0x80, 0x01, 1, &C1001Passive::presence},
    {0x80, 0x81, 1, &C1001Passive::presence},
    {0x80, 0x02, 1, &C1001Passive::movement},
    {0x80, 0x82, 1, &C1001Passive::movement},
    {0x80, 0x03, 1, &C1001Passive::bodyMove},
    {0x80, 0x83, 1, &C1001Passive::bodyMove},
    {0x80, 0x04, 2, &C1001Passive::distance},
    {0x80, 0x84, 2, &C1001Passive::distance},

    // 호흡 (con 0x81)
    {0x81, 0x01, 1, &C1001Passive::respState},
    {0x81, 0x81, 1, &C1001Passive::respState},
    {0x81, 0x02, 1, &C1001Passive::respRate},
    {0x81, 0x82, 1, &C1001Passive::respRate},

    // 심박 (con 0x85)
    {0x85, 0x02, 1, &C1001Passive::heartRate},
    {0x85, 0x82, 1, &C1001Passive::heartRate},

    // 수면 (con 0x84) 중 앉은 자세에서도 의미가 있는 두 개만 추적한다
    {0x84, 0x01, 1, &C1001Passive::inBed},
    {0x84, 0x81, 1, &C1001Passive::inBed},
    {0x84, 0x02, 1, &C1001Passive::sleepState},
    {0x84, 0x82, 1, &C1001Passive::sleepState},
};
static const uint8_t MAP_N = sizeof(MAP) / sizeof(MAP[0]);

// 정체는 확인했지만 추적하지 않는 프레임. UNK 로 시끄러워지지 않게 걸러 낸다
static const uint16_t IGNORED[] = {
    0x0501,             // 초기화 완료 통지
    0x0707,             // 주기적으로 올라오는 0/1 플래그. 용도 미상
    0x8000, 0x8080,     // 재실 기능 스위치
    0x8005,             // 인체 위치 6바이트 (전부 0으로 관측됨)
    0x8100,             // 호흡 기능 스위치
    0x8500,             // 심박 기능 스위치
    0x8400,             // 수면 기능 스위치
    0x8403, 0x8404,     // 각성 시간 / 얕은 수면 시간 [분]
    0x8405, 0x8406,     // 깊은 수면 시간 / 수면 품질 점수
    0x840C, 0x840D,     // 수면 종합 8바이트 / 수면 통계 12바이트
    0x840E, 0x8410,     // 수면 이상 / 품질 평가
    0x8413, 0x8414,     // 이상 몸부림 스위치 / 무인 판정 스위치
    0x8415, 0x8416,     // 무인 판정 시간 / 수면 종료 시간
    0x848C,             // 보고 모드
};
static const uint8_t IGNORED_N = sizeof(IGNORED) / sizeof(IGNORED[0]);

// 폴백 질의 목록. 모듈이 능동 보고를 하지 않을 때만 쓰인다
static const uint8_t NUDGE_LIST[][2] = {
    {0x80, 0x81},   // 재실
    {0x80, 0x82},   // 움직임
    {0x80, 0x83},   // 체동 파라미터
    {0x81, 0x82},   // 호흡수
    {0x81, 0x81},   // 호흡 상태
    {0x85, 0x82},   // 심박수
};
static const uint8_t NUDGE_N = sizeof(NUDGE_LIST) / sizeof(NUDGE_LIST[0]);

void C1001Passive::begin()
{
    _state = S_H1;
    _idx = _len = _sum = 0;
    _unkCount = 0;
    _nudgeIdx = 0;
    frames = ignored = badChecksum = unknown = resyncs = oversize = 0;
    startMs = lastFrameMs = millis();

    // 갱신 주기 통계만 리셋한다. 값 자체는 유지해야 리셋 직후 덤프로 받은
    // 내용을 잃지 않는다
    Field *fields[] = {&presence, &movement, &bodyMove, &distance,
                       &respState, &respRate, &heartRate, &inBed, &sleepState,
                       &respWave, &heartWave};
    for (uint8_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
        fields[i]->count = 0;
        fields[i]->periodMs = 0;
    }
}

bool C1001Passive::poll()
{
    uint32_t before = frames;
    while (_s.available() > 0) {
        feed((uint8_t)_s.read());
    }
    return frames != before;
}

// 프레임 경계를 잃었을 때 현재 바이트부터 다시 잡는다.
// 벤더 파서는 이 바이트를 그냥 버려서, 헤더가 바로 뒤따라오면 프레임을 통째로
// 놓친다. 여기서는 지금 바이트가 0x53 이면 그 자리에서 새 프레임을 시작한다
void C1001Passive::resync(uint8_t b)
{
    resyncs++;
    if (b == 0x53) {
        _sum = b;
        _state = S_H2;
    } else {
        _state = S_H1;
    }
}

void C1001Passive::feed(uint8_t b)
{
    switch (_state) {
    case S_H1:
        if (b == 0x53) {
            _sum = b;
            _state = S_H2;
        }
        break;

    case S_H2:
        if (b == 0x59) {
            _sum += b;
            _state = S_CON;
        } else {
            resync(b);
        }
        break;

    case S_CON:
        _con = b;
        _sum += b;
        _state = S_CMD;
        break;

    case S_CMD:
        _cmd = b;
        _sum += b;
        _state = S_LEN_H;
        break;

    case S_LEN_H:
        // 이 프로토콜의 데이터는 MAX_DATA(<256) 를 넘지 않으므로 상위 바이트는
        // 항상 0 이다. 0 이 아니면 프레임 경계를 잘못 잡은 것이다
        if (b != 0x00) {
            oversize++;
            resync(b);
        } else {
            _len = 0;
            _sum += b;
            _state = S_LEN_L;
        }
        break;

    case S_LEN_L:
        // 길이가 말이 안 되면 경계를 잘못 잡은 것이다.
        // 이때 이 바이트 자체가 진짜 헤더일 수 있으므로 (앞 프레임이 잘려서
        // 길이 자리에 다음 프레임의 0x53 이 들어온 경우) 버리지 않고 넘긴다
        if (b > MAX_DATA) {
            oversize++;
            resync(b);
        } else {
            _len = b;
            _sum += b;
            _idx = 0;
            _state = _len ? S_DATA : S_SUM;
        }
        break;

    case S_DATA:
        _data[_idx++] = b;
        _sum += b;
        if (_idx >= _len) _state = S_SUM;
        break;

    case S_SUM:
        if (b == (uint8_t)(_sum & 0xff)) {
            _state = S_T1;
        } else {
            badChecksum++;
            resync(b);
        }
        break;

    case S_T1:
        if (b == 0x54) _state = S_T2;
        else           resync(b);
        break;

    case S_T2:
        if (b == 0x43) {
            frames++;
            lastFrameMs = millis();
            dispatch();
            _state = S_H1;
        } else {
            resync(b);
        }
        break;
    }
}

void C1001Passive::store(Field &f, uint16_t v, uint32_t now)
{
    if (f.count > 0) {
        // 갱신 간격의 이동평균. 누적 평균과 달리 부팅 직후의 덤프에
        // 끌려다니지 않고 현재의 갱신 주기를 따라간다
        float dt = (float)(now - f.stamp);
        f.periodMs = (f.periodMs == 0) ? dt : (f.periodMs + 0.3f * (dt - f.periodMs));
    }
    f.value = v;
    f.stamp = now;
    f.count++;
    f.valid = true;
}

// 파형 샘플을 링에 넣고 peak-to-peak 를 필드에 담는다.
// 샘플은 0x80 을 기준선으로 하는 부호 없는 8비트다
void C1001Passive::pushWave(uint8_t *ring, uint8_t &idx, uint8_t &fill,
                            Field &f, uint32_t now)
{
    for (uint16_t i = 0; i < _len; i++) {
        ring[idx] = _data[i];
        idx = (uint8_t)((idx + 1) % WAVE_N);
        if (fill < WAVE_N) fill++;
    }

    uint8_t lo = 0xff, hi = 0x00;
    for (uint8_t i = 0; i < fill; i++) {
        if (ring[i] < lo) lo = ring[i];
        if (ring[i] > hi) hi = ring[i];
    }
    store(f, (uint16_t)(hi - lo), now);
}

void C1001Passive::storeText(char *dst, size_t cap)
{
    size_t n = (_len < cap - 1) ? _len : cap - 1;
    for (size_t i = 0; i < n; i++) {
        char c = (char)_data[i];
        dst[i] = (c >= 0x20 && c < 0x7f) ? c : '\0';
    }
    dst[n] = '\0';
}

void C1001Passive::dispatch()
{
    if (_len == 0) { noteUnknown(); return; }

    uint32_t now = millis();
    uint16_t key = ((uint16_t)_con << 8) | _cmd;

    // 0x01/0x01 통지. 실측상 약 60초 간격으로 규칙적으로 올라오므로 하트비트로
    // 보이지만, 불규칙해지면 리셋일 수 있으니 간격을 같이 찍어 판단할 수 있게 한다
    if (key == 0x0101) {
        resetNotices++;
        Serial.print("# HB (0x01/0x01) t=");
        Serial.print((now - startMs) / 1000.0, 1);
        Serial.print("s");
        if (_lastHbMs) {
            Serial.print(" interval=");
            Serial.print((now - _lastHbMs) / 1000.0, 1);
            Serial.print("s");
        }
        Serial.println();
        _lastHbMs = now;
        return;
    }

    // 호흡/심박 파형. 값이 0으로 안 나올 때 신호가 있기는 한지 보는 지표다
    if (key == 0x8105) { pushWave(_respRing,  _respIdx,  _respFill,  respWave,  now); return; }
    if (key == 0x8505) { pushWave(_heartRing, _heartIdx, _heartFill, heartWave, now); return; }

    // 모듈 식별자
    if (_con == 0x02) {
        switch (_cmd) {
        case 0xA2: storeText(fwVersion, sizeof(fwVersion)); ignored++; return;
        case 0xA3: storeText(hwModel,   sizeof(hwModel));   ignored++; return;
        case 0xA4: storeText(serialNo,  sizeof(serialNo));  ignored++; return;
        default: break;
        }
    }

    for (uint8_t i = 0; i < MAP_N; i++) {
        if (MAP[i].con != _con || MAP[i].cmd != _cmd) continue;
        uint16_t v = _data[0];
        if (MAP[i].width == 2 && _len >= 2) {
            v = (uint16_t)_data[0] << 8 | _data[1];
        }
        store(this->*(MAP[i].f), v, now);
        return;
    }

    for (uint8_t i = 0; i < IGNORED_N; i++) {
        if (IGNORED[i] == key) { ignored++; return; }
    }

    noteUnknown();
}

// 매핑에도 무시 목록에도 없는 (con,cmd) 를 처음 만났을 때만 한 줄 찍는다
void C1001Passive::noteUnknown()
{
    unknown++;
    if (!logUnknown) return;

    uint16_t key = ((uint16_t)_con << 8) | _cmd;
    for (uint8_t i = 0; i < _unkCount; i++) {
        if (_unkSeen[i] == key) return;
    }
    if (_unkCount < UNK_SLOTS) _unkSeen[_unkCount++] = key;
    else return;                    // 슬롯이 차면 더 찍지 않는다

    Serial.print("# UNK con=0x");
    if (_con < 0x10) Serial.print('0');
    Serial.print(_con, HEX);
    Serial.print(" cmd=0x");
    if (_cmd < 0x10) Serial.print('0');
    Serial.print(_cmd, HEX);
    Serial.print(" len=");
    Serial.print(_len);
    Serial.print(" data=");
    for (uint16_t i = 0; i < _len && i < 16; i++) {
        if (_data[i] < 0x10) Serial.print('0');
        Serial.print(_data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

void C1001Passive::request(uint8_t con, uint8_t cmd, uint8_t arg)
{
    uint8_t f[10];
    f[0] = 0x53;
    f[1] = 0x59;
    f[2] = con;
    f[3] = cmd;
    f[4] = 0x00;
    f[5] = 0x01;
    f[6] = arg;

    uint16_t sum = 0;
    for (uint8_t i = 0; i < 7; i++) sum += f[i];
    f[7] = (uint8_t)(sum & 0xff);
    f[8] = 0x54;
    f[9] = 0x43;

    _s.write(f, 10);        // 논블로킹. TX 버퍼에 넣고 바로 돌아온다
}

void C1001Passive::nudge()
{
    request(NUDGE_LIST[_nudgeIdx][0], NUDGE_LIST[_nudgeIdx][1]);
    _nudgeIdx = (uint8_t)((_nudgeIdx + 1) % NUDGE_N);
}
