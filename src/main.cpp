#include <Arduino.h>
#include "DFRobot_HumanDetection.h"
#include "C1001Passive.h"
#include "DrowsyDetector.h"

// ============================================================================
//  C1001 수신 전용 구조
//
//  초기화(모드 전환, LED, 리셋)만 벤더 라이브러리로 하고, 그 뒤 UART 는
//  C1001Passive 가 전담한다. loop() 는 블로킹 없이 돌면서 올라오는 프레임을
//  소화하고 1초에 한 번 상태를 찍는다.
//
//  출력의 괄호 안은 (마지막 갱신 이후 경과, 갱신 간격의 이동평균) 이다.
//  이 간격이 센서가 그 항목을 실제로 얼마나 자주 올리는지 그대로 보여준다.
//  presence 처럼 값이 바뀔 때만 올라오는 항목은 간격이 크게 벌어진다.
//
//  모듈이 능동 보고를 하지 않는 설정이면 프레임이 안 올라온다. 그때는
//  SILENT_MS 뒤부터 질의를 하나씩 넣어 (nudge) 응답을 같은 파서로 받는다.
// ============================================================================

#define PIN_RX      16      // 센서 TX -> ESP32 RX
#define PIN_TX      17      // 센서 RX <- ESP32 TX

#define PRINT_MS    1000    // 상태 출력 주기
#define SILENT_MS   2000    // 이만큼 프레임이 없으면 질의로 깨운다
#define NUDGE_MS    200     // 질의 간격

#define DUMP_QUIET_MS 400   // 리셋 직후 덤프가 끝났다고 볼 무음 구간
#define DUMP_MAX_MS   5000  // 덤프 대기 상한

// 보고 모드 실험용. -1 이면 건드리지 않는다 (모듈 기본값 1로 관측됨).
// 0 = 실시간 보고, 1 = 수면 보고.
// 수면 보고 모드에서는 재/이석(inBed) 판정이 서야 호흡·심박이 나오는 것으로
// 보이므로, 앉은 자세에서 값이 안 나오면 0 으로 바꿔 시험해 볼 수 있다
#define REPORT_MODE   0

DFRobot_HumanDetection hu(&Serial1);
C1001Passive radar(Serial1);
DrowsyDetector drowsy;

// ---------------------------------------------------------------------------
//  구간 통계. 임계값(MOVE_QUIET_MAX, HR_DROP_BPM)을 실측으로 맞추기 위한 것이다.
//  모니터에서 문자를 보내면 그 시점에 마커가 찍히고 통계가 리셋되므로,
//  "지금부터 가만히 있는다" 같은 구간을 나눠 각각의 분포를 볼 수 있다
// ---------------------------------------------------------------------------
struct Stats {
    uint32_t n = 0;
    uint32_t startMs = 0;
    float    mAvgMin = 1e9f, mAvgMax = 0.0f, mAvgSum = 0.0f;
    uint16_t bodyMin = 65535, bodyMax = 0;
    uint32_t bodySum = 0, bodyN = 0, bodyCount = 0;
    float    dropMin = 1e9f, dropMax = -1e9f;
    uint32_t dropN = 0;
    uint16_t medMin = 65535, medMax = 0;
    uint32_t stateTicks[5] = {0, 0, 0, 0, 0};   // NOPERSON/NOLOCK/WARMUP/AWAKE/DROWSY

    void reset(uint32_t now) {
        uint32_t keep = bodyCount;
        *this = Stats();
        bodyCount = keep;
        startMs = now;
    }
};
static Stats stats;
static char  markLabel = '-';

static void printSummary(uint32_t now)
{
    Serial.println("#");
    Serial.print("# ===== SEGMENT '");
    Serial.print(markLabel);
    Serial.print("'  ");
    Serial.print((now - stats.startMs) / 1000);
    Serial.print("s, ");
    Serial.print(stats.n);
    Serial.println(" ticks =====");

    if (stats.n == 0) { Serial.println("# (no data)"); return; }

    Serial.print("# state  NOPERSON/NOLOCK/WARMUP/AWAKE/DROWSY = ");
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(100UL * stats.stateTicks[i] / stats.n);
        Serial.print(i < 4 ? "/" : "%\n");
    }

    Serial.print("# body   min/mean/max = ");
    if (stats.bodyN) {
        Serial.print(stats.bodyMin); Serial.print(" / ");
        Serial.print(stats.bodySum / stats.bodyN); Serial.print(" / ");
        Serial.println(stats.bodyMax);
    } else Serial.println("(none)");

    Serial.print("# mAvg   min/mean/max = ");
    Serial.print(stats.mAvgMin, 2); Serial.print(" / ");
    Serial.print(stats.mAvgSum / stats.n, 2); Serial.print(" / ");
    Serial.println(stats.mAvgMax, 2);
    Serial.print("#        임계 MOVE_QUIET_MAX = ");
    Serial.println(MOVE_QUIET_MAX, 1);

    Serial.print("# blank  ");
    Serial.print(drowsy.blankPercent());
    Serial.println("%  (100 이면 MOVE_SPIKE 가 낮다)");

    Serial.print("# hrMed  min/max = ");
    if (stats.medMax) {
        Serial.print(stats.medMin); Serial.print(" / "); Serial.println(stats.medMax);
    } else Serial.println("(표본 부족)");

    Serial.print("# base   "); Serial.println(drowsy.hrBase(), 2);
    Serial.print("# drop   min/max = ");
    if (stats.dropN) {
        Serial.print(stats.dropMin, 2); Serial.print(" / "); Serial.println(stats.dropMax, 2);
    } else Serial.println("(표본 부족)");
    Serial.print("#        임계 HR_DROP_BPM = ");
    Serial.println(HR_DROP_BPM, 1);
    Serial.println("# ==========================================");
    Serial.println("#");
}

// 모니터에서 보낸 문자를 처리한다.
//   s        요약 출력 (통계는 유지)
//   그 외    마커를 찍고 통계를 리셋한다. 구간 라벨로 쓴다
static void handleInput(uint32_t now)
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') continue;

        if (c == 's' || c == 'S') {
            printSummary(now);
        } else {
            // 같은 마커가 연속으로 들어오면 (모니터가 줄을 여러 번 보내는 경우)
            // 빈 구간 요약이 줄줄이 찍히므로 무시한다
            if (stats.n == 0 && c == markLabel) continue;

            if (stats.n) printSummary(now);   // 직전 구간을 먼저 마무리하고
            markLabel = c;
            stats.reset(now);
            Serial.print("# MARK '");
            Serial.print(c);
            Serial.print("' t=");
            Serial.print((now - radar.startMs) / 1000);
            Serial.println("s  (새 구간 시작)");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // 능동 보고를 놓치지 않도록 수신 버퍼를 키운다. begin() 앞에서 해야 적용된다
    Serial1.setRxBufferSize(1024);
    Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("# Start initialization");
    while (hu.begin() != 0) {
        Serial.println("# init error!!!");
        delay(1000);
    }
    Serial.println("# Initialization successful");

    Serial.println("# Start switching work mode");
    while (hu.configWorkMode(hu.eSleepMode) != 0) {
        Serial.println("# mode error!!!");
        delay(1000);
    }
    Serial.println("# Work mode switch successful");

    hu.configLEDLight(hu.eHPLed, 1);

#if REPORT_MODE >= 0
    Serial.print("# set reporting mode = ");
    Serial.println(REPORT_MODE);
    hu.configSleep(hu.eReportingmodeC, REPORT_MODE);
#endif

    hu.sensorRet();     // 설정 후 리셋. 내부에서 10초 대기한다

    // 여기부터 UART 는 파서가 가져간다. 이후 hu 는 쓰지 않는다
    radar.begin();

    // 모듈은 리셋 직후 전체 상태를 한꺼번에 쏟아낸다 (실측 58프레임/0.1초).
    // 이 덤프를 갱신 주기 통계에 섞으면 간격이 엉뚱하게 짧게 나오므로,
    // 덤프를 먼저 받아 값만 챙기고 통계는 다시 0에서 시작한다
    uint32_t t0 = millis();
    while ((millis() - t0) < DUMP_MAX_MS) {
        radar.poll();
        if ((millis() - radar.lastFrameMs) > DUMP_QUIET_MS) break;
    }
    uint32_t dumpFrames = radar.frames;
    radar.begin();
    drowsy.begin(millis());

    Serial.println("#");
    Serial.print("# module fw=");
    Serial.print(radar.fwVersion);
    Serial.print(" hw=");
    Serial.print(radar.hwModel);
    Serial.print(" sn=");
    Serial.println(radar.serialNo);
    Serial.print("# boot dump frames=");
    Serial.println(dumpFrames);
    Serial.println("# passive parser start");
    Serial.println("# 아무 문자나 보내면 그 시점에 마커가 찍히고 구간 통계가 리셋됩니다");
    Serial.println("# s 를 보내면 현재 구간 요약을 출력합니다");
    Serial.println("#");
    stats.reset(millis());
}

void loop()
{
    radar.poll();                       // 논블로킹. 밀린 바이트를 전부 소화한다

    uint32_t now = millis();
    drowsy.update(radar, now);          // 새로 갱신된 샘플만 소비한다

    // 능동 보고가 없는 설정이면 질의로 폴백한다
    static uint32_t lastNudge = 0;
    if ((now - radar.lastFrameMs) > SILENT_MS && (now - lastNudge) >= NUDGE_MS) {
        radar.nudge();
        lastNudge = now;
    }

    handleInput(now);

    static uint32_t lastPrint = 0;
    if ((now - lastPrint) < PRINT_MS) return;
    lastPrint += PRINT_MS;
    if ((now - lastPrint) > PRINT_MS) lastPrint = now;   // 너무 밀렸으면 재동기

    // --- 구간 통계 누적 ---
    stats.n++;
    stats.stateTicks[drowsy.state()]++;
    float ma = drowsy.moveAvg();
    if (ma < stats.mAvgMin) stats.mAvgMin = ma;
    if (ma > stats.mAvgMax) stats.mAvgMax = ma;
    stats.mAvgSum += ma;

    if (radar.bodyMove.count != stats.bodyCount) {   // 새 샘플만 센다
        stats.bodyCount = radar.bodyMove.count;
        uint16_t b = radar.bodyMove.value;
        if (b < stats.bodyMin) stats.bodyMin = b;
        if (b > stats.bodyMax) stats.bodyMax = b;
        stats.bodySum += b;
        stats.bodyN++;
    }

    uint8_t medNow = drowsy.hrMedian(now);
    if (medNow) {
        if (medNow < stats.medMin) stats.medMin = medNow;
        if (medNow > stats.medMax) stats.medMax = medNow;
        float dr = drowsy.hrDrop(now);
        if (dr < stats.dropMin) stats.dropMin = dr;
        if (dr > stats.dropMax) stats.dropMax = dr;
        stats.dropN++;
    }

    // 파형(0x81/0x05, 0x85/0x05)은 부팅 덤프에서만 오고 스트리밍되지 않는 것으로
    // 확인돼 출력에서 뺐다. 필드 자체는 파서에 남아 있다
    Serial.print('[');
    Serial.print((now - radar.startMs) / 1000.0, 1);
    Serial.print("s ");
    Serial.print(markLabel);
    Serial.print("] ");
    Serial.print(drowsy.stateName());
    Serial.print(' ');

    Serial.print("pres=");   Serial.print(radar.presence.value);
    Serial.print(" body=");  Serial.print(radar.bodyMove.value);
    Serial.print(" mAvg=");  Serial.print(drowsy.moveAvg(), 2);
    Serial.print(" dist=");  Serial.print(radar.distance.value);
    Serial.print(" bed=");   Serial.print(radar.inBed.value);
    Serial.print(" lock=");  Serial.print(drowsy.locked() ? 1 : 0);
    Serial.print(" resp=");  Serial.print(radar.respRate.value);
    Serial.print("/st");     Serial.print(radar.respState.value);
    Serial.print(" hr=");    Serial.print(radar.heartRate.value);

    uint8_t med = drowsy.hrMedian(now);
    Serial.print(" med=");
    if (med) Serial.print(med); else Serial.print("--");
    Serial.print("/");       Serial.print(drowsy.hrSamples(now));
    Serial.print(" base=");
    if (drowsy.hrBase() > 0.0f) {
        Serial.print(drowsy.hrBase(), 1);
    } else {
        // 아직 기준선이 안 섰다. 표본이 몇 개까지 찼는지 보여준다
        Serial.print("--(");
        Serial.print(drowsy.hrSamples(now));
        Serial.print('/');
        Serial.print(HR_BASE_MIN_N);
        Serial.print(')');
    }
    Serial.print(" drop=");  Serial.print(drowsy.hrDrop(now), 1);
    Serial.print(" blank="); Serial.print(drowsy.hrBlanking(now) ? 1 : 0);
    Serial.print('/');       Serial.print(drowsy.blankPercent());
    Serial.print('%');

    Serial.print(" ev=");    Serial.print(drowsy.evidence());
    Serial.print("(r");      Serial.print(drowsy.respLost() ? 1 : 0);
    Serial.print("h");       Serial.print(drowsy.hrDown() ? 1 : 0);
    // 하락 조건은 걸렸는데 아직 30초를 못 채웠으면 진행 상황을 보여준다
    if (!drowsy.hrDown() && drowsy.hrDownPending()) {
        Serial.print('~');
        Serial.print(drowsy.hrDownMs(now) / 1000);
    }
    Serial.print(") spike-");
    Serial.print(drowsy.sinceSpike(now) / 1000);
    Serial.print("s ");
    // 징후가 성립하지 않은 동안에는 0으로 찍는다. 예전에는 마지막 상태 전환
    // 이후의 경과를 그대로 찍어서, 진행 중이 아닌데도 카운터가 올라가는 것처럼
    // 보였다 (212/180s 인데 AWAKE)
    Serial.print(drowsy.signActive() ? (drowsy.signMs(now) / 1000) : 0);
    Serial.print('/');
    Serial.print(drowsy.needMs() / 1000);
    Serial.print("s");

    Serial.print(" | f=");
    Serial.print(radar.frames);
    Serial.print(" unk=");
    Serial.print(radar.unknown);
    Serial.print(" bad=");
    Serial.print(radar.badChecksum);
    Serial.print(" silent=");
    Serial.print((now - radar.lastFrameMs) / 1000.0, 1);
    Serial.println("s");
}
