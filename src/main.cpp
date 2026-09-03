#include <Arduino.h>
#include "DFRobot_HumanDetection.h"

DFRobot_HumanDetection hu(&Serial1);

// ============================================================================
//  책상에 앉은 사람의 졸음 감지 (Sleep 모드)
//
//  실측으로 확인된 제약
//   - 센서 내장 수면 판정은 못 쓴다. 앉은 자세는 inBed=0 이라 전 필드가 0으로 온다.
//   - 엎드리면 가슴이 가려져 호흡·심박이 끊긴다. 이건 자세 그 자체의 지표다.
//   - 심박은 체동 1회에 66->125까지 튀고 복구에 1~2분 걸린다. 순간값은 쓸 수 없다.
//   - HRV는 불가능하다. 정수 bpm이 약 3.4초마다 갱신될 뿐 박동 간격을 주지 않는다.
//
//  판정 구조
//   주 신호 : 체동 이동평균이 낮게 유지  (이것 없이는 졸음으로 보지 않는다)
//   보조 증거 (1) 사람은 있는데 호흡이 안 잡힘  -> 엎드린 자세
//   보조 증거 (2) 심박 중앙값이 각성 기준선보다 하락
//   증거가 많을수록 확정에 필요한 지속 시간이 짧아진다.
//
//  자세 판별은 추후 VL53L9가 담당한다. 여기서는 자세를 추정하지 않는다.
// ============================================================================

#define INVALID_U8          0xff    // 라이브러리가 통신 실패 시 돌려주는 값
#define RESP_STATE_NONE     4       // getBreatheState: 호흡 없음

// --- 체동 ---
#define MOVE_TAU            15.0    // 체동 이동평균 시정수 [초]
#define MOVE_QUIET_MAX      4.0     // 이 아래면 "거의 안 움직임"
#define MOVE_SPIKE          20      // 이 값을 넘으면 실제 움직임 이벤트로 본다

// --- 심박 ---
#define HR_MIN              40      // 유효 심박 하한
#define HR_MAX              150     // 유효 심박 상한
#define HR_BLANK_MS         90000   // 체동 이벤트 후 심박을 버릴 시간.
                                    // 센서 추정기가 다시 수렴하는 데 1~2분 걸린다
#define HR_WIN_MS           120000  // 단기 중앙값을 낼 창 [ms]
#define HR_WIN_N            128     // 링버퍼 크기
#define HR_WIN_MIN_N        10      // 중앙값을 신뢰할 최소 샘플 수
#define HR_BASE_TAU         600.0   // 각성 기준선 시정수 [초]
#define HR_DROP_BPM         4.0     // 기준선 대비 이만큼 떨어지면 졸음 징후

// --- 확정 시간 (증거 개수에 따라 달라진다) ---
#define HOLD_NO_EVIDENCE_MS 180000  // 체동만 (3분)
#define HOLD_ONE_MS         90000   // 증거 1개 (1분 30초)
#define HOLD_TWO_MS         60000   // 증거 2개 (1분)
#define AWAKE_HOLD_MS       10000   // 징후가 이만큼 사라져야 각성으로 되돌린다

#define SUMMARY_AT_MS       1800000 // 부팅 후 이 시각에 요약을 한 번 출력한다 (30분)

float moveAvg = 0.0;                // 체동 이동평균
uint32_t lastEmaMs = 0;             // 이동평균 갱신 시각
uint32_t hrBlankUntil = 0;          // 이 시각까지 심박을 버린다

uint8_t  hrBuf[HR_WIN_N];           // 심박 링버퍼 (값)
uint32_t hrTime[HR_WIN_N];          // 심박 링버퍼 (수집 시각)
uint8_t  hrHead = 0;                // 다음에 쓸 위치
uint8_t  hrFill = 0;                // 채워진 개수

float hrBase = 0.0;                 // 각성 시 심박 기준선
uint32_t lastBaseMs = 0;            // 기준선 갱신 시각

bool drowsy = false;                // 현재 졸음 판정
uint32_t signSince = 0;             // 현재 징후 상태가 시작된 시각
bool prevSign = false;              // 직전 주기의 징후 여부

// --- 30분 요약용 누적 통계 ---
bool summaryDone = false;           // 요약을 이미 냈는지
uint32_t statCycles = 0;            // 전체 주기 수
uint32_t statPresent = 0;           // presence==1 이었던 주기 수
uint32_t statStill = 0;             // still 이었던 주기 수
uint32_t statBlank = 0;             // 심박 blanking 이었던 주기 수
float statMoveMin = 1e9, statMoveMax = 0, statMoveSum = 0;
uint8_t statHrMin = 255, statHrMax = 0;
uint32_t statHrSum = 0, statHrN = 0;

// 링버퍼에 심박 샘플을 넣는다
void hrPush(uint8_t v, uint32_t t) {
  hrBuf[hrHead] = v;
  hrTime[hrHead] = t;
  hrHead = (hrHead + 1) % HR_WIN_N;
  if (hrFill < HR_WIN_N) hrFill++;
}

// 최근 HR_WIN_MS 안의 샘플들로 중앙값을 낸다.
// 샘플이 부족하면 0을 반환한다. count 에는 사용된 샘플 수가 담긴다
uint8_t hrMedian(uint32_t now, uint8_t *count) {
  uint8_t tmp[HR_WIN_N];
  uint8_t n = 0;

  for (uint8_t i = 0; i < hrFill; i++) {
    if ((now - hrTime[i]) <= HR_WIN_MS) {
      tmp[n++] = hrBuf[i];
    }
  }
  *count = n;
  if (n < HR_WIN_MIN_N) return 0;

  for (uint8_t i = 1; i < n; i++) {          // 삽입 정렬
    uint8_t key = tmp[i];
    int16_t j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

// 누적 통계를 사람이 읽을 수 있는 형태로 출력한다.
// 모든 줄을 # 로 시작해 CSV 파서가 주석으로 건너뛸 수 있게 한다
void printSummary(uint32_t now) {
  Serial.println("#");
  Serial.println("# ================ SUMMARY ================");
  Serial.print("# elapsed        : "); Serial.print(now / 1000); Serial.println(" s");
  Serial.print("# cycles         : "); Serial.println(statCycles);
  if (statCycles == 0) { Serial.println("# (no data)"); return; }

  Serial.print("# presence ratio : ");
  Serial.print(100.0 * statPresent / statCycles, 1); Serial.println(" %");
  Serial.print("# still ratio    : ");
  Serial.print(100.0 * statStill / statCycles, 1); Serial.println(" %");
  Serial.print("# hr blank ratio : ");
  Serial.print(100.0 * statBlank / statCycles, 1); Serial.println(" %");

  Serial.println("#");
  Serial.println("# --- move avg (체동 이동평균) ---");
  Serial.print("#   min / mean / max : ");
  Serial.print(statMoveMin, 2); Serial.print(" / ");
  Serial.print(statMoveSum / statCycles, 2); Serial.print(" / ");
  Serial.println(statMoveMax, 2);
  Serial.print("#   현재 임계값 MOVE_QUIET_MAX = "); Serial.println(MOVE_QUIET_MAX, 1);

  Serial.println("#");
  Serial.println("# --- heart rate median (2분 창) ---");
  if (statHrN == 0) {
    Serial.println("#   유효 샘플 없음. 심박 증거는 사용 불가");
  } else {
    Serial.print("#   samples          : "); Serial.println(statHrN);
    Serial.print("#   min / mean / max : ");
    Serial.print(statHrMin); Serial.print(" / ");
    Serial.print((float)statHrSum / statHrN, 1); Serial.print(" / ");
    Serial.println(statHrMax);
    Serial.print("#   baseline         : "); Serial.println(hrBase, 2);
    Serial.print("#   현재 임계값 HR_DROP_BPM = "); Serial.println(HR_DROP_BPM, 1);
  }
  Serial.println("# =========================================");
  Serial.println("#");
}

void setup() {
  Serial.begin(115200);

  // ESP32 하드웨어 시리얼 핀 설정
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("Start initialization");
  while (hu.begin() != 0) {
    Serial.println("init error!!!");
    delay(1000);
  }
  Serial.println("Initialization successful");

  Serial.println("Start switching work mode");
  while (hu.configWorkMode(hu.eSleepMode) != 0) {
    Serial.println("error!!!");
    delay(1000);
  }
  Serial.println("Work mode switch successful");

  hu.configLEDLight(hu.eHPLed, 1);
  hu.sensorRet();

  uint32_t now = millis();
  lastEmaMs = now;
  lastBaseMs = now;
  signSince = now;
  // CSV 헤더. # 로 시작하는 줄은 주석이다
  Serial.println("#");
  Serial.println("# 아무 문자나 보내면 그 시점에 마커가 찍힙니다 (자세 바꾸기 직전에 누르세요)");
  Serial.println("# s 를 보내면 요약을 즉시 출력합니다");
  Serial.println("t_s,presence,move,moveAvg,resp,respState,hrRaw,hrMed,hrN,hrBase,hrDrop,blank,still,respLost,hrDown,ev,signS,needS,state,readMs,cycleMs");
}

void loop() {
  static uint32_t lastCycle = 0;
  uint32_t cycleStart = millis();
  uint32_t period = (lastCycle == 0) ? 0 : (cycleStart - lastCycle);
  lastCycle = cycleStart;

  // --- 센서 읽기 ---
  uint16_t presence    = hu.smHumanData(hu.eHumanPresence);
  uint16_t movingRange = hu.smHumanData(hu.eHumanMovingRange);
  uint8_t  resp        = hu.getBreatheValue();
  uint8_t  respState   = hu.getBreatheState();   // 1=정상 2=너무빠름 3=너무느림 4=없음
  uint8_t  rawHR       = hu.getHeartRate();

  uint32_t now = millis();

  // --- 체동 이동평균 (루프 주기와 무관하게 시간 기반) ---
  float dt = (now - lastEmaMs) / 1000.0;
  lastEmaMs = now;
  moveAvg += (dt / (MOVE_TAU + dt)) * (movingRange - moveAvg);

  // --- 심박 수집 ---
  // 체동 스파이크 뒤에는 센서 추정기가 흔들린 채로 남으므로 통째로 버린다
  if (movingRange > MOVE_SPIKE) {
    hrBlankUntil = now + HR_BLANK_MS;
  }
  bool hrBlanking = ((int32_t)(hrBlankUntil - now) > 0);

  bool hrOk = (rawHR != INVALID_U8) && (rawHR >= HR_MIN) && (rawHR <= HR_MAX);
  if (hrOk && !hrBlanking && presence == 1) {
    hrPush(rawHR, now);
  }

  uint8_t hrCount = 0;
  uint8_t hrMed = hrMedian(now, &hrCount);

  // --- 심박 기준선 ---
  // 각성 상태에서만 갱신해야 졸기 시작한 뒤의 낮은 심박이 기준선을 끌어내리지 않는다
  float dtBase = (now - lastBaseMs) / 1000.0;
  lastBaseMs = now;
  if (hrMed > 0) {
    if (hrBase == 0.0) {
      hrBase = hrMed;
    } else if (!drowsy) {
      hrBase += (dtBase / (HR_BASE_TAU + dtBase)) * (hrMed - hrBase);
    }
  }
  float hrDrop = (hrBase > 0.0 && hrMed > 0) ? (hrBase - hrMed) : 0.0;

  // --- 판정 ---
  bool present = (presence == 1);
  bool still   = (moveAvg < MOVE_QUIET_MAX);            // 주 신호

  // 보조 증거 (1) 사람은 있는데 호흡이 안 잡힌다 = 가슴이 가려진 자세
  bool respLost = present && (resp == 0 || respState == RESP_STATE_NONE);
  // 보조 증거 (2) 심박 중앙값이 기준선보다 유의하게 낮다
  bool hrDown   = (hrMed > 0) && (hrDrop >= HR_DROP_BPM);

  uint8_t evidence = (respLost ? 1 : 0) + (hrDown ? 1 : 0);
  bool sign = present && still;                          // 체동 없이는 판정하지 않는다

  if (sign != prevSign) {
    prevSign = sign;
    signSince = now;
  }
  uint32_t signMs = now - signSince;

  uint32_t needMs = (evidence >= 2) ? HOLD_TWO_MS
                  : (evidence == 1) ? HOLD_ONE_MS
                                    : HOLD_NO_EVIDENCE_MS;

  if (!drowsy && sign && signMs >= needMs) {
    drowsy = true;
  } else if (drowsy && !sign && signMs >= AWAKE_HOLD_MS) {
    drowsy = false;
  }

  // --- 누적 통계 ---
  statCycles++;
  if (present) statPresent++;
  if (still) statStill++;
  if (hrBlanking) statBlank++;
  if (moveAvg < statMoveMin) statMoveMin = moveAvg;
  if (moveAvg > statMoveMax) statMoveMax = moveAvg;
  statMoveSum += moveAvg;
  if (hrMed > 0) {
    if (hrMed < statHrMin) statHrMin = hrMed;
    if (hrMed > statHrMax) statHrMax = hrMed;
    statHrSum += hrMed;
    statHrN++;
  }

  // --- 구간 표시 ---
  // 시리얼 모니터에서 아무 문자나 보내면 그 시점에 마커가 찍힌다.
  // 자세를 바꾸기 직전에 눌러 두면 나중에 로그에서 구간을 나눌 수 있다.
  // 's' 를 보내면 요약을 즉시 출력한다.
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') continue;
    if (c == 's' || c == 'S') {
      printSummary(now);
    } else {
      Serial.print("# MARK ");
      Serial.print(c);
      Serial.print(" t=");
      Serial.println(now / 1000);
    }
  }

  // --- CSV 한 줄 출력 ---
  Serial.print(now / 1000);        Serial.print(',');
  Serial.print(presence);          Serial.print(',');
  Serial.print(movingRange);       Serial.print(',');
  Serial.print(moveAvg, 2);        Serial.print(',');
  Serial.print(resp);              Serial.print(',');
  Serial.print(respState);         Serial.print(',');
  Serial.print(rawHR);             Serial.print(',');
  Serial.print(hrMed);             Serial.print(',');
  Serial.print(hrCount);           Serial.print(',');
  Serial.print(hrBase, 2);         Serial.print(',');
  Serial.print(hrDrop, 2);         Serial.print(',');
  Serial.print(hrBlanking ? 1 : 0); Serial.print(',');
  Serial.print(still ? 1 : 0);     Serial.print(',');
  Serial.print(respLost ? 1 : 0);  Serial.print(',');
  Serial.print(hrDown ? 1 : 0);    Serial.print(',');
  Serial.print(evidence);          Serial.print(',');
  Serial.print(signMs / 1000);     Serial.print(',');
  Serial.print(needMs / 1000);     Serial.print(',');
  Serial.print(drowsy ? "DROWSY" : (present ? "AWAKE" : "NOPERSON"));
  Serial.print(',');
  Serial.print(millis() - cycleStart); Serial.print(',');
  Serial.println(period);

  // --- 30분 요약 ---
  if (!summaryDone && now >= SUMMARY_AT_MS) {
    printSummary(now);
    summaryDone = true;
  }

  delay(1000);
}
