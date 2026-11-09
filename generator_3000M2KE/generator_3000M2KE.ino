/*
  Контролер генератора Powermat PM-AGR-3000M2KE
  Плата:   WeAct ESP32-C6-MINI (ESP32C6 Dev Module, USB CDC On Boot: Enabled,
           Partition Scheme: Default 4MB with spiffs — потрібна для OTA)
  Ядро:    esp32 by Espressif 3.x
  Бібліотеки: OneWireNg (дає сумісний OneWire.h), DallasTemperature (Miles Burton)
  Версія:  1.4

  v1.4: аварія зберігається в NVS і переживає перезавантаження (скидання — reset);
        CPU 80 МГц і пауза в loop у простої; датчик палива і температура в простої
        раз на 60 с; насос зупиняється після 3 підтверджених вимірів; сторінка:
        кнопки за станом, шкала палива, прогрес OTA, пауза оновлення у фоні.

  Зв'язок із SITE: UART1 9600 8N1, TX=GPIO6, RX=GPIO7, через ADUM1201 біля SITE.
  Протокол: текстові кадри  $<тіло>*<CRC>\n , CRC = XOR усіх символів тіла (2 hex).
  Повний опис: PROTOCOL_GEN_SITE.md

  Wi-Fi на місці: мережа GEN-3000, сторінка http://192.168.4.1
    вмикається: кнопка BOOT 2 с, команда wifi on (Serial), CMD,WIFI,ON (SITE)
    вимикається сама через 10 хв без підключених, якщо двигун не працює

  Serial Monitor (115200, Newline) — введи help.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <stdarg.h>

#define FW_VERSION "1.4"

// ===================== ПІНИ =====================
const int PIN_STOP    = 20;  // IN1 СТОП: HIGH = запалювання дозволене, LOW = заглушено
const int PIN_START   = 21;  // IN2 СТАРТ: HIGH = стартер крутить
const int PIN_SSR     = 22;  // IN3 SSR: HIGH = 230 В на зарядні
const int PIN_PUMP    = 23;  // IN4 насос: HIGH = качає
const int PIN_DET     = 19;  // контакт MY3: LOW = 230 В є
const int PIN_SERVO   = 18;  // PWM серво чока
const int PIN_SRVPWR  = 0;   // затвор IRLZ44N: HIGH = живлення серво
const int PIN_BAT     = 1;   // АЦП, дільник 100k/18k
const int PIN_FUEL    = 2;   // АЦП, датчик палива
const int PIN_FUELPWR = 3;   // живлення підтяжки датчика палива (470 Ом)
const int PIN_TEMP    = 14;  // DS18B20 (АКБ + двигун на одній шині)
const int PIN_TX      = 6;   // UART до SITE
const int PIN_RX      = 7;
const int PIN_BOOTBTN = 9;   // кнопка BOOT на платі (лише читаємо під час роботи)

// ===================== WI-FI =====================
const char* AP_SSID = "GEN-3000";
const char* AP_PASS = "c52075c1";
const char* WEB_PIN = "22442244";
const uint32_t WIFI_IDLE_OFF_MS = 10UL * 60000UL;   // 10 хв без підключених — вимкнути
const uint32_t BOOTBTN_HOLD_MS  = 2000;

// ===================== КАЛІБРУВАННЯ =====================
const int   CHOKE_OPEN_US   = 1280;
const int   CHOKE_CLOSED_US = 1820;
const float BAT_K           = 6.616f;
const int   FUEL_TH[4]      = {155, 400, 600, 810};   // мВ, межі сходинок 1|2|3|4|5
const int   FUEL_OPEN_MV    = 2500;                    // вище — обрив датчика
DeviceAddress TEMP_BAT_ADDR = {0x28, 0x95, 0x12, 0x22, 0x00, 0x00, 0x00, 0xCC};
const float HOT_ENGINE_C    = 40.0f;   // двигун вважається теплим (уточнимо після встановлення датчика)

// ===================== ЧАСОВІ ПАРАМЕТРИ =====================
const uint32_t CRANK_MAX_MS        = 3000;     // максимум прокрутки за спробу
const uint32_t CRANK_WAIT_MS       = 3000;     // після прокрутки чекаємо 230 В
const uint32_t RETRY_PAUSE_MS      = 15000;    // пауза між спробами
const int      MAX_ATTEMPTS        = 3;
const uint32_t PRECRANK_MS         = 1500;     // пауза перед стартером
const uint32_t PUMP_SETTLE_MS      = 5000;     // пауза, якщо насос щойно працював
const uint32_t EARLY_STALL_MS      = 15000;    // заглух у перші 15 с — невдала спроба, пробуємо ще
const uint32_t EARLY_CONFIRM_MS    = 1000;
const uint32_t STALL_MS            = 3000;     // пізніше: 230 В зникла на 3 с = двигун заглух
const uint32_t STOP_WAIT_MS        = 15000;    // за цей час двигун мусить заглухнути
const uint32_t DARK_LIMIT_MS       = 30UL * 60000UL;     // start dark без work — 30 хв
const uint32_t HARD_MAX_MS         = 8UL * 3600000UL;    // абсолютний максимум роботи, 8 год
const uint32_t HOT_WINDOW_MS       = 20UL * 60000UL;     // 20 хв після зупинки двигун теплий
const uint32_t NOFUEL_REFUEL_MS    = 2UL * 60000UL;      // заглух без палива — дозаправка через 2 хв
const uint32_t BOOT_REFUEL_MIN_MS  = 5UL * 60000UL;      // після перезавантаження — не раніше ніж через 5 хв
const uint32_t LINK_TIMEOUT_MS     = 300000;   // 5 хв без кадрів = зв'язку немає
const uint32_t HB_FRESH_MS         = 300000;   // AUTO-старт лише зі свіжим HB
const uint32_t FLIP_VALID_S        = 8UL * 3600UL;       // після зміни день/ніч стан вірний ще 8 год
const uint32_t NOLINK_MARGIN_MS    = 25UL * 60000UL;     // без зв'язку: прогноз SITE + 25 хв запасу
const uint32_t AC_DEBOUNCE_MS      = 300;
const uint32_t MANUAL_CRANK_MAX_MS = 3000;
const float    MIN_START_V         = 10.5f;    // нижче — не крутимо стартер
const float    OVERTEMP_C          = 60.0f;    // перегрів стартерного АКБ

// опитування датчиків і економія
const uint32_t CPU_MHZ             = 80;       // мінімум для Wi-Fi
const uint32_t FUEL_ACTIVE_MS      = 1000;     // двигун працює / ручний режим / сторінка відкрита
const uint32_t FUEL_PUMP_MS        = 200;      // насос качає
const uint32_t FUEL_IDLE_MS        = 60000;    // простій
const uint32_t TEMP_ACTIVE_MS      = 10000;
const uint32_t TEMP_IDLE_MS        = 60000;
const uint32_t TEMP_CONV_MS        = 250;      // 10 біт: конверсія 188 мс
const int      PUMP_CONFIRM_N      = 3;        // стільки вимірів підряд, щоб зупинити насос
const uint32_t LOOP_IDLE_DELAY_MS  = 10;       // у простої процесор спить між проходами loop

// Налаштовувані через SET, зберігаються в NVS
uint32_t cfgMaxRunMin    = 300;  // максимальна робота без work, хв
uint32_t cfgWarmupS      = 60;   // прогрів до вмикання SSR, с
uint32_t cfgCooldownS    = 30;   // робота без навантаження перед зупинкою, с
uint32_t cfgPumpMaxS     = 900;  // максимальний час заправки, с
uint32_t cfgPumpStepS    = 300;  // сходинка не змінилась за цей час = каністра порожня, с
uint32_t cfgRefuelDlyMin = 30;   // дозаправка через N хв після роботи (0 = одразу)

// ===================== СТАНИ =====================
enum State { S_IDLE, S_PRECRANK, S_CRANK, S_CRANKWAIT, S_RETRYWAIT,
             S_WARMUP, S_RUN, S_COOLDOWN, S_STOPPING, S_FAULT, S_MANUAL };
const char* STATE_NAMES[] = { "IDLE", "PRECRANK", "CRANK", "CRANKWAIT", "RETRYWAIT",
                              "WARMUP", "RUN", "COOLDOWN", "STOPPING", "FAULT", "MANUAL" };

enum Fault { F_NONE, F_START_FAIL, F_STALL, F_STOP_FAIL, F_OVERTEMP };
const char* FAULT_NAMES[] = { "NONE", "START_FAIL", "STALL", "STOP_FAIL", "OVERTEMP" };

enum { W_LOWFUEL = 1, W_CANISTER = 2, W_FUELSENSOR = 4, W_NOTIME = 8 };
const char* WARN_NAMES[] = { "LOWFUEL", "CANISTER", "FUELSENSOR", "NOTIME" };

State    state = S_IDLE;
uint32_t stateT = 0;
bool     stateEntry = true;
Fault    fault = F_NONE;
Fault    pendingFault = F_NONE;
const char* stopReason = "-";
int      attempts = 0;
int      earlyStalls = 0;
bool     pumpedThisStart = false;
uint32_t runStartMs = 0;
uint32_t lastRunS = 0;
uint32_t lastStopMs = 0;
bool     engineRan = false;      // двигун працював від останнього простою

// параметри поточного запуску
const char* runSrc = "-";        // AUTO / MANUAL / DARK / SERVICE
bool     runDark = false;
bool     runNoChoke = false;
uint32_t workUntilMs = 0;        // 0 = таймер work не заданий
uint32_t chokeRampMsCur = 15000;
int      chokeFromUs = CHOKE_OPEN_US;

// насос і дозаправка
int      pumpTarget = 0;         // 0 = насос не працює за завданням
uint32_t pumpStartMs = 0, pumpStepMs = 0;
int      pumpLastStep = 0;
uint32_t pumpSeenSeq = 0;
int      pumpFullCnt = 0, pumpOpenCnt = 0;
const char* pumpReason = "-";
uint32_t refuelAtMs = 0;         // 0 = дозаправка не запланована
int      refuelTarget = 0;       // до якої сходинки дозаправити
int      pumpAfter = 0;          // 0 = за правилом (AUTO — до повного), -1 = не дозаправляти, 2..5 = до сходинки
int      carryTarget = 0;        // незроблена дозаправка, перенесена на кінець поточної роботи
uint8_t  warnSticky = 0;

// ===================== ВИМІРИ =====================
float vbat = 0;
int   fuelMv = 0;
int   fuelStep = 0;      // 1..5 після фільтра, 0 = обрив
uint32_t fuelSeq = 0;    // номер виміру палива (росте з кожним виміром)
uint32_t fuelLastMs = 0;
int   fuelCand = -1, fuelCandN = 0;
float tempC = 0;  bool tempOk = false;     // АКБ
DeviceAddress engAddr; bool engPresent = false;
float engT = 0;   bool engOk = false;      // двигун
bool  acRaw = false, ac = false;
uint32_t acChangeT = 0;
uint32_t acLostSince = 0;

// ===================== ВИХОДИ =====================
bool stopOn = false, startOn = false, ssrOn = false, pumpOn = false;
int  servoUs = CHOKE_OPEN_US;
bool servoPwr = false;
uint32_t servoReleaseAt = 0;

// сервісний режим
uint32_t crankUntil = 0;
bool     monOn = true;
bool     rawOn = false;

// ===================== ЗВ'ЯЗОК І ЧАС =====================
uint32_t lastRxMs = 0;
uint32_t lastHbMs = 0;
bool     hbDay = false;
uint32_t hbToChangeS = 0;        // секунд до зміни день/ніч на момент HB (0 = невідомо)
int32_t  hbChargeMin = -1;       // прогноз SITE: скільки ще хвилин заряджати (-1 = невідомо)
uint32_t badFrames = 0;
String   rxSite, rxUsb;

// ===================== WI-FI / ЖУРНАЛ =====================
bool     wifiOn = false;
uint32_t wifiLastClientMs = 0;
uint32_t pinFails = 0, pinLockUntil = 0;
bool     otaOk = false, otaReject = false;
String*  capture = nullptr;      // куди збирати відповідь для вебсторінки

const int LOG_N = 30;
struct LogEv { uint32_t ms; char txt[112]; };
LogEv    logBuf[LOG_N];
int      logHead = 0, logCount = 0;
uint32_t logTotal = 0;           // скільки подій записано від старту (сторінка тягне журнал лише при зміні)

OneWire oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);
Preferences prefs;
WebServer server(80);

// прототипи
void sendStatus();
void enterState(State s);
void beginStop(const char* reason, Fault after);
void pumpStop(const char* msg);
int  fuelStepFromMv(int mv);
void wifiStart(const char* why);
void wifiStop(const char* why);
void fuelSample();

// ===================================================================
//                         ВИВІД І ЖУРНАЛ
// ===================================================================
// обрізати незавершений UTF-8 символ у кінці (після vsnprintf у короткий буфер)
void utf8Trim(char* s) {
  size_t n = strlen(s);
  if (!n) return;
  size_t k = n - 1;
  while (k > 0 && ((uint8_t)s[k] & 0xC0) == 0x80) k--;
  uint8_t c = (uint8_t)s[k];
  size_t need = c < 0x80 ? 1 : (c & 0xE0) == 0xC0 ? 2 : (c & 0xF0) == 0xE0 ? 3 : (c & 0xF8) == 0xF0 ? 4 : 1;
  if (n - k < need) s[k] = 0;
}

void sayRaw(const char* s) {
  Serial.print(s);
  if (capture) *capture += s;
}
void say(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void say(const char* fmt, ...) {          // відповідь оператору (Serial + сторінка)
  char b[256];
  va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
  utf8Trim(b);
  sayRaw(b); sayRaw("\n");
}
void logEv(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logEv(const char* fmt, ...) {        // подія: Serial + сторінка + журнал
  char b[112];
  va_list a; va_start(a, fmt); vsnprintf(b, sizeof(b), fmt, a); va_end(a);
  utf8Trim(b);
  Serial.printf("[%lu] %s\n", (unsigned long)(millis() / 1000), b);
  if (capture) { *capture += b; *capture += "\n"; }
  LogEv& e = logBuf[logHead];
  e.ms = millis();
  strncpy(e.txt, b, sizeof(e.txt) - 1); e.txt[sizeof(e.txt) - 1] = 0;
  logHead = (logHead + 1) % LOG_N;
  if (logCount < LOG_N) logCount++;
  logTotal++;
}

// ===================================================================
//                              ВИХОДИ
// ===================================================================
void setStop(bool allow) { stopOn = allow; digitalWrite(PIN_STOP, allow ? HIGH : LOW); }
void setSsr(bool on)     { ssrOn = on;     digitalWrite(PIN_SSR,  on ? HIGH : LOW); }
void setPump(bool on)    { pumpOn = on;    digitalWrite(PIN_PUMP, on ? HIGH : LOW); }
void setStart(bool on) {
  if (on && (!stopOn || ac)) on = false;   // стартер лише з дозволеним запалюванням і коли двигун стоїть
  startOn = on; digitalWrite(PIN_START, on ? HIGH : LOW);
}
void allOutputsSafe() { setStart(false); pumpTarget = 0; setPump(false); setSsr(false); setStop(false); }

void servoPowerOn() {
  if (!servoPwr) { digitalWrite(PIN_SRVPWR, HIGH); servoPwr = true; delay(100); }
}
void servoWrite(int us) {
  servoPowerOn();
  uint32_t duty = (uint32_t)us * ((1UL << 14) - 1) / 20000;
  ledcWrite(PIN_SERVO, duty);
  servoUs = us;
}
void servoMove(int us) { servoWrite(us); servoReleaseAt = millis() + 1000; }
void servoOff() { ledcWrite(PIN_SERVO, 0); digitalWrite(PIN_SRVPWR, LOW); servoPwr = false; servoReleaseAt = 0; }

// ===================================================================
//                              ЧАС / ПЛАН
// ===================================================================
int dayState() {                 // 1 = день, 0 = ніч, -1 = невідомо
  if (lastHbMs == 0) return -1;
  uint32_t el = (millis() - lastHbMs) / 1000;
  if (hbToChangeS > 0) {
    if (el < hbToChangeS) return hbDay ? 1 : 0;
    if (el < hbToChangeS + FLIP_VALID_S) return hbDay ? 0 : 1;
    return -1;
  }
  if (el * 1000UL < HB_FRESH_MS) return hbDay ? 1 : 0;
  return -1;
}

int32_t msToNight() {            // скільки мс до ночі, -1 якщо невідомо або вже ніч
  if (!hbDay || hbToChangeS == 0 || dayState() != 1) return -1;
  uint32_t el = millis() - lastHbMs, ch = hbToChangeS * 1000UL;
  return el < ch ? (int32_t)(ch - el) : -1;
}

bool runningState() { return state == S_WARMUP || state == S_RUN; }
bool linkOk() { return lastRxMs != 0 && millis() - lastRxMs < LINK_TIMEOUT_MS; }

// генератор стоїть, насос не качає, сторінку ніхто не дивиться — опитуємо рідко
bool quietIdle() {
  if (state != S_IDLE && state != S_FAULT) return false;
  if (pumpOn || pumpTarget) return false;
  if (wifiOn && millis() - wifiLastClientMs < 5000) return false;
  return true;
}

// запланована зупинка: скільки лишилось і чому
bool planStop(uint32_t& leftMs, const char*& why) {
  if (!runningState()) return false;
  uint32_t now = millis(), deadline;
  if (workUntilMs)  { deadline = workUntilMs;                         why = "WORK"; }
  else if (runDark) { deadline = runStartMs + DARK_LIMIT_MS;          why = "DARK"; }
  else              { deadline = runStartMs + cfgMaxRunMin * 60000UL; why = "MAXRUN"; }
  uint32_t hard = runStartMs + HARD_MAX_MS;
  if ((int32_t)(deadline - hard) > 0) { deadline = hard; why = "HARDMAX"; }
  leftMs = ((int32_t)(deadline - now) > 0) ? deadline - now : 0;
  if (!workUntilMs && !runDark) {
    int32_t n = msToNight();
    if (n >= 0 && (uint32_t)n < leftMs) { leftMs = n; why = "NIGHT"; }
  }
  // зв'язок із SITE пропав: працюємо за останнім прогнозом зарядки + запас
  if (!workUntilMs && !linkOk() && hbChargeMin >= 0 && lastHbMs) {
    uint32_t dl = lastHbMs + (uint32_t)hbChargeMin * 60000UL + NOLINK_MARGIN_MS;
    uint32_t l2 = ((int32_t)(dl - now) > 0) ? dl - now : 0;
    if (l2 < leftMs) { leftMs = l2; why = "NOLINK"; }
  }
  return true;
}

uint8_t warnBits() {
  uint8_t w = warnSticky;
  if (fuelStep == 1) w |= W_LOWFUEL;
  if (fuelStep == 0) w |= W_FUELSENSOR;
  if (dayState() < 0) w |= W_NOTIME;
  return w;
}
void warnText(uint8_t w, char* out, size_t n) {
  out[0] = 0;
  if (!w) { snprintf(out, n, "-"); return; }
  for (int i = 0; i < 4; i++) if (w & (1 << i)) {
    if (out[0]) strlcat(out, "+", n);
    strlcat(out, WARN_NAMES[i], n);
  }
}

long refuelInS() {               // через скільки секунд дозаправка, -1 якщо не запланована
  if (!refuelAtMs) return -1;
  int32_t r = (int32_t)(refuelAtMs - millis());
  return r > 0 ? r / 1000 : 0;
}

// ===================================================================
//                              ЧОК
// ===================================================================
bool engineHot() {
  if (engPresent && engOk) return engT >= HOT_ENGINE_C;
  return lastStopMs != 0 && millis() - lastStopMs < HOT_WINDOW_MS;
}
uint32_t calcRampMs() {          // час плавного відкриття чока залежно від температури
  float t = (engPresent && engOk) ? engT : (tempOk ? tempC : 5.0f);
  uint32_t ms = (t < 0) ? 30000 : (t < 10 ? 15000 : 5000);
  if (earlyStalls > 0) ms *= 2;  // після невдалої спроби відкриваємо повільніше
  if (ms > 60000) ms = 60000;
  return ms;
}

// ===================================================================
//                              ПРОТОКОЛ
// ===================================================================
uint8_t crcOf(const String& s) { uint8_t c = 0; for (size_t i = 0; i < s.length(); i++) c ^= (uint8_t)s[i]; return c; }

void sendFrame(const String& body) {
  char tail[6]; snprintf(tail, sizeof(tail), "*%02X", crcOf(body));
  String f = "$" + body + tail;
  Serial1.print(f); Serial1.print('\n');
  if (rawOn) { Serial.print("> "); Serial.println(f); }
}

uint32_t runSeconds() {
  if (state == S_WARMUP || state == S_RUN || state == S_COOLDOWN) return (millis() - runStartMs) / 1000;
  return lastRunS;
}

void sendStatus() {
  char b[256], w[64];
  int tempD = tempOk ? (int)lroundf(tempC * 10) : -1270;
  int engD  = (engPresent && engOk) ? (int)lroundf(engT * 10) : -1270;
  uint32_t left = 0; const char* why = "-";
  long stopIn = planStop(left, why) ? (long)(left / 1000) : -1;
  if (stopIn < 0) why = "-";
  warnText(warnBits(), w, sizeof(w));
  snprintf(b, sizeof(b), "ST,%s,%s,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%s,%ld,%s,%s,%d,%s,%d,%ld,%d,%d,%d",
           STATE_NAMES[state], FAULT_NAMES[fault], (int)lroundf(vbat * 100), fuelStep, fuelMv,
           tempD, ac ? 1 : 0, ssrOn ? 1 : 0, (unsigned long)runSeconds(), attempts, linkOk() ? 1 : 0,
           stopReason, stopIn, why, w, engD, runSrc, pumpTarget,
           refuelInS(), refuelAtMs ? refuelTarget : 0, pumpAfter, wifiOn ? 1 : 0);
  sendFrame(b);
}

void sendAck(const String& seq, const char* err) {
  if (err == nullptr || err[0] == 0) sendFrame("ACK," + seq + ",OK");
  else sendFrame("ACK," + seq + ",ERR," + err);
}

void sendCfg() {
  char b[100];
  snprintf(b, sizeof(b), "CFG,%lu,%lu,%lu,%lu,%lu,%lu,%s",
           (unsigned long)cfgMaxRunMin, (unsigned long)cfgWarmupS, (unsigned long)cfgCooldownS,
           (unsigned long)cfgPumpMaxS, (unsigned long)cfgPumpStepS, (unsigned long)cfgRefuelDlyMin, FW_VERSION);
  sendFrame(b);
}

// ===================================================================
//                         НАСОС І ДОЗАПРАВКА
// ===================================================================
void refuelSave() { prefs.putInt("rf_t", refuelAtMs ? refuelTarget : 0); }

void refuelSchedule(int target, uint32_t delayMs, const char* why) {
  refuelTarget = target;
  refuelAtMs = millis() + delayMs;
  if (refuelAtMs == 0) refuelAtMs = 1;
  refuelSave();
  logEv("Дозаправка до сходинки %d через %lu хв (%s)", target, (unsigned long)(delayMs / 60000UL), why);
}
void refuelCancel(const char* why) {
  if (!refuelAtMs) return;
  refuelAtMs = 0; refuelTarget = 0;
  refuelSave();
  logEv("Дозаправку скасовано (%s)", why);
}

void pumpStop(const char* msg) {
  if (pumpOn || pumpTarget) {
    setPump(false); pumpTarget = 0;
    if (msg) logEv("%s", msg);
  }
}

const char* pumpStart(int n, const char* why) {
  if (state == S_FAULT) return "FAULT";
  if (n < 2 || n > 5) return "BAD_VALUE";
  fuelSample();                              // у простої останній вимір міг бути хвилину тому
  int raw = fuelStepFromMv(fuelMv);
  if (raw == 0) return "FUEL_SENSOR";
  if (raw >= n) return "FULL";
  pumpTarget = n; pumpReason = why;
  pumpStartMs = pumpStepMs = millis(); pumpLastStep = raw;
  pumpSeenSeq = fuelSeq; pumpFullCnt = 0; pumpOpenCnt = 0;
  setPump(true);
  logEv("Насос: заправка до сходинки %d (%s)", n, why);
  return "";
}

void pumpTask() {
  if (!pumpTarget) return;
  uint32_t now = millis();
  if (fuelSeq != pumpSeenSeq) {              // рішення лише за новим виміром; сплеск на поплавку не зупиняє насос
    pumpSeenSeq = fuelSeq;
    int raw = fuelStepFromMv(fuelMv);
    pumpOpenCnt = (raw == 0) ? pumpOpenCnt + 1 : 0;
    pumpFullCnt = (raw >= pumpTarget) ? pumpFullCnt + 1 : 0;
    if (pumpOpenCnt >= PUMP_CONFIRM_N) { pumpStop("Обрив датчика палива, насос вимкнено"); return; }
    if (pumpFullCnt >= PUMP_CONFIRM_N) {
      warnSticky &= ~W_CANISTER;
      char b[96]; snprintf(b, sizeof(b), "Заправлено до сходинки %d, насос вимкнено", raw);
      pumpStop(b); return;
    }
    if (raw > pumpLastStep) { pumpLastStep = raw; pumpStepMs = now; warnSticky &= ~W_CANISTER; }
  }
  if (now - pumpStepMs > cfgPumpStepS * 1000UL) {
    warnSticky |= W_CANISTER;
    pumpStop("Рівень не росте: у каністрі, схоже, закінчилось паливо. Насос вимкнено");
    return;
  }
  if (now - pumpStartMs > cfgPumpMaxS * 1000UL) pumpStop("Таймаут насоса");
}

// робота двигуна закінчилась (генератор у простої): чи плануємо дозаправку
void onRunEnded() {
  bool nofuel = strcmp(stopReason, "NOFUEL") == 0;
  int target = 0;
  if (pumpAfter > 0) target = pumpAfter;                       // явна команда pump after
  else if (pumpAfter == 0) target = strcmp(runSrc, "AUTO") == 0 ? 5 : carryTarget;
  if (pumpAfter < 0) logEv("Дозаправку після роботи пропущено (pump after off)");
  pumpAfter = 0; carryTarget = 0;
  if (target == 0) {
    if (fuelStepFromMv(fuelMv) < 5)
      logEv("Дозаправку не заплановано (запуск %s), бак %d/5", runSrc, fuelStep);
    return;
  }
  uint32_t dly = nofuel ? NOFUEL_REFUEL_MS : cfgRefuelDlyMin * 60000UL;
  refuelSchedule(target, dly, nofuel ? "закінчилось паливо" : "після роботи");
}

// ===================================================================
//                              СТАН-МАШИНА
// ===================================================================
void enterState(State s) {
  bool toIdle = (s == S_IDLE && state != S_IDLE);
  state = s; stateT = millis(); stateEntry = true;
  logEv("стан -> %s", STATE_NAMES[s]);
  if (toIdle && engineRan) { engineRan = false; onRunEnded(); }
  sendStatus();
}
uint32_t inState() { return millis() - stateT; }

void enterFault(Fault f) {
  allOutputsSafe();
  servoMove(CHOKE_OPEN_US);
  fault = f;
  prefs.putUChar("fault", (uint8_t)f);     // аварія переживає перезавантаження; знімається лише reset
  engineRan = false; pumpAfter = 0;        // після аварії дозаправка не планується
  logEv("АВАРІЯ: %s", FAULT_NAMES[f]);
  enterState(S_FAULT);
}

void beginStop(const char* reason, Fault after) {
  stopReason = reason;
  if (after != F_NONE) pendingFault = after;
  if (state == S_RUN) { setSsr(false); enterState(S_COOLDOWN); }
  else if (state != S_IDLE && state != S_FAULT && state != S_STOPPING && state != S_COOLDOWN) enterState(S_STOPPING);
}

void onCatch() {                 // двигун завівся
  chokeFromUs = servoUs;
  chokeRampMsCur = calcRampMs();
  runStartMs = millis();
  acLostSince = 0;
  if (chokeFromUs != CHOKE_OPEN_US)
    logEv("Двигун завівся (спроба %d). Чок відкривається за %lu с", attempts, (unsigned long)(chokeRampMsCur / 1000));
  else
    logEv("Двигун завівся (спроба %d)", attempts);
  enterState(S_WARMUP);
}

bool checkRunLimits() {          // true — зупинку розпочато
  if (tempOk && tempC > OVERTEMP_C) { beginStop("OVERTEMP", F_OVERTEMP); return true; }
  if (!workUntilMs && !runDark && dayState() == 0) { beginStop("NIGHT", F_NONE); return true; }
  uint32_t left; const char* why;
  if (planStop(left, why) && left == 0) { beginStop(why, F_NONE); return true; }
  return false;
}

void runStateMachine() {
  uint32_t now = millis();

  // двигун заглух сам
  if (state == S_WARMUP || state == S_RUN) {
    if (!ac) { if (!acLostSince) acLostSince = now; }
    else acLostSince = 0;
    if (acLostSince) {
      uint32_t lost = now - acLostSince;
      bool early = (acLostSince - runStartMs) < EARLY_STALL_MS;
      if (early && lost > EARLY_CONFIRM_MS) {
        acLostSince = 0; setSsr(false); setStop(false); earlyStalls++; runStartMs = 0;
        logEv("Двигун заглух одразу після запуску: невдала спроба");
        if (attempts >= MAX_ATTEMPTS) { stopReason = "STARTFAIL"; enterFault(F_START_FAIL); }
        else enterState(S_RETRYWAIT);
        return;
      }
      if (!early && lost > STALL_MS) {
        acLostSince = 0; setSsr(false);
        if (fuelStep <= 1) { stopReason = "NOFUEL"; logEv("Двигун заглух: закінчилось паливо"); }
        else { stopReason = "STALL"; pendingFault = F_STALL; logEv("Двигун заглух, а паливо є: можливо, датчик масла"); }
        enterState(S_STOPPING);
        return;
      }
    }
  }

  bool first = stateEntry;
  stateEntry = false;

  switch (state) {
    case S_IDLE:
      if (stopOn || startOn || ssrOn) { setStart(false); setSsr(false); setStop(false); }
      break;

    case S_PRECRANK: {
      if (first) {
        bool noChoke = (runNoChoke || engineHot()) && attempts == 0;   // запасний варіант: 2-га і 3-тя спроби з чоком
        servoMove(noChoke ? CHOKE_OPEN_US : CHOKE_CLOSED_US);
        logEv("Спроба %d: %s", attempts + 1, noChoke ? "без чока" : "з чоком");
        setStop(true);
      }
      uint32_t wait = pumpedThisStart ? PUMP_SETTLE_MS : PRECRANK_MS;
      if (inState() >= wait) { pumpedThisStart = false; attempts++; setStart(true); enterState(S_CRANK); }
      break;
    }

    case S_CRANK:
      if (ac) { setStart(false); onCatch(); break; }
      if (inState() > CRANK_MAX_MS) { setStart(false); enterState(S_CRANKWAIT); }
      break;

    case S_CRANKWAIT:
      if (ac) { onCatch(); break; }
      if (inState() > CRANK_WAIT_MS) {
        if (attempts >= MAX_ATTEMPTS) { stopReason = "STARTFAIL"; enterFault(F_START_FAIL); }
        else { setStop(false); enterState(S_RETRYWAIT); }
      }
      break;

    case S_RETRYWAIT:
      if (inState() > RETRY_PAUSE_MS) enterState(S_PRECRANK);
      break;

    case S_WARMUP: {
      if (servoUs != CHOKE_OPEN_US) {                  // чок відкривається одразу після запуску, плавно
        uint32_t t = inState();
        int us = (t >= chokeRampMsCur) ? CHOKE_OPEN_US
               : chokeFromUs + (int)((long)(CHOKE_OPEN_US - chokeFromUs) * (long)t / (long)chokeRampMsCur);
        static uint32_t lastStep = 0;
        if (now - lastStep >= 100 || us == CHOKE_OPEN_US) {
          servoWrite(us); lastStep = now;
          if (us == CHOKE_OPEN_US) logEv("Чок відкрито");
        }
      }
      if (checkRunLimits()) break;
      if (servoUs == CHOKE_OPEN_US && inState() > cfgWarmupS * 1000UL) { setSsr(true); enterState(S_RUN); }
      break;
    }

    case S_RUN:
      checkRunLimits();
      break;

    case S_COOLDOWN:
      if (!ac || inState() > cfgCooldownS * 1000UL) enterState(S_STOPPING);
      break;

    case S_STOPPING:
      if (first) {
        setSsr(false); setStart(false); setStop(false);
        if (pumpTarget) pumpStop("Насос вимкнено: зупинка");
        servoMove(CHOKE_OPEN_US);
      }
      if (!ac && inState() > 2000) {
        lastRunS = runStartMs ? (now - runStartMs) / 1000 : 0;
        runStartMs = 0; workUntilMs = 0; lastStopMs = now;
        if (pendingFault != F_NONE) { Fault f = pendingFault; pendingFault = F_NONE; enterFault(f); break; }
        enterState(S_IDLE);                            // тут же планується дозаправка (onRunEnded)
        break;
      }
      if (ac && inState() > STOP_WAIT_MS) { stopReason = "STOPFAIL"; enterFault(F_STOP_FAIL); }
      break;

    case S_FAULT:
      if (stopOn || startOn || ssrOn || pumpOn) allOutputsSafe();
      break;

    case S_MANUAL:
      if (startOn && (ac || (int32_t)(now - crankUntil) >= 0)) {
        setStart(false);
        logEv("%s", ac ? "Двигун завівся, стартер вимкнено" : "Стартер вимкнено");
      }
      break;
  }

  // зняти PWM після руху серво; у простої — знеструмити серво
  if (servoReleaseAt && (int32_t)(now - servoReleaseAt) >= 0) {
    ledcWrite(PIN_SERVO, 0); servoReleaseAt = 0;
    bool keep = state == S_WARMUP || state == S_RUN || state == S_COOLDOWN || state == S_PRECRANK ||
                state == S_CRANK || state == S_CRANKWAIT || (state == S_MANUAL && ac);
    if (!keep) servoOff();
  }
}

// ===================================================================
//                  КОМАНДИ (спільні для SITE, Serial і сторінки)
// ===================================================================
const char* cmdStart(bool autoSrc, bool nochoke, bool dark) {
  if (state == S_FAULT) return "FAULT";
  if (state == S_MANUAL) {
    if (autoSrc) return "BUSY";
    if (ac) return "AC_PRESENT";
    setStart(false); setSsr(false); setStop(false);
  } else if (state != S_IDLE) return "BUSY";
  if (ac) return "AC_PRESENT";
  if (autoSrc && dark) return "BAD_VALUE";
  if (autoSrc && (lastHbMs == 0 || millis() - lastHbMs > HB_FRESH_MS)) return "NO_TIME";
  if (!dark) { int d = dayState(); if (d == 0) return "NIGHT"; if (d < 0) return "NO_TIME"; }
  if (vbat < MIN_START_V) return "LOW_BAT";

  bool wasPumping = pumpTarget != 0;
  int carry = 0;                              // незроблена дозаправка переноситься на кінець цієї роботи
  if (refuelAtMs) { carry = refuelTarget; refuelAtMs = 0; refuelTarget = 0; refuelSave(); }
  if (wasPumping) { if (strcmp(pumpReason, "POSTRUN") == 0 && pumpTarget > carry) carry = pumpTarget;
                    pumpStop("Заправку перервано: запуск"); }
  carryTarget = carry;
  if (carry) logEv("Дозаправку до сходинки %d перенесено на після роботи", carry);
  runSrc = autoSrc ? "AUTO" : (dark ? "DARK" : "MANUAL");
  runDark = dark; runNoChoke = nochoke; workUntilMs = 0;
  attempts = 0; earlyStalls = 0; pendingFault = F_NONE; stopReason = "-";
  pumpedThisStart = wasPumping;
  logEv("Запуск: %s%s%s", runSrc, nochoke ? ", без чока" : "", dark ? ", ніч" : "");
  enterState(S_PRECRANK);
  return "";
}

const char* cmdStop(const char* why) {
  if (state == S_IDLE || state == S_FAULT) return "";
  beginStop(why, F_NONE);
  return "";
}

void cmdKill() {
  pumpStop(nullptr);
  allOutputsSafe();
  servoMove(CHOKE_OPEN_US);
  logEv("KILL: запалювання заглушено, усе вимкнено");
  if (state == S_FAULT) return;              // аварія лишається до RESET
  stopReason = "KILL";
  enterState(ac ? S_STOPPING : S_IDLE);
}

bool engineSessionActive() {     // йде запуск або робота (зокрема ручна)
  return state == S_PRECRANK || state == S_CRANK || state == S_CRANKWAIT || state == S_RETRYWAIT ||
         state == S_WARMUP || state == S_RUN || state == S_COOLDOWN || state == S_STOPPING ||
         (state == S_MANUAL && (ac || engineRan || stopOn));
}

const char* cmdWork(long minutes) {          // minutes < 0 — скасувати
  bool active = state == S_PRECRANK || state == S_CRANK || state == S_CRANKWAIT ||
                state == S_RETRYWAIT || state == S_WARMUP || state == S_RUN;
  if (!active) return "NOT_RUNNING";
  if (minutes < 0) { workUntilMs = 0; logEv("work off: діють звичайні правила"); return ""; }
  if (minutes < 1 || minutes > (long)(HARD_MAX_MS / 60000UL)) return "BAD_VALUE";
  workUntilMs = millis() + (uint32_t)minutes * 60000UL;
  if (workUntilMs == 0) workUntilMs = 1;
  logEv("work: зупинка через %ld:%02ld", minutes / 60, minutes % 60);
  return "";
}

const char* cmdPump(int n) {
  if (n == 0) { pumpStop("Насос вимкнено"); return ""; }
  return pumpStart(n, "CMD");
}

// n: 2..5 — після роботи дозаправити до сходинки; 0 — не дозаправляти (off)
const char* cmdPumpAfter(int n) {
  if (n != 0 && (n < 2 || n > 5)) return "BAD_VALUE";
  if (state == S_FAULT) return "FAULT";
  if (engineSessionActive()) {               // йде робота — виконається після неї
    pumpAfter = (n == 0) ? -1 : n;
    if (n == 0) logEv("pump after off: після цієї роботи не дозаправляти");
    else logEv("pump after: після роботи дозаправити до сходинки %d", n);
    return "";
  }
  // генератор стоїть — планується одразу (з тією самою затримкою) або скасовується
  if (n == 0) { refuelCancel("pump after off"); pumpAfter = 0; return ""; }
  fuelSample();
  if (fuelStepFromMv(fuelMv) >= n) return "FULL";
  refuelSchedule(n, cfgRefuelDlyMin * 60000UL, "pump after");
  return "";
}

const char* cmdReset() {
  if (state != S_FAULT) return "NOT_FAULT";
  if (ac) return "AC_PRESENT";
  fault = F_NONE; stopReason = "-"; attempts = 0; earlyStalls = 0;
  prefs.putUChar("fault", 0);
  logEv("Аварію скинуто");
  enterState(S_IDLE);
  return "";
}

const char* cmdSet(String k, uint32_t v) {
  k.toUpperCase();
  if (k == "MAXRUN" && v >= 10 && v <= 480)          { cfgMaxRunMin = v;    prefs.putUInt("maxrun", v); }
  else if (k == "WARMUP" && v >= 10 && v <= 600)     { cfgWarmupS = v;      prefs.putUInt("warmup", v); }
  else if (k == "COOLDOWN" && v <= 600)              { cfgCooldownS = v;    prefs.putUInt("cooldown", v); }
  else if (k == "PUMPMAX" && v >= 60 && v <= 1800)   { cfgPumpMaxS = v;     prefs.putUInt("pumpmax2", v); }
  else if (k == "PUMPSTEP" && v >= 30 && v <= 900)   { cfgPumpStepS = v;    prefs.putUInt("pumpstep", v); }
  else if (k == "REFUELDLY" && v <= 240)             { cfgRefuelDlyMin = v; prefs.putUInt("refueldly", v); }
  else return "BAD_VALUE";
  logEv("Налаштування: %s = %lu", k.c_str(), (unsigned long)v);
  sendCfg();
  return "";
}

// ===================================================================
//                       КОМАНДИ ВІД SITE
// ===================================================================
int splitCsv(const String& s, String out[], int maxN) {
  int n = 0, from = 0;
  while (n < maxN) {
    int c = s.indexOf(',', from);
    if (c < 0) { out[n++] = s.substring(from); break; }
    out[n++] = s.substring(from, c); from = c + 1;
  }
  return n;
}

void handleBody(const String& body) {
  String t[8];
  int n = splitCsv(body, t, 8);
  if (n == 0) return;

  if (t[0] == "HB") {                        // HB,<seq>,<unix>,<DAY|NIGHT>,<сек до зміни>,<хв зарядки>
    lastHbMs = millis();
    if (n >= 4) hbDay = (t[3] == "DAY");
    hbToChangeS = (n >= 5) ? (uint32_t)t[4].toInt() : 0;
    hbChargeMin = (n >= 6 && t[5].length() > 0) ? (int32_t)t[5].toInt() : -1;
    sendStatus();
    return;
  }
  if (t[0] != "CMD" || n < 3) return;

  String seq = t[1], cmd = t[2];
  if (cmd == "START") {
    bool autoSrc = (n >= 4 && t[3] == "AUTO");
    bool nochoke = false, dark = false;
    for (int i = 4; i < n; i++) { if (t[i] == "NOCHOKE") nochoke = true; else if (t[i] == "DARK") dark = true; }
    sendAck(seq, cmdStart(autoSrc, nochoke, dark));
  } else if (cmd == "STOP") {
    sendAck(seq, cmdStop("CMD"));
  } else if (cmd == "KILL") {
    cmdKill(); sendAck(seq, "");
  } else if (cmd == "STATUS") {
    sendAck(seq, ""); sendStatus();
  } else if (cmd == "WORK" && n >= 4) {
    sendAck(seq, cmdWork(t[3] == "OFF" ? -1 : t[3].toInt()));
    sendStatus();
  } else if (cmd == "PUMP" && n >= 4) {
    sendAck(seq, cmdPump(t[3].toInt()));
  } else if (cmd == "PUMPAFTER" && n >= 4) {
    sendAck(seq, cmdPumpAfter(t[3] == "OFF" ? 0 : t[3].toInt()));
    sendStatus();
  } else if (cmd == "WIFI" && n >= 4) {
    if (t[3] == "ON") { wifiStart("SITE"); sendAck(seq, ""); }
    else if (t[3] == "OFF") { wifiStop("SITE"); sendAck(seq, ""); }
    else sendAck(seq, "BAD_VALUE");
  } else if (cmd == "RESET") {
    sendAck(seq, cmdReset());
  } else if (cmd == "SET" && n >= 5) {
    sendAck(seq, cmdSet(t[3], (uint32_t)t[4].toInt()));
  } else if (cmd == "GET") {
    sendAck(seq, ""); sendCfg();
  } else {
    sendAck(seq, "UNKNOWN");
  }
}

// ===================================================================
//                SERIAL / СТОРІНКА (оператор і сервіс)
// ===================================================================
const char* errText(const char* e) {
  if (!strcmp(e, "BUSY"))        return "генератор зайнятий (вже запускається або працює)";
  if (!strcmp(e, "FAULT"))       return "активна аварія, спершу reset";
  if (!strcmp(e, "NIGHT"))       return "зараз ніч. Для примусового запуску: start dark";
  if (!strcmp(e, "NO_TIME"))     return "час невідомий (немає HB від SITE). Для запуску: start dark";
  if (!strcmp(e, "AC_PRESENT"))  return "двигун уже працює (230 В є)";
  if (!strcmp(e, "LOW_BAT"))     return "стартерний АКБ розряджений (нижче 10,5 В)";
  if (!strcmp(e, "FUEL_SENSOR")) return "обрив датчика палива";
  if (!strcmp(e, "FULL"))        return "бак уже заповнений до цієї сходинки";
  if (!strcmp(e, "NOT_FAULT"))   return "аварії немає";
  if (!strcmp(e, "BAD_VALUE"))   return "неправильне значення";
  if (!strcmp(e, "NOT_RUNNING")) return "генератор не працює";
  return e;
}

bool report(const char* err, const char* okMsg) {
  if (err[0] == 0) { if (okMsg) say("%s", okMsg); return true; }
  say("Не виконано: %s", errText(err));
  return false;
}

String fmtHM(uint32_t ms) {
  uint32_t m = (ms + 59999UL) / 60000UL;
  char b[12]; snprintf(b, sizeof(b), "%lu:%02lu", (unsigned long)(m / 60), (unsigned long)(m % 60));
  return String(b);
}

void printMon() {
  const char* choke = (servoUs == CHOKE_OPEN_US) ? "відкритий"
                    : (servoUs == CHOKE_CLOSED_US) ? "закритий" : "відкривається";
  static const char* FUEL_TXT[] = { "обрив датчика", "мало", "~25%", "~50%", "~75%", "повний" };
  char t[32], e[48];
  if (tempOk) snprintf(t, sizeof(t), "%.1f °C", tempC); else snprintf(t, sizeof(t), "нема даних");
  e[0] = 0;
  if (engPresent) { if (engOk) snprintf(e, sizeof(e), " | двиг. %.1f °C", engT); else snprintf(e, sizeof(e), " | двиг. помилка"); }
  Serial.printf("[%s] АКБ %.2f В | t АКБ %s%s | чок %s | паливо %d/5 %s (%d мВ) | 230 В: %s\n",
                STATE_NAMES[state], vbat, t, e, choke, fuelStep, FUEL_TXT[fuelStep], fuelMv, ac ? "Є" : "немає");
  Serial.printf("        реле: запалювання=%s  стартер=%s  SSR=%s  насос=%s",
                stopOn ? "дозволене" : "заглушене", startOn ? "ON" : "off", ssrOn ? "ON" : "off", pumpOn ? "ON" : "off");
  if (pumpTarget) Serial.printf(" (до сходинки %d)", pumpTarget);
  if (wifiOn) Serial.printf(" | Wi-Fi: %d підкл.", (int)WiFi.softAPgetStationNum());
  Serial.println();

  String l = "        ";
  int d = dayState();
  l += (d == 1) ? "день" : (d == 0) ? "ніч" : "час невідомий";
  if (state == S_WARMUP || state == S_RUN || state == S_COOLDOWN) l += " | робота " + fmtHM(runSeconds() * 1000UL);
  uint32_t left; const char* why;
  if (planStop(left, why)) { l += " | зупинка через " + fmtHM(left) + " ("; l += why; l += ")"; }
  if (refuelAtMs) l += " | дозаправка до " + String(refuelTarget) + " через " + String((refuelInS() + 59) / 60) + " хв";
  else if (state == S_IDLE && fuelStep > 0 && fuelStep < 5) l += " | дозаправка: не запланована";
  if (pumpAfter > 0) l += " | pump after " + String(pumpAfter);
  else if (pumpAfter < 0) l += " | pump after off";
  char w[64]; uint8_t wb = warnBits() & ~W_NOTIME;
  if (wb) { warnText(wb, w, sizeof(w)); l += " | попередження: "; l += w; }
  if (fault != F_NONE) { l += " | АВАРІЯ: "; l += FAULT_NAMES[fault]; }
  if (strcmp(stopReason, "-") != 0 && state == S_IDLE) { l += " | остання зупинка: "; l += stopReason; }
  Serial.println(l);
}

void printHelp() {
  Serial.println("--- Команди ---");
  Serial.println(" start                 запуск (вночі не заводиться). Працює до STOP від SITE, 5 год або ночі");
  Serial.println(" start nochoke         те саме, перша спроба без чока");
  Serial.println(" start dark            примусовий запуск уночі, без work — максимум 30 хв");
  Serial.println(" work 2:30 | work 45   зупинитися через 2 год 30 хв / 45 хв від цієї команди (макс 8 год)");
  Serial.println(" work off              скасувати таймер");
  Serial.println(" pump 4 | pump off     заправити зараз до 4-ї сходинки (2..5) / зупинити насос");
  Serial.println(" pump after [4] | off  після роботи дозаправити (до повного / до сходинки) / не дозаправляти");
  Serial.println(" stop                  зупинка з охолодженням");
  Serial.println(" kill                  негайна зупинка, вимкнути все");
  Serial.println(" reset                 скинути аварію");
  Serial.println(" set KEY VALUE         налаштування: maxrun warmup cooldown pumpmax pumpstep refueldly");
  Serial.println(" wifi on | off         точка доступу GEN-3000, сторінка 192.168.4.1");
  Serial.println(" st | log | mon on|off | raw on|off   статус / журнал / статус кожні 5 с / кадри протоколу");
  Serial.println("--- Сервіс (ручний режим) ---");
  Serial.println(" run                   дозволити запалювання");
  Serial.println(" crank [мс]            лише стартер, до 3000 мс");
  Serial.println(" choke open | close    чок (o / c)");
  Serial.println(" ssr on | off          230 В на зарядні");
  Serial.println(" auto                  вийти з ручного режиму (двигун працює — передати автоматиці)");
}

void printLog() {
  for (int i = 0; i < logCount; i++) {
    const LogEv& e = logBuf[(logHead - logCount + i + LOG_N) % LOG_N];
    Serial.printf("  -%4lu хв  %s\n", (unsigned long)((millis() - e.ms) / 60000UL), e.txt);
  }
}

bool serviceAllowed() {
  if (state == S_IDLE || state == S_MANUAL) return true;
  say("Зараз працює автоматика (стан %s). Спершу stop або kill", STATE_NAMES[state]);
  return false;
}
void enterManual() { if (state != S_MANUAL) { stopReason = "-"; runSrc = "SERVICE"; enterState(S_MANUAL); } }

long parseDuration(const String& s) {        // "2:30" -> 150, "45" -> 45, помилка -> -1
  int c = s.indexOf(':');
  if (c < 0) { long m = s.toInt(); return (m > 0) ? m : -1; }
  long h = s.substring(0, c).toInt(), m = s.substring(c + 1).toInt();
  if (h < 0 || m < 0 || m > 59 || (h == 0 && m == 0)) return -1;
  return h * 60 + m;
}

void handleService(String s) {
  s.trim(); s.toLowerCase();
  String w[4]; int n = 0, from = 0;
  while (n < 4) {
    while (from < (int)s.length() && s[from] == ' ') from++;
    if (from >= (int)s.length()) break;
    int sp = s.indexOf(' ', from);
    if (sp < 0) { w[n++] = s.substring(from); break; }
    w[n++] = s.substring(from, sp); from = sp + 1;
  }
  if (n == 0) return;
  const String& c = w[0];

  if (c == "help" || c == "?")   { printHelp(); return; }
  if (c == "st" || c == "status") { printMon(); return; }
  if (c == "log")  { printLog(); return; }
  if (c == "mon") { monOn = (w[1] != "off"); say("Статус кожні 5 с: %s", monOn ? "увімкнено" : "вимкнено"); return; }
  if (c == "raw") { rawOn = (w[1] == "on");  say("Кадри протоколу: %s", rawOn ? "показувати" : "сховати"); return; }
  if (c == "wifi") {
    if (w[1] == "on") wifiStart("команда");
    else if (w[1] == "off") wifiStop("команда");
    else say("Формат: wifi on або wifi off");
    return;
  }

  if (c == "start") {
    bool nc = false, dk = false;
    for (int i = 1; i < n; i++) {
      if (w[i] == "nochoke") nc = true;
      else if (w[i] == "dark") dk = true;
      else { say("Невідомий параметр. Приклад: start, start nochoke, start dark"); return; }
    }
    report(cmdStart(false, nc, dk), dk ? "Запуск прийнято (dark: без work максимум 30 хв)" : "Запуск прийнято");
    return;
  }
  if (c == "stop")  {
    const char* msg = (state == S_IDLE) ? "Генератор і так стоїть" : "Зупинка";
    report(cmdStop("CMD"), msg);
    return;
  }
  if (c == "kill")  { cmdKill(); return; }
  if (c == "reset") { report(cmdReset(), nullptr); return; }
  if (c == "work") {
    if (n < 2) { say("Формат: work 2:30, work 45 (хв) або work off"); return; }
    if (w[1] == "off") { report(cmdWork(-1), nullptr); return; }
    long m = parseDuration(w[1]);
    if (m < 0) { say("Формат: work 2:30 або work 45 (хвилини)"); return; }
    if (report(cmdWork(m), nullptr) && runStartMs && workUntilMs - runStartMs > HARD_MAX_MS)
      say("Увага: обмежено максимумом 8 год від запуску");
    return;
  }
  if (c == "pump") {
    if (n < 2) { say("Формат: pump 4 | pump off | pump after [4] | pump after off"); return; }
    if (w[1] == "after") {
      int k = 5;
      if (n >= 3) k = (w[2] == "off") ? 0 : w[2].toInt();
      report(cmdPumpAfter(k), nullptr);
      return;
    }
    if (w[1] == "off") { cmdPump(0); return; }
    int k = (w[1] == "on") ? 5 : w[1].toInt();
    report(cmdPump(k), nullptr);
    return;
  }
  if (c == "set") {
    if (n < 3) { say("Формат: set maxrun 300 (maxrun warmup cooldown pumpmax pumpstep refueldly)"); return; }
    report(cmdSet(w[1], (uint32_t)w[2].toInt()), "Збережено");
    return;
  }

  // ---- сервіс ----
  if (c == "run") {
    if (!serviceAllowed()) return;
    enterManual(); setStop(true);
    say("Запалювання дозволене (СТОП-реле під струмом)");
    return;
  }
  if (c == "crank") {
    if (!serviceAllowed()) return;
    if (!stopOn) { say("Спершу run"); return; }
    if (ac)      { say("Двигун уже працює (230 В є), стартер заблоковано"); return; }
    if (vbat < MIN_START_V) { say("АКБ %.2f В — замало для стартера", vbat); return; }
    long ms = (n >= 2) ? w[1].toInt() : 2000;
    if (ms < 200) ms = 200;
    if (ms > (long)MANUAL_CRANK_MAX_MS) ms = MANUAL_CRANK_MAX_MS;
    enterManual();
    crankUntil = millis() + (uint32_t)ms;
    setStart(true);
    say("Стартер до %ld мс", ms);
    return;
  }
  if (c == "choke" || c == "o" || c == "c") {
    if (!serviceAllowed()) return;
    bool open = (c == "o") || (c == "choke" && w[1] == "open");
    bool close = (c == "c") || (c == "choke" && w[1] == "close");
    if (!open && !close) { say("Формат: choke open або choke close"); return; }
    enterManual();
    servoMove(open ? CHOKE_OPEN_US : CHOKE_CLOSED_US);
    say("Чок: %s", open ? "відкрито" : "закрито");
    return;
  }
  if (c == "ssr") {
    if (!serviceAllowed()) return;
    if (w[1] != "on" && w[1] != "off") { say("Формат: ssr on або ssr off"); return; }
    enterManual(); setSsr(w[1] == "on");
    say("SSR: %s%s", ssrOn ? "увімкнено" : "вимкнено", (ssrOn && !ac) ? " (230 В поки немає)" : "");
    return;
  }
  if (c == "auto") {
    if (state != S_MANUAL) { say("Ручний режим не активний"); return; }
    setStart(false);
    if (ac && stopOn) {                      // двигун працює — передаємо автоматиці
      setSsr(false);
      runSrc = "MANUAL"; runDark = (dayState() != 1); runNoChoke = false; workUntilMs = 0;
      attempts = 0; earlyStalls = 0; pendingFault = F_NONE; stopReason = "-";
      chokeFromUs = servoUs; chokeRampMsCur = calcRampMs();
      runStartMs = millis() - EARLY_STALL_MS;   // зупинка після передачі — звичайна, без повторних спроб
      acLostSince = 0;
      say("%s", runDark ? "Передаю автоматиці. Ніч або час невідомий: максимум 30 хв, або задай work"
                        : "Передаю автоматиці: прогрів, потім зарядка");
      enterState(S_WARMUP);
    } else {
      setSsr(false); setStop(false);
      servoMove(CHOKE_OPEN_US);
      if (ac) { stopReason = "KILL"; enterState(S_STOPPING); }
      else enterState(S_IDLE);
    }
    return;
  }
  say("Невідома команда. Введи help");
}

void handleLine(String line, bool fromUsb) {
  line.trim();
  if (line.length() == 0) return;
  if (fromUsb && line[0] != '$' && !line.startsWith("CMD") && !line.startsWith("HB")) {
    handleService(line);
    return;
  }
  String body;
  if (line[0] == '$') {
    int star = line.lastIndexOf('*');
    if (star < 0 || star + 3 != (int)line.length()) { badFrames++; return; }
    body = line.substring(1, star);
    uint8_t got = (uint8_t)strtol(line.substring(star + 1).c_str(), nullptr, 16);
    if (got != crcOf(body)) { badFrames++; if (rawOn) Serial.println("Кадр із неправильним CRC"); return; }
  } else if (fromUsb) {
    body = line;                             // з USB дозволено без $ і CRC
  } else { badFrames++; return; }

  if (!fromUsb) lastRxMs = millis();
  if (rawOn || fromUsb) { Serial.print("< "); Serial.println(body); }
  handleBody(body);
}

void pollPort(Stream& s, String& buf, bool fromUsb) {
  while (s.available()) {
    char c = (char)s.read();
    if (c == '\n') { handleLine(buf, fromUsb); buf = ""; }
    else if (c != '\r') { if (buf.length() < 120) buf += c; else buf = ""; }
  }
}

// ===================================================================
//                       WI-FI І ВЕБСТОРІНКА
// ===================================================================
const char PAGE_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="uk"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>GEN-3000</title>
<style>
:root{--bg:#f2f4f1;--card:#fff;--fg:#1b2420;--mut:#5f6c66;--ln:#d9e0dc;--ac:#1f6b4f;--ok:#1f6b4f;--warn:#8a5a10;--bad:#a3352b}
@media(prefers-color-scheme:dark){:root{--bg:#131917;--card:#1b2320;--fg:#e4ebe7;--mut:#97a39d;--ln:#2c3833;--ac:#2f8a66;--ok:#7fd1ab;--warn:#e6b766;--bad:#f0998e}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif;padding:12px max(12px,env(safe-area-inset-left))}
h1{font-size:20px;margin:4px 0 10px}h2{font-size:13px;text-transform:uppercase;letter-spacing:.06em;color:var(--mut);margin:0 0 8px}
.c{background:var(--card);border:1px solid var(--ln);border-radius:10px;padding:12px;margin-bottom:12px}
.g{display:grid;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));gap:8px}
.k{font-size:12px;color:var(--mut)}.v{font-size:17px;font-weight:600;font-variant-numeric:tabular-nums}
.st{display:inline-block;padding:3px 10px;border-radius:99px;font-weight:600;background:var(--ln)}
.st.ok{background:var(--ok);color:var(--card)}.st.bd{background:var(--bad);color:var(--card)}
.ok{color:var(--ok)}.wn{color:var(--warn)}.bd{color:var(--bad)}
.row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin:6px 0}
button{font:inherit;padding:10px 14px;border-radius:8px;border:1px solid var(--ln);background:var(--bg);color:var(--fg);min-height:44px}
button.p{background:var(--ac);color:#fff;border-color:var(--ac);font-weight:600}button.r{background:#b3382c;color:#fff;border-color:#b3382c}
button:disabled{opacity:.35}body.busy button{pointer-events:none;opacity:.6}
input,select{font:inherit;padding:9px;border-radius:8px;border:1px solid var(--ln);background:var(--bg);color:var(--fg);min-height:44px;max-width:130px}
.fb{display:flex;gap:3px;margin-top:5px}.fb i{flex:1;height:8px;border-radius:2px;background:var(--ln)}.fb i.on{background:var(--ok)}.fb i.w{background:var(--warn)}
#out{white-space:pre-wrap;font:13px ui-monospace,monospace;color:var(--mut);min-height:1.4em}
#log div{font-size:13px;border-top:1px solid var(--ln);padding:4px 0}#log span{color:var(--mut);display:inline-block;min-width:64px}
#plan{color:var(--mut);margin-top:8px}#alerts div{margin-top:4px}#foot{text-align:center;margin:4px 0 16px}
</style></head><body>
<h1>Генератор GEN-3000 <span id="state" class="st">…</span></h1>
<div class="c"><div class="g" id="grid"></div><div id="plan"></div><div id="alerts"></div></div>
<div class="c"><h2>PIN</h2><div class="row"><input id="pin" type="password" inputmode="numeric" placeholder="PIN"><span class="k">потрібен для дій</span></div>
<div id="out"></div></div>
<div class="c"><h2>Запуск і зупинка</h2>
<div class="row"><button data-s="IDLE MANUAL" data-hi="IDLE" onclick="cmd('start')">Старт</button><button data-s="IDLE MANUAL" onclick="cmd('start nochoke')">Старт без чока</button><button data-s="IDLE MANUAL" onclick="if(confirm('Нічний запуск?'))cmd('start dark')">Старт уночі</button></div>
<div class="row"><button data-s="PRECRANK CRANK CRANKWAIT RETRYWAIT WARMUP RUN MANUAL" data-hi="PRECRANK CRANK CRANKWAIT RETRYWAIT WARMUP RUN" onclick="cmd('stop')">Стоп</button><button class="r" onclick="if(confirm('Негайно заглушити і вимкнути все?'))cmd('kill')">KILL</button><button data-s="FAULT" data-hi="FAULT" onclick="if(confirm('Скинути аварію? Спершу переконайся, що причину усунено.'))cmd('reset')">Скинути аварію</button></div>
<div class="row"><input id="work" placeholder="2:30"><button data-s="PRECRANK CRANK CRANKWAIT RETRYWAIT WARMUP RUN" onclick="cmd('work '+v('work'))">Працювати</button><button data-s="PRECRANK CRANK CRANKWAIT RETRYWAIT WARMUP RUN" onclick="cmd('work off')">Скасувати таймер</button></div></div>
<div class="c"><h2>Паливо</h2>
<div class="row"><select id="lvl"><option>5</option><option>4</option><option>3</option><option>2</option></select>
<button data-x="FAULT" onclick="cmd('pump '+v('lvl'))">Заправити зараз</button><button onclick="cmd('pump off')">Зупинити насос</button></div>
<div class="row"><button data-x="FAULT" onclick="cmd('pump after '+v('lvl'))">Дозаправити після роботи</button><button data-x="FAULT" onclick="cmd('pump after off')">Не дозаправляти</button></div></div>
<div class="c"><h2>Сервіс</h2>
<div class="row"><button data-s="IDLE MANUAL" onclick="cmd('run')">Запалювання</button><button data-s="IDLE MANUAL" onclick="cmd('crank 1500')">Стартер 1,5 с</button><button data-s="IDLE MANUAL" onclick="cmd('choke close')">Чок закрити</button><button data-s="IDLE MANUAL" onclick="cmd('choke open')">Чок відкрити</button></div>
<div class="row"><button data-s="IDLE MANUAL" onclick="cmd('ssr on')">SSR увімк.</button><button data-s="IDLE MANUAL" onclick="cmd('ssr off')">SSR вимк.</button><button data-s="MANUAL" data-hi="MANUAL" onclick="cmd('auto')">Вийти з ручного</button></div></div>
<div class="c"><h2>Налаштування</h2><div class="g" id="cfg"></div></div>
<div class="c"><h2>Журнал</h2><div id="log"></div></div>
<div class="c"><h2>Оновлення прошивки</h2><div class="row"><input type="file" id="fw" accept=".bin" style="max-width:100%"><button data-s="IDLE FAULT" onclick="ota()">Завантажити</button></div><div class="k">Лише коли генератор стоїть. Файл .bin: Sketch → Export Compiled Binary.</div></div>
<div class="k" id="foot"></div>
<script>
const $=id=>document.getElementById(id),v=id=>$(id).value.trim();
const esc=x=>String(x).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
try{$('pin').value=sessionStorage.getItem('pin')||''}catch(e){}
$('pin').oninput=()=>{try{sessionStorage.setItem('pin',$('pin').value)}catch(e){}};
const ST={IDLE:'Стоїть',PRECRANK:'Підготовка',CRANK:'Стартер',CRANKWAIT:'Чекає запуску',RETRYWAIT:'Пауза між спробами',WARMUP:'Прогрів',RUN:'Заряджає',COOLDOWN:'Охолодження',STOPPING:'Глушиться',FAULT:'АВАРІЯ',MANUAL:'Ручний режим'};
const W={LOWFUEL:'Мало палива',CANISTER:'Каністра порожня',FUELSENSOR:'Обрив датчика палива',NOTIME:'Час невідомий (немає зв’язку з SITE)'};
const CL={maxrun:'Макс. робота, хв',warmup:'Прогрів, с',cooldown:'Охолодження, с',pumpmax:'Насос максимум, с',pumpstep:'Каністра порожня, якщо сходинка не росте, с',refueldly:'Дозаправка після роботи, хв'};
const hm=s=>s<0?'—':Math.floor(s/3600)+':'+String(Math.floor(s%3600/60)).padStart(2,'0');
const has=(l,x)=>l.split(' ').includes(x);
let logs=[],logAt=0,logN=-1,cfgJ='',busy=false,otaBusy=false,tm=null;
function cell(k,val,cls){return '<div><div class="k">'+k+'</div><div class="v '+(cls||'')+'">'+val+'</div></div>'}
function fuelBar(n){let h='<div class="fb">';for(let i=1;i<=5;i++)h+='<i class="'+(i<=n?(n<=1?'w':'on'):'')+'"></i>';return h+'</div>'}
function gate(st){
document.querySelectorAll('button[data-s]').forEach(b=>b.disabled=!has(b.dataset.s,st));
document.querySelectorAll('button[data-x]').forEach(b=>b.disabled=has(b.dataset.x,st));
document.querySelectorAll('button[data-hi]').forEach(b=>b.classList.toggle('p',has(b.dataset.hi,st)))}
async function load(){if(otaBusy)return;try{const r=await fetch('/api/status');const s=await r.json();
$('state').textContent=ST[s.state]||s.state;$('state').className='st '+(s.state=='FAULT'?'bd':s.state=='RUN'?'ok':'');
gate(s.state);
const t=x=>x===null?'—':x.toFixed(1)+' °C';
$('grid').innerHTML=cell('230 В',s.ac?'Є':'немає',s.ac?'ok':'')+cell('АКБ стартера',s.vbat.toFixed(2)+' В',s.vbat<10.5?'bd':s.vbat<11.4?'wn':'')+
cell('Паливо',(s.fuel?s.fuel+'/5':'обрив')+fuelBar(s.fuel),s.fuel==0?'bd':s.fuel==1?'wn':'')+cell('t АКБ',t(s.tbat))+cell('t двигуна',t(s.teng))+
cell('Чок',s.choke)+cell('Час роботи',hm(s.run))+cell('Зв’язок SITE',s.link?'є':'немає',s.link?'':'wn')+
cell('Реле',(s.ign?'запал. ':'')+(s.starter?'стартер ':'')+(s.ssr?'SSR ':'')+(s.pump?'насос→'+s.pumpTo:'')||'—')+cell('Період',s.day==1?'день':s.day==0?'ніч':'невідомо');
let p=[];if(s.stopIn>=0)p.push('зупинка через '+hm(s.stopIn)+' ('+s.plan+')');
if(s.state=='IDLE'&&s.day!=1)p.push('звичайний старт не заведе ('+(s.day==0?'ніч':'час невідомий')+'), лише «Старт уночі»');
if(s.refuelIn>=0)p.push('дозаправка до '+s.refuelTo+' через '+Math.ceil(s.refuelIn/60)+' хв');else if(s.state=='IDLE'&&s.fuel>0&&s.fuel<5)p.push('дозаправка не запланована');
if(s.after>0)p.push('після роботи: дозаправити до '+s.after);if(s.after<0)p.push('після роботи: не дозаправляти');
if(s.reason!='-')p.push('остання зупинка: '+s.reason);$('plan').textContent=p.join(' · ');
let a=(s.warn=='-'?[]:s.warn.split('+')).map(x=>'<div class="wn">'+(W[x]||x)+'</div>');
if(s.fault!='NONE')a.unshift('<div class="bd"><b>Аварія: '+s.fault+'</b> — генератор заблоковано до «Скинути аварію»</div>');$('alerts').innerHTML=a.join('');
const cj=JSON.stringify(s.cfg);
if(cj!=cfgJ&&(!document.activeElement||document.activeElement.tagName!='INPUT')){cfgJ=cj;
$('cfg').innerHTML=Object.entries(s.cfg).map(([k,x])=>'<div><div class="k">'+(CL[k]||k)+'</div><div class="row"><input id="c_'+k+'" value="'+x+'" style="max-width:80px"><button onclick="cmd(\'set '+k+' \'+v(\'c_'+k+'\'))">OK</button></div></div>').join('')}
if(s.logN!==logN){logs=(await (await fetch('/api/log')).json()).reverse();logAt=Date.now();logN=s.logN}
const d=Math.floor((Date.now()-logAt)/1000);
$('log').innerHTML=logs.map(e=>{const g=e.ago+d;return '<div><span>'+(g<60?g+' с':g<3600?Math.floor(g/60)+' хв':hm(g))+'</span> '+esc(e.t)+'</div>'}).join('');
$('foot').textContent='Прошивка '+s.fw+' · датчик палива '+s.fuelMv+' мВ · плата працює '+hm(s.up);
}catch(e){$('state').textContent='немає зв’язку';$('state').className='st'}}
async function cmd(c){if(busy)return;busy=true;document.body.classList.add('busy');$('out').textContent='…';
try{const r=await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'pin='+encodeURIComponent(v('pin'))+'&cmd='+encodeURIComponent(c)});
$('out').textContent=await r.text()}catch(e){$('out').textContent='Помилка зв’язку'}
busy=false;document.body.classList.remove('busy');load()}
function ota(){const f=$('fw').files[0];if(!f){$('out').textContent='Обери файл .bin';return}if(!confirm('Оновити прошивку? Плата перезавантажиться.'))return;
otaBusy=true;$('out').textContent='Завантаження… 0%';const d=new FormData();d.append('f',f);const x=new XMLHttpRequest();
x.open('POST','/update?pin='+encodeURIComponent(v('pin')));
x.upload.onprogress=e=>{if(e.lengthComputable)$('out').textContent='Завантаження… '+Math.round(e.loaded*100/e.total)+'%'};
x.onload=()=>{$('out').textContent=x.responseText;if(x.status==200){$('out').textContent+='\nЧекаю перезапуску… якщо телефон від’єднався від GEN-3000 — підключись знову.';setTimeout(()=>{otaBusy=false;logN=-1;load()},10000)}else otaBusy=false};
x.onerror=()=>{$('out').textContent='Помилка завантаження';otaBusy=false};x.send(d)}
function poll(){clearInterval(tm);tm=null;if(!document.hidden){load();tm=setInterval(load,2000)}}
document.addEventListener('visibilitychange',poll);poll();
</script></body></html>)HTML";

String jsonEsc(const char* s) {
  String o;
  for (const char* p = s; *p; p++) {
    if (*p == '"' || *p == '\\') { o += '\\'; o += *p; }
    else if ((uint8_t)*p < 0x20) o += ' ';
    else o += *p;
  }
  return o;
}

void webStatus() {
  char b[1100];
  uint32_t left = 0; const char* why = "-";
  long stopIn = planStop(left, why) ? (long)(left / 1000) : -1;
  if (stopIn < 0) why = "-";
  char w[64]; warnText(warnBits(), w, sizeof(w));
  const char* choke = (servoUs == CHOKE_OPEN_US) ? "відкритий" : (servoUs == CHOKE_CLOSED_US) ? "закритий" : "відкривається";
  char tb[12], te[12];
  if (tempOk) snprintf(tb, sizeof(tb), "%.1f", tempC); else snprintf(tb, sizeof(tb), "null");
  if (engPresent && engOk) snprintf(te, sizeof(te), "%.1f", engT); else snprintf(te, sizeof(te), "null");
  snprintf(b, sizeof(b),
    "{\"state\":\"%s\",\"fault\":\"%s\",\"vbat\":%.2f,\"fuel\":%d,\"fuelMv\":%d,\"tbat\":%s,\"teng\":%s,"
    "\"ac\":%d,\"ign\":%d,\"starter\":%d,\"ssr\":%d,\"pump\":%d,\"pumpTo\":%d,\"choke\":\"%s\",\"run\":%lu,"
    "\"stopIn\":%ld,\"plan\":\"%s\",\"warn\":\"%s\",\"day\":%d,\"link\":%d,\"reason\":\"%s\",\"src\":\"%s\","
    "\"refuelIn\":%ld,\"refuelTo\":%d,\"after\":%d,\"attempts\":%d,\"fw\":\"%s\",\"logN\":%lu,\"up\":%lu,"
    "\"cfg\":{\"maxrun\":%lu,\"warmup\":%lu,\"cooldown\":%lu,\"pumpmax\":%lu,\"pumpstep\":%lu,\"refueldly\":%lu}}",
    STATE_NAMES[state], FAULT_NAMES[fault], vbat, fuelStep, fuelMv, tb, te,
    ac ? 1 : 0, stopOn ? 1 : 0, startOn ? 1 : 0, ssrOn ? 1 : 0, pumpOn ? 1 : 0, pumpTarget, choke,
    (unsigned long)((state == S_WARMUP || state == S_RUN || state == S_COOLDOWN) ? runSeconds() : 0),
    stopIn, why, w, dayState(), linkOk() ? 1 : 0, stopReason, runSrc,
    refuelInS(), refuelAtMs ? refuelTarget : 0, pumpAfter, attempts, FW_VERSION,
    (unsigned long)logTotal, (unsigned long)(millis() / 1000UL),
    (unsigned long)cfgMaxRunMin, (unsigned long)cfgWarmupS, (unsigned long)cfgCooldownS,
    (unsigned long)cfgPumpMaxS, (unsigned long)cfgPumpStepS, (unsigned long)cfgRefuelDlyMin);
  server.send(200, "application/json", b);
}

void webLog() {
  String o = "[";
  for (int i = 0; i < logCount; i++) {
    const LogEv& e = logBuf[(logHead - logCount + i + LOG_N) % LOG_N];
    if (i) o += ",";
    o += "{\"ago\":" + String((unsigned long)((millis() - e.ms) / 1000UL)) + ",\"t\":\"" + jsonEsc(e.txt) + "\"}";
  }
  o += "]";
  server.send(200, "application/json", o);
}

uint32_t pinLockLeftS() {
  if (!pinLockUntil) return 0;
  int32_t r = (int32_t)(pinLockUntil - millis());
  if (r <= 0) { pinLockUntil = 0; return 0; }
  return ((uint32_t)r + 999) / 1000;
}

bool pinOk(const String& p) {
  if (pinLockLeftS()) return false;
  if (p == WEB_PIN) { pinFails = 0; pinLockUntil = 0; return true; }
  if (++pinFails >= 5) { pinLockUntil = millis() + 5UL * 60000UL; pinFails = 0; logEv("Сторінка: 5 невірних PIN, блок на 5 хв"); }
  return false;
}

void webCmd() {
  if (!pinOk(server.arg("pin"))) {
    uint32_t left = pinLockLeftS();
    char m[96];
    if (left) snprintf(m, sizeof(m), "PIN заблоковано ще %lu хв (5 невірних спроб)", (unsigned long)((left + 59) / 60));
    else snprintf(m, sizeof(m), "Невірний PIN (лишилось спроб: %lu)", (unsigned long)(5 - pinFails));
    server.send(403, "text/plain; charset=utf-8", m);
    return;
  }
  String c = server.arg("cmd");
  c.trim();
  if (c.length() == 0 || c.length() > 60) { server.send(400, "text/plain; charset=utf-8", "Порожня команда"); return; }
  String reply;
  capture = &reply;
  handleService(c);
  capture = nullptr;
  if (reply.length() == 0) reply = "Виконано";
  server.send(200, "text/plain; charset=utf-8", reply);
}

void webUpdateUpload() {
  HTTPUpload& up = server.upload();
  esp_task_wdt_reset();
  if (up.status == UPLOAD_FILE_START) {
    otaOk = false;
    otaReject = !pinOk(server.arg("pin")) || !(state == S_IDLE || state == S_FAULT) || ac;
    if (otaReject) return;
    logEv("Оновлення прошивки: %s", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) otaReject = true;
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (!otaReject && Update.write(up.buf, up.currentSize) != up.currentSize) otaReject = true;
  } else if (up.status == UPLOAD_FILE_END) {
    if (!otaReject) otaOk = Update.end(true);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); otaReject = true;
  }
}

void webUpdateDone() {
  if (otaOk) {
    server.send(200, "text/plain; charset=utf-8", "Прошивку оновлено. Перезавантаження…");
    logEv("Прошивку оновлено, перезавантаження");
    prefs.putBool("wifiboot", true);         // після оновлення Wi-Fi вмикається сам, щоб перевірити версію
    delay(800);
    ESP.restart();
  } else {
    server.send(400, "text/plain; charset=utf-8",
                "Не оновлено: невірний PIN, генератор не стоїть або файл пошкоджений");
  }
}

void wifiStart(const char* why) {
  if (wifiOn) { wifiLastClientMs = millis(); say("Wi-Fi уже увімкнено: %s, 192.168.4.1", AP_SSID); return; }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.begin();
  wifiOn = true; wifiLastClientMs = millis();
  logEv("Wi-Fi увімкнено (%s): %s, 192.168.4.1", why, AP_SSID);
  sendStatus();
}

void wifiStop(const char* why) {
  if (!wifiOn) return;
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiOn = false;
  logEv("Wi-Fi вимкнено (%s)", why);
  sendStatus();
}

void wifiTask() {
  // кнопка BOOT: утримати 2 с — увімкнути Wi-Fi
  static uint32_t pressedAt = 0;
  if (digitalRead(PIN_BOOTBTN) == LOW) {
    if (!pressedAt) pressedAt = millis();
    else if (pressedAt != 1 && millis() - pressedAt >= BOOTBTN_HOLD_MS) { wifiStart("кнопка BOOT"); pressedAt = 1; }
  } else pressedAt = 0;

  if (!wifiOn) return;
  server.handleClient();
  if (WiFi.softAPgetStationNum() > 0 || ac) wifiLastClientMs = millis();   // під час роботи двигуна не вимикаємо
  if (millis() - wifiLastClientMs > WIFI_IDLE_OFF_MS) wifiStop("10 хв без підключень");
}

// ===================================================================
//                              ВИМІРИ
// ===================================================================
int fuelStepFromMv(int mv) {
  if (mv > FUEL_OPEN_MV) return 0;
  int s = 1;
  for (int i = 0; i < 4; i++) if (mv > FUEL_TH[i]) s = i + 2;
  return s;
}

void updateAC() {
  bool r = (digitalRead(PIN_DET) == LOW);
  if (r != acRaw) { acRaw = r; acChangeT = millis(); }
  if (ac != acRaw && millis() - acChangeT >= AC_DEBOUNCE_MS) {
    ac = acRaw;
    if (ac) engineRan = true;
    logEv("230 В: %s", ac ? "є" : "немає");
  }
}

bool tempValid(float t) { return t != DEVICE_DISCONNECTED_C && t != 85.0f && t > -55.0f && t < 125.0f; }

// один вимір палива: підтяжку вмикаємо лише на час виміру
void fuelSample() {
  digitalWrite(PIN_FUELPWR, HIGH);
  delay(10);
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(PIN_FUEL);
  digitalWrite(PIN_FUELPWR, LOW);
  fuelMv = sum / 16;
  fuelLastMs = millis();
  fuelSeq++;

  // фільтр сходинки: зміна приймається після 3 однакових вимірів
  int s = fuelStepFromMv(fuelMv);
  if (s == fuelStep) { fuelCand = -1; fuelCandN = 0; }
  else if (s == fuelCand) { if (++fuelCandN >= 3) { fuelStep = s; fuelCand = -1; fuelCandN = 0; } }
  else { fuelCand = s; fuelCandN = 1; }
}

void measure() {
  uint32_t now = millis();
  bool quiet = quietIdle();

  // напруга АКБ (дільник споживає завжди, вимір нічого не коштує)
  static uint32_t tBat = 0;
  if (tBat == 0 || now - tBat >= 1000) {
    tBat = now;
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(PIN_BAT);
    vbat = (sum / 16.0f) * BAT_K / 1000.0f;
  }

  // паливо: часто, коли качає насос; рідко в простої (рівень може змінитись лише від доливання вручну)
  uint32_t fInt = pumpOn ? FUEL_PUMP_MS : (quiet ? FUEL_IDLE_MS : FUEL_ACTIVE_MS);
  if (fuelLastMs == 0 || now - fuelLastMs >= fInt) fuelSample();

  // температура (усі датчики разом, без блокування)
  static uint32_t tReq = 0; static bool pending = false;
  uint32_t tInt = quiet ? TEMP_IDLE_MS : TEMP_ACTIVE_MS;
  if (!pending && (tReq == 0 || now - tReq >= tInt)) { sensors.requestTemperatures(); tReq = now; pending = true; }
  if (pending && now - tReq > TEMP_CONV_MS) {
    pending = false;
    float t = sensors.getTempC(TEMP_BAT_ADDR);
    tempOk = tempValid(t); if (tempOk) tempC = t;
    if (engPresent) { float e = sensors.getTempC(engAddr); engOk = tempValid(e); if (engOk) engT = e; }
  }
}

// ===================================================================
//                              SETUP / LOOP
// ===================================================================
void setup() {
  setCpuFrequencyMhz(CPU_MHZ);               // до Serial1.begin: UART налаштовується вже на новій частоті
  int outs[] = { PIN_STOP, PIN_START, PIN_SSR, PIN_PUMP, PIN_SRVPWR, PIN_FUELPWR };
  for (int p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(PIN_DET, INPUT_PULLUP);
  pinMode(PIN_BOOTBTN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);                  // без підключеного USB вивід не гальмує loop
  Serial1.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.printf("Контролер генератора, прошивка %s, CPU %lu МГц\n", FW_VERSION, (unsigned long)getCpuFrequencyMhz());

  WiFi.mode(WIFI_OFF);
  analogSetPinAttenuation(PIN_BAT, ADC_11db);
  analogSetPinAttenuation(PIN_FUEL, ADC_11db);

  ledcAttach(PIN_SERVO, 50, 14);
  ledcWrite(PIN_SERVO, 0);

  // датчики температури: АКБ за відомою адресою, будь-який інший — двигун
  sensors.begin();
  int cnt = sensors.getDeviceCount();
  DeviceAddress a;
  for (int i = 0; i < cnt; i++) {
    if (sensors.getAddress(a, i) && memcmp(a, TEMP_BAT_ADDR, 8) != 0) { memcpy(engAddr, a, 8); engPresent = true; }
  }
  sensors.setResolution(10);                 // 0,25 °C — досить для порогів 40 і 60 °C, конверсія вчетверо коротша
  sensors.setWaitForConversion(false);
  Serial.printf("Датчиків температури: %d. Датчик двигуна: %s\n", cnt, engPresent ? "є" : "немає (гарячий старт за таймером 20 хв)");
  if (engPresent) {
    Serial.print("  адреса датчика двигуна: ");
    for (int i = 0; i < 8; i++) Serial.printf("%02X", engAddr[i]);
    Serial.println();
  }

  prefs.begin("gen", false);
  cfgMaxRunMin    = prefs.getUInt("maxrun", cfgMaxRunMin);
  cfgWarmupS      = prefs.getUInt("warmup", cfgWarmupS);
  cfgCooldownS    = prefs.getUInt("cooldown", cfgCooldownS);
  cfgPumpMaxS     = prefs.getUInt("pumpmax2", cfgPumpMaxS);
  cfgPumpStepS    = prefs.getUInt("pumpstep", cfgPumpStepS);
  cfgRefuelDlyMin = prefs.getUInt("refueldly", cfgRefuelDlyMin);

  esp_task_wdt_config_t wdtCfg = { .timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_init(&wdtCfg) != ESP_OK) esp_task_wdt_reconfigure(&wdtCfg);
  esp_task_wdt_add(NULL);

  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", PAGE_HTML); });
  server.on("/api/status", HTTP_GET, webStatus);
  server.on("/api/log", HTTP_GET, webLog);
  server.on("/api/cmd", HTTP_POST, webCmd);
  server.on("/update", HTTP_POST, webUpdateDone, webUpdateUpload);

  for (int i = 0; i < 3; i++) { delay(AC_DEBOUNCE_MS / 2); updateAC(); }
  acRaw = ac = (digitalRead(PIN_DET) == LOW);
  measure();
  fuelStep = fuelStepFromMv(fuelMv);
  delay(TEMP_CONV_MS + 50); measure();       // дочитати температуру

  servoMove(CHOKE_OPEN_US);                  // при кожному старті чок відкритий

  // незавершена дозаправка до перезавантаження — продовжити
  int rf = prefs.getInt("rf_t", 0);
  if (rf >= 2 && rf <= 5) {
    uint32_t dly = cfgRefuelDlyMin * 60000UL;
    if (dly < BOOT_REFUEL_MIN_MS) dly = BOOT_REFUEL_MIN_MS;
    refuelSchedule(rf, dly, "після перезавантаження");
  }

  logEv("Старт плати, прошивка %s", FW_VERSION);
  uint8_t sf = prefs.getUChar("fault", 0);
  Fault savedFault = (sf > F_NONE && sf <= F_OVERTEMP) ? (Fault)sf : F_NONE;
  if (ac) {                                  // двигун працював до перезавантаження — реле вже заглушило його
    stopReason = "BOOT";
    runSrc = "-";
    runStartMs = 0;
    pendingFault = savedFault;               // після зупинки повернутися в аварію, якщо вона була
    enterState(S_STOPPING);
  } else if (savedFault != F_NONE) {
    allOutputsSafe();
    fault = savedFault;
    logEv("Аварія %s збережена з минулого разу: для запуску потрібен reset", FAULT_NAMES[fault]);
    enterState(S_FAULT);
  } else {
    enterState(S_IDLE);
  }
  if (prefs.getBool("wifiboot", false)) { prefs.remove("wifiboot"); wifiStart("після оновлення прошивки"); }
  sendCfg();
  printHelp();
  printMon();
}

void loop() {
  esp_task_wdt_reset();
  pollPort(Serial1, rxSite, false);
  pollPort(Serial, rxUsb, true);
  updateAC();
  measure();
  pumpTask();
  runStateMachine();
  wifiTask();

  // запланована дозаправка
  if (refuelAtMs && (int32_t)(millis() - refuelAtMs) >= 0 && state == S_IDLE && !pumpTarget) {
    int target = refuelTarget;
    refuelAtMs = 0; refuelTarget = 0; refuelSave();
    fuelSample();
    int raw = fuelStepFromMv(fuelMv);
    if (raw > 0 && raw < target) pumpStart(target, "POSTRUN");
    else logEv("Дозаправка не потрібна: бак %d/5", raw);
  }

  // повідомити SITE про нові попередження і роботу насоса
  static uint8_t lastWarn = 0xFF; static int lastPump = -1;
  uint8_t wb = warnBits();
  if (wb != lastWarn || pumpTarget != lastPump) { lastWarn = wb; lastPump = pumpTarget; sendStatus(); }

  static uint32_t tSt = 0;
  uint32_t period = (state == S_IDLE || state == S_FAULT) ? 60000 : 10000;
  if (millis() - tSt > period) { tSt = millis(); sendStatus(); }

  static uint32_t tMon = 0;
  if (monOn && millis() - tMon > 5000) { tMon = millis(); printMon(); }

  // у простої процесор спить між проходами (UART має буфер, кнопка й Wi-Fi не відчують 10 мс)
  if (quietIdle()) delay(LOOP_IDLE_DELAY_MS);
  else delay(1);
}
