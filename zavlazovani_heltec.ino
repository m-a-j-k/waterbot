/*
  ============================================================================
  ZAVLAZOVACI SYSTEM - Heltec Wireless Stick Lite V3 (ESP32-S3 + SX1262)
  ============================================================================
  Prepsano z puvodni Arduino Mega verze na ESP32-S3.

  Zmeny oproti puvodni verzi:
  - Display: LovyanGFX (ILI9486, 8bit paralelni), touch: TouchScreen (4 vlastni piny)
  - Radiovy prijem: nRF24L01 nahrazen za EBYTE E22-900T30D (LoRa, UART+M0+M1+AUX)
  - Pridan mereni proudu/napeti INA226 (I2C)
  - WiFi: nativni ESP32 WiFi (misto ESP8266 shield), Blynk pres BlynkSimpleEsp32
  - Zadne blokujici "while(...)" cekani na WiFi/Blynk - system nabootuje
    i zcela offline, prubeh bootovani se vypisuje na display
  - RTC DS3231 nyni pres I2C (RTClib), ne bit-banged knihovna
  - EEPROM na ESP32 vyzaduje EEPROM.begin()/commit()

  Solarni regulator EPSOLAR VS1024AU nema RS485/Modbus vystup - puvodni
  cteni pres Modbus bylo proto z kodu kompletne odstraneno. Napeti a proud
  na vystupu regulatoru se meri primo modulem INA226 pres I2C.

  DULEZITA ZMENA: Heltec Wireless Stick Lite V3 ma SX1262 integrovany PRIMO
  NA DESCE (ne externi EBYTE modul). Pouzivame knihovnu "Heltec ESP32 LoRa v3"
  (github.com/ropg/heltec_esp32_lora_v3, v Library Manageru hledej "heltec_esp32"),
  ktera uz ma spravne definovane piny primo overene v Heltec oficialnim
  repozitari a navic zabaluje RadioLib. Pouzitelne piny:
    SS=8, SCK=9, MOSI=10, MISO=11, RST_LoRa=12, BUSY_LoRa=13, DIO1=14
  GPIO12 a GPIO14 jsou tedy INTERNE propojene s radiem (RESET / DIO1) -
  puvodne jsme na nich meli LCD_D7 a LCD_RS/touch, coz by kolidovalo.
  Proto: LCD_D7 presunuto na GPIO48, LCD_RS presunuto na GPIO20.

  POZOR: knihovna definuje i LED_PIN=GPIO35, VEXT=GPIO36, VBAT_CTRL=GPIO37,
  VBAT_ADC=GPIO1 - to jsou presne nase piny pro backlight/touch/cerpadlo!
  Knihovna na ne ale SAMA OD SEBE nesahne, dokud nezavolame jeji funkce
  heltec_led()/heltec_ve()/heltec_vbat()/heltec_deep_sleep() - proto je
  V TOMTO KODU ZAMERNE NEPOUZIVAME. Nevolej je, pokud pridavas dalsi kod!
  ============================================================================
*/

// ------------------------------------------------------------------------
// KNIHOVNY (Library Manager - presne tyto nazvy hledej)
// ------------------------------------------------------------------------
#include <LovyanGFX.hpp>            // FreeSans9pt7b je uz vestaveny v lgfx::v1::fonts:: namespace
#include "esp_task_wdt.h"           // Task Watchdog Timer - automaticky restartuje pri zaseknuti firmware
#include <TouchScreen.h>
#include <Wire.h>
#include <RTClib.h>                 // "RTClib" by Adafruit
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>        // "Adafruit BME280 Library"
#include <INA226.h>                 // "INA226" by Rob Tillaart
// POZOR: RadioLib i Arduino_Modulino nezavisle na sobe definuji globalni
// tridu "Module" se stejnym jmenem - bez tohohle obaleni by se preklad
// nepovedl (redefinice tridy). Trik: docasne prejmenovat "Module" na neco
// jineho jen pro tenhle jeden include, pak vratit zpet - RadioLib (vcetne
// pres heltec_unofficial.h) uz pak dostane "svoje" puvodni jmeno zpet.
#define Module ModulinoInternalModule
#include <Arduino_Modulino.h>       // Modulino Latch Relay (baterie + solar)
#undef Module
#include <EEPROM.h>
#include <WiFi.h>
#include <Preferences.h>            // cteni WiFi/Blynk udaju z NVS (viz nvs_provision.ino)
#include <WiFiClientSecure.h>       // pro HTTPS pripojeni pri OTA
#include <HTTPUpdate.h>             // OTA aktualizace pres HTTP(S) - soucast ESP32 Arduino core
#include "esp_ota_ops.h"            // esp_ota_mark_app_valid_cancel_rollback() - potvrzeni funkcni firmware
#include "driver/gpio.h"            // gpio_hold_en/dis() - udrzeni stavu pinu behem deep sleep
#include "esp_system.h"             // esp_reset_reason() - zjisteni duvodu posledniho resetu
#define BLYNK_TEMPLATE_ID   "TMPL4waBZn_ut"
#define BLYNK_TEMPLATE_NAME "Waterbot40"
//#define BLYNK_AUTH_TOKEN    "xxx"
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>       // soucast knihovny "Blynk"
#define HELTEC_WIRELESS_STICK_LITE  // musi byt PRED includem - nase deska nema OLED
#include <heltec_unofficial.h>      // "Heltec ESP32 LoRa v3" - hledej "heltec_esp32" v Library Manageru
                                    // (obsahuje uz spravne nakonfigurovany radio objekt + RadioLib)

// ==========================================================================
// PINOUT - Heltec Wireless Stick Lite V3
// ==========================================================================
// --- Displej (LovyanGFX, 8bit paralelni ILI9486) ---
#define PIN_LCD_RST 4
#define PIN_LCD_CS  39
#define PIN_LCD_RS  20   // POZOR: presunuto z GPIO14 - ten je interne propojen s RADIO_DIO_1
#define PIN_LCD_WR  21
#define PIN_LCD_RD  26
#define PIN_LCD_D0  33
#define PIN_LCD_D1  34
#define PIN_LCD_D2  38
#define PIN_LCD_D3  41
#define PIN_LCD_D4  42
#define PIN_LCD_D5  3
#define PIN_LCD_D6  5
#define PIN_LCD_D7  48   // POZOR: presunuto z GPIO12 - ten je internich propojen s RADIO_RESET

// --- Touch (4 samostatne, nesdilene piny) ---
#define PIN_TOUCH_XP 47
#define PIN_TOUCH_XM 2
#define PIN_TOUCH_YP 1
#define PIN_TOUCH_YM 35   // POZOR: presunuto z GPIO36 - ten je Vext_Ctrl (viz nize), uvolneno
                          // pro napajeni 3.3V displeje. GPIO35 sdili sit s onboard LED
                          // (aktivne-HIGH) - pri cteni doteku muze jemne blikat, elektricky
                          // to ale nevadi (na rozdil od P-MOSFETu podsviceni, ktery tu drive byl).

// --- Backlight (P-MOSFET, aktivni LOW) ---
#define PIN_BACKLIGHT 19  // POZOR: presunuto z GPIO35 - ten sdili sit s onboard LED, ktera
                          // je (na rozdil od P-MOSFETu podsviceni) zapojena aktivne-HIGH.
                          // Kdyz jsme GPIO35 nastavovali HIGH = podsviceni vypnuto, LED se
                          // tim same nechtene rozsvitila. GPIO19 je bezpecny, protoze mame
                          // vypnute USB CDC On Boot (viz LCD_RS na sesterskem GPIO20).

// --- Vext (interni P-MOSFET desky, 3.3V vystup az 500mA) - napaji ultrazvukovy
// senzor hladiny a waterflow senzor (displej je nyni HW napajeny natvrdo z 3V3) ---
#define PIN_VEXT 36       // aktivni LOW (jako vetsina P-MOSFET rizeni na Heltec deskach)

// --- I2C (DS3231 + BME280 + INA226) ---
#define PIN_I2C_SDA 18
#define PIN_I2C_SCL 17

// --- Prutokomer YF-S201 ---
#define PIN_WATER_FLOW 6

// --- Ultrazvuk JSN-SR04T-V3.0 (vodotesna sonda pro hladinu v sudu) ---
#define PIN_TRIG 7
#define PIN_ECHO 40

// --- Cerpadlo (MOSFET) ---
#define PIN_PUMP 37

// --- Onboard SX1262 - piny definuje knihovna heltec_unofficial.h sama
//     (SS=8, SCK=9, MOSI=10, MISO=11, RST_LoRa=12, BUSY_LoRa=13, DIO1=14)

// ==========================================================================
// DISPLEJ - LovyanGFX konfigurace
// ==========================================================================
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9486 _panel_instance;
  lgfx::Bus_Parallel8 _bus_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.pin_wr = PIN_LCD_WR;
      cfg.pin_rd = PIN_LCD_RD;
      cfg.pin_rs = PIN_LCD_RS;
      cfg.pin_d0 = PIN_LCD_D0;
      cfg.pin_d1 = PIN_LCD_D1;
      cfg.pin_d2 = PIN_LCD_D2;
      cfg.pin_d3 = PIN_LCD_D3;
      cfg.pin_d4 = PIN_LCD_D4;
      cfg.pin_d5 = PIN_LCD_D5;
      cfg.pin_d6 = PIN_LCD_D6;
      cfg.pin_d7 = PIN_LCD_D7;
      cfg.freq_write = 20000000;
      cfg.freq_read  = 800000;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs  = PIN_LCD_CS;
      cfg.pin_rst = PIN_LCD_RST;
      cfg.pin_busy = -1;
      cfg.panel_width  = 320;
      cfg.panel_height = 480;
      cfg.readable = false;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.bus_shared = false;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX lcd;

#define BLACK       0x0000
#define RED         0xF800
#define GREEN       0x07E0
#define WHITE       0xFFFF
#define GREY        0x8410
#define DARKGREY2   0x632C
#define BLUE        0x001F
#define DARKDARKGREY 0x3186

// Barvy pro ikonu baterie a LoRa zarizeni (prevedeno z hex referencniho obrazku do RGB565)
#define BATTERYGRAY      0xB5B6  // #B5B5B5 telo baterie
#define BATTERYDARK      0x4A49  // #4A4A4A koncovky, minus znamenko
#define BATTERYRED       0xD1A5  // #D8362A plus znamenko
#define ARROWGREEN       0x3DE7  // #3FBF3F zaklad sipky
#define ARROWGREEN_LIGHT 0xA733  // #A6E6A0 pohybujici se useky
#define DEVICEBLUE       0xDF3E  // #DCE7F7 telo LoRa zarizeni
#define ANTENNAGRAY      0x9CB3  // #9C9C9C anténa

// ==========================================================================
// TOUCH
// ==========================================================================
#define MINPRESSURE 50
#define MAXPRESSURE 1000

// Kalibrace dotykove vrstvy (namereno pomoci touch_kalibrace.ino) - prevadi
// surove ADC hodnoty na skutecne pixelove souradnice. Oba smery jsou obracene
// (raw hodnota klesa s rostouci pixelovou souradnici), map() si s tim poradi sam.
// Kalibrace dotykove vrstvy (namereno pomoci touch_kalibrace.ino).
// KONVENCE: TOUCH_CAL_X1/X2 se pouzivaji vzdy pro vypocet pixelX,
// TOUCH_CAL_Y1/Y2 vzdy pro pixelY - bez ohledu na to, ktery surovy kanal
// (p.x/p.y) se uvnitr vzorce skutecne cte (viz handleTouch()).
#define TOUCH_CAL_X1 923
#define TOUCH_CAL_X2 148
#define TOUCH_CAL_Y1 853
#define TOUCH_CAL_Y2 233
TouchScreen ts = TouchScreen(PIN_TOUCH_XP, PIN_TOUCH_YP, PIN_TOUCH_XM, PIN_TOUCH_YM, 300);

unsigned long backLightON = 0;
bool backlightOn = true; // sleduje aktualni stav podsviceni, aby slo preskocit zbytecne prekreslovani
bool displayLogicOn = true; // POZOR: displej je nyni napajeny natvrdo z 3V3, tedy
                             // vzdy true - ponechano jen jako pojistka/pro pripadne
                             // budouci pouziti, uz nesleduje stav Vext
unsigned long backLightTime = 15000;
int doubletouch = 150; // znovu 300ms neumoznovalo rychlejsi opakovane klikani nez ~3.3x/s
                        // (napr. na tlacitkach +/- behem zalevani) - 150ms umoznuje az ~6.6x/s
unsigned long lastTouchCheck = 0;
const unsigned long touchInterval = 100;

// ==========================================================================
// RTC (DS3231 pres I2C)
// ==========================================================================
RTC_DS3231 rtc;
String lastDOW = "";

// ---------- OTA aktualizace pres HTTP(S) ----------
String otaFirmwareUrl = ""; // nastavuje se pres Blynk V98 (text input s URL na .bin soubor)
String actualDOW = "";
const char* dowNames[7] = {"Pondeli","Utery","Streda","Ctvrtek","Patek","Sobota","Nedele"};

void bootLog(const char* text, uint16_t color); // definice az niz v souboru

// ==========================================================================
// NTP - stazeni presneho casu z internetu a nastaveni RTC (jen pri startu,
// pokud je WiFi pripojene; jinak RTC jede dal ze sve baterie autonomne)
// ==========================================================================
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const long  gmtOffsetSec = 3600;      // CET = UTC+1
const int   daylightOffsetSec = 3600; // letni cas +1h navic (CEST), ESP32 si prepnuti spocita samo

void syncRtcFromNtp() {
  bootLog("Stahuji cas z NTP...", WHITE);
  configTime(gmtOffsetSec, daylightOffsetSec, ntpServer1, ntpServer2);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5000)) { // max 5s cekani na odpoved NTP serveru
    rtc.adjust(DateTime(
      timeinfo.tm_year + 1900,
      timeinfo.tm_mon + 1,
      timeinfo.tm_mday,
      timeinfo.tm_hour,
      timeinfo.tm_min,
      timeinfo.tm_sec
    ));
    char buf[32];
    sprintf(buf, "RTC synchr.: %02d.%02d.%04d %02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon + 1,
            timeinfo.tm_year + 1900, timeinfo.tm_hour, timeinfo.tm_min);
    bootLog(buf, GREEN);
  } else {
    bootLog("NTP se nepodarilo stahnout, RTC beri puvodni cas", RED);
  }
}

// ==========================================================================
// BME280
// ==========================================================================
Adafruit_BME280 bme;
#define BME280_ADRESA 0x76
#define NADMORSKA_VYSKA_M 355.0 // pro prepocet absolutniho tlaku na relativni (V85)
float T_out = 0, T_out_mem = 0;
float T_dev = 0, T_dev_mem = 0; // pro hysterezi teploty RTC cipu, stejne jako u T_out/vlhkost
float vlhkost = 0, vlhkost_mem = 0;

// ==========================================================================
// INA226 - mereni proudu a napeti (napr. na baterii/solar vetvi)
// ==========================================================================
INA226 ina226(0x40);

// ==========================================================================
// RELE - Modulino Latch Relay (spinani baterie/solar regulatoru)
// ==========================================================================
#define ADRESA_RELE_BATERIE 0x10
#define ADRESA_RELE_SOLAR   0x11
ModulinoLatchRelay releBaterie(ADRESA_RELE_BATERIE);
ModulinoLatchRelay releSolar(ADRESA_RELE_SOLAR);

#define RELE_ODSTUP_MS         2000UL  // rozestup mezi bat a solar v sekvenci
#define RELE_VEXT_HOLD_MS      5000UL  // jak dlouho drzet Vext zapnuty PO dokonceni
                                        // prepnuti - at jsou videt kontrolni LED na modulu
#define RELE_MERENI_ODSTUP_MS  10000UL // od sepnuti solaru do rozhodovaciho mereni napeti
#define RELE_NAPETI_PRAH       13.6    // pod touto hodnotou se rele odpoji
#define NOC_HODINA_OD          21      // 21:00 - 6:00 = rele vzdy odpojena
#define NOC_HODINA_DO          6

bool releBaterieSepnuto = false; // aktualizovano skutecnym stavem pres getStatus() v setup()
bool releSolarSepnuto = false;
bool releWakeRozhodnutoVTomtoWake = false; // v low-power modu - uz probehla pripojovaci+rozhodovaci sekvence behem TOHOTO probuzeni?
bool releManualniSekvence = false; // true = aktualne probihajici sekvence byla vyvolana rucne z Blynk (V87) -
                                     // preskoci rozhodovaci mereni napeti (i v low-power modu) a po dokonceni
                                     // posle echo na V88. Automaticke sekvence (noc, normalni rezim, low-power
                                     // probuzeni) tohle echo neposilaji.
bool releAutomatikaAktivni = true; // false = automatika (rizeniRele()) je docasne vypnuta rucnim zasahem
                                     // (ikonka zarovky na displeji, nebo V87) - NEPREZIVA restart/probuzeni,
                                     // po kazdem cerstvem startu se automatika vzdy znovu aktivuje (V89/V90).

// Sledovani stavu notifikace baterie u kazdeho cidla - 0=OK, 1=nizka
// (oranzova) jiz nahlasena, 2=kriticka (cervena) jiz nahlasena. Zabranuje
// opakovanemu spamovani stejnou notifikaci pri kazdem prijmu LoRa paketu.
int stavBaterieCidlo2 = 0, stavBaterieCidlo3 = 0, stavBaterieCidlo4 = 0, stavBaterieCidlo5 = 0;

// Notifikace "solar_mode" (automatika rele vypnuta) - ihned pri vypnuti,
// pak opakovane kazdych 4 hodiny, dokud zustava vypnuta.
unsigned long solarModeNotifikaceMillis = 0;
#define SOLAR_MODE_NOTIFIKACE_INTERVAL_MS (4UL * 3600UL * 1000UL) // 4 hodiny

// Neblokujici "cuknuti" cerpadlem (viz waterLevelCheck() - mitigace pri
// neocekavanem poklesu hladiny) - misto delay(1000) se stav sleduje
// prubezne (viz kontrolaCuknutiCerpadla() a jeji zaznam v timers[]).
bool cuknutiCerpadlaAktivni = false;
unsigned long cuknutiCerpadlaStartMillis = 0;
#define CUKNUTI_CERPADLA_MS 1000UL

enum StavReleSekvence {
  RELE_SEQ_KLID,
  RELE_SEQ_ZAPNI_BAT,
  RELE_SEQ_CEKAM_SOLAR_ZAP,
  RELE_SEQ_CEKAM_MERENI,
  RELE_SEQ_VYPNI_SOLAR,
  RELE_SEQ_CEKAM_BAT_VYP,
  RELE_SEQ_CEKAM_VEXT_OFF // drzi Vext zapnuty jeste par vterin po dokonceni
                           // prepnuti, at jsou videt kontrolni LED na modulu
};
StavReleSekvence releSekvenceStav = RELE_SEQ_KLID;
unsigned long releSekvenceCasKroku = 0;
float ina_busVoltage = 0, ina_current_mA = 0, ina_power_mW = 0;

// Sledovani denni spotreby (integrace vykonu v case)
RTC_DATA_ATTR float dailyEnergyWh = 0;   // akumulovana energie od pulnoci (Wh), prezije spanek
RTC_DATA_ATTR int32_t dailyEnergyDay = -1; // "den" identifikator pro detekci pulnoci
RTC_DATA_ATTR float emaCurrentA_1h = 0;   // klouzavy prumer proudu (A), casova konstanta ~1 hodina
RTC_DATA_ATTR float emaCurrentA_24h = 0;  // klouzavy prumer proudu (A), casova konstanta ~24 hodin
RTC_DATA_ATTR bool lastKnownLowPowerMode = false; // pro detekci prepnuti rezimu (vynulovani prumeru)
unsigned long lastINA226Millis = 0;      // pro presny vypocet uplynuleho casu mezi mereninimi
unsigned long lastDailyChargeEepromSave = 0; // throttling zapisu do EEPROM

// ==========================================================================
// Onboard SX1262 - objekt "radio" uz existuje, poskytuje ho heltec_unofficial.h
// ==========================================================================
volatile bool loraGotPacket = false;

void IRAM_ATTR onLoraDio1() {
  loraGotPacket = true;
}

// Struktura dat prijimanych ze vzdalenych LoRa cidel - MUSI presne sedet
// s SensorData na strane cidla (stejne typy, stejne poradi poli)!
struct RoomData {
  uint8_t  roomId;     // 2=kuchyn, 3=dolni WC, 4=horni WC, 5=studna
  float    temperature;
  float    humidity;
  uint16_t batteryMv;
  uint32_t unixtime;
  int8_t   currentTxPowerDbm; // aktualne POUZITY vysilaci vykon cidla (pro potvrzeni, ne prani)
  uint32_t msSinceSync; // kolik ms uplynulo od posledni synchronizace, podle VLASTNICH hodin cidla (test driftu oscilatoru)
  uint16_t potvrzenaMrizkaSec; // jakou mrizku (v s) si cidlo MYSLI, ze ma aktualne platit
                               // (podle posledni prijate odpovedi) - centrala tim
                               // pozna, ktera cidla uz vedi o nove mrizce po zmene V96
};

// Odpoved centraly cidlu - cas pro synchronizaci + pripadna zmena vykonu, a hlavne
// PRESNY POCET VTERIN, po ktery ma cidlo spat, aby se probudilo presne v okamziku
// dalsiho probuzeni centraly. Musi presne sedet s CentralReply na strane cidla.
struct CentralReply {
  uint8_t  targetRoomId;
  uint32_t currentUnixTime; // aktualni cas centraly (informativni/diagnosticke)
  uint8_t  newTxPower;      // pozadovany vykon v dBm s offsetem +9 (0=-9dBm..31=+22dBm), 0xFF = beze zmeny
  uint16_t sleepUntilNextWakeSec; // PRESNY pocet vterin do PRISTIHO probuzeni centraly,
                                   // pocitano ZA BEHU (ne pri usinani) - cidlo tuhle
                                   // hodnotu proste primo pouzije jako dobu sveho spanku,
                                   // cimz se automaticky srovna na centralinu mrizku,
                                   // nezavisle na tom, kdy presne k tomuto kontaktu doslo.
  uint16_t currentGridSec;  // aktualne CILOVA mrizka centraly (lowPowerWakeGridSec) -
                             // posila se VZDY, cidlo si ji jen pamatuje pro pripadne
                             // potvrzovaci echo (potvrzenaMrizkaSec) pri svem pristim vysilani.
};

RTC_DATA_ATTR uint16_t Baterie2 = 0, Baterie3 = 0, Baterie4 = 0, Baterie5 = 0; // napeti cidel v mV

RTC_DATA_ATTR float Teplota2 = NAN, Teplota3 = NAN, Teplota4 = NAN, Teplota5 = NAN;
RTC_DATA_ATTR float Vlhkost2 = NAN, Vlhkost3 = NAN, Vlhkost4 = NAN, Vlhkost5 = NAN;
RTC_DATA_ATTR unsigned long CH2 = 0, CH3 = 0, CH4 = 0, CH5 = 0; // cas posledniho prijmu (unix time), prezije spanek
// signalCheckTime (21 min) odstranen - byl to pozustatek z doby, kdy
// Kuchyn byvala site napajene cidlo s castym vysilanim. Ted jsou vsechna
// ctyri cidla identicka (LoRa, baterie, stejny interval), takze pouzivaji
// jednotne signalCheckTimeBatt.
int signalCheckTimeBatt = 86700;   // 24h 5min - bateriove cidlo
boolean pip2_podtrzeno = 1, pip3_podtrzeno = 1, pip4_podtrzeno = 1, pip5_podtrzeno = 1;

// ==========================================================================
// Prutokomer
// ==========================================================================
volatile int contagem = 0;
int litrAktualne = 0;
int litrUnikuBezCerpadla = 0; // pocitadlo litru, co protekly waterflow senzorem BEZ
                               // bezicicho cerpadla - detekce gravitacniho uniku (viz loop())
#define LITRU_UNIKU_PRAH 2 // od kolika litru se spusti notifikace + cuknuti cerpadlem
int litryLimitDocasne = 0; // docasna kopie litryLimit pro AKTUALNI zalevaci cyklus -
                            // upravitelna +/- na displeji behem zalevani, ale NIKDY
                            // se neuklada do EEPROM ani nemeni trvale nastaveni (litryLimit)
byte litryLimit = 15;

void IRAM_ATTR pin_ISR() {
  contagem++;
}

// ==========================================================================
// Ultrazvuk / hladina vody
// ==========================================================================
float odezva, vzdalenost, vzdalenostDilci, rozdilVzd;
float vzdalenostPredchozi = 0;
float celkovyObjem = 0;
int hladinaAnimace;
boolean vzdalenostRucne = 0;
long pauzaOdectuLimit = 600000;
int pauzaOdectu = 0;
unsigned long pauzaOdectuStartMillis;
float lastWaterLevel = 0;
float lastWaterLevelNotify = 0;
int hladinaPokles = 30;
int hladinaPrirustek = 20;
int hladinaMinimum = 150;
int spotrebaUspornyRezim = 15;
int MaxZasoba = 1000;
boolean rezimy = 0;
boolean hladinaMinimumBoolean = 0;
boolean polovinaBoolean = 0;

// ==========================================================================
// Cerpadlo
// ==========================================================================
long maxPumpOn = 480000;
boolean pumpStatus = 0;
int lastUsedAmount = 0;
unsigned long pumpStartTimeMillis;
int ubyloTyden = 0, naprseloTyden = 0, napusteno = 0, naprseloCelkem = 0, ubyloCelkem = 0;
String lastPumpUsed;
WidgetTerminal eventTerminal(V0); // log DST prechodu, tydenniho resetu apod.

// Pomocna funkce - vsechny zpravy do terminalu (V0) automaticky opatri
// casovym razitkem, kdy byla zprava skutecne odeslana
void logToTerminal(String message) {
  DateTime now = rtc.now();
  char buf[21];
  sprintf(buf, "%02d.%02d.%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
  eventTerminal.println(String(buf) + " " + message);
}
WidgetLED pumpLED(V6); // LED indikator stavu cerpadla v appce
WidgetLED timer1LED(V28);
WidgetLED timer2LED(V29);
WidgetLED timer3LED(V30);

// ==========================================================================
// Casovace zavlazovani
// ==========================================================================
boolean Timer1 = 1, Timer2 = 0, Timer3 = 1;
byte startHour1 = 8,  startMinute1 = 30;
byte startHour2 = 18, startMinute2 = 30;
byte startHour3 = 12, startMinute3 = 30;
long casovac1, casovac2, casovac3;
char tz[] = "Europe/Prague"; // pro echo TimeInput widgetu zpet do Blynku (V10/V11/V12)
boolean pauza1 = 1, pauza2 = 1, pauza3 = 1;

// ==========================================================================
// WiFi / Blynk - NEBLOKUJICI stavovy automat
// ==========================================================================
// Skutecne hodnoty se NACITAJI Z NVS (viz loadCredentialsFromNVS() v setup())
// - nejsou tu natvrdo napsane. Pred prvnim pouzitim spust jednou samostatny
// sketch "nvs_provision.ino", ktery udaje do NVS zapise.
char ssid[33] = "";
char pass[65] = "";
char auth[40] = "";

boolean wifiWasConnected = false;
boolean blynkWasConnected = false;
byte ConnectionTimeOut = 0;
boolean nowOnline = 1;
boolean nouzakButton = 0;
String nouzakStartTime;
boolean Notifikace = 0;

unsigned long lastWifiRetry = 0;
const unsigned long wifiRetryInterval = 30000; // zkousej znovu kazdych 30s pokud offline

// ==========================================================================
// Timery (nahrada BlynkTimer - jednoduchy vlastni scheduler)
// ==========================================================================
struct MyTimer {
  unsigned long interval;
  unsigned long last;
  void (*fn)();
};

void displayTime();
void getTemperature();
void timerCheck();
void resetPoTydnu();
void odhadZasob();
void saveDailyChargeToEEPROM();
void loadDailyChargeFromEEPROM();
String formatDny(int pocet);
float estimateBatterySOC(float voltage);
float estimateLipolPercent(float voltage);
void resyncAllToBlynk();
void checkWifiBlynk();
void loraReceive();
void injectRandomTestSensorValues();
void readINA226();
void updateBlynkBatteryIcon(float voltage);
void animatePowerFlow();
void wakeDisplay();
void sleepDisplay();
void prekresliVsechnyTeploty();
void updateBatteryIcon();
void checkDailyBatteryDecision();
void checkNavratDoUspornhoSpanku();
void waterLevel();
void waterLevelCheck();
void pumpStatusCheck();
void wifiRSSI();

MyTimer timers[] = {
  {1000L,   0, displayTime}, // 1x za sekundu kvuli tikajicim vterinam
  {70003L,  0, getTemperature},
  {55020L,  0, timerCheck},
  {50000L,  0, resetPoTydnu},
  {10000L,  0, checkWifiBlynk},
  {50L,     0, loraReceive}, // kontrola je levna (jen if(!loraGotPacket) return), voláme casto at odpoved cidlu neceka na plánovač
  {5000L,   0, injectRandomTestSensorValues}, // TEST_MODE_RANDOM_SENSOR_VALUES - bezi vzdy, ale uvnitr se hned vrati, pokud je vypnuto
  {45003L,  0, readINA226},
  {200L,    0, animatePowerFlow},
  {30000L,  0, checkDailyBatteryDecision},
  {10000L,  0, checkNavratDoUspornhoSpanku},
  {300001L, 0, waterLevel},
  {2222L,   0, pumpStatusCheck},
  {15007L,  0, wifiRSSI},
  {500L,    0, krokReleSekvence}, // caste volani - postupuje rozbehnutou sekvenci rele
  {200L,    0, kontrolaCuknutiCerpadla}, // caste volani - presne vypnuti po CUKNUTI_CERPADLA_MS
  {60000L,  0, kontrolaSolarModeNotifikace}, // kontrola kazdou minutu - staci na 4h interval opakovani
  {30000L,  0, rizeniRele},       // periodicka kontrola - nocni okno / rezim / potreba spustit sekvenci
};
const int timerCount = sizeof(timers) / sizeof(timers[0]);

void runTimers() {
  unsigned long now = millis();
  for (int i = 0; i < timerCount; i++) {
    if (now - timers[i].last >= timers[i].interval) {
      timers[i].last = now;
      timers[i].fn();
    }
  }
}

// ==========================================================================
// EEPROM helpery (ESP32 vyzaduje begin()/commit())
// ==========================================================================
void EEPROMWriteInt(int address, int value) {
  byte two = (value & 0xFF);
  byte one = ((value >> 8) & 0xFF);
  EEPROM.write(address, two);
  EEPROM.write(address + 1, one);
  EEPROM.commit();
}

int EEPROMReadInt(int address) {
  long two = EEPROM.read(address);
  long one = EEPROM.read(address + 1);
  return ((two << 0) & 0xFFFFFF) + ((one << 8) & 0xFFFFFFFF);
}

// ==========================================================================
// UTLUMOVY (LOW POWER) REZIM - hluboky spanek s planovanym probuzenim
// ==========================================================================
// RTC_DATA_ATTR promenne PREZIJI hluboky spanek (RTC pamet zustava napajena),
// ale pri realnem odpojeni napajeni (vybita baterie i pro RTC domenu) se
// vynuluji na vychozi hodnotu - tedy dalsi start bude uz normalni plny boot.
RTC_DATA_ATTR bool lowPowerMode = false;
RTC_DATA_ATTR uint32_t lowPowerWakeGridSec = 3600; // jak casto se centrala v low power rezimu probouzi (vychozi hodina)

// Prepinac (V97) - dokud je zapnuty, centrala neusne, i kdyz uz bezne bdele
// okno vyprselo (viz checkNavratDoUspornhoSpanku). MUSI byt RTC_DATA_ATTR -
// pokud po probuzeni z hlubokeho spanku (coz je technicky vzdy restart)
// nezustane zachovany, ztratila by se informace, ze ma zustat vzhuru.
RTC_DATA_ATTR bool preventNextSleep = false;

RTC_DATA_ATTR int32_t lastBatteryCheckDay = -1;  // "den" posledni denni kontroly baterie

#define BATTERY_LOW_VOLTAGE     12.4  // pod touto hodnotou (V) prejde system do uspornho rezimu
                                      // (posunuto z 11.5V - riziko sulfatace u trakcni baterie
                                      // OTL26-12 pri delsim setrvani v nizsim SOC, viz datasheet:
                                      // vypinaci napeti vyrobce 2.02V/clanek = 12.12V pri <0.1C)
#define BATTERY_RECOVER_VOLTAGE 12.6  // nad touto hodnotou se system vrati do normalniho rezimu
#define BATTERY_CAPACITY_AH     26.0  // jmenovita kapacita baterie OTL26-12 (viz datasheet)

// Odhadovana spotreba behem samotneho deep sleep (ESP32 + periferie na desce).
// NELZE ZMERIT PRIMO - CPU je behem spanku vypnuty, nemuze se ptat INA226 na nic.
// 0.2 mA je typicky odhad pro ESP32-S3 deep sleep + zbytek desky, ale pro
// presnejsi vysledek doporucuji zmerit skutecnou hodnotu (napr. laboratornim
// zdrojem misto baterie, zarizeni v deep sleep, precist proud primo na zdroji).
#define SLEEP_CURRENT_A 0.0002

// Vlastni spotreba SOLARNIHO REGULATORU SAMOTNEHO - SKUTECNE ZMERENO
// (laboratorni zdroj na baterii regulatoru, panel i vystup do zateze
// odpojeny). Katalogovy udaj 10mA neodpovidal realite - skutecne zmereno
// 22mA. Tenhle proud bere regulator VZDY, nezavisle na stavu ESP32 (spanek
// i bdeni), a NEPROTEKA pres INA226 shunt (ten meri jen ESP32/zatez za
// regulatorem) - proto se musi pricitat SAMOSTATNE ke vsem odhadum vydrze.
#define REGULATOR_QUIESCENT_A 0.022

// EEPROM adresy pro ulozeni klouzavych prumeru - prezije i restart (na rozdil
// od RTC_DATA_ATTR, ktere prezije jen deep sleep, ne skutecny restart).
#define EEPROM_ADDR_EMA_1H       170 // float, 4 bajty (170-173)
#define EEPROM_ADDR_EMA_24H      174 // float, 4 bajty (174-177)
#define EEPROM_ADDR_LOW_POWER    178 // byte, 1 bajt (178)

// EEPROM adresy pro sdileny interval probouzeni (V96) a vysilaci vykony
// jednotlivych cidel (V120-V123) - aby se obnovily i po skutecnem restartu.
#define EEPROM_ADDR_WAKE_GRID_SEC 180 // uint32_t, 4 bajty (180-183)
#define EEPROM_ADDR_TXPOWER_BASE  184 // 4 bajty (184-187), jeden na kazde cidlo (roomId 2-5)
#define EEPROM_SAVE_INTERVAL_MS 3600000UL // ukladat nejvyse jednou za hodinu
#define BATTERY_CHECK_HOUR      1     // denni kontrola baterie v 1:30 - solar nenabiji, klidovy stav
#define BATTERY_CHECK_MINUTE    30

// Per-cidlo pozadovana konfigurace (posila se v odpovedi centraly) - index 2..5 = roomId
// Vysilaci vykon cidel v dBm - SX1262 (Heltec WSL V3 i CubeCell) umi -9 az
// +22 dBm po 1 dBm. V CentralReply se posila s offsetem +9 (0..31), protoze
// newTxPower je uint8_t a 0xFF znamena "beze zmeny". Vychozi 10 dBm
// (= vychozi hodnota RadioLib radio.begin()).
RTC_DATA_ATTR int8_t desiredTxPower[6] = {10,10, 10,10,10,10}; // index = roomId (2-5), dBm

// Sdileny interval probouzeni pro CENTRALU I VSECHNA CIDLA dohromady - musi
// byt vzdy stejny, aby se vsichni potkali ve stejnem okne. Zmena (V96) se
// NOVY MECHANISMUS (nahrazuje puvodni "pending + delayed activation"):
// zmena V96 se aplikuje OKAMZITE (lowPowerWakeGridSec), ale dokud vsechna
// jiz drive aktivni cidla nepotvrdi, ze o nove mrizce vedi (viz
// RoomData.potvrzenaMrizkaSec), centrala se BUDI SOUBEZNE na STARE i NOVE
// mrizce, aby nepropasla cidlo, ktere jeste spi podle stare hodnoty.
RTC_DATA_ATTR uint32_t predchoziGridSec = 0; // 0 = zadna stara mrizka k soubeznemu sledovani
RTC_DATA_ATTR bool gridZmenaPotvrzena[6] = {true, true, true, true, true, true}; // indexy 2-5 pouzite; true = netreba cekat

// ==========================================================================
// Vypocita, za kolik sekund od "now" nastane nejblizsi udalost - bud pravidelne
// naslouchaci okno (zarovnane na hodinovou hranici, minus 60s predstih), nebo
// denni kontrola baterie v BATTERY_CHECK_HOUR:BATTERY_CHECK_MINUTE.
// ==========================================================================
// ==========================================================================
// DOCASNY TESTOVACI PREPINAC - overeni funkcnosti/spotreby uspornho rezimu
// true  = presne 60s spanek + 60s bdeni (bez ohledu na V96/mrizku)
// false = normalni provoz (hodinova mrizka podle V96, 5 minut bdeni)
// PO DOKONCENI TESTU NEZAPOMEN VRATIT NA false A ZNOVU NAHRAT FIRMWARE!
// ==========================================================================
#define TEST_MODE_SHORT_CYCLES false

// ==========================================================================
// DOCASNY TESTOVACI PREPINAC - nahodni teplota/vlhkost pro overeni zobrazeni
// na displeji i v Blynku (cely rozsah -20 az 40 stupnu, 0 az 100 % vlhkosti)
// PO DOKONCENI TESTU NEZAPOMEN VRATIT NA false A ZNOVU NAHRAT FIRMWARE!
// ==========================================================================
#define TEST_MODE_RANDOM_SENSOR_VALUES false

// ==========================================================================
// DOCASNY TESTOVACI PREPINAC - overeni realne detekce nizkeho/zotaveneho
// stavu baterie, misto cekani na jedine denni okno v 1:30.
// true  = kontrola napeti kazde 2 minuty (bez ohledu na cas), misto 1x denne
// false = normalni chovani (jen 1:30-1:32)
// PO DOKONCENI TESTU NEZAPOMEN VRATIT NA false A ZNOVU NAHRAT FIRMWARE!
// ==========================================================================
#define TEST_MODE_FREQUENT_BATTERY_CHECK false
#define TEST_BATTERY_CHECK_INTERVAL_MS 120000UL // 2 minuty
unsigned long lastTestBatteryCheckMillis = 0;

// Spolecny vypocet: nejblizsi hranice dane mrizky (zarovnane na unix epoch),
// minus 60s predstih (bootovaci rezerva). Pouziva se pro NOVOU mrizku vzdy,
// a pro STAROU mrizku jen dokud vsechna cidla nepotvrdi prijeti zmeny.
uint32_t vypocitejHraniciMrizky(uint32_t nowUnix, uint32_t gridSec) {
  uint32_t hranice = ((nowUnix / gridSec) + 1) * gridSec;
  uint32_t wake = hranice - 60;
  if (wake <= nowUnix) wake += gridSec;
  return wake;
}

// Vraci nejblizsi okamzik (unix cas), kdy bude centrala PRISTE naslouchat -
// bere v potaz i soubezne sledovani stare mrizky, pokud jeste nemame
// potvrzeni od vsech cidel. NEZAHRNUJE denni kontrolu baterie (ta je
// relevantni pro planovani spanku CENTRALY, ne pro to, co rict CIDLU).
uint32_t vypocitejDalsiMrizkoveOkno(uint32_t nowUnix) {
  uint32_t wakeNova = vypocitejHraniciMrizky(nowUnix, lowPowerWakeGridSec);

  bool vsechnyPotvrdily = gridZmenaPotvrzena[2] && gridZmenaPotvrzena[3] &&
                          gridZmenaPotvrzena[4] && gridZmenaPotvrzena[5];
  if (!vsechnyPotvrdily && predchoziGridSec != 0) {
    uint32_t wakeStara = vypocitejHraniciMrizky(nowUnix, predchoziGridSec);
    return min(wakeNova, wakeStara);
  }
  return wakeNova;
}

uint32_t secondsToNextWake(DateTime now) {
  if (TEST_MODE_SHORT_CYCLES) return 300; // test: vzdy presne 5 minut, bez ohledu na mrizku V96
  uint32_t nowUnix = now.unixtime();

  // A) nejblizsi mrizkove okno (nove, pripadne i stare soubezne - viz vyse)
  uint32_t wakeA = vypocitejDalsiMrizkoveOkno(nowUnix);

  // B) nejblizsi vyskyt denni kontroly baterie (dnes, pokud jeste neproslo, jinak zitra)
  DateTime todayCheck(now.year(), now.month(), now.day(), BATTERY_CHECK_HOUR, BATTERY_CHECK_MINUTE, 0);
  uint32_t wakeB;
  if (todayCheck.unixtime() > nowUnix) {
    wakeB = todayCheck.unixtime();
  } else {
    DateTime tomorrowCheck = todayCheck + TimeSpan(1, 0, 0, 0);
    wakeB = tomorrowCheck.unixtime();
  }

  uint32_t nextWake = min(wakeA, wakeB);
  uint32_t diff = nextWake - nowUnix;
  return (diff < 5) ? 5 : diff; // pojistka proti nulovemu/zapornemu spanku
}

void enterLowPowerSleep() {
  digitalWrite(PIN_BACKLIGHT, HIGH);   // zhasnout podsviceni
  backlightOn = false;
  digitalWrite(PIN_PUMP, LOW);         // pro jistotu vypnout cerpadlo pred spankem

  lowPowerMode = true;

  // POZNAMKA: puvodni logika "aplikuj cekajici mrizku, pokud uz nastal jeji
  // aktivacni cas" tady uz neni potreba - lowPowerWakeGridSec se ted meni
  // OKAMZITE primo v BLYNK_WRITE(V96). Soubezne sledovani stare mrizky (dokud
  // vsechna cidla nepotvrdi) resi vypocitejDalsiMrizkoveOkno() uvnitr
  // secondsToNextWake() nize, ne tento blok.
  DateTime now = rtc.now();
  uint32_t sleepSec = secondsToNextWake(now);
  DateTime wakeTime(now.unixtime() + sleepSec);

  // Zapsat do terminalu JESTE PRED odpojenim WiFi, jinak by uz nebylo
  // pripojeni, kterym by se to stihlo poslat. DULEZITE: kontrolujeme
  // Blynk.connected(), ne jen WiFi.status() - WiFi muze byt pripojene, i kdyz
  // se Blynk handshake behem tohohle probuzeni nikdy nestihl dokoncit (napr.
  // pomalejsi pripojeni), a v tom pripade by se zprava stejne neodeslala.
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    // Cerstve zmereni napeti - pripadna predchozi hodnota v Blynku (napr.
    // z posledni periodicke readINA226()) uz muze byt zastarala
    ina_busVoltage = ina226.getBusVoltage();
    Blynk.virtualWrite(V80, ina_busVoltage);

    // Odeslat i vsechna ostatni aktualne znama data, at appka tesne pred
    // usnutim ukazuje opravdu posledni stav, ne stare hodnoty z minula
    Blynk.virtualWrite(V1, T_out);
    Blynk.virtualWrite(V2, vlhkost);
    if (!isnan(Teplota2)) { Blynk.virtualWrite(V41, Teplota2); Blynk.virtualWrite(V49, Vlhkost2); }
    if (!isnan(Teplota3)) { Blynk.virtualWrite(V42, Teplota3); Blynk.virtualWrite(V51, Vlhkost3); }
    if (!isnan(Teplota4)) { Blynk.virtualWrite(V43, Teplota4); Blynk.virtualWrite(V52, Vlhkost4); }
    if (!isnan(Teplota5)) { Blynk.virtualWrite(V45, Teplota5); Blynk.virtualWrite(V53, Vlhkost5); }
    Blynk.virtualWrite(V3, celkovyObjem);
    Blynk.virtualWrite(V7, celkovyObjem);

    char sleepBuf[21], wakeBuf[21];
    sprintf(sleepBuf, "%02d.%02d.%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    sprintf(wakeBuf, "%02d.%02d.%04d %02d:%02d:%02d", wakeTime.day(), wakeTime.month(), wakeTime.year(), wakeTime.hour(), wakeTime.minute(), wakeTime.second());
    logToTerminal(String("Usinam v ") + sleepBuf + ", planovane probuzeni v " + wakeBuf);

    delay(500); // prodlouzeno - posila se ted vic hodnot najednou, potrebuji cas na odeslani
  }

  WiFi.disconnect(true);
  radio.begin();
  radio.sleep(false);                 // uspi i LoRa cip, jinak zbytecne zere proud

  // Bez tohohle by GPIO_BACKLIGHT behem deep sleep "plaval" (digitalni jadro
  // prestane pin aktivne ridit) - a protoze na hradle P-MOSFETu neni pull-up,
  // mohl by se pin sesunout k LOW a podsviceni by se behem spanku nechtene
  // rozsvitilo. gpio_hold_en() rekne RTC domene, ať drzi posledni nastavenou
  // (HIGH = zhasnuto) uroven po celou dobu spanku.
  gpio_hold_en((gpio_num_t)PIN_BACKLIGHT);
  gpio_deep_sleep_hold_en();

  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
  esp_deep_sleep_start();              // odsud dal se kod NEVRACI, cip se pri probuzeni restartuje
}

// Denni rozhodnuti o rezimu podle napeti baterie namerene v klidovem case (1:30),
// kdy solar uz nenabiji a nemichaji se do toho prechody letniho/zimniho casu.
// Pouziva se jak v normalnim behu (volano z timers[]), tak behem low power probuzeni.
// Jak dlouho zustat vzhuru v PLNEM rezimu po probuzeni z uspornho rezimu,
// nez (pokud neni aktivni V97) zarizeni znovu usne. Nahrazuje puvodni
// kratke bdele okno v premostene lowPowerWakeCycle() - ted bezi plny setup
// (WiFi/NTP/Blynk pripojeni chvili trva), takze dava smysl mit okno
// o neco velkorysejsi.
#define AWAKE_WINDOW_LOWPOWER_MS 60000UL

// Kolik vterin rezervy pridat k dobe spanku posilane cidlum - centrala
// potrebuje cca 20s od probuzeni do plneho provozu (WiFi, Blynk), takze
// cidlo ma dorazit az PO tehle rezerve, ne primo na hranici probuzeni.
#define CENTRALA_BOOT_REZERVA_SEC 20

void checkNavratDoUspornhoSpanku() {
  if (!lowPowerMode) return;      // normalni 24/7 provoz - zadny periodicky navrat do spanku
  if (preventNextSleep) return;   // uzivatel explicitne pozadal (V97) zustat vzhuru

  if (millis() > AWAKE_WINDOW_LOWPOWER_MS) {
    Serial.println("Uplynulo bdele okno v uspornem rezimu, vracim se do spanku");
    enterLowPowerSleep(); // nikdy se nevrati - ESP jde rovnou spat
  }
}

void checkDailyBatteryDecision() {
  if (TEST_MODE_FREQUENT_BATTERY_CHECK) {
    if (!lowPowerMode) {
      // Normalni provoz: skutecne limitovat na cca 2 minuty - jinak by se to
      // volalo kazdych 30s, jak casovac v timers[] doopravdy bezi.
      if (millis() - lastTestBatteryCheckMillis < TEST_BATTERY_CHECK_INTERVAL_MS) return;
      lastTestBatteryCheckMillis = millis();
    }
    // V uspornem rezimu ZADNY throttling neni potreba a ani nesmi byt - kazde
    // probuzeni je uz samo o sobe rozestupene (5 minut), a navic je to fresh
    // restart, takze millis() i lastTestBatteryCheckMillis by byly porad
    // blizko sebe (obe zacinaji od nuly) - throttling by tak kontrolu vzdy
    // zablokoval, presny opak toho, co chceme.
  } else {
    DateTime now = rtc.now();
    if (now.hour() != BATTERY_CHECK_HOUR) return;
    if (now.minute() < BATTERY_CHECK_MINUTE || now.minute() > BATTERY_CHECK_MINUTE + 2) return;

    int32_t today = now.unixtime() / 86400L; // jednoduchy "den" identifikator
    if (today == lastBatteryCheckDay) return; // uz dnes probehlo
    lastBatteryCheckDay = today;
  }

  ina_busVoltage = ina226.getBusVoltage();
  Serial.print("Kontrola baterie: ");
  Serial.print(ina_busVoltage);
  Serial.println(" V");

  if (ina_busVoltage < BATTERY_LOW_VOLTAGE && !lowPowerMode) {
    if (preventNextSleep) {
      // Uzivatel explicitne pozadal o zabraneni spanku (V97) - respektujeme
      // to i pres nizke napeti, jinak by checkNavratDoUspornhoSpanku() (nebo
      // primo tady) zarizeni znovu uspalo, coz neni to, co uzivatel chtel.
      Serial.println("Baterie nizka, ale V97 (zabraneni spanku) je aktivni - zustavam v normalnim rezimu");
    } else {
      Serial.println("-> prechazim do uspornho (zimniho) rezimu");
      if (WiFi.status() == WL_CONNECTED) {
        Blynk.logEvent("low_batt_heltec", "Kapacita baterie je menší než 80%, centrála přešla do úsporného režimu.");
      }
      enterLowPowerSleep(); // nikdy se nevrati, ESP jde rovnou spat
    }
  } else if (ina_busVoltage >= BATTERY_RECOVER_VOLTAGE && lowPowerMode) {
    Serial.println("-> baterie zotavena, navrat do normalniho (24/7) rezimu");
    lowPowerMode = false;
    ESP.restart(); // cisty navrat do plneho normalniho setup()
  }
  // mezi thresholdy (hystereze) - zustava beze zmeny
}

// ==========================================================================
// RELE - SPINANI BATERIE/SOLAR REGULATORU
//
// Logika (potvrzeno v rozhovoru):
// 1) Pri probuzeni z uspornho spanku (mimo nocni okno): sepnout rele BATERIE,
//    +2000ms sepnout rele SOLAR, +10000ms OD SEPNUTI SOLARU zmerit napeti.
// 2) Napeti >= 13.6V -> rele zustavaji sepnuta do dalsiho probuzeni.
//    Napeti < 13.6V -> odpojit SOLAR, +2000ms odpojit BATERII.
// 3) Normalni (ne usporny) rezim - rele VZDY sepnuta, ale JEN MIMO nocni okno.
// 4) Nocni okno (21:00-6:00) MA PREDNOST PRED VSIM (normalni i usporny
//    rezim) - po celou tuhle dobu nikdy ani jedno rele sepnute. Prechod
//    do okna spusti odpojovaci sekvenci (solar prvni, +2000ms baterie).
// 5) Bezne, uz existujici mereni napeti baterie (readINA226, denni kontrola
//    v 1:30 apod.) jsou NA STAVU RELE ZCELA NEZAVISLA - stanice je k baterii
//    pripojena natvrdo, napeti lze merit i po fyzickem odpojeni od solaru.
//    Tato sekce pouziva VLASTNI, oddelenou funkci pro cteni napeti pro
//    rozhodovani o rele (zmerNapetiBaterieProRele()), aby nedoslo k
//    ovlivneni zadne z existujicich funkci.
// ==========================================================================

bool jeNocniOkno(DateTime now) {
  int h = now.hour();
  return (h >= NOC_HODINA_OD || h < NOC_HODINA_DO);
}

// Samostatna funkce pro cteni napeti VYHRADNE pro rozhodovani o rele -
// zamerne oddelena od ina_busVoltage/readINA226()/checkDailyBatteryDecision(),
// aby vysledek nijak neovlivnil zobrazeni, Blynk, ani denni kontrolu baterie.
float zmerNapetiBaterieProRele() {
  return ina226.getBusVoltage();
}

// Vraci true, pokud NEKDO JINY (cerpadlo - waterflow senzor, nebo prave
// probihajici rele sekvence) prave potrebuje Vext zapnuty - pouziva se
// pred vypnutim Vext, aby nedoslo k odebrani napajeni necemu, co ho jeste
// potrebuje. Sdileny zdroj mezi waterLevel()/cerpadlem/rele.
bool nekdoPotrebujeVext() {
  return pumpStatus || (releSekvenceStav != RELE_SEQ_KLID);
}

void moznaVypniVext() {
  // Mimo usporny rezim Vext NIKDY nevypinat - oba senzory (waterflow i
  // ultrazvuk) musi byt neustale aktivni kvuli prubeznemu hlidani
  // neocekavaneho uniku, ne jen behem kratkych mericich oken. V zime
  // (usporny rezim, prazdne sudy, nezaleva se) tahle kontrola nevadi, ze
  // chybi - proto se tam Vext dal spina/vypina podle skutecne potreby.
  if (!lowPowerMode) return;

  if (!nekdoPotrebujeVext()) {
    digitalWrite(PIN_VEXT, HIGH);
  }
}

void releZapniBaterii() {
  digitalWrite(PIN_VEXT, LOW);
  delay(20);
  releBaterie.set();
  releBaterieSepnuto = true;
  Serial.println("Rele BATERIE -> sepnuto");
}

void releZapniSolar() {
  digitalWrite(PIN_VEXT, LOW);
  delay(20);
  releSolar.set();
  releSolarSepnuto = true;
  Serial.println("Rele SOLAR -> sepnuto");
}

void releVypniSolar() {
  digitalWrite(PIN_VEXT, LOW);
  delay(20);
  releSolar.reset();
  releSolarSepnuto = false;
  Serial.println("Rele SOLAR -> rozepnuto");
}

void releVypniBaterii() {
  digitalWrite(PIN_VEXT, LOW);
  delay(20);
  releBaterie.reset();
  releBaterieSepnuto = false;
  Serial.println("Rele BATERIE -> rozepnuto");
}

void spustPripojovaciSekvenciRele(bool rucne = false) {
  if (releSekvenceStav != RELE_SEQ_KLID) return; // uz neco probiha, nezasahovat
  releManualniSekvence = rucne;
  Serial.println(rucne ? "Rele: RUCNE zahajuji pripojovaci sekvenci (baterie -> +2s solar)"
                        : "Rele: zahajuji pripojovaci sekvenci (baterie -> +2s solar)");
  releSekvenceStav = RELE_SEQ_ZAPNI_BAT;
}

void spustOdpojovaciSekvenciRele(bool rucne = false) {
  if (releSekvenceStav != RELE_SEQ_KLID) return;
  if (!releBaterieSepnuto && !releSolarSepnuto) {
    // Uz je vse odpojeno - u rucniho pozadavku i tak posleme echo hned,
    // at widget v Blynku nezustane viset na "cekam"
    if (rucne && WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V88, "Odpojeno");
    return;
  }
  releManualniSekvence = rucne;
  Serial.println(rucne ? "Rele: RUCNE zahajuji odpojovaci sekvenci (solar -> +2s baterie)"
                        : "Rele: zahajuji odpojovaci sekvenci (solar -> +2s baterie)");
  releSekvenceStav = RELE_SEQ_VYPNI_SOLAR;
  releSekvenceCasKroku = millis();
}

// Postupuje prave probihajici sekvenci (pokud nejaka bezi) - nezpusobuje
// zadne blokujici cekani, volat casto (viz timers[]).

// Po dokonceni sekvence posle echo do Blynku (V88), ale JEN pokud byla
// vyvolana rucne (V87) - automaticke sekvence (noc, normalni rezim,
// low-power probuzeni) echo neposilaji. Zaroven sjednoti stav prepinace
// V87 se skutecnym stavem rele, at nezustane zobrazovat neco jineho, nez
// je realita (napr. po nocnim automatickem odpojeni).
// Synchronizuje V89 (switch - stav automatiky) a V90 (String "AUTO"/"MANUAL")
// se skutecnou hodnotou releAutomatikaAktivni. Volat pri kazde zmene teto
// promenne, at Blynk vzdy ukazuje aktualni stav.
void aktualizujStavAutomatikyRele() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V89, releAutomatikaAktivni ? 1 : 0);
    Blynk.virtualWrite(V90, releAutomatikaAktivni ? "AUTO" : "MANUAL");
  }
}

// Vypne automatiku rele a POSLE OKAMZITOU notifikaci (volat pri kazdem
// rucnim zasahu - ikonka zarovky, V87, nebo V89 nastavene na 0). Dalsi
// opakovane notifikace (kazde 4h, dokud zustava vypnuta) resi
// kontrolaSolarModeNotifikace() nize.
void vypniAutomatikuRele() {
  releAutomatikaAktivni = false;
  aktualizujStavAutomatikyRele();
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.logEvent("solar_mode", "UPOZORNĚNÍ: Automatické odpojování solárního regulátoru je vypnuté.");
  }
  solarModeNotifikaceMillis = millis();
}

// Periodicka kontrola (viz timers[]) - pokud je automatika stale vypnuta,
// opakuje notifikaci "solar_mode" kazdych SOLAR_MODE_NOTIFIKACE_INTERVAL_MS.
void kontrolaSolarModeNotifikace() {
  if (!releAutomatikaAktivni && WiFi.status() == WL_CONNECTED) {
    if (millis() - solarModeNotifikaceMillis >= SOLAR_MODE_NOTIFIKACE_INTERVAL_MS) {
      Blynk.logEvent("solar_mode", "UPOZORNĚNÍ: Automatické odpojování solárního regulátoru je vypnuté.");
      solarModeNotifikaceMillis = millis();
    }
  }
}

// ==========================================================================
// IKONKA ZAROVKY NA DISPLEJI - rucni pripojeni/odpojeni solar regulatoru
// (svetlo v zahradnim domku je napojene na jeho vystup). Kresleno vektorove
// podle schvaleneho navrhu (50x75 puvodni mockup, prepocitano na 50x90
// s vertikalnim vycentrovanim).
// ==========================================================================
#define ZAROVKA_X 169
#define ZAROVKA_Y 230
#define ZAROVKA_W 62
#define ZAROVKA_H 112

void kresliIkonuZarovky(bool sviti) {
  int cx = ZAROVKA_X + 31;
  int cy = ZAROVKA_Y + 9 + 28; // +9 vertikalni vycentrovani, +28 puvodni lokalni stred banky (prepocitano 125%)
  int r = 19; // 15 * 1.25

  lcd.fillRect(ZAROVKA_X, ZAROVKA_Y + 3, ZAROVKA_W, ZAROVKA_H - 3, BLACK); // orizle nahore o 3px

  uint16_t svetleSeda1 = lcd.color565(230, 230, 230);
  uint16_t svetleSeda2 = lcd.color565(220, 220, 220);
  uint16_t svetleSeda3 = lcd.color565(210, 210, 210);
  uint16_t svetleSeda4 = lcd.color565(200, 200, 200);
  uint16_t stredniSeda = lcd.color565(160, 160, 160);

  if (sviti) {
    uint16_t zluta = lcd.color565(255, 213, 74);
    uint16_t oranzova = lcd.color565(255, 179, 0);
    uint16_t paprsek = lcd.color565(255, 204, 51);

    // Paprsky (5 kratkych usecek, jen horni polovina)
    lcd.drawLine(cx - r, cy, cx - r - 8, cy, paprsek);
    lcd.drawLine(cx + r, cy, cx + r + 8, cy, paprsek);
    lcd.drawLine(cx, cy - r, cx, cy - r - 8, paprsek);
    lcd.drawLine(cx - (int)(r * 0.7), cy - (int)(r * 0.7), cx - (int)(r * 0.7) - 5, cy - (int)(r * 0.7) - 5, paprsek);
    lcd.drawLine(cx + (int)(r * 0.7), cy - (int)(r * 0.7), cx + (int)(r * 0.7) + 5, cy - (int)(r * 0.7) - 5, paprsek);

    // Banka - vyplnena zluta
    lcd.fillCircle(cx, cy, r, zluta);
    lcd.drawCircle(cx, cy, r, oranzova);

    // Text "ON" doprostred banky
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextSize(1);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(lcd.color565(90, 60, 0));
    lcd.drawString("ON", cx, cy + 2);
    lcd.setTextDatum(TL_DATUM);

    // Hrdlo a zavit - svetle sede
    lcd.fillRect(cx - 8, cy + r - 3, 16, 10, svetleSeda1);
    lcd.fillRect(cx - 10, cy + r + 8, 20, 12, svetleSeda2);
    lcd.drawFastHLine(cx - 10, cy + r + 11, 20, stredniSeda);
    lcd.drawFastHLine(cx - 10, cy + r + 16, 20, stredniSeda);
    lcd.fillRect(cx - 6, cy + r + 20, 12, 5, svetleSeda4);
  } else {
    // Banka - jen obrys
    lcd.drawCircle(cx, cy, r, svetleSeda1);
    lcd.drawCircle(cx, cy, r - 1, svetleSeda1); // 2px tloustka obrysu

    // Vlakno uvnitr (jednoduchy cikcak)
    lcd.drawLine(cx - 10, cy + 5, cx - 4, cy - 8, svetleSeda4);
    lcd.drawLine(cx - 4, cy - 8, cx + 4, cy + 5, svetleSeda4);
    lcd.drawLine(cx + 4, cy + 5, cx + 10, cy - 8, svetleSeda4);

    // Hrdlo a zavit - svetle sede
    lcd.fillRect(cx - 8, cy + r - 3, 16, 10, svetleSeda3);
    lcd.fillRect(cx - 10, cy + r + 8, 20, 12, svetleSeda2);
    lcd.drawFastHLine(cx - 10, cy + r + 11, 20, stredniSeda);
    lcd.drawFastHLine(cx - 10, cy + r + 16, 20, stredniSeda);
    lcd.fillRect(cx - 6, cy + r + 20, 12, 5, svetleSeda4);
  }
}

void poDokonceniSekvenceRele() {
  bool pripojeno = releBaterieSepnuto && releSolarSepnuto;
  if (WiFi.status() == WL_CONNECTED) {
    // Oboje se aktualizuje VZDY, bez ohledu na to, jestli sekvenci vyvolala
    // automatika, nebo rucni zasah - jinak V88 zustavalo na stare, uz
    // neplatne hodnote z posledni rucni akce, i kdyz mezitim automatika
    // stav zmenila.
    Blynk.virtualWrite(V87, pripojeno ? 1 : 0);
    Blynk.virtualWrite(V88, pripojeno ? "Pripojeno" : "Odpojeno");
  }

  // Prekreslit ikonku zarovky, jen pokud je dashboard prave viditelny
  // (ne behem zalevani, kdy displej ukazuje jinou obrazovku)
  if (displayLogicOn && !pumpStatus) {
    kresliIkonuZarovky(pripojeno);
  }

  releManualniSekvence = false; // reset pro pristi (mozna automatickou) sekvenci
}

// Dokonci "cuknuti" cerpadlem (viz waterLevelCheck()) - vypne ho presne
// CUKNUTI_CERPADLA_MS po zapnuti, bez blokujiciho cekani. Volat casto
// (viz timers[]).
void kontrolaCuknutiCerpadla() {
  if (cuknutiCerpadlaAktivni && (millis() - cuknutiCerpadlaStartMillis >= CUKNUTI_CERPADLA_MS)) {
    digitalWrite(PIN_PUMP, LOW);
    cuknutiCerpadlaAktivni = false;
  }
}

void krokReleSekvence() {
  unsigned long ted = millis();

  switch (releSekvenceStav) {
    case RELE_SEQ_KLID:
      break; // nic se nedeje

    case RELE_SEQ_ZAPNI_BAT:
      releZapniBaterii();
      releSekvenceCasKroku = ted;
      releSekvenceStav = RELE_SEQ_CEKAM_SOLAR_ZAP;
      break;

    case RELE_SEQ_CEKAM_SOLAR_ZAP:
      if (ted - releSekvenceCasKroku >= RELE_ODSTUP_MS) {
        releZapniSolar();
        releSekvenceCasKroku = ted;
        if (lowPowerMode && !releManualniSekvence) {
          releSekvenceStav = RELE_SEQ_CEKAM_MERENI; // uspory rezim - jeste rozhodovaci mereni
        } else {
          releSekvenceStav = RELE_SEQ_CEKAM_VEXT_OFF; // normalni rezim (nebo rucni pozadavek) - hotovo,
                                                        // jen jeste chvili podrzet Vext (viz konstanta)
        }
      }
      break;

    case RELE_SEQ_CEKAM_MERENI:
      if (ted - releSekvenceCasKroku >= RELE_MERENI_ODSTUP_MS) {
        float napeti = zmerNapetiBaterieProRele();
        Serial.print("Rele: rozhodovaci mereni napeti = "); Serial.print(napeti); Serial.println(" V");

        releWakeRozhodnutoVTomtoWake = true;

        if (napeti < RELE_NAPETI_PRAH) {
          Serial.println("Rele: napeti pod prahem -> zahajuji odpojovaci sekvenci");
          releSekvenceStav = RELE_SEQ_VYPNI_SOLAR;
          releSekvenceCasKroku = ted;
        } else {
          Serial.println("Rele: napeti OK -> zustavaji sepnuta do dalsiho probuzeni");
          releSekvenceStav = RELE_SEQ_CEKAM_VEXT_OFF;
          releSekvenceCasKroku = ted;
        }
      }
      break;

    case RELE_SEQ_VYPNI_SOLAR:
      releVypniSolar();
      releSekvenceCasKroku = ted;
      releSekvenceStav = RELE_SEQ_CEKAM_BAT_VYP;
      break;

    case RELE_SEQ_CEKAM_BAT_VYP:
      if (ted - releSekvenceCasKroku >= RELE_ODSTUP_MS) {
        releVypniBaterii();
        releSekvenceCasKroku = ted;
        releSekvenceStav = RELE_SEQ_CEKAM_VEXT_OFF;
      }
      break;

    case RELE_SEQ_CEKAM_VEXT_OFF:
      if (ted - releSekvenceCasKroku >= RELE_VEXT_HOLD_MS) {
        moznaVypniVext();
        poDokonceniSekvenceRele();
        releSekvenceStav = RELE_SEQ_KLID;
      }
      break;
  }
}

// Hlavni periodicka rozhodovaci funkce - urcuje, JESTLI je potreba spustit
// pripojovaci nebo odpojovaci sekvenci, podle nocniho okna a rezimu.
// Samotne KROKOVANI uz probihajici sekvence resi krokReleSekvence() vyse.
void rizeniRele() {
  if (!releAutomatikaAktivni) return; // automatika docasne vypnuta rucnim zasahem - nic sama nedelat

  DateTime now = rtc.now();
  bool noc = jeNocniOkno(now);

  if (noc) {
    // Nocni okno ma prednost pred vsim - pokud je cokoliv sepnute, odpojit
    if (releSekvenceStav == RELE_SEQ_KLID && (releBaterieSepnuto || releSolarSepnuto)) {
      spustOdpojovaciSekvenciRele();
    }
  } else if (!lowPowerMode) {
    // Normalni rezim, mimo noc - rele maji byt vzdy sepnuta OBE. Kontrola
    // "neni sepnuto obe" (ne jen "obe jsou vypnuta") zachyti i castecne
    // nekonzistentni stav, napr. po vypadku napajeni uprostred sekvence.
    if (releSekvenceStav == RELE_SEQ_KLID && !(releBaterieSepnuto && releSolarSepnuto)) {
      spustPripojovaciSekvenciRele();
    }
  } else {
    // Usporny rezim, mimo noc - spustit pripojovaci+rozhodovaci sekvenci
    // jen JEDNOU za tohle konkretni probuzeni
    if (!releWakeRozhodnutoVTomtoWake && releSekvenceStav == RELE_SEQ_KLID) {
      spustPripojovaciSekvenciRele();
    }
  }
}

// Probuzeni z low power rezimu - denni kontrola baterie, LoRa prijem od cidel
// (vcetne odeslani odpovedi s casem/konfiguraci), a az 5 minut bdeleho okna
// pro pripojeni k Blynk, aby mel uzivatel realny cas na interakci pres appku.
void lowPowerWakeCycle() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  bme.begin(BME280_ADRESA);
  ina226.begin();
  ina226.setAverage(INA226_128_SAMPLES);
  ina226.setBusVoltageConversionTime(INA226_8300_us);
  ina226.setShuntVoltageConversionTime(INA226_8300_us);
  rtc.begin();
  delay(50);

  // Pokud se prave trefujeme do 1:30 okna, rozhodne se tu o rezimu na dalsi den.
  // Pri zotaveni baterie tato funkce sama zavola ESP.restart() a sem se uz nevratime.
  checkDailyBatteryDecision();

  float temp = bme.readTemperature();
  ina_busVoltage = ina226.getBusVoltage();

  int loraState = radio.begin(868.125, 125.0, 7, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22); // SF7 (uspora energie cidel, sladeno s cidly), vykon 22dBm (centrala ma velkou baterii + solar)
  if (loraState == RADIOLIB_ERR_NONE) {
    radio.setDio1Action(onLoraDio1);
    radio.startReceive();
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long wakeStart = millis();
  const unsigned long maxAwakeMs = TEST_MODE_SHORT_CYCLES ? (60UL * 1000UL) : (30UL * 1000UL); // test: 1 minuta, jinak 30s
  bool blynkReady = false;
  bool valuesSent = false;

  // Integrace spotreby behem bdeni - jedina cast cyklu, kterou muzeme skutecne
  // zmerit (INA226 je dostupny jen kdyz CPU bezi)
  float awakeChargeAh = 0;
  unsigned long lastCurrentSampleMillis = wakeStart;

  while (millis() - wakeStart < maxAwakeMs) {
    loraReceive(); // prijem od cidel + automaticka odpoved s casem/konfigurací (viz uvnitr funkce)

    float currentA = -ina226.getCurrent_mA() / 1000.0; // stejna korekce polarity jako v readINA226()
    unsigned long nowMillisSample = millis();
    float elapsedHoursSample = (nowMillisSample - lastCurrentSampleMillis) / 3600000.0;
    if (elapsedHoursSample > 0 && elapsedHoursSample < 0.01) { // pojistka proti nesmyslnemu skoku
      awakeChargeAh += currentA * elapsedHoursSample;
    }
    lastCurrentSampleMillis = nowMillisSample;

    if (WiFi.status() == WL_CONNECTED) {
      if (!blynkReady) {
        Blynk.config(auth);
        if (Blynk.connect(3000)) blynkReady = true;
      } else {
        Blynk.run();
        if (!valuesSent) {
          Blynk.virtualWrite(V1, temp);
          Blynk.virtualWrite(V80, ina_busVoltage);
          if (!isnan(Teplota2)) { Blynk.virtualWrite(V41, Teplota2); Blynk.virtualWrite(V49, Vlhkost2); }
          if (!isnan(Teplota3)) { Blynk.virtualWrite(V42, Teplota3); Blynk.virtualWrite(V51, Vlhkost3); }
          if (!isnan(Teplota4)) { Blynk.virtualWrite(V43, Teplota4); Blynk.virtualWrite(V52, Vlhkost4); }
          if (!isnan(Teplota5)) { Blynk.virtualWrite(V45, Teplota5); Blynk.virtualWrite(V53, Vlhkost5); }
          Blynk.virtualWrite(V95, "Online");
          {
            DateTime now = rtc.now();
            char buf[21];
            sprintf(buf, "%02d.%02d.%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
            logToTerminal(String("Probuzeni v ") + buf);
          }
          valuesSent = true;
        }
      }
    }

    esp_task_wdt_reset(); // "nakrmit" WDT - tahle smycka bezne trva jen 30-60s, ale pro jistotu
    delay(50);
  }

  // ---------- Odhad prumerne spotreby v uspornem rezimu ----------
  // Spankovou fazi neni mozne zmerit primo - pouziva se odhad/naměřená
  // konstanta SLEEP_CURRENT_A (viz definice). Bdeni je zmereno skutecne.
  // K obojimu se navic pricita REGULATOR_QUIESCENT_A - vlastni spotreba
  // solarniho regulatoru, kterou INA226 nevidi, ale bere se porad.
  float awakeHours = maxAwakeMs / 3600000.0;
  float sleepHours = lowPowerWakeGridSec / 3600.0;
  float totalCycleHours = awakeHours + sleepHours;
  float totalChargePerCycleAh = awakeChargeAh
                               + (SLEEP_CURRENT_A * sleepHours)
                               + (REGULATOR_QUIESCENT_A * totalCycleHours);
  float avgChargePerHourLowPower = (totalCycleHours > 0) ? (totalChargePerCycleAh / totalCycleHours) : 0;
  float avgChargePerDayLowPower = avgChargePerHourLowPower * 24.0;

  Serial.print("Usporny rezim - spotreba behem bdeni: "); Serial.print(awakeChargeAh, 5);
  Serial.print(" Ah | prumer: "); Serial.print(avgChargePerHourLowPower, 4);
  Serial.print(" Ah/hod, "); Serial.print(avgChargePerDayLowPower, 3); Serial.println(" Ah/den");

  if (WiFi.status() == WL_CONNECTED && blynkReady) {
    Blynk.virtualWrite(V82, avgChargePerHourLowPower);
    Blynk.virtualWrite(V83, avgChargePerDayLowPower);

    // Odhad vydrze do UPLNEHO vybiti (ne jen do prahu uspornho rezimu, tam uz jsme) -
    // dostupny naboj se pocita od aktualniho SOC az k 0 %.
    float currentSOC = estimateBatterySOC(ina_busVoltage);
    float availableAhUntilDead = currentSOC / 100.0 * BATTERY_CAPACITY_AH;
    float estimatedDaysUntilDead = (avgChargePerDayLowPower > 0.001) ? (availableAhUntilDead / avgChargePerDayLowPower) : 0;
    Blynk.virtualWrite(V84, formatDny((int)round(estimatedDaysUntilDead)));
    Blynk.setProperty(V84, "label", "ÚSPORNÝ REŽIM");

    Serial.print("Odhad vydrze do uplneho vybiti: "); Serial.print(estimatedDaysUntilDead, 1); Serial.println(" dni");
  }

  // Pokud uzivatel behem bdeleho okna zapnul V97 (napr. kvuli chystanemu OTA),
  // zustaneme vzhuru dal - kontrolujeme LoRa a Blynk, dokud prepinac sam
  // nevypne. WDT je aktivni (inicializuje se uz na zacatku setup(), pred
  // rozhodnutim o skoku sem) - pokud by se tahle smycka nekdy skutecne
  // zaseknula, WDT ji po 10 minutach sam restartuje.
  //
  // POZNAMKA: tahle cela funkce (lowPowerWakeCycle) je nyni PREMOSTENA
  // v setup() a nikdy se nevola - ponechano nedotcene pro pripadny navrat
  // k puvodnimu chovani (odlehcene probuzeni bez displeje).
  while (preventNextSleep) {
    loraReceive();
    if (WiFi.status() == WL_CONNECTED) {
      if (!blynkReady) {
        Blynk.config(auth);
        if (Blynk.connect(3000)) blynkReady = true;
      } else {
        Blynk.run();
      }
    }
    esp_task_wdt_reset(); // DULEZITE - tahle smycka muze bezet libovolne dlouho (dokud V97 nevypnes)
    delay(50);
  }

  enterLowPowerSleep(); // spocita dalsi cil (hodinova hranice, nebo 1:30) a jde znovu spat
}

// ==========================================================================
// BOOT OBRAZOVKA - vypisuje prubeh inicializace na displej
// ==========================================================================
int bootLine = 20;
void bootLog(const char* text, uint16_t color = WHITE) {
  lcd.setFont(&fonts::FreeSans9pt7b);
  lcd.setTextSize(1);
  lcd.setTextColor(color, BLACK);
  lcd.setCursor(5, bootLine);
  lcd.print(text);
  bootLine += 20;
  Serial.println(text);
  delay(50);
}

// ==========================================================================
// SETUP
// ==========================================================================
// ==========================================================================
// NACTENI WIFI/BLYNK UDAJU Z NVS
// Predpoklada, ze uz byl jednou spusten "nvs_provision.ino" (samostatny
// sketch), ktery udaje do NVS zapsal. Pokud NVS zadne udaje neobsahuje
// (poprve pouzivas tenhle firmware, nebo provisioning jeste neprobehl),
// ssid/pass/auth zustanou prazdne - WiFi/Blynk pripojeni proste selze
// a bootLog() o tom napise varovani.
// ==========================================================================
// ==========================================================================
// DUVOD POSLEDNIHO RESETU - ulozeno do RTC pameti, prezije i deep sleep,
// takze se da poslat do Blynku hned pri prvnim uspesnem pripojeni.
// Klicove pro diagnostiku zaseknuti na dalku, bez nutnosti pripojovat USB
// (u zarizeni v terenu casto samo o sobe zpusobi reset pri pripojeni).
// ==========================================================================
RTC_DATA_ATTR char lastResetReasonMsg[48] = "";

String resetReasonToText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "Zapnuti napajeni";
    case ESP_RST_EXT:       return "Externi reset (tlacitko/USB DTR)";
    case ESP_RST_SW:        return "Softwarovy restart (ESP.restart)";
    case ESP_RST_PANIC:     return "PAD - Guru Meditation / vyjimka";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG - hlavni smycka se zasekla";
    case ESP_RST_WDT:       return "Jiny watchdog";
    case ESP_RST_DEEPSLEEP: return "Probuzeni z deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT - podpeti";
    case ESP_RST_SDIO:      return "SDIO reset";
    default:                return "Neznamy duvod (" + String((int)reason) + ")";
  }
}

void checkLastResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  String text = resetReasonToText(reason);
  Serial.print("Duvod posledniho resetu: ");
  Serial.println(text);
  text.toCharArray(lastResetReasonMsg, sizeof(lastResetReasonMsg));
}

void loadCredentialsFromNVS() {
  Preferences credPrefs;
  credPrefs.begin("creds", true); // "true" = jen pro cteni

  String s = credPrefs.getString("ssid", "");
  String p = credPrefs.getString("pass", "");
  String a = credPrefs.getString("auth", "");

  credPrefs.end();

  s.toCharArray(ssid, sizeof(ssid));
  p.toCharArray(pass, sizeof(pass));
  a.toCharArray(auth, sizeof(auth));

  if (strlen(ssid) == 0 || strlen(auth) == 0) {
    Serial.println("VAROVANI: WiFi/Blynk udaje v NVS chybi nebo jsou prazdne!");
    Serial.println("Spust nejdriv samostatny sketch nvs_provision.ino.");
  }
}

void setup()
{
  heltec_setup(); // Serial.begin(115200) + SPI sbernice pro LoRa - vse uz uvnitr

  // KRITICKE - nastavit HNED, jako uplne prvni vec: bez tohohle GPIO pin
  // cerpadla nema po zapnuti definovany stav az do teto chvile, coz
  // zpusobovalo cca 1s "zabliknuti" cerpadla pri kazdem startu (nez se
  // pin nastavil, driv az mnohem pozdeji v puvodnim poradi setup()).
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);

  // Vext JIZ NENAPAJI DISPLEJ (ten je nyni HW zapojeny natvrdo na 3V3) -
  // slouzi ultrazvukovemu senzoru hladiny, waterflow senzoru a modulum
  // rele (baterie/solar). Zapnuto CO NEJDRIV (hned po pumpe) - rele
  // potrebuji I2C komunikaci uz pri sve inicializaci o dost pozdeji v
  // teto funkci, takze cim driv Vext zapneme, tim vic casu maji moduly na
  // ustaleni napajeni pred prvnim pokusem o komunikaci. Zustava zapnuty
  // po celou dobu bootovani, vypne se az na konci setup(), pokud uz nic
  // nepotrebuje (viz moznaVypniVext() volane tam).
  //
  // Diagnostika senzoru hladiny (drive neodpovidal spravne): problem byl
  // v poradi volani v waterLevel() (Blynk.run()/runTimers() volane pred
  // pulseIn() misto az po nem) - opraveno a POTVRZENO funkcni na hardwaru.
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW); // Vext ON po celou dobu bootovani

  analogReadResolution(10);

  checkLastResetReason(); // zjistit a ulozit duvod posledniho resetu (viz lastResetReasonMsg)

  // DOCASNY DIAGNOSTICKY VYPIS - jaky je SKUTECNY stav pinu podsviceni hned
  // na zacatku, driv nez se ho nas kod vubec dotkne. POZOR: zamerne NEMENIME
  // pinMode() pred tímto ctenim - mohlo by to narusit pripadny aktivni
  // "hold" z deep sleep drive, nez ho stihneme precist.
  Serial.print("DBG: PIN_BACKLIGHT skutecny stav hned na zacatku setup() = ");
  Serial.println(digitalRead(PIN_BACKLIGHT) == HIGH ? "HIGH (spravne, podsviceni OFF)" : "LOW (spatne, podsviceni ON!)");

  // Nacteni WiFi/Blynk udaju z NVS - MUSI byt pred jakymkoliv pouzitim
  // ssid/pass/auth, vcetne pripadneho lowPowerWakeCycle() nize
  loadCredentialsFromNVS();

  // Snizeni taktu CPU - projekt nepotrebuje zadny vykon (dotyk, senzory, LoRa
  // jsou vsechny pomale ve srovnani s CPU), setrime tim aktivni spotrebu jadra
  setCpuFrequencyMhz(80);

  // ---------- Task Watchdog Timer ----------
  // DULEZITE: musi byt nastaveny JESTE PRED rozhodnutim o skoku do
  // lowPowerWakeCycle() nize - jinak by ochrana proti zaseknuti platila jen
  // pro normalni provoz, ne pro odlehcene probuzeni v uspornem rezimu.
  // Pokud se firmware nekdy zasekne (nekonecna smycka, zamrzla periferie),
  // WDT ho sam restartuje - dulezite pro zarizeni bez fyzickeho dohledu.
  // Timeout 10 minut - u tohohle zarizeni nezalezi na rychlosti reakce,
  // hlavni je mit jistotu, ze se i pri zaseknuti sam vzpamatuje.
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 600000, // 10 minut
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  // POZOR: esp_task_wdt_init() by tu SELHAL - framework (Arduino-ESP32 jadro
  // / heltec_setup()) uz TWDT sam automaticky inicializuje jeste pred timhle
  // bodem (viz "TWDT already initialized" v Serial vypisu hned na startu),
  // s vlastnim (kratkym) vychozim timeoutem. Proto pouzivame reconfigure(),
  // ktery zmeni JIZ existujici konfiguraci na nasi - init() by jen ticha
  // (nebo hlucne v logu) selhal a nase 10minutove nastaveni by se vubec
  // nepouzilo, takze by dal platil ten puvodni kratky timeout.
  esp_task_wdt_reconfigure(&wdtConfig);
  esp_task_wdt_add(NULL); // prihlasit aktualni (hlavni) ulohu pod dohled WDT
                          // (pokud uz ji framework sam prihlasil driv, tenhle
                          // radek jen bezpecne nic neudela / vypise hlaseni)

  // DOCASNY DIAGNOSTICKY VYPIS - zjistujeme, proc se po probuzeni prepadava
  // do plneho bootu misto lowPowerWakeCycle()
  Serial.print("DBG: lowPowerMode="); Serial.print(lowPowerMode);
  Serial.print(", wakeup_cause="); Serial.println(esp_sleep_get_wakeup_cause());
  Serial.println("DBG: (ocekavano lowPowerMode=1, wakeup_cause=4 pro ESP_SLEEP_WAKEUP_TIMER)");

  // PREMOSTENO (docasne, na zadost) - lowPowerWakeCycle() se uz nevola,
  // vzdy probehne plny boot s displejem, i po probuzeni z uspornho rezimu.
  // Rozhodnuti "kdy jit zase spat" se nyni resi v normalnim loop() pres
  // checkNavratDoUspornhoSpanku() - viz tam. Funkce lowPowerWakeCycle()
  // zustava v kodu nedotcena pro pripadny navrat k puvodnimu chovani.
  //
  // if (lowPowerMode && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
  //   pinMode(19, OUTPUT);
  //   digitalWrite(19, HIGH);
  //   lowPowerWakeCycle();
  // }
  if (lowPowerMode && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    pinMode(19, OUTPUT);
    digitalWrite(19, HIGH); // podsviceni zustava zhasnute, dokud ho uzivatel sam nezapne
  }

  pinMode(PIN_BACKLIGHT, OUTPUT);
  digitalWrite(PIN_BACKLIGHT, HIGH); // zhasnuto, aktivni LOW

  // Uvolnit pripadny "hold" z minuleho deep sleep (viz enterLowPowerSleep()) -
  // DULEZITE POREADI: pin musi byt nejdriv nastaveny do znameho stavu (viz
  // vyse) a teprve POTOM se hold uvolni - jinak hrozi kratky glitch podle
  // oficialni dokumentace (ESP-IDF gpio_hold_dis).
  gpio_hold_dis((gpio_num_t)PIN_BACKLIGHT);

  backlightOn = false;

  // ---------- Display ----------
  lcd.init();
  lcd.setRotation(1);
  lcd.fillScreen(BLACK);
  bootLog("Displej inicializovan", GREEN);

  digitalWrite(PIN_BACKLIGHT, LOW); // behem bootu necham svitit, at je videt prubeh
  backlightOn = true;
  backLightON = millis();

  // ---------- EEPROM ----------
  EEPROM.begin(512);
  bootLog("EEPROM pripravena", GREEN);

  // ---------- I2C sbernice ----------
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  bootLog("I2C sbernice spustena", GREEN);

  // ---------- RTC ----------
  if (rtc.begin()) {
    bootLog("RTC DS3231 - OK", GREEN);
    if (rtc.lostPower()) {
      bootLog("RTC ztratil napajeni, nastav cas rucne!", RED);
    }
    loadDailyChargeFromEEPROM(); // obnovit klouzave prumery spotreby, pokud existuji v EEPROM
  } else {
    bootLog("RTC DS3231 - CHYBA", RED);
  }

  // ---------- BME280 ----------
  if (bme.begin(BME280_ADRESA)) {
    bootLog("BME280 - OK", GREEN);
  } else {
    bootLog("BME280 - CHYBA (zkontroluj adresu)", RED);
  }

  // ---------- INA226 ----------
  if (ina226.begin()) {
    ina226.setMaxCurrentShunt(8.0, 0.010); // R010 = 0.010 ohm, overeno oznacenim na fyzickem modulu + potvrzeno mereni proti OWON SPE6102
    // Hardwarove prumerovani: 128 vzorku x (8.3ms bus + 8.3ms shunt) = ~2.1s na davku.
    // Cteme softwarove jednou za 3s, takze kazda davka je vzdy hotova a cerstva,
    // a pokryje ~70% casu mezi cteninmi - zachyti i kratkou LoRa vysilaci spicku,
    // ktera by se do puvodniho ~2ms okna trefila jen nahodou.
    ina226.setAverage(INA226_128_SAMPLES);
    ina226.setBusVoltageConversionTime(INA226_8300_us);
    ina226.setShuntVoltageConversionTime(INA226_8300_us);
    bootLog("INA226 - OK", GREEN);
  } else {
    bootLog("INA226 - CHYBA", RED);
  }

  // ---------- Rele (baterie/solar) ----------
  // Vext uz je zapnuty od uplneho zacatku setup() (viz tam), zadne dalsi
  // zapinani/cekani tedy netreba.
  Modulino.begin();

  bool releBaterieOk = releBaterie.begin();
  bool releSolarOk = releSolar.begin();

  if (releBaterieOk && releSolarOk) {
    bootLog("Rele baterie/solar - OK", GREEN);

    // Zjistit SKUTECNY stav rele (latch rele drzi stav i bez napajeni,
    // takze po restartu/vypadku nemusi nase softwarove sledovani
    // odpovidat realite - overit primo pres getStatus()).
    int statusBat = releBaterie.getStatus();
    if (statusBat >= 0) {
      releBaterieSepnuto = (statusBat == 1);
    } else {
      bootLog("Rele BATERIE - stav se nepodarilo zjistit", RED);
    }

    int statusSolar = releSolar.getStatus();
    if (statusSolar >= 0) {
      releSolarSepnuto = (statusSolar == 1);
    } else {
      bootLog("Rele SOLAR - stav se nepodarilo zjistit", RED);
    }

    Serial.print("Rele skutecny stav pri startu: BATERIE=");
    Serial.print(releBaterieSepnuto ? "ON" : "OFF");
    Serial.print(", SOLAR=");
    Serial.println(releSolarSepnuto ? "ON" : "OFF");
  } else {
    bootLog("Rele baterie/solar - CHYBA (modul nenalezen)", RED);
  }

  // Vext ZAMERNE NEVYPINAME tady - ma zustat zapnuty po celou dobu
  // bootovaci sekvence (viz komentar u puvodniho zapnuti na zacatku
  // setup()). Vypne se az na uplnem konci setup(), pokud uz nic nepotrebuje.

  // ---------- Ultrazvuk ----------
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  bootLog("Ultrazvuk JSN-SR04T-V3.0 pripraven", GREEN);

  // ---------- Cerpadlo ----------
  // pinMode/digitalWrite uz probehly uplne na zacatku setup() (viz tam) -
  // tady uz jen informativni zaznam do boot logu.
  bootLog("Cerpadlo - vypnuto", GREEN);

  // ---------- Prutokomer ----------
  pinMode(PIN_WATER_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_WATER_FLOW), pin_ISR, FALLING);
  bootLog("Prutokomer YF-S201 pripraven", GREEN);

  // ---------- Onboard SX1262 (heltec_unofficial.h + RadioLib) ----------
  int loraState = radio.begin(868.125, 125.0, 7, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22); // SF7 (uspora energie cidel, sladeno s cidly), vykon 22dBm (centrala ma velkou baterii + solar)
  if (loraState == RADIOLIB_ERR_NONE) {
    radio.setDio1Action(onLoraDio1);
    radio.startReceive();
    bootLog("Onboard SX1262 - OK, naslouchani spusteno", GREEN);
  } else {
    bootLog("Onboard SX1262 - CHYBA inicializace", RED);
  }

  // ---------- Nacteni hodnot z EEPROM ----------
  hladinaPokles = EEPROM.read(21);
  hladinaPrirustek = EEPROM.read(22);
  hladinaMinimum = EEPROM.read(23);
  spotrebaUspornyRezim = EEPROM.read(24);
  rezimy = EEPROM.read(16);
  Notifikace = EEPROM.read(20);
  lastUsedAmount = EEPROM.read(14);
  litryLimit = EEPROM.read(9);
  Timer1 = EEPROM.read(17);
  Timer2 = EEPROM.read(18);
  Timer3 = EEPROM.read(19);
  startHour1 = EEPROM.read(10);   startMinute1 = EEPROM.read(110);
  startHour2 = EEPROM.read(11);   startMinute2 = EEPROM.read(111);
  startHour3 = EEPROM.read(12);   startMinute3 = EEPROM.read(112);
  ubyloTyden = EEPROMReadInt(154);
  naprseloTyden = EEPROMReadInt(150);
  napusteno = EEPROMReadInt(166);
  ubyloCelkem = EEPROMReadInt(162);
  naprseloCelkem = EEPROMReadInt(158);
  casovac1 = (startHour1 * 3600L) + (startMinute1 * 60L);
  casovac2 = (startHour2 * 3600L) + (startMinute2 * 60L);
  casovac3 = (startHour3 * 3600L) + (startMinute3 * 60L);

  // Sdileny interval probouzeni (V96) - pojistka proti nezapsane flash pameti
  // (cetla by se jako 0xFFFFFFFF, coz je naprosto nesmyslna hodnota)
  uint32_t savedGridSec = 0;
  EEPROM.get(EEPROM_ADDR_WAKE_GRID_SEC, savedGridSec);
  if (savedGridSec > 0 && savedGridSec < 86400UL) { // rozumny rozsah: 1s az 24h
    lowPowerWakeGridSec = savedGridSec;
  }

  // Vysilaci vykony jednotlivych cidel (V120-V123) - ulozeno s offsetem +9
  // (0..31 odpovida -9..+22 dBm), pojistka proti nezapsane flash (0xFF)
  for (int i = 0; i < 4; i++) {
    byte savedPower = EEPROM.read(EEPROM_ADDR_TXPOWER_BASE + i);
    if (savedPower <= 31) {
      desiredTxPower[2 + i] = (int8_t)savedPower - 9; // zpet na dBm
    }
  }

  bootLog("Hodnoty z EEPROM nacteny", GREEN);

  // ---------- WiFi - NEBLOKUJICI pokus o pripojeni ----------
  bootLog("Pripojuji WiFi (bez cekani)...", WHITE);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  unsigned long wifiAttemptStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiAttemptStart) < 8000) {
    delay(200); // max 8s cekani pri startu, POTOM POKRACUJEME I OFFLINE
  }
  if (WiFi.status() == WL_CONNECTED) {
    bootLog("WiFi pripojena", GREEN);
    WiFi.setSleep(true); // modem sleep - WiFi rusi vysilac/prijimac mezi beacony, setri proud
    syncRtcFromNtp(); // stahne cas z internetu a nastavi RTC - jen pri startu
    Blynk.config(auth);
    if (Blynk.connect(3000)) {
      bootLog("Blynk pripojen", GREEN);
      blynkWasConnected = true;
      logToTerminal("Start firmware " + rtc.now().timestamp(DateTime::TIMESTAMP_FULL) + ". Duvod posledniho resetu: " + String(lastResetReasonMsg));
      // Nativni "Signal Level" a "Battery Level" widgety v zahlavi jsou od
      // zarizeni SKRYTE, dokud se explicitne neodkryji - jinak po ulozeni
      // v appce nejsou videt
      Blynk.setProperty(V65, "isHidden", false);
      Blynk.setProperty(V66, "isHidden", false);
    } else {
      bootLog("Blynk nepripojen - pokracuji offline", RED);
    }
    wifiWasConnected = true;
  } else {
    bootLog("WiFi nedostupna - RTC beri cas z baterie/posledniho nastaveni", RED);
  }

  // ---------- Vychozi vykresleni dashboardu ----------
  delay(1500);
  drawDashboard();
  displayTime();
  getTemperature();
  waterLevel();
  readINA226();

  // Pocatecni vykresleni ikonky zarovky (podle skutecneho stavu rele
  // zjisteneho pri jejich inicializaci vyse) a synchronizace stavu
  // automatiky (vzdy aktivni po cerstvem startu, viz releAutomatikaAktivni)
  kresliIkonuZarovky(releBaterieSepnuto && releSolarSepnuto);
  aktualizujStavAutomatikyRele();

  // Prvni vyhodnoceni rele hned po startu (RTC uz je pripraveny) - necekat
  // az na prvni tik periodickeho casovace (viz timers[])
  rizeniRele();

  // Jednorazovy zapis casu startu - pro pripadnou kontrolu, jestli nedochazi
  // k neocekavanym (samovolnym) restartum
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V58, rtc.now().timestamp(DateTime::TIMESTAMP_FULL));
    resyncAllToBlynk(); // az ted, po vsech pocatecnich merenich - at appka hned ukazuje spravna data
  }

  // Prvni zobrazeni WiFi/Blynk teček a sily signalu HNED po startu, ne az
  // za 15-30s od spusteni casovacu. checkWifiBlynk() ma vlastni vnitrni
  // brzdu (30s) pocitanou od "lastWifiRetry", ktera po startu zacina na 0 -
  // takze by se prvnich ~30s sama blokovala, i kdyz uz stav pripojeni znama.
  lastWifiRetry = millis() - wifiRetryInterval - 1; // obejit vnitrni brzdu jen pro tohle prvni volani
  checkWifiBlynk();
  wifiRSSI();

  // Potvrzeni funkcni firmware pro OTA rollback pojistku. Dulezite: neni to
  // podminene pripojenim WiFi/Blynk - staci, ze setup() dobehl az sem bez
  // pádu. Kdyby to bylo vazane na WiFi, bezny vypadek site by mohl vyvolat
  // zbytecny rollback, i kdyz je firmware v poradku.
  esp_ota_mark_app_valid_cancel_rollback();

  Serial.println("Setup hotovo, vstupuji do loop()");
}

// ==========================================================================
// LOOP
// ==========================================================================
void loop()
{
  heltec_loop(); // jen sleduje PRG tlacitko (GPIO0), nesaha na zadny nas pin

  // PRIORITA: dotyk se musi zkontrolovat driv, nez cokoliv sitoveho - kdyby
  // WiFi.status() hlasilo "pripojeno" i kdyz je AP ve skutecnosti nedostupne
  // (bezne pri vypadku, nez si toho ESP32 samo vsimne), Blynk.run() by mohl
  // na TCP timeoutu viset i desitky vterin a displej by po tu dobu vubec
  // nereagoval na dotek.
  handleTouch();

  unsigned long dbgPredTimers = millis();
  runTimers();
  unsigned long dbgPoTimers = millis();

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }
  unsigned long dbgPoBlynk = millis();

  // DOCASNA DIAGNOSTIKA - hledame, proc echo LED reaguje se zpozdenim
  // 1-5s, i kdyz LoRa cidlo je odpojene. Vypise se jen pokud nektera cast
  // trvala podezrele dlouho (>150ms), at to nezaplavi Serial pri normalnim
  // rychlem behu.
  unsigned long dbgTimersMs = dbgPoTimers - dbgPredTimers;
  unsigned long dbgBlynkMs = dbgPoBlynk - dbgPoTimers;
  if (dbgTimersMs > 150 || dbgBlynkMs > 150) {
    Serial.print("DBG POMALY LOOP: runTimers()="); Serial.print(dbgTimersMs);
    Serial.print("ms, Blynk.run()="); Serial.print(dbgBlynkMs); Serial.println("ms");
  }

  if (contagem > 450) {
    litrAktualne++;
    ubyloTyden++;
    ubyloCelkem++;
    contagem -= 450;

    // Detekce neocekavaneho uniku pres waterflow senzor (napr. gravitacni
    // odtok pres selhany/ucpany zpetny ventil - viz vysvetleni v chatu).
    // Postup stejny jako u ultrazvukove detekce poklesu hladiny:
    // notifikace + kratke cuknuti cerpadlem (sdileny mechanismus, viz
    // waterLevelCheck() a kontrolaCuknutiCerpadla()).
    if (!pumpStatus) {
      litrUnikuBezCerpadla++;
      if (litrUnikuBezCerpadla >= LITRU_UNIKU_PRAH) {
        // Bezpecnostni mitigace (cuknuti cerpadlem) probehne VZDY, bez
        // ohledu na nastaveni notifikaci (V20/Notifikace) i na WiFi - jde
        // o fyzickou akci, ne o informativni hlaseni.
        digitalWrite(PIN_PUMP, HIGH);
        cuknutiCerpadlaAktivni = true;
        cuknutiCerpadlaStartMillis = millis();

        // Notifikace se posle VZDY (nezavisle na V20/Notifikace), pokud
        // je WiFi pripojene - bez pripojeni fyzicky neni jak ji odeslat.
        if (WiFi.status() == WL_CONNECTED) {
          Blynk.logEvent("hladina_pokles", "Pozor, waterflow senzor detekoval únik vody bez běhu čerpadla");
        }

        litrUnikuBezCerpadla = 0; // pocitat znovu od nuly pro pripadny dalsi pokracujici unik
      }
    }
  }

  esp_task_wdt_reset(); // "nakrmit" watchdog - potvrdit, ze hlavni smycka bezi normalne

  delay(1); // dulezite: uvolni CPU, jinak FreeRTOS idle task nikdy nedostane prostor
            // a WiFi modem sleep se prakticky neuplatni, i kdyz je zapnuty
}

// ==========================================================================
// DASHBOARD - zakladni rozlozeni obrazovky (uprav si dle libosti)
// ==========================================================================
// ==========================================================================
// IKONA BATERIE -> SIPKA (PROUD) -> LORA ZARIZENI
// Staticka cast (telo baterie, terminaly, obrys sipky, zarizeni) se kresli
// jen jednou v drawDashboard(). Pohybujici se svetle zelene useky uvnitr
// sipky kresli animatePowerFlow(), volana casto (viz timers[]).
// ==========================================================================
#define ARROW_BODY_X 337
#define ARROW_BODY_Y 82
#define ARROW_BODY_W 39
#define ARROW_BODY_H 26

int flowOffset = 0;

void drawBatteryDeviceIcon() {
  // Telo baterie - posunuto o dalsich 5px vlevo (zarizeni zustava na miste)
  lcd.fillRoundRect(248, 63, 77, 63, 5, BATTERYGRAY);
  lcd.drawRoundRect(248, 63, 77, 63, 5, BATTERYDARK);

  // Terminaly - symetricky rozmistene (8px od kazdeho okraje baterie)
  lcd.fillRoundRect(256, 55, 10, 8, 1, BATTERYDARK);
  lcd.fillRoundRect(307, 55, 10, 8, 1, BATTERYDARK);

  // Plus/minus znacky - symetricky rozmistene (10px od kazdeho okraje baterie)
  lcd.fillRoundRect(258, 72, 17, 17, 3, BATTERYRED);
  lcd.fillRoundRect(298, 72, 17, 17, 3, BATTERYDARK);
  lcd.setFont(&fonts::FreeSans9pt7b);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(WHITE);
  lcd.drawString("+", 267, 81);
  lcd.drawString("-", 307, 81);

  // Sipka - posunuta o 5px vlevo a o 5px nahoru
  lcd.fillTriangle(370, 75, 392, 95, 370, 115, ARROWGREEN);
  lcd.fillRect(ARROW_BODY_X, ARROW_BODY_Y, ARROW_BODY_W, ARROW_BODY_H, ARROWGREEN);

  // LoRa zarizeni - zustava na puvodnim miste (nehybe se)
  lcd.fillRoundRect(461, 55, 4, 19, 2, ANTENNAGRAY);
  lcd.fillRoundRect(406, 73, 69, 45, 8, DEVICEBLUE);
  lcd.fillRoundRect(416, 82, 49, 27, 2, BLACK);

  lcd.setTextDatum(TL_DATUM); // vratit vychozi zarovnani pro ostatni funkce
}

// Animuje pohyb svetle zelenych useku uvnitr tela sipky - simuluje proudeni
// proudu od baterie k zarizeni. Kazde volani prekresli jen malou oblast
// (telo sipky), ne cely dashboard.
void animatePowerFlow() {
  if (!backlightOn || pumpStatus) return; // nikdo to stejne nevidi (zhasnuto, nebo prekryto obrazovkou zalevani)

  lcd.fillRect(ARROW_BODY_X, ARROW_BODY_Y, ARROW_BODY_W, ARROW_BODY_H, ARROWGREEN);

  int segW = 7, gap = 5, period = segW + gap;
  int segH = 14;
  int segY = ARROW_BODY_Y + (ARROW_BODY_H - segH) / 2;

  for (int sx = ARROW_BODY_X - period + (flowOffset % period); sx < ARROW_BODY_X + ARROW_BODY_W; sx += period) {
    int x0 = sx;
    int x1 = sx + segW;
    if (x1 <= ARROW_BODY_X) continue;
    if (x0 >= ARROW_BODY_X + ARROW_BODY_W) continue;
    int drawX = (x0 < ARROW_BODY_X) ? ARROW_BODY_X : x0;
    int drawW = ((x1 > ARROW_BODY_X + ARROW_BODY_W) ? (ARROW_BODY_X + ARROW_BODY_W) : x1) - drawX;
    if (drawW > 0) {
      lcd.fillRect(drawX, segY, drawW, segH, ARROWGREEN_LIGHT);
    }
  }
  flowOffset = (flowOffset + 1) % period;
}

void drawDashboard() {
  lcd.fillScreen(BLACK);

  // Font se nastavuje uz tady (drive nez delici cary), aby fontHeight()
  // nize mohl spolehlive spocitat skutecnou vysku textu "Waterbot" -
  // presnejsi nez odhadovat pevny pocet pixelu.
  lcd.setFont(&fonts::FreeSans12pt7b);
  lcd.setTextSize(1);
  int vyskaFontu = lcd.fontHeight();

  // Delici cary
  lcd.drawFastHLine(0, 29, 480, WHITE);
  lcd.drawFastVLine(240, 30, 290, WHITE);
  lcd.drawFastHLine(240, 160, 240, WHITE);
  // Zaklad na y=212 byl ve skutecnosti horni okraj textu "Waterbot"
  // (setCursor+print pouziva horni okraj, ne baseline) - prodlouzeno o
  // skutecnou vysku fontu, aby cara koncila az na spravnem zakladu textu.
  lcd.drawFastVLine(170, 40, 172 + vyskaFontu, WHITE);
  // puvodni cara na x=408 (delila napeti/proud sloupce) odstranena - kolidovala
  // by s novou pozici ikony zarizeni

  // Popisky radku vlevo - jako v originale FreeSans12pt7b (MPPT vyrazeno, nemame solar cidlo)
  lcd.setTextColor(WHITE, BLACK);
  lcd.setCursor(0, 67);  lcd.print("Venkovni");
  lcd.setCursor(0, 96); lcd.print("Kuchyn");
  lcd.setCursor(0, 125); lcd.print("Dolni WC");
  lcd.setCursor(0, 154); lcd.print("Horni WC");
  lcd.setCursor(0, 183); lcd.print("Studna");
  lcd.setCursor(0, 212); lcd.print("Waterbot"); // zde se zobrazuje teplota RTC cipu (Baterie odstranena - nemerime)

  // Sloupcove hlavicky teplota/vlhkost - jako v originale FreeSans9pt7b
  lcd.setFont(&fonts::FreeSans9pt7b);
  lcd.setCursor(111, 38); lcd.print("teplota");
  lcd.setCursor(178, 38); lcd.print("vlhkost");

  // Jednotky (stupne C + kolecko) - u vsech 6 radku, kresli se jen jednou
  int rowY[] = {83, 112, 141, 170, 199, 228};
  for (int i = 0; i < 6; i++) {
    lcd.setCursor(151, rowY[i] - 16);
    lcd.print("C");
    lcd.drawCircle(146, rowY[i] - 13, 2, WHITE);
  }
  for (int i = 0; i < 5; i++) { // procenta jen u radku s vlhkosti (Venkovni..Studna)
    lcd.setCursor(215, rowY[i] - 16);
    lcd.print("%");
  }

  // Vychozi "NA" placeholder pro cidla, dokud nedorazi prvni LoRa packet
  // (i pro Venkovni - BME280 taky nemusi byt jeste pripojene/funkcni)
  lcd.setTextColor(GREY, BLACK);
  for (int i = 0; i < 5; i++) { // Venkovni, Kuchyn, Dolni WC, Horni WC, Studna
    lcd.setCursor(113, rowY[i] - 12);
    lcd.print("NA");
    lcd.setCursor(184, rowY[i] - 12);
    lcd.print("NA");
  }

  // Prava strana - ikona baterie -> sipka (proud) -> LoRa zarizeni, misto textovych popisku
  drawBatteryDeviceIcon();

  // Baterie ikonka (staticky ramecek, hodnota se prekresluje zvlast)
  lcd.drawRoundRect(428, 4, 38, 21, 3, WHITE);
  lcd.drawRoundRect(429, 5, 36, 19, 2, WHITE);
  lcd.drawFastVLine(465, 11, 7, WHITE);
  lcd.drawFastVLine(466, 11, 7, WHITE);
  lcd.drawFastVLine(467, 11, 7, WHITE);
  lcd.fillRect(431, 6, 31, 17, BLACK);
  lcd.setTextColor(WHITE);
  lcd.setCursor(441, 7);
  lcd.print("?");

  // WiFi / Blynk popisky (tecky se prekreslujou v checkWifiBlynk())
  lcd.setTextColor(WHITE, BLACK);
  lcd.setCursor(3, 303);  lcd.print("WiFi");
  lcd.setCursor(75, 303); lcd.print("Blynk");

  // tlacitko cerpadla - text FreeSans9pt7b jako v originale, vycentrovano v tlacitku
  lcd.fillRoundRect(265, 210, 110, 60, 5, GREEN);
  lcd.setTextColor(BLACK, GREEN);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString("ZAPNOUT", 320, 240); // 320,240 = stred tlacitka (265+55, 210+30)
  lcd.setTextDatum(TL_DATUM);

  // sud (hladina)
  lcd.fillRoundRect(400, 190, 70, 91, 3, GREEN);
  lcd.fillRect(403, 190, 64, 88, RED);
}

// ==========================================================================
// SPRAVA PODSVICENI DISPLEJE
// Vext JIZ NENI spojeny s napajenim displeje (viz komentar v setup()) -
// displej je nyni HW napajeny natvrdo primo z 3V3, takze puvodni obavy
// z ESD zpetneho proudu pres LCD cip (kdyz zustavaly piny sbernice aktivne
// rizene i po ztrate napajeni LCD) uz nejsou relevantni pro Vext vypinani.
// Vext se ted pouziva vyhradne pro ultrazvuk/waterflow senzory (viz
// waterLevel() a aplyCmd()).
// ==========================================================================
// ==========================================================================
// Prekresli VSECHNY teploty/vlhkosti (Venkovni/BME280, RTC cip, vsechna 4
// LoRa cidla) aktualne znamymi hodnotami. Volat pri KAZDEM rozsviceni
// podsviceni (wakeDisplay()) - bez tohohle zustavaly hodnoty zastarale
// (napr. "NA"), dokud nedorazil dalsi LoRa packet PRESNE ve chvili, kdy
// uz bylo podsviceni zapnute (viz nalezeny bug).
// ==========================================================================
void prekresliVsechnyTeploty() {
  if (!backlightOn || pumpStatus) return; // nema smysl kreslit na zhasnuty displej, ani prepisovat obrazovku zalevani

  lcd.setFont(&fonts::FreeSans12pt7b);
  lcd.setTextSize(1);
  lcd.setTextColor(WHITE, BLACK);

  // Venkovni (BME280)
  if (!isnan(T_out)) {
    lcd.fillRect(103, 63, 40, 28, BLACK);
    int rt = round(T_out);
    if (rt >= 0 && rt < 10) lcd.setCursor(126, 67);
    else if (rt >= 10) lcd.setCursor(113, 67);
    else if (rt < 0 && rt > -10) lcd.setCursor(118, 67);
    else lcd.setCursor(105, 67);
    lcd.print(rt);
  }
  // vlhkost (BME280) je jiz deklarovana drive v souboru (radek 266), viditelna primo
  if (!isnan(vlhkost)) {
    lcd.fillRect(171, 63, 41, 28, BLACK);
    int rv = round(vlhkost);
    if (rv < 100) lcd.setCursor(186, 67);
    else if (rv == 100) lcd.setCursor(173, 67);
    else lcd.setCursor(199, 67);
    lcd.print(rv);
  }

  // Kuchyn
  if (!isnan(Teplota2)) {
    lcd.fillRect(103, 92, 40, 28, BLACK);
    int rt2 = round(Teplota2);
    if (rt2 >= 0 && rt2 < 10) lcd.setCursor(126, 96);
    else if (rt2 >= 10) lcd.setCursor(113, 96);
    else if (rt2 < 0 && rt2 > -10) lcd.setCursor(118, 96);
    else lcd.setCursor(105, 96);
    lcd.print(rt2);
  }
  if (!isnan(Vlhkost2)) {
    lcd.fillRect(171, 92, 41, 28, BLACK);
    int rv2 = round(Vlhkost2);
    if (rv2 < 100) lcd.setCursor(186, 96);
    else if (rv2 == 100) lcd.setCursor(173, 96);
    else lcd.setCursor(199, 96);
    lcd.print(rv2);
  }

  // Dolni WC
  if (!isnan(Teplota3)) {
    lcd.fillRect(103, 121, 40, 28, BLACK);
    int rt3 = round(Teplota3);
    if (rt3 >= 0 && rt3 < 10) lcd.setCursor(126, 125);
    else if (rt3 >= 10) lcd.setCursor(113, 125);
    else if (rt3 < 0 && rt3 > -10) lcd.setCursor(118, 125);
    else lcd.setCursor(105, 125);
    lcd.print(rt3);
  }
  if (!isnan(Vlhkost3)) {
    lcd.fillRect(171, 121, 41, 28, BLACK);
    int rv3 = round(Vlhkost3);
    if (rv3 < 100) lcd.setCursor(186, 125);
    else if (rv3 == 100) lcd.setCursor(173, 125);
    else lcd.setCursor(199, 125);
    lcd.print(rv3);
  }

  // Horni WC
  if (!isnan(Teplota4)) {
    lcd.fillRect(103, 150, 40, 28, BLACK);
    int rt4 = round(Teplota4);
    if (rt4 >= 0 && rt4 < 10) lcd.setCursor(126, 154);
    else if (rt4 >= 10) lcd.setCursor(113, 154);
    else if (rt4 < 0 && rt4 > -10) lcd.setCursor(118, 154);
    else lcd.setCursor(105, 154);
    lcd.print(rt4);
  }
  if (!isnan(Vlhkost4)) {
    lcd.fillRect(171, 150, 41, 28, BLACK);
    int rv4 = round(Vlhkost4);
    if (rv4 < 100) lcd.setCursor(186, 154);
    else if (rv4 == 100) lcd.setCursor(173, 154);
    else lcd.setCursor(199, 154);
    lcd.print(rv4);
  }

  // Studna
  if (!isnan(Teplota5)) {
    lcd.fillRect(103, 179, 40, 28, BLACK);
    int rt5 = round(Teplota5);
    if (rt5 >= 0 && rt5 < 10) lcd.setCursor(126, 183);
    else if (rt5 >= 10) lcd.setCursor(113, 183);
    else if (rt5 < 0 && rt5 > -10) lcd.setCursor(118, 183);
    else lcd.setCursor(105, 183);
    lcd.print(rt5);
  }
  if (!isnan(Vlhkost5)) {
    lcd.fillRect(171, 179, 41, 28, BLACK);
    int rv5 = round(Vlhkost5);
    if (rv5 < 100) lcd.setCursor(186, 183);
    else if (rv5 == 100) lcd.setCursor(173, 183);
    else lcd.setCursor(199, 183);
    lcd.print(rv5);
  }

  // Teplota RTC cipu
  if (!isnan(T_dev)) {
    lcd.fillRect(103, 208, 40, 28, BLACK);
    int rd = round(T_dev);
    if (rd >= 0 && rd < 10) lcd.setCursor(126, 212);
    else if (rd >= 10) lcd.setCursor(113, 212);
    else if (rd < 0 && rd > -10) lcd.setCursor(118, 212);
    else lcd.setCursor(105, 212);
    lcd.print(rd);
  }
}

void wakeDisplay() {
  digitalWrite(PIN_BACKLIGHT, LOW); // aktivni LOW
  backlightOn = true;
  backLightON = millis();
  prekresliVsechnyTeploty(); // OPRAVA: okamzite obnovit vsechny hodnoty
                              // aktualnimi daty, misto cekani na dalsi packet
}

void sleepDisplay() {
  digitalWrite(PIN_BACKLIGHT, HIGH);
  backlightOn = false;
}

// ==========================================================================
// TOUCH ZPRACOVANI
// ==========================================================================
void handleTouch() {
  if (millis() - lastTouchCheck < touchInterval) return;
  lastTouchCheck = millis();

  TSPoint p = ts.getPoint();
  bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);

  if (pressed) {
    unsigned long timeSinceLastTouch = millis() - backLightON; // zachytit PRED wakeDisplay(), ktera by backLightON prepsala

    wakeDisplay();

    // Prevod surovych ADC hodnot na skutecne pixelove souradnice (viz kalibrace vyse)
    // Osy jsou u tohodle zapojeni prohozene - pro pixelX proto cteme surovy
    // kanal p.y (ne p.x), pro pixelY naopak surovy kanal p.x
    int pixelX = map(p.y, TOUCH_CAL_X1, TOUCH_CAL_X2, 20, 460) + 10; // korekce +10px, doladeno vizualnim testem
    int pixelY = map(p.x, TOUCH_CAL_Y1, TOUCH_CAL_Y2, 20, 300);

    // Obe hranice jsou dulezite: dolni (300ms) odfiltruje opakovane zaznamenani
    // stejneho souvisleho doteku; horni (backLightTime = 15s) zajisti, ze prvni
    // "budici" tuknuti po delsi odmlce tlacitko NEAKTIVUJE - jen kdyz prijde
    // DALSI tuknuti behem 15s od toho prvniho, teprve pak se tlacitko povazuje
    // za zamerne stisknute.
    if (timeSinceLastTouch > doubletouch && timeSinceLastTouch < backLightTime) {
      if (pumpStatus) {
        // Cerpadlo bezi - obrazovka zalevani prekryva celou obrazovku,
        // takze puvodni tlacitko cerpadla (265-375,210-270) uz neni videt.
        // Aktivni jsou misto toho: STOP tlacitko, minus a plus kolecko.
        if (pixelX > 310 && pixelX < 460 && pixelY > 255 && pixelY < 300) {
          // STOP tlacitko (fillRoundRect(310, 255, 150, 45, ...))
          pumpStatus = 0;
          aplyCmd();
        } else if (pixelX > 23 && pixelX < 67 && pixelY > 163 && pixelY < 207) {
          // minus kolecko (stred 45,185, polomer 22)
          if (litryLimitDocasne > 1) {
            litryLimitDocasne--;
            aktualizujObrazovkuZalevani();
          }
        } else if (pixelX > 413 && pixelX < 457 && pixelY > 163 && pixelY < 207) {
          // plus kolecko (stred 435,185, polomer 22)
          if (litryLimitDocasne < 50) {
            litryLimitDocasne++;
            aktualizujObrazovkuZalevani();
          }
        }
      } else {
        // tlacitko cerpadla - skutecne pixelove souradnice (265-375, 210-270),
        // presne tam, kde je nakreslene (viz fillRoundRect(265, 210, 110, 60, ...))
        if (pixelX > 265 && pixelX < 375 && pixelY > 210 && pixelY < 270) {
          pumpStatus = !pumpStatus;
          if (pumpStatus) pumpStartTimeMillis = millis();
          aplyCmd();
        }

        // Ikonka zarovky - rucni pripojeni/odpojeni solar regulatoru
        // (svetlo v zahradnim domku), viz ZAROVKA_X/Y/W/H
        if (pixelX > ZAROVKA_X && pixelX < ZAROVKA_X + ZAROVKA_W &&
            pixelY > ZAROVKA_Y && pixelY < ZAROVKA_Y + ZAROVKA_H) {
          bool pripojeno = releBaterieSepnuto && releSolarSepnuto;

          vypniAutomatikuRele(); // rucni zasah vypina automatiku + posle notifikaci

          if (pripojeno) {
            spustOdpojovaciSekvenciRele(true);
          } else {
            spustPripojovaciSekvenciRele(true);
          }

          // Okamzita vizualni odezva - ukazat CILOVY stav hned po tuknuti,
          // nez skutecne dobehne cela sekvence (2s mezi rele + 5s podrzeni
          // Vext na konci, cca 7s celkem) - jinak to pusobilo, jako by
          // dotyk vubec nezabral.
          kresliIkonuZarovky(!pripojeno);
        }
      }
    }
  }

  if (!pumpStatus && millis() - backLightON > backLightTime) {
    sleepDisplay();
  }
}

// ==========================================================================
// OBRAZOVKA ZALEVANI - zobrazi se pres celou obrazovku pokazde, kdyz bezi
// cerpadlo (touch, Blynk, i casovac). Nahrazuje puvodni maly popisek
// "CERPADLO"/"VYPNOUT" - ten uz proto neni nikdy videt, takze ho dal
// nevolame. Staticke prvky (titulek, kolecka +/-, drazka slideru, popisky)
// se kresli jen jednou v kresliObrazovkuZalevani(); dynamicke hodnoty
// (statusbar, "Nastaveno: X l", pozice slideru, "Zalito: X l") se
// prubezne obnovuji v aktualizujObrazovkuZalevani().
// ==========================================================================
void kresliObrazovkuZalevani() {
  lcd.fillScreen(BLACK);

  lcd.setFont(&fonts::FreeSans18pt7b); // nativni vetsi velikost, ne skalovana - skalovani (setTextSize>1)
                                       // zpusobovalo zubatost, nativni velikost fontu by mela byt hladka
  lcd.setTextColor(WHITE, BLACK);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextSize(1);
  lcd.drawString("Probiha zalevani", 240, 30); // bez diakritiky - tenhle font ji neumi
  lcd.setFont(&fonts::FreeSans12pt7b); // zpet na standardni font pro zbytek obrazovky

  // Statusbar - leva staticka znacka "0 l"
  lcd.setTextDatum(TL_DATUM);
  lcd.setCursor(20, 60);
  lcd.print("0 l");

  // Statusbar - prazdne pozadi (vyplnena cast se dokresli dynamicky)
  lcd.fillRect(20, 85, 440, 20, lcd.color565(42, 42, 42));

  // Minus/plus kolecka a symboly (staticke, nemeni se behem cyklu).
  // Kresleno rucne jako obdelniky (ne fontem) - umoznuje presnou kontrolu
  // tvaru, konkretne asymetrickou svislou cast znaku "+" (viz nize).
  lcd.fillCircle(45, 185, 22, RED);
  lcd.fillCircle(435, 185, 22, GREEN);
  {
    int symCy = 185; // stred znaku - presne na stredu kolecka (posunuto o 1px dolu z puvodnich 184)
    int tloustka = 4;
    int polovinaDelky = 8;

    // Minus (45,185) - jen vodorovny pruh
    lcd.fillRect(45 - polovinaDelky, symCy - tloustka / 2, polovinaDelky * 2, tloustka, WHITE);

    // Plus (435,185) - vodorovny pruh + svisly pruh
    lcd.fillRect(435 - polovinaDelky, symCy - tloustka / 2, polovinaDelky * 2, tloustka, WHITE);
    // Svisla cast: nahoru stejnych 8px jako vodorovna, dolu 7px
    // (prodlouzeno o 1px oproti predchozimu, porad o 1px kratsi nez
    // symetricky tvar by mel)
    lcd.fillRect(435 - tloustka / 2, symCy - polovinaDelky, tloustka, polovinaDelky + (polovinaDelky - 1), WHITE);
  }

  // Slider - drazka (staticka cast, "kulicka" se kresli dynamicky)
  lcd.fillRoundRect(75, 182, 330, 6, 3, lcd.color565(42, 42, 42));

  // "Zalito" popisek (staticky text, hodnota vedle nej dynamicka)
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(WHITE, BLACK);
  lcd.setCursor(20, 255);
  lcd.print("Zalito");

  // STOP tlacitko (staticke, nemeni se behem cyklu)
  lcd.fillRoundRect(310, 255, 150, 45, 8, RED);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(WHITE, RED);
  lcd.drawString("Stop", 385, 278); // nativni velikost FreeSans12pt7b, bez skalovani (skalovani zpusobovalo zubatost)
  lcd.setTextDatum(TL_DATUM);

  aktualizujObrazovkuZalevani(); // hned dokreslit i dynamicke casti (0 l zalito na startu atd.)
}

void aktualizujObrazovkuZalevani() {
  lcd.setFont(&fonts::FreeSans12pt7b);
  lcd.setTextSize(1);

  // Cilova hodnota na pravem konci statusbaru
  lcd.fillRect(370, 55, 90, 25, BLACK);
  char cilBuf[8];
  sprintf(cilBuf, "%d l", litryLimitDocasne);
  lcd.setTextDatum(TR_DATUM);
  lcd.setTextColor(WHITE, BLACK);
  lcd.drawString(cilBuf, 460, 60);

  // Vyplnena cast statusbaru (0 az litrAktualne, vuci litryLimitDocasne)
  int sirkaVyplne = 0;
  if (litryLimitDocasne > 0) {
    sirkaVyplne = (int)(440.0 * litrAktualne / litryLimitDocasne);
    if (sirkaVyplne > 440) sirkaVyplne = 440;
  }
  lcd.fillRect(20, 85, sirkaVyplne, 20, lcd.color565(4, 220, 237));
  lcd.fillRect(20 + sirkaVyplne, 85, 440 - sirkaVyplne, 20, lcd.color565(42, 42, 42));

  // "Nastaveno: X l"
  lcd.fillRect(140, 125, 200, 30, BLACK);
  char nastavenoBuf[20];
  sprintf(nastavenoBuf, "Nastaveno: %d l", litryLimitDocasne);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(WHITE, BLACK);
  lcd.drawString(nastavenoBuf, 240, 140);

  // Poloha "kulicky" na slideru (rozsah 1-50, stejny jako V9 v Blynku)
  // DULEZITE: kolecko ma polomer 8px (vyska 16px), ale drazka je jen 6px -
  // pred prekreslenim je nutne smazat CELOU vysku kolecka (ne jen drazku),
  // jinak zustavaji "duchy" nad a pod drazkou z predchozi pozice.
  lcd.fillRect(75 - 8, 185 - 8, 330 + 16, 17, BLACK); // smazat cely pruh vc. presahu kolecka (+1px dolu navic, aby nezustavala linka duchu)
  lcd.fillRoundRect(75, 182, 330, 6, 3, lcd.color565(42, 42, 42)); // znovu nakreslit tenkou drazku
  int thumbX = 75 + (int)(330.0 * (litryLimitDocasne - 1) / (50 - 1));
  lcd.fillCircle(thumbX, 185, 8, WHITE);

  // "Zalito: X l"
  lcd.fillRect(20, 275, 150, 35, BLACK);
  char zalitoBuf[16];
  sprintf(zalitoBuf, "%d l", litrAktualne);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(WHITE, BLACK);
  lcd.drawString(zalitoBuf, 20, 280);

  lcd.setTextDatum(TL_DATUM);
}

// ==========================================================================
// CERPADLO - aplikace prikazu
// ==========================================================================
// Popisek "CERPADLO" pod tlacitkem - VOLA SE UZ JEN Z HISTORICKYCH DUVODU,
// prakticky ale neni nikdy videt (viz kresliObrazovkuZalevani() vyse, ktera
// prekryva celou obrazovku pokazde, kdyz cerpadlo bezi).
// Zaobleny obdelnik presahuje text o 6px nahore i po stranach.
void updatePumpLabel() {
  int centerX = 320;   // stred pod tlacitkem (265+55)
  int centerY = 294;   // puvodnich 288 + 6px posun celeho popisku dolu

  if (pumpStatus) {
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextSize(1);
    int textW = lcd.textWidth("CERPADLO");
    int textH = 16; // priblizna vyska 9pt fontu
    int boxW = textW + 12; // 6px presah po kazde strane
    int boxH = textH + 12; // 6px presah nahore i dole
    lcd.fillRoundRect(centerX - boxW / 2, centerY - boxH / 2, boxW, boxH, 4, RED);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(WHITE, RED);
    lcd.drawString("CERPADLO", centerX, centerY + 2); // text o 2px nize v ramci obdelniku
    lcd.setTextDatum(TL_DATUM);
  } else {
    // smazat cely prostor, kde by napis s podbarvenim mohl byt
    lcd.fillRect(265, 276, 110, 36, BLACK);
  }
}

void aplyCmd() {
  if (pumpStatus) {
    digitalWrite(PIN_PUMP, HIGH);
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V8, HIGH);
    pumpLED.on();

    // Reset pocitadla uniku - prutok od tohoto okamziku uz je legitimni
    // (cerpadlo bezi), nema smysl ho pocitat jako podezrely unik.
    litrUnikuBezCerpadla = 0;

    // Vext ON - waterflow senzor je nyni napajeny z Vext, ne z trvaleho
    // 3V3. Musi zustat zapnuty po CELOU dobu behu cerpadla (na rozdil od
    // ultrazvuku ve waterLevel(), ktery se meri jen chvilkove).
    digitalWrite(PIN_VEXT, LOW);

    wakeDisplay(); // probudit displej (jen podsviceni, displej samotny je na 3V3)

    litryLimitDocasne = litryLimit; // docasna kopie PRO TENTO cyklus, neuklada se
    kresliObrazovkuZalevani();
  } else {
    digitalWrite(PIN_PUMP, LOW);
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V8, LOW);
    pumpLED.off();

    // Reset casovace podsviceni - behem behu cerpadla se auto-vypnuti
    // ignorovalo (viz !pumpStatus podminky v handleTouch()/displayTime()),
    // takze backLightON je "stary" (z okamziku spusteni cerpadla). Bez
    // resetu by podsviceni zhaslo prakticky hned po zastaveni. Timhle
    // zajistime plnych 15s svicani POCITANO OD TOHOTO OKAMZIKU.
    backLightON = millis();

    // Vext OFF - waterflow senzor uz neni potreba. moznaVypniVext() navic
    // zohlednuje, jestli zrovna nebezi mereni hladiny nebo sekvence rele -
    // jednovlaknove provadeni, zadna skutecna soubeznost, jen sdileny zdroj.
    moznaVypniVext();

    {
      DateTime now = rtc.now();
      char buf[18];
      sprintf(buf, "%02d.%02d.%04d %02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute());
      lastPumpUsed = String(buf);
    }
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V13, lastPumpUsed);
    lastUsedAmount = litrAktualne;
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V14, lastUsedAmount);
    // ubyloTyden/ubyloCelkem se prubezne pocitaji uz v loop() (waterflow
    // senzor), ale do Blynku se doposud propisovaly jen pri resyncAllToBlynk(),
    // tydennim resetu, nebo rucnim resetu - chybelo tu, presne podle
    // puvodniho zdrojaku, propsani hned po dobehnuti zalevani.
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V26, ubyloTyden);
      Blynk.virtualWrite(V47, ubyloCelkem);
    }
    litrAktualne = 0;
    pauzaOdectuStartMillis = millis();
    pauzaOdectu = 1;

    // Kreslit jen pokud je displej logicky aktivni (nyni vzdy true, viz
    // deklarace displayLogicOn - displej ma vlastni napajeni z 3V3)
    if (displayLogicOn) {
      // Navrat na normalni dashboard - obrazovka zalevani prekryvala UPLNE
      // vse, takze je potreba kompletni obnova (ne jen maleho tlacitka jako
      // drive). prekresliVsechnyTeploty() navic vynuceně obnovi i
      // teplotu/vlhkost/RTC-teplotu bez ohledu na hysterezi (viz drivejsi
      // opravu "NA" bugu) - jinak by po fillScreen() v drawDashboard()
      // zustaly tyhle hodnoty prazdne, dokud by se nezmenily aspon o 0.5.
      drawDashboard();
      prekresliVsechnyTeploty();

      // Datum ma stejny typ hysterezni pojistky (kresli se jen pri zmene
      // dne) - bez tohoto resetu by po fillScreen() zustalo prazdne, dokud
      // by se den skutecne nezmenil. Vynutit prekresleni tim, ze "posledni
      // znamy" den nastavime na neco, co se zarucene lisi od aktualniho.
      lastDOW = "";
      displayTime();

      getTemperature();
      waterLevel();

      // Ikonka zarovky - drawDashboard() vyse vymazal celou obrazovku,
      // takze ji treba obnovit podle aktualniho stavu rele
      kresliIkonuZarovky(releBaterieSepnuto && releSolarSepnuto);

      // WiFi/Blynk tecky a sila signalu - stejny trik jako v setup() pro
      // obejiti vnitrni brzdy (10s/15s), aby se zobrazily hned, ne az pri
      // pristim pravidelnem tiknuti casovace
      lastWifiRetry = millis() - wifiRetryInterval - 1;
      checkWifiBlynk();
      wifiRSSI();

      readINA226();
    }

    EEPROMWriteInt(154, ubyloTyden);
    EEPROMWriteInt(162, ubyloCelkem);
    EEPROM.write(14, lastUsedAmount);
    EEPROM.commit();
  }
}

// ==========================================================================
// KONTROLA BEHU CERPADLA (zachranna brzda + limit litru)
// ==========================================================================
void pumpStatusCheck() {
  handleTouch(); // prednostni kontrola doteku

  if (pumpStatus) {
    lastUsedAmount = litrAktualne;
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V14, lastUsedAmount);
    if (displayLogicOn) aktualizujObrazovkuZalevani(); // prubezne obnovovat behem zalevani
    if (((millis() - pumpStartTimeMillis) > maxPumpOn) || litrAktualne >= litryLimitDocasne) {
      pumpStatus = 0;
      aplyCmd();
    }
  } else {
    if (((millis() - pauzaOdectuStartMillis) > pauzaOdectuLimit) && pauzaOdectu == 1) {
      pauzaOdectu = 0;
      lastWaterLevel = celkovyObjem;
      lastWaterLevelNotify = celkovyObjem;
    }
  }
}

// ==========================================================================
// CASOVACE ZAVLAZOVANI
// ==========================================================================
void timerCheck() {
  handleTouch(); // prednostni kontrola doteku

  DateTime now = rtc.now();

  if (Timer1 && now.hour() == startHour1 && now.minute() == startMinute1) {
    if (pauza1) {
      pumpStartTimeMillis = millis();
      pumpStatus = 1;
      aplyCmd();
      pauza1 = 0;
      if (Notifikace && WiFi.status() == WL_CONNECTED) Blynk.logEvent("pump_on", "Spustilo se čerpadlo (Timer1)");
    }
  } else pauza1 = 1;

  if (Timer2 && now.hour() == startHour2 && now.minute() == startMinute2) {
    if (pauza2) {
      pumpStartTimeMillis = millis();
      pumpStatus = 1;
      aplyCmd();
      pauza2 = 0;
      if (Notifikace && WiFi.status() == WL_CONNECTED) Blynk.logEvent("pump_on", "Spustilo se čerpadlo (Timer2)");
    }
  } else pauza2 = 1;

  if (Timer3 && now.hour() == startHour3 && now.minute() == startMinute3) {
    if (pauza3) {
      pumpStartTimeMillis = millis();
      pumpStatus = 1;
      aplyCmd();
      pauza3 = 0;
      if (Notifikace && WiFi.status() == WL_CONNECTED) Blynk.logEvent("pump_on", "Spustilo se čerpadlo (Timer3)");
    }
  } else pauza3 = 1;
}

// ==========================================================================
// ZOBRAZENI CASU
// ==========================================================================
// ==========================================================================
// ZALOZNI (RUCNI) PRECHOD LETNI/ZIMNI CAS - pro pripad velmi dlouheho vypadku
// WiFi/NTP synchronizace. Prepsano cisteji nez original: mista krehkeho
// parsovani textoveho retezce data pouzivame primo pole DateTime objektu.
// Za normalnich okolnosti se o tohle postara NTP pri kazdem restartu/OTA.
// ==========================================================================
boolean blokace_stridani_casu = false;

void checkManualDstFallback(DateTime now) {
  if (now.dayOfTheWeek() != 0) { // RTClib: 0 = Nedele
    blokace_stridani_casu = false;
    return;
  }

  // Posledni nedele v mesici - pokud by pridani 7 dni prekrocilo pocet dni v mesici
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int dim = daysInMonth[now.month() - 1];
  bool isLeap = (now.year() % 4 == 0 && now.year() % 100 != 0) || (now.year() % 400 == 0);
  if (now.month() == 2 && isLeap) dim = 29;
  bool isLastSunday = (now.day() + 7) > dim;

  if (!isLastSunday || blokace_stridani_casu) return;

  if (now.month() == 3 && now.hour() == 2) {
    // Posledni nedele v breznu, 2:00 -> posun na letni cas (3:00)
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), 3, now.minute(), now.second()));
    blokace_stridani_casu = true;
    Serial.println("DST zaloha: posunuji cas o hodinu dopredu (letni cas)");
    if (WiFi.status() == WL_CONNECTED) logToTerminal("RTC - posouvam cas o 1 hodinu dopredu (letni cas)");
  } else if (now.month() == 10 && now.hour() == 3) {
    // Posledni nedele v rijnu, 3:00 -> posun na zimni cas (2:00)
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), 2, now.minute(), now.second()));
    blokace_stridani_casu = true;
    Serial.println("DST zaloha: posunuji cas o hodinu dozadu (zimni cas)");
    if (WiFi.status() == WL_CONNECTED) logToTerminal("RTC - posouvam cas o 1 hodinu dozadu (zimni cas)");
  }
}

void displayTime() {
  DateTime now = rtc.now();
  actualDOW = dowNames[(now.dayOfTheWeek() + 6) % 7]; // preskladani 0=Ne...6=So na 0=Po...6=Ne

  checkManualDstFallback(now); // zaloha letni/zimni cas pro pripad dlouheho vypadku NTP

  if (displayLogicOn && !pumpStatus) { // zadny smysl kreslit, kdyz bezi zalevani (jina obrazovka)
    lcd.setFont(&fonts::FreeSans12pt7b); // stejny font jako v puvodnim kodu pro den/datum/cas
    lcd.setTextSize(1);   // dulezite: setTextSize(2) z jinych casti kodu by font znovu nafouklo a rozpixelovalo
    lcd.setTextColor(WHITE, BLACK); // pozadi nutne - jinak by uzsi novy znak neprekryl sirsi stary (ghosting)

    if (actualDOW != lastDOW) {
      // Den v tydnu + datum - stejne misto jako v originale (spolecny box, prekresluje se jen pri zmene dne)
      lcd.fillRect(0, 0, 240, 26, BLACK);
      lcd.setCursor(0, 3);
      lcd.print(actualDOW);
      lcd.setCursor(100, 3);
      char buf[11];
      sprintf(buf, "%02d.%02d.%04d", now.day(), now.month(), now.year());
      lcd.print(buf);
      lastDOW = actualDOW;
    }

    // Cas vcetne vterin - hned za datem (original mel jen HH:MM, my navic pridavame vteriny)
    lcd.fillRect(241, 0, 100, 26, BLACK);
    lcd.setCursor(241, 3);
    char timeBuf[9];
    sprintf(timeBuf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    lcd.print(timeBuf);
  }

  if (!pumpStatus && backlightOn && (millis() - backLightON) > backLightTime) {
    sleepDisplay();
  }
}

// ==========================================================================
// BME280 - VENKOVNI TEPLOTA/VLHKOST + TEPLOTA RTC CIPU
// ==========================================================================
void getTemperature() {
  handleTouch(); // prednostni kontrola doteku pred moznymi Blynk volanimi nize

  T_out = bme.readTemperature();
  vlhkost = bme.readHumidity();
  float tlak = bme.readPressure() / 100.0F; // Pa -> hPa
  T_dev = rtc.getTemperature();

  if (TEST_MODE_RANDOM_SENSOR_VALUES) {
    T_out = random(-200, 401) / 10.0;   // -20.0 az 40.0
    vlhkost = random(0, 1001) / 10.0;   // 0.0 az 100.0
    T_dev = random(-200, 401) / 10.0;
  }

  if (backlightOn && !pumpStatus) {
    lcd.setFont(&fonts::FreeSans12pt7b);
    lcd.setTextSize(1);
    lcd.setTextColor(WHITE, BLACK);
  }

  if (T_out <= 125 && vlhkost <= 100) {
    if (abs(T_out - T_out_mem) >= 0.5) {
      if (backlightOn && !pumpStatus) {
        lcd.fillRect(103, 63, 40, 28, BLACK);
        int rt = round(T_out);
        if (rt >= 0 && rt < 10) lcd.setCursor(126, 67);
        else if (rt >= 10) lcd.setCursor(113, 67);
        else if (rt < 0 && rt > -10) lcd.setCursor(118, 67);
        else lcd.setCursor(105, 67);
        lcd.print(rt);
        // OPRAVA: T_out_mem se aktualizuje jen kdyz se SKUTECNE nakreslilo -
        // driv se nastavovalo vzdy (i s vypnutym podsvicenim), coz trvale
        // zablokovalo jakekoliv budouci prekresleni (viz nalezeny bug).
        T_out_mem = T_out;
      }
      if (WiFi.status() == WL_CONNECTED) {
        Blynk.virtualWrite(V1, T_out);
        char toutBuf[16];
        sprintf(toutBuf, "Tout %d°C", (int)round(T_out));
        Blynk.virtualWrite(V69, toutBuf);
      }
    }
    if (abs(vlhkost - vlhkost_mem) >= 5) {
      if (backlightOn && !pumpStatus) {
        lcd.fillRect(171, 63, 41, 28, BLACK);
        int rv = round(vlhkost);
        if (rv < 100) lcd.setCursor(186, 67);
        else if (rv == 100) lcd.setCursor(173, 67);
        else lcd.setCursor(199, 67);
        lcd.print(rv);
        vlhkost_mem = vlhkost; // OPRAVA: jen kdyz se skutecne nakreslilo
      }
      if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V2, vlhkost);
    }
  } else if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V1, "ERR");
    Blynk.virtualWrite(V2, "ERR");
  }

  // Tlak (BME280) - jen do Blynku, na displeji se nezobrazuje.
  // Prepocet z absolutniho na relativni (na hladinu more), teplotne
  // kompenzovany vzorec: P0 = P * (1 - (0.0065*h)/(T + 0.0065*h + 273.15))^-5.257
  // Kontrola T_out > -50 && <= 125 navic - vstupuje primo do vzorce, takze
  // by neplatna (NaN) hodnota nenapadne zkazila i jinak platny tlak.
  if (tlak > 300 && tlak < 1100 && T_out > -50 && T_out <= 125 && WiFi.status() == WL_CONNECTED) {
    float tlakRelativni = tlak * pow(1.0 - (0.0065 * NADMORSKA_VYSKA_M) / (T_out + 0.0065 * NADMORSKA_VYSKA_M + 273.15), -5.257);
    Blynk.virtualWrite(V85, tlakRelativni);
  }

  if (abs(T_dev - T_dev_mem) >= 0.5) {
    if (backlightOn && !pumpStatus) {
      lcd.fillRect(103, 208, 40, 28, BLACK);
      int rd = round(T_dev);
      if (rd >= 0 && rd < 10) lcd.setCursor(126, 212);
      else if (rd >= 10) lcd.setCursor(113, 212);
      else if (rd < 0 && rd > -10) lcd.setCursor(118, 212);
      else lcd.setCursor(105, 212);
      lcd.print(rd);
      T_dev_mem = T_dev; // OPRAVA: jen kdyz se skutecne nakreslilo
    }
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V40, T_dev);
  }
}

// ==========================================================================
// INA226 - PROUD A NAPETI
// ==========================================================================
// ==========================================================================
// ODHAD STAVU NABITI (SOC) Z NAPETI - piecewise-linearni interpolace.
// Horni body (60-100%) jsou z obecne standardni tabulky pro 12V Pb akumulatory.
// Spodni bod (0% = 12.12V) je z datasheetu OTL26-12: vypinaci napeti vyrobce
// 2.02V/clanek pri vybijecim proudu <0.1C (nase centrala odebira jen jednotky
// stovek mA, hluboko pod 0.1C = 2.6A pro tuhle 26Ah baterii).
// POZOR: plati jen pro KLIDOVE napeti (bez nabijeni) - viz pouziti nize.
// ==========================================================================
// ==========================================================================
// PERZISTENCE KLOUZAVYCH PRUMERU DO EEPROM
// RTC_DATA_ATTR prezije jen deep sleep, ne skutecny restart (OTA, watchdog,
// vypadek napajeni) - proto se prumery navic ukladaji do EEPROM, aby po
// restartu nebylo nutne zacinat "od nuly".
// ==========================================================================
void saveDailyChargeToEEPROM() {
  EEPROM.put(EEPROM_ADDR_EMA_1H, emaCurrentA_1h);
  EEPROM.put(EEPROM_ADDR_EMA_24H, emaCurrentA_24h);
  EEPROM.write(EEPROM_ADDR_LOW_POWER, lastKnownLowPowerMode ? 1 : 0);
  EEPROM.commit();
}

void loadDailyChargeFromEEPROM() {
  float saved1h = 0, saved24h = 0;
  EEPROM.get(EEPROM_ADDR_EMA_1H, saved1h);
  EEPROM.get(EEPROM_ADDR_EMA_24H, saved24h);
  byte savedMode = EEPROM.read(EEPROM_ADDR_LOW_POWER);

  // Sanitni kontrola - nezapsana flash pamet (0xFF bajty) da nesmyslny/NaN float
  bool valid1h = !isnan(saved1h) && saved1h >= 0 && saved1h < 100;
  bool valid24h = !isnan(saved24h) && saved24h >= 0 && saved24h < 100;

  if (valid1h && valid24h) {
    emaCurrentA_1h = saved1h;
    emaCurrentA_24h = saved24h;
    lastKnownLowPowerMode = (savedMode == 1);
    Serial.print("Nacteno z EEPROM: EMA 1h=");
    Serial.print(emaCurrentA_1h, 4);
    Serial.print(" A, EMA 24h=");
    Serial.print(emaCurrentA_24h, 4);
    Serial.println(" A - pokracuji bez cekani od nuly");
  } else {
    Serial.println("EEPROM neobsahuje platna data - klouzave prumery zacinaji od nuly");
  }
}

// Odhad stavu nabiti LiPol 1S clanku (cidla) - UPLNE JINA krivka nez olovena
// baterie centraly (estimateBatterySOC nize) - nezamenovat, napeti se nedaji
// srovnavat primo (LiPol 1S: ~4.2V plny, ~3.3-3.0V prazdny)
float estimateLipolPercent(float voltage) {
  struct SocPoint { float v; float soc; };
  static const SocPoint table[] = {
    {4.20, 100},
    {4.10, 95},
    {4.00, 87},
    {3.90, 77},
    {3.80, 65},
    {3.70, 50},
    {3.60, 25},
    {3.50, 12},
    {3.40, 5},
    {3.30, 2},
    {3.00, 0}
  };
  const int n = sizeof(table) / sizeof(table[0]);

  if (voltage >= table[0].v) return 100.0;
  if (voltage <= table[n - 1].v) return 0.0;

  for (int i = 0; i < n - 1; i++) {
    if (voltage <= table[i].v && voltage >= table[i + 1].v) {
      float ratio = (voltage - table[i + 1].v) / (table[i].v - table[i + 1].v);
      return table[i + 1].soc + ratio * (table[i].soc - table[i + 1].soc);
    }
  }
  return 0.0;
}

float estimateBatterySOC(float voltage) {
  struct SocPoint { float v; float soc; };
  static const SocPoint table[] = {
    {12.70, 100},
    {12.50, 90},
    {12.42, 80},
    {12.32, 70},
    {12.20, 60},
    {12.12, 0}
  };
  const int n = sizeof(table) / sizeof(table[0]);

  if (voltage >= table[0].v) return 100.0;
  if (voltage <= table[n - 1].v) return 0.0;

  for (int i = 0; i < n - 1; i++) {
    if (voltage <= table[i].v && voltage >= table[i + 1].v) {
      float ratio = (voltage - table[i + 1].v) / (table[i].v - table[i + 1].v);
      return table[i + 1].soc + ratio * (table[i].soc - table[i + 1].soc);
    }
  }
  return 0.0; // pojistka, nemelo by nastat
}

// Blynk Icon widget (V62) - 0-7 = stav nabiti baterie, 8 = probiha nabijeni.
// Puvodne prah 13.0V, zmeneno na 13.6V (stejny prah jako RELE_NAPETI_PRAH) -
// zjisteno, ze baterie si i PO ODPOJENI od solar regulatoru drzi napeti
// kolem 13.1V jeste znatelnou dobu, coz by s puvodnim prahem 13.0V
// nespravne ukazovalo "nabijeni", i kdyz uz aktivni nabijeni davno skoncilo.
#define CHARGING_VOLTAGE_THRESHOLD 13.6

void updateBlynkBatteryIcon(float voltage) {
  if (WiFi.status() != WL_CONNECTED) return;

  float soc = estimateBatterySOC(voltage); // pocita se vzdy, i behem nabijeni (pro V65)
  bool nabiji = (voltage > CHARGING_VOLTAGE_THRESHOLD);

  // V65 je nyni String (drive Integer) - behem nabijeni ukazuje symbol
  // blesku misto procent, protoze SOC z napeti behem nabijeni nedava smysl
  // (napeti je umele vytazene regulatorem, viz komentar vyse)
  if (nabiji) {
    Blynk.virtualWrite(V65, "⚡");
  } else {
    Blynk.virtualWrite(V65, String((int)round(soc)));
  }

  // V86 - stejna informace, ale VZDY jako Integer (pro widget v zahlavi
  // Blynk appky, ktery string neumi). Behem nabijeni napevno 100, aby
  // ikonka v zahlavi ukazovala "plno", stejne jako blesk u V65.
  Blynk.virtualWrite(V86, nabiji ? 100 : (int)round(soc));

  if (nabiji) {
    Blynk.virtualWrite(V62, 8);
    Blynk.setProperty(V62, "label", "Solar");
    Serial.println("V62 -> icon=8, label=\"Solar\"");
  } else {
    int icon = constrain((int)round(soc / 100.0 * 7.0), 0, 7);
    Blynk.virtualWrite(V62, icon);
    char label[8];
    sprintf(label, "%d%%", (int)round(soc));
    Blynk.setProperty(V62, "label", label);
    Serial.print("V62 -> icon="); Serial.print(icon);
    Serial.print(", label=\""); Serial.print(label); Serial.println("\"");
  }
}

void readINA226() {


  ina_busVoltage = ina226.getBusVoltage();
  ina_current_mA = -ina226.getCurrent_mA(); // korekce obracene polarity mereni proudu (shunt zapojen opacne)
  ina_power_mW = ina_busVoltage * ina_current_mA;
  float power_W = ina_power_mW / 1000.0;

  // ---------- Integrace energie (Wh) - jen pro interni Serial log, beze zmeny ----------
  unsigned long nowMillis = millis();
  float elapsedHours = 0;
  if (lastINA226Millis != 0) {
    elapsedHours = (nowMillis - lastINA226Millis) / 3600000.0;
    // pojistka: pokud je interval nesmyslne velky (napr. po restartu/spanku, kdy millis() zacne od 0),
    // tento prirustek preskocime, aby nam nezkreslil prumer
    if (elapsedHours > 0 && elapsedHours < 0.1) {
      dailyEnergyWh += power_W * elapsedHours;
    } else {
      elapsedHours = 0; // mimo rozumny rozsah - neaplikovat ani na klouzave prumery nize
    }
  }
  lastINA226Millis = nowMillis;

  // ---------- Detekce zmeny rezimu (plny/usporny) - vynulovani klouzavych prumeru ----------
  if (lowPowerMode != lastKnownLowPowerMode) {
    emaCurrentA_1h = 0;
    emaCurrentA_24h = 0;
    lastKnownLowPowerMode = lowPowerMode;
    Serial.println("Zmena rezimu (plny/usporny) - klouzave prumery spotreby vynulovany");
  }

  // ---------- Klouzave prumery proudu (EMA) - "jakoby" posledni hodina / poslednich 24h ----------
  // alpha se pocita dynamicky z aktualne uplynuleho casu, aby casova konstanta
  // zustala spravna i pri kolisajicim intervalu mezi mereninimi
  if (elapsedHours > 0) {
    float currentA = ina_current_mA / 1000.0;
    float elapsedSeconds = elapsedHours * 3600.0;
    float alpha1h = 1.0 - exp(-elapsedSeconds / 3600.0);
    float alpha24h = 1.0 - exp(-elapsedSeconds / 86400.0);
    emaCurrentA_1h = emaCurrentA_1h * (1.0 - alpha1h) + currentA * alpha1h;
    emaCurrentA_24h = emaCurrentA_24h * (1.0 - alpha24h) + currentA * alpha24h;
  }

  // ---------- Periodicke ulozeni klouzavych prumeru do EEPROM (max 1x za hodinu) ----------
  if (nowMillis - lastDailyChargeEepromSave > EEPROM_SAVE_INTERVAL_MS) {
    saveDailyChargeToEEPROM();
    lastDailyChargeEepromSave = nowMillis;
  }

  // ---------- Reset Wh akumulace o pulnoci - jen pro interni Serial log ----------
  DateTime nowDt = rtc.now();
  int32_t today = nowDt.unixtime() / 86400L;
  if (today != dailyEnergyDay) {
    if (dailyEnergyDay != -1) {
      Serial.print("=== Denni spotreba za predchozi den celkem: ");
      Serial.print(dailyEnergyWh, 2);
      Serial.println(" Wh ===");
    }
    dailyEnergyWh = 0;
    dailyEnergyDay = today;
  }

  // ---------- Vysledne hodnoty pro Blynk - primo z klouzavych prumeru ----------
  // DULEZITE: k EMA prumeru (co meri jen ESP32/zatez) se pricita REGULATOR_QUIESCENT_A -
  // vlastni spotreba solarniho regulatoru samotneho, kterou INA226 vubec nevidi
  // (regulator je PRED shuntem), ale bere se z te same baterie porad.
  float avgChargePerHourAh = emaCurrentA_1h + REGULATOR_QUIESCENT_A;        // uz je to primo "Ah za hodinu" (A x 1h)
  float avgChargePerDayAh = (emaCurrentA_24h + REGULATOR_QUIESCENT_A) * 24.0; // prumerny proud za 24h x 24h = Ah/den

  // ---------- Odhad vydrze baterie pred prepnutim do uspornho rezimu ----------
  // POZOR: napeti se meri POD ZATEZEM (ne v klidu), takze SOC tabulka (kalibrovana
  // na klidove napeti) tu vnasi urcitou nepresnost - berte jako orientacni odhad.
  float currentSOC = estimateBatterySOC(ina_busVoltage);
  float thresholdSOC = estimateBatterySOC(BATTERY_LOW_VOLTAGE);
  float availableAh = (currentSOC - thresholdSOC) / 100.0 * BATTERY_CAPACITY_AH;
  if (availableAh < 0) availableAh = 0; // uz jsme pod prahem uspornho rezimu
  float estimatedDaysLeft = (avgChargePerDayAh > 0.01) ? (availableAh / avgChargePerDayAh) : 0;

  Serial.print("INA226 -> ");
  Serial.print(ina_busVoltage, 2); Serial.print(" V | ");
  Serial.print(ina_current_mA, 0); Serial.print(" mA | ");
  Serial.print(power_W, 2); Serial.print(" W okamzite | ");
  Serial.print(avgChargePerHourAh, 3); Serial.print(" Ah prumer/hod | ");
  Serial.print(avgChargePerDayAh, 2); Serial.print(" Ah prumer/den | ");
  Serial.print(estimatedDaysLeft, 1); Serial.println(" dni odhad vydrze");

  // Vykresleni na displej ma smysl jen kdyz je podsviceni zapnute - jinak by to
  // jen zbytecne zatezovalo LCD sbernici, aniz by to kdokoliv videl
  unsigned long dbgPredKreslenim = millis(); // DOCASNA DIAGNOSTIKA
  if (backlightOn && !pumpStatus) {
    // Napeti - uvnitr tela baterie, cislo + "V" s 5px mezerou, cele vycentrovane
    lcd.fillRect(251, 99, 70, 26, BATTERYGRAY);
    lcd.setFont(&fonts::FreeSans12pt7b);
    lcd.setTextSize(1);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(WHITE);
    {
      char numBuf[6];
      sprintf(numBuf, "%.1f", ina_busVoltage);
      int numW = lcd.textWidth(numBuf);
      int unitW = lcd.textWidth("V");
      int gap = 5;
      int leftX = 286 - (numW + gap + unitW) / 2; // 286 = stred baterie po posunu o 5px vlevo
      lcd.drawString(numBuf, leftX + numW / 2, 112);
      lcd.drawString("V", leftX + numW + gap + unitW / 2, 112);
    }

    // Proud - nad sipkou, posunuto o 5px vlevo a o 10px nahoru celkem (aby nezasahoval do hrotu sipky)
    // Pod 1000mA se zobrazuje v mA (cele cislo), od 1000mA vyse v A na 2 desetinna mista
    // (cerpadlo muze brat i nekolik A, v mA by to bylo neprehledne)
    lcd.fillRect(330, 55, 70, 18, BLACK);
    lcd.setFont(&fonts::FreeSans9pt7b);
    lcd.setTextColor(ina_current_mA < 0 ? WHITE : GREEN);
    {
      char currNumBuf[8];
      char unitStr[4];
      if (fabs(ina_current_mA) >= 1000.0) {
        sprintf(currNumBuf, "%.2f", ina_current_mA / 1000.0);
        strcpy(unitStr, "A");
      } else {
        sprintf(currNumBuf, "%.0f", ina_current_mA);
        strcpy(unitStr, "mA");
      }
      int numW = lcd.textWidth(currNumBuf);
      int unitW = lcd.textWidth(unitStr);
      int gap = 5;
      int leftX = 365 - (numW + gap + unitW) / 2; // 365 = stred sipky po posunu o 5px vlevo
      lcd.drawString(currNumBuf, leftX + numW / 2, 64);
      lcd.drawString(unitStr, leftX + numW + gap + unitW / 2, 64);
    }
    lcd.setTextDatum(TL_DATUM); // vratit vychozi zarovnani

    updateBatteryIcon();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V80, ina_busVoltage);
    Serial.print("V80 -> "); Serial.print(ina_busVoltage, 2); Serial.println(" V");
    Blynk.virtualWrite(V81, ina_current_mA);
    Blynk.virtualWrite(V82, avgChargePerHourAh);

    handleTouch(); // prednostni kontrola doteku mezi bloky Blynk volani

    Blynk.virtualWrite(V83, avgChargePerDayAh);
    Blynk.virtualWrite(V84, formatDny((int)round(estimatedDaysLeft)));
    Blynk.setProperty(V84, "label", lowPowerMode ? "ÚSPORNÝ REŽIM" : "NORMÁLNÍ REŽIM");
  }
  // DOCASNA DIAGNOSTIKA - kolik trvalo kresleni displeje + Blynk zapisy uvnitr bloku
  unsigned long dbgKresleniMs = millis() - dbgPredKreslenim;
  if (dbgKresleniMs > 100) {
    Serial.print("DBG: blok backlightOn (kresleni+Blynk) trval "); Serial.print(dbgKresleniMs);
    Serial.print("ms | backlightOn="); Serial.print(backlightOn);
    Serial.print(" | millis()-backLightON="); Serial.print(millis() - backLightON);
    Serial.print(" | backLightTime="); Serial.println(backLightTime);
  }

  handleTouch(); // pred updateBlynkBatteryIcon(), ktera ma taky vlastni Blynk volani

  unsigned long dbgPredIconou = millis(); // DOCASNA DIAGNOSTIKA
  updateBlynkBatteryIcon(ina_busVoltage); // orientacni ikonka baterie v appce (0-7, 8=nabijeni)
  unsigned long dbgIconaMs = millis() - dbgPredIconou;
  if (dbgIconaMs > 100) {
    Serial.print("DBG: updateBlynkBatteryIcon() trvalo "); Serial.print(dbgIconaMs); Serial.println("ms");
  }
}

// Odhad stavu nabiti (SOC) pro fyzickou ikonku na displeji - vyuziva stejnou
// piecewise-linearni tabulku jako Blynk ikonka (viz estimateBatterySOC() vyse).
void updateBatteryIcon() {
  int percent = (int)round(estimateBatterySOC(ina_busVoltage));
  percent = constrain(percent, 0, 100);

  lcd.setFont(&fonts::FreeSans9pt7b);
  lcd.fillRect(431, 6, 31, 17, BLACK);
  lcd.setTextColor(WHITE);
  if (percent < 100 && percent >= 10) lcd.setCursor(437, 7);
  else if (percent == 100) lcd.setCursor(431, 7);
  else lcd.setCursor(442, 7);
  lcd.print(percent);
}

// ==========================================================================
// ONBOARD SX1262 (RadioLib) - PRIJEM DAT ZE VZDALENYCH CIDEL
// Predpoklad: vzdaleny senzor posila strukturu RoomData (roomId,temp,humidity)
// Format je nutne sjednotit s kodem na vysilaci strane!
// ==========================================================================
// ==========================================================================
// DOCASNA TESTOVACI FUNKCE - simuluje prijem dat ze vsech 4 LoRa cidel najednou
// s nahodnymi hodnotami, aby slo overit zobrazeni na displeji i v Blynku bez
// nutnosti mit fyzicky pripojena cidla. Kresli stejnym zpusobem jako
// loraReceive() (viz tam), jen bez zavislosti na skutecnem prijmu packetu.
// ==========================================================================
void injectRandomTestSensorValues() {
  if (!TEST_MODE_RANDOM_SENSOR_VALUES) return;

  Teplota2 = random(-200, 401) / 10.0; Vlhkost2 = random(0, 1001) / 10.0;
  Teplota3 = random(-200, 401) / 10.0; Vlhkost3 = random(0, 1001) / 10.0;
  Teplota4 = random(-200, 401) / 10.0; Vlhkost4 = random(0, 1001) / 10.0;
  Teplota5 = random(-200, 401) / 10.0; Vlhkost5 = random(0, 1001) / 10.0;

  if (backlightOn && !pumpStatus) {
    lcd.setFont(&fonts::FreeSans12pt7b);
    lcd.setTextSize(1);
    lcd.setTextColor(WHITE, BLACK);

    float teploty[] = {Teplota2, Teplota3, Teplota4, Teplota5};
    float vlhkosti[] = {Vlhkost2, Vlhkost3, Vlhkost4, Vlhkost5};
    int fillY[] = {92, 121, 150, 179};
    int textY[] = {96, 125, 154, 183};

    for (int i = 0; i < 4; i++) {
      lcd.fillRect(103, fillY[i], 40, 28, BLACK);
      int rt = round(teploty[i]);
      if (rt >= 0 && rt < 10) lcd.setCursor(126, textY[i]);
      else if (rt >= 10) lcd.setCursor(113, textY[i]);
      else if (rt < 0 && rt > -10) lcd.setCursor(118, textY[i]);
      else lcd.setCursor(105, textY[i]);
      lcd.print(rt);

      lcd.fillRect(171, fillY[i], 41, 28, BLACK);
      int rv = round(vlhkosti[i]);
      if (rv < 100) lcd.setCursor(186, textY[i]);
      else if (rv == 100) lcd.setCursor(173, textY[i]);
      else lcd.setCursor(199, textY[i]);
      lcd.print(rv);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V41, Teplota2); Blynk.virtualWrite(V49, Vlhkost2);
    Blynk.virtualWrite(V42, Teplota3); Blynk.virtualWrite(V51, Vlhkost3);
    Blynk.virtualWrite(V43, Teplota4); Blynk.virtualWrite(V52, Vlhkost4);
    Blynk.virtualWrite(V45, Teplota5); Blynk.virtualWrite(V53, Vlhkost5);

    char tinBuf[16];
    sprintf(tinBuf, "Tin %d°C", (int)round(Teplota2));
    Blynk.virtualWrite(V67, tinBuf);
  }
}

void loraReceive() {
  if (!loraGotPacket) return;
  loraGotPacket = false;
  unsigned long prijemMillis = millis(); // pro mereni, jak dlouho trva odeslat odpoved

  size_t packetLen = radio.getPacketLength(); // nutne PRED readData (viz RadioLib dokumentace)
  RoomData data;
  int state = radio.readData((uint8_t*)&data, sizeof(RoomData));
  radio.startReceive(); // znovu spustit naslouchani na dalsi packet

  // DOCASNE VRACENO PRO SROVNAVACI TEST - viz predchozi komentar v historii:
  // podezirame, ze tyhle vypisy (bez pripojeneho Serial monitoru na
  // ESP32-S3 nativnim USB) mohou blokovat a zpozdit odeslani CentralReply.
  // Ted testujeme, o kolik presne se "zpozdeni" v terminalovem logu zhorsi.
  Serial.print("DBG loraReceive: state="); Serial.print(state);
  Serial.print(" (0=OK), delka="); Serial.print(packetLen);
  Serial.print(", RSSI="); Serial.print(radio.getRSSI());
  Serial.print("dBm, SNR="); Serial.print(radio.getSNR());
  Serial.print("dB, sizeof(RoomData)="); Serial.println(sizeof(RoomData));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("DBG surova data: roomId="); Serial.print(data.roomId);
    Serial.print(", teplota="); Serial.print(data.temperature);
    Serial.print(", vlhkost="); Serial.println(data.humidity);
  }

  if (state != RADIOLIB_ERR_NONE) return;

  // Pojistka: packet s jinou delkou nez RoomData je bud cizi LoRa provoz,
  // nebo smeti - nesmi se interpretovat jako data cidla
  if (packetLen != sizeof(RoomData)) {
    Serial.println("DBG: packet ma spatnou delku, zahazuji");
    return;
  }

  unsigned long nowUnix = rtc.now().unixtime();

  handleTouch(); // prednostni kontrola doteku pred zpracovanim (max 2 Blynk volani na mistnost)

  // Zachytit stary cas posledniho kontaktu JESTE PRED prepsanim v switchi
  // nize - potrebujeme to pro vypis "stari predchoziho prijmu" do terminalu
  unsigned long predchoziKontakt = 0;
  switch (data.roomId) {
    case 2: predchoziKontakt = CH2; break;
    case 3: predchoziKontakt = CH3; break;
    case 4: predchoziKontakt = CH4; break;
    case 5: predchoziKontakt = CH5; break;
  }

      switch (data.roomId) {
        case 2:
          Teplota2 = data.temperature; Vlhkost2 = data.humidity; Baterie2 = data.batteryMv; CH2 = nowUnix;
          if (backlightOn && !pumpStatus) {
            lcd.setFont(&fonts::FreeSans12pt7b);
            lcd.setTextSize(1);
            lcd.setTextColor(WHITE, BLACK);
            lcd.fillRect(103, 92, 40, 28, BLACK);
            int rt2 = round(Teplota2);
            if (rt2 >= 0 && rt2 < 10) lcd.setCursor(126, 96);
            else if (rt2 >= 10) lcd.setCursor(113, 96);
            else if (rt2 < 0 && rt2 > -10) lcd.setCursor(118, 96);
            else lcd.setCursor(105, 96);
            lcd.print(rt2);

            lcd.fillRect(171, 92, 41, 28, BLACK);
            int rv2 = round(Vlhkost2);
            if (rv2 < 100) lcd.setCursor(186, 96);
            else if (rv2 == 100) lcd.setCursor(173, 96);
            else lcd.setCursor(199, 96);
            lcd.print(rv2);
          }
          if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(V41, Teplota2);
            Blynk.virtualWrite(V49, Vlhkost2);
            char tinBuf[16];
            sprintf(tinBuf, "Tin %d°C", (int)round(Teplota2));
            Blynk.virtualWrite(V67, tinBuf);
            float napetiBat2 = Baterie2 / 1000.0;
            Blynk.virtualWrite(V72, napetiBat2); // napeti baterie cidla (V)
            Blynk.virtualWrite(V73, estimateLipolPercent(napetiBat2)); // % dle LiPol krivky
            const char* barvaBat2 = (napetiBat2 >= 3.5) ? "#23C48E" : (napetiBat2 >= 3.3) ? "#ED9D00" : "#D3435C";
            Blynk.setProperty(V72, "color", barvaBat2);
            Blynk.setProperty(V73, "color", barvaBat2);
            if (napetiBat2 < 3.3) {
              if (stavBaterieCidlo2 != 2) {
                Blynk.logEvent("crit_batt_cubecell", "Čidlo Kuchyň hlásí kritický stav baterie, co nejdříve jej dobijte");
                stavBaterieCidlo2 = 2;
              }
            } else if (napetiBat2 < 3.5) {
              if (stavBaterieCidlo2 == 0) {
                Blynk.logEvent("low_batt_cubecell", "Čidlo Kuchyň hlásí nízký stav baterie, je vhodné jej brzy dobít.");
                stavBaterieCidlo2 = 1;
              }
            } else {
              stavBaterieCidlo2 = 0;
            }
          }
          pip2_podtrzeno = 1;
          break;
        case 3:
          Teplota3 = data.temperature; Vlhkost3 = data.humidity; Baterie3 = data.batteryMv; CH3 = nowUnix;
          if (backlightOn && !pumpStatus) {
            lcd.setFont(&fonts::FreeSans12pt7b);
            lcd.setTextSize(1);
            lcd.setTextColor(WHITE, BLACK);
            lcd.fillRect(103, 121, 40, 28, BLACK);
            int rt3 = round(Teplota3);
            if (rt3 >= 0 && rt3 < 10) lcd.setCursor(126, 125);
            else if (rt3 >= 10) lcd.setCursor(113, 125);
            else if (rt3 < 0 && rt3 > -10) lcd.setCursor(118, 125);
            else lcd.setCursor(105, 125);
            lcd.print(rt3);

            lcd.fillRect(171, 121, 41, 28, BLACK);
            int rv3 = round(Vlhkost3);
            if (rv3 < 100) lcd.setCursor(186, 125);
            else if (rv3 == 100) lcd.setCursor(173, 125);
            else lcd.setCursor(199, 125);
            lcd.print(rv3);
          }
          if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(V42, Teplota3);
            Blynk.virtualWrite(V51, Vlhkost3);
            float napetiBat3 = Baterie3 / 1000.0;
            Blynk.virtualWrite(V74, napetiBat3);
            Blynk.virtualWrite(V75, estimateLipolPercent(napetiBat3));
            const char* barvaBat3 = (napetiBat3 >= 3.5) ? "#23C48E" : (napetiBat3 >= 3.3) ? "#ED9D00" : "#D3435C";
            Blynk.setProperty(V74, "color", barvaBat3);
            Blynk.setProperty(V75, "color", barvaBat3);
            if (napetiBat3 < 3.3) {
              if (stavBaterieCidlo3 != 2) {
                Blynk.logEvent("crit_batt_cubecell", "Čidlo Dolní WC hlásí kritický stav baterie, co nejdříve jej dobijte");
                stavBaterieCidlo3 = 2;
              }
            } else if (napetiBat3 < 3.5) {
              if (stavBaterieCidlo3 == 0) {
                Blynk.logEvent("low_batt_cubecell", "Čidlo Dolní WC hlásí nízký stav baterie, je vhodné jej brzy dobít.");
                stavBaterieCidlo3 = 1;
              }
            } else {
              stavBaterieCidlo3 = 0;
            }
          }
          pip3_podtrzeno = 1;
          break;
        case 4:
          Teplota4 = data.temperature; Vlhkost4 = data.humidity; Baterie4 = data.batteryMv; CH4 = nowUnix;
          if (backlightOn && !pumpStatus) {
            lcd.setFont(&fonts::FreeSans12pt7b);
            lcd.setTextSize(1);
            lcd.setTextColor(WHITE, BLACK);
            lcd.fillRect(103, 150, 40, 28, BLACK);
            int rt4 = round(Teplota4);
            if (rt4 >= 0 && rt4 < 10) lcd.setCursor(126, 154);
            else if (rt4 >= 10) lcd.setCursor(113, 154);
            else if (rt4 < 0 && rt4 > -10) lcd.setCursor(118, 154);
            else lcd.setCursor(105, 154);
            lcd.print(rt4);

            lcd.fillRect(171, 150, 41, 28, BLACK);
            int rv4 = round(Vlhkost4);
            if (rv4 < 100) lcd.setCursor(186, 154);
            else if (rv4 == 100) lcd.setCursor(173, 154);
            else lcd.setCursor(199, 154);
            lcd.print(rv4);
          }
          if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(V43, Teplota4);
            Blynk.virtualWrite(V52, Vlhkost4);
            float napetiBat4 = Baterie4 / 1000.0;
            Blynk.virtualWrite(V76, napetiBat4);
            Blynk.virtualWrite(V77, estimateLipolPercent(napetiBat4));
            const char* barvaBat4 = (napetiBat4 >= 3.5) ? "#23C48E" : (napetiBat4 >= 3.3) ? "#ED9D00" : "#D3435C";
            Blynk.setProperty(V76, "color", barvaBat4);
            Blynk.setProperty(V77, "color", barvaBat4);
            if (napetiBat4 < 3.3) {
              if (stavBaterieCidlo4 != 2) {
                Blynk.logEvent("crit_batt_cubecell", "Čidlo Horní WC hlásí kritický stav baterie, co nejdříve jej dobijte");
                stavBaterieCidlo4 = 2;
              }
            } else if (napetiBat4 < 3.5) {
              if (stavBaterieCidlo4 == 0) {
                Blynk.logEvent("low_batt_cubecell", "Čidlo Horní WC hlásí nízký stav baterie, je vhodné jej brzy dobít.");
                stavBaterieCidlo4 = 1;
              }
            } else {
              stavBaterieCidlo4 = 0;
            }
          }
          pip4_podtrzeno = 1;
          break;
        case 5:
          Teplota5 = data.temperature; Vlhkost5 = data.humidity; Baterie5 = data.batteryMv; CH5 = nowUnix;
          if (backlightOn && !pumpStatus) {
            lcd.setFont(&fonts::FreeSans12pt7b);
            lcd.setTextSize(1);
            lcd.setTextColor(WHITE, BLACK);
            lcd.fillRect(103, 179, 40, 28, BLACK);
            int rt5 = round(Teplota5);
            if (rt5 >= 0 && rt5 < 10) lcd.setCursor(126, 183);
            else if (rt5 >= 10) lcd.setCursor(113, 183);
            else if (rt5 < 0 && rt5 > -10) lcd.setCursor(118, 183);
            else lcd.setCursor(105, 183);
            lcd.print(rt5);

            lcd.fillRect(171, 179, 41, 28, BLACK);
            int rv5 = round(Vlhkost5);
            if (rv5 < 100) lcd.setCursor(186, 183);
            else if (rv5 == 100) lcd.setCursor(173, 183);
            else lcd.setCursor(199, 183);
            lcd.print(rv5);
          }
          if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(V45, Teplota5);
            Blynk.virtualWrite(V53, Vlhkost5);
            float napetiBat5 = Baterie5 / 1000.0;
            Blynk.virtualWrite(V78, napetiBat5);
            Blynk.virtualWrite(V79, estimateLipolPercent(napetiBat5));
            const char* barvaBat5 = (napetiBat5 >= 3.5) ? "#23C48E" : (napetiBat5 >= 3.3) ? "#ED9D00" : "#D3435C";
            Blynk.setProperty(V78, "color", barvaBat5);
            Blynk.setProperty(V79, "color", barvaBat5);
            if (napetiBat5 < 3.3) {
              if (stavBaterieCidlo5 != 2) {
                Blynk.logEvent("crit_batt_cubecell", "Čidlo Studna hlásí kritický stav baterie, co nejdříve jej dobijte");
                stavBaterieCidlo5 = 2;
              }
            } else if (napetiBat5 < 3.5) {
              if (stavBaterieCidlo5 == 0) {
                Blynk.logEvent("low_batt_cubecell", "Čidlo Studna hlásí nízký stav baterie, je vhodné jej brzy dobít.");
                stavBaterieCidlo5 = 1;
              }
            } else {
              stavBaterieCidlo5 = 0;
            }
          }
          pip5_podtrzeno = 1;
          break;
  }

  // Odeslani odpovedi cidlu - aktualni cas + pripadny vykon + PRESNY pocet
  // vterin do PRISTIHO probuzeni centraly (pocitano PRAVE TED, ne pri usinani)
  if (data.roomId >= 2 && data.roomId <= 5) {
    // Cidlo timhle prijmem potvrzuje, jakou mrizku aktualne pouziva (viz
    // RoomData.potvrzenaMrizkaSec, plnene z jeho posledni prijate odpovedi) -
    // pokud se shoduje s nasi cilovou hodnotou, oznacime tohle cidlo jako
    // "uz vi o zmene", coz muze ukoncit soubezne sledovani stare mrizky.
    if (data.potvrzenaMrizkaSec == (uint16_t)lowPowerWakeGridSec) {
      gridZmenaPotvrzena[data.roomId] = true;
    }

    CentralReply reply;
    reply.targetRoomId = data.roomId;
    reply.currentUnixTime = nowUnix;
    reply.newTxPower = (uint8_t)(desiredTxPower[data.roomId] + 9); // dBm s offsetem +9 (0..31), 0xFF = beze zmeny

    // +20s rezerva - centrala potrebuje cca 20s od probuzeni do plneho
    // provozu (WiFi, Blynk atd.). Bez tohohle by mohlo cidlo dorazit
    // driv, nez je centrala skutecne pripravena naslouchat.
    uint32_t dalsiOknoUnix = vypocitejDalsiMrizkoveOkno(nowUnix);
    uint32_t zakladniDeltaSec = (dalsiOknoUnix > nowUnix) ? (dalsiOknoUnix - nowUnix) : 5; // pojistka
    reply.sleepUntilNextWakeSec = (uint16_t)(zakladniDeltaSec + CENTRALA_BOOT_REZERVA_SEC);

    reply.currentGridSec = (uint16_t)lowPowerWakeGridSec;
    int replyState = radio.transmit((uint8_t*)&reply, sizeof(reply));
    unsigned long zpozdeniMs = millis() - prijemMillis; // cas od prijmu do odeslani odpovedi
    // DULEZITE: dokonceni VLASTNIHO vysilani (TX done) vyhodi na DIO1 take
    // preruseni - bez tohohle by loraGotPacket zustal nastaveny a pristi beh
    // loraReceive() by precetl prazdny/smeti buffer jako "novy packet"
    // (projevovalo se jako nesmyslne hodnoty typu 2147483647 na displeji
    // par sekund po kazdem uspesnem prijmu, nasledovane padem)
    loraGotPacket = false;
    radio.startReceive(); // zpet do prijmu po odeslani odpovedi

    // Prehledovy zapis do terminalu pri kazdem prijmu - VSECHNO, co cidlo
    // posila (vcetne jeho vlastniho casu), kvalita spojeni, stari
    // predchoziho kontaktu, baterie v % podle LiPol krivky, a aktualni
    // nastaveni (pozadovany vykon, sdileny interval)
    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
      const char* nazvyMistnosti[] = {"", "", "Kuchyn", "Dolni WC", "Horni WC", "Studna"};
      float batV = data.batteryMv / 1000.0;
      float batProc = estimateLipolPercent(batV);

      char kontaktBuf[24];
      if (predchoziKontakt == 0) {
        snprintf(kontaktBuf, sizeof(kontaktBuf), "prvni kontakt");
      } else {
        unsigned long odstupS = nowUnix - predchoziKontakt;
        snprintf(kontaktBuf, sizeof(kontaktBuf), "pred %lus", odstupS);
      }

      const char* vykonOK = (data.currentTxPowerDbm == desiredTxPower[data.roomId]) ? "OK" : "!!! NESEDI !!!";

      // Drift vlastniho oscilatoru cidla vuci RTC centraly - "predchoziKontakt"
      // je unix cas POSLEDNI uspesne odpovedi (tedy posledniho bodu synchronizace
      // z pohledu obou stran), data.msSinceSync je totez, ale zmerene vlastnimi
      // (nesynchronizovanymi) hodinami cidla od te doby.
      char driftBuf[48];
      if (predchoziKontakt == 0) {
        snprintf(driftBuf, sizeof(driftBuf), "drift: n/a (prvni kontakt)");
      } else {
        long skutecnyElapsedMs = (long)(nowUnix - predchoziKontakt) * 1000L;
        long driftMs = (long)data.msSinceSync - skutecnyElapsedMs;
        float driftPpm = (skutecnyElapsedMs > 0) ? (driftMs * 1000000.0f / skutecnyElapsedMs) : 0;
        snprintf(driftBuf, sizeof(driftBuf), "drift: %+ldms (%.0fppm) za %lds", driftMs, driftPpm, skutecnyElapsedMs / 1000);
      }

      const char* mrizkaOK = (data.potvrzenaMrizkaSec == (uint16_t)lowPowerWakeGridSec) ? "OK" : "!!! NESEDI !!!";

      char msg[400];
      snprintf(msg, sizeof(msg),
        "%s: T=%.1fC H=%.1f%% bat=%umV(%.0f%%) cidlo_unix=%lu | RSSI %.0fdBm SNR %.0fdB freq_err %.0fHz | posl. kontakt: %s | %s | vykon: chci=%ddBm, cidlo hlasi=%ddBm [%s] (odpoved stav=%d, zpozdeni=%lums) | mrizka: cil=%lus, cidlo hlasi=%us [%s], posilam sleep=%us",
        nazvyMistnosti[data.roomId], data.temperature, data.humidity,
        data.batteryMv, batProc, (unsigned long)data.unixtime,
        radio.getRSSI(), radio.getSNR(), radio.getFrequencyError(),
        kontaktBuf, driftBuf,
        desiredTxPower[data.roomId], data.currentTxPowerDbm, vykonOK, replyState, zpozdeniMs,
        lowPowerWakeGridSec, data.potvrzenaMrizkaSec, mrizkaOK, reply.sleepUntilNextWakeSec);
      logToTerminal(String(msg));
    }
  }

}

// ==========================================================================
// HLADINA VODY (ultrazvuk HY-SRF05/HC-SR04 - puvodni, nevodotesne cidlo)
// Predelano z vodotesne sondy JSN-SR04T-V3.0 zpet na puvodni cidlo (nemoznost
// vymeny za vodotesnou verzi). Casovani (trigger pulz, timeout 30000us,
// zadna extra delay() mezi merenimi) je PRESNE to, co bylo v puvodnim
// zdrojaku pred prechodem na JSN-SR04T - tedy uz prakticky overene.
// Blind zone / max range filtr puvodni kod nemel (jen vzdalenostDilci > 0),
// ale pridavame ho jako rozumnou pojistku proti nesmyslnym odrazum.
// ==========================================================================
#define HCSR04_BLIND_ZONE_M 0.02   // HC-SR04/HY-SRF05 nespolehlive pod cca 2cm
#define HCSR04_MAX_RANGE_M  4.0    // typicky spolehlivy dosah ~4m

void waterLevel() {
  // Vext ON - ultrazvukovy senzor je nyni napajeny z Vext, ne z trvaleho 3V3.
  // Kratke ustaleni pred prvnim trigger pulzem.
  digitalWrite(PIN_VEXT, LOW);
  delay(20);

  int N = 5;
  vzdalenost = 5000;

  for (int i = 0; i < N; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(20);
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(15);

    // KLICOVA OPRAVA: pulseIn() hned po trigger pulzu, driv nez cokoliv
    // jineho - Blynk.run()/runTimers()/handleTouch() se pred timhle radkem
    // volaly DRIV, coz mohlo trvat promenlivou (nekdy i vice ms) dobu.
    // Behem ni uz skutecny echo pulz mohl cely doběhnout a skoncit, takze
    // pulseIn() pak zachytil jen nahodny zbytek/sum (odtud nesmyslne male
    // hodnoty jako 45-47us namisto realneho mereni).
    odezva = pulseIn(PIN_ECHO, HIGH); // bez explicitniho timeoutu - sjednoceno
                                       // s overene funkcnim kodem (vychozi ~1s)

    if (WiFi.status() == WL_CONNECTED) Blynk.run();
    runTimers();
    handleTouch();

    vzdalenostDilci = odezva / 5831.0;
    if (vzdalenostDilci > HCSR04_BLIND_ZONE_M && vzdalenostDilci < HCSR04_MAX_RANGE_M
        && vzdalenost > vzdalenostDilci) {
      vzdalenost = vzdalenostDilci;
    }
  }

  rozdilVzd = abs(vzdalenostPredchozi - vzdalenost);

  if ((rozdilVzd < 0.07) || (vzdalenostPredchozi == 0) || vzdalenostRucne) {
    vzdalenostRucne = 0;
    vzdalenost -= 0.06;

    if (vzdalenost <= 0.90) {
      celkovyObjem = ((0.90 - vzdalenost) * 584.4) + ((1.08 - vzdalenost) * 523.7);
    } else {
      celkovyObjem = (1.08 - vzdalenost) * 523.7;
    }
    if (celkovyObjem > 1000) celkovyObjem = 1003;
    if (celkovyObjem < 0) celkovyObjem = 3;

    vzdalenostPredchozi = vzdalenost;
    if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V57, vzdalenost);
  }

  if (lastWaterLevel == 0) {
    lastWaterLevelNotify = celkovyObjem;
    lastWaterLevel = celkovyObjem;
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V70, lastWaterLevel);
      Blynk.virtualWrite(V71, lastWaterLevelNotify);
    }
  }

  hladinaAnimace = round((MaxZasoba - celkovyObjem) / 11.36);
  if (backlightOn && !pumpStatus) {
    if (celkovyObjem == 1003) {
      // Chybovy stav (mereni mimo rozsah) - cely sud cervene jako varovani
      lcd.fillRect(403, 190, 64, 88, RED);
    } else {
      lcd.fillRect(403, 190, 64, hladinaAnimace, BLACK); // prazdna cast (vzduch)
      lcd.fillRect(403, 190 + hladinaAnimace, 64, 88 - hladinaAnimace, BLUE); // voda
    }

    // Litry pod ikonou sudu - vycentrovano, bez zbytecnych desetinnych mist
    lcd.fillRect(400, 288, 70, 24, BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextColor(WHITE);
    {
      char litrBuf[6];
      sprintf(litrBuf, "%d", (int)round(celkovyObjem));
      lcd.setFont(&fonts::FreeSans12pt7b);
      lcd.setTextSize(1);
      int numW = lcd.textWidth(litrBuf);
      lcd.setFont(&fonts::FreeSerif12pt7b); // jednotka "l" - stejne jako v originale, jiny font jen pro tento znak
      int unitW = lcd.textWidth("l");
      int gap = 5;
      int leftX = 435 - (numW + gap + unitW) / 2; // 435 = stred sudu (400+70/2)
      lcd.setFont(&fonts::FreeSans12pt7b);
      lcd.drawString(litrBuf, leftX + numW / 2, 300);
      lcd.setFont(&fonts::FreeSerif12pt7b);
      lcd.drawString("l", leftX + numW + gap + unitW / 2, 301); // +2px dolu oproti cislu (300), sladit vizualne
    }
    lcd.setTextDatum(TL_DATUM);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V3, celkovyObjem);
    Blynk.virtualWrite(V7, celkovyObjem); // duplicitni s V3, ale takhle to bylo i v originale
    Blynk.virtualWrite(V5, vzdalenost);
  }

  // Volano primo tady, ne jako samostatny casovac - presne podle puvodniho
  // zamyslu (byl to v originale zakomentovany casovac s poznamkou "checkne
  // se to primo pri mereni z waterLevel")
  waterLevelCheck();
  odhadZasob();

  // Vext OFF - ale jen pokud ho nepotrebuje nic jineho (cerpadlo/waterflow
  // senzor, nebo prave probihajici sekvence rele)
  moznaVypniVext();
}

// ==========================================================================
// KONTROLA HLADINY (notifikace)
// ==========================================================================
void waterLevelCheck() {
  handleTouch(); // prednostni kontrola doteku pred moznymi Blynk.logEvent() volanimi nize

  // Neocekavany pokles hladiny (mozny unik pres selhany/ucpany zpetny
  // ventil na odvzdusnovacim kolinku hadice - viz vysvetleni v chatu).
  // BEZPECNOSTNI KONTROLA - BEZ OHLEDU NA NASTAVENI NOTIFIKACI
  // (V20/Notifikace), musi fungovat vzdy. pauzaOdectu zustava (reseni
  // presnosti mereni po cerpani - vyrovnavani hladin mezi sudy - ne
  // uzivatelska preference). Explicitni kontrola !pumpStatus - bez ni by
  // teoreticky mohl pravidelny 5minutovy casovac waterLevel() "trefit"
  // chvili, kdy cerpadlo prave cerpa, a legitimni pokles (davkovani vody)
  // by spustil falesny poplach.
  if (pauzaOdectu == 0 && !pumpStatus && (lastWaterLevel - celkovyObjem) >= hladinaPokles) {
    lastWaterLevel = celkovyObjem;

    // Pokus o "cuknuti" vodnim sloupcem - kratke sepnuti cerpadla na 1s
    // muze pomoci znovu roztrhnout hadici (nasat vzduch pres jednocestny
    // ventil), pokud se sifonovy efekt nechtene obnovil (napr. ventil
    // se zasekl/ucpal). Primy zapis pinu, NEJDE pres aplyCmd()/pumpStatus -
    // tohle neni normalni zalevaci cyklus, jen kratka bezpecnostni akce.
    // NEBLOKUJICI - jen se tady zapne a nastavi priznak, vypnuti resi
    // kontrolaCuknutiCerpadla() (viz timers[]). Probehne VZDY, nezavisle
    // na Notifikace i na WiFi - je to fyzicka mitigace, ne hlaseni.
    digitalWrite(PIN_PUMP, HIGH);
    cuknutiCerpadlaAktivni = true;
    cuknutiCerpadlaStartMillis = millis();

    // Notifikace se posle VZDY (nezavisle na V20/Notifikace), pokud je
    // WiFi pripojene - bez pripojeni fyzicky neni jak ji odeslat.
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.logEvent("hladina_pokles", "Pozor, neočekávaný pokles hladiny - možný únik nebo porucha zpětného ventilu");
    }
  }

  if (Notifikace && pauzaOdectu == 0 && WiFi.status() == WL_CONNECTED) {
    // Prirustek hladiny (destova voda) - hlasi se jen v rezimu 0 (sud na destovku)
    if ((celkovyObjem - lastWaterLevelNotify) >= hladinaPrirustek) {
      if (!rezimy) {
        int x = celkovyObjem - lastWaterLevelNotify;
        Blynk.logEvent("hladina_prirustek", String("Do sudu napršelo ") + x + String(" litrů vody"));
      }
      lastWaterLevelNotify = celkovyObjem;
    }

    // Blizka plna kapacita - relevantni jen v rezimu 1 (aktivne doplnovana nadrz)
    if (rezimy && lastWaterLevel > (MaxZasoba - 20)) {
      Blynk.logEvent("max_zasoba", "Pozor, sudy budou brzy plné!");
    }

    // Nizka zasoba - upozorni A ROVNOU snizi limit litru na jednu zalivku
    if (lastWaterLevel < hladinaMinimum && !hladinaMinimumBoolean) {
      Blynk.logEvent("hladina_minimum", String("Zásoba vody je méně než ") + hladinaMinimum + String(" litrů, byl spuštěn úsporný režim"));
      litryLimit = spotrebaUspornyRezim; // aplikovat rovnou lokalne (Blynk.virtualWrite sam o sobe BLYNK_WRITE handler nespusti)
      EEPROM.write(9, litryLimit);
      EEPROM.commit(); // DULEZITE: bez commit() se zapis na ESP32 neulozi do flash, jen do RAM bufferu
      Blynk.virtualWrite(V9, spotrebaUspornyRezim); // promitnout i do slideru v appce
      hladinaMinimumBoolean = 1;
    } else if ((lastWaterLevel - hladinaMinimum) > 50) {
      hladinaMinimumBoolean = 0;
    }

    // Poloviční zasoba - jen informativni upozorneni
    if (lastWaterLevel < (MaxZasoba / 2) && !polovinaBoolean) {
      Blynk.logEvent("polovicni_zasoba", "Zásoba vody klesla na polovinu");
      polovinaBoolean = 1;
    } else if ((lastWaterLevel - (MaxZasoba / 2)) > 50) {
      polovinaBoolean = 0;
    }
  }

  // Sledovani prirustku/spotreby podle rezimu - bezi vzdy, nezavisle na Notifikace/WiFi
  if (!rezimy && (celkovyObjem - lastWaterLevel) > 15) {
    naprseloTyden += (celkovyObjem - lastWaterLevel);
    naprseloCelkem += (celkovyObjem - lastWaterLevel);
    lastWaterLevel = celkovyObjem;
    EEPROMWriteInt(150, naprseloTyden);
    EEPROMWriteInt(158, naprseloCelkem);
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V25, naprseloTyden);
      Blynk.virtualWrite(V46, naprseloCelkem);
    }
  }
  if (rezimy && (celkovyObjem - lastWaterLevel) > 15) {
    napusteno += (celkovyObjem - lastWaterLevel);
    lastWaterLevel = celkovyObjem;
    EEPROMWriteInt(166, napusteno);
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V27, napusteno);
    }
  }
}

// ==========================================================================
// TYDENNI RESET STATISTIK
// ==========================================================================
// ==========================================================================
// ODHAD ZASOBY VODY - kolik "zalivek" jeste zbyva pri aktualnim objemu,
// nastavenem limitu litru a poctu aktivnich casovacu za den
// ==========================================================================

// Ceske sklonovani slova "den" podle pocet (1 den, 2-4 dny, 0 a 5+ dnu)
String formatDny(int pocet) {
  String jednotka;
  if (pocet == 1) jednotka = "den";
  else if (pocet >= 2 && pocet <= 4) jednotka = "dny";
  else jednotka = "dnů";
  return String(pocet) + " " + jednotka;
}

void odhadZasob() {
  if (WiFi.status() != WL_CONNECTED) return;

  if ((Timer1 + Timer2 + Timer3) == 0) {
    Blynk.virtualWrite(V15, "Čas nenastaven");
  } else {
    int dny = (int)round(celkovyObjem / litryLimit / (Timer1 + Timer2 + Timer3));
    Blynk.virtualWrite(V15, formatDny(dny));
  }
}

// ==========================================================================
// KOMPLETNI RESYNCHRONIZACE VSECH KLICOVYCH HODNOT DO BLYNKU
// Volat po (znovu)pripojeni, aby appka hned ukazala vsechno aktualni -
// posledni zalevani, kolik litru, nastaveni casovacu atd. - stejne jako
// v originale.
// ==========================================================================
void resyncAllToBlynk() {
  if (WiFi.status() != WL_CONNECTED) return;

  // Pokud jsme v normalnim provozu (ne v odlehcenem probuzeni z uspornho
  // rezimu), oznacit se jako "nespim" - jinak by po navratu z uspornho
  // rezimu (pres ESP.restart()) zustal priznak V95 navzdy neaktualni,
  // protoze normalni setup()/loop() uz ho jinak nikde nenuluje.
  if (!lowPowerMode) {
    Blynk.virtualWrite(V95, "Online");
  }

  // Pro jistotu odkryt i pri kazdem znovupripojeni (viz stejny komentar v setup())
  Blynk.setProperty(V65, "isHidden", false);
  Blynk.setProperty(V66, "isHidden", false);

  // Ciste momentalni potvrzovaci tlacitka (jednorazovy trigger) vzdy vratit
  // na vychozi 0 - at appka po restartu neukazuje stary "nataceny" stav
  Blynk.virtualWrite(V63, 0);
  Blynk.virtualWrite(V99, 0);
  Blynk.virtualWrite(V101, 0);

  // V97 je TRVALY prepinac (ne momentalni trigger) - posilame SKUTECNOU
  // hodnotu, ne natvrdo 0. Dulezite hlavne po vlastnim restartu (viz
  // checkNavratDoUspornhoSpanku()) - preventNextSleep tam zustava true
  // schvalne (RTC_DATA_ATTR), a appka by jinak zavadejici ukazovala
  // "vypnuto", i kdyz zarizeni skutecne zustava vzhuru.
  Blynk.virtualWrite(V97, preventNextSleep ? 1 : 0);

  // Echo potvrzeni aktualne platneho sdileneho intervalu a vykonu cidel
  // (napr. po startu, aby appka hned ukazovala to, co je skutecne nactene z EEPROM)
  Blynk.virtualWrite(V103, lowPowerWakeGridSec / 60);
  Blynk.virtualWrite(V124, desiredTxPower[2]);
  Blynk.virtualWrite(V125, desiredTxPower[3]);
  Blynk.virtualWrite(V126, desiredTxPower[4]);
  Blynk.virtualWrite(V127, desiredTxPower[5]);

  // Hladinove parametry a rezim
  Blynk.virtualWrite(V21, hladinaPokles);
  Blynk.virtualWrite(V31, hladinaPokles);
  Blynk.virtualWrite(V22, hladinaPrirustek);
  Blynk.virtualWrite(V32, hladinaPrirustek);
  Blynk.virtualWrite(V23, hladinaMinimum);
  Blynk.virtualWrite(V33, hladinaMinimum);
  Blynk.virtualWrite(V24, spotrebaUspornyRezim);
  Blynk.virtualWrite(V34, spotrebaUspornyRezim);
  Blynk.virtualWrite(V16, rezimy);
  Blynk.virtualWrite(V20, Notifikace);

  handleTouch(); // prednostni kontrola doteku mezi bloky

  // Cerpadlo - posledni pouziti a stav
  Blynk.virtualWrite(V14, lastUsedAmount);
  Blynk.virtualWrite(V13, lastPumpUsed);
  Blynk.virtualWrite(V8, pumpStatus);
  if (pumpStatus) pumpLED.on(); else pumpLED.off();

  // Limit litru na zalivku
  Blynk.virtualWrite(V9, litryLimit);
  Blynk.virtualWrite(V39, litryLimit);

  handleTouch();

  // Casovace - zapnuti/vypnuti, LED, casy
  Blynk.virtualWrite(V17, Timer1);
  Blynk.virtualWrite(V18, Timer2);
  Blynk.virtualWrite(V19, Timer3);
  if (Timer1) timer1LED.on(); else timer1LED.off();
  if (Timer2) timer2LED.on(); else timer2LED.off();
  if (Timer3) timer3LED.on(); else timer3LED.off();
  Blynk.virtualWrite(V10, casovac1, 0, tz);
  Blynk.virtualWrite(V11, casovac2, 0, tz);
  Blynk.virtualWrite(V12, casovac3, 0, tz);

  handleTouch();

  odhadZasob();

  // Statistiky srazek/spotreby (tydenni i celkove) a doplnovani
  Blynk.virtualWrite(V25, naprseloTyden);
  Blynk.virtualWrite(V26, ubyloTyden);
  Blynk.virtualWrite(V46, naprseloCelkem);
  Blynk.virtualWrite(V47, ubyloCelkem);
  Blynk.virtualWrite(V27, napusteno);

  handleTouch();

  // Hladina v sudu
  Blynk.virtualWrite(V3, celkovyObjem);
  Blynk.virtualWrite(V7, celkovyObjem);
  Blynk.virtualWrite(V5, vzdalenost);
}

void resetPoTydnu() {
  handleTouch(); // prednostni kontrola doteku

  DateTime now = rtc.now();
  if (now.dayOfTheWeek() == 1 && now.hour() == 0 && now.minute() == 10) {
    naprseloTyden = 0;
    ubyloTyden = 0;
    EEPROMWriteInt(154, ubyloTyden);
    EEPROMWriteInt(150, naprseloTyden);
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V25, naprseloTyden);
      Blynk.virtualWrite(V26, ubyloTyden);
      logToTerminal("Reset provoden (automaticky, tydenni)");
    }
  }
}

// ==========================================================================
// WiFi/BLYNK - NEBLOKUJICI KONTROLA A OBNOVA SPOJENI
// ==========================================================================
// Kontrola, jestli jsme blizko naplanovaneho spusteni nektereho z casovacu -
// pouziva se k tomu, aby WiFi reconnect (disconnect()+begin(), chvilku
// narocne na CPU/casovani) nerusil presne spusteni zalevani.
bool isNearAnyTimerStart(DateTime now, int marginMin) {
  int nowMin = now.hour() * 60 + now.minute();
  int t1 = startHour1 * 60 + startMinute1;
  int t2 = startHour2 * 60 + startMinute2;
  int t3 = startHour3 * 60 + startMinute3;

  if (Timer1 && abs(nowMin - t1) <= marginMin) return true;
  if (Timer2 && abs(nowMin - t2) <= marginMin) return true;
  if (Timer3 && abs(nowMin - t3) <= marginMin) return true;
  return false;
}

void checkWifiBlynk() {
  if (millis() - lastWifiRetry < wifiRetryInterval) return;
  lastWifiRetry = millis();

  if (isNearAnyTimerStart(rtc.now(), 2)) return; // nerusit WiFi reconnectem tesne kolem planovaneho zalevani

  handleTouch(); // prednostni kontrola doteku pred moznym zdrzenim nize

  if (WiFi.status() != WL_CONNECTED) {
    if (backlightOn && !pumpStatus) lcd.fillCircle(50, 309, 5, RED);
    WiFi.disconnect();
    WiFi.begin(ssid, pass); // asynchronni pokus, NEBLOKUJE
    ConnectionTimeOut++;

    if (ConnectionTimeOut >= 4 && !nouzakButton) {
      nouzakButton = 1;
      nouzakStartTime = rtc.now().timestamp(DateTime::TIMESTAMP_FULL);
    }
  } else {
    if (backlightOn && !pumpStatus) lcd.fillCircle(50, 309, 5, GREEN);
    if (ConnectionTimeOut > 0) WiFi.setSleep(true); // po cerstvem znovupripojeni znovu zapnout modem sleep
    ConnectionTimeOut = 0;

    handleTouch(); // pred Blynk.connect(), ktery muze chvili trvat

    if (!Blynk.connected()) {
      Blynk.connect(2000);
    }

    handleTouch(); // po Blynk.connect(), pred pripadnou resyncAllToBlynk()

    if (Blynk.connected()) {
      if (backlightOn && !pumpStatus) lcd.fillCircle(130, 309, 5, GREEN);
      if (nowOnline) {
        // po znovupripojeni propiseme uplne vsechny klicove hodnoty
        resyncAllToBlynk(); // ma sve vlastni kontroly doteku uvnitr, viz nize
        nowOnline = 0;
      }
    } else {
      if (backlightOn && !pumpStatus) lcd.fillCircle(130, 309, 5, RED);
    }
  }
}

// ==========================================================================
// SILA WIFI SIGNALU
// ==========================================================================
void wifiRSSI() {
  handleTouch(); // prednostni kontrola doteku

  int bars = 0;
  uint16_t barva = GREEN;

  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();

    if (backlightOn && !pumpStatus) {
      // smazani pripadneho "X" (offline znacky) pres cernou barvu
      lcd.drawLine(394, 4, 399, 9, BLACK);
      lcd.drawLine(393, 4, 399, 10, BLACK);
      lcd.drawLine(393, 5, 398, 10, BLACK);
      lcd.drawLine(393, 9, 398, 4, BLACK);
      lcd.drawLine(393, 10, 399, 4, BLACK);
      lcd.drawLine(394, 10, 399, 5, BLACK);
    }

    if (rssi > -55) bars = 5;
    else if (rssi >= -65) bars = 4;
    else if (rssi >= -70) bars = 3;
    else if (rssi >= -78) bars = 2;
    else if (rssi >= -82) { bars = 1; barva = RED; }
    else bars = 0;

    Blynk.virtualWrite(V61, bars); // Icon widget (0-5) - stejne sloupecky jako na displeji
    Blynk.virtualWrite(V66, rssi); // surove dBm pro nativni "Signal Level" widget v zahlavi
  } else {
    if (backlightOn && !pumpStatus) {
      // cervene "X" pres sloupecky signalu, jako v originale
      lcd.drawLine(394, 4, 399, 9, RED);
      lcd.drawLine(393, 4, 399, 10, RED);
      lcd.drawLine(393, 5, 398, 10, RED);
      lcd.drawLine(393, 9, 398, 4, RED);
      lcd.drawLine(393, 10, 399, 4, RED);
      lcd.drawLine(394, 10, 399, 5, RED);
    }
    bars = 0;
  }

  if (backlightOn && !pumpStatus) {
    for (int b = bars + 1; b <= 5; b++) {
      lcd.fillRect(390 + (b * 5), 25 - (b * 4), 3, b * 4, DARKDARKGREY);
    }
    for (int b = 0; b <= bars; b++) {
      lcd.fillRect(390 + (b * 5), 25 - (b * 4), 3, b * 4, barva);
    }
  }
}

// ==========================================================================
// BLYNK CALLBACKY
// ==========================================================================
BLYNK_WRITE(V8) {
  pumpStatus = param.asInt();
  if (pumpStatus) pumpStartTimeMillis = millis();
  aplyCmd();
}

BLYNK_WRITE(V9) {
  litryLimit = param.asInt();
  EEPROM.write(9, litryLimit); // POZOR: litryLimit je jen 1 bajt (byte) - EEPROMWriteInt() by
                              // zapsal 2 bajty (adresa 9 i 10) a adresa 10 patri startHour1!
  EEPROM.commit(); // DULEZITE: bez commit() se zapis na ESP32 neulozi do flash, jen do RAM bufferu
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V39, litryLimit);
  odhadZasob();
}

BLYNK_WRITE(V17) { Timer1 = param.asInt(); EEPROM.write(17, Timer1); EEPROM.commit(); if (Timer1) timer1LED.on(); else timer1LED.off(); odhadZasob(); }

// ==========================================================================
// RUCNI OKAMZITE PREMERENI HLADINY
// ==========================================================================
// ==========================================================================
// RUCNI PRIPOJENI/ODPOJENI RELE (baterie/solar) PRES BLYNK SWITCH
// V87 = switch (1=ON="Odpojit" napis na tlacitku, 0=OFF="Pripojit" napis
//       na tlacitku - popisky se nastavuji primo ve widgetu v Blynk appce)
// V88 = echo (String) - "Pripojeno"/"Odpojeno" po dokonceni sekvence
//
// Rucni akce vzdy respektuje bezpecnou sekvenci (baterie->+2s solar pri
// pripojovani, solar->+2s baterie pri odpojovani) - nikdy nenastane stav,
// kdy by byl solar pripojeny bez baterie. Nijak neovlivnuje budouci
// automatiku (rizeniRele()) - ta pri svem dalsim behu proste znovu
// vyhodnoti aktualni stav (noc/rezim) a udela, co ma, bez ohledu na to,
// jestli k soucasnemu stavu doslo automaticky, nebo rucnim zasahem.
// ==========================================================================
BLYNK_WRITE(V87) {
  bool pripojit = param.asInt();

  vypniAutomatikuRele(); // jakykoliv rucni zasah vypina automatiku + posle notifikaci

  if (pripojit) {
    spustPripojovaciSekvenciRele(true); // true = rucne vyvolana sekvence
  } else {
    spustOdpojovaciSekvenciRele(true);
  }

  // Okamzita vizualni odezva na displeji, stejny duvod jako u dotykove
  // ikonky - jen pokud je dashboard prave viditelny
  if (displayLogicOn && !pumpStatus) {
    kresliIkonuZarovky(pripojit);
  }
}

// V89 - switch pro rucni zapnuti/vypnuti automatiky rele. 1 = automatika
// aktivni (rizeniRele() zase sama rozhoduje), 0 = automatika vypnuta
// (stejny stav jako po rucnim zasahu pres ikonu zarovky nebo V87).
BLYNK_WRITE(V89) {
  bool zapnout = param.asInt();
  if (zapnout) {
    releAutomatikaAktivni = true;
    aktualizujStavAutomatikyRele();
  } else {
    vypniAutomatikuRele(); // stejna cesta jako ostatni rucni vypnuti - posle i notifikaci
  }
}

// ==========================================================================
// RUCNI OKAMZITE PREMERENI HLADINY
// ==========================================================================
BLYNK_WRITE(V50) {
  vzdalenostRucne = 1;
  waterLevel();
  if (WiFi.status() == WL_CONNECTED) {
    logToTerminal("vzdálenost hladiny=" + String(vzdalenostDilci));
    Blynk.virtualWrite(V50, 0);
  }
}

// ==========================================================================
// TERMINAL PRO RUCNI NASTAVENI CASU/DATA (zaloha pro dlouhy vypadek NTP)
// Sjednoceno na V0 spolu s eventTerminal (log DST prechodu, OTA stavu, atd.) -
// prikazy se mísí s ostatnimi logy, coz je zamerne (stejny princip jako
// prikazova radka, kde taky vidis logy i pises prikazy na jednom miste).
// Na rozdil od originalu neni potreba zadavat den v tydnu - nase RTClib
// (Adafruit) si ho sama spocita z data, netreba nastavovat rucne.
// ==========================================================================
BLYNK_WRITE(V0) {
  String cmd = param.asStr();

  if (cmd.substring(0, 4) == "help") {
    logToTerminal("GET vrati aktualni datum a cas");
    logToTerminal("SET time hh:mm:ss");
    logToTerminal("SET date dd.mm.rrrr");
  } else if (cmd.substring(0, 3) == "get" || cmd.substring(0, 3) == "GET") {
    DateTime now = rtc.now();
    char buf[32];
    sprintf(buf, "%02d.%02d.%04d %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());
    logToTerminal(String(buf));
  } else if (cmd.substring(0, 3) == "set" || cmd.substring(0, 3) == "SET") {
    DateTime now = rtc.now();
    if (cmd.substring(4, 8) == "time") {
      int hh = cmd.substring(9, 11).toInt();
      int mm = cmd.substring(12, 14).toInt();
      int ss = cmd.substring(15, 17).toInt();
      rtc.adjust(DateTime(now.year(), now.month(), now.day(), hh, mm, ss));
      logToTerminal("Cas nastaven");
      displayTime();
    } else if (cmd.substring(4, 8) == "date") {
      int dd = cmd.substring(9, 11).toInt();
      int mo = cmd.substring(12, 14).toInt();
      int yy = cmd.substring(15, 19).toInt();
      rtc.adjust(DateTime(yy, mo, dd, now.hour(), now.minute(), now.second()));
      logToTerminal("Datum nastaveno");
      displayTime();
    } else {
      logToTerminal("Neplatny prikaz, napis help");
    }
  } else {
    logToTerminal("Neplatny prikaz, napis help");
  }
}

// ==========================================================================
// NASTAVENI HLADINOVYCH PARAMETRU (viz waterLevelCheck())
// ==========================================================================
BLYNK_WRITE(V21) { hladinaPokles = param.asInt(); EEPROM.write(21, hladinaPokles); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V31, hladinaPokles); }
BLYNK_WRITE(V22) { hladinaPrirustek = param.asInt(); EEPROM.write(22, hladinaPrirustek); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V32, hladinaPrirustek); }
BLYNK_WRITE(V23) { hladinaMinimum = param.asInt(); EEPROM.write(23, hladinaMinimum); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V33, hladinaMinimum); }
BLYNK_WRITE(V24) { spotrebaUspornyRezim = param.asInt(); EEPROM.write(24, spotrebaUspornyRezim); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V34, spotrebaUspornyRezim); }
BLYNK_WRITE(V16) { rezimy = param.asInt(); EEPROM.write(16, rezimy); EEPROM.commit(); }
BLYNK_WRITE(V18) { Timer2 = param.asInt(); EEPROM.write(18, Timer2); EEPROM.commit(); if (Timer2) timer2LED.on(); else timer2LED.off(); odhadZasob(); }
BLYNK_WRITE(V19) { Timer3 = param.asInt(); EEPROM.write(19, Timer3); EEPROM.commit(); if (Timer3) timer3LED.on(); else timer3LED.off(); odhadZasob(); }

BLYNK_WRITE(V10) {
  TimeInputParam t(param);
  startHour1 = t.getStartHour();
  startMinute1 = t.getStartMinute();
  casovac1 = (startHour1 * 3600L) + (startMinute1 * 60L);
  EEPROM.write(10, startHour1);
  EEPROM.write(110, startMinute1);
  EEPROM.commit();
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V10, casovac1, 0, tz);
}

BLYNK_WRITE(V11) {
  TimeInputParam t(param);
  startHour2 = t.getStartHour();
  startMinute2 = t.getStartMinute();
  casovac2 = (startHour2 * 3600L) + (startMinute2 * 60L);
  EEPROM.write(11, startHour2);
  EEPROM.write(111, startMinute2);
  EEPROM.commit();
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V11, casovac2, 0, tz);
}

BLYNK_WRITE(V12) {
  TimeInputParam t(param);
  startHour3 = t.getStartHour();
  startMinute3 = t.getStartMinute();
  casovac3 = (startHour3 * 3600L) + (startMinute3 * 60L);
  EEPROM.write(12, startHour3);
  EEPROM.write(112, startMinute3);
  EEPROM.commit();
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V12, casovac3, 0, tz);
}

BLYNK_WRITE(V20) {
  Notifikace = param.asInt();
  EEPROM.write(20, Notifikace);
  EEPROM.commit();
}

// ==========================================================================
// LOW POWER - rucni ovladani z Blynk aplikace
// ==========================================================================
BLYNK_WRITE(V94) { // tlacitko "Uspat ted"
  if (param.asInt()) {
    enterLowPowerSleep();
  }
}

BLYNK_WRITE(V96) { // slider - novy sdileny interval probouzeni (v MINUTACH) - APLIKUJE SE OKAMZITE
  uint32_t newGridSec = (uint32_t)param.asInt() * 60UL;
  if (newGridSec == 0 || newGridSec == lowPowerWakeGridSec) return; // nic se nemeni

  long rozdilSec = (long)newGridSec - (long)lowPowerWakeGridSec;
  if (rozdilSec < 0) rozdilSec = -rozdilSec;

  if (rozdilSec > 10) {
    // Vyznamna zmena - potreba soubezneho sledovani obou oken, dokud vsechna
    // JIZ AKTIVNI cidla (CHx != 0, tedy uz nekdy komunikovala) nepotvrdi
    // prijeti nove mrizky. Cidlo, co jeste nikdy nekomunikovalo, netreba cekat.
    predchoziGridSec = lowPowerWakeGridSec;
    gridZmenaPotvrzena[2] = (CH2 == 0);
    gridZmenaPotvrzena[3] = (CH3 == 0);
    gridZmenaPotvrzena[4] = (CH4 == 0);
    gridZmenaPotvrzena[5] = (CH5 == 0);
    Serial.print("Vyznamna zmena mrizky (rozdil "); Serial.print(rozdilSec);
    Serial.println("s) - zahajuji soubezne sledovani stare i nove mrizky");
  }
  // Male zmeny (<=10s rozdil) nepotrebuji dualni okno - jsou v tolerance i pro
  // cidlo, ktere jeste nestihlo dostat aktualizaci (viz stejna tolerance na cidle)

  lowPowerWakeGridSec = newGridSec; // APLIKUJE SE OKAMZITE - delta-mechanismus
                                     // (sleepUntilNextWakeSec) zajisti spravnou
                                     // synchronizaci kazdeho cidla pri jeho
                                     // nejblizsim kontaktu, zadne zpozdeni netreba.
  EEPROM.put(EEPROM_ADDR_WAKE_GRID_SEC, lowPowerWakeGridSec);
  EEPROM.commit();

  Serial.print("Novy sdileny interval "); Serial.print(newGridSec); Serial.println("s - aplikovano ihned");

  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V103, param.asInt()); // echo potvrzeni prijeti (minuty)
}

BLYNK_WRITE(V97) { // prepinac - dokud je ON, centrala neusne (napr. behem OTA)
  preventNextSleep = param.asInt();
  Serial.println(preventNextSleep ? "Zabraneni spanku: ZAPNUTO" : "Zabraneni spanku: vypnuto");
}

// ==========================================================================
// Nastaveni vysilaciho vykonu jednotlivych cidel na dalku (zustava per-room,
// nezavisle). Interval vysilani jiz NENI per-room nastavitelny - viz V96
// (sdileny interval pro centralu i vsechna cidla dohromady).
// Hodnota se posle cidlu v CentralReply pri jeho nejblizsim kontaktu
// (viz loraReceive) - neni potreba zadny zvlastni "push" prikaz.
// ==========================================================================
BLYNK_WRITE(V120) { desiredTxPower[2] = constrain(param.asInt(), -9, 22); EEPROM.write(EEPROM_ADDR_TXPOWER_BASE + 0, desiredTxPower[2] + 9); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V124, desiredTxPower[2]); } // kuchyn - vykon v dBm
BLYNK_WRITE(V121) { desiredTxPower[3] = constrain(param.asInt(), -9, 22); EEPROM.write(EEPROM_ADDR_TXPOWER_BASE + 1, desiredTxPower[3] + 9); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V125, desiredTxPower[3]); } // dolni WC
BLYNK_WRITE(V122) { desiredTxPower[4] = constrain(param.asInt(), -9, 22); EEPROM.write(EEPROM_ADDR_TXPOWER_BASE + 2, desiredTxPower[4] + 9); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V126, desiredTxPower[4]); } // horni WC
BLYNK_WRITE(V123) { desiredTxPower[5] = constrain(param.asInt(), -9, 22); EEPROM.write(EEPROM_ADDR_TXPOWER_BASE + 3, desiredTxPower[5] + 9); EEPROM.commit(); if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V127, desiredTxPower[5]); } // studna
// ==========================================================================
// RUCNI RESET CELKOVYCH STATISTIK (V63/V64 - stejny vzor jako tydenni reset)
// ==========================================================================
boolean reset_celkovy = false;

BLYNK_WRITE(V63) {
  reset_celkovy = param.asInt();
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V64, reset_celkovy);
}

BLYNK_WRITE(V64) {
  if (reset_celkovy) {
    ubyloCelkem = 0;
    EEPROMWriteInt(162, ubyloCelkem);
    naprseloCelkem = 0;
    EEPROMWriteInt(158, naprseloCelkem);
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V47, ubyloCelkem);
      Blynk.virtualWrite(V46, naprseloCelkem);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V63, 0);
    Blynk.virtualWrite(V64, 0);
  }
  reset_celkovy = false;
}
// Puvodne V61/V62 - precislovano na V101/V102, protoze V61/V62 uz pouzivame
// pro WiFi sloupecky a ikonu baterie.
// ==========================================================================
boolean reset_tyden = false;

BLYNK_WRITE(V101) { // spusti potvrzovaci dotaz
  reset_tyden = param.asInt();
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V102, reset_tyden);
}

BLYNK_WRITE(V102) { // tlacitko ANO - potvrzeni resetu
  if (reset_tyden) {
    ubyloTyden = 0;
    EEPROMWriteInt(154, ubyloTyden);
    naprseloTyden = 0;
    EEPROMWriteInt(150, naprseloTyden);
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.virtualWrite(V26, ubyloTyden);
      Blynk.virtualWrite(V25, naprseloTyden);
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V101, 0);
    Blynk.virtualWrite(V102, 0);
  }
  reset_tyden = false;
}

// ==========================================================================
// OTA AKTUALIZACE FIRMWARU PRES HTTP(S) - vzdalene, odkudkoliv z internetu
// ==========================================================================
// Postup pouziti:
// 1) Zkompiluj novy firmware v Arduino IDE -> Sketch -> Export compiled Binary
//    (vznikne .bin soubor vedle .ino souboru)
// 2) Nahraj ten .bin nekam s verejnou HTTPS URL (napr. GitHub - "raw" odkaz
//    na soubor, nebo vlastni web hosting)
// 3) V Blynk appce zadej tu URL do textoveho pole (V98) a stiskni tlacitko
//    "Spustit OTA" (V99)
// 4) Zarizeni si firmware stahne, naflashuje a samo restartuje
//
// BEZPECNOSTNI POJISTKA: V setup() se po uspesnem pripojeni WiFi+Blynk vola
// esp_ota_mark_app_valid_cancel_rollback() - tim aktualni bezici firmware
// "potvrdi" jako funkcni. Pokud by nova (OTA) firmware nekdy nedosla az k
// tomuhle potvrzeni (napr. spadne pri startu, nepripoji WiFi...), bootloader
// se pri dalsim restartu SAM automaticky vrati na predchozi, overenou verzi -
// dulezite prave proto, ze se k zarizeni fyzicky nedostanes.
// ==========================================================================
void performOTAUpdate(String url) {
  if (url.length() < 8) {
    Serial.println("OTA: prazdna nebo neplatna URL, presksakuji");
    if (WiFi.status() == WL_CONNECTED) {
      logToTerminal("OTA: chybi nebo je spatna URL");
    }
    return;
  }

  Serial.print("OTA: zacinam stahovat a flashovat z ");
  Serial.println(url);
  if (WiFi.status() == WL_CONNECTED) {
    logToTerminal("OTA: zahajuji stahovani...");
    Blynk.run(); // odeslat hned, pred blokujicim stahovanim nize
  }

  WiFiClientSecure client;
  client.setInsecure(); // POZOR: preskakuje overeni TLS certifikatu serveru.
                        // Jednodussi pro hobby projekt, ale nekdo teoreticky
                        // by mohl podvrhnout jiny firmware, kdyby ovladl
                        // sit/DNS po ceste. Pro vyssi bezpecnost by bylo
                        // potreba pripojit spravny root certifikat serveru.

  // Restart resime rucne (ne pres rebootOnUpdate), abychom stihli poslat
  // vysledek do Blynk appky drive, nez se zarizeni restartuje
  // Sledovani presmerovani - defaultne je VYPNUTE, ale GitHub CDN muze
  // za urcitych okolnosti odpovedet presmerovanim misto primeho 200 OK
  // (napr. kvuli hlavickam, ktere HTTPUpdate posila). Bez tohohle by to
  // skoncilo chybou "-104 Wrong HTTP Code".
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(false);

  t_httpUpdate_return ret = httpUpdate.update(client, url);

  String statusMsg;
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      statusMsg = "OTA CHYBA (" + String(httpUpdate.getLastError()) + "): " + httpUpdate.getLastErrorString();
      Serial.println(statusMsg);
      break;
    case HTTP_UPDATE_NO_UPDATES:
      statusMsg = "OTA: server hlasi, ze neni co aktualizovat";
      Serial.println(statusMsg);
      break;
    case HTTP_UPDATE_OK:
      statusMsg = "OTA: uspech, restartuji se do nove firmware...";
      Serial.println(statusMsg);
      break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    logToTerminal(statusMsg);
    Blynk.run();
    delay(1500); // cas na skutecne odeslani zpravy pred pripadnym restartem
  }

  if (ret == HTTP_UPDATE_OK) {
    ESP.restart();
  }
}

// Pri kazdem (znovu)pripojeni k Blynk si vyzadame posledni ulozenou hodnotu
// V98 - jinak by BLYNK_WRITE(V98) nikdy nedostal sanci se zavolat, pokud jsi
// URL zadal v nejake drivejsi session, ne prave ted behem tohohle pripojeni
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V98);
}

BLYNK_WRITE(V98) { // textove pole - URL na .bin soubor nove firmware
  otaFirmwareUrl = param.asStr();
  if (WiFi.status() == WL_CONNECTED) {
    logToTerminal("Prijata URL: " + otaFirmwareUrl);
  }
}

boolean ota_potvrzeni = false;

BLYNK_WRITE(V99) { // "Spustit OTA" - spusti jen potvrzovaci dotaz, stejny vzor jako u resetu
  ota_potvrzeni = (param.asInt() == 1);
  if (WiFi.status() == WL_CONNECTED) Blynk.virtualWrite(V100, ota_potvrzeni);
}

BLYNK_WRITE(V100) { // tlacitko ANO - skutecne spusti OTA aktualizaci
  if (ota_potvrzeni) {
    performOTAUpdate(otaFirmwareUrl);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.virtualWrite(V99, 0);
    Blynk.virtualWrite(V100, 0);
  }
  ota_potvrzeni = false;
}
