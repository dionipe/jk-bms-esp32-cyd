/**
 * ESP32-2432S028R - JK BMS Bluetooth Monitor
 *
 * Fitur:
 *  - Scan & koneksi otomatis ke BMS JK (Jikong) via BLE
 *  - Tampilkan data BMS pada layar TFT ILI9341 2.8" (240x320)
 *  - Halaman utama : SOC, Voltase, Arus, Suhu, Siklus, Status
 *  - Halaman cell  : Voltase per-sel dengan warna kondisi
 *  - Ganti halaman dengan sentuhan layar
 *
 * Pinout ESP32-2432S028R:
 *  Display (ILI9341) : MOSI=13, MISO=12, CLK=14, CS=15, DC=2, BL=21
 *  Touch (XPT2046)   : MOSI=32, MISO=39, CLK=25, CS=33, IRQ=36
 *  RGB LED           : R=4, G=16, B=17
 *
 * Kalibrasi protokol JK BMS:
 *  Jika nilai voltase/arus tidak sesuai, sesuaikan konstanta SCALE di bawah.
 *  Aktifkan DEBUG_PROTOCOL=1 untuk melihat raw TLV via Serial Monitor.
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <NimBLEDevice.h>
#include <XPT2046_Touchscreen.h>

// ============================================================
// Kalibrasi Protokol (sesuaikan jika nilai tampil tidak tepat)
// ============================================================
#define TOTAL_V_DIVISOR     1000.0f  // raw tag 0x83 (4 bytes) dalam mV
#define CURRENT_DIVISOR     1000.0f  // raw tag 0x84 (4 bytes, signed) dalam mA
#define CAPACITY_DIVISOR    1000.0f  // raw tag 0x88 (4 bytes) dalam mAh → Ah
#define REMAIN_DIVISOR      1000.0f  // raw tag 0xAA (4 bytes) dalam mAh → Ah
#define TEMP_OFFSET         2731     // raw - 2731 = tenths of °C
#define DEBUG_PROTOCOL      0        // 1 = cetak raw TLV ke Serial

// ============================================================
// Pin
// ============================================================
#define LED_RED     4
#define LED_GREEN   16
#define LED_BLUE    17
#define TOUCH_CS    33
#define TOUCH_IRQ   36

// ============================================================
// BLE - JK BMS
// ============================================================
#define JK_SERVICE_UUID   "0000ffe0-0000-1000-8000-00805f9b34fb"
#define JK_NOTIFY_UUID    "0000ffe1-0000-1000-8000-00805f9b34fb"  // FFE1: NOTIFY
#define JK_WRITE_UUID     "0000ffe2-0000-1000-8000-00805f9b34fb"  // FFE2: WRITE
#define JK_LEGACY_SERVICE_UUID "0000ff10-0000-1000-8000-00805f9b34fb"
// Fallback: beberapa model gabung write+notify di FFE1
#define JK_CHAR_UUID      JK_NOTIFY_UUID

// Perintah request data BMS — old protocol 55 AA EB 90
// Checksum = sum(bytes[0..18]) mod 256
static const uint8_t CMD_CELL_INFO[20] = {  // 0x96 = cell info, chk = 0x10
    0xAA, 0x55, 0x90, 0xEB, 0x96, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10
};
static const uint8_t CMD_DEV_INFO[20] = {   // 0x97 = device info, chk = 0x11
    0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x11
};
// ANT BMS status request: 7E A1 01 00 00 BE [CRC_LO CRC_HI] AA 55
// CRC-16/Modbus over bytes [1..5] = A1 01 00 00 BE → 0x5518
static const uint8_t CMD_ANT_STATUS[10] = {
    0x7E,0xA1,0x01,0x00,0x00,0xBE,0x18,0x55,0xAA,0x55};

static const uint8_t LEGACY_PROBE_COMMANDS[] = {0x95, 0x98};

struct LegacyProbeFormat {
    uint8_t length;
    uint32_t value;
    const char* name;
};

struct LegacyRequestFamily {
    uint8_t header0;
    uint8_t header1;
    uint8_t header2;
    uint8_t header3;
    const char* name;
};

static const LegacyProbeFormat LEGACY_PROBE_FORMATS[] = {
    {0x00, 0x00000000UL, "L0V0"},
    {0x01, 0x00000001UL, "L1V1"},
    {0x04, 0x00000000UL, "L4V0"},
};

static const LegacyRequestFamily LEGACY_REQUEST_FAMILIES[] = {
    {0xAA, 0x55, 0x90, 0xEB, "AA55-90EB"},
    {0x55, 0xAA, 0xEB, 0x90, "55AA-EB90"},
};

struct LegacyShortProbe {
    uint8_t address;
    uint8_t command;
    uint16_t value;
    const char* name;
};

static const LegacyShortProbe LEGACY_SHORT_PROBES[] = {
    {0x81, 0xFF, 0x0000, "RS485-ALLDATA-A129"},
    {0x00, 0xFF, 0x0000, "RS485-ALLDATA-A0"},
    {0x01, 0xFF, 0x0000, "RS485-ALLDATA-A1"},
    {0x02, 0xFF, 0x0000, "RS485-ALLDATA-A2"},
    {0x10, 0xFF, 0x0000, "RS485-ALLDATA-A16"},
};

// ============================================================
// Data BMS
// ============================================================
struct BMSData {
    uint8_t  num_cells    = 0;
    uint16_t cell_mv[24]  = {};   // voltase sel (mV)
    uint16_t cell_res_mohm[24] = {}; // resistansi internal sel (mΩ)
    bool     cell_res_valid = false; // true jika data resistansi diterima
    float    total_v      = 0.0f; // voltase total (V)
    float    current_a    = 0.0f; // arus (A), + = charge, - = discharge
    uint8_t  soc          = 0;    // State of Charge (%)
    float    temp_mos     = 0.0f; // suhu MOSFET (°C)
    float    temp_bat1    = 0.0f; // suhu sensor baterai 1 (°C)
    float    temp_bat2    = 0.0f; // suhu sensor baterai 2 (°C)
    bool     temp_bat1_valid = false; // true jika data sensor BAT1 diterima
    bool     temp_bat2_valid = false; // true jika data sensor BAT2 diterima
    uint32_t cycle_cnt    = 0;    // siklus charge
    float    cap_ah       = 0.0f; // kapasitas total (Ah)
    float    remain_ah    = 0.0f; // kapasitas tersisa (Ah)
    uint32_t alarm_flags  = 0;
    bool     charging     = false;
    bool     discharging  = false;
    bool     balancing    = false;
    char     bms_name[17] = "JK BMS";
    bool     valid        = false;
    uint32_t last_update  = 0;
};

// ============================================================
// Globals
// ============================================================
TFT_eSPI tft = TFT_eSPI();

// Touch pada VSPI (terpisah dari display HSPI)
SPIClass vspi(VSPI);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

BMSData bms;

// BLE
enum BMSType  { BMS_JK = 0, BMS_ANT = 1 };
enum BLEState { BLE_IDLE, BLE_SCANNING, BLE_SELECT, BLE_CONNECTING, BLE_CONNECTED };
volatile BLEState bleState  = BLE_IDLE;
NimBLEClient*                  pClient      = nullptr;
NimBLERemoteCharacteristic*    pJKChar      = nullptr;  // FFE2: write w/o response
NimBLERemoteCharacteristic*    pJKCharAlt   = nullptr;  // FFE1: write w/ response
NimBLERemoteCharacteristic*    pJKCharLegacy = nullptr; // FF10 write fallback
NimBLERemoteCharacteristic*    pJKCharLegacyAlt = nullptr; // FF10 notify fallback
NimBLERemoteCharacteristic*    pJKCharLegacyExtra = nullptr; // FF10 secondary char for probe/subscription

// Daftar perangkat hasil scan untuk layar pilih
#define MAX_SCAN_DEVICES 5
struct ScanDevice {
    NimBLEAddress bleAddr;
    char name[20];
    char addr[18];
    BMSType bmsType;
};
ScanDevice    scanDevices[MAX_SCAN_DEVICES];
uint8_t       scanDeviceCount  = 0;
NimBLEAddress selectedBLEAddr;
bool          scanStarted      = false;
uint32_t      lastScanRestartMs = 0;
bool          bmsScreenReady   = false;  // sudah full-redraw setelah data pertama
BMSType       connectedBmsType  = BMS_JK;
BMSType       selectedDeviceType = BMS_JK;

// Buffer penerimaan BLE (multi-packet)
uint8_t  rxBuf[640];
uint16_t rxLen = 0;
volatile bool newDataReady = false;

// Display
uint8_t  currentPage    = 0;  // 0 = main, 1 = cells
bool     needFullRedraw = true;
uint32_t lastRequestMs  = 0;
uint32_t lastTouchMs    = 0;
uint32_t lastBlinkMs    = 0;
bool     btIconState    = false;

#define REQUEST_INTERVAL_MS  2000
#define TOUCH_DEBOUNCE_MS    600

// LED
uint32_t ledMs    = 0;
uint8_t  ledPhase = 0;

// AT mode detection / recovery
uint16_t atRxCount      = 0;    // AT\r\n berturutan tanpa frame
bool     atEscapeSent   = false; // sudah coba AT+EXIT recovery
uint32_t lastAtEscapeMs = 0;
uint32_t lastNotifyMs   = 0;     // waktu notify terakhir diterima
uint32_t lastFrameMs    = 0;     // waktu frame BMS valid terakhir diterima
uint8_t  noDataRetryCount = 0;   // retry init jika connect tapi tidak ada data sama sekali
bool     legacyAckSeen  = false; // FF10 merespons ACK pendek, gunakan polling legacy
uint8_t  legacyProbeIndex = 0;   // probe opcode alternatif untuk BMS legacy ACK-only
uint8_t  legacyFormatProbeIndex = 0; // probe variasi length/value untuk BMS legacy ACK-only
uint8_t  legacyFamilyProbeIndex = 0; // probe variasi header request untuk BMS legacy ACK-only
uint8_t  legacyShortProbeIndex = 0; // probe frame pendek gaya RS485 untuk BMS legacy ACK-only
bool     legacyUseShortNext = true; // ACK-only mode: kirim short probe dulu agar cepat dapat EB90
uint8_t  legacyJkSequence = 1; // sequence counter untuk frame JK 20-byte yang dibangun dinamis

// ============================================================
// LED Helper (common anode = aktif LOW)
// ============================================================
void setLED(bool r, bool g, bool b) {
    digitalWrite(LED_RED,   r ? LOW : HIGH);
    digitalWrite(LED_GREEN, g ? LOW : HIGH);
    digitalWrite(LED_BLUE,  b ? LOW : HIGH);
}

// ============================================================
// Ukuran field tiap tag JK BMS (byte SETELAH byte tag)
// -1 = variable (diawali 1-byte panjang data)
//  0 = tidak diketahui → hentikan parsing
// ============================================================
static int tagSize(uint8_t tag) {
    switch (tag) {
        case 0x79: return -1;
        case 0x80: case 0x81: case 0x82: return 2;   // suhu
        case 0x83: return 4;                           // voltase total
        case 0x84: return 4;                           // arus
        case 0x85: return 1;                           // SOC
        case 0x86: return 1;                           // jumlah sensor suhu
        case 0x87: return 4;                           // siklus
        case 0x88: return 4;                           // kapasitas
        case 0x89: return 4;                           // alarm flags
        case 0x8A: return 2;                           // status (CHG/DCH/BAL)
        case 0x8B: return 2;
        case 0x8C: return 2;                           // arus balance
        case 0x8D: return 2;
        case 0x8E: case 0x8F: case 0x90: return 4;
        case 0x91: case 0x92: case 0x93: return 4;
        case 0x94: case 0x95: return 2;
        case 0x96: case 0x97: case 0x98: return 4;
        case 0x99: case 0x9A: return 2;
        case 0x9B: return 4;
        case 0x9C: return 2;
        case 0x9D: return 1;
        case 0x9E: case 0x9F: return 2;
        case 0xA0: case 0xA1: case 0xA2: case 0xA3: case 0xA4: return 2;
        case 0xA5: case 0xA6: case 0xA7: case 0xA8: return 2; // signed
        case 0xA9: return 1;                           // jumlah sel aktual
        case 0xAA: return 4;                           // kapasitas tersisa
        case 0xAB: return 1;
        case 0xAC: return 1;
        case 0xAD: return 2;
        case 0xAE: return 1;
        case 0xAF: return 1;
        case 0xB0: return 2;
        case 0xB1: return 1;
        case 0xB2: return 16;
        case 0xB3: return 1;
        case 0xB4: return 8;
        case 0xB5: return 6;
        case 0xB6: return 4;
        case 0xB7: return 16;
        case 0xB8: return 1;
        case 0xB9: return 4;
        case 0xBA: return 16;                          // nama BMS
        case 0xBB: return 4;
        case 0xBC: return 4;
        default:   return 0;
    }
}

static float parseCapacityFromName(const char* name) {
    if (!name || !name[0]) return 0.0f;
    for (int i = 0; name[i]; i++) {
        if (name[i] >= '0' && name[i] <= '9') {
            int j = i;
            int v = 0;
            while (name[j] >= '0' && name[j] <= '9') {
                v = v * 10 + (name[j] - '0');
                j++;
            }
            char a = name[j];
            char h = name[j + 1];
            if ((a == 'A' || a == 'a') && (h == 'H' || h == 'h') && v >= 10 && v <= 500) {
                return (float)v;
            }
        }
    }
    return 0.0f;
}

static uint16_t calcCellDeltaMv(const BMSData& d) {
    if (d.num_cells < 2) return 0;
    uint16_t mn = d.cell_mv[0], mx = d.cell_mv[0];
    for (int i = 1; i < d.num_cells; i++) {
        if (d.cell_mv[i] < mn) mn = d.cell_mv[i];
        if (d.cell_mv[i] > mx) mx = d.cell_mv[i];
    }
    return mx - mn;
}

static void drawStatusBadges(const BMSData& data) {
    const uint16_t BG = tft.color565(0, 5, 20);
    int statusX = 110;
    tft.fillRect(110, 172, 128, 20, BG);
    tft.setTextSize(1);

    if (data.charging) {
        tft.fillRoundRect(statusX, 172, 38, 18, 4, tft.color565(0, 45, 15));
        tft.drawRoundRect(statusX, 172, 38, 18, 4, TFT_GREEN);
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(statusX + 10, 178);
        tft.print("CHG");
        statusX += 44;
    }
    if (data.discharging) {
        tft.fillRoundRect(statusX, 172, 38, 18, 4, tft.color565(55, 15, 0));
        tft.drawRoundRect(statusX, 172, 38, 18, 4, TFT_ORANGE);
        tft.setTextColor(TFT_ORANGE);
        tft.setCursor(statusX + 10, 178);
        tft.print("DCH");
        statusX += 44;
    }
    if (data.balancing) {
        tft.fillRoundRect(statusX, 172, 38, 18, 4, tft.color565(0, 12, 40));
        tft.drawRoundRect(statusX, 172, 38, 18, 4, TFT_CYAN);
        tft.setTextColor(TFT_CYAN);
        tft.setCursor(statusX + 10, 178);
        tft.print("BAL");
    }
}

static float smoothCurrentReading(float previousCurrent,
                                  float currentReading,
                                  bool previousCharging,
                                  bool previousDischarging,
                                  bool charging,
                                  bool discharging) {
    if (currentReading > -0.05f && currentReading < 0.05f) return 0.0f;

    bool sameMode = (previousCharging == charging) &&
                    (previousDischarging == discharging);
    bool previousActive = previousCharging || previousDischarging;
    if (!sameMode || !previousActive) return currentReading;

    // Light IIR smoothing: keep movement visible but damp small jitter.
    return previousCurrent * 0.65f + currentReading * 0.35f;
}

// ============================================================
// Parse frame JK BMS
// ============================================================
bool parseJKFrame(const uint8_t* buf, uint16_t len) {
    if (len < 12) return false;
    if (buf[0] != 0x4E || buf[1] != 0x57) return false;
    // buf[8] = record type (0x01/0x03/0x06 tergantung firmware BMS)
    Serial.printf("[BMS] Frame %d byte, tipe=0x%02X counter=0x%02X\n",
                  len, buf[8], buf[9]);

    BMSData d;
    const uint8_t* p   = buf + 10;          // awal data TLV (setelah 4E 57 LEN LEN ADDR4 TYPE CTR)
    const uint8_t* end = buf + len - 4;     // sebelum 4 byte checksum

    while (p < end) {
        uint8_t tag = *p++;
        int sz = tagSize(tag);

        if (sz == 0) break;   // tag tidak dikenal, hentikan

        // Pastikan data cukup
        int needed = (sz < 0) ? 1 : sz;
        if (p + needed > end) break;

        if (sz == -1) {
            // Variable length: 1-byte panjang data
            uint8_t dlen = *p++;
            if (tag == 0x79) {
                // Voltase per sel: [index(1B) + mv(2B)] per sel
                uint8_t ncells = dlen / 3;
                if (ncells > 24) ncells = 24;
                d.num_cells = ncells;
                for (uint8_t i = 0; i < ncells && p + 2 <= end; i++) {
                    p++;  // skip index
                    d.cell_mv[i] = ((uint16_t)p[0] << 8) | p[1];
                    p += 2;
                }
            } else {
                p += dlen;  // skip data tag variable lainnya
            }
        } else {
#if DEBUG_PROTOCOL
            Serial.printf("  [0x%02X] sz=%d : ", tag, sz);
            for (int i = 0; i < sz; i++) Serial.printf("%02X ", p[i]);
            Serial.println();
#endif
            switch (tag) {
                case 0x80: {
                    uint16_t r = ((uint16_t)p[0]<<8)|p[1];
                    d.temp_mos = (r > TEMP_OFFSET) ? (r - TEMP_OFFSET) / 10.0f : 0.0f;
                    break;
                }
                case 0x81: {
                    uint16_t r = ((uint16_t)p[0]<<8)|p[1];
                    if (r > TEMP_OFFSET) {
                        d.temp_bat1 = (r - TEMP_OFFSET) / 10.0f;
                        d.temp_bat1_valid = true;
                    }
                    Serial.printf("[TEMP] 0x81 raw=0x%04X → %.1f°C\n", r, d.temp_bat1);
                    break;
                }
                case 0x82: {
                    uint16_t r = ((uint16_t)p[0]<<8)|p[1];
                    if (r > TEMP_OFFSET) {
                        d.temp_bat2 = (r - TEMP_OFFSET) / 10.0f;
                        d.temp_bat2_valid = true;
                    }
                    Serial.printf("[TEMP] 0x82 raw=0x%04X → %.1f°C\n", r, d.temp_bat2);
                    break;
                }
                case 0x83: {
                    uint32_t r = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                                 ((uint32_t)p[2]<<8)|p[3];
                    d.total_v = r / TOTAL_V_DIVISOR;
                    break;
                }
                case 0x84: {
                    int32_t r = (int32_t)(((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                                          ((uint32_t)p[2]<<8)|p[3]);
                    d.current_a = r / CURRENT_DIVISOR;
                    break;
                }
                case 0x85: d.soc = p[0]; break;
                case 0x87: {
                    d.cycle_cnt = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                                  ((uint32_t)p[2]<<8)|p[3];
                    break;
                }
                case 0x88: {
                    uint32_t r = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                                 ((uint32_t)p[2]<<8)|p[3];
                    d.cap_ah = r / CAPACITY_DIVISOR;
                    break;
                }
                case 0x89: {
                    d.alarm_flags = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                                    ((uint32_t)p[2]<<8)|p[3];
                    break;
                }
                case 0x8A: {
                    uint16_t s = ((uint16_t)p[0]<<8)|p[1];
                    d.charging    = (s >> 0) & 1;
                    d.discharging = (s >> 1) & 1;
                    d.balancing   = (s >> 2) & 1;
                    break;
                }
                case 0xA9: {
                    // Override jumlah sel jika field ini ada
                    if (p[0] > 0 && p[0] <= 24) d.num_cells = p[0];
                    break;
                }
                case 0xAA: {
                    uint32_t r = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                                 ((uint32_t)p[2]<<8)|p[3];
                    d.remain_ah = r / REMAIN_DIVISOR;
                    break;
                }
                case 0xBA: {
                    // Nama BMS (16 byte ASCII)
                    int copyLen = sz < 16 ? sz : 16;
                    memcpy(d.bms_name, p, copyLen);
                    d.bms_name[copyLen] = '\0';
                    // Hilangkan karakter non-printable
                    for (int i = 0; i < copyLen; i++) {
                        if (d.bms_name[i] < 0x20 || d.bms_name[i] > 0x7E)
                            d.bms_name[i] = '\0';
                    }
                    break;
                }
                default: break;
            }
            p += sz;
        }
    }

    // Fallback: hitung total voltase dari jumlah sel
    if (d.total_v < 0.5f && d.num_cells > 0) {
        float sum = 0;
        for (int i = 0; i < d.num_cells; i++) sum += d.cell_mv[i];
        d.total_v = sum / 1000.0f;
    }

    // Fallback: hitung sisa kapasitas dari SOC
    if (d.remain_ah < 0.01f && d.cap_ah > 0.01f) {
        d.remain_ah = d.cap_ah * d.soc / 100.0f;
    }

    d.valid       = (d.num_cells > 0 || d.total_v > 0.5f || d.soc > 0);
    d.last_update = millis();
    if (d.valid) {
        bms = d;
        Serial.printf("[BMS] SOC=%d%% V=%.2fV I=%.2fA Cells=%d T_MOS=%.1f°C T_BAT1=%s T_BAT2=%s Alarm=0x%08X\n",
                      bms.soc, bms.total_v, bms.current_a, bms.num_cells,
                      bms.temp_mos,
                      bms.temp_bat1_valid ? String(bms.temp_bat1, 1).c_str() : "---",
                      bms.temp_bat2_valid ? String(bms.temp_bat2, 1).c_str() : "---",
                      bms.alarm_flags);
    } else {
        Serial.printf("[BMS] Parse selesai tapi valid=false (cells=%d V=%.2f SOC=%d)\n",
                      d.num_cells, d.total_v, d.soc);
    }
    return d.valid;
}

// ============================================================
// Parse frame protokol lama JK BMS (55 AA EB 90) — 300 byte tetap
// Struktur: byte[0..3]=header, byte[4]=type, byte[5]=counter,
//           byte[6..]=data LE, byte[299]=CRC (sum[0..298] mod 256)
// Type: 0x01=settings, 0x02=cell info, 0x03=device info
// ============================================================
bool parseOldProtoFrame(const uint8_t* buf, uint16_t len) {
    if (len < 300) return false;
    if (buf[0]!=0x55||buf[1]!=0xAA||buf[2]!=0xEB||buf[3]!=0x90) return false;

    // Verifikasi CRC
    uint8_t crc = 0;
    for (int i = 0; i < 299; i++) crc += buf[i];
    if (crc != buf[299]) {
        Serial.printf("[OldProto] CRC FAIL type=0x%02X: hitung=0x%02X terima=0x%02X\n",
                      buf[4], crc, buf[299]);
        return false;
    }

    uint8_t frame_type = buf[4];
    Serial.printf("[OldProto] type=0x%02X counter=%d CRC OK\n", frame_type, buf[5]);

    auto get16 = [&](int i) -> uint16_t {
        return ((uint16_t)buf[i+1] << 8) | buf[i];
    };
    auto get32 = [&](int i) -> uint32_t {
        return ((uint32_t)buf[i+3]<<24)|((uint32_t)buf[i+2]<<16)|
               ((uint32_t)buf[i+1]<<8)|buf[i];
    };

    if (frame_type == 0x02) {
        // Frame info sel (cell info) — semua nilai little-endian.
        // Jangan mewarisi field dinamis dari frame sebelumnya agar nilai arus/daya/status
        // tidak menjadi stale saat satu frame tidak membawa field yang sama.
        BMSData d = {};
        strncpy(d.bms_name, bms.bms_name, sizeof(d.bms_name) - 1);
        d.bms_name[sizeof(d.bms_name) - 1] = '\0';
        d.valid = false;

        // Voltase sel: byte[6+i*2 .. 7+i*2] LE uint16 (mV), zero = tidak ada
        uint8_t ncells = 0;
        uint16_t tempMv[24] = {};
        for (int i = 0; i < 24; i++) {
            uint16_t mv = get16(6 + i*2);
            tempMv[i] = mv;
            if (mv > 0) ncells++;
        }
        if (ncells > 0) {
            d.num_cells = ncells;
            int idx = 0;
            for (int i = 0; i < 24 && idx < 24; i++)
                if (tempMv[i] > 0) d.cell_mv[idx++] = tempMv[i];
        }

        // Resistansi internal sel: scan offset 54..110 untuk cari posisi data yang valid.
        // Berbeda firmware JK BMS (standard vs variant) menyimpan resistance di offset berbeda.
        // "Valid" = uint16 LE bernilai 50-2000 mΩ (rentang wajar resistansi sel Li-ion).
        {
            // Auto-detect: cari startOffset yang paling banyak sel-aktif masuk rentang [50,2000]
            int bestOff   = 64;   // default referensi ESPHome JK02 24S
            int bestScore = 0;
            for (int startOff = 54; startOff <= 108; startOff += 2) {
                int score = 0;
                for (int i = 0; i < 24; i++) {
                    if (tempMv[i] == 0) continue;
                    uint16_t r = get16(startOff + i * 2);
                    if (r >= 50 && r <= 2000) score++;
                }
                if (score > bestScore) {
                    bestScore = score;
                    bestOff   = startOff;
                }
            }

            // Post-correction: cek C01 vs rata-rata C02+. Jika C01 outlier (di luar ±50%
            // mean sel lain), berarti bestOff menunjuk 2 byte TERLALU AWAL — boundary field
            // sebelumnya kebetulan masuk rentang valid. Geser +2 jika C01 alternatif konsisten.
            if (ncells >= 3) {
                long sum = 0; int cnt = 0; int pos = 0;
                for (int i = 0; i < 24 && pos < ncells; i++) {
                    if (tempMv[i] == 0) continue;
                    if (pos >= 1) {
                        uint16_t v = get16(bestOff + i * 2);
                        if (v >= 50 && v <= 2000) { sum += v; cnt++; }
                    }
                    pos++;
                }
                if (cnt >= 2) {
                    long     mean    = sum / cnt;
                    uint16_t c01_cur = get16(bestOff);
                    uint16_t c01_alt = get16(bestOff + 2);
                    bool cur_bad = (c01_cur < (uint16_t)(mean / 2))
                                || ((long)c01_cur > mean * 2);
                    bool alt_ok  = (c01_alt >= (uint16_t)(mean / 2))
                                && ((long)c01_alt <= mean * 2)
                                && (c01_alt >= 50) && (c01_alt <= 2000);
                    if (cur_bad && alt_ok) {
                        Serial.printf("[OldProto] C01 outlier (val=%u vs mean=%ld),"
                                      " adjust offset %d->%d\n",
                                      c01_cur, mean, bestOff, bestOff + 2);
                        bestOff += 2;
                    }
                }
            }

            Serial.printf("[OldProto] finalOffset=%d score=%d/%d\n", bestOff, bestScore, ncells);

            bool anyRes = false;
            int resIdx  = 0;
            for (int i = 0; i < 24; i++) {
                if (tempMv[i] == 0) continue;
                uint16_t r = get16(bestOff + i * 2);
                d.cell_res_mohm[resIdx++] = r;
                if (r > 0) anyRes = true;
            }
            // Valid jika minimal setengah sel punya nilai plausible
            d.cell_res_valid = (bestScore >= (ncells > 0 ? (ncells + 1) / 2 : 1));
            if (d.cell_res_valid) {
                Serial.printf("[OldProto] Resistansi: off=%d C01=%u C02=%u C03=%u mOhm\n",
                              bestOff,
                              resIdx > 0 ? d.cell_res_mohm[0] : 0,
                              resIdx > 1 ? d.cell_res_mohm[1] : 0,
                              resIdx > 2 ? d.cell_res_mohm[2] : 0);
            }
        }

        // Voltase total: byte[118..121] LE uint32 × 0.001V
        float tv = (float)get32(118) * 0.001f;
        if (tv > 0.5f) d.total_v = tv;

        // Arus: default layout referensi memakai 126..129, tetapi firmware varian
        // user menyimpan arus charge/discharge di 194..195 (mA).
        d.current_a = (float)(int32_t)get32(126) * 0.001f;

        // Layout firmware varian yang terverifikasi dari dump user:
        // 144..145 = MOS, 158..161 = current (uint32, mA)
        // 162..163 = BAT1, 164..165 = BAT2
        // 170..171 = balance current (int16, mA)
        // 173      = SOC
        // 174..177 = remain capacity (mAh)
        // 178..181 = total capacity (mAh)
        // 182..185 = cycle count
        // 186..189 = cycle capacity (mAh)
        // 198      = charging, 199 = discharging
        bool variantLayout = (buf[173] > 0 && buf[173] <= 100 &&
                              get32(174) > 1000 && get32(178) > 1000);

        if (variantLayout) {
            d.temp_mos  = (float)(int16_t)get16(144) * 0.1f;
            int16_t b1 = (int16_t)get16(162);
            int16_t b2 = (int16_t)get16(164);
            d.temp_bat1 = b1 * 0.1f;
            d.temp_bat2 = b2 * 0.1f;
            d.temp_bat1_valid = (b1 > -400 && b1 != 0 && b1 < 1500);
            d.temp_bat2_valid = (b2 > -400 && b2 != 0 && b2 < 1500);
            d.current_a = (float)(int32_t)get32(158) * 0.001f;
            d.soc       = buf[173];
            d.remain_ah = (float)get32(174) * 0.001f;
            d.cap_ah    = (float)get32(178) * 0.001f;
            d.cycle_cnt = get32(182);
            d.charging    = (buf[198] != 0);
            d.discharging = (buf[199] != 0);
            float balA = (float)(int16_t)get16(170) * 0.001f;
            d.balancing = (balA > 0.01f || buf[241] != 0);
            if (d.charging && !d.discharging) {
                if (d.current_a < 0.0f) d.current_a = -d.current_a;
            } else if (d.discharging && !d.charging) {
                if (d.current_a > 0.0f) d.current_a = -d.current_a;
            } else if (!d.charging && !d.discharging) {
                d.current_a = 0.0f;
            }
            if (d.current_a > -0.05f && d.current_a < 0.05f) d.current_a = 0.0f;
        } else {
            // Layout referensi JK lama / ESPHome
            int16_t b1 = (int16_t)get16(130);
            int16_t b2 = (int16_t)get16(132);
            d.temp_bat1 = b1 * 0.1f;
            d.temp_bat2 = b2 * 0.1f;
            d.temp_bat1_valid = (b1 > -400 && b1 != 0 && b1 < 1500);
            d.temp_bat2_valid = (b2 > -400 && b2 != 0 && b2 < 1500);
            d.temp_mos  = (float)(int16_t)get16(134) * 0.1f;
            d.soc = buf[141];
            float rem = (float)get32(142) * 0.001f;
            if (rem > 0.001f) d.remain_ah = rem;
            float cap = (float)get32(146) * 0.001f;
            if (cap > 0.001f) d.cap_ah = cap;
            d.cycle_cnt = get32(150);
            d.charging    = (buf[166] != 0);
            d.discharging = (buf[167] != 0);
            d.balancing   = (buf[140] != 0 || buf[169] != 0);
        }

        // Alarm: byte[136..137] (big-endian di ESPHome: buf[136]<<8 | buf[137])
        d.alarm_flags = ((uint32_t)buf[136] << 8) | buf[137];

        // Fallback suhu BAT: beberapa varian firmware JK menyimpan suhu baterai
        // di offset lebih tinggi (248/254) dalam format LE uint16 × 0.1°C.
        // Digunakan hanya jika offset standar tidak menghasilkan nilai valid.
        if (!d.temp_bat1_valid && len >= 250) {
            uint16_t r = get16(248);
            if (r > 0 && r < 1000) {   // 0.1 – 99.9°C
                d.temp_bat1 = r * 0.1f;
                d.temp_bat1_valid = true;
            }
        }
        if (!d.temp_bat2_valid && len >= 256) {
            uint16_t r = get16(254);
            if (r > 0 && r < 1000) {
                d.temp_bat2 = r * 0.1f;
                d.temp_bat2_valid = true;
            }
        }

        // Validasi voltase total terhadap jumlah sel. Pada beberapa BMS old-protocol,
        // offset total_v berbeda walaupun layout field lain sama, jadi nilai mentah bisa
        // sangat tidak masuk akal. Dalam kasus itu, pakai jumlah voltase sel sebagai sumber
        // kebenaran karena data per-sel sudah terbukti konsisten.
        if (d.num_cells > 0) {
            float sum = 0.0f;
            for (int i = 0; i < d.num_cells; i++) sum += d.cell_mv[i];
            float cellSumV = sum / 1000.0f;
            float diffV = d.total_v - cellSumV;
            if (diffV < 0.0f) diffV = -diffV;
            float maxReasonableV = (float)d.num_cells * 4.5f;

            if (d.total_v < 0.5f || d.total_v > maxReasonableV || diffV > 5.0f) {
                d.total_v = cellSumV;
            }
        }
        // Fallback: ambil kapasitas dari nama device, contoh JK_Ion_90Ah
        if (d.cap_ah < 0.001f) {
            float nameCap = parseCapacityFromName(bms.bms_name);
            if (nameCap > 0.001f) d.cap_ah = nameCap;
        }
        // Fallback: kapasitas tersisa dari SOC
        if (d.remain_ah < 0.001f && d.cap_ah > 0.001f && d.soc > 0)
            d.remain_ah = d.cap_ah * d.soc / 100.0f;
        // Fallback: SOC dari remain/cap (jika byte[141] = 0 tapi kapasitas ada)
        if (d.soc == 0 && d.cap_ah > 0.1f && d.remain_ah > 0.001f) {
            d.soc = (uint8_t)(d.remain_ah / d.cap_ah * 100.0f + 0.5f);
            if (d.soc > 100) d.soc = 100;
        }

        if (d.current_a > -0.05f && d.current_a < 0.05f) {
            d.current_a    = 0.0f;
            d.charging     = false;
            d.discharging  = false;
        }

        if (bms.valid) {
            d.current_a = smoothCurrentReading(bms.current_a, d.current_a,
                                               bms.charging, bms.discharging,
                                               d.charging, d.discharging);
        }

        // Sinkronkan badge CHG/DCH dengan tanda arus final yang ditampilkan.
        // Untuk user, tanda '+' harus berarti charge saja, dan '-' berarti discharge saja.
        if (d.current_a > 0.05f) {
            d.charging = true;
            d.discharging = false;
        } else if (d.current_a < -0.05f) {
            d.charging = false;
            d.discharging = true;
        } else {
            d.charging = false;
            d.discharging = false;
            d.current_a = 0.0f;
        }

        d.valid       = (d.num_cells > 0 || d.total_v > 0.5f || d.remain_ah > 0.1f);
        d.last_update = millis();
        if (d.valid) {
            bms = d;
            Serial.printf("[OldProto] Sel=%d SOC=%d%% V=%.2fV I=%.3fA T_MOS=%.1f°C "
                          "Cap=%.1fAh Rem=%.1fAh Siklus=%lu\n",
                          bms.num_cells, bms.soc, bms.total_v, bms.current_a,
                          bms.temp_mos, bms.cap_ah, bms.remain_ah, bms.cycle_cnt);
        } else {
            Serial.printf("[OldProto] type=0x02 data tidak valid (cells=%d V=%.2f SOC=%d)\n",
                          d.num_cells, d.total_v, d.soc);
        }
        return d.valid;
    }

    if (frame_type == 0x01) {
        // Frame settings — log saja
        Serial.println("[OldProto] Settings frame (0x01) diterima, tidak diparse");
        return false;
    }

    if (frame_type == 0x03) {
        // Frame device info
        char vendor[17] = {}, devname[17] = {};
        memcpy(vendor,  &buf[6],  16);
        memcpy(devname, &buf[46], 16);
        Serial.printf("[OldProto] DevInfo: Vendor=%.16s Name=%.16s\n", vendor, devname);
        // Update nama BMS jika ASCII valid
        const char* src = (devname[0] >= 0x20 && devname[0] < 0x7F) ? devname : vendor;
        bool valid = (src[0] >= 0x20 && src[0] < 0x7F);
        for (int i = 0; i < 16 && src[i]; i++)
            if (src[i] < 0x20 || src[i] > 0x7E) { valid = false; break; }
        if (valid) { strncpy(bms.bms_name, src, 16); bms.bms_name[16] = '\0'; }
        return false;
    }

    Serial.printf("[OldProto] Type 0x%02X tidak dikenal\n", frame_type);
    return false;
}

// ============================================================
// ANT BMS: CRC-16/MODBUS helper (poly 0xA001, init 0xFFFF)
// ============================================================
static uint16_t crc16Ant(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001u : (crc >> 1);
    }
    return crc;
}

// ============================================================
// JK Legacy RS485: Parse upload frame (EB 90 ...), fixed 74 bytes
// Ref request: 55 AA <addr> FF 00 00 <checksum>
// ============================================================
static bool parseLegacyRs485Frame(const uint8_t* buf, uint16_t len) {
    if (len < 74) return false;
    if (buf[0] != 0xEB || buf[1] != 0x90) return false;

    uint8_t crc = 0;
    for (int i = 0; i < 73; i++) crc = (uint8_t)(crc + buf[i]);
    if (crc != buf[73]) {
        Serial.printf("[RS485] CRC FAIL calc=%02X recv=%02X\n", crc, buf[73]);
        return false;
    }

    auto be16 = [&](int i) -> uint16_t {
        return ((uint16_t)buf[i] << 8) | buf[i + 1];
    };

    BMSData d = bms;
    strncpy(d.bms_name, "JK RS485", sizeof(d.bms_name) - 1);
    d.bms_name[sizeof(d.bms_name) - 1] = '\0';

    d.total_v = (float)be16(4) * 0.01f;   // unit 10mV
    uint8_t recognizedCells = buf[8];
    if (recognizedCells > 24) recognizedCells = 24;
    d.num_cells = recognizedCells;

    d.balancing   = ((buf[11] & 0x03) != 0);
    d.alarm_flags = buf[12];

    // Cell voltages start at offset 23, each 2 bytes (big-endian, mV).
    for (uint8_t i = 0; i < d.num_cells; i++) {
        int off = 23 + i * 2;
        if (off + 1 >= 73) break;
        d.cell_mv[i] = be16(off);
    }

    if (d.total_v < 0.5f && d.num_cells > 0) {
        float sum = 0.0f;
        for (uint8_t i = 0; i < d.num_cells; i++) sum += d.cell_mv[i];
        d.total_v = sum / 1000.0f;
    }

    d.current_a = 0.0f;
    d.charging = false;
    d.discharging = false;
    d.valid = (d.num_cells > 0 || d.total_v > 0.5f);
    d.last_update = millis();

    if (d.valid) {
        bms = d;
        Serial.printf("[RS485] addr=%02X cmd=%02X cells=%u V=%.2f Bal=%d Alarm=0x%08X\n",
                      buf[2], buf[3], bms.num_cells, bms.total_v,
                      bms.balancing ? 1 : 0, bms.alarm_flags);
    }

    return d.valid;
}

// ============================================================
// ANT BMS: Parse status frame (7E A1 11 ...)
// Ref: https://github.com/syssi/esphome-ant-bms (ant_bms_ble component)
// ============================================================
bool parseAntBmsFrame(const uint8_t* buf, uint16_t len) {
    if (len < 10) return false;
    if (buf[0] != 0x7E || buf[1] != 0xA1) return false;
    if (buf[2] != 0x11) return false;  // status response only
    uint16_t data_len  = buf[5];
    uint16_t total_len = (uint16_t)(data_len + 10);
    if (len < total_len) return false;

    // CRC-16/Modbus over buf[1..total_len-5]
    uint16_t crc_calc = crc16Ant(buf + 1, total_len - 5);
    uint16_t crc_recv = (uint16_t)buf[total_len-4] | ((uint16_t)buf[total_len-3] << 8);
    if (crc_calc != crc_recv) {
        Serial.printf("[ANT] CRC fail calc=%04X recv=%04X\n", crc_calc, crc_recv);
        return false;
    }

    auto g16  = [&](uint16_t i) -> uint16_t {
        return (uint16_t)buf[i] | ((uint16_t)buf[i+1] << 8);
    };
    auto g32  = [&](uint16_t i) -> uint32_t {
        return (uint32_t)buf[i]            | ((uint32_t)buf[i+1] << 8) |
               ((uint32_t)buf[i+2] << 16) | ((uint32_t)buf[i+3] << 24);
    };
    auto gi16 = [&](uint16_t i) -> int16_t { return (int16_t)g16(i); };

    uint8_t temp_sensors = buf[8]; if (temp_sensors > 6) temp_sensors = 6;
    uint8_t cells        = buf[9];
    if (cells == 0 || cells > 24) return false;

    uint16_t off2 = (uint16_t)(cells * 2 + temp_sensors * 2);
    if ((uint32_t)(34 + off2 + 58) > total_len) return false;

    BMSData d = {};
    strncpy(d.bms_name, "ANT BMS", sizeof(d.bms_name) - 1);
    d.num_cells = cells;

    // Voltase sel: buf[34 + i*2], LE uint16, unit mV
    for (uint8_t i = 0; i < cells; i++)
        d.cell_mv[i] = g16(34 + i * 2);

    // Sensor suhu
    uint16_t tmp_base = (uint16_t)(34 + cells * 2);
    if (temp_sensors >= 1) {
        d.temp_bat1       = (float)gi16(tmp_base);
        d.temp_bat1_valid = (d.temp_bat1 > -50.0f && d.temp_bat1 < 120.0f);
    }
    if (temp_sensors >= 2) {
        d.temp_bat2       = (float)gi16(tmp_base + 2);
        d.temp_bat2_valid = (d.temp_bat2 > -50.0f && d.temp_bat2 < 120.0f);
    }
    d.temp_mos  = (float)gi16(34 + off2);
    d.total_v   = (float)g16(38 + off2) * 0.01f;
    d.current_a = (float)gi16(40 + off2) * 0.1f;
    d.soc       = (uint8_t)(g16(42 + off2) & 0xFF);
    if (d.soc > 100) d.soc = 100;

    d.charging    = (buf[46 + off2] == 0x01);
    d.discharging = (buf[47 + off2] == 0x01);
    d.balancing   = (buf[48 + off2] == 0x04);

    d.cap_ah    = (float)g32(50 + off2) * 0.000001f;
    d.remain_ah = (float)g32(54 + off2) * 0.000001f;

    if (d.current_a > -0.05f && d.current_a < 0.05f) {
        d.current_a = 0.0f; d.charging = false; d.discharging = false;
    }

    d.valid       = (d.num_cells > 0 && d.total_v > 0.5f);
    d.last_update = millis();
    if (d.valid) {
        bms = d;
        Serial.printf("[ANT] Sel=%d SOC=%d%% V=%.2fV I=%.2fA Tmos=%.1f\n",
                      bms.num_cells, bms.soc, bms.total_v, bms.current_a, bms.temp_mos);
    }
    return d.valid;
}

// ============================================================
// Proses buffer penerimaan BLE
// ============================================================
// Cek apakah buffer berisi teks ASCII murni (protokol AT)
static bool isAllPrintable(const uint8_t* buf, uint16_t len) {
    for (int i = 0; i < len; i++)
        if (buf[i] < 0x20 && buf[i] != 0x09 && buf[i] != 0x0D && buf[i] != 0x0A) return false;
    return len > 0;
}

void processRxBuffer() {
    if (rxLen == 0) return;

    auto markLegacyAckSeen = [&]() {
        legacyAckSeen = true;
        // Force next poll soon, useful for ACK-only notify bridges (e.g. FF12).
        if (millis() > REQUEST_INTERVAL_MS)
            lastRequestMs = millis() - REQUEST_INTERVAL_MS;
        else
            lastRequestMs = 0;
    };

    // ── ANT BMS: 7E A1 frame assembler ──────────────────────────────────────
    if (connectedBmsType == BMS_ANT) {
        int startIdx = -1;
        for (int i = 0; i < (int)rxLen - 1; i++) {
            if (rxBuf[i] == 0x7E && rxBuf[i+1] == 0xA1) { startIdx = i; break; }
        }
        if (startIdx < 0) { rxLen = 0; return; }
        if (startIdx > 0) { memmove(rxBuf, rxBuf+startIdx, rxLen-startIdx); rxLen -= (uint16_t)startIdx; }
        if (rxLen < 6) return;
        uint16_t data_len  = rxBuf[5];
        uint16_t total_len = (uint16_t)(data_len + 10);
        if (total_len > sizeof(rxBuf)) { rxLen = 0; return; }
        if (rxLen < total_len) return;
        bool ok = parseAntBmsFrame(rxBuf, total_len);
        if (rxLen > total_len) { memmove(rxBuf, rxBuf+total_len, rxLen-total_len); rxLen -= total_len; }
        else rxLen = 0;
        if (ok) { newDataReady = true; lastFrameMs = millis(); }
        if (rxLen >= 6) processRxBuffer();
        return;
    }

    auto logLegacyRs485Frame = [&](const uint8_t* frame, uint16_t len) {
        if (len < 5) return;
        uint8_t crc = 0;
        for (int i = 0; i < len - 1; i++) crc = (uint8_t)(crc + frame[i]);
        Serial.printf("[RX] RS485 legacy frame addr=%02X cmd=%02X len=%u crc=%s calc=%02X recv=%02X\n",
                      frame[2], frame[3], len,
                      crc == frame[len - 1] ? "OK" : "FAIL",
                      crc, frame[len - 1]);
        Serial.print("[RX] RS485 bytes:");
        for (int i = 0; i < len && i < 24; i++) Serial.printf(" %02X", frame[i]);
        if (len > 24) Serial.print(" ...");
        Serial.println();
    };

    // Deteksi protokol biner vs AT-text
    bool hasBinaryHeader = false;
    for (int i = 0; i < (int)rxLen - 1; i++) {
        if ((rxBuf[i] == 0x4E && rxBuf[i+1] == 0x57) ||
            (rxBuf[i] == 0xEB && rxBuf[i+1] == 0x90) ||
            (i < (int)rxLen-3 && rxBuf[i]==0x55 && rxBuf[i+1]==0xAA &&
             rxBuf[i+2]==0xEB && rxBuf[i+3]==0x90))
        { hasBinaryHeader = true; break; }
    }

    // Mode AT-text: buffer semua teks ASCII
    if (!hasBinaryHeader && isAllPrintable(rxBuf, rxLen)) {
        // Tunggu sampai buffer berakhir \n atau cukup besar
        bool hasNewline = false;
        for (int i = 0; i < rxLen; i++)
            if (rxBuf[i] == 0x0A) { hasNewline = true; break; }
        if (!hasNewline && rxLen < 200) return; // tunggu lebih banyak data

        // Print sebagai teks
        rxBuf[rxLen < sizeof(rxBuf)-1 ? rxLen : sizeof(rxBuf)-1] = 0;
        // Ganti \r\n dengan [LF] agar mudah dibaca
        Serial.print("[AT-RX] ");
        for (int i = 0; i < rxLen; i++) {
            if (rxBuf[i] == 0x0D) Serial.print("\\r");
            else if (rxBuf[i] == 0x0A) Serial.print("\\n ");
            else Serial.print((char)rxBuf[i]);
        }
        Serial.println();
        atRxCount++;   // hitung AT\r\n tanpa frame data
        rxLen = 0;
        return;
    }

    // Mode biner: buang AT\r\n prefix
    while (rxLen >= 4 &&
           rxBuf[0]==0x41 && rxBuf[1]==0x54 &&
           rxBuf[2]==0x0D && rxBuf[3]==0x0A) {
        memmove(rxBuf, rxBuf+4, rxLen-4);
        rxLen -= 4;
    }

    // ACK pendek legacy bisa datang sendiri atau bercampur dengan byte lain.
    // Tangkap semua pola FC xx 06 di mana pun posisinya dalam buffer.
    bool ackFound = false;
    for (int i = 0; i <= (int)rxLen - 3; ) {
        if (rxBuf[i] == 0xFC && rxBuf[i + 2] == 0x06) {
            Serial.printf("[RX] ACK pendek legacy: FC %02X 06\n", rxBuf[i + 1]);
            markLegacyAckSeen();
            ackFound = true;
            memmove(rxBuf + i, rxBuf + i + 3, rxLen - (uint16_t)(i + 3));
            rxLen -= 3;
            continue;
        }
        i++;
    }
    if (ackFound && rxLen == 0) return;

    // Cari header 4E57 atau 55AAEB90
    int startIdx = -1;
    bool oldProto = false;
    bool rs485Proto = false;
    for (int i = 0; i < (int)rxLen - 1; i++) {
        if (rxBuf[i] == 0x4E && rxBuf[i+1] == 0x57) { startIdx = i; break; }
        if (rxBuf[i] == 0xEB && rxBuf[i+1] == 0x90) { startIdx = i; rs485Proto = true; break; }
        if (i < (int)rxLen - 3 &&
            rxBuf[i]==0x55 && rxBuf[i+1]==0xAA &&
            rxBuf[i+2]==0xEB && rxBuf[i+3]==0x90) { startIdx = i; oldProto = true; break; }
    }
    if (startIdx < 0) {
        if (rxLen > 0) {
            Serial.printf("[RX] Tidak ada header di %d bytes:", rxLen);
            for (int i = 0; i < (int)rxLen && i < 16; i++) Serial.printf(" %02X", rxBuf[i]);
            Serial.println();
        }
        rxLen = 0; return;
    }
    if (oldProto) Serial.println("[RX] Protokol lama 55AAEB90 terdeteksi");
    if (rs485Proto) Serial.println("[RX] Protokol legacy RS485 EB90 terdeteksi");
    if (startIdx > 0) { memmove(rxBuf, rxBuf+startIdx, rxLen-startIdx); rxLen -= startIdx; }
    if (rxLen < 4) return;

    if (rs485Proto) {
        const uint16_t totalLen = 74;
        if (rxLen < totalLen) return;
        logLegacyRs485Frame(rxBuf, totalLen);
        bool ok = parseLegacyRs485Frame(rxBuf, totalLen);
        if (rxLen > totalLen) { memmove(rxBuf, rxBuf + totalLen, rxLen - totalLen); rxLen -= totalLen; }
        else rxLen = 0;
        if (ok) {
            newDataReady = true;
            lastFrameMs = millis();
        }
        if (rxLen >= 8) processRxBuffer();
        return;
    }

    // Hitung panjang frame berdasarkan protokol
    uint16_t frameBodyLen, totalLen;
    if (oldProto) {
        // Protokol lama: frame size TETAP 300 byte
        frameBodyLen = 294;
        totalLen     = 300;
    } else {
        // Protokol baru: panjang tersimpan di byte[2:3] big-endian
        frameBodyLen = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
        totalLen = frameBodyLen + 4;
        if (totalLen > (uint16_t)sizeof(rxBuf) || frameBodyLen < 8) totalLen = frameBodyLen;
    }

    Serial.printf("[RX] proto=%s type=%02X total=%d rxLen=%d\n",
                  oldProto ? "55AAEB90" : "4E57",
                  rxLen >= 5 ? rxBuf[4] : 0xFF,
                  totalLen, rxLen);

    if (rxLen < totalLen) return;

    Serial.printf("[RX] Frame lengkap %d bytes, proses...\n", totalLen);
    bool parseOk;
    if (oldProto) {
        parseOk = parseOldProtoFrame(rxBuf, totalLen);
    } else {
        parseOk = parseJKFrame(rxBuf, totalLen);
    }
    if (rxLen > totalLen) { memmove(rxBuf, rxBuf+totalLen, rxLen-totalLen); rxLen -= totalLen; }
    else rxLen = 0;
    if (parseOk) {
        newDataReady = true;
        lastFrameMs  = millis();
        atRxCount    = 0;   // reset: BMS aktif kirim data
        atEscapeSent = false;
    }
    // Proses frame berikutnya jika ada (settings + cell info = 2 frame per command)
    if (rxLen >= 8) processRxBuffer();
}

// ============================================================
// BLE Callbacks
// ============================================================
class JKClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* c) override {
        bleState = BLE_CONNECTED;
        Serial.println("[BLE] Terhubung ke BMS");
        needFullRedraw = true;
    }
    void onDisconnect(NimBLEClient* c) override {
        pJKChar        = nullptr;
        pJKCharAlt     = nullptr;
        pJKCharLegacy  = nullptr;
        pJKCharLegacyAlt = nullptr;
        pJKCharLegacyExtra = nullptr;
        connectedBmsType = BMS_JK;
        bmsScreenReady = false;
        Serial.println("[BLE] Terputus");
        // Jika daftar device masih ada, kembali ke layar pilih
        if (scanDeviceCount > 0) {
            bleState = BLE_SELECT;
        } else {
            bleState    = BLE_SCANNING;
            scanStarted = false;
        }
        needFullRedraw = true;
    }
};

static void notifyCallback(NimBLERemoteCharacteristic* c,
                           uint8_t* pData, size_t length, bool /*isNotify*/) {
    lastNotifyMs = millis();
    noDataRetryCount = 0;
    std::string src = c ? c->getUUID().toString() : std::string("unknown");
    Serial.printf("[NOTIFY %s] %d bytes:", src.c_str(), (int)length);
    for (int i = 0; i < (int)length && i < 32; i++) Serial.printf(" %02X", pData[i]);
    // Cetak juga sebagai ASCII jika semua byte printable (protokol AT)
    bool allPrintable = true;
    for (int i = 0; i < (int)length; i++)
        if (pData[i] < 0x20 && pData[i] != 0x0D && pData[i] != 0x0A) { allPrintable = false; break; }
    if (allPrintable && length > 0) {
        Serial.print(" [TXT: \"");
        for (int i = 0; i < (int)length; i++) {
            if (pData[i] == 0x0D) Serial.print("\\r");
            else if (pData[i] == 0x0A) Serial.print("\\n");
            else Serial.print((char)pData[i]);
        }
        Serial.print("\"]");
    }
    Serial.println();

    // Fast-path: ACK pendek legacy FC xx 06 dari bridge FF10/FF12.
    // Proses langsung di callback agar tidak salah baca saat buffer bercampur data lain.
    bool sawLegacyAck = false;
    for (size_t i = 0; i + 2 < length; i++) {
        if (pData[i] == 0xFC && pData[i + 2] == 0x06) {
            Serial.printf("[RX] ACK pendek legacy (notify): FC %02X 06\n", pData[i + 1]);
            legacyAckSeen = true;
            sawLegacyAck = true;
        }
    }
    if (sawLegacyAck) {
        if (millis() > REQUEST_INTERVAL_MS)
            lastRequestMs = millis() - REQUEST_INTERVAL_MS;
        else
            lastRequestMs = 0;
    }

    // ANT BMS: flush buffer on every 7E A1 preamble to prevent frame corruption
    if (connectedBmsType == BMS_ANT && length >= 2 && pData[0] == 0x7E && pData[1] == 0xA1)
        rxLen = 0;
    if (rxLen + length >= sizeof(rxBuf)) {
        Serial.println("[NOTIFY] Buffer penuh, reset");
        rxLen = 0;
    }
    memcpy(rxBuf + rxLen, pData, length);
    rxLen += (uint16_t)length;
    processRxBuffer();
}

static void subscribeNotifyIfSupported(NimBLERemoteCharacteristic* ch,
                                       const char* label) {
    if (!ch || (!ch->canNotify() && !ch->canIndicate())) return;
    bool ok = ch->subscribe(true, notifyCallback);
    Serial.printf("[BLE] Subscribe %s: %s\n", label, ok ? "OK" : "FAIL");
}

static void buildJkReadCommand(uint8_t command,
                               uint8_t length,
                               uint32_t value,
                               uint8_t* frame,
                               const LegacyRequestFamily* family = nullptr) {
    static const LegacyRequestFamily defaultFamily = {0xAA, 0x55, 0x90, 0xEB, "AA55-90EB"};
    const LegacyRequestFamily* activeFamily = family ? family : &defaultFamily;

    memset(frame, 0, 20);
    frame[0] = activeFamily->header0;
    frame[1] = activeFamily->header1;
    frame[2] = activeFamily->header2;
    frame[3] = activeFamily->header3;
    frame[4] = command;
    frame[5] = length;
    frame[6] = (uint8_t)(value >> 0);
    frame[7] = (uint8_t)(value >> 8);
    frame[8] = (uint8_t)(value >> 16);
    frame[9] = (uint8_t)(value >> 24);
    frame[16] = legacyJkSequence++;

    uint8_t crc = 0;
    for (int i = 0; i < 19; i++) crc = (uint8_t)(crc + frame[i]);
    frame[19] = crc;
}

static bool sendReadCommandEx(NimBLERemoteCharacteristic* ch,
                              uint8_t command,
                              uint8_t length,
                              uint32_t value,
                              const char* label,
                              const char* variantLabel,
                              const LegacyRequestFamily* family = nullptr) {
    if (!ch || (!ch->canWrite() && !ch->canWriteNoResponse())) return false;

    uint8_t frame[20];
    buildJkReadCommand(command, length, value, frame, family);
    const char* familyName = family ? family->name : LEGACY_REQUEST_FAMILIES[0].name;
    Serial.printf("[BLE] %s: CMD 0x%02X %s %s seq=%u\n",
                  label,
                  command,
                  variantLabel ? variantLabel : "",
                  familyName,
                  frame[16]);
    ch->writeValue(frame, sizeof(frame), false);
    return true;
}

static bool sendReadCommand(NimBLERemoteCharacteristic* ch,
                            uint8_t command,
                            const char* label) {
    return sendReadCommandEx(ch, command, 0x00, 0x00000000UL, label, "L0V0");
}

static bool sendLegacyShortProbe(NimBLERemoteCharacteristic* ch,
                                 const LegacyShortProbe& probe,
                                 const char* label) {
    if (!ch || (!ch->canWrite() && !ch->canWriteNoResponse())) return false;

    uint8_t frame[7];
    frame[0] = 0x55;
    frame[1] = 0xAA;
    frame[2] = probe.address;
    frame[3] = probe.command;
    frame[4] = (uint8_t)(probe.value >> 8);
    frame[5] = (uint8_t)(probe.value & 0xFF);

    uint8_t crc = 0;
    for (int i = 0; i < 6; i++) crc = (uint8_t)(crc + frame[i]);
    frame[6] = crc;

    Serial.printf("[BLE] %s: SHORT %s 55 AA %02X %02X %02X %02X %02X\n",
                  label,
                  probe.name,
                  frame[2],
                  frame[3],
                  frame[4],
                  frame[5],
                  frame[6]);
    ch->writeValue(frame, sizeof(frame), false);
    return true;
}

static bool sendInitCommands(NimBLERemoteCharacteristic* ch, const char* label) {
    if (!ch || (!ch->canWrite() && !ch->canWriteNoResponse())) return false;
    sendReadCommand(ch, 0x97, label);
    delay(300);
    sendReadCommand(ch, 0x96, label);
    return true;
}

static void sendLegacyCellInfoPoll() {
    NimBLERemoteCharacteristic* chars[3] = {
        pJKCharLegacy,
        pJKCharLegacyAlt,
        pJKCharLegacyExtra,
    };

    for (int i = 0; i < 3; i++) {
        NimBLERemoteCharacteristic* ch = chars[i];
        if (!ch || (!ch->canWrite() && !ch->canWriteNoResponse())) continue;

        bool duplicate = false;
        for (int j = 0; j < i; j++) {
            if (chars[j] == ch) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        std::string uuid = ch->getUUID().toString();
        sendReadCommand(ch, 0x96, uuid.c_str());
        delay(120);
    }
}

static void sendLegacyAltProbe() {
    bool useShortProbe = legacyUseShortNext;
    legacyUseShortNext = !legacyUseShortNext;

    uint8_t command = LEGACY_PROBE_COMMANDS[legacyProbeIndex % (sizeof(LEGACY_PROBE_COMMANDS) / sizeof(LEGACY_PROBE_COMMANDS[0]))];
    const LegacyProbeFormat& format = LEGACY_PROBE_FORMATS[legacyFormatProbeIndex % (sizeof(LEGACY_PROBE_FORMATS) / sizeof(LEGACY_PROBE_FORMATS[0]))];
    const LegacyRequestFamily& family = LEGACY_REQUEST_FAMILIES[legacyFamilyProbeIndex % (sizeof(LEGACY_REQUEST_FAMILIES) / sizeof(LEGACY_REQUEST_FAMILIES[0]))];
    const LegacyShortProbe& shortProbe = LEGACY_SHORT_PROBES[legacyShortProbeIndex % (sizeof(LEGACY_SHORT_PROBES) / sizeof(LEGACY_SHORT_PROBES[0]))];

    if (useShortProbe) {
        legacyShortProbeIndex++;
    } else {
        legacyProbeIndex++;
        legacyFormatProbeIndex++;
        legacyFamilyProbeIndex++;
    }

    NimBLERemoteCharacteristic* chars[3] = {
        pJKCharLegacy,
        pJKCharLegacyAlt,
        pJKCharLegacyExtra,
    };

    for (int i = 0; i < 3; i++) {
        NimBLERemoteCharacteristic* ch = chars[i];
        if (!ch || (!ch->canWrite() && !ch->canWriteNoResponse())) continue;

        bool duplicate = false;
        for (int j = 0; j < i; j++) {
            if (chars[j] == ch) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        std::string uuid = ch->getUUID().toString();
        char label[48];
        snprintf(label, sizeof(label), "Legacy probe via %s", uuid.c_str());
        if (useShortProbe) {
            sendLegacyShortProbe(ch, shortProbe, label);
        } else {
            sendReadCommandEx(ch, command, format.length, format.value, label, format.name, &family);
        }
        delay(150);
    }
}

static void sendLegacyInitProbe() {
    NimBLERemoteCharacteristic* chars[3] = {
        pJKCharLegacy,
        pJKCharLegacyAlt,
        pJKCharLegacyExtra,
    };

    subscribeNotifyIfSupported(pJKCharLegacyAlt, "FF10 primary");
    if (pJKCharLegacyExtra && pJKCharLegacyExtra != pJKCharLegacyAlt) {
        subscribeNotifyIfSupported(pJKCharLegacyExtra, "FF10 secondary");
    }

    for (int i = 0; i < 3; i++) {
        NimBLERemoteCharacteristic* ch = chars[i];
        if (!ch) continue;

        bool duplicate = false;
        for (int j = 0; j < i; j++) {
            if (chars[j] == ch) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        std::string uuid = ch->getUUID().toString();
        if (sendInitCommands(ch, uuid.c_str())) {
            delay(150);
        }
    }
}

class JKScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        std::string name = dev->getName();
        bool isANT = (name.find("ANT") != std::string::npos || name.find("ant") != std::string::npos);
        bool isJK  = (name.find("JK")  != std::string::npos);
        // Fallback: cek service UUID FFE0
        if (!isJK && !isANT && dev->haveServiceUUID()) {
            isJK = dev->isAdvertisingService(NimBLEUUID(JK_SERVICE_UUID));
        }
        if (!isJK && !isANT) return;

        NimBLEAddress devAddr = dev->getAddress();
        std::string   addrStr = devAddr.toString();
        // Cek duplikat
        for (int i = 0; i < scanDeviceCount; i++) {
            if (strcmp(scanDevices[i].addr, addrStr.c_str()) == 0) return;
        }
        if (scanDeviceCount >= MAX_SCAN_DEVICES) {
            NimBLEDevice::getScan()->stop();
            return;
        }
        // Simpan perangkat baru
        BMSType     dtype   = isANT ? BMS_ANT : BMS_JK;
        const char* defName = isANT ? "ANT BMS" : "JK BMS";
        scanDevices[scanDeviceCount].bleAddr  = devAddr;
        scanDevices[scanDeviceCount].bmsType  = dtype;
        strncpy(scanDevices[scanDeviceCount].name,
                name.empty() ? defName : name.c_str(), 19);
        scanDevices[scanDeviceCount].name[19] = '\0';
        strncpy(scanDevices[scanDeviceCount].addr, addrStr.c_str(), 17);
        scanDevices[scanDeviceCount].addr[17] = '\0';
        scanDeviceCount++;

        Serial.printf("[BLE] Ditemukan [%d]: %s [%s]\n",
                      scanDeviceCount, name.c_str(), addrStr.c_str());
        // Tampilkan layar pilih, scan tetap jalan untuk temukan lebih banyak
        if (bleState == BLE_SCANNING) {
            bleState = BLE_SELECT;
        }
        // Refresh daftar setiap kali ada perangkat baru
        needFullRedraw = true;

        // Stop scan hanya jika sudah penuh
        if (scanDeviceCount >= MAX_SCAN_DEVICES) {
            NimBLEDevice::getScan()->stop();
        }
    }
};

// ============================================================
// Koneksi ke BMS
// ============================================================
bool connectToBMS() {
    if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new JKClientCallbacks(), false);
        pClient->setConnectionParams(12, 12, 0, 51);
        pClient->setConnectTimeout(5);
    }

    Serial.printf("[BLE] Menghubungkan ke %s...\n",
                  selectedBLEAddr.toString().c_str());

    if (!pClient->connect(selectedBLEAddr)) {
        Serial.println("[BLE] Gagal terhubung");
        return false;
    }

    // Enumerate semua service (untuk diagnostic)
    std::vector<NimBLERemoteService*>* allSvcs = pClient->getServices(true);
    Serial.printf("[BLE] %d services: ", (int)allSvcs->size());
    for (auto& s : *allSvcs) Serial.printf("%s ", s->getUUID().toString().c_str());
    Serial.println();

    NimBLERemoteService* svc = pClient->getService(JK_SERVICE_UUID);
    NimBLERemoteService* legacySvc = pClient->getService(JK_LEGACY_SERVICE_UUID);
    if (!svc) {
        Serial.println("[BLE] Service FFE0 tidak ditemukan");
        pClient->disconnect();
        return false;
    }

    // FFE1 = NOTIFY, FFE2 = WRITE (terpisah)
    NimBLERemoteCharacteristic* notifyChar = svc->getCharacteristic(JK_NOTIFY_UUID);
    NimBLERemoteCharacteristic* writeChar  = svc->getCharacteristic(JK_WRITE_UUID);

    // Fallback: jika FFE2 tidak ada, gunakan FFE1 untuk keduanya
    if (!writeChar) {
        Serial.println("[BLE] FFE2 tidak ada, pakai FFE1 untuk write");
        writeChar = notifyChar;
    }
    if (!notifyChar) {
        Serial.println("[BLE] FFE1 tidak ada");
        pClient->disconnect();
        return false;
    }

    // Log semua service UUID (aman, tidak fetch char)
    Serial.println("[BLE] Semua services:");
    for (auto& s : *allSvcs)
        Serial.printf("  SVC %s\n", s->getUUID().toString().c_str());

    // Log chars di FFE0 menggunakan cache (sudah di-fetch via getCharacteristic di atas)
    Serial.println("[BLE] Chars di FFE0 (cache):");
    auto* ffeChars = svc->getCharacteristics(false);
    if (ffeChars) {
        for (auto& c : *ffeChars) {
            Serial.printf("  [%s] N=%d I=%d W=%d WNR=%d\n",
                          c->getUUID().toString().c_str(),
                          (int)c->canNotify(), (int)c->canIndicate(),
                          (int)c->canWrite(), (int)c->canWriteNoResponse());
        }
    }

    NimBLERemoteCharacteristic* legacyNotifyChar = nullptr;
    NimBLERemoteCharacteristic* legacyWriteChar  = nullptr;
    NimBLERemoteCharacteristic* legacyExtraChar  = nullptr;
    NimBLERemoteCharacteristic* legacyFirstChar  = nullptr;
    NimBLERemoteCharacteristic* legacySecondChar = nullptr;
    if (legacySvc) {
        Serial.println("[BLE] Chars di FF10:");
        auto* legacyChars = legacySvc->getCharacteristics(true);
        if (legacyChars) {
            for (auto& c : *legacyChars) {
                Serial.printf("  [%s] N=%d I=%d W=%d WNR=%d\n",
                              c->getUUID().toString().c_str(),
                              (int)c->canNotify(), (int)c->canIndicate(),
                              (int)c->canWrite(), (int)c->canWriteNoResponse());
                if (!legacyFirstChar) legacyFirstChar = c;
                else if (!legacySecondChar && c != legacyFirstChar) legacySecondChar = c;
                if (c->canNotify() || c->canIndicate()) {
                    if (!legacyNotifyChar) legacyNotifyChar = c;
                    else if (!legacyExtraChar && c != legacyNotifyChar) legacyExtraChar = c;
                }
                if (!legacyWriteChar && (c->canWrite() || c->canWriteNoResponse()))
                    legacyWriteChar = c;
            }
        }
        if (!legacyExtraChar) {
            if (legacyFirstChar && legacyFirstChar != legacyNotifyChar) legacyExtraChar = legacyFirstChar;
            else if (legacySecondChar && legacySecondChar != legacyNotifyChar) legacyExtraChar = legacySecondChar;
        }
        if (legacyNotifyChar == legacyWriteChar) {
            if (legacyExtraChar && legacyExtraChar != legacyWriteChar) {
                legacyNotifyChar = legacyExtraChar;
            } else if (legacySecondChar && legacySecondChar != legacyWriteChar) {
                legacyNotifyChar = legacySecondChar;
            }
        }
        if (legacyExtraChar == legacyNotifyChar) {
            if (legacyFirstChar && legacyFirstChar != legacyNotifyChar) legacyExtraChar = legacyFirstChar;
            else if (legacySecondChar && legacySecondChar != legacyNotifyChar) legacyExtraChar = legacySecondChar;
        }
    }

    // PENTING: assign pointer SETELAH semua log selesai, TANPA re-fetch
    pJKChar    = writeChar;
    pJKCharAlt = notifyChar;
    pJKCharLegacy = legacyWriteChar;
    pJKCharLegacyAlt = legacyNotifyChar;
    pJKCharLegacyExtra = legacyExtraChar;
    Serial.printf("[BLE] Write(FFE2): %s | Alt(FFE1): %s\n",
                  writeChar  ? writeChar->getUUID().toString().c_str()  : "none",
                  notifyChar ? notifyChar->getUUID().toString().c_str() : "none");
    if (pJKCharLegacy || pJKCharLegacyAlt) {
        Serial.printf("[BLE] Legacy FF10 write=%s notify=%s extra=%s\n",
                      pJKCharLegacy ? pJKCharLegacy->getUUID().toString().c_str() : "none",
                      pJKCharLegacyAlt ? pJKCharLegacyAlt->getUUID().toString().c_str() : "none",
                      pJKCharLegacyExtra ? pJKCharLegacyExtra->getUUID().toString().c_str() : "none");
    }

    subscribeNotifyIfSupported(notifyChar, "FFE1");
    subscribeNotifyIfSupported(pJKCharLegacyAlt, "FF10 primary");
    if (pJKCharLegacyExtra && pJKCharLegacyExtra != pJKCharLegacyAlt) {
        subscribeNotifyIfSupported(pJKCharLegacyExtra, "FF10 secondary");
    }

    // Set tipe BMS sebelum subscribe agar callback routing sudah benar
    connectedBmsType = selectedDeviceType;

    if (connectedBmsType == BMS_ANT) {
        // ANT BMS: write & notify via FFE1; tidak perlu legacy service
        pJKChar    = notifyChar;
        pJKCharAlt = notifyChar;
        pJKCharLegacy = nullptr; pJKCharLegacyAlt = nullptr; pJKCharLegacyExtra = nullptr;
        subscribeNotifyIfSupported(notifyChar, "ANT FFE1");
        delay(500);
        rxLen = 0; atRxCount = 0; atEscapeSent = false;
        lastNotifyMs = 0; lastFrameMs = 0; noDataRetryCount = 0;
        legacyAckSeen = false;
        notifyChar->writeValue((uint8_t*)CMD_ANT_STATUS, sizeof(CMD_ANT_STATUS), false);
        Serial.println("[ANT] Terhubung, permintaan status dikirim");
    } else {
        delay(1000);

        // Reset AT mode recovery state
        atRxCount    = 0;
        atEscapeSent = false;
        lastNotifyMs = 0;
        lastFrameMs  = 0;
        noDataRetryCount = 0;
        legacyAckSeen = false;
        legacyProbeIndex = 0;
        legacyFormatProbeIndex = 0;
        legacyFamilyProbeIndex = 0;
        legacyShortProbeIndex = 0;
        legacyUseShortNext = true;
        legacyJkSequence = 1;

        // Kirim CMD_DEV_INFO untuk mendapat info perangkat (type 0x03)
        Serial.println("[BLE] Init: CMD_DEV_INFO -> FFE2 (chk=0x11)");
        pJKChar->writeValue((uint8_t*)CMD_DEV_INFO, sizeof(CMD_DEV_INFO), false);
        delay(800);

        // Kirim CMD_CELL_INFO untuk mendapat settings (type 0x01) + cell info (type 0x02)
        Serial.println("[BLE] Init: CMD_CELL_INFO -> FFE2 (chk=0x10)");
        pJKChar->writeValue((uint8_t*)CMD_CELL_INFO, sizeof(CMD_CELL_INFO), false);
        delay(800);

        if (pJKCharLegacy || pJKCharLegacyAlt || pJKCharLegacyExtra) {
            Serial.println("[BLE] Init tambahan via FF10 probe...");
            sendLegacyInitProbe();
        }
    }

    Serial.println("[BLE] Init selesai, loop poll setiap 5s");
    return true;
}

// ============================================================
// Warna Helpers
// ============================================================
uint16_t colorSOC(uint8_t soc) {
    if (soc >= 60) return TFT_GREEN;
    if (soc >= 30) return TFT_YELLOW;
    return TFT_RED;
}

uint16_t colorTemp(float t) {
    if (t > 50) return TFT_RED;
    if (t > 40) return TFT_ORANGE;
    return TFT_CYAN;
}

uint16_t colorCurrent(float a) {
    if (a > 0.05f) return TFT_GREEN;
    if (a < -0.05f) return TFT_ORANGE;
    return TFT_LIGHTGREY;
}

// Warna sel berdasarkan voltase relatif
uint16_t colorCell(uint16_t mv, uint16_t mn, uint16_t mx) {
    uint16_t delta = mx - mn;
    if (delta < 10) return TFT_GREEN;
    if (mv == mn) return TFT_RED;
    if (mv == mx) return TFT_ORANGE;
    if (delta > 100) return TFT_YELLOW;
    return TFT_GREEN;
}

// Warna resistansi internal sel (mΩ)
uint16_t colorRes(uint16_t mohm) {
    if (mohm == 0)    return tft.color565(40, 60, 80); // tidak diketahui
    if (mohm < 300)   return TFT_GREEN;
    if (mohm < 600)   return TFT_YELLOW;
    if (mohm < 1000)  return TFT_ORANGE;
    return TFT_RED;
}

// ============================================================
// Gambar Header (bersama untuk semua halaman)
// ============================================================
void drawHeader(bool connected, const char* title) {
    const uint16_t HDR    = tft.color565(0, 12, 35);
    const uint16_t ACCENT = TFT_CYAN;
    const uint16_t DIM    = tft.color565(0, 60, 100);

    tft.fillRect(0,  0, 240, 36, HDR);
    tft.fillRect(0,  0, 240,  2, ACCENT);   // garis aksen atas
    tft.fillRect(0, 35, 240,  1, DIM);      // garis pemisah bawah

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(8, 8);
    tft.print(title);

    // Indikator BLE
    if (connected) {
        tft.fillCircle(224, 18, 8, TFT_GREEN);
        tft.drawCircle(224, 18, 9, tft.color565(100, 255, 100));
        tft.setTextColor(TFT_BLACK);
        tft.setCursor(219, 10);
        tft.print("B");
    } else {
        tft.fillCircle(224, 18, 8, tft.color565(5, 10, 25));
        tft.drawCircle(224, 18, 8, DIM);
        tft.setTextColor(DIM);
        tft.setCursor(219, 10);
        tft.print("B");
    }
}

// ============================================================
// Gambar Halaman Scanning / Connecting
// ============================================================
void drawScanScreen(const char* msg) {
    const uint16_t BG     = tft.color565(0, 5, 20);
    const uint16_t ACCENT = TFT_CYAN;
    const uint16_t DIM    = tft.color565(0, 60, 100);
    const uint16_t CARD   = tft.color565(0, 14, 33);

    bool isConnecting = (strstr(msg, "ubung") != nullptr);
    uint16_t iconClr  = isConnecting ? TFT_YELLOW : ACCENT;

    tft.fillScreen(BG);
    drawHeader(false, "JK BMS Monitor");

    // Baris aksen bawah
    tft.fillRect(0, 315, 240, 4, ACCENT);
    tft.fillRect(0, 314, 240, 1, DIM);

    // ── Ikon BLE (center 120, 105) ────────────────────────────
    const int cx = 120, cy = 105;
    tft.drawCircle(cx, cy, 44, tft.color565(0, 18, 35));   // ring luar redup
    tft.drawCircle(cx, cy, 33, tft.color565(0, 38, 65));   // ring tengah
    tft.drawCircle(cx, cy, 22, isConnecting
                            ? tft.color565(50, 50, 0) : DIM);
    tft.fillCircle(cx, cy, 17, tft.color565(0, 18, 42));   // isi dalam
    tft.drawCircle(cx, cy, 17, iconClr);                   // border dalam
    tft.setTextColor(iconClr);
    tft.setTextSize(3);
    tft.setCursor(cx - 9, cy - 12);  // huruf B (18×24) terpusat
    tft.print("B");

    // ── Judul besar ──────────────────────────────────────────
    tft.setTextColor(iconClr);
    tft.setTextSize(2);
    if (isConnecting) {
        // "Menghubungkan" 13 × 12px = 156px → x = 42
        tft.setCursor(42, 158);
        tft.print("Menghubungkan");
    } else {
        // "Cari BMS JK/ANT" 15 × 12px = 180px → x = 30
        tft.setCursor(30, 158);
        tft.print("Cari BMS JK/ANT");
    }

    // ── Pesan status ─────────────────────────────────────────
    tft.setTextColor(tft.color565(70, 140, 170));
    tft.setTextSize(1);
    int msgW = (int)strlen(msg) * 6;
    if (msgW > 220) msgW = 220;
    tft.setCursor(120 - msgW / 2, 177);
    tft.print(msg);

    // ── Info card (hanya saat scanning) ──────────────────────
    if (!isConnecting) {
        tft.fillRoundRect(10, 193, 220, 80, 6, CARD);
        tft.drawRoundRect(10, 193, 220, 80, 6, DIM);
        tft.setTextColor(tft.color565(90, 160, 180));
        tft.setTextSize(1);
        tft.setCursor(22, 202); tft.print("Pastikan BMS menyala");
        tft.setCursor(22, 216); tft.print("dan Bluetooth BMS aktif.");
        tft.setTextColor(tft.color565(55, 105, 130));
        tft.setCursor(22, 231); tft.print("Nama: \"JK...\" atau \"ANT...\"");
        tft.setTextColor(tft.color565(75, 145, 170));
        tft.setCursor(22, 246); tft.print("Contoh: JK_B2A24S, ANT BMS");
    }
}

// ============================================================
// Gambar Layar Pilih Perangkat BLE
// ============================================================
void drawSelectScreen() {
    const uint16_t BG     = tft.color565(0, 5, 20);
    const uint16_t ACCENT = TFT_CYAN;
    const uint16_t DIM    = tft.color565(0, 60, 100);
    const uint16_t CARD   = tft.color565(0, 14, 33);
    const uint16_t CBDR   = tft.color565(0, 48, 75);

    tft.fillScreen(BG);
    drawHeader(false, "Pilih BLE BMS");

    // Baris aksen bawah
    tft.fillRect(0, 315, 240, 4, ACCENT);
    tft.fillRect(0, 314, 240, 1, DIM);

    // ── Strip status (y=36..52) ───────────────────────────────
    tft.fillRect(0, 36, 240, 17, tft.color565(0, 10, 28));
    tft.drawFastHLine(0, 52, 240, DIM);
    tft.setTextSize(1);
    if (NimBLEDevice::getScan()->isScanning()) {
        tft.fillCircle(10, 45, 3, TFT_YELLOW);
        tft.setTextColor(TFT_YELLOW);
        tft.setCursor(18, 41);
        tft.print("Mencari lebih banyak...");
    } else {
        tft.fillCircle(10, 45, 3, TFT_GREEN);
        char sbuf[36];
        snprintf(sbuf, sizeof(sbuf), "Ditemukan %d perangkat", scanDeviceCount);
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(18, 41);
        tft.print(sbuf);
    }

    if (scanDeviceCount == 0) {
        // ── Empty state ──────────────────────────────────────
        tft.drawCircle(120, 165, 30, DIM);
        tft.drawCircle(120, 165, 21, tft.color565(0, 30, 50));
        tft.fillCircle(120, 165, 15, tft.color565(0, 12, 28));
        tft.setTextColor(tft.color565(0, 50, 80));
        tft.setTextSize(2);
        tft.setCursor(111, 153);
        tft.print("?");
        tft.setTextColor(tft.color565(55, 100, 125));
        tft.setTextSize(1);
        tft.setCursor(22, 205); tft.print("Belum ada perangkat JK/ANT.");
        tft.setCursor(42, 221); tft.print("Tekan SCAN ULANG di bawah.");
    } else {
        // ── Daftar perangkat (y=54+i*46, touch-zone 42px) ────
        for (int i = 0; i < scanDeviceCount && i < MAX_SCAN_DEVICES; i++) {
            int y = 54 + i * 46;
            tft.fillRoundRect(5, y, 230, 42, 5, CARD);
            tft.drawRoundRect(5, y, 230, 42, 5, CBDR);
            // Ikon BLE kiri
            tft.fillCircle(18, y + 21, 5, tft.color565(0, 50, 100));
            tft.drawCircle(18, y + 21, 5, ACCENT);
            tft.drawCircle(18, y + 21, 9, tft.color565(0, 35, 70));
            // Nama perangkat (max 14 char agar muat di lebar kartu)
            tft.setTextColor(TFT_WHITE);
            tft.setTextSize(2);
            tft.setCursor(32, y + 6);
            char dn[15];
            strncpy(dn, scanDevices[i].name, 14);
            dn[14] = '\0';
            tft.print(dn);
            // Alamat MAC
            tft.setTextColor(tft.color565(45, 90, 120));
            tft.setTextSize(1);
            tft.setCursor(32, y + 29);
            tft.print(scanDevices[i].addr);
            // Chevron kanan
            tft.setTextColor(CBDR);
            tft.setTextSize(2);
            tft.setCursor(214, y + 13);
            tft.print(">");
        }
    }

    // ── Tombol SCAN ULANG (touch-zone ty >= 285) ─────────────
    tft.fillRoundRect(18, 283, 204, 30, 7, tft.color565(0, 18, 45));
    tft.drawRoundRect(18, 283, 204, 30, 7, ACCENT);
    tft.drawFastHLine(25, 284, 190, tft.color565(0, 75, 115));
    tft.setTextColor(ACCENT);
    tft.setTextSize(2);
    // "SCAN ULANG" 10 × 12px = 120px → x = 60
    tft.setCursor(60, 291);
    tft.print("SCAN ULANG");
}

// ============================================================
// Gambar Halaman Utama (full redraw)
// ============================================================
void drawMainFull() {
    const uint16_t BG     = tft.color565(0, 5, 20);
    const uint16_t ACCENT = TFT_CYAN;
    const uint16_t DIM    = tft.color565(0, 60, 100);
    const uint16_t LBL    = tft.color565(80, 140, 160);

    tft.fillScreen(BG);
    drawHeader(bleState == BLE_CONNECTED, bms.bms_name);

    // Baris aksen bawah
    tft.fillRect(0, 315, 240, 4, ACCENT);
    tft.fillRect(0, 314, 240, 1, DIM);

    if (!bms.valid) {
        tft.setTextColor(LBL);
        tft.setTextSize(1);
        tft.setCursor(30, 160);
        tft.print("Menunggu data BMS...");
        return;
    }

    char buf[20];

    // -- SOC --------------------------------------------------
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    tft.setCursor(8, 44);
    tft.print("Remain Battery");
    uint16_t deltaMv = calcCellDeltaMv(bms);
    snprintf(buf, sizeof(buf), "%.1f/%.1fAh  D:%dmV", bms.remain_ah, bms.cap_ah, deltaMv);
    tft.setCursor(240 - (int)strlen(buf) * 6 - 4, 44);
    tft.print(buf);

    // Angka SOC besar
    snprintf(buf, sizeof(buf), "%3d%%", bms.soc);
    tft.setTextColor(colorSOC(bms.soc));
    tft.setTextSize(4);
    tft.setCursor(60, 56);
    tft.print(buf);

    // Bar SOC rounded + highlight
    int barW = (int)(200 * bms.soc / 100);
    tft.fillRoundRect(19, 100, 202, 14, 3, tft.color565(0, 10, 25));
    tft.drawRoundRect(19, 100, 202, 14, 3, DIM);
    if (barW > 0) {
        tft.fillRoundRect(20, 101, barW, 12, 3, colorSOC(bms.soc));
        uint16_t hlt = colorSOC(bms.soc) == TFT_RED    ? tft.color565(255, 90,  90)  :
                       colorSOC(bms.soc) == TFT_YELLOW ? tft.color565(255, 255, 130) :
                                                          tft.color565(130, 255, 130);
        tft.drawFastHLine(20, 102, barW > 2 ? barW - 2 : barW, hlt);
    }

    tft.drawFastHLine(0, 118, 240, DIM);

    // -- Voltase & Arus ---------------------------------------
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    tft.setCursor(8,   124); tft.print("Voltase");
    tft.setCursor(126, 124); tft.print("Arus");

    snprintf(buf, sizeof(buf), "%.2fV", bms.total_v);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(8, 136);
    tft.print(buf);

    snprintf(buf, sizeof(buf), "%+.1fA", bms.current_a);
    tft.setTextColor(colorCurrent(bms.current_a));
    tft.setCursor(126, 136);
    tft.print(buf);

    // -- Daya -------------------------------------------------
    tft.drawFastHLine(0, 158, 240, DIM);
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    tft.setCursor(8, 163);
    tft.print("Daya");

    float pwr = bms.total_v * bms.current_a;
    snprintf(buf, sizeof(buf), "%+.0fW", pwr);
    tft.setTextColor(colorCurrent(pwr));
    tft.setTextSize(2);
    tft.setCursor(8, 174);
    tft.print(buf);

    drawStatusBadges(bms);

    tft.drawFastHLine(0, 196, 240, DIM);

    // -- Suhu -------------------------------------------------
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    tft.setCursor(8,   201); tft.print("MOS");
    tft.setCursor(86,  201); tft.print("BAT1");
    tft.setCursor(164, 201); tft.print("BAT2");

    snprintf(buf, sizeof(buf), "%.0f°C", bms.temp_mos);
    tft.setTextColor(colorTemp(bms.temp_mos));
    tft.setTextSize(1); tft.setCursor(8, 212); tft.print(buf);

    if (bms.temp_bat1_valid) {
        snprintf(buf, sizeof(buf), "%.0f°C", bms.temp_bat1);
        tft.setTextColor(colorTemp(bms.temp_bat1));
    } else {
        strcpy(buf, "---");
        tft.setTextColor(tft.color565(60, 80, 100));
    }
    tft.setCursor(86, 212); tft.print(buf);

    if (bms.temp_bat2_valid) {
        snprintf(buf, sizeof(buf), "%.0f°C", bms.temp_bat2);
        tft.setTextColor(colorTemp(bms.temp_bat2));
    } else {
        strcpy(buf, "---");
        tft.setTextColor(tft.color565(60, 80, 100));
    }
    tft.setCursor(164, 212); tft.print(buf);

    tft.drawFastHLine(0, 226, 240, DIM);

    // -- Kapasitas & Siklus -----------------------------------
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    tft.setCursor(8,   231); tft.print("Tersisa");
    tft.setCursor(100, 231); tft.print("Total");
    tft.setCursor(172, 231); tft.print("Siklus");

    snprintf(buf, sizeof(buf), "%.1fAh", bms.remain_ah);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1); tft.setCursor(8, 242); tft.print(buf);

    snprintf(buf, sizeof(buf), "%.1fAh", bms.cap_ah);
    tft.setCursor(100, 242); tft.print(buf);

    snprintf(buf, sizeof(buf), "%lu", bms.cycle_cnt);
    tft.setCursor(172, 242); tft.print(buf);

    tft.drawFastHLine(0, 256, 240, DIM);

    // -- Alarm ------------------------------------------------
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    tft.setCursor(8, 261); tft.print("Alarm:");

    if (bms.alarm_flags == 0) {
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(55, 261); tft.print("NORMAL");
    } else {
        tft.setTextColor(TFT_RED);
        snprintf(buf, sizeof(buf), "0x%08X", bms.alarm_flags);
        tft.setCursor(55, 261); tft.print(buf);
    }

    // -- Footer -----------------------------------------------
    tft.fillRect(0, 276, 240, 39, tft.color565(0, 12, 35));
    tft.drawFastHLine(0, 276, 240, DIM);
    tft.fillRect(0, 315, 240, 4, ACCENT);
    tft.setTextColor(LBL);
    tft.setTextSize(1);
    // "TAP: Lihat Voltase Sel" 22 x 6 = 132px -> x = 54
    tft.setCursor(54, 289);
    tft.print("TAP: Lihat Voltase Sel");
}

// ============================================================
// Update Inkremental Halaman Utama (hindari flicker)
// ============================================================
void updateMainScreen() {
    if (!bms.valid) return;
    const uint16_t BG  = tft.color565(0, 5, 20);
    const uint16_t DIM = tft.color565(0, 60, 100);
    char buf[32];

    // Subtitle remain/cap + cell diff
    tft.fillRect(100, 44, 140, 8, BG);
    uint16_t deltaMv = calcCellDeltaMv(bms);
    snprintf(buf, sizeof(buf), "%.1f/%.1fAh D:%dmV", bms.remain_ah, bms.cap_ah, deltaMv);
    tft.setTextColor(tft.color565(80, 140, 160));
    tft.setTextSize(1);
    tft.setCursor(240 - (int)strlen(buf) * 6 - 4, 44);
    tft.print(buf);

    // SOC number
    tft.fillRect(60, 56, 120, 32, BG);
    snprintf(buf, sizeof(buf), "%3d%%", bms.soc);
    tft.setTextColor(colorSOC(bms.soc));
    tft.setTextSize(4);
    tft.setCursor(60, 56);
    tft.print(buf);

    // SOC bar (rounded + highlight, konsisten dengan drawMainFull)
    int barW = (int)(200 * bms.soc / 100);
    tft.fillRoundRect(19, 100, 202, 14, 3, tft.color565(0, 10, 25));
    tft.drawRoundRect(19, 100, 202, 14, 3, DIM);
    if (barW > 0) {
        tft.fillRoundRect(20, 101, barW, 12, 3, colorSOC(bms.soc));
        uint16_t hlt = colorSOC(bms.soc) == TFT_RED    ? tft.color565(255, 90,  90)  :
                       colorSOC(bms.soc) == TFT_YELLOW ? tft.color565(255, 255, 130) :
                                                          tft.color565(130, 255, 130);
        tft.drawFastHLine(20, 102, barW > 2 ? barW - 2 : barW, hlt);
    }

    // Voltase
    tft.fillRect(8, 136, 100, 16, BG);
    snprintf(buf, sizeof(buf), "%.2fV", bms.total_v);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(8, 136);
    tft.print(buf);

    // Arus
    tft.fillRect(126, 136, 100, 16, BG);
    snprintf(buf, sizeof(buf), "%+.1fA", bms.current_a);
    tft.setTextColor(colorCurrent(bms.current_a));
    tft.setCursor(126, 136);
    tft.print(buf);

    // Daya + badge status
    tft.fillRect(8, 172, 102, 20, BG);
    float pwr = bms.total_v * bms.current_a;
    snprintf(buf, sizeof(buf), "%+.0fW", pwr);
    tft.setTextColor(colorCurrent(pwr));
    tft.setTextSize(2);
    tft.setCursor(8, 174);
    tft.print(buf);
    drawStatusBadges(bms);

    // Suhu
    tft.fillRect(8, 212, 60, 10, BG);
    snprintf(buf, sizeof(buf), "%.1fC", bms.temp_mos);
    tft.setTextColor(colorTemp(bms.temp_mos));
    tft.setTextSize(1);
    tft.setCursor(8, 212);
    tft.print(buf);

    tft.fillRect(86, 212, 60, 10, BG);
    if (bms.temp_bat1_valid) {
        snprintf(buf, sizeof(buf), "%.1fC", bms.temp_bat1);
        tft.setTextColor(colorTemp(bms.temp_bat1));
    } else {
        strcpy(buf, "---");
        tft.setTextColor(tft.color565(60, 80, 100));
    }
    tft.setCursor(86, 212);
    tft.print(buf);

    tft.fillRect(164, 212, 60, 10, BG);
    if (bms.temp_bat2_valid) {
        snprintf(buf, sizeof(buf), "%.1fC", bms.temp_bat2);
        tft.setTextColor(colorTemp(bms.temp_bat2));
    } else {
        strcpy(buf, "---");
        tft.setTextColor(tft.color565(60, 80, 100));
    }
    tft.setCursor(164, 212);
    tft.print(buf);

    // Kapasitas & Siklus
    tft.fillRect(8, 242, 224, 10, BG);
    tft.setTextColor(TFT_WHITE);
    snprintf(buf, sizeof(buf), "%.1fAh", bms.remain_ah);
    tft.setCursor(8, 242);
    tft.print(buf);

    snprintf(buf, sizeof(buf), "%.1fAh", bms.cap_ah);
    tft.setCursor(100, 242);
    tft.print(buf);

    snprintf(buf, sizeof(buf), "%lu", bms.cycle_cnt);
    tft.setCursor(172, 242);
    tft.print(buf);

    // Alarm
    tft.fillRect(55, 261, 160, 10, BG);
    if (bms.alarm_flags == 0) {
        tft.setTextColor(TFT_GREEN);
        tft.setCursor(55, 261);
        tft.print("NORMAL");
    } else {
        tft.setTextColor(TFT_RED);
        snprintf(buf, sizeof(buf), "0x%08X", bms.alarm_flags);
        tft.setCursor(55, 261);
        tft.print(buf);
    }
}

// ============================================================
// Gambar Halaman Voltase Sel
// ============================================================
void drawCellsFull() {
    tft.fillScreen(TFT_BLACK);
    drawHeader(bleState == BLE_CONNECTED, "Voltase Sel");

    if (!bms.valid || bms.num_cells == 0) {
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.setTextSize(1); tft.setCursor(30, 160);
        tft.print("Belum ada data sel.");
        return;
    }

    // Hitung min/max untuk pewarnaan
    uint16_t mn = bms.cell_mv[0], mx = bms.cell_mv[0];
    for (int i = 1; i < bms.num_cells; i++) {
        if (bms.cell_mv[i] < mn) mn = bms.cell_mv[i];
        if (bms.cell_mv[i] > mx) mx = bms.cell_mv[i];
    }

    // Baris ringkasan
    char buf[24];
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextSize(1);
    snprintf(buf, sizeof(buf), "Min:%d  Max:%d  D:%d mV", mn, mx, mx - mn);
    tft.setCursor(6, 40); tft.print(buf);

    tft.drawFastHLine(0, 52, 240, TFT_DARKGREY);

    // Grid sel: 3 kolom × n baris
    // Setiap sel: lebar 78px, tinggi 28px
    const int COLS = 3;
    const int CW   = 78;  // lebar kolom
    const int CH   = 28;  // tinggi baris
    const int XOFF = 3;
    const int YOFF = 56;

    int maxVisible = (240 / CH) > 9 ? 9 : (240 / CH); // maks baris
    maxVisible = (320 - YOFF - 30) / CH;               // ~8 baris

    for (int i = 0; i < bms.num_cells; i++) {
        int row = i / COLS;
        int col = i % COLS;
        if (row >= maxVisible) break;

        int x = XOFF + col * (CW + 2);
        int y = YOFF + row * CH;

        uint16_t clr = colorCell(bms.cell_mv[i], mn, mx);
        tft.drawRect(x, y, CW, CH - 2, clr);

        // Nomor sel
        snprintf(buf, sizeof(buf), "C%02d", i + 1);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.setCursor(x + 2, y + 3);
        tft.print(buf);

        // Voltase
        snprintf(buf, sizeof(buf), "%d", bms.cell_mv[i]);
        tft.setTextColor(clr, TFT_BLACK);
        tft.setCursor(x + 2, y + 14);
        tft.print(buf);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.print("mV");
    }

    // Jika ada sel lebih dari yang ditampilkan
    if (bms.num_cells > COLS * maxVisible) {
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setCursor(8, YOFF + maxVisible * CH + 2);
        snprintf(buf, sizeof(buf), "+%d sel tidak tampil",
                 bms.num_cells - COLS * maxVisible);
        tft.print(buf);
    }

    // Footer
    tft.fillRect(0, 295, 240, 25, TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setTextSize(1);
    tft.setCursor(42, 303);
    tft.print("TAP: Lihat Resistansi");
}

// ============================================================
// Update Inkremental Halaman Voltase Sel (hindari flicker)
// ============================================================
void updateCellsScreen() {
    if (!bms.valid || bms.num_cells == 0) return;
    char buf[24];

    // Hitung min/max
    uint16_t mn = bms.cell_mv[0], mx = bms.cell_mv[0];
    for (int i = 1; i < bms.num_cells; i++) {
        if (bms.cell_mv[i] < mn) mn = bms.cell_mv[i];
        if (bms.cell_mv[i] > mx) mx = bms.cell_mv[i];
    }

    // Update baris ringkasan (hapus baris dulu)
    tft.fillRect(0, 37, 240, 14, TFT_BLACK);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextSize(1);
    snprintf(buf, sizeof(buf), "Min:%d  Max:%d  D:%d mV", mn, mx, mx - mn);
    tft.setCursor(6, 40);
    tft.print(buf);

    // Update tiap sel: border + voltase saja (label C01 tidak berubah)
    const int COLS = 3, CW = 78, CH = 28, XOFF = 3, YOFF = 56;
    const int maxVisible = (320 - YOFF - 30) / CH;

    for (int i = 0; i < bms.num_cells; i++) {
        int row = i / COLS, col = i % COLS;
        if (row >= maxVisible) break;
        int x = XOFF + col * (CW + 2);
        int y = YOFF + row * CH;

        uint16_t clr = colorCell(bms.cell_mv[i], mn, mx);
        // Perbarui warna border
        tft.drawRect(x, y, CW, CH - 2, clr);
        // Hapus area voltase lalu tulis ulang
        tft.fillRect(x + 2, y + 14, CW - 4, 9, TFT_BLACK);
        snprintf(buf, sizeof(buf), "%d", bms.cell_mv[i]);
        tft.setTextColor(clr, TFT_BLACK);
        tft.setCursor(x + 2, y + 14);
        tft.print(buf);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.print("mV");
    }
}

// ============================================================
// Halaman 2: Resistansi Internal Sel
// ============================================================
void drawResistanceFull() {
    tft.fillScreen(TFT_BLACK);
    drawHeader(bleState == BLE_CONNECTED, "Resistansi Sel");

    const int COLS  = 3;
    const int CW    = 78;   // lebar kotak sel
    const int CH    = 28;   // tinggi kotak sel
    const int XOFF  = 3;    // margin kiri
    const int YOFF  = 56;   // mulai setelah header

    int n = (bms.num_cells > 0) ? bms.num_cells : 0;

    if (!bms.cell_res_valid) {
        // Tidak ada data resistansi
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, 120);
        tft.println("Data resistansi tidak tersedia.");
        tft.setCursor(10, 138);
        tft.println("Hanya protokol lama (55AAEB90)");
        tft.setCursor(10, 156);
        tft.println("yang membawa data ini.");
    } else {
        for (int i = 0; i < n; i++) {
            int col = i % COLS;
            int row = i / COLS;
            int x   = XOFF + col * (CW + 3);
            int y   = YOFF + row * (CH + 3);

            uint16_t mohm = bms.cell_res_mohm[i];
            uint16_t fc   = colorRes(mohm);

            tft.fillRect(x, y, CW, CH, tft.color565(10, 20, 30));
            tft.drawRect(x, y, CW, CH, tft.color565(30, 50, 70));

            // Label sel
            char lbl[5];
            snprintf(lbl, sizeof(lbl), "C%02d", i + 1);
            tft.setTextSize(1);
            tft.setTextColor(TFT_DARKGREY, tft.color565(10, 20, 30));
            tft.setCursor(x + 3, y + 3);
            tft.print(lbl);

            // Nilai resistansi
            char val[8];
            if (mohm == 0) {
                snprintf(val, sizeof(val), "---");
            } else if (mohm < 1000) {
                snprintf(val, sizeof(val), "%umO", mohm);
            } else {
                // >= 1000 mΩ: tampilkan dalam Ω dengan 1 desimal
                snprintf(val, sizeof(val), "%d.%dO", mohm / 1000, (mohm % 1000) / 100);
            }
            tft.setTextSize(1);
            tft.setTextColor(fc, tft.color565(10, 20, 30));
            // Pusatkan nilai secara horizontal
            int vw = strlen(val) * 6;
            tft.setCursor(x + (CW - vw) / 2, y + 14);
            tft.print(val);
        }
    }

    // Footer
    tft.fillRect(0, 295, 240, 25, TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setTextSize(1);
    tft.setCursor(55, 303);
    tft.print("TAP: Kembali ke Utama");
}

void updateResistanceScreen() {
    if (!bms.valid) return;

    const int COLS  = 3;
    const int CW    = 78;
    const int CH    = 28;
    const int XOFF  = 3;
    const int YOFF  = 56;

    int n = (bms.num_cells > 0) ? bms.num_cells : 0;

    if (!bms.cell_res_valid) return; // tidak ada data, layar sudah menampilkan pesan

    for (int i = 0; i < n; i++) {
        int col = i % COLS;
        int row = i / COLS;
        int x   = XOFF + col * (CW + 3);
        int y   = YOFF + row * (CH + 3);

        uint16_t mohm = bms.cell_res_mohm[i];
        uint16_t fc   = colorRes(mohm);

        // Hapus area nilai saja (baris bawah kotak)
        tft.fillRect(x + 1, y + 12, CW - 2, CH - 13, tft.color565(10, 20, 30));

        char val[8];
        if (mohm == 0) {
            snprintf(val, sizeof(val), "---");
        } else if (mohm < 1000) {
            snprintf(val, sizeof(val), "%umO", mohm);
        } else {
            snprintf(val, sizeof(val), "%d.%dO", mohm / 1000, (mohm % 1000) / 100);
        }
        tft.setTextSize(1);
        tft.setTextColor(fc, tft.color565(10, 20, 30));
        int vw = strlen(val) * 6;
        tft.setCursor(x + (CW - vw) / 2, y + 14);
        tft.print(val);
    }
}

// ============================================================
// Splash Boot Indah
// ============================================================
static void drawBootBattery(int bx, int by, int bw, int bh) {
    // Terminal positif di kanan
    int termH = bh / 3;
    int termY = by + (bh - termH) / 2;
    tft.fillRoundRect(bx + bw, termY, 9, termH, 2, TFT_CYAN);

    // Badan baterai — latar gelap
    tft.fillRoundRect(bx, by, bw, bh, 6, tft.color565(5, 20, 40));
    tft.drawRoundRect(bx,     by,     bw,     bh,     6, TFT_CYAN);
    tft.drawRoundRect(bx + 1, by + 1, bw - 2, bh - 2, 5, tft.color565(0, 60, 100));

    // Isi baterai 80% — hijau
    int fillW = (bw - 10) * 80 / 100;
    tft.fillRoundRect(bx + 5, by + 5, fillW, bh - 10, 3, tft.color565(0, 160, 60));
    // Highlight atas (kilap)
    tft.fillRect(bx + 5, by + 5, fillW, 4, tft.color565(60, 230, 130));
    // Garis terang di ujung isi
    tft.drawFastVLine(bx + 5 + fillW, by + 5, bh - 10, tft.color565(80, 255, 150));

    // Simbol petir (kuning) di tengah baterai
    int cx = bx + bw / 2;
    int t  = by + 7;
    int b2 = by + bh - 7;
    int m  = (t + b2) / 2;
    tft.drawLine(cx + 6, t,  cx - 2, m,  TFT_YELLOW);
    tft.drawLine(cx - 2, m,  cx + 3, m,  TFT_YELLOW);
    tft.drawLine(cx + 3, m,  cx - 6, b2, TFT_YELLOW);
    tft.drawLine(cx + 7, t,  cx - 1, m,  TFT_YELLOW);
    tft.drawLine(cx - 1, m,  cx + 4, m,  TFT_YELLOW);
    tft.drawLine(cx + 4, m,  cx - 5, b2, TFT_YELLOW);
}

static void updateSplashProgress(uint8_t percent, const char* msg) {
    const uint16_t BG = tft.color565(0, 5, 20);

    // Isi progress bar
    int barW = (int)(196 * percent / 100);
    if (barW > 196) barW = 196;
    if (barW > 0) {
        tft.fillRoundRect(22, 267, barW, 14, 4, tft.color565(0, 140, 160));
        // Highlight baris atas
        tft.drawFastHLine(22, 268, barW > 1 ? barW - 1 : 1,
                          tft.color565(100, 240, 255));
        // Kilap di ujung bar
        tft.fillRect(22 + barW - 3 < 22 ? 22 : 22 + barW - 3,
                     267, 3, 14, tft.color565(60, 210, 230));
    }

    // Update label status
    if (msg) {
        tft.fillRect(20, 244, 200, 12, BG);
        tft.setTextColor(tft.color565(80, 150, 180));
        tft.setTextSize(1);
        int msgW = (int)strlen(msg) * 6;
        tft.setCursor(120 - msgW / 2, 247);
        tft.print(msg);
    }
}

static void drawSplash() {
    const uint16_t BG     = tft.color565(0, 5, 20);
    const uint16_t ACCENT = TFT_CYAN;
    const uint16_t DIM    = tft.color565(0, 60, 100);

    tft.fillScreen(BG);

    // Baris aksen warna atas & bawah
    tft.fillRect(0,   0, 240, 4, ACCENT);
    tft.fillRect(0,   4, 240, 1, DIM);
    tft.fillRect(0, 315, 240, 4, ACCENT);
    tft.fillRect(0, 314, 240, 1, DIM);

    // ── Ikon baterai (terpusat 240px) ────────────────────────
    // Body 90×48 + terminal 9px = total lebar 99, mulai x=71
    drawBootBattery(71, 12, 90, 48);

    // ── Judul ────────────────────────────────────────────────
    // "JK BMS" textSize=3: 6 chars × 18px = 108px → x = 66
    tft.setTextColor(ACCENT);
    tft.setTextSize(3);
    tft.setCursor(66, 70);
    tft.print("JK BMS");

    // "Monitor" textSize=2: 7 chars × 12px = 84px → x = 78
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(78, 98);
    tft.print("Monitor");

    // ── Garis dekoratif dengan titik ────────────────────────
    tft.drawFastHLine(20, 124, 200, DIM);
    tft.fillRect(114, 121, 12, 7, BG);   // celah untuk titik tengah
    tft.fillCircle(120, 124, 3, ACCENT);
    tft.fillCircle(85,  124, 2, DIM);
    tft.fillCircle(155, 124, 2, DIM);

    // ── Info hardware ─────────────────────────────────────────
    // "ESP32-2432S028R" 15 chars × 6px = 90px → x = 75
    tft.setTextColor(tft.color565(100, 180, 200));
    tft.setTextSize(1);
    tft.setCursor(75, 133);
    tft.print("ESP32-2432S028R");

    // Tagline: 28 chars × 6px = 168px → x = 36
    tft.setTextColor(tft.color565(50, 110, 140));
    tft.setCursor(36, 147);
    tft.print("Jikong BMS Bluetooth Monitor");

    // ── Badge fitur (3 badge, lebar 64px masing-masing) ─────
    // Badge 1 — BT BLE (x=12..76, center=44)
    tft.fillRoundRect(12, 162, 64, 20, 4, tft.color565(0, 20, 50));
    tft.drawRoundRect(12, 162, 64, 20, 4, ACCENT);
    tft.setTextColor(ACCENT);
    tft.setCursor(26, 168); tft.print("BT 5.0");

    // Badge 2 — ILI9341 (x=88..152, center=120)
    tft.fillRoundRect(88, 162, 64, 20, 4, tft.color565(0, 30, 10));
    tft.drawRoundRect(88, 162, 64, 20, 4, TFT_GREEN);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(99, 168); tft.print("ILI9341");

    // Badge 3 — XPT2046 (x=164..228, center=196)
    tft.fillRoundRect(164, 162, 64, 20, 4, tft.color565(35, 25, 0));
    tft.drawRoundRect(164, 162, 64, 20, 4, TFT_YELLOW);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(175, 168); tft.print("XPT2046");

    // ── Versi ─────────────────────────────────────────────────
    // "v1.2.1 22 May 2026" 14 chars × 6px = 84px → x = 78
    tft.setTextColor(tft.color565(40, 80, 100));
    tft.setCursor(78, 190);
    tft.print("v1.2.1 22 May 2026");

    // ── Garis pembatas area loading ───────────────────────────
    tft.drawFastHLine(20, 207, 200, tft.color565(0, 25, 45));

    // ── Progress bar ─────────────────────────────────────────
    tft.drawRoundRect(20, 265, 200, 18, 6, DIM);
    tft.fillRoundRect(21, 266, 198, 16, 5, tft.color565(0, 8, 20));

    // Label awal progress
    tft.setTextColor(tft.color565(60, 120, 150));
    // "Menginisialisasi..." 19 chars × 6px = 114px → x = 63
    tft.setCursor(63, 247);
    tft.print("Menginisialisasi...");
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== JK BMS Monitor - ESP32-2432S028R ===");

    // LED
    pinMode(LED_RED,   OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE,  OUTPUT);
    setLED(false, false, true); // biru = booting

    // TFT
    tft.init();
    tft.setRotation(0);
    pinMode(21, OUTPUT);  // TFT Backlight pin
    digitalWrite(21, HIGH);
    tft.fillScreen(TFT_BLACK);

    // ── Splash screen indah ───────────────────────────────────
    drawSplash();
    updateSplashProgress(15, "TFT OK...");

    // Touch (VSPI)
    vspi.begin(25, 39, 32, 33); // CLK, MISO, MOSI, CS
    touch.begin(vspi);
    touch.setRotation(0);
    updateSplashProgress(30, "Touch OK...");

    // BLE - inisialisasi stack
    delay(200); // beri waktu sebelum BT init
    Serial.println("[BLE] Inisialisasi NimBLE...");
    updateSplashProgress(50, "Inisialisasi BLE...");
    NimBLEDevice::init("");
    NimBLEDevice::setMTU(512);  // minta MTU besar agar respons BMS tidak terpotong
    Serial.println("[BLE] NimBLE OK, konfigurasi scan...");
    updateSplashProgress(75, "Konfigurasi BLE...");

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new JKScanCallbacks());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(50);
    updateSplashProgress(100, "Siap!");
    // Countdown 5 detik di splash sebelum mulai scan BLE
    for (int cnt = 5; cnt >= 1; cnt--) {
        char cbuf[22];
        snprintf(cbuf, sizeof(cbuf), "BLE mulai dalam %d...", cnt);
        updateSplashProgress(100, cbuf);
        delay(1000);
    }

    // Scan dimulai dari loop() — tidak block setup()
    bleState = BLE_SCANNING;
    Serial.println("[BLE] Siap. Scan akan dimulai di loop...");

    drawScanScreen("Scan BLE aktif...");
    setLED(false, false, false);
}

// ============================================================
// Loop
// ============================================================
void loop() {
    uint32_t now = millis();

    // ---- Mulai scan BLE pada iterasi pertama ----
    if (!scanStarted && bleState == BLE_SCANNING) {
        scanStarted = true;
        lastScanRestartMs = now;
        Serial.println("[BLE] Memulai scan...");
        NimBLEDevice::getScan()->start(8, false); // scan 8 detik, non-blocking
    }

    // ---- Setelah scan 8 detik selesai, langsung tampilkan daftar pilih ----
    if (bleState == BLE_SCANNING && scanStarted &&
        !NimBLEDevice::getScan()->isScanning() &&
        now - lastScanRestartMs >= 8000) {
        Serial.printf("[BLE] Scan selesai, ditemukan %d perangkat → daftar pilih\n", scanDeviceCount);
        bleState = BLE_SELECT;
        needFullRedraw = true;
    }

    // ---- Restart scan dari layar daftar pilih setiap 12 detik jika tombol SCAN ULANG tidak ditekan ----
    if (bleState == BLE_SELECT &&
        now - lastScanRestartMs > 12000 &&
        !NimBLEDevice::getScan()->isScanning()) {
        lastScanRestartMs = now;
        Serial.println("[BLE] Auto-rescan...");
        NimBLEDevice::getScan()->start(8, false);
        needFullRedraw = true;
    }

    // ---- Manajemen LED ----
    if (now - ledMs >= 800) {
        ledMs = now;
        switch (bleState) {
            case BLE_SCANNING:   setLED(false, false, true);  break; // biru
            case BLE_SELECT:     setLED(false, true,  true);  break; // cyan
            case BLE_CONNECTING: setLED(true,  true,  false); break; // kuning
            case BLE_CONNECTED:  setLED(false, ledPhase % 2, false); ledPhase++; break; // hijau kedip
            default:             setLED(true,  false, false); break; // merah
        }
    }

    // ---- State machine BLE ----
    if (bleState == BLE_CONNECTING) {
        drawScanScreen("Menghubungkan...");
        if (!connectToBMS()) {
            // Gagal → kembali ke daftar jika ada, atau scan ulang
            if (scanDeviceCount > 0) {
                bleState = BLE_SELECT;
            } else {
                bleState    = BLE_SCANNING;
                scanStarted = false;
                NimBLEDevice::getScan()->start(30, false);
            }
        }
        needFullRedraw = true;
    }

    // ---- Tampilan ----
    if (needFullRedraw) {
        needFullRedraw = false;
        if (bleState == BLE_SCANNING) {
            drawScanScreen("Mencari JK/ANT BMS via BLE...");
        } else if (bleState == BLE_SELECT) {
            drawSelectScreen();
        } else if (bleState == BLE_CONNECTED || bms.valid) {
            if (currentPage == 0)      drawMainFull();
            else if (currentPage == 1) drawCellsFull();
            else                       drawResistanceFull();
        }
    }

    // ---- Update inkremental (setiap ada data baru) ----
    if (newDataReady && bleState == BLE_CONNECTED) {
        newDataReady = false;
        if (bms.valid) {
            if (!bmsScreenReady) {
                // Data pertama → full redraw agar semua label tergambar
                bmsScreenReady = true;
                needFullRedraw = true;
            } else if (currentPage == 0) {
                updateMainScreen();
            } else if (currentPage == 1) {
                updateCellsScreen();
            } else {
                updateResistanceScreen();
            }
        }
    }

    // ---- Kirim request data ke BMS setiap 5 detik ----
    if (bleState == BLE_CONNECTED && now - lastRequestMs >= REQUEST_INTERVAL_MS) {
        lastRequestMs = now;
        if (connectedBmsType == BMS_ANT) {
            if (pJKChar) pJKChar->writeValue((uint8_t*)CMD_ANT_STATUS, sizeof(CMD_ANT_STATUS), false);
        } else if (legacyAckSeen && (pJKCharLegacy || pJKCharLegacyAlt || pJKCharLegacyExtra)) {
            if (lastFrameMs == 0) {
                // ACK-only mode: kirim probe bertahap, hindari flood command.
                sendLegacyAltProbe();
            } else {
                sendLegacyCellInfoPoll();
            }
        } else if (pJKChar) {
            Serial.println("[POLL] CMD_CELL_INFO -> FFE2 (chk=0x10)");
            pJKChar->writeValue((uint8_t*)CMD_CELL_INFO, sizeof(CMD_CELL_INFO), false);
        }
    }

    // ---- Recovery untuk BMS yang connect tapi tidak pernah mengirim notify (JK only) ----
    if (bleState == BLE_CONNECTED && connectedBmsType == BMS_JK && pJKChar && pJKCharAlt &&
        lastFrameMs == 0 && noDataRetryCount < 3 &&
        now > 6000 && now - lastRequestMs > 2500) {
        noDataRetryCount++;
        lastRequestMs = now;
        if (pJKCharLegacy || pJKCharLegacyAlt || pJKCharLegacyExtra) {
            Serial.printf("[BLE] No data after connect, retry legacy init #%d via FF10...\n",
                          noDataRetryCount);
            sendLegacyInitProbe();
            sendLegacyAltProbe();
        } else {
            Serial.printf("[BLE] No data after connect, retry alt init #%d via FFE1...\n",
                          noDataRetryCount);
            subscribeNotifyIfSupported(pJKCharAlt, "FFE1 retry");
            sendInitCommands(pJKCharAlt, "FFE1 retry");
        }
    }

    // ---- Deteksi BMS stuck AT mode: coba AT+EXIT lalu retry via FFE1 (JK only) ----
    if (bleState == BLE_CONNECTED && connectedBmsType == BMS_JK && pJKChar &&
        atRxCount >= 8 && !atEscapeSent &&
        now - lastAtEscapeMs > 5000) {
        lastAtEscapeMs = now;
        atEscapeSent   = true;
        Serial.printf("[BLE] AT mode? (%d AT\\r\\n), coba AT+EXIT...\n", atRxCount);
        static const uint8_t AT_EXIT[] = {'A','T','+','E','X','I','T','\r','\n'};
        pJKChar->writeValue((uint8_t*)AT_EXIT, sizeof(AT_EXIT), false);
        delay(500);
        // Retry init via FFE1 (alternate channel)
        NimBLERemoteCharacteristic* ch = pJKCharAlt ? pJKCharAlt : pJKChar;
        Serial.println("[BLE] Retry CMD_DEV_INFO -> FFE1");
        ch->writeValue((uint8_t*)CMD_DEV_INFO,  sizeof(CMD_DEV_INFO),  false);
        delay(500);
        ch->writeValue((uint8_t*)CMD_CELL_INFO, sizeof(CMD_CELL_INFO), false);
    }

    // ---- Deteksi sentuhan layar ----
    if (now - lastTouchMs >= TOUCH_DEBOUNCE_MS && touch.tirqTouched() && touch.touched()) {
        lastTouchMs = now;
        TS_Point tp = touch.getPoint();
        // Kalibrasi koordinat touch → layar (sesuaikan jika perlu)
        int tx = (int)map(tp.x, 200, 3800, 0, 239);
        int ty = (int)map(tp.y, 200, 3800, 0, 319);
        tx = constrain(tx, 0, 239);
        ty = constrain(ty, 0, 319);
        Serial.printf("[TOUCH] raw=(%d,%d) screen=(%d,%d)\n", tp.x, tp.y, tx, ty);

        if (bleState == BLE_SELECT) {
            if (ty >= 285) {
                // Tombol Scan Ulang: reset daftar dan mulai scan baru
                scanDeviceCount   = 0;
                scanStarted       = true;
                lastScanRestartMs = millis();
                bleState          = BLE_SELECT;
                needFullRedraw    = true;
                if (!NimBLEDevice::getScan()->isScanning())
                    NimBLEDevice::getScan()->start(8, false);
                Serial.println("[TOUCH] Scan ulang...");
            } else {
                // Tap pada item daftar (y = 54 + i*46, tinggi 42)
                for (int i = 0; i < scanDeviceCount; i++) {
                    int itemY = 54 + i * 46;
                    if (ty >= itemY && ty < itemY + 42) {
                        selectedBLEAddr    = scanDevices[i].bleAddr;
                        selectedDeviceType = scanDevices[i].bmsType;
                        bleState           = BLE_CONNECTING;
                        needFullRedraw     = true;
                        Serial.printf("[TOUCH] Pilih: %s (%s)\n", scanDevices[i].name,
                                      scanDevices[i].bmsType == BMS_ANT ? "ANT" : "JK");
                        break;
                    }
                }
            }
        } else if (bleState == BLE_CONNECTED) {
            currentPage    = (currentPage >= 2) ? 0 : currentPage + 1;
            needFullRedraw = true;
            Serial.printf("[TOUCH] Ganti ke halaman %d\n", currentPage);
        }
    }

    // ---- Watchdog: jika koneksi hilang, kembali ke daftar (jangan hapus device list) ----
    if (bleState == BLE_CONNECTED && pClient && !pClient->isConnected()) {
        pJKChar        = nullptr;
        pJKCharAlt     = nullptr;
        bmsScreenReady = false;
        needFullRedraw = true;
        if (scanDeviceCount > 0) {
            bleState = BLE_SELECT;
        } else {
            bleState    = BLE_SCANNING;
            scanStarted = false;
            NimBLEDevice::getScan()->start(30, false);
        }
    }

    delay(10);
}
