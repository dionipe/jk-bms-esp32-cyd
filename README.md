# ESP32-2432S028R — JK BMS & ANT BMS Bluetooth Monitor

Monitor baterai **JK BMS** dan **ANT BMS** via Bluetooth Low Energy (BLE), ditampilkan pada layar sentuh TFT 2.8" bawaan board **ESP32-2432S028R** (Cheap Yellow Display / CYD).

## Fitur Utama

- **Splash screen** animasi boot dengan countdown 3 detik sebelum BLE scan dimulai
- **Scan BLE otomatis** selama 8 detik, lalu langsung masuk daftar perangkat — mendeteksi **JK BMS** dan **ANT BMS** secara bersamaan
- **Auto-rescan** setiap 12 detik dari layar pilih perangkat
- **3 halaman display** yang dapat berpindah dengan tap layar:
  - **Halaman 1 – Utama**: voltase total, arus, SOC, suhu MOS/baterai, kapasitas, siklus, status charging/discharging/balancing
  - **Halaman 2 – Voltase Sel**: grid 3 kolom dengan voltase tiap sel, warna-kode diferensial
  - **Halaman 3 – Resistansi Sel**: grid 3 kolom dengan resistansi internal tiap sel (mΩ), hanya tersedia untuk protokol lama JK (55AAEB90)
- **Dukungan dua protokol JK BMS**:
  - Protokol baru TLV `4E57` (JK02 standar BLE)
  - Protokol lama fixed-frame `55AAEB90` (JK01 / variant firmware)
- **Dukungan ANT BMS** via protokol `7E A1` (CRC-16/Modbus) — voltase sel, arus, SOC, suhu, kapasitas, status charging/discharging/balancing
- **Auto-deteksi offset** data resistansi antar varian firmware
- **LED RGB** indikator status koneksi
- **Watchdog & reconnect** otomatis jika koneksi BLE putus

## Hardware

| Komponen | Spesifikasi |
|---|---|
| Board | ESP32-2432S028R (CYD) |
| MCU | ESP32 dual-core 240 MHz |
| Display | ILI9341 2.8" TFT 240×320, SPI (HSPI) |
| Touchscreen | XPT2046, SPI (VSPI) |
| LED | RGB LED (active low) |
| Wireless | BLE via NimBLE |

### Pinout

| Komponen | Pin |
|---|---|
| TFT MOSI | GPIO 13 |
| TFT MISO | GPIO 12 |
| TFT CLK  | GPIO 14 |
| TFT CS   | GPIO 15 |
| TFT DC   | GPIO 2  |
| TFT BL (Backlight) | GPIO 21 |
| LED Merah | GPIO 4 |
| LED Hijau | GPIO 16 |
| LED Biru  | GPIO 17 |
| Touch CS  | GPIO 33 |
| Touch IRQ | GPIO 36 |

## Protokol BMS yang Didukung

### JK BMS — Protokol Baru `4E57` (JK02)
- BLE Service UUID: `0000ffe0-0000-1000-8000-00805f9b34fb`
- Notify: `FFE1` · Write: `FFE2`
- Frame header: `4E 57` (TLV tag-length-value)
- Data: voltase sel, arus, SOC, suhu, kapasitas, alarm, status MOSFET

### JK BMS — Protokol Lama `55AAEB90` (JK01 / Variant)
- Frame header: `55 AA EB 90`
- Frame type `0x02`: data sel (voltase + resistansi internal per sel)
- Resistansi sel: offset auto-detected, unit mΩ (nilai raw = mΩ langsung)
- Data tambahan: suhu, arus, SOC, kapasitas, alarm

### ANT BMS — Protokol `7E A1`
- BLE Service UUID: `0000ffe0-0000-1000-8000-00805f9b34fb`
- Notify & Write: `FFE1` (satu karakteristik untuk dua arah)
- Frame header: `7E A1` · Response type: `0x11`
- CRC: CRC-16/Modbus atas byte payload
- Request: 10-byte command `7E A1 01 00 00 BE 18 55 AA 55`
- Data: voltase tiap sel (LE uint16, mV), suhu sensor, suhu MOS, voltase total, arus, SOC, status charging/discharging/balancing, kapasitas
- Deteksi perangkat: nama BLE mengandung `"ANT"` atau `"ant"`

## Persyaratan

- [PlatformIO IDE](https://platformio.org/) (extension VS Code)
- Board ESP32-2432S028R terhubung via USB
- JK BMS **atau** ANT BMS dengan BLE aktif

## Build & Upload

```bash
# Build
pio run

# Build dan upload ke board
pio run --target upload

# Buka serial monitor (115200 baud)
pio device monitor --baud 115200
```

Atau gunakan tombol **Upload** (→) di toolbar PlatformIO VS Code.

## Struktur Proyek

```
ESP32-2432S028R/
├── src/
│   └── main.cpp          # Firmware utama
├── platformio.ini        # Konfigurasi build PlatformIO
├── README.md
└── CHANGELOG.md
```

## Serial Monitor

Buka serial monitor 115200 baud untuk melihat log diagnostik, termasuk:
- Proses BLE scan dan koneksi
- Parsed data dari BMS setiap frame
- Auto-deteksi offset resistansi (`[OldProto] finalOffset=...`)
- Outlier correction C01 jika aktif (`[OldProto] C01 outlier ...`)
