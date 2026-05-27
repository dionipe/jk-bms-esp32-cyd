# Changelog

Semua perubahan penting pada proyek ini dicatat di file ini.

---

## [v1.3.1] — 2026-05-27

### Added
- Parser `parseLegacyRs485Frame()` untuk frame upload legacy RS485 `EB 90` (74 byte) dengan validasi checksum
- Dukungan parse data RS485 ke `BMSData`: voltase total, jumlah sel, voltase per-sel, flag balancing, alarm
- Probe alamat legacy tambahan `0x81` pada daftar short-probe (`55 AA <addr> FF 00 00 <chk>`)

### Changed
- Deteksi ACK pendek legacy `FC xx 06` dibuat lebih robust (tidak harus buffer persis 3 byte)
- Fast-path ACK di `notifyCallback()` agar ACK bridge (`FF10/FF12`) tetap terbaca valid meski data tercampur
- Scanner ACK pada `processRxBuffer()` kini mencari pola ACK di seluruh buffer (bukan hanya di awal buffer)
- Saat ACK legacy diterima, scheduler request dipercepat agar polling data berikutnya langsung jalan
- Polling periodik koneksi BLE diselaraskan memakai konstanta `REQUEST_INTERVAL_MS` (2 detik)

### Fixed
- Kasus perangkat legacy yang sebelumnya hanya mengirim ACK pendek tanpa data kini dapat lanjut ke poll kompatibel RS485

---

## [v1.3.0] — 2026-05-24

### Added
- **Dukungan ANT BMS** via BLE — deteksi otomatis dari nama perangkat (`"ANT"` / `"ant"`)
- Parsing frame `7E A1 11` (ANT BMS status response) dengan validasi **CRC-16/Modbus**
- `crc16Ant()` — helper CRC-16/Modbus (poly 0xA001, init 0xFFFF)
- `parseAntBmsFrame()` — dekode voltase sel, suhu sensor, suhu MOS, voltase total, arus, SOC, kapasitas, status charging/discharging/balancing
- `enum BMSType { BMS_JK, BMS_ANT }` — routing protokol terpisah untuk JK dan ANT
- Field `BMSType bmsType` pada `ScanDevice` — daftar scan menampung perangkat JK dan ANT sekaligus
- Global `connectedBmsType` dan `selectedDeviceType` untuk routing callback dan poll loop
- ANT BMS connect branch di `connectToBMS()`: subscribe `FFE1` untuk write sekaligus notify, kirim `CMD_ANT_STATUS` saat terhubung
- ANT BMS periodic poll — kirim `CMD_ANT_STATUS` setiap 5 detik
- ANT preamble buffer flush di `notifyCallback()` — reset buffer setiap ada preamble `7E A1` untuk mencegah korupsi frame akibat fragmentasi BLE
- Guard `connectedBmsType == BMS_JK` pada no-data recovery dan AT-mode recovery agar tidak mengganggu ANT BMS

### Changed
- Layar scan: judul berubah dari **"Mencari BMS JK"** → **"Cari BMS JK/ANT"**
- Info card scan: nama panduan diperbarui dari `"Nama diawali \"JK\""` → `"Nama: \"JK...\" atau \"ANT...\""`, contoh dari `JK_B2A24S, JK-B1A8S` → `JK_B2A24S, ANT BMS`
- Info card scan: teks `"Pastikan BMS JK menyala"` → `"Pastikan BMS menyala"` (generik)
- Layar pilih perangkat kosong: `"Belum ada perangkat JK BMS."` → `"Belum ada perangkat JK/ANT."`
- Loop scan text: `"Mencari JK BMS via BLE..."` → `"Mencari JK/ANT BMS via BLE..."`
- Touch handler: `selectedDeviceType` di-set dari `scanDevices[i].bmsType` saat pengguna memilih perangkat

---

## [v1.2.1]

### Added
- Halaman **Resistansi Sel** (Halaman 3) — grid 3 kolom tampilkan resistansi internal per sel dalam mΩ, hanya tersedia dari protokol lama `55AAEB90`
- Field `cell_res_mohm[24]` dan `cell_res_valid` pada struct `BMSData`
- Fungsi `colorRes()` — warna-kode resistansi: hijau (<300 mΩ), kuning (300–600), oranye (600–1000), merah (≥1000)
- Fungsi `drawResistanceFull()` dan `updateResistanceScreen()` untuk rendering halaman 3
- **Auto-deteksi offset** data resistansi dalam frame `55AAEB90` — scan byte 54–108 untuk menemukan posisi data yang valid secara otomatis (menangani perbedaan antar varian firmware JK BMS)
- **Post-correction outlier C01** — setelah auto-deteksi, jika nilai C01 menyimpang >50% dari rata-rata sel lain, offset digeser +2 byte untuk koreksi boundary field

### Changed
- Navigasi halaman (tap layar) diperluas dari 2 halaman menjadi **3 halaman**: Utama → Voltase Sel → Resistansi Sel → Utama
- Footer **Halaman Voltase Sel** diubah dari "TAP: Kembali ke Utama" → "TAP: Lihat Resistansi"
- Footer **Halaman Resistansi** menampilkan "TAP: Kembali ke Utama"

---

## [v1.2.0] — 2026-05-22

### Added
- **Splash screen countdown** — setelah progress bar 100%, tampilkan hitung mundur "BLE mulai dalam 3 / 2 / 1..." sebelum memulai scan BLE

### Changed
- Durasi BLE scan dipersingkat dari **30 detik** → **8 detik**
- Setelah scan selesai, tampilan **langsung pindah ke daftar perangkat** (`BLE_SELECT`) tanpa menunggu interaksi
- Tombol **SCAN ULANG** di layar pilih perangkat tidak lagi reset ke state `BLE_SCANNING`; tetap di `BLE_SELECT` dan memulai scan 8 detik di latar belakang
- Auto-rescan setiap **12 detik** dari layar `BLE_SELECT`

---

## [v1.1.0] — 2026-05-21

### Added
- Halaman **Voltase Sel** (Halaman 2) — grid 3 kolom dengan voltase tiap sel, warna-kode berdasarkan deviasi dari min/max
- Fungsi `drawCellsFull()` dan `updateCellsScreen()` untuk halaman sel
- Navigasi 2 halaman dengan tap layar sentuh
- Dukungan protokol lama `55AAEB90` — parsing fixed-frame JK01/variant (frame type `0x02`)
- Auto-deteksi layout firmware varian (`variantLayout`) berdasarkan posisi SOC di byte 173
- Fallback suhu dari offset tinggi (248/254) untuk varian firmware tertentu
- Legacy service `FF10` probe untuk BMS lama
- Watchdog reconnect otomatis jika koneksi BLE putus

### Changed
- Struct `BMSData` diperluas: tambah `temp_bat1_valid`, `temp_bat2_valid`, `cycle_cnt`, `remain_ah`, `alarm_flags`
- Warna indikator disesuaikan: `colorSOC()`, `colorTemp()`, `colorCurrent()`, `colorCell()`

---

## [v1.0.0] — 2026-05-20

### Added
- Proyek awal: ESP32-2432S028R sebagai monitor JK BMS via BLE
- Splash screen animasi boot
- BLE scan dan koneksi ke JK BMS (service `FFE0`, notify `FFE1`, write `FFE2`)
- Parsing protokol baru TLV `4E57` — tag-length-value per field
- Halaman **Utama** (Halaman 1): voltase total, arus, SOC, suhu MOS/baterai, kapasitas, siklus, status charging/discharging/balancing, nama BMS
- LED RGB indikator: merah = scanning, hijau = connected, biru = connecting
- Layar pilih perangkat BLE (`BLE_SELECT`) dengan tombol SCAN ULANG
- Serial Monitor output diagnostik frame BMS
