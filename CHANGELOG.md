# Changelog

Semua perubahan penting pada proyek ini dicatat di file ini.

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
