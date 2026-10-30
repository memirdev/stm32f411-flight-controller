# STM32 Quadcopter Uçuş Kontrolcüsü

[English](README.md) | **Türkçe**

STM32F411CEU6 (Black Pill) üzerinde, STM32 HAL ile C dilinde sıfırdan yazdığım
bir quadcopter uçuş kontrol yazılımı.

![Dron](docs/dron.jpg)

> Proje hâlâ geliştirme aşamasında; kod ve ayarlar değişebilir.

## Özellikler

- Açı modu: roll/pitch çubukla verilen açıda tutulur (±15°), yaw çubuğu dönüş
  hızını belirler (60 °/s)
- 100 Hz kontrol döngüsü: kumanda → IMU → tamamlayıcı filtre → PID → X mikser → ESC
- MPU6500 (SPI) IMU; ivmeölçer yalnızca ~1 g ölçerken açı hesabına katılır
- FlySky iBUS alıcı, 400 Hz ESC PWM
- Güvenlik: arm koşulları, kumanda kaybında motor durdurma, IMU hata toleransı,
  gözcü (IWDG)

## Donanım

| Parça | Kullanılan |
|---|---|
| Mikrodenetleyici | STM32F411CEU6 "Black Pill" |
| IMU | MPU6500 modülü (SPI) |
| Kumanda / alıcı | FlySky FS-i6X / FS-iA10B (iBUS) |
| ESC | 4 × Hobbywing Skywalker 30A V2 |
| Motor / pervane | 4 × A2212 1000KV / 1047 |
| Pil | 3S LiPo |

## Pin bağlantıları

![STM32F411 pin yerleşimi (CubeMX)](docs/pinout.png)

PB13–PB15 sarı görünür: SPI2 CubeMX'te değil, kodda (`mpu6500.c`) kurulur.
PB6/PB7 (I2C1) isteğe bağlı I2C IMU seçeneği içindir; PA2 kullanılmaz.

| Pin | İşlev | Bağlantı |
|---|---|---|
| PA6 / PA7 / PB0 / PB1 | TIM3 CH1–CH4 | M1 / M2 / M3 / M4 ESC |
| PA3 | USART2_RX | FS-iA10B iBUS |
| PB12 / PB13 / PB14 / PB15 | SPI2 (CS / SCK / MISO / MOSI) | MPU6500 (modülde NCS / SCL / AD0 / SDA) |

```
        ön
   M1 (CW)   M2 (CCW)
        \     /
         \   /
         /   \
        /     \
  M3 (CCW)   M4 (CW)
```

Kumanda kanalları (Mode 2): CH1 roll, CH2 pitch, CH3 gaz, CH4 yaw, CH5 arm.

## Derleme

STM32CubeIDE 1.19 ile derlenir (*File → Import → Existing Projects into
Workspace*). SPI2, gözcü ve 96 MHz saat ayarı CubeMX'te değil kodda yapılır;
bu yüzden CubeMX'te SPI2 modu kapalı kalsın.

## Bilinen sınırlamalar

- Yalnızca açı modu var; akro (rate) modu yok.
- Yaw yalnızca dönüş hızını kontrol eder, yönü tutmaz: dış bir etkiyle dönen
  dron eski yönüne kendiliğinden dönmez. Jiroskop sıfırındaki küçük kaymalar
  yavaş bir dönmeye yol açabilir; kumandanın yaw trimiyle düzeltilir.
- İrtifa tutma, GPS ve telemetri henüz yok.
- Pil gerilimi ölçülmüyor; düşük pil uyarısı yok.
- Açılıştaki jiroskop kalibrasyonu sırasında (~10 s) dron hareketsiz durmalıdır.

## Eksenler ve ayarlar

- **Eksen yönleri:** roll + sağ taraf aşağı, pitch + burun yukarı, yaw + üstten
  bakınca saat yönü. IMU modülü X ekseni ileri, Y sola, Z yukarı bakacak
  şekilde takılıdır (eksen çevrimi `imu_filter.c` içinde).
- **Motor tepkisi:** dron eğildiğinde alçakta kalan taraftaki motorlar
  hızlanır; yaw düzeltmesi CW ve CCW motor çiftlerinin hız farkıyla yapılır.
- **Seviye ofseti:** dron düzken sensörün okuduğu açı
  (`IMU_LEVEL_*_OFFSET_DEG`, `main.c`); gövdeye ve sensör montajına özeldir.
- **Ayar değerleri:** PID kazançları `main.c` içinde, en büyük açı ve yaw hızı
  `rc_input.h` içinde, kumanda kanal aralıkları `rc_input.c` içindedir; hepsi
  kendi dronuma göre ayarlandı.

## Proje yapısı

```
Core/Src/main.c                kontrol döngüsü, başlatma, arm, güvenlik
Core/Src/mpu6500.c             IMU sürücüsü
Core/Src/imu_filter.c          tamamlayıcı filtre
Core/Src/pid_controller.c      PID
Core/Src/motor_control.c       arm kontrolleri, X mikser
Core/Src/esc_pwm.c             ESC PWM
Core/Src/ibus_receiver.c       iBUS alıcı
Core/Src/rc_input.c, rc_arm.c  kumanda girişleri, arm/disarm
Core/Src/flight_diagnostics.c  titreşim ve döngü istatistikleri
```

## Planlananlar

- Tasarladığım PCB'ye geçiş
- Pil gerilimi ölçümü ve düşük pil uyarısı
- Barometreyle (BME280) irtifa tutma
- HC-12 ile telemetri ve GPS

## Lisans

[MIT](LICENSE). `Drivers/` altındaki STM32 HAL ve CMSIS dosyaları
STMicroelectronics'e aittir ve kendi lisanslarına tabidir.
