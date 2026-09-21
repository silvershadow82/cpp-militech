
# Наявне обладнання

## Схема підключень зібраного дрона

```mermaid
flowchart TB
  subgraph GROUND["Земля"]
    TX["RadioMaster Boxer<br/>пульт"]
    VRX["Video Receiver 5.8G<br/>приймач відео"]
  end

  subgraph AIR["Борт"]
    BAT["4S LiPo 3000 mAh<br/>акумулятор"]
    BUCK["D36V28F5<br/>step-down 5V 3A"]

    FC["Kakute H7<br/>польотний контролер"]
    ESC["ESC"]
    MOT["4 мотори"]
    GPS["GPS"]
    RX["ERLS RX<br/>+ антена"]
    VTX["VTX 5.8G<br/>+ антена"]
    RPI["Raspberry Pi 4B"]
    CAM["RPi Camera Module<br/>imx219"]
  end

  %% Живлення
  BAT ==>|"4S"| FC
  BAT ==>|"4S"| BUCK
  BUCK ==>|"5V 3A"| RPI

  %% Силова частина
  FC -->|"PWM / DShot"| ESC
  ESC -->|"3 фази"| MOT

  %% Периферія FC
  GPS <-->|"UART"| FC
  RX -->|"CRSF"| FC
  FC -->|"відеосигнал"| VTX

  %% Контур керування: камера -> трекінг -> MAVLink
  CAM -->|"CSI ribbon"| RPI
  RPI <-->|"UART 921600<br/>MAVLink"| FC

  %% Контур відео: накладання з RPi -> вхід камери FC -> VTX
  RPI -->|"аналогове відео<br/>(накладання)"| FC

  %% Радіоканали
  TX -.->|"RF 2.4 ГГц"| RX
  VTX -.->|"RF 5.8 ГГц"| VRX

  classDef power fill:#fde68a,stroke:#b45309,color:#111
  classDef compute fill:#bfdbfe,stroke:#1d4ed8,color:#111
  class BAT,BUCK power
  class RPI,CAM compute
```

Через дрон проходять два окремі контури:

- **Контур керування.** Камера по CSI віддає кадри на Raspberry Pi, той веде ціль і надсилає
  команди швидкості й курсу по UART як MAVLink у польотний контролер. Це шлях, який описують
  [README.md](README.md) і [docs/VISION.md](docs/VISION.md).
- **Контур відео.** Raspberry Pi малює накладання (зона захоплення, рамка цілі, стан) у фреймбуфер і
  віддає аналоговий сигнал на вхід камери польотного контролера, звідки він іде у VTX і далі на
  наземний приймач. Саме тому пілот бачить те саме, що бачить трекер — `vision.framebuffer` і
  `vision.overlay_fps` у `follow.json` керують цим контуром.

Живлення: обидва споживачі йдуть від одного 4S-акумулятора, але Raspberry Pi — через понижувальний
перетворювач D36V28F5 (5 В, 3 А), а не напряму.

## Перелік обладнання

| Component | Quantity | Info |
| --- | --- | --- |
| RPI5 | 1 | Raspberry Pi 5 8Gb |
| RPI4B | 1 | Raspberry Pi 4B 2Gb |
| RPI4 Zero 2W | 1 | Raspberry Pi Zero 2W |
| Orange PI5 | 1 | Orange Pi 5 8Gb + NPU |
| Radxa Zero | 1 | Radxa Zero 3W |
| SpeedyBee F4 | 1 | SpeedyBee F4 v3 |
| SpeedyBee H7 | 1 | SpeedyBee H7 v3 |
| Kakute H7 | 1 | Holybro Kakute H7 |
| ESP32 | 4 | ESP32 DevKit |
| SN65HVD230 | 5 | |
| BMP388 | 2 | Barometer |
| MPU9250 | 1 | |
| D36V28F5 | 1 | |
| MG996R | 4 | |
| GPS | 2 | |
| MCP2515 | 4 | |
| Hailo 8l | 1 | HAT M.2 module |
| ERLS RX | 1 | |
| Radiomaster Boxer | 1 | |
| VTX 5.8G | 1 | |
| RunCam camera | 2 | |
| Video Receiver 5.8G | 1 | |
| Google Coral | 1 | |
