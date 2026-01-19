# Real-Time ECG Monitor with STM32 and FreeRTOS

A real-time embedded ECG monitoring system developed using the STM32F407VGT6 microcontroller, FreeRTOS, an AD8232 ECG sensor and an SSD1306 OLED display.
The system performs ECG acquisition, real-time signal processing, heart rate detection, HRV analysis, and live visualization, using a robust multi-tasking architecture.

## Project Overview

This project implements a single-lead real-time ECG monitor using FreeRTOS to manage multiple concurrent tasks with strict timing constraints.
ECG signals are acquired via the AD8232 ECG sensor, processed using digital filtering techniques, and displayed in real time on an OLED screen.

The project was developed as part of an academic course on Real-Time Systems Design.

## Key Features

- Real-time ECG acquisition using ADC + DMA
- FreeRTOS-based multi-task architecture (4 concurrent tasks)
- Advanced signal processing pipeline
  - Median filter
  - High-pass IIR filter (baseline wander removal)
  - Adaptive baseline correction (EMA)
- Adaptive QRS peak detection
- Heart Rate (BPM) calculation
- HRV analysis (SDNN)
- Lead-off (electrode disconnection) detection
- Arrhythmia classification
  - Bradycardia
  - Tachycardia
- Real-time ECG waveform visualization on SSD1306 OLED
- Startup animation and user-friendly interface

## Hardware Components

| Component       | Model            | Description                 |
|-----------------|------------------|-----------------------------|
| Microcontroller | STM32F407VGT6    | ARM Cortex-M4 @ 168 MHz     |
| ECG Sensor      | AD8232           | Single-lead ECG acquisition |
| Display         | SSD1306 OLED     | 128×64 pixels, I2C          |
| Power Supply    | USB / External   | 5V (regulated to 3.3V)      |
| Electrodes      | 3 ECG electrodes | RA, LA, RL                  |

## Pin Configuration

### STM32F407 ↔ AD8232

| STM32 Pin | AD8232 Pin | Function                     |
|-----------|------------|------------------------------|
| PA0       | OUTPUT     | ECG analog signal (ADC1_IN0) |
| PB12      | LO+        | Right electrode detection    |
| PB13      | LO-        | Left electrode detection     |
| 3.3V      | VCC        | Power                        |
| GND       | GND        | Ground                       |

### STM32F407 ↔ SSD1306 (I2C)

| STM32 Pin | OLED Pin |
|-----------|----------|
| PB6       | SCL      |
| PB7       | SDA      |
| 3.3V      | VCC      |
| GND       | GND      |

### ECG Electrode Placement

- **RA** – Right arm or wrist
- **LA** – Left arm or wrist
- **RL** – Right leg or ankle (reference)

## Software Architecture

### FreeRTOS Task Structure

| Task         | Priority     | Period       | Description                       |
|--------------|--------------|--------------|-----------------------------------|
| ADC_Task     | High         | 5 ms         | ADC acquisition using DMA         |
| Monitor_Task | High         | 20 ms        | Lead-off detection                |
| Process_Task | Above Normal | Event-driven | Signal filtering & HR calculation |
| Display_Task | Normal       | 50 ms        | OLED rendering                    |

### Synchronization Mechanisms

- **Mutex**: Protection of shared ECG data
- **Queue**: ADC samples (100 elements)
- **Semaphore**: Display update triggering

### Data Flow
```
ADC (DMA @ 200Hz)
      ↓
ADC Queue
      ↓
Signal Filtering
      ↓
Peak Detection
      ↓
HR & HRV Calculation
      ↓
OLED Display
```

## Signal Processing Pipeline
```
Raw ADC Signal
      ↓
Median Filter (5 samples)
      ↓
High-Pass IIR Filter (fc = 0.5 Hz)
      ↓
Exponential Moving Average (EMA)
      ↓
Adaptive Peak Detection
      ↓
RR Interval Calculation
      ↓
Heart Rate & HRV (SDNN)
```

### Algorithms Used

- **Median Filter**: Removes impulsive noise while preserving QRS peaks
- **High-Pass Filter**: Removes baseline wander using α = 0.995
- **Adaptive Threshold**: `Threshold = (Signal_max − Signal_min) × 0.65`
- **Refractory Period**: 200 ms
- **Valid RR Interval**: 300 ms – 2000 ms

### Heart Rate Calculation
```
HR (BPM) = 60000 / RR_interval (ms)
```

- Smoothed using a 5-beat moving average
- Valid range: 30 – 220 BPM

### Metrics Computed

- Heart Rate (BPM)
- HRV – SDNN
- Signal Quality Indicator

## User Interface

### Startup Screen

- Beating heart animation
- Title: ECG MONITOR
- Credits: Ghofrane & Esra
- Duration: ~3 seconds

### Main Screen

- Real-time ECG waveform (scrolling)
- Heart rate display
- Status indicator:
  - NORMAL (60–100 BPM)
  - BRADY (< 60 BPM)
  - TACHY (> 100 BPM)
- Blinking heart icon synchronized with beats
- Signal quality indicator
- LEADS OFF warning when electrodes are disconnected

## Getting Started

### Prerequisites

- STM32CubeIDE ≥ 1.12
- ST-Link debugger
- FreeRTOS (included in CubeIDE)
- USB cable

### Build & Flash

1. Open the project in STM32CubeIDE
2. Build: Project → Build All
3. Flash: Run → Debug
4. Connect ECG electrodes
5. Observe ECG waveform and heart rate

## Real-Time Performance

| Task         | Execution Time |
|--------------|----------------|
| ADC_Task     | < 100 µs       |
| Process_Task | < 500 µs       |
| Display_Task | < 5 ms         |
| Monitor_Task | < 50 µs        |


## Troubleshooting

- **No display** → Check I2C wiring & address (0x3C)
- **Unstable HR** → Improve electrode contact
- **LEADS OFF** → Verify electrode connections
- **System freeze** → Increase FreeRTOS stack size

## Educational Value

This project demonstrates:

- Real-time embedded system design
- FreeRTOS task scheduling & synchronization
- Digital signal processing
- ECG bio-signal analysis
- Embedded C programming with STM32
- Hardware/software co-design

## Authors

- **Ghofrane Ben Hassan**
- **Esra Ben Ltaief**

**Supervisor**: Mr. Khaled Jelassi  
**Academic Year**: 2025–2026  
**Course**: Real-Time Systems Design

## Medical Disclaimer

**This project is for educational purposes only.**  
It is not a certified medical device and must not be used for diagnosis or treatment.
