# VeloGuard

A self-contained bicycle crash detection and emergency alert device. When a crash is confirmed, it sends a GPS-linked SMS to up to three emergency contacts - no smartphone required, no subscription.

Built as an independent research project targeting the Indian cycling market, where commercial crash detection solutions (Garmin Varia, ~₹33,000) are out of reach for most riders. Total hardware cost: ~₹3,500.

---

## How it works

The device runs a four-phase finite state machine (FSM) on live accelerometer data:

1. **IDLE** - monitors acceleration magnitude continuously
2. **FREE_FALL** - triggered when acceleration drops below 3.0 m/s², indicating the bike is airborne
3. **IMPACT_SEEN** - triggered when acceleration spikes above 25.0 m/s² within 700ms of free-fall
4. **CONFIRMING** - confirms crash only if device is tilted beyond 50° and stationary after impact

All three conditions must occur in sequence. A pothole or speed bump produces an impact spike but no preceding free-fall and no sustained tilt - so it is rejected. On confirmed crash, a GPS-linked Google Maps URL is sent via SMS over 4G LTE.

---

## Hardware

| Component | Purpose | Cost |
|---|---|---|
| Waveshare ESP32-S3 SIM7670G | Microcontroller + 4G LTE + GPS | ~₹2,200 |
| MPU6050 MEMS IMU | Accelerometer + gyroscope | ~₹150 |
| Samsung 18650 30Q (3000mAh) | Battery | ~₹400 |
| 3D printed enclosure | Housing | ~₹200 (filament) |
| **Total** | | **~₹3,500** |

Wiring: SDA=GPIO15, SCL=GPIO16, MODEM_PWR=GPIO33, TX=GPIO17, RX=GPIO18

---

## Web dashboard

The device creates a WiFi access point on boot (`FallDetector`). Connect and open `192.168.4.1` to access a browser-based dashboard for live telemetry, threshold tuning, and SMS contact configuration. All settings are stored in EEPROM.

---

## Research

This project is being submitted to:
- **IRIS National Science Fair** (August 2025)
- **S.T. Yau High School Science Award** - Physics category
- **Journal of Emerging Investigators**

Research question: *How accurately can a threshold-based inertial FSM distinguish genuine bicycle crash events from common road disturbances, and what threshold combinations optimise this discrimination?*

Baseline analysis of 811,781 samples from the Bike&Safe dataset (Blauth da Silva & Tavares, 2022) shows that normal cycling produces acceleration spikes up to 111.7 m/s² - demonstrating that impact magnitude alone is insufficient for crash discrimination, and that post-impact tilt angle is the primary separating feature.

---

## References

- Bourke, A.K. et al. (2007). Evaluation of a threshold-based tri-axial accelerometer fall detection algorithm. *Gait & Posture*, 26(2), 194–199.
- Blauth da Silva, G.; Tavares, J. (2022). Bike&Safe Dataset. Mendeley Data, V2. https://data.mendeley.com/datasets/3j9yh8znj4/2
- Ministry of Road Transport & Highways, India (2022). Road Accidents in India.

---

*Yuveer Rajpal - The Shri Ram School, Moulsari, Gurugram*

