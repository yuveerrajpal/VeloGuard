# Analysis

## baseline_analysis.py
Processes the Bike&Safe dataset (Blauth da Silva & Tavares, 2022) to characterise normal cycling inertial signatures. Classifies riding into five phases (stationary, accelerating, cruising, braking, turning) and extracts the acceleration, tilt, jerk, and gyroscope envelope for each. Outputs threshold values that a crash detector must exceed to avoid false positives during normal riding.

Run with:
```
python baseline_analysis.py --data-dir /path/to/BikeAndSafe --out-dir output
```

Dataset: https://data.mendeley.com/datasets/3j9yh8znj4/2

## output/
Generated outputs from baseline_analysis.py.

`baseline_thresholds.json` — upper envelope of normal riding across 811,781 samples from 9 laps. Key finding: total acceleration reaches 111.7 m/s² in normal riding, confirming that impact magnitude alone cannot discriminate crashes from road disturbances.

`baseline_summary.csv` — per-phase descriptive statistics for all measured signals.

`phase_distributions.png` — box plots of speed, linear acceleration, gyro magnitude, and tilt across all five cycling phases.

## crashsim/
Python library for simulating and tuning the VeloGuard FSM against recorded IMU data. In development.

## data/
Raw data files are not committed to this repository. Download the Bike&Safe dataset from the link above.
