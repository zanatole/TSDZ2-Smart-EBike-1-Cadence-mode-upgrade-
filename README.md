Cadence Punch and Sliding Power Control
🎯 Purpose​
The firmware adds a “punch” feature: a short boost of assistance when you suddenly accelerate your pedaling cadence or apply strong torque. This makes the bike feel more responsive in traffic or when restarting, while still staying within legal limits thanks to a sliding average power limiter.


🔹 Punch Trigger​
The punch can be activated in two ways:

Cadence acceleration
The controller compares the average cadence over ~0.75 s (WINDOW_PAST) with the average over ~0.3 s (WINDOW_CURRENT).
If the short‑term cadence is at least +3% higher (punch_threshold_percent = 103) and above 50 rpm (punch_cadence_floor), the punch is triggered.
High pedal torque
If the pedal torque ADC value exceeds a threshold (TORQUE_PUNCH_THRESHOLD = 220), the punch is also triggered, even without cadence acceleration.

🔹 Punch Duration and Intensity​
punch_duration[] → defines how long the punch lasts depending on assist level (e.g. 3 s in ECO, 10 s in TURBO).
punch_coef_x100[] → defines the intensity multiplier (e.g. 110 = +10% in ECO, 125 = +25% in TURBO).
PUNCH_RAMPDOWN_SEC → smooth ramp‑down time after punch (default 2 s).
PUNCH_COOLDOWN_SEC → minimum cooldown before another punch can trigger (default 3 s).
rampdown_steps → number of steps for the ramp‑down smoothing (default 20).

🔹 Sliding Power Control (Street Compliance)​
To remain legal, the firmware continuously monitors average power over 2 minutes:

POWER_BUFFER_SIZE = 120 → 120 samples at 1 Hz = 2 minutes.
THRESHOLD_REDUCE = 80 → if average power > 80% of Street limit, punch is reduced (shorter).
THRESHOLD_BLOCK = 92 → if average power > 92% of Street limit, punch is blocked.
STREET_MODE_POWER_LIMIT → defines the legal Street limit (e.g. 290 W for me because it give less than 250W on shaft of motor<250W legal).
👉 Even if the punch temporarily boosts power, the 2‑minute sliding average ensures the motor never exceeds the legal Street limit in sustained use (250W on the shaft is legal limit for everage power).


✅ Summary​
Punch = short boost triggered by cadence acceleration or high torque, only above a minimum cadence.
Duration and strength are set by punch_duration[] and punch_coef_x100[].
Ramp‑down and cooldown make the punch smooth and predictable.
A 2‑minute sliding power limiter ensures the bike always respects the Street power limit, so the assistance never exceeds what is legally allowed.
⚖️ Legal Compliance​
For legal reasons, the firmware always enforces the Street power limit. Even if punch temporarily boosts assistance (by torque or cadence), the 2‑minute sliding average is capped at STREET_MODE_POWER_LIMIT.

If the average approaches the limit, punch is reduced or blocked.
This ensures the motor never exceeds the legal Street assistance limit in sustained use.
Offroad mode is only used internally during punch, but the controller automatically returns to Street mode afterwards.

----------------------------------------------------------------------------------------------------------------------------------
The parameters:
1. Punch Trigger (cadence and torque)​
punch_cadence_floor Minimum cadence (in RPM) required before punch can trigger.Example: 50 → punch only works above 50 rpm.
punch_threshold_percent Relative cadence increase threshold (in %) over a short window.Example: 103 → punch triggers if cadence rises by +3% within ~300 ms.
TORQUE_PUNCH_THRESHOLD Torque ADC threshold. If pedal torque exceeds this value, punch can also trigger.Example: 220.
WINDOW_CURRENT / WINDOW_PAST Defines the averaging windows for cadence detection.
WINDOW_CURRENT = 12 (~300 ms)
WINDOW_PAST = 30 (~750 ms)The firmware compares the short‑term average (avg_cur) with the longer one (avg_past) to detect acceleration.
2. Punch Duration and Intensity​
punch_duration[] Duration of punch per assist level (OFF, ECO, TOUR, SPORT, TURBO).Example: 3 s in ECO, 10 s in TURBO.
punch_coef_x100[] Multiplicative factor applied during punch (x100).Example: 110 = +10% in ECO, 125 = +25% in TURBO.
PUNCH_RAMPDOWN_SEC Time for smooth ramp‑down after punch (default 2 s).
PUNCH_COOLDOWN_SEC Minimum cooldown before punch can trigger again (default 3 s).
rampdown_steps Number of steps for ramp‑down smoothing (default 20).
3. Sliding Power Control (Street compliance)​
POWER_BUFFER_SIZE Size of the rolling buffer for average power (default 120 → 2 minutes).
POWER_SAMPLE_CYCLES Sampling rate (40 cycles = 1 s).
THRESHOLD_REDUCE / THRESHOLD_BLOCK Percentage thresholds of the Street power limit:
Below 80% → punch allowed.
Between 80–92% → punch reduced (shorter duration).
Above 92% → punch blocked.
STREET_MODE_POWER_LIMIT Legal Street power limit (e.g. 250 W).The firmware computes a 2‑minute sliding average of power and ensures it never exceeds this value.



-------------------------------------------------------------------------------------------------------------------------------
![GitHub issues](https://img.shields.io/github/issues/emmebrusa/TSDZ2-Smart-EBike-1) [![Build Action](../../actions/workflows/build.yaml/badge.svg)](../../actions/workflows/build.yaml)

This repository is updated by mbrusa.

This fork is based on the TSDZ2-v0.20beta1 adapted for Tongsheng protocol displays, like stock VLCD5, VLCD6, XH18 or other displays with the same protocol and 6-pin Tonsheng connector, SW102, DZ41, 850C or 860C for TSDZ2.
With these last displays, the visualization of data and errors must be checked..

Endless Sphere forum reference thread: [endless-sphere.com.](https://endless-sphere.com/forums/viewtopic.php?f=30&t=110682).

See the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/wiki) for instructions

This ebike motor controller firmware project is to be used with the Tongsheng TSDZ2 mid drive motor.
Note: firmware can't be written to Enerdan sold TSDZ2 motors and controllers because they are equipped with V2 controller and XMC1300 microprocessor instead of STM8.

It has the following benefits compared to the stock firmware:
* The motor runs more efficient therefore it becomes more powerful and consumes less energy.
* The ebike will feel more responsive and agile.
* Using other supported displays and pheriperals will provide more functionality and features.
* Because this project is in heavy development more features will be added.

This project is being developed and maintained for free by a community of users. Some of them are developers who work professionally developing this type of technology for very well known companies.

## Building and flashing with Java tool
### Windows 
- Install [SDCC](http://sdcc.sourceforge.net/index.php#Download).
  version 4.4.0 or higher required.
- Install [ST Visual Development](http://www.st.com/en/development-tools/stvd-stm8.html).
- Install [Java](https://www.java.com/endownload/).
- Open JavaConfigurator.jar customize the parameters and click Compile & Flash
- Or use supplied .bat scripts, e.g. `src/compile_and_flash.bat` 
- With 32-bit Windows systems, replace the files in the \tools\cygwin\bin folder with those in the bin_32.zip file

### Linux and MacOS
- Install [SDCC](http://sdcc.sourceforge.net/index.php#Download).
  version 4.1.0 or higher required.
- Install Stm8flash `git clone https://github.com/vdudouyt/stm8flash.git && cd stm8flash && make && sudo make install`
- Install [Java](https://www.java.com/endownload/).
- Open JavaConfigurator.jar customize the parameters and click Compile & Flash
- And/Or use supplied shell script `compile_and_flash_20.sh` 

### For more information, go to the [wiki](https://github.com/emmebrusa/TSDZ2-Smart-EBike-1/wiki) instructions.

## Development / contributing
### Setup
1. Clone this repository
2. Install the SDCC compiler

### Debugging
- You can do `cd src & make -j3`, import _elf_ into [STMStdudio](https://www.st.com/en/development-tools/stm-studio-stm8.html) and plot global variables in real-time
- Or use VScode and one of the debugging setups from `.vscode/launch.json`
 1. on Windows, unpack stm8-gdb.exe binary to folder located in your system's PATH environment variable
 2. make sure OpenOCD 0.12 is installed
 3. install [cpptools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
 4. press F5 in VScode to build, flash, and debug (`STM8-gdb` profile)

## Testing
### Setup
Initialize virtual environment (optional)::

`py -m venv .venv`

Enable virtual environment or let VScode to do it automatically:

`.venv\Scripts\activate` or `source .venv\Scripts\activate`

Install dependencies:

`pip install .`

### Usage

Run tests:

`pytest`

Any changes should have a corresponding unit test added, unless unfeasible.

Calculate coverage and generate html report (probably will not work on Windows):
`pytest --coverage`

Tests with coverage are executed in the CI as well.

### Compile the firmware manually
- `cd src/` and use `make` or `compile.bat` to compile the firmware.

### Flashing the firmware manually
- Use `make flash` or `flash.bat` to flash the firmware.
- If you have Android with OTG you can [transfer](https://dl.google.com/tag/s/appguid%3D%7B232066FE-FF4D-4C25-83B4-3F8747CF7E3A%7D%26iid%3D%7B4A198779-3904-500B-CF23-602510C07E5B%7D%26lang%3Den%26browser%3D4%26usagestats%3D0%26appname%3DNearby%2520Better%2520Together%26needsadmin%3Dtrue/better_together/BetterTogetherSetup.exe) `main.hex`to your phone and use [Stm8 updater](https://play.google.com/store/apps/details?id=com.yatrim.stlinkp8) app to flash the `stm8s105s6`
- For advanced flashing and option bytes managment use [ST Visual Programmer STM8](https://www.st.com/en/development-tools/stvp-stm8.html). You can use preconfigured project file from `tools/ST_Vision_Programming.stp`.


### Editing environment
1. VScode can be used for the development.
  a) open project top folder as workspace
  b) install extensions from recommended popup
  c) configure Intellisense by going to Settings `ctrl+,` and specifying `@id:C_Cpp.default.systemIncludePath` according to SDCC installation folder, e.g: `C:\\Program Files\\SDCC\\include`
  d) `Ctrl+Shift+b` to build the firmware


## IMPORTANT NOTES
* Installing this firmware will void your warranty of the TSDZ2 mid drive.
* We are not responsible for any personal injuries or accidents caused by use of this firmware.
* There is no guarantee of safety using this firmware, please use it at your own risk.
* We advise you to consult the laws of your country and tailor the motor configuration accordingly.
* Please be aware of your surroundings and maintain a safe riding style.
