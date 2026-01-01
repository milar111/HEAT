# HEAT – Smart Hot Plate Controller

> **⚠️ HIGH‑VOLTAGE WARNING**
>
> This project operates directly from **mains AC voltage (110–240 VAC)** and drives a high‑power resistive heater through a solid‑state relay (SSR).
>
> **Building, modifying, or operating this project is inherently dangerous.**
>
> * Mains voltage can cause **serious injury or death**.
> * Fire risk exists if wiring, insulation, grounding, or firmware safety logic is incorrect.
> * By building or using this project, **you take full responsibility for all risks**.
>
> The author provides this project **for educational purposes only** and assumes **no liability** for damages, injuries, or losses of any kind.

---

## 1. Project Overview

**HEAT** is a microcontroller‑based **smart hot‑plate / reflow heater controller** designed for electronics work such as SMD soldering, thermal experiments, and controlled heating tasks.

The system combines:

* A **STM32F103C8T6** microcontroller
* A **zero‑cross AC SSR** for safe mains switching
* A **100 kΩ NTC thermistor** for temperature feedback
* A **Smart Pulse & Coast control algorithm** (not classical PID)
* An **SSD1306 OLED UI** with buttons
* A **forced‑cooling fan** with safety logic
* Persistent settings storage in **internal flash**

![0-02-05-5d3824effdbe4cbf30e6dd0a9a942f0fa9f9ccf81f6ea32574e28215983978b9_7c2cbd72a6d31529](https://github.com/user-attachments/assets/9bd01952-3055-4f48-a69a-dd3da2f71a55)

---

## 2. System Architecture (High Level)

### 2.1 Functional Blocks

* **Power Input**

  * AC mains → AC‑DC module → 5 V → LDO → 3.3 V

* **Control Core**

  * STM32F103C8T6 running a cooperative main loop

* **Sensing**

  * NTC thermistor read via ADC

* **Actuation**

  * SSR for heater control (time‑proportional)
  * DC fan via MOSFET with software PWM

* **User Interface**

  * SSD1306 OLED (I²C)
  * Three buttons

* **Non‑Volatile Storage**

  * Internal Flash (last page)

---

## 3. Hardware Design



<table>
  <tr>
    <td>
      <img width="1094" height="655" alt="Screenshot 2025-12-13 144943" src="https://github.com/user-attachments/assets/a5c5deff-9450-414a-b4ab-51347898a725" />
    </td>
    <td>
      <img width="1192" height="751" alt="Screenshot 2025-12-13 145002" src="https://github.com/user-attachments/assets/8d7afc13-9d9a-4ac3-bb80-a3ee3d34a65c" />
    </td>
  </tr>
</table>


### 3.1 MCU Core

* **MCU:** STM32F103C8T6
* **Supply:** 3.3 V

Key reasons for choosing this MCU:

* Mature HAL support
* Adequate ADC performance
* Flash storage capability
* Simple debugging (SWD)

  


<table align="center">
  <tr>
    <td>
      <img width="792" height="761" alt="Screenshot 2025-12-13 141536" src="https://github.com/user-attachments/assets/7f1db103-cbd9-4cd0-b71a-c8e0edf1df54" />
    </td>
    <td>
      <img width="678" height="865" alt="Screenshot 2025-12-13 150124" src="https://github.com/user-attachments/assets/d529bf92-650c-499c-acab-29dabd59c255"/>
    </td>
  </tr>
</table>



---

### 3.2 Power Supply

#### AC-DC Conversion and Protection

* **AC-DC Module:** HLK-PM01 (isolated offline SMPS)
* **Input Voltage:** 110–240 VAC

**Input-side protection and conditioning (mains side):**

* **Fuse (1 A):** protects wiring and PCB against catastrophic faults and shorts.
* **MOV (varistor):** clamps high-voltage transients and spikes from the mains.
* **Bulk capacitor (220 nF, X2-rated):** reduces conducted EMI and high-frequency noise on the AC line.

**Output-side conditioning (low-voltage side):**

* **Electrolytic capacitor (220 µF, polarized):** provides bulk energy storage, reduces ripple, and stabilizes load transients from the MCU, fan, and SSR drive.

**Regulation:**

* **AMS1117-3.3 LDO:** drops 5 V to 3.3 V for the MCU and logic domain.

<table>
  <tr>
    <td>
      <img width="2241" height="709" alt="image" src="https://github.com/user-attachments/assets/3ddaa7b5-0509-4810-a96d-e0f0e8e89467" />
    </td>
    <td>
      <img width="1519" height="680" alt="image" src="https://github.com/user-attachments/assets/acd6354e-d177-4913-8509-cacb7060ac66" />
    </td>
  </tr>
</table>

---

### 3.3 Heater & SSR Stage

#### Electrical Structure

* **Optotriac:** MOC3063M (zero-cross)
* **Power Triac:** BTA16-600B
* **Heater Power:** up to ~400 W resistive load
* **AC Fuse:** **3.15 A**, standard **6.3 × 32 mm (3AG)** cartridge

#### Fuse Placement and Purpose

The **3.15 A slow-blow fuse** on the heater line protects against:

* Triac failure (shorted MT1–MT2)
* Heater short circuits
* Wiring or connector faults

The slow-blow characteristic tolerates inrush current while still providing reliable fault protection.

#### How the SSR Works (Step-by-Step)

1. MCU GPIO drives the LED inside the MOC3063M
2. At the next AC zero crossing, the optotriac turns on
3. The optotriac triggers the gate of the BTA16 triac
4. The triac conducts AC current to the heater until the next zero crossing
5. Firmware decides when to enable or disable the optotriac

#### Time-Proportional Control

Instead of switching at mains frequency, the firmware uses **windowed control**:

* Fixed window: **250 ms**
* Heater ON for a percentage of that window

This approach is:

* SSR-safe
* Thermally smooth
* Simple and predictable
<p align='center'>
  <img width="1750" height="893" alt="image" src="https://github.com/user-attachments/assets/cbd84d84-6baa-4be1-81bc-1f99aab8097d" />
</p>

#### Interactive Simulation

Falstad simulaiton:

https://www.falstad.com/circuit/circuitjs.html?ctz=CQAgjCAMB0l3BWcMBMcUHYMGZIA4UA2ATmIxAUgpABZsKBTAWjDACg1DwMuwU9whKihpUxsSEhiJkVAGYBDADYBnBm1Hkm2FNy4694NgCcQxLiKpgaNEJaghsQk2d79wN8O6pPILlCJeAgG2YDwO2MRsAMZ2gWDEuiF2orTgzAIshNA0CMQ0hAgYxHwIhDR4ttKQEH6mLHyGTOWGVHR+AOYgTBVxtj3BqWJsAO7dvcnaSfhQo90tYVxTrXML4XhW3qu2aAIbKcNjLbseOzN+Y-v25gezY9Y7qYu3FyD7fAI3H3enQYbfr2e9mS9kB4W+yQBc0hiT6KzGkPcmF0UIRgWBGCSQzYKm6z2eLHczysIEUqnUY0JAgJrBR4T8ACU8eDPCxwY0xLQfNB6JyYAg5mzeOEhb8-AA3RwIFFIzF-ElCOy4BziAVgPAQZG3bDSl7gPAbaBEDXlYoVQgLOBecSSKCwAVjHUygROl5sAD2dnILTakFIjjQjiNDiS5GwHrsXB9XP9Om5KLskccbCAA

---

#### Why Zero-Cross Switching Matters for Heaters

The **MOC3063M** is a *zero-cross optotriac*, meaning it will only trigger the power triac when the AC waveform crosses **approximately 0 V**. Even if the MCU enables the optocoupler at an arbitrary time, the output remains inactive until a zero crossing occurs.

This behavior is critical for resistive heater loads:

- Switching at the voltage peak causes **large instantaneous current steps**
- These steps generate **EMI**, **audible noise**, and **electrical stress**
- Zero-cross turn-on ensures **minimum dv/dt and di/dt** at conduction start

The waveforms shown in the images demonstrate that random turn-on injects sharp current spikes, while zero-cross switching produces smooth sinusoidal current flow.

<p align='center'>
  <img width="1108" height="559" alt="image" src="https://github.com/user-attachments/assets/d253bc64-9d80-42dc-8c9e-eadb49fc119c" />
</p>
---

#### Why Phase-Angle Control Was Avoided

Phase-angle control deliberately turns the triac on mid-cycle. While useful for lamp dimming or motor speed control, it is **unsuitable for thermal systems**:

- Introduces significant EMI and harmonic distortion
- Increases thermal stress on the triac
- Requires complex timing logic synchronized to mains
- Provides no thermal benefit for a slow system like a heater

For heaters, only the **average power over time** matters—not instantaneous waveform shaping.

---

#### Zero-Cross + Time-Proportional Control

Because zero-cross optotriacs cannot modulate within a half-cycle, power is controlled using **time-proportional (windowed) control**:

- The firmware defines a fixed window (e.g. **250 ms**)
- The heater is enabled for a percentage of that window
- Each ON event starts at a zero crossing

This results in:

- Clean switching behavior
- Stable thermal output
- SSR-safe operation
- Predictable power delivery

The heater naturally integrates the power due to its thermal mass, eliminating the need for high-frequency modulation.

---

### 3.4 Thermistor Input

* **Type:** 100 kΩ NTC
* **Beta:** 3950
* **Divider:** 100 kΩ series resistor
* **ADC Reference:** 3.3 V

The firmware:

* Averages 16 ADC samples
* Applies a low‑pass filter
* Uses the Beta equation to convert resistance → temperature
<p align='center'>
  <img width="927" height="857" alt="image" src="https://github.com/user-attachments/assets/4b798e95-4f69-4ada-acea-c9a574624499" />
</p>

---

### 3.5 Fan Driver

* **Fan type:** 5 V, 2-wire DC fan
* **MOSFET:** IRLML2502 (logic-level)
* **Flyback diode:** 1N5819

Because the fan is a **2-wire fan** (no tach, no PWM pin), speed control is implemented entirely in firmware using **software PWM**.

The PWM logic controls:

* Airflow rate
* Cooling aggressiveness
* Noise level

The firmware enforces:

* A minimum duty cycle where the fan reliably spins
* A short 100% startup boost to overcome static friction
* Full-speed override during cooling or fault conditions
<p align='center'>
  <img width="1341" height="1039" alt="Screenshot 2025-12-29 182645" src="https://github.com/user-attachments/assets/e8ec08be-e6f0-481c-be43-c12c587439ca" />
</p>

---

### 3.6 OLED Display

* **Controller:** SSD1306
* **Resolution:** 128×64
* **Interface:** I²C @ 400 kHz

Pull‑ups:

* 2.2 kΩ on SDA/SCL

---

### 3.7 Buttons

* 3 momentary buttons
* Internal pull‑ups
* Active‑low

Used for:

* Start / Stop
* Navigation
* Editing values

---
## PCB Fabrication Notes

For PCB fabrication, the following stack-up has been tested and works reliably for this project:

- **Base material:** FR-4 S1000H (TG150)  
- **Copper thickness:** 1 oz  
- **Surface finish:** HASL (lead-free)  
- **Board thickness:** 1.6 mm  

If you want the **best no-compromise option**, especially for improved thermal stability, mechanical rigidity, and long-term reliability, I recommend:

- **Base material:** S1000-2M (TG170)  
- **Copper thickness:** 2 oz  
- **Surface finish:** ENIG  
- **Solder mask:** Black  
- **Board thickness:** 2.0 mm (optional but recommended)  

This configuration improves heat spreading, reduces copper resistance, and provides superior pad durability during repeated high-temperature operation.

---

## Electrical Safety Notice – Live Heatsink

> ⚠️ **IMPORTANT**
>
> The **heatsink attached to the triac is electrically live** and is directly connected to mains voltage.
>
> - Do **not** touch the heatsink while the device is powered.  
>
> Improper handling can result in **serious injury or electric shock**.

---
## 4. Firmware Architecture

### 4.1 File Structure

```
firmware/
├── main.c / main.h
├── control.c / control.h
├── heater.c / heater.h
├── fan.c / fan.h
├── thermistor.c / thermistor.h
├── ssd1306.c / ssd1306.h
├── ui.c / ui.h
├── storage.c / storage.h
```

---

### 4.2 Execution Model

* Single main loop
* No RTOS
* Deterministic timing via HAL_GetTick()
* ISR only used for fan PWM timing

**Main loop order:**

1. `Control_Update()`
2. `Heater_Update()`
3. `UI_Task()`

---

### 4.3 Control Module (`control.c`)

This module is the **central decision-making unit** of the entire system. All other subsystems react to values produced here.

#### Responsibilities

* Read raw temperature from the thermistor module
* Filter temperature readings
* Detect sensor failures
* Enforce absolute safety limits
* Execute the heating state machine
* Output a **heater power percentage** (0–100%)

No hardware is driven directly from this module. Instead, it produces *intent* values that other modules act upon.

---

### Control Timing

* Update period: **100 ms**
* Time base: `HAL_GetTick()`

Each control cycle performs the following steps **in strict order**:

1. Read raw temperature
2. Validate sensor range
3. Apply low-pass filtering
4. Check over-temperature cutoff
5. Run the heating state machine
6. Compute heater power percentage

If **any safety condition fails**, the module immediately:

* Sets heater power to 0%
* Forces the system into IDLE

---

### Why PID Was Abandoned

Early versions of the firmware used a classic PID controller with extensive tuning.

Despite significant tuning effort, the best achievable result still produced **~25 °C overshoot**.

Root causes:

* Heater power: ~400 W
* Plate mass: ~30 g
* Extremely fast thermal rise
* Significant thermal delay between heater and NTC

The thermistor simply could not observe the temperature rise quickly enough, causing the PID to react too late.

This led to the current solution: **Smart Pulse & Coast**, which is slower by design but **far more precise and repeatable**.

| State    | Meaning                 |
|----------|-------------------------|
| IDLE     | Waiting                 |
| HEATING  | Rising to setpoint      |
| HOLDING  | Maintaining temperature |
| COOLING  | Forced cooldown         |
| DONE     | Cycle finished          |


### 4.4 Smart Pulse & Coast Algorithm

This control method explicitly accounts for **thermal inertia and delay**.

Instead of continuously adjusting output, the heater is driven in **full-power pulses**, followed by observation periods.

#### Core Idea

1. Apply a controlled heat pulse
2. Turn the heater fully off
3. Observe how temperature continues to rise
4. Decide the next pulse based on real thermal response

This prevents the controller from "chasing" delayed measurements.

#### Pulse Zones

Pulse width is selected based on the ratio:

```
current_temperature / setpoint
```

| Zone | Ratio  | Pulse Width |
| ---- | ------ | ----------- |
| Far  | < 70%  | 2000 ms     |
| Mid  | 70–85% | 500 ms      |
| Near | > 85%  | 250 ms      |

#### Coast Phase Logic

During the coast phase:

* Heater is guaranteed OFF
* The firmware tracks the **peak temperature**
* A new pulse is allowed only after:

  * A minimum wait time has passed **and**
  * Temperature has begun to fall

This ensures the full thermal effect of each pulse is observed.

---

### 4.5 Heater Power, Mass, and Thermal Reality

To understand why the control strategy must be conservative, it helps to look at the **physics of the heater–plate system**.

#### Ideal Electrical Power Calculation

Even thoough the heater is listed as 400W, it was measured to have an approximate **cold resistance of 340 Ω**.

Using Ohm’s law:

```
P = V² / R
```

For a 230 V RMS mains supply:

```
P = (230 V)² / 340 Ω ≈ 155 W
```

This calculation is **electrically correct** and provides a useful order-of-magnitude estimate.

#### Why This Value Is Not Reliable in Practice

Although the formula is correct, it is **not something the control system can rely on precisely**, for several reasons:

1. **Resistance changes with temperature**
   Heating elements increase resistance as they get hotter. The 340 Ω value is a cold measurement and rises significantly during operation.

2. **Mains voltage is not constant**
   Real-world mains can vary by ±10% or more, directly affecting power.

3. **Thermal coupling dominates behavior**
   The system response is governed by how fast heat flows into the plate, not by instantaneous electrical power.

4. **Plate mass is very small**
   The aluminum plate mass is approximately **30 g**, which means:

   * Temperature rises extremely fast
   * Small energy pulses produce large temperature changes

#### Energy Perspective (Why Overshoot Happens)

The temperature rise of the plate is governed by:

```
ΔT = Q / (m · c)
```

Where:

* `Q` = heat energy (J)
* `m` = mass (kg)
* `c` = specific heat capacity (J/kg·K)

With:

* `m ≈ 0.03 kg`
* Aluminum `c ≈ 900 J/kg·K`

Even a **short, high-power pulse** injects enough energy to raise the temperature dramatically before the thermistor can respond.

This is the fundamental reason why:

* PID control overshot by ~25 °C
* Slower, observation-based control is required

The Smart Pulse & Coast approach deliberately limits **energy per pulse**, making the system predictable even when electrical power and resistance vary.


---

### 4.5 Heater Module (`heater.c`)

* Implements time‑windowed SSR control
* Enforces immediate OFF on any error

**Return behavior:**

* No return value
* Acts directly on GPIO

---

### 4.6 Fan Module (`fan.c`)

* Software PWM via TIM2
* Emergency override logic

Conditions forcing 100%:

* Cooling phase
* Sensor error
* Over‑temperature
* Temperature freeze detection

---

### 4.7 Thermistor Module (`thermistor.c`)

* ADC averaging
* Beta equation

`Thermistor_ReadTemperatureC()` returns:

* Temperature in °C
* Raw (unfiltered)

---

### 4.8 UI Module (`ui.c`)

Provides three screens:

1. **Status**

   * Current temp
   * Target
   * Heater power
   * Fan power

2. **Settings**

   * Setpoint
   * PID values (future‑proofing)
   * Max temperature
   * Hold time

3. **Graph**

   * Live temperature plot
   * Setpoint overlay

---

### 4.9 Storage Module (`storage.c`)

* Uses last flash page (0x0800FC00)
* Stores struct with magic number

Saved parameters:

* Setpoint
* PID values
* Max temperature
* Hold time

---

## 5. File Interaction Graph

To visualize module interactions, generate a graph using:

* **Graphviz**
* **Doxygen** with call graphs

Example idea:

```
control → heater
control → fan
control → ui
ui → storage
storage → control
thermistor → control
```

---

## 6. Cable Length & Wiring (As Built)

The following cable lengths reflect the **actual physical implementation** of the current build.

| Connection       | Cable Length |
| ---------------- | ------------ |
| Thermistor → PCB | ~160 cm       |
| OLED (I²C) → PCB | ~51 mm       |
| Buttons → PCB    | ~130 mm       |
| Fan → PCB        | ~120 mm       |
| AC inlet → PCB   | ~75 mm       |  
| AC inlet Earth → PCB   | ~115 mm       |  

---
## 7. Enclosure, Printing, and Mechanical Assembly



<table>
  <tr>
    <td>
      <img width="1369" height="1022" alt="Screenshot 2025-12-13 140320" src="https://github.com/user-attachments/assets/6915f4b7-c824-4573-b66a-3ee9c00d1b1b" />
    </td>
    <td>
      <img width="1277" height="969" alt="Screenshot 2025-12-13 140330" src="https://github.com/user-attachments/assets/7ae8aad4-a68a-408c-aaf2-24c7971ad4a3" />
    </td>
  </tr>
</table>

### Enclosure Material and Printing

The enclosure is **3D printed from ASA**.

ASA was chosen because:
- It has **much better heat resistance** than PLA
- It is **dimensionally stable** near warm electronics and heater-adjacent areas
- It offers **good mechanical strength** while remaining printable on consumer printers
- It is **UV- and heat-resistant**, making it suitable for long-term use

PLA was intentionally avoided because it softens at relatively low temperatures and would deform over time in this application.

**Print settings used:**
- **Layer height:** 0.2 mm
- **Material:** ASA
- **Infill / walls:** standard structural settings (not critical to functionality)

### Optional Embedded Nut Reinforcement

<table>
  <tr>
    <td>
      <img width="1132" height="940" alt="Screenshot 2025-12-13 140419" src="https://github.com/user-attachments/assets/ffdddf84-e563-4c30-be70-6b06c980a733" />
    </td>
    <td>
      <img width="1328" height="1006" alt="Screenshot 2025-12-13 140355" src="https://github.com/user-attachments/assets/7caaba84-6668-413f-9359-cb0743988dfe" />
    </td>
    <td>
      <img width="1189" height="968" alt="Screenshot 2025-12-13 143520" src="https://github.com/user-attachments/assets/74c98fb3-16dc-48e8-8b75-a15b5ff5b849" />
    </td>
  </tr>
</table>

At **layer 155** (optional), the print can be **paused** to insert **M4 nuts** directly into the enclosure.

- **Nut type:** M4 hex nut
- **Across flats:** 6.9 mm
- **Height:** 3 mm
- **Orientation:** inserted from side to side into the printed cavities

This reinforcement:
- Significantly increases mechanical rigidity
- Improves long-term durability
- Makes the enclosure feel substantially more solid

#### If You Do Not Insert Nuts

The enclosure is designed so that:
- The heater mounting holes are placed **slightly inward**
- This creates **mechanical tension** that can hold the heater assembly even **without screws**

This approach:
- **Works**, and the device can be assembled without embedded nuts
- Is **not ideal**, as the structure:
  - Feels less rigid
  - Is more prone to loosening or disassembly over time

For best results, inserting the nuts is strongly recommended.

---

### Fasteners and Hardware Used

| Application                              | Fastener Used                     |
|------------------------------------------|-----------------------------------|
| Heater mounting                          | M4 × 60 mm bolts + M4 lock nuts   |
| Fan mounting                             | M3 × 30 mm bolts + M3 lock nuts   |
| Power inlet mounting                     | M3 × 16 mm screws + M3 lock nuts  |
| Triac → heatsink connection              | M3 × 10 mm bolt                  |
| Protective earth (ground) connection     | Wire to heater bolt using 2× M4 lock nuts + 2× M4 washers|

The protective earth wire from the AC inlet is mechanically bonded to the **heater mounting bolt**, ensuring a low-impedance safety ground connection to exposed metal parts.

---

## 8. Parts and External Resources

The table below contains links to the main components used in this project.  

| Component / Resource | Description | Link |
|---------------------|-------------|------|
| Heater | AC resistive heater (400 W - 220V) | [LINK](https://www.aliexpress.com/item/1005007420346819.html?spm=a2g0o.order_list.order_list_main.74.5d041802ybo8pW) |
| Thermistor | 100 kΩ NTC, Beta ≈ 3950 | [LINK](https://www.aliexpress.com/item/1005006897238013.html?spm=a2g0o.order_list.order_list_main.79.5d041802ybo8pW) |
| OLED Display | SSD1306, 128×64, I²C | [LINK](https://www.aliexpress.com/item/1005006365875586.html?spm=a2g0o.order_list.order_list_main.186.5d041802ybo8pW) |
| Fan | 5 V, 2-wire DC fan | [LINK](https://www.aliexpress.com/item/1005006306536871.html?spm=a2g0o.order_list.order_list_main.5.5d041802ybo8pW) |
| Buttons | 5.5mm x 6mm |  |


# Interactive BOM (iBOM)

This project includes an **interactive Bill of Materials (iBOM)** generated directly from the PCB design.

The iBOM allows you to:
- Inspect all components directly on the PCB
- Click a part to see its reference, value, and footprint
- Highlight parts on the board from the BOM list
- Better understand component placement and assembly

You can access the interactive BOM here:

➡️ **[Open the Interactive BOM](https://heat-delta.vercel.app)**

---
## 9. Licensing

```
License

Documentation in this repository, including this README, is licensed under
Creative Commons Attribution–NonCommercial–ShareAlike 4.0 (CC BY-NC-SA).

Hardware design files are located in /hardware and are licensed under
the same license.

Firmware source code is located in /firmware and licensed under GNU GPL v3.
```

---




