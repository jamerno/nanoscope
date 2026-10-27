# Nano Scope

A slow multi-channel oscilloscope built from an **Arduino Nano** and a **laptop GUI**.
Up to 6 channels, exact time/div, pre-trigger, single shot, roll mode, cursors, automatic
measurements (frequency, duty cycle, Vpp, rise/fall time, ...) and CSV export.

![Nano Scope GUI](docs/screenshot.png)

*Screenshot taken in `--demo` mode with simulated signals (no hardware attached).*

> ## 🤖 Built entirely by Claude
>
> **This whole project was written by [Claude](https://claude.ai) (an AI assistant made by Anthropic).**
> The firmware, the GUI, the serial protocol, the tests and this README were all produced by Claude
> in a chat conversation. The human described what they wanted, built the hardware and tested it, but wrote
> no code. If you reuse it, treat it like any other code you did not write yourself: read it first.

## Status

| Part | How it was checked |
|---|---|
| **Real hardware** | An earlier version (before roll mode) was built and tested on a real Arduino Nano with **4.7 kΩ series resistors** on the inputs and **worked great**. Version 3 (roll mode, automatic recovery, saved settings) has so far only been tested in simulation. |
| Firmware | Compiles without warnings (`-Wall -Wextra`) with avr-gcc 7.3 against the official Arduino AVR core: 4458 bytes flash, 1250 of 2048 bytes RAM. |
| Firmware logic | The real firmware code was compiled on a PC with a simulated ADC, timer and serial port, and checked in 19 scenarios: rising/falling trigger, pre-trigger ring buffer order, auto / normal / single, 6 channels, roll mode, aborting on new commands. |
| GUI | 27 automated checks run headless against the built-in simulated Nano: roll mode, debounced commands, stale frames, resync, settings, Ctrl+C. |
| Measurements | Checked against synthetic signals with known answers (e.g. 100 Hz / 50 % came out as 99.9997 Hz / 50.0 %). |

The sample timing constants are estimates. If the Nano cannot keep up, it reports an overrun and the
GUI lowers its speed limit automatically. Findings from real hardware are welcome as issues.

## What it can and cannot do

The Nano has a single 10-bit ADC that is switched between pins. That sets the limits:

| Channels on | Max sample rate per channel (estimate) | Sensible signals up to |
|---|---|---|
| 1 | ~40 kS/s | ~4 kHz |
| 2 | ~24 kS/s | ~2 kHz |
| 4 | ~13 kS/s | ~1 kHz |
| 6 | ~9 kS/s | ~900 Hz |

* **Good for:** slow clocks and control lines (e.g. a breadboard CPU running at 1 Hz to a few kHz),
  sensors, slow PWM, audio-range signals, low-frequency power-supply ripple, checking whether a
  signal is square, sine or triangle.
* **Not for:** anything fast (MHz buses, SPI, fast edges, glitch hunting). Microsecond glitches fall
  between samples. For that use a logic analyzer (a ~10 EUR USB one or a Raspberry Pi Pico).
* **Resolution:** 8 bit (about 20 mV steps), 0 to 5 V at the pin.
* **Aliasing:** the program cannot know the true frequency of a signal faster than the sample rate.
  Frequency readings with fewer than 10 samples per cycle are marked with `~`, below 4 they are not shown.
* The channels are sampled one after another, not simultaneously (about 17 us apart).

## Features

* 1 to 6 channels (A0 to A5), each switched on or off with a button
* **Exact time/div:** type any value (`3.3ms`, `250us`, `1s`), step with the arrows, or use the mouse wheel. From 100 us/div to 10 s/div
* **Roll mode** for slow signals: continuous scrolling display, starts automatically from 200 ms/div in Auto mode
* Volts/div and center, plus an Auto-fit button
* Trigger: source, rising/falling, draggable level line, "50 %" button, position in the window (pre-trigger), Auto / Normal / Single
* **Voltage multiplier and offset per channel**, so a voltage divider or a biased input shows the real voltage (see below)
* Measurements per channel: Vmax, Vmin, Vpp, Vavg, Vrms, frequency, period, duty cycle, rise time, fall time
* Two time cursors and two voltage cursors with ΔT, 1/ΔT, ΔV
* Save CSV and PNG screenshot
* Remembers its settings between runs
* Recovers by itself if a command to the Nano gets lost
* `--demo` mode with a simulated Nano, so you can try the GUI before building anything

## Hardware

* Arduino Nano (ATmega328P, 16 MHz), USB cable
* Per input: a **series resistor** and a clamp to protect the pin (5.1 V zener to GND, or two Schottky
  diodes to GND and 5 V). **4.7 kΩ was used in the hardware test and works well.** 1 kΩ also works.
  Keep the total source resistance (series resistor plus any divider) below about 10 kΩ, otherwise
  the ADC cannot settle and channels start to influence each other
* Common ground between the Nano and the circuit under test

| Nano pin | Channel |
|---|---|
| A0 to A5 | CH1 to CH6 |

### Unused inputs

**Switch off every channel you have not connected, or tie its pin to GND.** All pins share one
sample-and-hold capacitor in the ADC. A pin with nothing connected cannot recharge it, so it still holds
the voltage of the previously measured channel and shows a copy of that signal. If two channels show the
same trace while only one is wired up, this is why.

### Measuring higher voltages (voltage multiplier)

The pin itself only takes 0 to 5 V. To measure more, use a resistive divider and tell the software its
ratio, so the screen shows the **real** voltage (trace, measurements, cursors and trigger level):

```
signal ──[ R1 ]──┬──[ 4.7k ]──► Nano pin (A0..A5)      (plus the clamp at the pin)
                 │
               [ R2 ]
                 │
                GND
```

* Divider ratio = (R1 + R2) / R2. Enter it in the **Voltage ×** box of that channel.
* Example: R1 = 10 kΩ, R2 = 3.3 kΩ gives a ratio of about 4.03. Enter `4.03`. 12 V then reaches the pin
  as 2.98 V and is displayed as 12 V. That divider is fine up to about 16 V (20 V would already hit 5 V at the pin).
* Resistor tolerance matters. Compare against a multimeter and adjust **Voltage ×** until they agree.
* The divider's output resistance (R1 ∥ R2, about 2.5 kΩ in the example) adds to the series resistor, so keep both together under ~10 kΩ.
* The resolution gets coarser by the same factor (about 80 mV steps at ×4).
* **Offset** is for signals biased to a DC level at the pin, for example a function generator
  signal around 0 V that you shifted to 2.5 V. The display shows (pin voltage − Offset) × Voltage ×.
* Inputs are **0 to 5 V at the pin only**. Negative voltages need a bias circuit.

**Safety**

* A divider does not make a high-voltage circuit safe to touch or measure. Only use it on low-voltage circuits you understand.
* **Never connect this to mains.** Use a proper isolated, rated instrument for that.

## Installation

### 1. Flash the Nano

Needs [arduino-cli](https://arduino.github.io/arduino-cli/) or the Arduino IDE. No extra libraries.

```bash
arduino-cli core install arduino:avr
arduino-cli compile -b arduino:avr:nano nano_scope
arduino-cli upload  -b arduino:avr:nano -p /dev/ttyUSB0 nano_scope
```

Many clone Nanos have the old bootloader. If the upload fails, use `-b arduino:avr:nano:cpu=atmega328old`.

### 2. Run the GUI

Arch Linux:

```bash
sudo pacman -S python-pyqt6 python-pyqtgraph python-pyserial python-numpy
sudo usermod -aG uucp $USER      # serial port access, log out and in again
python scope_gui.py
```

Other systems: `pip install PyQt6 pyqtgraph pyserial numpy`

```bash
python scope_gui.py                        # pick the port in the window
python scope_gui.py --port /dev/ttyUSB0    # connect directly
python scope_gui.py --demo                 # simulated Nano, no hardware
```

The default baud rate is **500000**. If your USB-serial chip has trouble with that, change `BAUD`
in `nano_scope.ino` and select the same rate in the GUI (115200 works, but frames update more slowly).
The Nano resets when the port is opened, so the first frame appears after about two seconds.

Settings are stored in `~/.config/nano-scope/`. Delete that folder to reset them.

## Using it

| Control | What it does |
|---|---|
| Mouse wheel on the plot | Time/div (zoom the time axis in and out) |
| Mouse wheel on the voltage axis | Volts/div |
| Drag the dashed horizontal line | Trigger level |
| `Space` | Run / Stop |
| `S` | Single shot |
| `1` to `6` | Toggle channels |
| Arrow keys | Left/Right time/div, Up/Down volts/div |
| `Ctrl+C` in the terminal | Closes the window cleanly |

**Modes**

* **Auto:** triggers when it can, shows a free-running capture when there is no trigger, and switches to roll mode from 200 ms/div.
* **Normal:** only shows triggered captures.
* **Roll:** continuous scrolling display. The Nano sends every sample immediately, at most one every 2 ms, so the shortest roll time/div is about 6.4 ms.
* **Single:** one triggered capture, then it stops.

Tips for a slow clock around 100 Hz: switch on the clock channel, set about 2 to 5 ms/div, trigger on
the same channel. For a 1 Hz clock use roll mode at 200 ms/div or slower.

## Troubleshooting

| Problem | Fix |
|---|---|
| Two channels show the same trace | One of them is not connected. Switch it off or tie it to GND (see above) |
| "NO DATA" | Wrong port or baud rate, sketch not flashed, or the Nano is still resetting after connecting |
| Upload fails | Try `arduino:avr:nano:cpu=atmega328old` |
| Permission denied on `/dev/ttyUSB0` | `sudo usermod -aG uucp $USER` and log in again |
| Window does not start on Wayland | Install `qt6-wayland` or run with `QT_QPA_PLATFORM=xcb` |
| Time/div jumps up when enabling channels | More channels sample slower, so the shortest time/div is limited |

## How it works

* The Nano samples continuously into a ring buffer on a hardware timer tick (Timer1, 0.5 to 64 us
  resolution), so the sample spacing is exact. All enabled channels share one 1020 byte buffer, so fewer
  channels means more samples each.
* It watches the trigger channel (8-bit level, about 60 mV hysteresis). After the trigger it keeps
  sampling until the pre-trigger history plus the post-trigger part fill the window, then sends one frame.
* While waiting for the next sample it also listens for commands, so Stop and setting changes act at once, even at 10 s/div.
* Auto mode sends what it has if no trigger arrives within one window. Normal waits. Single sends once and stops.
* The GUI turns time/div into a sample interval and sample count, sends it to the Nano, and does all
  scaling and measurements on the laptop. Quick successive changes (dragging a slider) are merged into one command.
* Frames carry the settings they were captured with. If they do not match what the GUI asked for, it sends the settings again.

### Serial protocol

All numbers little endian.

Laptop to Nano (14 bytes):

```
A5 | mask | prescaler idx | ticks (2) | n (2) | pre (2) | trig ch | edge | level | mode | xor
```

`mode`: 0 stop, 1 auto, 2 normal, 3 single, 4 roll. `xor` is the XOR of bytes 1 to 12.

Nano to laptop, one frame per capture:

```
AA 55 | mask | flags | prescaler idx | ticks (2) | Vcc mV (2) | n (2) | trig idx (2) | data[n][nch] | xor
```

`data` is sample major (all channels of sample 0, then sample 1, ...), oldest sample first.
Flags: 1 triggered, 2 overrun, 4 forced (no trigger), 8 single shot.

Nano to laptop in roll mode, one packet per sample set:

```
AB CD | mask | prescaler idx | ticks (2) | Vcc mV (2) | data[nch] | xor
```

The sample interval is `ticks × {0.5, 4, 16, 64} µs`. In frames and roll packets `xor` covers all bytes after the 2-byte sync word.

## Project layout

```
nano_scope/nano_scope.ino   firmware for the Arduino Nano
scope_gui.py                laptop GUI (PyQt6 + pyqtgraph), includes the --demo simulator
docs/screenshot.png         screenshot (demo mode)
```

## Known limitations / ideas

* Roll mode and the other version 3 additions are not yet verified on real hardware (see Status)
* No FFT, no math channels, no logic-analyzer view
* 8-bit samples only; the ADC runs at a 1 MHz clock, above the 200 kHz the datasheet recommends for full accuracy
* Frequency accuracy is limited to about one sample over the measured span

Issues and pull requests are welcome, especially measurements from real hardware.

## License

MIT
