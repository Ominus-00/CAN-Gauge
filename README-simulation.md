# Six-parameter ECU and ESP32-S3 gauge

The six controls are IAT, boost, coolant temperature, oil temperature, EGT, and oil pressure. RPM has been replaced by oil pressure as requested.

## Controls and test ranges

| Control | Style | Range | Test CAN ID | Payload unit |
|---|---|---|---|---|
| IAT | Knob | -40 to 120 C | 0x123 | 1 C |
| Boost | Knob | -1.00 to +3.00 bar gauge | 0x124 | 0.01 bar |
| Coolant temperature | Slider | -40 to 140 C | 0x125 | 1 C |
| Oil temperature | Slider | -40 to 180 C | 0x126 | 1 C |
| EGT | Slider | 0 to 1100 C | 0x127 | 1 C |
| Oil pressure | Knob | 0 to 10.00 bar | 0x128 | 0.01 bar |

These are configurable simulation ranges, not engine safety limits. Initial values are approximately 30 C, 0 bar boost, 90 C coolant, 100 C oil, 450 C EGT, and 4 bar oil pressure. The 0..1023 potentiometer positions introduce small rounding differences.

Click a control, then drag it or use Left/Right for fine changes, Page Up/Page Down for larger changes, and Home/End for endpoints. See [Wokwi potentiometer controls](https://docs.wokwi.com/parts/wokwi-potentiometer) and [slide potentiometers](https://docs.wokwi.com/parts/wokwi-slide-potentiometer).

## Signal path

Each potentiometer SIG connects to its own fake ECU input: IAT, BOOST, COOLANT, OIL_TEMP, EGT, or OIL_P. The ECU samples one input per frame and cycles through all six. Each classical CAN frame uses an 11-bit ID and two data bytes: a signed 16-bit two's-complement integer, high byte first. Pressures are scaled by 100. The gauge divides pressures by 100 for display.

Potentiometers → ECU → CANH/CANL → TCAN3413 RXD → GPIO 2 → CRC validation → OLED. GPIO 1 returns the CAN acknowledgment via TCAN3413 TXD. No potentiometer connects directly to the ESP32.

The OLED shows all six readings and RX/error counters. A reading becomes `---` if that channel has not updated for three seconds. A missing channel therefore cannot leave an old value appearing live indefinitely.

## Simulation and hardware modes

The default environment uses the tested **1 kbit/s software CAN bench mode**. It is a workaround for the native TWAI behavior observed in Wokwi; the simulator documents TWAI support as partial: https://docs.wokwi.com/guides/esp32 . A complete six-value scan takes approximately 1.1 seconds.

The separate PlatformIO `hardware` environment sets `WOKWI_SOFT_CAN=0` and compiles the normal ESP32-S3 TWAI controller at 500 kbit/s. It is compile-checked, not physically tested. These synthetic IDs are not verified BMW messages and this ECU does not answer OBD-II requests. The software bench receiver is not suitable for a vehicle bus.

The chip models use ideal digital CANH/CANL signaling and omit analog voltages, supply supervision, real TXD timeouts, and other protections. The software receiver handles stuffing, CRC, acknowledgment and fixed frame fields on this two-node bench, without full CAN arbitration/resynchronization/error confinement.

## Files and running

- `src/main.cpp`: PlatformIO firmware. `sketch.ino`: the same firmware for Wokwi Web.
- `ecu.chip.c` / `.json`: six-input fake ECU model and pin list.
- `tca3413.chip.c` / `.json`: existing transceiver model. The filename/type spelling remains consistent with your project.
- Both `.chip.wasm` files: compiled models required by VS Code; `wokwi.toml` registers them.
- `diagram.json`: complete wiring and six labeled controls.
- `platformio.ini`: default simulator build plus the separate `hardware` environment.
- `wokwi-api.h`: custom-chip API declarations for the C compiler/editor.

Keep custom-chip files in the project root, outside `src`. In VS Code, build with PlatformIO and run **Wokwi: Start Simulator**. After changing `diagram.json`, close and reopen the simulator tab if it retains the old layout. In Wokwi Web, use `sketch.ino`, the diagram, both chip C/JSON pairs, and `libraries.txt`.

The local project has been updated at `C:\Users\opate\OneDrive\VS Code Projects\CAN Gauge`. The web editor is updated but remains unsaved to your Wokwi account because saving would publish the code publicly. The downloadable bundle preserves all source files.

## Rebuilding the custom chips

Run from the project folder using the official Wokwi CLI:

```powershell
wokwi-cli chip compile ecu.chip.c -o ecu.chip.wasm
wokwi-cli chip compile tca3413.chip.c -o tca3413.chip.wasm
```

The CLI used on this computer is at `C:\Users\opate\Documents\Codex\2026-09-25\referenced-chatgpt-conversation-this-is-an\work\wokwi-cli.exe`. If it is not on PATH, use that full path with PowerShell's `&` operator. Building ESP32 firmware alone does not rebuild custom-chip C files. Web Wokwi compiles the custom-chip C sources itself.

## Validation

Both PlatformIO modes and the ECU WASM compile. The included independent test executes the compiled WASM with simulated pin/timer calls and decodes its emitted bits. It checks 30 six-channel input combinations (180 frames), signed values, independent channels, CAN IDs, stuffing, CRC by polynomial division, frame tails, ACK, disabled EN and busy-bus gating.

```text
node test-ecu.cjs ecu.chip.wasm
```

The previously working one-RPM configuration is preserved separately in `outputs/can-gauge-fixed.zip` in the Codex workspace.

Live verification: the expanded circuit was run in both VS Code and Wokwi Web. All six values were decoded with CRC=OK and errors=0. Web chip logs confirmed ACK=yes for every sensor. Oil pressure was independently moved from 4.00 to 10.00 bar in both simulators; web IAT -40 C and EGT 1100 C were also observed without changing the other inputs. Both simulations remain open. Live knobs may still show these deliberate test endpoints; the saved diagram contains the initial values listed above.
