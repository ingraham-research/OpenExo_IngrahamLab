# How the ExoCode Files Work (Explained for High Schoolers)

This guide explains what's inside the `ExoCode/` folder — the computer program ("firmware") that
runs the exoskeleton. You don't need to know C++ to follow along. Think of the exoskeleton like a
robot with two small computer "brains" that talk to each other, a bunch of sensors (like a smart
shoe), and motors that push/pull on your joints.

---

## The Big Picture First

The exoskeleton actually has **two microcontrollers** (small computer chips) working as a team:

1. **The Teensy board — the "muscle brain."** This one reads all the sensors, does the math to
   decide how much help to give your joint, and tells the motors what to do. It has to do this
   **500 times every second**, so it needs to be fast and focused. Think of it like a reflex —
   it doesn't stop to think much, it just reacts.
2. **The Nano board — the "phone brain."** This one talks to the phone/tablet app over Bluetooth,
   passes messages back and forth to the Teensy over a wire, and reads a couple of its own
   sensors (like an altimeter that senses if you're going up or down stairs). Think of it like a
   receptionist — it takes requests from the app and passes them along.

**One file, `ExoCode.ino`, is actually shared by both boards.** It uses a trick (like a fork in
the road, `#if this board / #else that board`) so the same file compiles into different programs
depending on which chip it's flashed onto.

### How data flows, step by step

```
Phone App (Bluetooth)
      ↓
   Nano board reads the message, relays it over a wire (UART) to...
      ↓
   Teensy board updates a big shared "whiteboard" of data (ExoData)
      ↓
   Teensy reads sensors (foot pressure, joint angle, torque)
      ↓
   Teensy's "Controller" math decides: how hard should the motor push right now?
      ↓
   Teensy sends that command to the Motor
      ↓
   Motor physically pushes/pulls the joint
      ↓
   Sensor readings + status flow back the same path to update the phone app's live display
```

The most important idea to understand is **`ExoData`**: it's basically one big shared notepad
that almost every other piece of code reads from or writes to, instead of every function passing
tons of separate variables around. It's organized like a filing cabinet:

- `ExoData` (the whole cabinet) contains...
- `SideData` (one drawer per leg — left and right) contains...
- `JointData` (one folder per joint — hip, knee, ankle, etc.) contains...
- `MotorData` / `ControllerData` (pages inside the folder — raw motor numbers and control settings)

---

## 1. The Starting Point

| File | What it does |
|---|---|
| `ExoCode.ino` | The main program. Contains `setup()` (runs once at power-on) and `loop()` (runs forever after). Different code runs depending on whether it's flashed to the Teensy or the Nano. |
| `Exo.h` / `Exo.cpp` | Represents "the whole exoskeleton" on the Teensy side. Owns the left leg, the right leg, and the status lights. Its `run()` function is the heartbeat — it checks if it's time for the next 1/500th-of-a-second tick, then updates everything. |

---

## 2. The Shared Notepad (Data Structures)

These files don't *do* much on their own — they just define what information gets stored and
passed around, like the labeled boxes in that filing cabinet analogy above.

- **`ExoData`** — everything about the whole exo: both legs, the battery, emergency-stop status,
  and any errors.
- **`SideData`** — everything about one leg: which joints it has, foot-pressure readings, and an
  estimate of where you are in your walking cycle (0–100%, like a percentage through one step).
- **`JointData`** — everything about one joint: its current angle, how fast it's moving, how much
  torque (twisting force) it's feeling, and safety-check flags.
- **`MotorData`** — the raw motor numbers: position, speed, electrical current, and the torque
  command being sent to it.
- **`ControllerData`** — the "settings" for whichever control strategy is currently picked (like
  the dials on a thermostat).

---

## 3. Sensors — How the Exo "Feels" You

Think of these like the sensors in a smartwatch or a smart shoe.

- **`FSR.h/.cpp`** — Force-Sensitive Resistors are pressure sensors under your heel and toe, kind
  of like a tiny bathroom scale in your shoe. They detect when your foot touches the ground, which
  tells the exo where you are in your step.
- **`AnkleAngles.h/.cpp`** — A magnetic sensor at the ankle that works like a built-in protractor,
  measuring how bent or straight your ankle is.
- **`AnkleIMU.h/.cpp`** and **`ThIMU.h/.cpp`** — IMU stands for "Inertial Measurement Unit" — the
  same kind of chip that lets your phone know which way is up. One sits at the ankle, one at the
  thigh, and together they help figure out your leg's orientation (useful for detecting inclines).
- **`TorqueSensor.h/.cpp`** — Measures the actual twisting force the exoskeleton is applying to
  your joint, so the software can double check the motor is doing what it was told to do.
- **`WaistBarometer.h/.cpp`** — A barometer measures air pressure. Air pressure changes very
  slightly with elevation, so this sensor can tell if you're walking up or down stairs, similar to
  how a weather app or altimeter works.
- **`InclinationDetector.h/.cpp`** and **`InclineDetector.h/.cpp`** — Two different methods (one
  using ankle angle, one using the barometer) for figuring out if you're on flat ground, going
  uphill, or going downhill.
- **`Battery.h`** — Currently unused/placeholder code for reading how much charge is left in the
  battery.

---

## 4. Motors and Control — How the Exo "Decides" and "Acts"

- **`Joint.h/.cpp`** — Think of a `Joint` as a mini-supervisor for one joint (say, the left
  ankle). Every 1/500th of a second, it reads that joint's sensors, asks the `Controller` "how
  much help should I give right now?", and passes that answer to the `Motor`.
- **`Controller.h/.cpp`** — This is the "brain" behind the assistance strategy. Different
  controllers are like different coaching styles:
  - `ZeroTorque` — do nothing, a safe default.
  - `ProportionalJointMoment` — push harder the harder you push your toe into the ground.
  - `ZhangCollins` / `FranksCollinsHip` — pre-programmed torque patterns based on published
    research studies on how walking assistance should be timed.
  - `Spline` — lets researchers draw a smooth custom curve through 5 points to shape assistance.
  - `TREC` / `SPV2` / `PJMC_PLUS` — newer, smarter controllers that adapt to changing terrain.
  - `Chirp` / `Step` — not for real walking; these send test signals used to measure how the
    hardware responds, kind of like tapping a microphone to test it.
- **`Motor.h/.cpp`** — The code that actually talks to the physical motor. There's a common
  "template" (`_Motor`) that every motor type follows, plus specific versions for different motor
  brands/models (e.g., Maxon motors, or CubeMars "AK-series" motors that communicate over a CAN
  bus — the same kind of network cars use internally).
- **`Side.h/.cpp`** — Represents one whole leg: owns all its joints and its two foot pressure
  sensors, and figures out percentage-through-your-step and calibration.
- **`ListCtrlParams.h/.cpp`** — Reads the list of all available controllers and their adjustable
  settings from the SD card and packages it up so the phone app knows what options to show in its
  menus.

---

## 5. Communication — How the Phone App Talks to the Exo

- **`ComsMCU.h/.cpp`** — The Nano's version of `Exo.cpp` — it's the "traffic controller" that
  loops through checking Bluetooth, reading local sensors, and relaying messages to the Teensy.
- **`ExoBLE.h/.cpp`** — Handles the actual Bluetooth radio: advertising the exo's name so your
  phone can find it, connecting, and sending/receiving data.
- **`BleMessage.h/.cpp`**, **`BleMessageQueue.h/.cpp`**, **`BleParser.h/.cpp`** — Together these
  turn raw Bluetooth data (just a stream of 1s and 0s) into organized "messages" the code can
  understand, and hold them in a line (a queue) so none get lost if several arrive at once.
- **`ble_commands.h`** — A dictionary of every command the phone app can send (like `'E'` = start
  a trial, `'G'` = stop) and what the exo should do in response.
- **`GattDb.h`** — Defines the official Bluetooth "channels" (like TV channels) that the phone can
  tune into for different types of data.
- **`UARTHandler.h/.cpp`**, **`uart_commands.h`**, **`UART_msg_t.h`** — Handle the *wired*
  connection between the Teensy and the Nano (UART is just a simple two-wire way for two chips to
  talk), including packaging messages so both sides know where one message ends and the next
  begins.
- **`ParamUpdateValidation.h`** — A safety check: before applying a setting change requested by
  the app, this makes sure the new value actually makes sense and isn't dangerous.
- **`GetBulkChar.h/.cpp`** / **`SendBulkChar.h/.cpp`** — A pair of helpers for sending one big
  chunk of text (like the full list of controllers) across the wire in one go instead of in tiny
  pieces.

---

## 6. Configuration — How the Exo Knows Its Own Setup

- **`IniFile.h/.cpp`** — A general-purpose tool for reading settings files (`.ini` files) off an
  SD card — basically a way to read a simple text file full of `setting = value` lines.
  `ParseIni.h/.cpp` — Maps the *specific* settings in the exo's `config.ini` (which joints are
  installed, which motors, which controller to start with) into codes the rest of the program
  understands. Think of this as the exo reading its own instruction manual on startup.
- **`ParamsFromSD.h/.cpp`** — Loads the specific number values (like "peak torque = 20") for
  whichever controller is selected, reading them from a spreadsheet-like file (CSV) on the SD
  card.
- **`Config.h`** — Central settings file: which board version this is, how often the main loop
  runs (500 times a second), and various safety thresholds.
- **`Board.h`** — A "wiring diagram written in code" — maps friendly names (like "left heel
  sensor pin") to the actual physical pin numbers on the chip, since different exo versions wire
  things differently.

---

## 7. Status Lights — How the Exo "Talks" Without Words

- **`StatusLed.h/.cpp`** — An RGB (multi-color) light that shows the exo's mood: for example,
  green pulsing might mean "trial running," red blinking might mean "error." Like a mood ring.
- **`SyncLed.h/.cpp`** — A light that blinks in a very precise pattern so that cameras in a
  research lab can line up their video recordings with the exo's data — like a clapperboard in a
  movie shoot.
- **`ComsLed.h/.cpp`** — A light on the Nano board specifically showing Bluetooth/connection
  status.
- **`StatusDefs.h/.cpp`** — The master list of every possible status and error code, shared by all
  the lights and the phone app so everyone's speaking the same language.

---

## 8. Errors and Logging — How the Exo Catches Problems

- **`error_codes.h`** — A list of every possible thing that could go wrong (torque too high,
  motor not responding, etc.), each with its own ID number.
- **`error_types.h/.cpp`** — For each error, defines how to *check* if it's happening and what to
  *do* about it (usually: log it, and sometimes shut off the motor for safety).
- **`error_map.h`** — Connects each error ID to its checking/handling code.
- **`ErrorManager.h/.cpp`** — Runs all those error checks every single loop, for every joint, like
  a safety inspector doing rounds 500 times a second.
- **`ErrorReporter.h`** — Sends any triggered error from the Teensy over to the Nano, which
  forwards it to the phone app so a human can see it.
- **`Logger.h`** / **`LogLevels.h`** — A tool for printing debug messages, with a dial to control
  how much detail gets printed (useful when troubleshooting vs. running for real).
- **`PiLogger.h`** — Prints a specific, neatly formatted stream of data meant to be read by an
  external computer (like a Raspberry Pi) doing extra data logging.

---

## 9. Everyday Helper Tools

- **`Utilities.h/.cpp`** — A toolbox of small, reusable math helpers: converting between degrees
  and radians, smoothing out noisy sensor data, packing numbers into the right format to send over
  a wire, and more.
- **`Time_Helper.h/.cpp`** — Keeps track of how much time has passed, which is how the code
  enforces "run this 500 times per second" timing.
- **`I2CHandler.h`** / **`RealTimeI2C.h/.cpp`** — Handle a different communication protocol (I2C)
  used to talk to some of the sensors.
- **`CAN.h`** — Handles the CAN bus protocol used to talk to certain motors (the same protocol
  used inside cars).
- **`SystemReset.h`** — A simple "reboot the whole computer" button, built into software.
- **`PlottingTitles.h/.cpp`** — Generates readable column labels (like "Desired Torque (Left)")
  for graphs in the companion app, so the data makes sense to a human looking at it.

---

## 10. `systemCheck/` — Practice Sketches for Testing Hardware by Itself

These are small, separate programs (not part of the main exo program) used to test **one piece of
hardware at a time** — kind of like testing each ingredient before baking the whole cake. Useful
when building or troubleshooting a new exo.

| Sketch | What it tests |
|---|---|
| `Fsr/Fsr.ino` | Prints raw foot-pressure sensor readings, to check the sensors are wired right. |
| `Motor/Motor.ino` | Basic "can the motor spin at all?" test. |
| `MotorEnable/MotorEnable.ino` | Tests the on/off switch logic for the motor. |
| `MotorValidation/MotorValidation.ino` | A more careful test comparing the motor's actual torque against a reference measurement, to calibrate it. |
| `SPI/SPI.ino` | Tests a different communication protocol (SPI), likely for older hardware. |
| `StatusLed/StatusLed.ino` | Cycles the status light through all its colors/patterns to confirm it works. |
| `SyncLed/SyncLed.ino` | Tests the camera-sync blinking light. |
| `ThighIMU/ThighIMU.ino` | Confirms the thigh motion sensor is working and streaming data. |
| `TorqueSensor/TorqueSensor.ino` | Prints raw torque readings from both legs, to check wiring/calibration. |
| `UART/UART.ino` | Tests the wired connection between the Teensy and Nano by itself. |

---

## TL;DR

- Two chips (Teensy = fast reflexes, Nano = phone communicator) split the work.
- Sensors (`FSR`, `AnkleAngles`, IMUs, `TorqueSensor`, barometer) feed measurements into a shared
  data "filing cabinet" (`ExoData` → `SideData` → `JointData`).
- A `Controller` decides how much help to give based on that data.
- A `Motor` file sends that command to the real motor.
- Bluetooth/UART files relay everything to and from the phone app.
- LEDs show status, error files catch problems, and `systemCheck/` sketches test hardware pieces
  one at a time.
