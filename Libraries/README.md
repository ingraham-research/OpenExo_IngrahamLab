# Libraries Added

## SPISlave_T4
- Git: https://github.com/tonton81/SPISlave_T4
- Commit: 28d8c1fd6082335d597483d45121d9db4c9cfc5c
- Modifications: 
    - SPISlave_T4.tpp 
        - line 18 changed to 

            ```static void lpspi4_slave_isr() {``` 

            to avoid multiple definition error.
## TSPISlave
- Git: https://github.com/tonton81/TSPISlave
- Commit: 492100f7f41a6d9d3888ed68b1e326a58f11f47a
- Modifications: None

## IniFile
- Git: https://github.com/stevemarple/IniFile
- Commit: 880edeac620f262b804fce500bfbbb1227c5cba9
- Modifications: None

## Adafruit_BluefruitLE_nRF51
- Git: https://github.com/adafruit/Adafruit_BluefruitLE_nRF51
- Commit: 16412c28c5d25eb2577c0d5bbac85f3c7dc7baae
- Modifications: 
    - Adafruit_BluefruitLE_SPI.cpp 
        - line 45 changed to:  
        
            ```SPISettings bluefruitSPI(1000000, MSBFIRST, SPI_MODE0);```  
            
            Due to teensy speed issue

## ArduinoBLE
- Git: https://github.com/arduino-libraries/ArduinoBLE
- Version: 2.1.0 (this folder previously held an unpatched 1.2.1; replaced 2026-09-15)
- Modifications: **YES, and the Nano firmware will not link without them. Do not replace this folder with the Library Manager version.**
    - `src/utility/HCI.cpp` - three changes, each marked `LOCAL MODIFICATION` in the file with its own explanation and revert steps. The untouched original is kept beside it as `HCI.cpp.orig-openexo-backup`.
        1. `HCIClass::sendAclPkt()` now waits at most `ARDUINOBLE_ACL_WAIT_TIMEOUT_MS` (50 ms) for the Bluetooth controller to free a packet slot, and drops that packet instead of waiting forever. Upstream bug [ArduinoBLE issue #45](https://github.com/arduino-libraries/ArduinoBLE/issues/45), open since 2019 - the unbounded wait hangs the Nano when the BLE link stalls. **This one changes behaviour.**
        2. LE Connection Complete - records the connection parameters the central chose into `exo_ble_cp_*`. Record only.
        3. LE Connection Update Complete (HCI LE meta subevent 0x03, which upstream does not handle at all) - records the updated parameters into `exo_ble_cu_*`. Record only.
    - Changes 2 and 3 define the symbols declared `extern` in `ExoCode/src/SystemReset.h` and reported in the connect banner as `CPi`/`UPi`. Without them the Nano build fails with ``undefined reference to `exo_ble_cp_interval'``. The full explanation lives in the build note above that `extern "C"` block, and in `Modification log with claude/Nano-Hang-Watchdog-And-Breadcrumbs.md` (§8, §13, §14.2, §15.2-§15.3).
    - To check that a sketchbook install still has the patch: `grep -c ARDUINOBLE_ACL_WAIT_TIMEOUT_MS <sketchbook>/libraries/ArduinoBLE/src/utility/HCI.cpp` - 0 means it is gone, and it can be restored from this folder.