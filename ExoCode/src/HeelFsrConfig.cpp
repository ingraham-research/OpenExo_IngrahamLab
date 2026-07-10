#include "HeelFsrConfig.h"

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include "IniFile.h"
#include "ParseIni.h"   // ini_config::buffer_length

bool heel_fsr_present()
{
    static int cached = -1;   // -1 = not yet read

    if (cached < 0)
    {
        cached = 0;   // default: no heel FSR installed

        IniFile ini("/config.ini");
        if (ini.open())
        {
            // Buffer must hold the longest line in config.ini while scanning (comments are long),
            // so match the main parser's size rather than the short key/value length.
            char buf[ini_config::buffer_length];
            int v;
            if (ini.getValue("Sensors", "heelFsrPresent", buf, sizeof(buf), v))
            {
                cached = (v != 0);
            }
            ini.close();
        }
    }

    return cached != 0;
}

#else   // Nano / other MCU: no SD card, no heel FSR handling

bool heel_fsr_present() { return false; }

#endif
