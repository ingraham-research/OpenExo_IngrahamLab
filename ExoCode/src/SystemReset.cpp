/**
 * @file SystemReset.cpp
 * @brief The mbed fatal-error hook. Definition only - see SystemReset.h for the reasoning.
 *
 * This exists as its own translation unit because SystemReset.h is included from several .cpp
 * files, and mbed_error_hook is a real (non-inline) function: defining it in the header gives
 * "multiple definition of mbed_error_hook" at link. Nothing here is called by our code - mbed
 * calls it from mbed_error() when it is about to halt.
 *
 * Compiles to nothing on the Teensy, which has neither mbed nor GPREGRET.
 */
#include "SystemReset.h"

//Nano-side storage for the RT-I2C health counters fetched from the Teensy at boot. Defined here
//because it needs exactly ONE definition across the build and this file is already a shared TU;
//a `static` in the header would give every translation unit its own copy and the banner would read
//a different one than the fetch wrote to.
float exo_link_stats[6]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
bool  exo_link_stats_valid = false;

#if defined(EXO_HAVE_WDT)
/*
 * NO-INIT BREADCRUMB.
 *
 * WHY THIS EXISTS: GPREGRET2 does not survive a WATCHDOG reset on this part. Evidence, 2026-09-11:
 * three different builds, three different stamping schemes, and every single DOG banner came back
 * STAGE_0 and dog1 - while a BLESTALL marker in GPREGRET sailed through an SREQ untouched. The
 * stage was demonstrably 8-15 during boot and 1-6 in the loop when the dog bit, so the register was
 * cleared by the reset itself, not by our code.
 *
 * A .noinit variable lives in RAM that the C runtime startup does NOT zero, so it survives any warm
 * reset and is lost only on power loss - which is exactly the retention we need and apparently
 * cannot get from GPREGRET2.
 *
 * The magic in the high bits is what distinguishes "a real breadcrumb" from whatever random bytes
 * happen to be in that RAM after a power-on.
 */
#define EXO_NOINIT_MAGIC 0x5A5A0000ul
#define EXO_NOINIT_MASK  0xFFFF0000ul

__attribute__((section(".noinit"))) volatile uint32_t exo_noinit_stage;

/*
 * IS .noinit ACTUALLY RETAINED? B13 came back STAGEn0_g0 on a build that stamps SETUP_ENTRY at the
 * very top of setup(), which is only possible if this RAM is being zeroed at startup - i.e. the
 * mbed linker is not honouring the section and the variable is living in .bss like anything else.
 *
 * This counter settles it rather than leaving it as a theory: it increments once per boot and is
 * reported as "bN" in the banner. If every boot reports b1, the section is being zeroed and the
 * whole breadcrumb approach needs a different store. If it climbs across resets, the RAM is
 * retained and the zeros mean something else.
 */
__attribute__((section(".noinit"))) volatile uint32_t exo_noinit_boots;

uint8_t exo_noinit_boot_count()
{
    static bool counted = false;
    static uint8_t value = 0;
    if (!counted)
    {
        counted = true;
        if ((exo_noinit_stage & EXO_NOINIT_MASK) != EXO_NOINIT_MAGIC)
        {
            exo_noinit_boots = 0;   //no valid breadcrumb => treat this as a cold start
        }
        exo_noinit_boots = exo_noinit_boots + 1u;
        value = (uint8_t)(exo_noinit_boots > 255u ? 255u : exo_noinit_boots);
    }
    return value;
}

void exo_noinit_stage_set(uint8_t stage)
{
    exo_noinit_stage = EXO_NOINIT_MAGIC | (uint32_t)stage;
}

uint8_t exo_noinit_stage_get()
{
    static bool captured = false;
    static uint8_t latched = 0;
    if (!captured)
    {
        captured = true;
        const uint32_t v = exo_noinit_stage;
        latched = ((v & EXO_NOINIT_MASK) == EXO_NOINIT_MAGIC) ? (uint8_t)(v & 0xFFu) : 0u;
    }
    return latched;
}
#endif

#if defined(EXO_HAVE_CRASH_TRAP)

/**
 * @brief mbed's fatal-error hook. Runs in fault context: no printing, no allocation, no blocking.
 *
 * mbed declares this weak and calls it from mbed_error() before it halts. We record and reboot.
 */
extern "C" void mbed_error_hook(const mbed_error_ctx *error_context)
{
    uint8_t marker = (uint8_t)NRF_POWER->GPREGRET;
    uint8_t count = ((marker & EXO_CRASH_MAGIC_MASK) == EXO_CRASH_MAGIC)
                    ? (uint8_t)(marker & EXO_CRASH_COUNT_MASK) : 0u;

    if (count >= EXO_CRASH_MAX_AUTO_RESETS)
    {
        return;   //Give up and let mbed halt, rather than boot-looping on a fault that reproduces
    }

    uint8_t err_byte = 0;
    if (error_context != 0)
    {
        //Low 8 bits of the mbed error CODE. Enough to name the fault class (hard fault, assert,
        //out-of-memory, stack overflow) without needing more retained registers than we have.
        err_byte = (uint8_t)(MBED_GET_ERROR_CODE(error_context->error_status) & 0xFFu);
    }

    NRF_POWER->GPREGRET  = (uint32_t)(EXO_CRASH_MAGIC | ((count + 1u) & EXO_CRASH_COUNT_MASK));
    NRF_POWER->GPREGRET2 = (uint32_t)err_byte;

    NVIC_SystemReset();   //Includes the DSB that flushes the two writes above
}
#endif
