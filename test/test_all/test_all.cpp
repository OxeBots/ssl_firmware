// Merged test binary dispatcher. Every non-interactive suite is linked into
// this one firmware so a single flash + run covers everything (and a single
// gcov dump collects all coverage counters). Interactive suites keep their
// own mains.
//
// The per-suite wrapper translation units and the suite table are generated
// by scripts/generate_merged_tests.py from the test_logic and test_hardware
// filters. Suite order and bus ownership constraints are documented there.

#include <esp_app_trace.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <unity.h>

#include "merged_suites.h"

static void (*s_setup)(void) = nullptr;
static void (*s_teardown)(void) = nullptr;

void test_all_set_active_hooks(void (*setup)(void), void (*teardown)(void))
{
    s_setup = setup;
    s_teardown = teardown;
}

// The unity library's UnityDefaultTestRun() always calls the global
// setUp/tearDown; forward them to the active suite's own pair (nullptr when
// the suite defines none).
void setUp(void)
{
    if (s_setup != nullptr)
        s_setup();
}

void tearDown(void)
{
    if (s_teardown != nullptr)
        s_teardown();
}

extern "C" void app_main(void)
{
    int total_failures = 0;

    for (int i = 0; i < MERGED_SUITE_COUNT; ++i)
    {
        printf(
          "\n===== [SUITE %d/%d] %s =====\n", i + 1, MERGED_SUITE_COUNT, MERGED_SUITES[i].name);
        test_all_set_active_hooks(MERGED_SUITES[i].setup, MERGED_SUITES[i].teardown);
        MERGED_SUITES[i].run();
        // Each suite's UNITY_BEGIN() resets the counters, so read the failure
        // count while it still belongs to the suite that just ran.
        total_failures += (int)Unity.TestFailures;
    }

    printf("\nALL SUITES DONE: %d suites, %d total failures\n", MERGED_SUITE_COUNT, total_failures);
    fflush(stdout);

#if defined(CONFIG_APPTRACE_GCOV_ENABLE)
    // Merged run coverage: every suite's counters are collected in one dump.
    esp_gcov_dump();
#endif
}
