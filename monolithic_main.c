
#define BUILD_MONOLITHIC 1
#include "monolithic_examples.h"

#define USAGE_NAME   "fmtlog_tests"

#include "monolithic_main_internal_defs.h"

MONOLITHIC_CMD_TABLE_START()

    { "bench", {.f = fmtlog_bench_main } },
	{ "enc_dec_test", {.f = fmtlog_enc_dec_test_main } },
	{ "link_test", {.f = fmtlog_link_test_main } },
	{ "log_test", {.f = fmtlog_log_test_main } },
	{ "multithread_test", {.f = fmtlog_multithread_test_main } },

MONOLITHIC_CMD_TABLE_END();

#include "monolithic_main_tpl.h"
