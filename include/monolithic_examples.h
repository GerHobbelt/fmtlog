
#pragma once

#if defined(BUILD_MONOLITHIC)

#ifdef __cplusplus
extern "C" {
#endif

	int fmtlog_bench_main(void);
	int fmtlog_enc_dec_test_main(void);
	int fmtlog_link_test_main(void);
	int fmtlog_log_test_main(void);
	int fmtlog_multithread_test_main(void);

#ifdef __cplusplus
}
#endif

#endif
