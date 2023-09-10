#include "lib.h"
#include "../fmtlog.h"

#include "monolithic_examples.h"


#if defined(BUILD_MONOLITHIC)
#define main     fmtlog_link_test_main
#endif

int main() {
  logi("link test: {}", 123);
  libFun(321);
  return 0;
}
