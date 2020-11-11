#ifndef BM64_DRIVER_H
#define BM64_DRIVER_H

#define BM64_NOERROR 0
#define BM64_DATA_TOOSHORT 1

int bm64_init();
void bm64_test();

void bm64_accept_call();
void bm64_reject_call();
void bm64_end_call();

void __attribute__((weak)) bm64_on_incomming_call();

#define CALL_STATUS_INCOMMING_CALL 0x02

#endif
