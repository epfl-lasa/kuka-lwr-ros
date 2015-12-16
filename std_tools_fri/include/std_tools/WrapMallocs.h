#include <stdlib.h>
#include <stdio.h>

#ifdef CHECK_N_WRAP_MALLOCS
extern "C" void *__real_malloc(size_t);
extern "C" void *__wrap_malloc(size_t c);

extern "C" void __real_free(void * c);
extern "C" void __wrap_free(void * c);
#endif

void EnableWrappers(bool on);
void EnableMallocWrappers(bool on);
void EnableFreeWrappers(bool on);
void SetWrappersOptions(bool bPrintf, bool bSaveBacktrace);


