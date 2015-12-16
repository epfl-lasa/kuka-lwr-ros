#include "WrapMallocs.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <execinfo.h>
#include <string.h>

/* For Debugging: This function will execute when a RealTime thread switches to
 * secondary mode.  Also, one can look at /proc/xenomai/stat and MSW will
 * tell you the number of switches a thread has made.
 */


bool bEnableWrapMallocCheck     = false;
bool bEnableWrapFreeCheck       = false;
bool bEnableWrapPrint           = true;
bool bEnableWrapBacktrace       = true;
   
#ifdef CHECK_N_WRAP_MALLOCS

void saveBackTrace(const char *msg = NULL)
{
    void *bt[32];
    char txt[256];
    int nentries;
    int fd;

     // Open backtrace file as APPEND. Create new file if necessary (chmod 644)
    fd = open("backtrace.txt", O_WRONLY | O_APPEND | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if(msg!=NULL)
        snprintf(txt,255,"---- %s:\n",msg);
    else
        snprintf(txt,255,"----\n");

    write(fd, txt, strlen(txt)); // Output a separator line

    nentries = backtrace(bt, sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt, nentries, fd);

    close(fd);
}


void * __wrap_malloc (size_t c){
    if(bEnableWrapMallocCheck){
        if(bEnableWrapPrint) printf ("Catched malloc called with size %zu\n", c);
        if(bEnableWrapBacktrace) saveBackTrace("malloc");
    }
    return __real_malloc (c);
}

void* operator new(size_t c) {
    if(bEnableWrapMallocCheck){
        if(bEnableWrapPrint) printf ("Catched new called with size %zu\n", c);
        if(bEnableWrapBacktrace) saveBackTrace("new");
    }
  void* m = malloc(c);
  return m;
}
void *operator new[](size_t c){
    if(bEnableWrapMallocCheck){
        if(bEnableWrapPrint) printf ("Catched new[] called with size %zu\n", c);
        if(bEnableWrapBacktrace) saveBackTrace("new[]");
    }
  void* m = __real_malloc(c);
  return m;

}

void __wrap_free (void * c){
    if(bEnableWrapFreeCheck){
        if(bEnableWrapPrint) printf ("Catched free\n");
        if(bEnableWrapBacktrace) saveBackTrace("free");
    }
    __real_free (c);
}

void operator delete(void* m) {
    if(bEnableWrapFreeCheck){
        if(bEnableWrapPrint) printf ("Catched delete \n");
        if(bEnableWrapBacktrace) saveBackTrace("delete");
    }
    __real_free(m);
}

void operator delete[](void* m) {
    if(bEnableWrapFreeCheck){
        if(bEnableWrapPrint) printf ("Catched delete[]  \n");
        if(bEnableWrapBacktrace) saveBackTrace("delete[]");
    }
    __real_free(m);
}


#endif



void EnableWrappers(bool on){
    bEnableWrapMallocCheck = on;
    bEnableWrapFreeCheck   = on;
}
void EnableMallocWrappers(bool on){
    bEnableWrapMallocCheck = on;
}
void EnableFreeWrappers(bool on){
    bEnableWrapFreeCheck = on;
}
void SetWrappersOptions(bool bPrintf, bool bSaveBacktrace){
    bEnableWrapPrint           = bPrintf;
    bEnableWrapBacktrace       = bSaveBacktrace;
}


