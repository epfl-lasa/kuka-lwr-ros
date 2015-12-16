#include "WrapMallocs.h"


#ifdef CHECK_N_WRAP_MALLOCS
void * __real_malloc (size_t c){
    return malloc (c);
}

void __real_free (void * c){
    free (c);
}
#endif


