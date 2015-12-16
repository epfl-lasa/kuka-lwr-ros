/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "Streamable.h"
#include <string.h>


int         Streamable::GetStreamStringSize(const char* string){
    int len = strlen(string)+1;
    if((len & 0x03)!=0)
        len += (4-(len & 0x03));
    return len;
}
int         Streamable::SetStreamString(const char* string, void* memory){
    strcpy((char*)memory,string);
    int len = strlen(string)+1;
    if((len & 0x03)!=0)
        len += (4-(len & 0x03));
    return len;
}
int         Streamable::GetStreamString(char* string, const void* memory){
    char *mem = (char*)memory;
    if(string!=NULL)
        strcpy(string,mem);
    int len = strlen(mem)+1;
    if((len & 0x03)!=0)
        len += (4-(len & 0x03));
    return len;
}
int         Streamable::CheckStreamTag(const char* tag, const void* memory){
    int size = strlen(tag);
    if(strncmp(tag,(char*)memory,size+1)==0){
        int len = strlen(tag)+1;
        if((len & 0x03)!=0)
            len += (4-(len & 0x03));
        return len;
    }else{
        return 0;
    }
}
