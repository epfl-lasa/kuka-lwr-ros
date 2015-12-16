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

#ifndef STREAMABLE_H_
#define STREAMABLE_H_

/// Generic class interface for object I/O ffrom, to memory stream or array
class Streamable
{
public:
    /// Return the stream size that should be allowed to the object (memory size it takes)
    virtual int         StreamSize() = 0;
    /// Return the stream size that current memory location uses of this object
    virtual int         StreamSizeFromStream(const void* memory) = 0;
    /// Set the object stream into memory. Returns the size in bytes that is occupied by the stream
    virtual int         SetStream(void* memory) = 0;
    /// Set the object according to the stream in memory. Return the size in bytes that was read
    virtual int         SetFromStream(const void* memory) = 0;


    /// Tool function that return the size of a string (taking into acount an alignment to 32 bits)
    static  int         GetStreamStringSize(const char* string);
    /// Tool function that set a string at memory (taking into acount an alignment to 32 bits)
    static  int         SetStreamString(const char* string, void* memory);
    /// Tool function that read a string at memory (taking into acount an alignment to 32 bits)
    static  int         GetStreamString(char* string, const void* memory);

    /// Tool function to check that the current memory position is holding a given tag
    static  int         CheckStreamTag(const char* string, const void* memory);
};

#endif
