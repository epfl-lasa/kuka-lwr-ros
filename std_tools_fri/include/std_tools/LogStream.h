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

#ifndef LOGSTREAM_H_
#define LOGSTREAM_H_


#include "XmlTree.h"
#include <stdarg.h>

#define LOGSTREAM_ALL           -1
#define LOGSTREAM_DEFAULTENTRY  "Default"
#define LOGSTREAM_MAXINDENT     32

typedef vector<string> StringVector;

class LogStream
{
protected:
  pXmlTree        mLogTree;
  pXmlTree        mOptionsTree;
  
  pXmlTree        mCurrentEntry;
  pXmlTree        mCurrentOptions;
  
  StringVector    mStringList;
 
  char            mSpaces[LOGSTREAM_MAXINDENT+1];

  bool            bDefaultAutoPrint;
  bool            bCurrentEntryLocked; 
public:
          LogStream();
  virtual ~LogStream();  
  
          void            ClearAll();
  
          void            SetCurrentEntry(const string entry, bool lock = false);
          void            LockCurrentEntry();
          void            UnlockCurrentEntry();

          void            Clear();
          void            Clear(const string entry);
          
          void            Append(const string data);  
          void            AppendToEntry(const string entry, const string data);
          void            Append(const char* txt, ...);  
          void            AppendToEntry(const string entry, const char* txt, ...);
          
          StringVector&   GetStrings(int nbLines = LOGSTREAM_ALL);
          StringVector&   GetStrings(const string entry, int nbLines = LOGSTREAM_ALL);
          
          string          GetLastString();
          string          GetLastString(const string entry);

          void            SetAutoPrint(bool bAuto=true);
          void            SetAutoPrint(const string entry, bool bAuto=true);
          void            SetDefaultAutoPrint(bool bAuto=true);

          void            SetDeltaIndent(int dIndent);
          void            SetDeltaIndent(const string entry, int dIndent);
                    
          void            Print(int nbLines = LOGSTREAM_ALL);
          void            Print(const string entry, int nbLines = LOGSTREAM_ALL);

          void            SetOStream(std::ostream & ostr);
          void            SetOStream(const string entry, std::ostream & ostr);

          
          int             Save(const string filename);
          int             Load(const string filename);
          
          void            PrintAll();
};


#ifdef LOGSTREAM_CPP_
            LogStream   globalLogStream;
#else
    extern  LogStream   globalLogStream;
#endif

#define gLOG    globalLogStream

#endif /*LOGSTREAM_H_*/
