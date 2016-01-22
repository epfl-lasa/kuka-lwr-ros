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

#ifndef NCCONSOLE_H_
#define NCCONSOLE_H_

#include "Console.h"


class NCConsole
{
protected:
    Console *mConsole;
    Console *mMainConsole;
    int      mCurrConsole;

public:
            NCConsole();
    virtual ~NCConsole();

            void  SetConsole(Console *myConsole);

            void  InitNCurses();
            void  FreeNCurses();

            void  Print(string s);

    virtual void  Render();

            void  Process();
            void  Resize();

            void  NextConsole();
            void  PrevConsole();

            void  SetTopStaticLinesCount(int count);
            void  SetTopStaticLine(int id,string line);

protected:
    bool    bNCursesReady;
    bool    bNeedRefresh;
    int     mScreenWidth;
    int     mScreenHeight;
    int             mNbTopStaticLines;
    vector<string>  mTopStaticLines;

};


#endif
