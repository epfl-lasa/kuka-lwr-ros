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

#include "NCConsole.h"

#include <ncurses.h>
#include <string.h>

NCConsole::NCConsole(){
  mScreenHeight = 0;
  mMainConsole  = NULL;
  mConsole      = NULL;
  mCurrConsole  = -1;
  bNeedRefresh      = true;
  bNCursesReady     = false;
  SetTopStaticLinesCount(4);
}
NCConsole::~NCConsole(){
    if(bNCursesReady)
        FreeNCurses();
}

void  NCConsole::SetConsole(Console *myConsole){
  mMainConsole  = myConsole;
  mConsole      = myConsole;
  mCurrConsole  = -1;
  mConsole->SetMaxLines(20);
}

void  NCConsole::Print(string s){
    mConsole->Print(s);
}


void NCConsole::InitNCurses(){
    if(bNCursesReady) return;
    initscr();
    cbreak();
    noecho();
    nonl();
    keypad(stdscr, TRUE);
    timeout(0);
    clear();
    Resize();
    bNCursesReady = true;
}
void  NCConsole::FreeNCurses(){
    if(!bNCursesReady) return;
    endwin();
    bNCursesReady = false;
}

void  NCConsole::Resize(){
    int x,y;
    getmaxyx(stdscr,y,x);
    if(mScreenHeight!=y){
        mScreenHeight = y;
        if(y<16) y= 16;
        mConsole->SetMaxLines(y);
    }
    mScreenWidth  = x;
    bNeedRefresh = true;
}
void  NCConsole::Render(){
    if(!bNCursesReady)
        InitNCurses();

    if(!bNeedRefresh) return;
    if(mScreenHeight<=0) return;
    clear();

    mConsole->Update();

    int linePos = 0;
    int nbLinesAvailable = mScreenHeight-1-mNbTopStaticLines;
    if(nbLinesAvailable>=0){
        for(int i=0;i<int(mNbTopStaticLines);i++){
            mvprintw(linePos,0,mTopStaticLines[i].c_str());
            linePos++;
        }
    }else{
        for(int i=0;i<int(mNbTopStaticLines+nbLinesAvailable);i++){
            mvprintw(linePos,0,mTopStaticLines[i].c_str());
            linePos++;
        }
    }

    if(nbLinesAvailable>0){
        linePos     = (nbLinesAvailable-mConsole->m_Lines.size());
        int startLine   = 0;
        if(linePos<0){
            startLine   = -linePos;
            linePos     = 0;
        }
        linePos += mNbTopStaticLines;

        for(int i=startLine;i<int(mConsole->m_Lines.size());i++){
            mvprintw(linePos,0,mConsole->m_Lines[i].c_str());
            linePos++;
        }
    }

    string s;
    s = mConsole->GetName();
    s.append("> ");
    s.append(mConsole->m_CurrCmd);
    mvprintw(linePos,0, s.c_str());
    move(linePos,mConsole->m_CursorPos+2+mConsole->m_Name.length());

    refresh();
}
void NCConsole::Process(){
    Resize();
    bNeedRefresh = true;
    int key;
    do{
        key = wgetch(stdscr);

        if(key>=0)
            bNeedRefresh = true;
        else if(key<0)
            break;

        if((key>=' ') && (key<'~')){
            mConsole->AddChar(key);
        }else{
            switch(key){
            case KEY_BACKSPACE:
                mConsole->EraseChar(true);
                break;
            case 330:
                mConsole->EraseChar(false);
                break;
            case 13:
            case KEY_ENTER:
                mConsole->Accept();
                break;
            case 12:
                mConsole->ClearLine();
                break;
            case '\t':
                mConsole->AutoCompletion();
                break;
            case KEY_UP:
                mConsole->HistoryPrev();
                break;
            case KEY_DOWN:
                mConsole->HistoryNext();
                break;
            case KEY_LEFT:
                mConsole->MoveLeft();
                break;
            case KEY_RIGHT:
                mConsole->MoveRight();
                break;
            case KEY_RESIZE:
                Resize();
                /*{
                char str[128];
                for(int i=0;i<strlen(str);i++)
                    AddChar(str[i]);
                }*/
                break;
            case 339: //PgUp
                PrevConsole();
                break;
            case 338: //PgDown
                NextConsole();
                break;
            default:
                /*
                char str[128];
                sprintf(str,"<%d> ",key);
                for(int i=0;i<strlen(str);i++)
                    mConsole->AddChar(str[i]);
                */                
                break;
            }
        }
    }while(key>=0);

    if(bNeedRefresh)
        Render();
    bNeedRefresh = false;
}

void NCConsole::SetTopStaticLinesCount(int count){
    mNbTopStaticLines = (count<0?0:count);
    mTopStaticLines.resize(mNbTopStaticLines);
    for(int i=0;i<mNbTopStaticLines;i++){
        mTopStaticLines[i] = "";
    }
}
void  NCConsole::SetTopStaticLine(int id, string line){
    if((id>=0)&&(id<mNbTopStaticLines)){
        mTopStaticLines[id] = line;
    }
}

void  NCConsole::NextConsole(){
    mCurrConsole++;
    Console* cc = mMainConsole->GetSubConsole(mCurrConsole);
    if(cc==NULL){
        mCurrConsole--;
    }else{
        mConsole = cc;
        Resize();
    }
}
void  NCConsole::PrevConsole(){
    if(mCurrConsole==0){
        mCurrConsole = -1;
        mConsole = mMainConsole;
        Resize();
    }else if(mCurrConsole > 0){
        mCurrConsole--;
        mConsole = mMainConsole->GetSubConsole(mCurrConsole);
        Resize();
    }
}

