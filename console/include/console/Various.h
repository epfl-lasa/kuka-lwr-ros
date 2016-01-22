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

#ifndef __VARIOUS_H__
#define __VARIOUS_H__

#include <string>
#include <vector>
#include <deque>
using namespace std;

string Int01ToString(int i);
string Int02ToString(int i);
string Int03ToString(int i);
string Int04ToString(int i);

string IntToString(int i);
string FloatToString(float f);
string DoubleToString(double d);
string BoolToString(bool b);

vector<string> Tokenize(string params);
vector<string> Tokenize(string params, string delim, string exDelim);
string Serialize(vector<string>, int start=0, int cnt=-1);

string RemoveSpaces(string s);

string GetPathFromFilename(string fname);
string GetFileFromFilename(string fname);

int    GetConsecutiveFileCount(const char * dir, const char *nameTag, int maxS=20);
bool   FileExists(string strFilename);

class FileFinder
{
protected:
    static deque<string>    mBasePaths;
    static deque<string>    mAdditionalPaths;
    static string           mCurrentFile;

public:
    static bool        Find(const char* filename);
    static bool        Find(string filename);

    static const char* GetCStr();
    static string      GetString();

    static void                    SetBasePaths(deque<string> & paths);
    static void                    AddBasePath(string newPath);
    static void                    ClearBasePaths();

    static void                    AddAdditionalPath(string newPath);
    static void                    ClearAdditionalPaths();

    static string                  GetTopLevelPath();

    static const deque<string>     GetAllPaths();
};



#ifdef WIN32          // Linux Wrap for Win32
#define usleep(X)     Sleep((X) / 1000)
#define sleep(X)      Sleep((X) * 1000)

#define	MSG_DONTWAIT	0
#define socklen_t     int FAR


#else                 // Windows warp for linux

long int GetTickCount();

#endif

vector<string> ScanDir(string dirName);
vector<string> AutoCompletion(vector<string> & choices, string & target);



#endif


