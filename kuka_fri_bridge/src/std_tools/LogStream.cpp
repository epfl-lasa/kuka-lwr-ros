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

#define LOGSTREAM_CPP_

#include "LogStream.h"
#include "Various.h"



LogStream::LogStream(){
  mLogTree      = new XmlTree();
  mOptionsTree  = new XmlTree();
  bDefaultAutoPrint = false;
  bCurrentEntryLocked = false;
  for(int i=0;i<LOGSTREAM_MAXINDENT;i++) mSpaces[i] = ' ';
  mSpaces[LOGSTREAM_MAXINDENT] = 0;
  ClearAll();  
}

LogStream::~LogStream(){
  ClearAll();
  delete mLogTree;  
  delete mOptionsTree;
}
  
void            LogStream::ClearAll(){
  mStringList.clear();
  
  mLogTree->Clear();
  mLogTree->SetName("Log");

  mOptionsTree->Clear();
  mOptionsTree->SetName("Log");

  mCurrentEntry   = new XmlTree(LOGSTREAM_DEFAULTENTRY);
  mCurrentEntry->SetDataPtr(&std::cout);
  mCurrentOptions = new XmlTree(LOGSTREAM_DEFAULTENTRY);

  mLogTree->AddSubTree(mCurrentEntry);
  mOptionsTree->AddSubTree(mCurrentOptions);
}

void            LogStream::SetCurrentEntry(const string entry, bool lock){
    if(!bCurrentEntryLocked){
        mCurrentEntry   = mLogTree->Find(entry);
        mCurrentOptions = mOptionsTree->Find(entry);
        if(mCurrentEntry == NULL){
            mCurrentEntry   = new XmlTree(entry);
            mCurrentEntry->SetDataPtr(&std::cout);
            mCurrentOptions = new XmlTree(entry);
            mLogTree->AddSubTree(mCurrentEntry);
            mOptionsTree->AddSubTree(mCurrentOptions);

            mCurrentOptions->Set("AutoPrint",bDefaultAutoPrint);
        }
        if(lock) LockCurrentEntry();
    }
}
void            LogStream::LockCurrentEntry(){
    bCurrentEntryLocked = true;
}
void            LogStream::UnlockCurrentEntry(){
    bCurrentEntryLocked = false;
}


void            LogStream::Clear(){
  string entryName = mCurrentEntry->GetName(); 
  void * ptr = mCurrentEntry->GetDataPtr();
  mCurrentEntry->Clear();
  mCurrentOptions->Clear(); 
  mCurrentEntry->SetName(entryName);
  mCurrentEntry->SetDataPtr(ptr);
  mCurrentOptions->SetName(entryName);
}
void            LogStream::Clear(const string entry){
  SetCurrentEntry(entry);
  Clear(); 
}

void            LogStream::Append(const string data){
  int cIndent = mCurrentOptions->CGet("Indent",0);
  mSpaces[cIndent] = 0;
  string text(mSpaces);
  text.append(data);
  mCurrentEntry->AddSubTree(new XmlTree("Entry",text));
  if(mCurrentOptions->CGet("AutoPrint",false))
    Print(1);  
  mSpaces[cIndent] = ' ';
}  

void            LogStream::AppendToEntry(const string entry, const string data){
  SetCurrentEntry(entry);
  Append(data); 
}

void            LogStream::Append(const char* txt, ...){
    char buffer[512];

    va_list args;
    va_start( args, txt );
    vsnprintf( buffer, 512, txt, args );
    va_end( args );

    Append(string(buffer));
} 
void            LogStream::AppendToEntry(const string entry, const char* txt, ...){
    SetCurrentEntry(entry);
    
    char buffer[512];

    va_list args;
    va_start( args, txt );
    vsnprintf( buffer, 512, txt, args );
    va_end( args );

    Append(string(buffer));
}


StringVector&   LogStream::GetStrings(int nbLines){
  pXmlTreeList list = mCurrentEntry->GetSubTrees();
  unsigned int size = list->size();  

  unsigned int start = 0;
  if(nbLines>0){
    if(nbLines>int(size)){
      nbLines = int(size);  
    }
  }
  start = list->size() - ((unsigned int)nbLines); 
  
  mStringList.resize(size-start);
  for(unsigned int i=start;i<size;i++){
    mStringList[i-start] = list->at(i)->GetData();
  }
  return mStringList;
}

StringVector&   LogStream::GetStrings(const string entry, int nbLines){
  SetCurrentEntry(entry);
  return GetStrings(nbLines);  
}


string          LogStream::GetLastString(){
  pXmlTreeList list = mCurrentEntry->GetSubTrees();
  unsigned int size = list->size();
  if(size>0){
    return list->at(size-1)->GetData();
  }else{
    return "";  
  }  
}

string          LogStream::GetLastString(const string entry){
  SetCurrentEntry(entry);
  return GetLastString();
}


void            LogStream::Print(int nbLines){
  pXmlTreeList list = mCurrentEntry->GetSubTrees();
  unsigned int size = list->size();  

  unsigned int start = 0;
  if(nbLines>0){
    if(nbLines>int(size)){
      nbLines = int(size);  
    }
    start = list->size() - ((unsigned int)nbLines); 
  }else{
    start = 0;
  }
  
  for(unsigned int i=start;i<size;i++){
    std::ostream *ccout = (std::ostream*)mCurrentEntry->GetDataPtr();
    (*ccout) << mCurrentEntry->GetName()<<": "<<list->at(i)->GetData() << endl;
  }
}

void            LogStream::Print(const string entry, int nbLines){
  SetCurrentEntry(entry);
  Print(nbLines);
}

void            LogStream::SetDefaultAutoPrint(bool bAuto){
    bDefaultAutoPrint = bAuto;
}
void            LogStream::SetAutoPrint(bool bAuto){
  mCurrentOptions->Set("AutoPrint",bAuto);
}
void            LogStream::SetAutoPrint(const string entry, bool bAuto){
  SetCurrentEntry(entry);
  SetAutoPrint(bAuto);
}
void            LogStream::SetDeltaIndent(int dIndent){
  int cIndent = mCurrentOptions->CGet("Indent",0) + dIndent;
  if(cIndent<0) cIndent = 0;
  if(cIndent>LOGSTREAM_MAXINDENT) cIndent = LOGSTREAM_MAXINDENT;
  mCurrentOptions->Set("Indent",cIndent);
}
void            LogStream::SetDeltaIndent(const string entry, int dIndent){
  SetCurrentEntry(entry);
  SetDeltaIndent(dIndent);
}
void            LogStream::SetOStream(std::ostream & ostr){
    mCurrentEntry->SetDataPtr(&ostr);
}
void            LogStream::SetOStream(const string entry, std::ostream & ostr){
  SetCurrentEntry(entry);
  SetOStream(ostr);
}


void            LogStream::PrintAll(){
  pXmlTreeList list = mLogTree->GetSubTrees();
  for(unsigned int i=0;i<list->size();i++){
    mCurrentEntry = list->at(i);
    pXmlTreeList list2 = mCurrentEntry->GetSubTrees();
    if(list2->size()>0){
      cout << "********************"<<endl;
      cout << "* ENTRY: <"<< mCurrentEntry->GetName() <<">"<<endl;
      cout << "*-------------------"<<endl;
      Print();
    }
  }
  cout << "********************"<<endl;
  
}


int             LogStream::Save(const string filename){
  return mLogTree->SaveToFile(filename);  
}

int             LogStream::Load(const string filename){
  return mLogTree->LoadFromFile(filename);    
}

