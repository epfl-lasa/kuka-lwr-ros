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

#include "lwr_console/Console.h"
#include "lwr_console/Various.h"


// Switch console mode (at least in the viewer)
// Add self namespace
// Check word wrap

#include <iostream>
using namespace std;

Console::Command::Command(string name){
  m_Name     = name;
  m_FilePath = ".";
}
Console::Command::Command(string name, string filepath){
  m_Name     = name;
  m_FilePath = filepath;
}
Console::Command::~Command(){
}

int Console::Command::Execute(string args){
  return 0;
}

streambuf * Console::m_Stdout = NULL;
streambuf * Console::m_Stderr = NULL;

Console::Console(){
  if(m_Stdout == NULL)
    m_Stdout = cout.rdbuf();
  if(m_Stderr == NULL)
    m_Stderr = cerr.rdbuf();

  m_Name = "Default";
  m_IsActive = true;
  m_Notifier = NULL;
  m_Parent   = NULL;

  Free();
}
Console::~Console(){
  Free();

  if(cout.rdbuf() == GetStreamBuf()){
    cout.rdbuf(m_Stdout);
  }
  if(cerr.rdbuf() == GetErrStreamBuf())
    cerr.rdbuf(m_Stderr);
}

void Console::Clear(){
  m_Lines.clear();
  m_History.clear();
  m_CurrY       = 0;
  m_CurrX       = 0;
  m_CurrCmd     = "";
  m_CurrHistory = 0;
  m_oStream.str("");
  m_oErrStream.str("");
  m_CursorPos   = 0;
}

void Console::clearLastLine(){
    m_Lines.pop_back();
}

void Console::Free(){
  for(unsigned int i=0;i<m_OwnedCommands.size();i++)
    delete m_OwnedCommands[i];

  m_Commands.clear();
  m_OwnedCommands.clear();
  m_MaxLines    = 80;
  m_MaxHistory  = 40;
  m_Notifier    = NULL;
  Clear();
}

Console*  Console::GetTopConsole(){
    if(m_Parent!=NULL) return m_Parent->GetTopConsole();
    return this;
}

Console*  Console::GetParent(){
    return m_Parent;
}

void      Console::AddConsole(Console* console){
    if(console==NULL) return;

  if(FindConsole(console->m_Name)){
    string cName = console->m_Name +string("0");
    int cnt=1;
    while(FindConsole(cName)){
      cName = console->m_Name + IntToString(cnt);
      cnt++;
    }
    Print(string("Warning: console ")+console->m_Name+string(" already exists. Renaming to ")+cName);
    console->m_Name = cName;
  }
  console->m_Parent = this;

  m_Consoles.push_back(console);
}
void      Console::ClearConsoles(){
    Console_List::iterator it = m_Consoles.begin();
    
    while(it!=m_Consoles.end()){
        if( (*it)->m_Parent == this){
            (*it)->m_Parent = NULL;
        }
        it++;
    }
    m_Consoles.clear();
}

void      Console::SetName(string name){
    m_Name = name;
}
void      Console::SetActive(bool active){
    m_IsActive = active;
}
bool      Console::IsActive(){
    return m_IsActive;
}

void Console::SetMaxLines(int size){
  if(size<=0)
    return;

  if(size>m_MaxLines)
    m_MaxLines = size;


  while(size<m_MaxLines){
    if(m_Lines.size()>(unsigned int)size)
      m_Lines.erase(m_Lines.begin());
    m_MaxLines--;
  }
}

void Console::SetMaxHistory(int size){
  if(size<=0)
    return;
  while(size<m_MaxHistory){
    m_History.erase(m_History.begin());
    m_CurrHistory--;
  }
}

void Console::AddLine(string line){
  if(m_Lines.size() >= (unsigned int)m_MaxLines){
    m_Lines.erase(m_Lines.begin());
    m_Lines.push_back(line);
  }else{
    m_Lines.push_back(line);
  }
}

void Console::AddHistory(string line){
  if(m_History.size() >= (unsigned int)m_MaxHistory){
    m_History.erase(m_History.begin());
    m_History.push_back(line);
  }else{
    m_History.push_back(line);
  }
  m_CurrHistory = m_History.size()-1;
}

void Console::MoveRight(bool bSkipWord){
  if(!m_IsActive) return;

  if(bSkipWord){
    bool spFound=false;
    bool bDone = false;
    for(int i=m_CursorPos;i<int(m_CurrCmd.size());i++){
      if(m_CurrCmd[i]==' '){
        spFound=true;
      }else{
        if(spFound==true){
          m_CursorPos=i;
          bDone = true;
          break;
        }
      }
    }
    if(!bDone)
      m_CursorPos=m_CurrCmd.size();
  }else{
    m_CursorPos++;
  }
  if(m_CursorPos>int(m_CurrCmd.size()))
    m_CursorPos=m_CurrCmd.size();
}

void Console::MoveLeft(bool bSkipWord){
  if(!m_IsActive) return;
  if(bSkipWord){
    bool bDone = false;
    bool spFound=false;
    bool txFound=false;
    for(int i=m_CursorPos-1;i>=0;i--){
      if(m_CurrCmd[i]==' '){
        spFound=true;
        if(txFound){
          m_CursorPos=i+1;
          bDone=true;
          break;
        }
      }else{
        txFound = true;
        /*if(spFound==true){
          m_CursorPos=i+2;
          bDone=true;
          break;
        }*/
      }
    }
    if(!bDone)
      m_CursorPos=0;
  }else{
    m_CursorPos--;
  }
  if(m_CursorPos<0)
    m_CursorPos=0;
 }

void Console::AddChar(char c){
  if(!m_IsActive) return;
  string s(1,c);
  //m_CurrCmd.append(s);
  m_CurrCmd = m_CurrCmd.substr(0,m_CursorPos)+s+m_CurrCmd.substr(m_CursorPos,m_CurrCmd.size()-m_CursorPos);
  MoveRight();
}

void Console::EraseChar(bool bkw){
  if(!m_IsActive) return;
  if(bkw){
    if((m_CurrCmd.size()>0)&&(m_CursorPos>0)){
      //m_CurrCmd = m_CurrCmd.substr(0,m_CurrCmd.size()-1);
      m_CurrCmd = m_CurrCmd.substr(0,m_CursorPos-1)+m_CurrCmd.substr(m_CursorPos,m_CurrCmd.size()-m_CursorPos);
      MoveLeft();
    }
  }else{
    if((m_CurrCmd.size()>0)&&(m_CursorPos<(int)m_CurrCmd.size())){
      m_CurrCmd = m_CurrCmd.substr(0,m_CursorPos)+m_CurrCmd.substr(m_CursorPos+1,m_CurrCmd.size()-m_CursorPos);
    }
  }
}

void Console::ClearLine(){
  if(!m_IsActive) return;
  m_CurrCmd = "";
  m_CursorPos=0;
}
void Console::Accept(bool bToHistory){
  if(!m_IsActive) return;
  if(bToHistory){
    if(m_CurrCmd.size()>0)
      AddHistory(m_CurrCmd);
  }

    string s = m_Name;
    s.append("> ");
    s.append(m_CurrCmd);
    AddLine(s);

    string cmd,nspace,args;
    ParseCommand(m_CurrCmd,cmd,nspace,args);

    if(nspace.length()>0){
        Console * cons = FindConsole(nspace);
        if(cons){
            if(cmd.length()>0){
                pCommand pCmd = cons->FindCommand(cmd);
                if(pCmd!=NULL){
                    pCmd->Execute(args);
                }else{
                    string s = nspace + "::"+cmd;
                    s = s.append(": Command not found");
                    AddLine(s);
                }
            }else{
                string s = nspace + "::";
                s = s.append(": No command provided");
                AddLine(s);
            }
        }else{
            string s = nspace;
            s = s.append(": Namespace not found");
            AddLine(s);
        }
    }else if(cmd.length()>0){
        pCommand pCmd = FindCommand(cmd);
        if(pCmd!=NULL){
            pCmd->Execute(args);
        }else{
            string s = cmd;
            s = s.append(": Command not found");
            AddLine(s);
        }
    }

    m_CurrCmd = "";
    m_CursorPos = 0;

    if(m_Notifier) m_Notifier->NeedUpdate();

}

void Console::ParseCommand(string name, string & cmd, string & nspace, string & args){
    cmd     = "";
    nspace  = "";
    args    = "";

    
    size_t cmdStart = name.find_first_not_of(" ");
    if(cmdStart != string::npos){

        name = name.substr(cmdStart);

        size_t cmdEnd = name.find_first_of(" ");
        if(cmdEnd == string::npos){
            cmd   = name;
        }else{
            cmd   = m_CurrCmd.substr(0,cmdEnd);
            args  = m_CurrCmd.substr(cmdEnd+1);
        }
        if(cmd.length()>0){
          size_t nsEnd  = cmd.find_first_of(":");
          if(nsEnd != string::npos){
            if(nsEnd+1<cmd.length()){
              if(cmd[nsEnd+1]==':'){
                //cout << cmd<<endl;
                nspace  = cmd.substr(0,nsEnd);
                cmd     = cmd.substr(nsEnd+2);
              }
            }
          }
        }
    }
}

void Console::Execute(string cmd, bool bToHistory){
  ClearLine();
  for(unsigned int i=0;i<cmd.size();i++){
    AddChar(cmd.at(i));
  }
  Accept(bToHistory);
}

Console*  Console::FindConsole(string name){
  unsigned int i;
  if(m_Name.compare(name)==0)
    return this;

  for(i=0;i<m_Consoles.size();i++){
    if(m_Consoles[i]->m_Name.compare(name)==0){
      return m_Consoles[i];
    }
  }
  return NULL;
}

string    Console::GetCurrentCommand(){
    return m_CurrCmd;
}
int    Console::GetCursorPos(){
    return m_CursorPos;
}

string    Console::GetName(){
    return m_Name;
}
vector<string> & Console::GetLines(){
    return m_Lines;
}
Console* Console::GetSubConsole(int i){
    if((i>=0)&&(i<int(m_Consoles.size()))){
        return m_Consoles[i];
    }
    return NULL;
}

Console::pCommand  Console::FindCommand(string name){
  unsigned int i;
  for(i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.compare(name)==0)
      return m_Commands[i];
  }
  return NULL;
}

vector<string> Console::AutoCompleteString(string cmd){
  vector<string> res;

  int len = cmd.size();

  for(size_t i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.substr(0,len).compare(cmd)==0)
      res.push_back(m_Commands[i]->m_Name);
  }
  return res;
}

vector<string> Console::AutoCompleteNameSpace(string cmd){
  vector<string> res;
  size_t len = cmd.size();

  size_t cmdEnd  = m_CurrCmd.find_first_of(":");
  size_t cmdEnd2 = m_CurrCmd.find_last_of(":");
  if(cmdEnd != string::npos){
    if(cmdEnd==0){
        return res;
    }else if(cmdEnd2+1<len){
        return res;   
    }else if((cmdEnd2-cmdEnd)>=2){
        return res;
    }else{
        len = cmdEnd;
    }
  }

  if(m_Name.substr(0,len).compare(cmd.substr(0,len))==0){
      string match = m_Name;
      match.append("::");
      res.push_back(match);
  }
  for(size_t i=0;i<m_Consoles.size();i++){
    //cout <<"\""<< cmd <<"\" \""<< m_Consoles[i]->m_Name<<"\""<<endl;
    if(m_Consoles[i]->m_Name.substr(0,len).compare(cmd.substr(0,len))==0){
      string match = m_Consoles[i]->m_Name;
      match.append("::");
      res.push_back(match);
    }
  }

  return res;
}


int Console::AutoCompletion(){
  if(!m_IsActive) return 0;

  unsigned int i;



  m_AutoMatches.clear();

  size_t cmdStart = m_CurrCmd.find_first_not_of(" ");
  string cmdToCompare = "";
  string cmdIndent    ="";
  if(cmdStart != string::npos){
    cmdToCompare = m_CurrCmd.substr(cmdStart);
    if(cmdStart>0){
      cmdIndent = m_CurrCmd.substr(0,cmdStart);
    }
  }else{
    cmdIndent = m_CurrCmd;
  }
  size_t len = cmdToCompare.size();

  bool bHasNamespace = false;
  string namesp="";
  size_t nsEnd  = m_CurrCmd.find_first_of(":");
  if(nsEnd != string::npos){
    if(nsEnd+1<len){
      if(m_CurrCmd[nsEnd+1]==':'){
        bHasNamespace = true;
        namesp = cmdToCompare.substr(0,nsEnd);
        cmdToCompare = cmdToCompare.substr(nsEnd+2);
        len = cmdToCompare.size();
        //cout <<"NS: "<<namesp<<endl;
        //cout <<"NS: "<<cmdToCompare<<endl;
      }
    }
  }

  vector<string> autoList;

  if(bHasNamespace){
    Console *cons = FindConsole(namesp);
    if(cons){
        vector<string> cs = cons->AutoCompleteString(cmdToCompare);
        for(size_t i=0;i<cs.size();i++){
            autoList.push_back(cs[i]);
        }
    }
    namesp.append("::");
  }else{

    vector<string> cs = AutoCompleteString(cmdToCompare);
    for(size_t i=0;i<cs.size();i++){
        autoList.push_back(cs[i]);
    }

    vector<string> ns = AutoCompleteNameSpace(cmdToCompare);
    for(size_t i=0;i<ns.size();i++)
        autoList.push_back(ns[i]);
  }




  if(autoList.size()==1){
    m_CurrCmd = cmdIndent + namesp+autoList[0];
    if(m_CurrCmd[m_CurrCmd.length()-1]!=':')
        m_CurrCmd.append(" ");
    m_CursorPos = m_CurrCmd.size();
  }else if(autoList.size()>1){
    int maxLen = len;
    while(1){
      string first = autoList[0];
      bool ok = true;
      for(i=1;i<autoList.size();i++)
          ok &= (autoList[i].substr(0,maxLen+1).compare(first.substr(0,maxLen+1))==0);
      if(ok)
          maxLen++;
      else
          break;
    }
    string s = m_Name;
    s.append("> ");
    s.append(m_CurrCmd);
    AddLine(s);
    s = "";
    for(int i=0;i<int(autoList.size());i++){
      if(i>0) s.append(" ");
      s.append(autoList[i]);
    }
    AddLine(s);
    m_CurrCmd   = cmdIndent + namesp+autoList[0].substr(0,maxLen);
    m_CursorPos = m_CurrCmd.size();
  }

#if 0
  for(i=0;i<m_Commands.size();i++){
    if(m_Commands[i]->m_Name.substr(0,len).compare(cmdToCompare)==0)
      m_AutoMatches.push_back(i);
  }

  if(m_AutoMatches.size()==1){
    m_CurrCmd = cmdIndent + m_Commands[m_AutoMatches[0]]->m_Name;
    m_CurrCmd.append(" ");
    m_CursorPos = m_CurrCmd.size();
  }else if(m_AutoMatches.size()>1){
    int maxLen = len;
    while(1){
      string first = m_Commands[m_AutoMatches[0]]->m_Name;
      bool ok = true;
      for(i=1;i<m_AutoMatches.size();i++)
          ok &= (m_Commands[m_AutoMatches[i]]->m_Name.substr(0,maxLen+1).compare(first.substr(0,maxLen+1))==0);
      if(ok)
          maxLen++;
      else
          break;
    }
    string s = m_Name;
    s.append("> ");
    s.append(m_CurrCmd);
    AddLine(s);
    s = "";
    for(int i=0;i<int(m_AutoMatches.size());i++){
      if(i>0) s.append(" ");
      s.append(m_Commands[m_AutoMatches[i]]->m_Name);
    }
    AddLine(s);
    m_CurrCmd   = cmdIndent + m_Commands[m_AutoMatches[0]]->m_Name.substr(0,maxLen);
    m_CursorPos = m_CurrCmd.size();
  }
#endif
  if(m_Notifier) m_Notifier->NeedUpdate();

  return m_AutoMatches.size();
}

vector<string> Console::AutoCompleteFilename(){
  vector<string> results;
  if(!m_IsActive) return results;

  unsigned int pos = m_CurrCmd.find_last_of(" ");
  if(pos!=string::npos){

    string sDir = ".";
    vector<string> cmds = Tokenize(m_CurrCmd);
    if(cmds.size()>=1){
      sDir = FindCommand(cmds[0])->m_FilePath;
    }

    vector<string> filesList = ScanDir(sDir);
    string target = "";

    if(m_CurrCmd[m_CurrCmd.size()-1]!=' '){
      target.append(m_CurrCmd.substr(pos+1));
    }

    results = ::AutoCompletion(filesList,target);
    if(results.size()==1){
      target.append(" ");
    }
    m_CurrCmd = m_CurrCmd.substr(0,pos+1);
    m_CurrCmd.append(target);
    m_CursorPos = m_CurrCmd.size();

  }
  return results;
}

void Console::AddCommand(pCommand cmd,bool bOwnership){
  if(cmd==NULL)
    return;
  if(FindCommand(cmd->m_Name)){
    string cName = cmd->m_Name +string("0");
    int cnt=1;
    while(FindCommand(cName)){
      cName = cmd->m_Name + IntToString(cnt);
      cnt++;
    }
    Print(string("Warning: command ")+cmd->m_Name+string(" already exists. Renaming to ")+cName);
    cmd->m_Name = cName;
  }
  m_Commands.push_back(cmd);
  if(bOwnership)
    m_OwnedCommands.push_back(cmd);
}

void Console::HistoryPrev(){
  if(m_History.size()>0){
    m_CurrCmd = m_History[m_CurrHistory];
    m_CursorPos = m_CurrCmd.size();
    m_CurrHistory--;
    if(m_CurrHistory<0)
      m_CurrHistory=0;
  }
}
void Console::HistoryNext(){
  if(m_History.size()>0){
    m_CurrHistory++;
    if(m_CurrHistory<(int)m_History.size()){
      m_CurrCmd = m_History[m_CurrHistory];
      m_CursorPos = m_CurrCmd.size();
    }else{
      m_CurrCmd = "";
      m_CursorPos = 0;
      m_CurrHistory--;
    }
  }
}

void Console::Print(string line){
  AddLine(line);
  if(m_Notifier) m_Notifier->NeedUpdate();
}

bool Console::Update(){
  string s = m_oStream.str();

  size_t cpos = 0;
  size_t opos = 0;

  bool bNeedUpdate = false;

  if(s.size()>0){
    for(size_t i=0;i<s.size();i++){
      opos = cpos;
      cpos = s.find("\n",opos);
      string ss = s.substr(opos,cpos-opos);
      if(ss.size()>0){
        AddLine(ss);
        bNeedUpdate = true;
      }
      if(cpos==string::npos)
        break;
      cpos++;
    }
  }
  m_oStream.str("");

  s = m_oErrStream.str();

  cpos = 0;
  opos = 0;
  if(s.size()>0){
    for(size_t i=0;i<s.size();i++){
      opos = cpos;
      cpos = s.find("\n",opos);
      string ss = s.substr(opos,cpos-opos);
      if(ss.size()>0){
        AddLine(ss);
        bNeedUpdate = true;
      }
      if(cpos==string::npos)
        break;
      cpos++;
    }
  }
  m_oErrStream.str("");

  if(bNeedUpdate){
      if(m_Notifier) m_Notifier->NeedUpdate();
  }
  return bNeedUpdate;
}

ostream  *Console::GetStream(){
  return &m_oStream;
}
ostream  *Console::GetErrStream(){
  return &m_oErrStream;
}

streambuf *Console::GetStreamBuf(){
  return m_oStream.rdbuf();
}
streambuf *Console::GetErrStreamBuf(){
  return m_oErrStream.rdbuf();
}

void Console::SetStdout(){
  cout.rdbuf(GetStreamBuf());
}
void Console::SetStderr(){
  cerr.rdbuf(GetErrStreamBuf());
}
/*
ofstream outFile("some_file_name.dat");//create output file object
streambuf * cout_backup=cout.rdbuf();//create backup of standard out
cout.rdbuf(outFile.rdbuf());//assign cout stream to outFile stream
cout<<"hello!"<<endl;//call any function that calls cout
cout.rdbuf(cout_backup);//restore the standard stream
outFile.close();//best to be neat and tidy about these things
*/
void Console::SetNotifier(ConsoleNotification * notifier){
    m_Notifier = notifier;
}

void ConsoleNotification::NeedUpdate(){}

