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

#include "Timer.h"

#define TRUE  1
#define FALSE 0

Chrono::Chrono(){
#ifdef WIN32
  QueryPerformanceFrequency((LARGE_INTEGER*)&m_Frequency);
#endif
  Start();
}

void Chrono::Start(){
  bIsPaused = false;
#ifdef WIN32
  QueryPerformanceCounter((LARGE_INTEGER*)&m_StartTime);
#else
  gettimeofday(&m_StartTime,NULL);
  //m_StartTime  = m_TV.tv_sec  * 1000;
  //m_StartTime += m_TV.tv_usec / 1000;
#endif
}

double  Chrono::ElapsedTime() const{
    double currTime;
#ifdef WIN32
  __int64 mCT;
  if(bIsPaused){
    mCT = m_PauseTime;
  }else{
    QueryPerformanceCounter((LARGE_INTEGER*)&mCT);
  }
  currTime = double(mCT - m_StartTime) / double(m_Frequency);
  return currTime;
#else
  struct timeval mCTV;
  struct timeval mDTV;
  if(bIsPaused){
    mCTV = m_PauseTime;
  }else{
      gettimeofday(&mCTV,NULL);
  }
  timersub(&mCTV,&m_StartTime,&mDTV);
  currTime      = double(mDTV.tv_sec);
  currTime     += double(mDTV.tv_usec)*0.000001;
  return (currTime);
#endif
}

#ifndef WIN32
double Chrono::ElapsedTime(struct timeval baseTime) const{
  double currTime;
  struct timeval mDTV;
  timersub(&baseTime,&m_StartTime,&mDTV);
  currTime      = double(mDTV.tv_sec);
  currTime     += double(mDTV.tv_usec)*0.000001;
  return (currTime);
}
#else
double Chrono::ElapsedTime(__int64 baseTime) const{
  double currTime;
  currTime = double(baseTime - m_StartTime) / double(m_Frequency);
  return currTime;
}
#endif

long long Chrono::ElapsedTimeMs() const{
  long long currTime;
#ifdef WIN32
  __int64 mCT;
  if(bIsPaused){
    mCT = m_PauseTime;
  }else{
    QueryPerformanceCounter((LARGE_INTEGER*)&mCT);
  }
  currTime = (long long)(((mCT - m_StartTime) * 1000) / m_Frequency);
  return currTime;
#else
  struct timeval mCTV;
  struct timeval mDTV;
  if(bIsPaused){
    mCTV = m_PauseTime;
  }else{
      gettimeofday(&mCTV,NULL);
  }
  timersub(&mCTV,&m_StartTime,&mDTV);
  currTime      = (long long)(mDTV.tv_sec*((long long)(1000))) ;
  currTime     += (long long)(mDTV.tv_usec/((long long)(1000)));
  return (currTime);
#endif
}

long long Chrono::ElapsedTimeUs() const{
  long long currTime;
#ifdef WIN32
  __int64 mCT;
  if(bIsPaused){
    mCT = m_PauseTime;
  }else{
    QueryPerformanceCounter((LARGE_INTEGER*)&mCT);
  }
  currTime = (long long)(((mCT - m_StartTime) * 1000000) / m_Frequency);
  return currTime;
#else
  struct timeval mCTV;
  struct timeval mDTV;
  if(bIsPaused){
    mCTV = m_PauseTime;
  }else{
      gettimeofday(&mCTV,NULL);
  }
  timersub(&mCTV,&m_StartTime,&mDTV);
  currTime      = (long long)(mDTV.tv_sec *((long long)(1000000)));
  currTime     += (long long)(mDTV.tv_usec);
  return (currTime);
#endif
}
void  Chrono::Pause(){
  if(!bIsPaused){
#ifdef WIN32
    QueryPerformanceCounter((LARGE_INTEGER*)&m_PauseTime);
#else
    gettimeofday(&m_PauseTime,NULL);
#endif
    bIsPaused = true;
  }
}

bool  Chrono::IsPaused(){
    return bIsPaused;
}
void  Chrono::Resume(){
  if(bIsPaused){
#ifdef WIN32
  __int64 mCT;
  __int64 mDT;
  QueryPerformanceCounter((LARGE_INTEGER*)&mCT);
  mDT = mCT - m_PauseTime;
  m_StartTime += mDT;
#else
    struct timeval mCTV;
    struct timeval mDTV;
    gettimeofday(&mCTV,NULL);
    timersub(&mCTV,&m_PauseTime,&mDTV);
    mCTV = m_StartTime;
    timeradd(&mCTV,&mDTV,&m_StartTime);
#endif
    bIsPaused = false;
  }
}

/*
void  Chrono::AddSeconds(long secs){
    m_StartTime.tv_sec += secs;
    if(bIsPaused){
        m_PauseTime.tv_sec += secs;
    }
}
*/

Chrono PerformanceEstimator::mChrono;
