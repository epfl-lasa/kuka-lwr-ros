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

#ifndef BASEOBJECT_H_
#define BASEOBJECT_H_

#include <vector>
#include <string.h>
#include <stdio.h>
#include <typeinfo>

/* HowTo:
 * One level Hierarchical Oriented Object Macros
 *
 * Header:
 * - In place of
 *     class myA {
 * - Write
 *     BASEOBJECT_TOPCLASS(myA)
 *
 * - In place of
 *     class myB : public myA
 *  -Write
 *     BASEOBJECT_CLASS(myB,myA)
 *
 * Definition:
 * - Add:
 *     BASEOBJECT_TOPDEF(myA);
 *     BASEOBJECT_DEF(myB);
 *
 * All this mess provides:
 *  static  int  myA::GetClassType();        // with result = 0
 *  static  int  myB::GetClassType();        // with result = 1
 *  static  int  myC::GetClassType();        // with result = ...
 *
 *  static  int  myA::GetClassTypeCount();   // with result = # of defined subtypes +1 (the father...)
 *  static  myA* myA::Cast(const myA* obj);  // return myA if the class is good or NULL otherwise
 *          int  objA.GetObjectType();       // if objA is of type myA, result = 0
 *                                           //                    myB, result = 1
 *                                           //                    myC, result = ...
 *
 *          myA* objA.ClassClone();          // which does the same as "new" but the
 *                                           //  result will be of the same type as objA
 *
 *
 *
 */


//  static int mClassName[256];
//  static char* SetClassName() { strcpy((char*)mClassName,"className"); return "className";};

#define BASEOBJECT_DECL(className, topClass) \
private: \
  static int  mClassType; \
public: \
  virtual inline int GetObjectType() const {return mClassType;}  \
  static  inline int GetClassType() {return mClassType;} \
  static  inline topClass* BaseNew(){return new className();}; \
  static  inline className* Cast(const topClass* obj){if(obj){if(mClassType==obj->GetObjectType()) return (className*)obj;} return NULL;} \
  virtual topClass* ClassClone() {return topClass::CreateByObjType(*this);}

#define BASEOBJECT_CLASS(className, topClass) \
class className : public topClass { \
BASEOBJECT_DECL(className, topClass)


#define BASEOBJECT_TOPCLASS(className) \
class className { \
private: \
  static int mClassTypeCount; \
  static std::vector<className* (*)()> mConstructors; \
protected: \
  static int RegisterNewType(className* (*constr)()){ \
    mConstructors.push_back(constr); \
    return mClassTypeCount++;} \
public: \
  static int GetClassTypeCount(){ return mClassTypeCount;} \
  static className* CreateByType(int id){ \
    if((id>=0)&&(id<mClassTypeCount)) \
      return mConstructors[id](); \
    return NULL; \
  } \
  static className* CreateByObjType(className& object){ \
    return mConstructors[object.GetObjectType()](); \
  } \
  BASEOBJECT_DECL(className,className);


#define BASEOBJECT_DEF(className) \
  int className::mClassType = className::RegisterNewType(&className::BaseNew);

#define BASEOBJECT_TEMPLATEDEF( tmplinfo, className) \
  tmplinfo int className::mClassType = className::RegisterNewType(&className::BaseNew);

#define BASEOBJECT_TOPDEF(className) \
int className::mClassTypeCount = 0; \
std::vector<className* (*)()> className::mConstructors; \
BASEOBJECT_DEF(className);




#endif /*BASEOBJECT_H_*/
