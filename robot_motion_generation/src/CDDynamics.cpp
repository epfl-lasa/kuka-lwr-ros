/*
 * CDDynamics.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Seungsu KIM
 */

/*
 * how to use

	CDDynamics *testDyn;
	testDyn = new CDDynamics(dim, dt, wn);

	testDyn->SetVelocityLimits(velLimits);
	testDyn->SetState(initial);
	testDyn->SetTarget(target);


	start loop
		// if new target is set
		testDyn->SetTarget(target);

		// update dynamics
		testDyn->Update();

		// get state
		testDyn->GetState(state);
	end loop
 */

#include "robot_motion_generation/CDDynamics.h"
#include <iostream>
namespace motion{

CDDynamics::CDDynamics(int dim, double dt, double Wn)
{
	// set environment
	mDim = dim;
	mDT = dt;
	mWn = Wn;

	// resize variables
    mTarget.resize(mDim);
    mTargetVelocity.resize(mDim);
    mState.resize(mDim);
    mStateVelocity.resize(mDim);
    mPositionLimits.resize(mDim);
    mVelocityLimits.resize(mDim);

	// set initial values
    mState.setZero();
    mStateVelocity.setZero();
    mTarget.setZero();
    mTargetVelocity.setZero();

    mPositionLimits.setZero();
    mVelocityLimits.setZero();
}

void CDDynamics::SetState(const Vector & Position)
{
    if(mDim == Position.size()){
		mState = Position - mTarget;
	}
	else{
        std::cout<<"Dimension error! @ CDDynamics::SetState() "<< std::endl;
	}
}

void CDDynamics::SetState(const Vector & Position, const Vector & Velocity)
{
    if( (mDim == Position.size()) && (mDim == Velocity.size())){
		mState = Position - mTarget;
		mStateVelocity = Velocity - mTargetVelocity;
	}
	else{
        std::cout<<"Dimension error! @ CDDynamics::SetState() "<< std::endl;
	}
}

void CDDynamics::SetTarget(const Vector & target)
{
    if(mDim == target.size()){
		mState += (mTarget-target);
		mTarget = target;
	}
	else{
        std::cout<<"Dimension error! @ CDDynamics::SetTarget() "<< std::endl;
	}
}

void CDDynamics::SetStateTarget(const Vector & Position, const Vector & Target)
{
	SetState(Position);
	SetTarget(Target);
}

/*
void CDDynamics::SetTarget(const Vector & target, const Vector & targetVel)
{
	if( (mDim == target.Size()) && (mDim == targetVel.Size())){
		mState += (mTarget-target);
		mStateVelocity += (mTargetVelocity-targetVel);

		mTarget = target;
		mTargetVelocity = targetVel;
	}
	else{
		cout<<"Dimension error! @ CDDynamics::SetTarget() "<<endl;
	}
}
*/

void CDDynamics::SetDt(double dt)
{
	mDT = dt;
}

void CDDynamics::SetWn(double Wn)
{
	mWn = Wn;
}

void CDDynamics::SetVelocityLimits(const Vector & velLimits)
{
    if(mDim == velLimits.size()){
		mVelocityLimits = velLimits;
	}
	else{
        std::cout<<"Dimension error! @ CDDynamics::SetVelocityLimits() "<< std::endl;
	}
}

void CDDynamics::RemoveVelocityLimits(void)
{
    mVelocityLimits.setZero();
}

void CDDynamics::SetPositionLimits(const Vector & posLimits)
{
    if(mDim == posLimits.size()){
		mPositionLimits = posLimits;
	}
	else{
        std::cout<<"Dimension error! @ CDDynamics::SetPositionLimits() "<< std::endl;
	}
}

void CDDynamics::RemovePositionLimits(void)
{
    mPositionLimits.setZero();
}

void CDDynamics::GetTarget(Vector & target)
{
	target = mTarget;
}
/*
void CDDynamics::GetTarget(Vector & target, Vector & targetVel)
{
	target = mTarget;
	targetVel = mTargetVelocity;
}
*/
void CDDynamics::GetState(Vector & Position)
{
	Position = mState + mTarget;
}

void CDDynamics::GetState(Vector & Position, Vector & Velocity)
{
	Position = mState + mTarget;
	Velocity = mStateVelocity + mTargetVelocity;
}

void CDDynamics::Update()
{
	Update(mDT, 1.0);
}

void CDDynamics::Update(double dt)
{
	Update(dt, 1.0);
}

void CDDynamics::Update(double dt, double muxVel)
{
	// A      = x(0);
	// B      = x_d(0) + w*x(0)
	// x(t)   = (A+Bt)e^(-w*t)
	// x_d(t) = (-w*A+(1-w*t)B ) e^(-w*t)

	double B;
	double x;
	for(unsigned int i=0; i<mDim; i++ ){
		x = mState(i);
		B = mStateVelocity(i)+mWn*x;

		//mStateVelocity(i) = ( -mWn*mState(i) + (1.0-mWn*dt*muxVel)*B )* exp(-mWn*dt*muxVel);
		//mState(i)         = ( mState(i)+B*dt*muxVel )*exp(-mWn*dt*muxVel);
		mState(i)         = x+mStateVelocity(i)*dt;
		mStateVelocity(i) = ( -mWn*x + (1.0-mWn*dt)*B )* exp(-mWn*dt) *muxVel;

		if( mPositionLimits(i)>0){
			if     ( mState(i) >  mPositionLimits(i) ) mState(i) =  mPositionLimits(i);
			else if( mState(i) < -mPositionLimits(i) ) mState(i) = -mPositionLimits(i);
		}

		if( mVelocityLimits(i)>0){
			if     ( mStateVelocity(i) >  mVelocityLimits(i) ) mStateVelocity(i) =  mVelocityLimits(i);
			else if( mStateVelocity(i) < -mVelocityLimits(i) ) mStateVelocity(i) = -mVelocityLimits(i);
		}
	}
}

}
