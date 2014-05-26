#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_

#include "geo3d.h"	

class Shape
{
public:
  virtual vector3D& getSize() = 0;
  virtual real calculateVolume() = 0;
  virtual matrix3D calculateInverseBodyInertiaTensor(real mass) = 0;
};

class BoxShape : public Shape
{
  vector3D size;
public:
  BoxShape( const vector3D& v ) 
    : size(v)
  {
  }
  vector3D& getSize() { return size; }
  real calculateVolume() { return ( size.getX() * size.getY() * size.getZ() ); }
  matrix3D calculateInverseBodyInertiaTensor( real mass )
  {
    matrix3D inverseI;
    real dX = size.getX();
    real dY = size.getY();
    real dZ = size.getZ();
    inverseI(0,0) = r(3) / ( mass * (dY * dY + dZ * dZ) );
    inverseI(1,1) = r(3) / ( mass * (dX * dX + dZ * dZ) );
    inverseI(2,2) = r(3) / ( mass * (dX * dX + dY * dY) );
    
    return inverseI;
  }
};

class SphereShape : public Shape
{
  
  real radius;
public:

  SphereShape( const real& r) 
    : radius(r)
  {
  }
  vector3D& getSize()  { return vector3D( radius, 0 , 0); }
  real calculateVolume() { return ( real(4.f/3.f) * PI * radius * radius * radius ); }
  matrix3D calculateInverseBodyInertiaTensor( real mass )
  {
    matrix3D inverseI;
    inverseI(0,0) = r(5) / ( r(2) * mass * radius * radius );
    inverseI(1,1) = r(5) / ( r(2) * mass * radius * radius );
    inverseI(2,2) = r(5) / ( r(2) * mass * radius * radius );
    
    return inverseI;
  }

};

const int MaxNumberOfBoundingVertices = 9;

struct rigid_body
{
  Shape* shape;
  real mass;
  matrix3D InverseBodyInertiaTensor;
  real restitution;
  real friction;	
	bool frintionEnabled;

  int unsigned NumberOfBoundingVertices;
  vector3D localBoundingVertices[MaxNumberOfBoundingVertices];
  vector3D worldBoundingVertices[MaxNumberOfBoundingVertices];

	void setMass(real value){ mass=value; }
	real getMass(){ return mass;}
	real getInvMass(){ return (1.0f/mass); }

  struct DynamicState
  {
      vector3D CMPosition;
      matrix3D Orientation;

      vector3D CMVelocity;
      vector3D AngularMomentum;

      vector3D CMForce;
      vector3D Torque;

      matrix3D InverseWorldInertiaTensor;
      vector3D AngularVelocity;
        
  } dynamicState[2];
};


#endif