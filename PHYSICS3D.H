/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include <vector>

#if !defined(PHYSICS3D_H)
#define PHYSICS3D_H

#include "geo3d.h"	
#include "thinkGL.h"

struct plane
{
    vector3D Normal;        
    real d;                 
};

const int MaxNumberOfBoundingVertices = 50;

struct rigid_body
{
	  vector3D size;
    real mass;
    matrix3D InverseBodyInertiaTensor;
    real restitution;
    real friction;	
	  bool frintionEnabled;

    int unsigned NumberOfBoundingVertices;
    vector3D localBoundingVertices[MaxNumberOfBoundingVertices];
    vector3D worldBoundingVertices[MaxNumberOfBoundingVertices];

	  void setMass(real value){ mass=value;}
	  real getMass(){ return mass;}
	  real getInvMass(){ return (1.0f/mass);}

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

struct collision_pair
{
	int body1Index;
	int body2Index;
	int wall_Index;
	int nPoints;
	vector3D point[16];
	vector3D normal[16];	
};


real const WorldSize = r(9);
const     int CurrentState=0;
const     int TargetState=1;
const     int NumberOfBodies = 16;

class dynamic_world
{
public:

    void BuildBodies( void );
	void IntitilizeBodiesInWorld( void );
    void InitializeRendering( void );
	void ResetBodiesPosition();
	void ClearCollisions( void );

	void Simulate( real DeltaTime );
    void Render( void );

	thinkGL* getRender(){ return currentRender;}

    dynamic_world( real XLength, real YLength, real ZLength );    
    ~dynamic_world( void );

private:


    enum collision_state
    {
        Clear,
        Colliding
    } CollisionState;	


    void ComputeForces( int state );
    void Integrate( real DeltaTime );
    void UpdateWorldTransform();
    void UpdateCurrentState();
    collision_state CheckForInterCollisions( int state );
    collision_state CheckForWallCollisions( int state );
    void ResolveCollisions( int ConfigurationIndex );

    rigid_body bodies[NumberOfBodies];
	plane walls[6];

    std::vector<collision_pair*> activePairs;

	thinkGL* currentRender;

};

#endif
	