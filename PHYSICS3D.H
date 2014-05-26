/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include <vector>

#if !defined(PHYSICS3D_H)
#define PHYSICS3D_H

#include "geo3d.h"	
#include "thinkGL.h"
#include "RIGIDBODY.H"

struct plane
{
    vector3D Normal;        
    real d;                 
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
const     int CurrentState = 0;
const     int TargetState  = 1;
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
	