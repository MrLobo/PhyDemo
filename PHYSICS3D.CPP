/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/


#include <windows.h>
#include <string>

#include <assert.h>
#include <stdlib.h>
#include <limits.h>
#include <time.h>

#include "physics3D.h"

using namespace std;

dynamic_world *pWorld = 0;

extern vector3D edge_p0,edge_p1;
extern vector3D edge_q0,edge_q1;
extern vector3D last_col_point[4];
extern vector3D last_col_normal[4];

float GetTime( void );

int GravityActive  = 1;
int FrictionActive = 0;
vector3D Gravity(r(0),r(0),r(-10));
        
dynamic_world::~dynamic_world( void )
{
	ClearCollisions();
	delete currentRender;currentRender=0;
}

dynamic_world::dynamic_world( real XLength, real YLength, real ZLength )
{
	  currentRender = new thinkGL();
	  if( !currentRender)return;
	  currentRender->setWordlSize(VALUE_MAX(ZLength, VALUE_MAX( XLength,YLength)));

    BuildBodies();
    IntitilizeBodiesInWorld();
	
    // initialize walls
    walls[0].Normal = vector3D(r(0),r(-1),r(0));
    walls[0].d = r(YLength/2);
    walls[1].Normal = vector3D(r(0),r(1),r(0));
    walls[1].d = r(YLength/2);

    walls[2].Normal = vector3D(r(-1),r(0),r(0));
    walls[2].d = r(XLength/2);
    walls[3].Normal = vector3D(r(1),r(0),r(0));
    walls[3].d = r(XLength/2);

    walls[4].Normal = vector3D(r(0),r(0),r(-1));
    walls[4].d = r(ZLength/2);
    walls[5].Normal = vector3D(r(0),r(0),r(1));
    walls[5].d = r(ZLength/2);

    UpdateWorldTransform();
}


void dynamic_world::BuildBodies( void )
{
	string String;

  for(int i = 0; i < NumberOfBodies; i++)
  {
      // initialize rigid bodies by randomly generating boxes
      rigid_body& Body = bodies[i];

		  real dX = GenerateUnitRandomReal()*1.0f;
		  real dY = GenerateUnitRandomReal()*1.0f;
		  real dZ = GenerateUnitRandomReal()*1.0f;

      dX = VALUE_MAX(dX,0.4f);
      dY = VALUE_MAX(dY,0.4f);
      dZ = VALUE_MAX(dZ,0.4f);
			
      
      Body.shape = new BoxShape( vector3D(dX,dY,dZ) );

      real Density = r(0.4);
      real Mass = r(8) * Density * Body.shape->calculateVolume();
      assert( Mass > r(0) );

      //Body.OneOverMass = r(1)/Mass;
		  Body.setMass(Mass);
      Body.InverseBodyInertiaTensor = Body.shape->calculateInverseBodyInertiaTensor( Mass );

      Body.restitution = r(0.5f); 
		  Body.friction = r(0.5f);

      // Body.adynamicStates[0].CMPosition;
      
      // assign the body-space bounding volume vertices
      Body.NumberOfBoundingVertices = 8;
      Body.localBoundingVertices[0] = vector3D( dX, dY, dZ);
      Body.localBoundingVertices[1] = vector3D( dX, dY,-dZ);
      Body.localBoundingVertices[2] = vector3D( dX,-dY, dZ);
      Body.localBoundingVertices[3] = vector3D( dX,-dY,-dZ);
      Body.localBoundingVertices[4] = vector3D(-dX, dY, dZ);
      Body.localBoundingVertices[5] = vector3D(-dX, dY,-dZ);
      Body.localBoundingVertices[6] = vector3D(-dX,-dY, dZ);
      Body.localBoundingVertices[7] = vector3D(-dX,-dY,-dZ);

		  float color[3];
		  color[0] = GenerateUnitRandomReal();
      color[1] = GenerateUnitRandomReal();
      color[2] = GenerateUnitRandomReal();
		  currentRender->StoreBox(i, color, dX, dY, dZ);
  }
}

void  dynamic_world::IntitilizeBodiesInWorld()
{
	  float maxSpeed=25.0f;
    //CurrentState=0;
    //TargetState=1;
    for(int i = 0;i < NumberOfBodies; i++)
    {

        rigid_body::DynamicState &Source = bodies[i].dynamicState[CurrentState];
        rigid_body::DynamicState &Target = bodies[i].dynamicState[TargetState];

		    Source.CMForce = vector3D();
		    Source.Torque = vector3D();
		    Target.CMForce = vector3D();
		    Target.Torque = vector3D();

		    //assign random position
		    real px = (GenerateUnitRandomReal()-0.5f) * WorldSize;
		    real py = (GenerateUnitRandomReal()-0.5f) * WorldSize;
		    real pz = (GenerateUnitRandomReal()-0.5f) * WorldSize;
		    vector3D position(px,py,pz);
		    Source.CMPosition=position;
		    Target.CMPosition=position;

		    //assign random velocities
		    real vx = (GenerateUnitRandomReal()-0.5f)*maxSpeed;
		    real vy = (GenerateUnitRandomReal()-0.5f)*maxSpeed;
		    real vz = (GenerateUnitRandomReal()-0.5f)*maxSpeed;

		    vector3D velocity(vx,vy,vz);
		    Source.CMVelocity=velocity;
		    Target.CMVelocity=velocity;

		    vector3D w(0,0,0);
		    Source.AngularMomentum = w;
		    Source.AngularVelocity = w;
		    Target.AngularMomentum = w;
		    Target.AngularVelocity = w;

		    Source.Orientation.setIdentity();
		    Target.Orientation.setIdentity();

        Source.InverseWorldInertiaTensor = Source.Orientation *
                bodies[i].InverseBodyInertiaTensor *
                Transpose(Source.Orientation);

        Target.InverseWorldInertiaTensor = Target.Orientation *
                bodies[i].InverseBodyInertiaTensor *
                Transpose(Target.Orientation);
	  }
}

void dynamic_world::Render( void )
{
	currentRender->clearTransformList();
  for(int i=0; i < NumberOfBodies;i++)
  {
    matrix3D& Orientation = bodies[i].dynamicState[CurrentState].Orientation;
    vector3D& CMPosition = bodies[i].dynamicState[CurrentState].CMPosition;

		thkTransform myGlTransform;
        BuildOpenGLTransformMatrix(Orientation,CMPosition,myGlTransform.aM);
		currentRender->addTransform( myGlTransform);
	}

	if( currentRender->bDrawCollisionInfo)
	{
		float color[3];
		//col edges
		color[0]=1.0f;color[1]=0.0f;color[2]=0.0f;
		currentRender->clearLinesList();
		currentRender->addLine(edge_p0.getArray(), edge_p1.getArray(), color);
		currentRender->addLine(edge_q0.getArray(), edge_q1.getArray(), color);
		//col normal
		color[0]=0.0f;color[1]=1.0f;color[2]=0.0f;
		vector3D end_point = last_col_normal[0] + last_col_point[0];
		currentRender->addLine(last_col_point[0].getArray(), end_point.getArray(), color);
	}
	currentRender->Render();
}

void dynamic_world::ClearCollisions( void )
{
  for(size_t i = 0; i < activePairs.size(); ++i)
  {
    collision_pair* pair = activePairs[i];
	  delete pair;
  }
	activePairs.clear();	
	CollisionState = Clear;
	currentRender->bDrawCollisionInfo=true;
	currentRender->clearDrawInfo();
}

void dynamic_world::ComputeForces( int state )
{
    for(int i = 0;i < NumberOfBodies;i++)
    {
      rigid_body &Body = bodies[i];
      rigid_body::DynamicState &dynState = Body.dynamicState[state];

      // clear forces

      dynState.Torque = vector3D();
      dynState.CMForce = vector3D();

      if(GravityActive)
      {
          dynState.CMForce += Gravity / Body.getInvMass(); 
      }
    }
}   

void dynamic_world::UpdateWorldTransform()
{
    for(int i = 0;i < NumberOfBodies;i++)
    {
        rigid_body &Body = bodies[i];
        rigid_body::DynamicState &dynamicState0 = Body.dynamicState[CurrentState];
        rigid_body::DynamicState &dynamicState1 = Body.dynamicState[TargetState];
            
        matrix3D const &A = dynamicState1.Orientation;
        vector3D const &R = dynamicState1.CMPosition;

        assert(Body.NumberOfBoundingVertices < MaxNumberOfBoundingVertices);
        for(int unsigned j = 0;j < Body.NumberOfBoundingVertices;j++)
        {
            Body.worldBoundingVertices[j] =
                R + A * Body.localBoundingVertices[j];
        }

    }
}

void dynamic_world::UpdateCurrentState()
{
    for(int i = 0;i < NumberOfBodies;i++)
    {
        rigid_body &Body = bodies[i];
        rigid_body::DynamicState &dynamicState0 = Body.dynamicState[CurrentState];
        rigid_body::DynamicState &dynamicState1 = Body.dynamicState[TargetState];

		dynamicState0.AngularMomentum= dynamicState1.AngularMomentum;
		dynamicState0.AngularVelocity= dynamicState1.AngularVelocity;
		dynamicState0.CMPosition = dynamicState1.CMPosition;
		dynamicState0.CMVelocity = dynamicState1.CMVelocity;
		dynamicState0.InverseWorldInertiaTensor = dynamicState1.InverseWorldInertiaTensor;
		dynamicState0.Orientation = dynamicState1.Orientation;
		dynamicState0.Torque = dynamicState1.Torque;
	}
}

void dynamic_world::Integrate( real DeltaTime )
{
	const real airFriction=0.0005f;

    for(int i= 0;i < NumberOfBodies;i++)
    {
        rigid_body::DynamicState &Source =bodies[i].dynamicState[CurrentState];
        rigid_body::DynamicState &Target =bodies[i].dynamicState[TargetState];

        Target.CMVelocity = Source.CMVelocity +
                (DeltaTime * bodies[i].getInvMass()) * Source.CMForce;
		    Target.CMVelocity *=(1.0f - airFriction);

        Target.CMPosition = Source.CMPosition +
                DeltaTime * Source.CMVelocity;

        Target.AngularMomentum = Source.AngularMomentum + DeltaTime * Source.Torque;
		    Target.AngularMomentum *=(1.0f - airFriction);

        Target.Orientation = Source.Orientation +
                DeltaTime *
                matrix3D(Source.AngularVelocity,matrix3D::SkewSymmetric) *
                Source.Orientation;

		    OrthonormalizeOrientation(Target.Orientation);

        Target.InverseWorldInertiaTensor = Target.Orientation *
                bodies[i].InverseBodyInertiaTensor *
                Transpose(Target.Orientation);

        Target.AngularVelocity = Target.InverseWorldInertiaTensor *
                Target.AngularMomentum;

		    while( Target.AngularVelocity.squaredLength() > 1000.0f)
		    {
			    //damp it until acceptable
			    Target.AngularMomentum *=(1.0f - airFriction);
	            Target.AngularVelocity = Target.InverseWorldInertiaTensor *
                    Target.AngularMomentum;
		    }

    }
}


dynamic_world::collision_state dynamic_world::CheckForInterCollisions( int state )
{
    for( int BodyIndex1 = 0;(BodyIndex1 < NumberOfBodies) ;BodyIndex1++ )
    {

      rigid_body &Body1 = bodies[BodyIndex1];
      rigid_body::DynamicState &dynamicState1 = Body1.dynamicState[state];
		  Bbox bbox1( dynamicState1.Orientation, dynamicState1.CMPosition, Body1.shape->getSize() );

	    for( int BodyIndex2 = BodyIndex1 + 1 ; (BodyIndex2 < NumberOfBodies); BodyIndex2++ )
		  {
			  rigid_body &Body2 = bodies[BodyIndex2];
			  rigid_body::DynamicState &dynamicState2 = Body2.dynamicState[state];
			  Bbox bbox2( dynamicState2.Orientation, dynamicState2.CMPosition, Body2.shape->getSize() );

			  if( boxTestBox(bbox1, bbox2) )
			  {
				  int nPoints=0;
				  vector3D col_point[16];
				  vector3D col_normal[16];
				  if( boxFindBox( bbox1, bbox2, col_point, col_normal, nPoints))
				  {
					  collision_pair* pair = new collision_pair;
					  if( pair)
					  {
						  pair->body1Index=BodyIndex1;
						  pair->body2Index=BodyIndex2;
						  for( int j=0; j <nPoints; j++)
						  {
							  pair->normal[j] = col_normal[j];
							  pair->point[j] = col_point[j];
						  }
						  pair->nPoints=nPoints;
						  pair->wall_Index = -1;
					  }
					  activePairs.push_back(pair);
					  CollisionState = Colliding;
					  Body1.frintionEnabled=false;
					  Body2.frintionEnabled=false;	
  			    //currentRender->bDrawCollisionInfo=true;
				  }
			  }
		}
	}
	return CollisionState;
}

dynamic_world::collision_state dynamic_world::CheckForWallCollisions( int state )
{
    float const DepthEpsilon = 0.001f;

    for( int BodyIndex = 0; (BodyIndex < NumberOfBodies); BodyIndex++ )
    {
        rigid_body &Body = bodies[BodyIndex];
        rigid_body::DynamicState &dynamicState = Body.dynamicState[state];
		    Body.frintionEnabled=true;

        for( unsigned int i = 0; i < Body.NumberOfBoundingVertices; i++)
        {
            vector3D Position = Body.worldBoundingVertices[i];
            vector3D U = Position - dynamicState.CMPosition;
            vector3D Velocity = dynamicState.CMVelocity +
                        CrossProduct(dynamicState.AngularVelocity,U);

            for( int WallIndex = 0; (WallIndex < 6); WallIndex++ )
            {
                plane& Wall = walls[WallIndex];

                real penetration = Dot(Position,Wall.Normal) + Wall.d;

                if( penetration < DepthEpsilon )
                {
                    real RelativeVelocity = Dot(Wall.Normal,Velocity);

                    if( RelativeVelocity < r(0) )
                    {
                        CollisionState = Colliding;
						
						            //prevent interpenetration with ground
						            if( walls[WallIndex].Normal.getZ() > 0.99f)
							            dynamicState.CMPosition-=penetration*walls[WallIndex].Normal; 

						            collision_pair* pair = new collision_pair;
						            if( pair)
						            {
							            pair->body1Index=BodyIndex;
							            pair->body2Index=-1;
							            pair->normal[0] = walls[WallIndex].Normal;
							            pair->point[0] = Position;
							            pair->nPoints=1;
							            pair->wall_Index = WallIndex;
						            }
						            activePairs.push_back(pair);
                    }
                }
            }
        }
    }

    return CollisionState;
}


void dynamic_world::ResolveCollisions( int dynamicStateIndex )
{
	//NOTE: there can be interpenetrions in the following cases
	// very fast moving/rotating objects
	// large bodies fall over smaller bodies
	// multiple bodies involved in a collsion
	// multiple bodies and walls involved in a collision
	// iterations arent enough to clear collisions

    for(size_t i = 0; i < activePairs.size(); ++i)
    {
        collision_pair* pair = activePairs[i];
		if( pair->wall_Index >= 0)
		{
			//resolve wall collision
			rigid_body &Body = bodies[pair->body1Index];
			vector3D normal = pair->normal[0];//walls[pair->wall_Index].Normal;
			rigid_body::DynamicState &dynamicState = Body.dynamicState[dynamicStateIndex];

			vector3D Position = pair->point[0];//dynamicState.aBoundingVertices[pair->vertex1Index];
			vector3D R = Position - dynamicState.CMPosition;
			vector3D Velocity = dynamicState.CMVelocity +
				CrossProduct(dynamicState.AngularVelocity,R);
			real ImpulseNumerator = -(r(1) + Body.restitution) *
				Dot(Velocity,normal);
			real ImpulseDenominator = Body.getInvMass() +
				Dot(CrossProduct(dynamicState.InverseWorldInertiaTensor *
									CrossProduct(R,normal),R),normal);

			real impn =  (ImpulseNumerator/ImpulseDenominator);
			if( impn < Epsilon)
			{
				//sticky impulse
				continue;
			}

			vector3D Impulse = impn * normal;

			// apply impulse to primary quantities
			dynamicState.CMVelocity += Body.getInvMass() * Impulse;
			dynamicState.AngularMomentum += CrossProduct(R,Impulse);
    
			// compute affected auxiliary quantities
			dynamicState.AngularVelocity = dynamicState.InverseWorldInertiaTensor *
				dynamicState.AngularMomentum;


			if( FrictionActive)// && Body.frintionEnabled)
			{
				// apply friction
			
				Velocity = dynamicState.CMVelocity +
					CrossProduct(dynamicState.AngularVelocity,R);

				vector3D tand = Velocity - Dot(Velocity,normal)*normal;
				tand.normalize();
				{
					Impulse = Body.friction *(-impn) * tand;

					// apply impulse to primary quantities
					dynamicState.CMVelocity += Body.getInvMass() * Impulse;
					dynamicState.AngularMomentum += CrossProduct(R,Impulse);
    
					// compute affected auxiliary quantities
					dynamicState.AngularVelocity = dynamicState.InverseWorldInertiaTensor *
						dynamicState.AngularMomentum;
				}
			}

		}
		else
		{
			//resolve bodies collision
			rigid_body &body1 = bodies[pair->body1Index];
			rigid_body &body2 = bodies[pair->body2Index];
			rigid_body::DynamicState &dynamicState1 = body1.dynamicState[dynamicStateIndex];
			rigid_body::DynamicState &dynamicState2 = body2.dynamicState[dynamicStateIndex];

			real restitution =  (body1.restitution + body2.restitution)*.5f;
			int ite =0;
			while( ite < 100)//max number of iteration allowed
			{
				for( int j=0; j < pair->nPoints; j++)
				{
					vector3D normal = pair->normal[j];//walls[pair->wall_Index].Normal;
					vector3D Position = pair->point[j];//dynamicState.aBoundingVertices[pair->vertex1Index];
					vector3D R1 = Position - dynamicState1.CMPosition;
					vector3D R2 = Position - dynamicState2.CMPosition;

					vector3D Velocity1 = dynamicState1.CMVelocity +
						CrossProduct(dynamicState1.AngularVelocity,R1);
					vector3D Velocity2 = dynamicState2.CMVelocity +
						CrossProduct(dynamicState2.AngularVelocity,R2);

					vector3D relative_velocity = Velocity1 - Velocity2;
		//		    vector3D relative_velocity = Velocity1;
					real vn = Dot( relative_velocity, normal);
					if( vn > Epsilon)
					{
						//false collision
						continue;
					}

					real rIrn1 = body1.getInvMass() +
						Dot( normal,CrossProduct( dynamicState1.InverseWorldInertiaTensor*
						CrossProduct(R1, normal), R1)); 
					real rIrn2 = body2.getInvMass() +
						Dot( normal,CrossProduct( dynamicState2.InverseWorldInertiaTensor*
						CrossProduct(R2, normal), R2)); 
					real impn = -((r(1)+restitution)*vn)/(rIrn1+rIrn2);
		//		real impn = -((r(1)+CoefficientOfRestitution)*vn)/rIrn1;
					if( impn < Epsilon)
					{
						//sticky impulse
						continue;
					}

					vector3D Impulse = impn*normal;

					dynamicState1.CMVelocity += body1.getInvMass() * Impulse;
					dynamicState1.AngularMomentum += CrossProduct(R1,Impulse);
    
					// compute affected auxiliary quantities
					dynamicState1.AngularVelocity = dynamicState1.InverseWorldInertiaTensor *
						dynamicState1.AngularMomentum;

					dynamicState2.CMVelocity -= body2.getInvMass() * Impulse;
					dynamicState2.AngularMomentum -= CrossProduct(R2,Impulse);
    
					 //compute affected auxiliary quantities
					dynamicState2.AngularVelocity = dynamicState2.InverseWorldInertiaTensor *
						dynamicState2.AngularMomentum;
				}
				int solvedCounter=0;
				for( int j=0; j < pair->nPoints; j++)
				{
					vector3D normal = pair->normal[j];//walls[pair->wall_Index].Normal;
					vector3D Position = pair->point[j];//dynamicState.aBoundingVertices[pair->vertex1Index];
					vector3D R1 = Position - dynamicState1.CMPosition;
					vector3D R2 = Position - dynamicState2.CMPosition;

					vector3D Velocity1 = dynamicState1.CMVelocity +
						CrossProduct(dynamicState1.AngularVelocity,R1);
					vector3D Velocity2 = dynamicState2.CMVelocity +
						CrossProduct(dynamicState2.AngularVelocity,R2);

					vector3D relative_velocity = Velocity1 - Velocity2;
		//		    vector3D relative_velocity = Velocity1;
					real vn = Dot( relative_velocity, normal);
					if( vn > Epsilon)
					{
						solvedCounter++;
					}
				}
				if( solvedCounter >=pair->nPoints)
				{
					break;
				}
				else
				{
					ite++;
//					assert( ite < 2);
				}
			}
		}
	}
}

void dynamic_world::Simulate( real DeltaTime )
{
  real CurrentTime = r(0);
  real TargetTime = DeltaTime;

	//move bodies
  ComputeForces(CurrentState);
  Integrate(TargetTime-CurrentTime);
  UpdateWorldTransform();

	//manage collisions
	ClearCollisions();

  CheckForWallCollisions(TargetState);
  CheckForInterCollisions(TargetState);
  // either colliding or clear
  if(CollisionState == Colliding )
  {
      ResolveCollisions(TargetState);
  }
            
	UpdateCurrentState();
}


