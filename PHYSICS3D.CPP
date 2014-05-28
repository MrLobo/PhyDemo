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

int GravityActive = 1;
int FrictionActive = 1;
const float restitution=0.7f;
const float friction=0.3f;

vector3D Gravity(r(0),r(0),r(-10));
        
dynamic_world::~dynamic_world( void )
{
	ClearCollisions();
	delete currentRender;currentRender=0;
}

dynamic_world::dynamic_world(
    real XLength, real YLength, real ZLength )
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

}


void dynamic_world::BuildBodies( void )
{
	string String;

    for(int i = 0; i < NumberOfBodies;i++)
    {
      
      // initialize rigid bodies by randomly generating boxes
      rigid_body &Body = bodies[i];
		  
      //if( i%2 == 1 )

			 Body.shape=SPHERE_SHAPE;		
	
			 //Body.shape=BOX_SHAPE;
 

      real dX = GenerateUnitRandomReal()*1.0f;
		  real dY = GenerateUnitRandomReal()*1.0f;
		  real dZ = GenerateUnitRandomReal()*1.0f;
		  dX = VALUE_MAX(dX,0.4f);
		  dY = VALUE_MAX(dY,0.4f);
		  dZ = VALUE_MAX(dZ,0.4f);

		  if( Body.shape==BOX_SHAPE)
		  {
			  Body.size= vector3D(dX,dY,dZ);
			  Body.radius = sqrt(dX*dX+dY*dY+dZ*dZ);

			  real Density = r(1.0);
			  real Mass = r(8)*Density*dX*dY*dZ;
			  assert(Mass > r(0));

			  Body.setMass(Mass);
			  //factor 3 becouse "size" is half of the whole size - porque estamos usando la mitad del lado
			  Body.InverseBodyInertiaTensor(0,0) = r(3)/(Mass*(dY*dY+dZ*dZ));
			  Body.InverseBodyInertiaTensor(1,1) = r(3)/(Mass*(dX*dX+dZ*dZ));
			  Body.InverseBodyInertiaTensor(2,2) = r(3)/(Mass*(dX*dX+dY*dY));

			  Body.restitution =restitution;
			  Body.friction = friction;

			  Body.color[0]=GenerateUnitRandomReal();
			  Body.color[1]=GenerateUnitRandomReal();
			  Body.color[2]=GenerateUnitRandomReal();
			  currentRender->StoreBox(i, Body.color, dX,dY,dZ);
		  }
      //TODO: 
		  else if( Body.shape == SPHERE_SHAPE)
		  {
        Body.size = vector3D(dX,dY,dZ);
			  real radius = sqrt(dX*dX+dY*dY+dZ*dZ);
        Body.radius = radius;

			  real Density = r(1.6f);
			  real Mass =  Density * 4.f/3.f * PI * radius * radius * radius ;
			  assert(Mass > r(0));

        Body.setMass(Mass);
        Body.InverseBodyInertiaTensor(0,0) = r(5) / ( r(2) * Mass * radius * radius );
			  Body.InverseBodyInertiaTensor(1,1) = r(5) / ( r(2) * Mass * radius * radius );
			  Body.InverseBodyInertiaTensor(2,2) = r(5) / ( r(2) * Mass * radius * radius );

        Body.restitution =restitution;
			  Body.friction = friction;

        Body.color[0]=GenerateUnitRandomReal();
			  Body.color[1]=GenerateUnitRandomReal();
			  Body.color[2]=GenerateUnitRandomReal();
		  }
   }

}
void  dynamic_world::IntitilizeBodiesInWorld()
{
	float maxSpeed=25.0f;
//	CurrentState=0;
//	TargetState=1;
    for(int i = 0;i < NumberOfBodies;i++)
    {
      rigid_body::DynamicState &Source = bodies[i].dynamicState[CurrentState];
      rigid_body::DynamicState &Target = bodies[i].dynamicState[TargetState];

		  Source.CMForce = vector3D();
		  Source.Torque = vector3D();
		  Target.CMForce = vector3D();
		  Target.Torque = vector3D();

		  //assign random position
		  real px = (GenerateUnitRandomReal()-0.5f)*WorldSize;
		  real py = (GenerateUnitRandomReal()-0.5f)*WorldSize;
		  real pz = (GenerateUnitRandomReal()-0.5f)*WorldSize;

 /*     real px = 0.f;
		  real py = 0.3f;
		  real pz = 0.f;*/

		  //if( i==0)
		  //{
		  //	px = 0.0f;
		  //	py = 0.0f;
		  //	pz = -WorldSize*.45f;
		  //}
		  //else
		  //if( i==1)
		  //{
		  //	px = -0.2f;
		  //	py = -0.2f;
		  //	pz = -WorldSize*.05f;
		  //}	

		  vector3D position(px,py,pz);
		  Source.CMPosition=position;
		  Target.CMPosition=position;

		  //assign random velocities
		  real vx =(GenerateUnitRandomReal()-0.5f)*maxSpeed;
		  real vy =(GenerateUnitRandomReal()-0.5f)*maxSpeed;
		  real vz =(GenerateUnitRandomReal()-0.5f)*maxSpeed;

    //  real vx = 0.f;
		  //real vy = 0.f;
		  //real vz = 0.f;

		  //vx = -5.0f;
		  //vy = -5.0f;
		  //vz = 0.0f;//maxSpeed*.25f;

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
	currentRender->clearBoxTransformList();
	currentRender->clearSphereTransformList();
    for(int i=0; i < NumberOfBodies;i++)
    {
        matrix3D& Orientation = bodies[i].dynamicState[CurrentState].Orientation;
        vector3D& CMPosition = bodies[i].dynamicState[CurrentState].CMPosition;

		thkTransform myGlTransform;
        BuildOpenGLTransformMatrix(Orientation,CMPosition,myGlTransform.aM);
		if( bodies[i].shape==BOX_SHAPE)
		{
			currentRender->addBoxTransform( myGlTransform);
		}
		else
		if( bodies[i].shape==SPHERE_SHAPE)
		{
			myGlTransform.radius=bodies[i].radius;
			myGlTransform.color[0]=bodies[i].color[0];
			myGlTransform.color[1]=bodies[i].color[1];
			myGlTransform.color[2]=bodies[i].color[2];
			currentRender->addSphereTransform( myGlTransform);
//			currentRender->addBoxTransform( myGlTransform);
		}
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

void dynamic_world::ApplyForces( int state )
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

        Target.CMPosition = Source.CMPosition +
                DeltaTime * Source.CMVelocity;

        Target.CMVelocity = Source.CMVelocity +
                (DeltaTime * bodies[i].getInvMass()) * Source.CMForce;
		Target.CMVelocity *=(1.0f - airFriction);

        Target.Orientation = Source.Orientation +
                DeltaTime *
                matrix3D(Source.AngularVelocity,matrix3D::SkewSymmetric) *
                Source.Orientation;

		OrthonormalizeOrientation(Target.Orientation);

        Target.InverseWorldInertiaTensor = Target.Orientation *
                bodies[i].InverseBodyInertiaTensor *
                Transpose(Target.Orientation);

        Target.AngularMomentum = Source.AngularMomentum + DeltaTime * Source.Torque;
		Target.AngularMomentum *=(1.0f - airFriction);

        Target.AngularVelocity = Target.InverseWorldInertiaTensor *
                Target.AngularMomentum;

		if( airFriction > Epsilon)
		{
			while( Target.AngularVelocity.squaredLength() > 1000.0f)
			{
				//damp it until acceptable
				Target.AngularMomentum *=(1.0f - airFriction);
				Target.AngularVelocity = Target.InverseWorldInertiaTensor *
					Target.AngularMomentum;
			}
		}
    }
}

dynamic_world::collision_state dynamic_world::CheckForWallCollisions( int state )
{
  
  float const DepthEpsilon = Epsilon*100.0f;//0.001f;

	for(int WallIndex = 0;(WallIndex < 6);WallIndex++)
	{
		plane &Wall = walls[WallIndex];

		for(int BodyIndex = 0;(BodyIndex < NumberOfBodies) ;BodyIndex++)
		{
			rigid_body &Body = bodies[BodyIndex];
			rigid_body::DynamicState &dynamicState =
					Body.dynamicState[state];

			if( Body.shape==BOX_SHAPE)		
			{
				vector3D worldBoundingVertices[8];
				Bbox bbox(dynamicState.Orientation,dynamicState.CMPosition,Body.size);
				//a partir de los vertices de la caja, rellenamos el array de vertices de la caja en coordenadas del mundo
        box2vertex( bbox, worldBoundingVertices);

				for(int unsigned i= 0;(i < 8) ;i++)
				{			
					vector3D Position = worldBoundingVertices[i];
					vector3D R = Position - dynamicState.CMPosition;

          //velocidad de cada vertice, velocidad del centro de masa + cross 
					vector3D Velocity = dynamicState.CMVelocity +
								CrossProduct(dynamicState.AngularVelocity,R);

          // computamos la colision
					real penetration = Dot(Position,Wall.Normal) + Wall.d;
					if(penetration < DepthEpsilon)
					{
            // sólo hay colision si la velocidad normal relativa es negativa
						real RelativeVelocity = Dot(Wall.Normal,Velocity);
						if(RelativeVelocity < Epsilon)
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
      else if( Body.shape==SPHERE_SHAPE)
      {
        vector3D sphereCMPos = dynamicState.CMPosition;
        real penetration = Dot(sphereCMPos, Wall.Normal) + Wall.d - Body.radius;

        if(penetration < DepthEpsilon)
        {
          vector3D colPos = sphereCMPos - ( Wall.Normal * ( Body.radius - penetration ) );
          vector3D R = colPos - sphereCMPos;
          vector3D Velocity = dynamicState.CMVelocity + CrossProduct(dynamicState.AngularVelocity, R);

          real RelativeVelocity = Dot(Wall.Normal, Velocity);
          if(RelativeVelocity < Epsilon)
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
							pair->point[0] = colPos;
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

dynamic_world::collision_state dynamic_world::CheckForInterCollisions( int state )
{
    for(int BodyIndex1 = 0;(BodyIndex1 < NumberOfBodies) ;BodyIndex1++)
    {
        rigid_body &Body1 = bodies[BodyIndex1];
        rigid_body::DynamicState &dynamicState1 =
                Body1.dynamicState[state];
		Bbox bbox1(dynamicState1.Orientation,dynamicState1.CMPosition,Body1.size);

	    for(int BodyIndex2 = BodyIndex1 + 1 ;(BodyIndex2 < NumberOfBodies) ;BodyIndex2++)
		{
			rigid_body &Body2 = bodies[BodyIndex2];
			rigid_body::DynamicState &dynamicState2 =
					Body2.dynamicState[state];
			Bbox bbox2(dynamicState2.Orientation,dynamicState2.CMPosition,Body2.size);

			if( Body1.shape==BOX_SHAPE && Body2.shape==BOX_SHAPE)		
			{
				//box detection
				if( boxTestBox(bbox1, bbox2))
				{
					int nPoints=0;
					vector3D col_point[16];
					vector3D col_normal[16];
					if( boxFindBox( bbox1, bbox2, col_point, col_normal, nPoints))
					{
						CollisionState = Colliding;
						collision_pair* pair = new collision_pair;
						if( pair)
						{
							pair->body1Index=BodyIndex1;
							pair->body2Index=BodyIndex2;
							int num = VALUE_MIN(nPoints, 16);
							for( int j=0; j < num; j++)
							{
								pair->normal[j] = col_normal[j];
								pair->point[j] = col_point[j];
							}
							pair->nPoints=nPoints;
							pair->wall_Index = -1;
						}
						activePairs.push_back(pair);
	//					currentRender->bDrawCollisionInfo=true;
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

	unsigned int numCollisions= activePairs.size();
    for(size_t i = 0; i < numCollisions; ++i)
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
			real ImpulseNumerator = -(r(1) + restitution) *
				Dot(Velocity,normal);
			real ImpulseDenominator = Body.getInvMass() +
				Dot(CrossProduct(dynamicState.InverseWorldInertiaTensor *
									CrossProduct(R,normal),R),normal);

			real impn =  (ImpulseNumerator/ImpulseDenominator);
      //si el impulso es negativo lo eliminamos
			if( impn < Epsilon)
			{
				//sticky impulse
				continue;
			}

			vector3D Impulse = impn * normal;

			// apply impulse to primary quantities
			dynamicState.CMVelocity += Body.getInvMass() * Impulse;
			dynamicState.AngularMomentum += CrossProduct(R, Impulse);
    
			// compute affected auxiliary quantities
			dynamicState.AngularVelocity = dynamicState.InverseWorldInertiaTensor *
				dynamicState.AngularMomentum;

      //impulso en tangencial
			if( FrictionActive)
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

          //si otra colision ya hubiera modificado las velocidades 
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
		//			real impn = -((r(1)+CoefficientOfRestitution)*vn)/rIrn1;
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

          //desactivida la friccion entre los objetos para que no se interpenetren por el solver
					if(0)// FrictionActive)//removed becouse increase interpenetrations
					{
						// apply friction			
						vector3D Velocity1 = dynamicState1.CMVelocity +
							CrossProduct(dynamicState1.AngularVelocity,R1);
						vector3D Velocity2 = dynamicState2.CMVelocity +
							CrossProduct(dynamicState2.AngularVelocity,R2);

						vector3D velocity = Velocity1 - Velocity2;

						vector3D tand = velocity - Dot(velocity,normal)*normal;
						tand.normalize();
						{
							real friction =  (body1.friction+  body2.friction)*.5f;
							Impulse = friction *(-impn) * tand;

							// apply impulse to primary quantities
							dynamicState1.CMVelocity += body1.getInvMass() * Impulse;
							dynamicState1.AngularMomentum += CrossProduct(R1,Impulse);
    
							// compute affected auxiliary quantities
							dynamicState1.AngularVelocity = dynamicState1.InverseWorldInertiaTensor *
								dynamicState1.AngularMomentum;
							dynamicState2.CMVelocity -= body2.getInvMass() * Impulse;
							dynamicState2.AngularMomentum -= CrossProduct(R2,Impulse);
    
							// compute affected auxiliary quantities
							dynamicState2.AngularVelocity = dynamicState2.InverseWorldInertiaTensor *
								dynamicState2.AngularMomentum;
						}
					}

				}
				int solvedCounter=0;
        //iteramos por cada punto de la colision -> colisiones multiples de forma secuencial
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

void dynamic_world::Step( real DeltaTime )
{
  real CurrentTime = r(0);
  real TargetTime = DeltaTime;

  //if( bDrawCollisionInfo)
  //{
  //	assert(0);
  //}
	  //move bodies
  ApplyForces(CurrentState);
  Integrate(TargetTime-CurrentTime);

	//manage collisions 
  // clean collision list
	ClearCollisions();

  CheckForWallCollisions(TargetState);
  CheckForInterCollisions(TargetState);
  // either colliding or clear
  if( CollisionState == Colliding )
  {
      ResolveCollisions(TargetState);
  }
            
	UpdateCurrentState();
}


