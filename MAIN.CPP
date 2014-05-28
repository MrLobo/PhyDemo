/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include <windows.h>

#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <mmsystem.h>

#include "geo3d.h"
#include "physics3D.h"


extern int unsigned WindowWidth;
extern int unsigned WindowHeight;
extern real const WorldSize;
extern int GravityActive;
extern int FrictionActive;
extern dynamic_world *pWorld;

float GetTime( void )
{
#if 0   
    static clock_t StartTime = clock();
    assert(StartTime != (clock_t)(-1));
    return ((float)(clock()-StartTime))/((float)CLOCKS_PER_SEC);
#else
    static DWORD StartMilliseconds;
    if(!StartMilliseconds)
    {
        StartMilliseconds = timeGetTime();
    }

    DWORD CurrentMilliseconds = timeGetTime();
    return float(CurrentMilliseconds - StartMilliseconds) / 1000.0f;
#endif  
}


void Initialize( void )
{
    srand(time(0));

	//init world
    pWorld = new dynamic_world(WorldSize,WorldSize,WorldSize);
    assert(pWorld);
	//init render
	pWorld->getRender()->Init(WindowWidth, WindowHeight);
}

void Execute( float zoom,  float height, float angle )
{
    assert(pWorld);
	pWorld->getRender()->clearView(zoom,height,angle);

    static real LastTime = GetTime();

#define FIXEDTIMESTEP 0
    real const MaxTimeStep = r(0.007);
    
#if FIXEDTIMESTEP
    real Time = LastTime + MaxTimeStep;
#else
    real Time = GetTime();
#endif

    while(LastTime < Time)
    {
        real DeltaTime = Time - LastTime;
        if(DeltaTime > MaxTimeStep)
        {
            DeltaTime = MaxTimeStep;
        }
        
        pWorld->Step(DeltaTime);
        LastTime += DeltaTime;
    }
    LastTime = Time;

    pWorld->Render();
}

void Reset( void )
{
	pWorld->ClearCollisions();
	pWorld->IntitilizeBodiesInWorld();		
}


void Destroy( void )
{
    delete pWorld;
}

void ToggleGravity( void )
{
    GravityActive = GravityActive ? 0 : 1;
}

void ToggleFriction( void )
{
    FrictionActive = FrictionActive ? 0 : 1;
}

