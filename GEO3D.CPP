/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include "geo3d.h"

//*** allow double to float conversion
#pragma warning ( disable : 4136 )
#pragma warning ( disable : 4244 )
//*** Turn on float conversion warning
#pragma warning ( default : 4136 )
#pragma warning ( disable : 4514 ) // unused inline function

#include <assert.h>
#include <math.h>
#include <stdlib.h>


bool bDrawCollisionInfo=false;
vector3D last_col_point[4];
vector3D last_col_normal[4];
vector3D edge_p0,edge_p1;
vector3D edge_q0,edge_q1;

matrix3D operator*( matrix3D const &Operand,
        matrix3D const &Multiplier )
{
    matrix3D Result;

    for(int i = 0;i < 3;i++)
    {
        for(int j = 0;j < 3;j++)
        {
            real Value = 0;
          
            for(int k = 0;k < 3;k++)
            {
                Value += Operand.GetElement(i,k) *
                        Multiplier.GetElement(k,j);
            }

            Result.SetElement(i,j,Value);
        }
    }

    return Result;
}

vector3D operator*( matrix3D const &Operand,
        vector3D const &Multiplier )
{
    vector3D Result;

    for(int i = 0;i < 3;i++)
    {
        real Value = 0;
      
        for(int k = 0;k < 3;k++)
        {
            Value += Operand.GetElement(i,k) *
                    Multiplier(k);
        }

        Result.set(i,Value);
    }

    return Result;
}


matrix3D::matrix3D( void )
{
    for(int Counter = 0;Counter < 9;Counter++)
    {
        element[0][Counter] = 0;
    }

    element[0][0] = element[1][1] = element[2][2] = 1;
}

void matrix3D::setIdentity()
{
    for(int Counter = 0;Counter < 9;Counter++)
    {
        element[0][Counter] = 0;
    }

    element[0][0] = element[1][1] = element[2][2] = 1;
}


real vector3D::length( void ) const
{
    return sqrt(getX()*getX() + getY()*getY() + getZ()*getZ());
}

real vector3D::squaredLength( void ) const
{
    return (getX()*getX() + getY()*getY() + getZ()*getZ());
}

vector3D &vector3D::normalize( void )
{
    real Length = length();

    if(Length != 0)
    {
        setX(getX() / Length);
        setY(getY() / Length);
        setZ(getZ() / Length);
    }

    return *this;
}
  
void OrthonormalizeOrientation( matrix3D &Orientation )
{
    vector3D X(Orientation(0,0),Orientation(1,0),Orientation(2,0));
    vector3D Y(Orientation(0,1),Orientation(1,1),Orientation(2,1));
    vector3D Z;

    X.normalize();
    Z = CrossProduct(X,Y).normalize();
    Y = CrossProduct(Z,X).normalize();

    Orientation(0,0) = X(0); Orientation(0,1) = Y(0); Orientation(0,2) = Z(0);
    Orientation(1,0) = X(1); Orientation(1,1) = Y(1); Orientation(1,2) = Z(1);
    Orientation(2,0) = X(2); Orientation(2,1) = Y(2); Orientation(2,2) = Z(2);
}



real GenerateUnitRandomReal( void )
{
    real Random = ((real)rand())/((real)RAND_MAX);
    assert((Random >= r(0)) && (Random <= r(1)));
    return Random;
}


real sqrDistanceSegs(const vector3D& p0, const vector3D& p1,
                    const vector3D& q0, const vector3D& q1,
                    real& pfSegP0, real& pfSegP1)
{
    vector3D dirP = p1 - p0;
    vector3D dirQ = q1 - q0; 
    vector3D kDiff = p0 - q0;
    real fA00 = dirP.squaredLength();
    real fA01 = -Dot( dirP, dirQ);
    real fA11 = dirQ.squaredLength();
    real fB0 = Dot( kDiff, dirP);
    real fC = kDiff.squaredLength();
    real fDet = fabs(fA00*fA11-fA01*fA01);
    real fB1, fS, fT, fSqrDist, fTmp;

    if ( fDet > Epsilon )
    {
        // line segments are not parallel
        fB1 = -Dot(kDiff, dirQ);
        fS = fA01*fB1-fA11*fB0;
        fT = fA01*fB0-fA00*fB1;
        
        if ( fS > (-Epsilon) )
        {
            if ( fS < fDet )
            {
                if ( fT >= (-Epsilon) )
                {
                    if ( fT < fDet )  // region 0 (interior)
                    {
                        // minimum at two interior points of 3D lines
                        real fInvDet = 1.0f/fDet;
                        fS *= fInvDet;
                        fT *= fInvDet;
                        fSqrDist = fS*(fA00*fS+fA01*fT+2.0f*fB0) +
                            fT*(fA01*fS+fA11*fT+2.0f*fB1)+fC;
                    }
                    else  // region 3 (side)
                    {
                        fT = 1.0f;
                        fTmp = fA01+fB0;
                        if ( fTmp > (-Epsilon) )
                        {
                            fS = 0.0f;
                            fSqrDist = fA11+2.0f*fB1+fC;
                        }
                        else if ( (-fTmp) > fA00 )
                        {
                            fS = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
                        }
                        else
                        {
                            fS = -fTmp/fA00;
                            fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
                        }
                    }
                }
                else  // region 7 (side)
                {
                    fT = 0.0f;
                    if ( fB0 > (-Epsilon) )
                    {
                        fS = 0.0f;
                        fSqrDist = fC;
                    }
                    else if ( (-fB0) >= fA00 )
                    {
                        fS = 1.0f;
                        fSqrDist = fA00+2.0f*fB0+fC;
                    }
                    else
                    {
                        fS = -fB0/fA00;
                        fSqrDist = fB0*fS+fC;
                    }
                }
            }
            else
            {
                if ( fT > (-Epsilon) )
                {
                    if ( fT < fDet )  // region 1 (side)
                    {
                        fS = 1.0f;
                        fTmp = fA01+fB1;
                        if ( fTmp > (-Epsilon) )
                        {
                            fT = 0.0f;
                            fSqrDist = fA00+2.0f*fB0+fC;
                        }
                        else if ( (-fTmp) > fA11 )
                        {
                            fT = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
                        }
                        else
                        {
                            fT = -fTmp/fA11;
                            fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
                        }
                    }
                    else  // region 2 (corner)
                    {
                        fTmp = fA01+fB0;
                        if ( (-fTmp) < fA00 )
                        {
                            fT = 1.0f;
                            if ( fTmp > (-Epsilon) )
                            {
                                fS = 0.0f;
                                fSqrDist = fA11+2.0f*fB1+fC;
                            }
                            else
                            {
                                 fS = -fTmp/fA00;
                                 fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
                            }
                        }
                        else
                        {
                            fS = 1.0f;
                            fTmp = fA01+fB1;
                            if ( fTmp > (-Epsilon) )
                            {
                                fT = 0.0f;
                                fSqrDist = fA00+2.0f*fB0+fC;
                            }
                            else if ( (-fTmp) > fA11 )
                            {
                                fT = 1.0f;
                                fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
                            }
                            else
                            {
                                fT = -fTmp/fA11;
                                fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
                            }
                        }
                    }
                }
                else  // region 8 (corner)
                {
                    if ( (-fB0) < fA00 )
                    {
                        fT = 0.0f;
                        if ( fB0 > (-Epsilon) )
                        {
                            fS = 0.0f;
                            fSqrDist = fC;
                        }
                        else
                        {
                            fS = -fB0/fA00;
                            fSqrDist = fB0*fS+fC;
                        }
                    }
                    else
                    {
                        fS = 1.0f;
                        fTmp = fA01+fB1;
                        if ( fTmp > (-Epsilon) )
                        {
                            fT = 0.0f;
                            fSqrDist = fA00+2.0f*fB0+fC;
                        }
                        else if ( (-fTmp) > fA11 )
                        {
                            fT = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB0+fTmp);
                        }
                        else
                        {
                            fT = -fTmp/fA11;
                            fSqrDist = fTmp*fT+fA00+2.0f*fB0+fC;
                        }
                    }
                }
            }
        }
        else 
        {
            if ( fT > (-Epsilon) )
            {
                if ( fT < fDet )  // region 5 (side)
                {
                    fS = 0.0f;
                    if ( fB1 > 0.0f )
                    {
                        fT = 0.0f;
                        fSqrDist = fC;
                    }
                    else if ( (-fB1) > fA11 )
                    {
                        fT = 1.0f;
                        fSqrDist = fA11+2.0f*fB1+fC;
                    }
                    else
                    {
                        fT = -fB1/fA11;
                        fSqrDist = fB1*fT+fC;
                    }
                }
                else  // region 4 (corner)
                {
                    fTmp = fA01+fB0;
                    if ( fTmp < 0.0f )
                    {
                        fT = 1.0f;
                        if ( (-fTmp) > fA00 )
                        {
                            fS = 1.0f;
                            fSqrDist = fA00+fA11+fC+2.0f*(fB1+fTmp);
                        }
                        else
                        {
                            fS = -fTmp/fA00;
                            fSqrDist = fTmp*fS+fA11+2.0f*fB1+fC;
                        }
                    }
                    else
                    {
                        fS = 0.0f;
                        if ( fB1 > (-Epsilon) )
                        {
                            fT = 0.0f;
                            fSqrDist = fC;
                        }
                        else if ( (-fB1) > fA11 )
                        {
                            fT = 1.0f;
                            fSqrDist = fA11+2.0f*fB1+fC;
                        }
                        else
                        {
                            fT = -fB1/fA11;
                            fSqrDist = fB1*fT+fC;
                        }
                    }
                }
            }
            else   // region 6 (corner)
            {
                if ( fB0 < (Epsilon) )
                {
                    fT = 0.0f;
                    if ( (-fB0) > fA00 )
                    {
                        fS = 1.0f;
                        fSqrDist = fA00+2.0f*fB0+fC;
                    }
                    else
                    {
                        fS = -fB0/fA00;
                        fSqrDist = fB0*fS+fC;
                    }
                }
                else
                {
                    fS = 0.0f;
                    if ( fB1 > (-Epsilon) )
                    {
                        fT = 0.0f;
                        fSqrDist = fC;
                    }
                    else if ( (-fB1) > fA11 )
                    {
                        fT = 1.0f;
                        fSqrDist = fA11+2.0f*fB1+fC;
                    }
                    else
                    {
                        fT = -fB1/fA11;
                        fSqrDist = fB1*fT+fC;
                    }
                }
            }
        }
    }
    else
    {
        // line segments are parallel
        if ( fA01 > (-Epsilon) )
        {
            // direction vectors form an obtuse angle
            if ( fB0 > (-Epsilon) )
            {
                fS = 0.0f;
                fT = 0.0f;
                fSqrDist = fC;
            }
            else if ( (-fB0) < fA00 )
            {
                fS = -fB0/fA00;
                fT = 0.0f;
                fSqrDist = fB0*fS+fC;
            }
            else
            {
                fB1 = -Dot( kDiff, dirQ);
                fS = 1.0f;
                fTmp = fA00+fB0;
                if ( (-fTmp) > fA01 )
                {
                    fT = 1.0f;
                    fSqrDist = fA00+fA11+fC+2.0f*(fA01+fB0+fB1);
                }
                else
                {
                    fT = -fTmp/fA01;
                    fSqrDist = fA00+2.0f*fB0+fC+fT*(fA11*fT+2.0f*(fA01+fB1));
                }
            }
        }
        else
        {
            // direction vectors form an acute angle
            if ( (-fB0) > fA00 )
            {
                fS = 1.0f;
                fT = 0.0f;
                fSqrDist = fA00+2.0f*fB0+fC;
            }
            else if ( fB0 < (-Epsilon) )
            {
                fS = -fB0/fA00;
                fT = 0.0f;
                fSqrDist = fB0*fS+fC;
            }
            else
            {
                fB1 = -Dot(kDiff, dirQ);
                fS = 0.0f;
                if ( fB0 > -fA01 )
                {
                    fT = 1.0f;
                    fSqrDist = fA11+2.0f*fB1+fC;
                }
                else
                {
                    fT = -fB0/fA01;
                    fSqrDist = fC+fT*(2.0f*fB1+fA11*fT);
                }
            }
        }
    }

     pfSegP0 = fS;
     pfSegP1 = fT;

    return fabs(fSqrDist);
}

bool SphereTestBox(const vector3D& center, const real radius, const Bbox& q)
{
	real dotX = Dot( center - q.center, q.getXaxis());
	real dotY = Dot( center - q.center, q.getYaxis());
	real dotZ = Dot( center - q.center, q.getZaxis());

	real distX = q.size.getX() + ColMargin + radius - fabs(dotX);
	real distY = q.size.getY() + ColMargin + radius - fabs(dotY);
	real distZ = q.size.getZ() + ColMargin + radius - fabs(dotZ);
	if( distX  > Epsilon && distY > Epsilon && distZ  > Epsilon)
	{
		return true;
	}

	return false;
}
bool boxTestBox(const Bbox& p, const Bbox& q)
{
// Separation-Axis theorem

    // compute difference of box centers, D = C1-C0
    vector3D kD = q.center - p.center;

    real aafC[3][3];     // matrix C = A^T B, c_{ij} = Dot(A_i,B_j)
    real aafAbsC[3][3];  // |c_{ij}|
    real afAD[3];        // Dot(A_i,D)
    real fR0, fR1, fR;   // interval radii and distance between centers
    real fR01;           // = R0 + R1
    
    // axis C0+t*A0
    aafC[0][0] = Dot( p.getXaxis(), q.getXaxis());
    aafC[0][1] = Dot( p.getXaxis(), q.getYaxis());
    aafC[0][2] = Dot( p.getXaxis(), q.getZaxis());
    afAD[0] = Dot( p.getXaxis(), kD);
    aafAbsC[0][0] = fabs(aafC[0][0]);
    aafAbsC[0][1] = fabs(aafC[0][1]);
    aafAbsC[0][2] = fabs(aafC[0][2]);
    fR = fabs(afAD[0]);
    fR1 = q.size.getX()*aafAbsC[0][0]+q.size.getY()*aafAbsC[0][1]+q.size.getZ()*aafAbsC[0][2];
    fR01 = p.size.getX() + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1
    aafC[1][0] = Dot( p.getYaxis(), q.getXaxis());
    aafC[1][1] = Dot( p.getYaxis(), q.getYaxis());
    aafC[1][2] = Dot( p.getYaxis(), q.getZaxis());
    afAD[1] = Dot( p.getYaxis(), kD);
    aafAbsC[1][0] = fabs(aafC[1][0]);
    aafAbsC[1][1] = fabs(aafC[1][1]);
    aafAbsC[1][2] = fabs(aafC[1][2]);
    fR = fabs(afAD[1]);
    fR1 = q.size.getX()*aafAbsC[1][0]+q.size.getY()*aafAbsC[1][1]+q.size.getZ()*aafAbsC[1][2];
    fR01 = p.size.getY() + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2
    aafC[2][0] = Dot( p.getZaxis(), q.getXaxis());
    aafC[2][1] = Dot( p.getZaxis(), q.getYaxis());
    aafC[2][2] = Dot( p.getZaxis(), q.getZaxis());
    afAD[2] = Dot( p.getZaxis(), kD);
    aafAbsC[2][0] = fabs(aafC[2][0]);
    aafAbsC[2][1] = fabs(aafC[2][1]);
    aafAbsC[2][2] = fabs(aafC[2][2]);
    fR = fabs(afAD[2]);
    fR1 = q.size.getX()*aafAbsC[2][0]+q.size.getY()*aafAbsC[2][1]+q.size.getZ()*aafAbsC[2][2];
    fR01 = p.size.getZ() + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*B0
    fR = fabs(Dot( q.getXaxis(), kD));
    fR0 = p.size.getX()*aafAbsC[0][0]+p.size.getY()*aafAbsC[1][0]+p.size.getZ()*aafAbsC[2][0];
    fR01 = fR0 + q.size.getX();
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*B1
    fR = fabs(Dot( q.getYaxis(), kD));
    fR0 = p.size.getX()*aafAbsC[0][1]+p.size.getY()*aafAbsC[1][1]+p.size.getZ()*aafAbsC[2][1];
    fR01 = fR0 + q.size.getY();
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*B2
    fR = fabs(Dot( q.getZaxis(), kD));
    fR0 = p.size.getX()*aafAbsC[0][2]+p.size.getY()*aafAbsC[1][2]+p.size.getZ()*aafAbsC[2][2];
    fR01 = fR0 + q.size.getZ();
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A0xB0
    fR = fabs(afAD[2]*aafC[1][0]-afAD[1]*aafC[2][0]);
    fR0 = p.size.getY()*aafAbsC[2][0] + p.size.getZ()*aafAbsC[1][0];
    fR1 = q.size.getY()*aafAbsC[0][2] + q.size.getZ()*aafAbsC[0][1];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A0xB1
    fR = fabs(afAD[2]*aafC[1][1]-afAD[1]*aafC[2][1]);
    fR0 = p.size.getY()*aafAbsC[2][1] + p.size.getZ()*aafAbsC[1][1];
    fR1 = q.size.getX()*aafAbsC[0][2] + q.size.getZ()*aafAbsC[0][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A0xB2
    fR = fabs(afAD[2]*aafC[1][2]-afAD[1]*aafC[2][2]);
    fR0 = p.size.getY()*aafAbsC[2][2] + p.size.getZ()*aafAbsC[1][2];
    fR1 = q.size.getX()*aafAbsC[0][1] + q.size.getY()*aafAbsC[0][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1xB0
    fR = fabs(afAD[0]*aafC[2][0]-afAD[2]*aafC[0][0]);
    fR0 = p.size.getX()*aafAbsC[2][0] + p.size.getZ()*aafAbsC[0][0];
    fR1 = q.size.getY()*aafAbsC[1][2] + q.size.getZ()*aafAbsC[1][1];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1xB1
    fR = fabs(afAD[0]*aafC[2][1]-afAD[2]*aafC[0][1]);
    fR0 = p.size.getX()*aafAbsC[2][1] + p.size.getZ()*aafAbsC[0][1];
    fR1 = q.size.getX()*aafAbsC[1][2] + q.size.getZ()*aafAbsC[1][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A1xB2
    fR = fabs(afAD[0]*aafC[2][2]-afAD[2]*aafC[0][2]);
    fR0 = p.size.getX()*aafAbsC[2][2] + p.size.getZ()*aafAbsC[0][2];
    fR1 = q.size.getX()*aafAbsC[1][1] + q.size.getY()*aafAbsC[1][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2xB0
    fR = fabs(afAD[1]*aafC[0][0]-afAD[0]*aafC[1][0]);
    fR0 = p.size.getX()*aafAbsC[1][0] + p.size.getY()*aafAbsC[0][0];
    fR1 = q.size.getY()*aafAbsC[2][2] + q.size.getZ()*aafAbsC[2][1];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2xB1
    fR = fabs(afAD[1]*aafC[0][1]-afAD[0]*aafC[1][1]);
    fR0 = p.size.getX()*aafAbsC[1][1] + p.size.getY()*aafAbsC[0][1];
    fR1 = q.size.getX()*aafAbsC[2][2] + q.size.getZ()*aafAbsC[2][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    // axis C0+t*A2xB2
    fR = fabs(afAD[1]*aafC[0][2]-afAD[0]*aafC[1][2]);
    fR0 = p.size.getX()*aafAbsC[1][2] + p.size.getY()*aafAbsC[0][2];
    fR1 = q.size.getX()*aafAbsC[2][1] + q.size.getY()*aafAbsC[2][0];
    fR01 = fR0 + fR1;
    if ( fR > (fR01+ColMargin) )
    {
        return false;
    }

    return true;
}

void box2vertex(const Bbox& _box, vector3D* vertexList)
{	
  static vector3D sizeX;
  static vector3D sizeY;
  static vector3D sizeZ;

  sizeX = vector3D( _box.orientation(0,0),_box.orientation(1,0),_box.orientation(2,0));  
  sizeY = vector3D( _box.orientation(0,1),_box.orientation(1,1),_box.orientation(2,1));  
  sizeZ = vector3D( _box.orientation(0,2),_box.orientation(1,2),_box.orientation(2,2));  

  sizeX *=_box.size.getX();
  sizeY *=_box.size.getY();
  sizeZ *=_box.size.getZ();

  vertexList[0]=sizeX + sizeY + sizeZ;
  vertexList[1]=sizeX + sizeY - sizeZ;
  vertexList[2]=sizeX - sizeY -	sizeZ;
  vertexList[3]=sizeX - sizeY + sizeZ;
  vertexList[4]= -vertexList[0];
  vertexList[5]= -vertexList[3];
  vertexList[6]= -vertexList[2];
  vertexList[7]= -vertexList[1];
  vertexList[0]+=_box.center;	
  vertexList[1]+=_box.center;	
  vertexList[2]+=_box.center;	
  vertexList[3]+=_box.center;	
  vertexList[4]+=_box.center;	
  vertexList[5]+=_box.center;	
  vertexList[6]+=_box.center;	
  vertexList[7]+=_box.center;	
}

bool boxFindBox(const Bbox& p, const Bbox& q, vector3D* col_point, vector3D* col_normal, int& nPoints)
{
	static int counter =0;
	counter++;

//	assert(counter<20);

	nPoints=0;

	vector3D vertex1[8];
	vector3D vertex2[8];

	edge_p0 = vector3D();
	edge_p1 = vector3D();
	edge_q0 = vector3D();
	edge_q1 = vector3D();

	box2vertex(p, vertex1);
	box2vertex(q, vertex2);

	//vertex-face collision
	int index_v1=-1;
	real min_dist1=REAL_MAX;
	real size1=0;
	vector3D gess_normal1;
	for( int i=0; i < 8; i++)
	{
		vector3D ax = q.getXaxis();
		vector3D ay = q.getYaxis();
		vector3D az = q.getZaxis();

		real dotX = Dot( vertex1[i] - q.center, q.getXaxis());
		real dotY = Dot( vertex1[i] - q.center, q.getYaxis());
		real dotZ = Dot( vertex1[i] - q.center, q.getZaxis());

		real distX = q.size.getX() + ColMargin - fabs(dotX);
		real distY = q.size.getY() + ColMargin - fabs(dotY);
		real distZ = q.size.getZ() + ColMargin - fabs(dotZ);
		if( distX  > Epsilon && distY > Epsilon && distZ  > Epsilon)
		{
			//inside
			real dist = VALUE_MIN(distX,VALUE_MIN(distY,distZ));
			if(1)// dist < min_dist1)
			{
				//deeper than last point
				if( dist < distY && dist < distZ)
				{
					real discX = Dot( vertex1[i] - p.center, q.getXaxis());
					if( dotX > Epsilon && discX < Epsilon)
					{
						//collide from above
						size1 = q.size.getX();
						gess_normal1 = q.getXaxis();
						min_dist1 = dist;
						index_v1 = i;
					}
					else
					if( dotX < Epsilon && discX > Epsilon)
					{
						//collide from below
						size1 = q.size.getX();
						gess_normal1 = -q.getXaxis();
						min_dist1 = dist;
						index_v1 = i;
					}
					col_point[nPoints] = vertex1[i];
					col_normal[nPoints] = gess_normal1;		
					nPoints++;
				}
				else
				if( dist < distX && dist < distZ)
				{
					real discY = Dot( vertex1[i] - p.center, q.getYaxis());
					if( dotY > Epsilon && discY < Epsilon)
					{
						//collide from above
						size1 = q.size.getY();
						gess_normal1 = q.getYaxis();
						min_dist1 = dist;
						index_v1 = i;
					}
					else
					if( dotY < Epsilon && discY > Epsilon)
					{
						//collide from below
						size1 = q.size.getY();
						gess_normal1 = -q.getYaxis();
						min_dist1 = dist;
						index_v1 = i;
					}
					col_point[nPoints] = vertex1[i];
					col_normal[nPoints] = gess_normal1;		
					nPoints++;

				}
				else
				if( dist < distX && dist < distY)
				{
					real discZ = Dot( vertex1[i] - p.center, q.getZaxis());
					if( dotZ > Epsilon && discZ < Epsilon)
					{
						//collide from above
						size1 = q.size.getZ();
						gess_normal1 = q.getZaxis();
						min_dist1 = dist;
						index_v1 = i;
					}
					else
					if( dotZ < Epsilon && discZ > Epsilon)
					{
						//collide from below
						size1 = q.size.getZ();
						gess_normal1 = -q.getZaxis();
						min_dist1 = dist;
						index_v1 = i;
					}
					col_point[nPoints] = vertex1[i];
					col_normal[nPoints] = gess_normal1;		
					nPoints++;
				}
			}
		}
	}

	real min_dist2=REAL_MAX;
	int index_v2=-1;
	real size2=0;
	vector3D gess_normal2;
	for( int i=0; i < 8; i++)
	{
		real dotX = Dot( vertex2[i] - p.center, p.getXaxis());
		real dotY = Dot( vertex2[i] - p.center, p.getYaxis());
		real dotZ = Dot( vertex2[i] - p.center, p.getZaxis());
		real distX = p.size.getX() + ColMargin - fabs(dotX);
		real distY = p.size.getY() + ColMargin - fabs(dotY);
		real distZ = p.size.getZ() + ColMargin - fabs(dotZ);
		if( distX  > Epsilon && distY > Epsilon && distZ  > Epsilon)
		{
			//inside
			real dist = VALUE_MIN(distX,VALUE_MIN(distY,distZ));
			if(1)// dist < min_dist2)
			{
				//deeper than last point
				if( dist < distY && dist < distZ)
				{
					real discX = Dot( vertex2[i] - q.center, p.getXaxis());
					if( dotX > Epsilon && discX < Epsilon)
					{
						size2 = p.size.getX();
						gess_normal2 = p.getXaxis();
						min_dist2 = dist;
						index_v2 = i;
					}
					else
					if( dotX < Epsilon && discX > Epsilon)
					{
						size2 = p.size.getX();
						gess_normal2 = -p.getXaxis();
						min_dist2 = dist;
						index_v2 = i;
					}					
					col_point[nPoints] = vertex2[i];
					col_normal[nPoints] = -gess_normal2;		
					nPoints++;
				}
				else
				if( dist < distX && dist < distZ)
				{
					real discY = Dot( vertex1[i] - q.center, p.getYaxis());
					if( dotY > Epsilon && discY < Epsilon)
					{
						size2 = p.size.getY();
						gess_normal2 = p.getYaxis();
						min_dist2 = dist;
						index_v2 = i;
					}
					else
					if( dotY < Epsilon && discY > Epsilon)
					{
						size2 = p.size.getY();
						gess_normal2 = -p.getYaxis();
						min_dist2 = dist;
						index_v2 = i;
					}
					col_point[nPoints] = vertex2[i];
					col_normal[nPoints] = -gess_normal2;		
					nPoints++;
				}
				else
				if( dist < distX && dist < distY)
				{
					real discZ = Dot( vertex1[i] - q.center, p.getZaxis());
					if( dotZ > Epsilon && discZ < Epsilon)
					{
						size2 = p.size.getZ();
						gess_normal2 = p.getZaxis();
						min_dist2 = dist;
						index_v2 = i;
					}
					else
					if( dotZ < Epsilon && discZ > Epsilon)
					{
						size2 = p.size.getZ();
						gess_normal2 = -p.getZaxis();
						min_dist2 = dist;
						index_v2 = i;
					}
					col_point[nPoints] = vertex2[i];
					col_normal[nPoints] = -gess_normal2;		
					nPoints++;
				}
			}
		}
	}
/*
	real min_dist_vertex=-REAL_MAX;
	vector3D vexterface_point;
	vector3D vextexface_normal;
	if( index_v2 >= 0 )//&& min_dist2 < min_dist1)
	{
//		assert(0);
		vexterface_point = vertex2[index_v2];
//		vector3D q = p.center + size2*gess_normal2;
//		real dot2 = Dot( vexterface_point - q, gess_normal2);
		real dot = Dot( vexterface_point - p.center, gess_normal2);
		if( dot > 0.0f) 
		{
			vextexface_normal= -gess_normal2;
			min_dist_vertex = size2 - dot;
		}
		else
		{
			vextexface_normal= gess_normal2;
			min_dist_vertex = size2 + dot;
		}
		col_point[nPoints] = vexterface_point;
		col_normal[nPoints] = vextexface_normal;		
		nPoints++;
	}
	else
	if( index_v1 >= 0 )//&& min_dist1 < min_dist2)
	{
//		assert(0);
		vexterface_point = vertex1[index_v1];
		real dot = Dot( vexterface_point - q.center, gess_normal1);
		if( dot < 0.0f) 
		{
			vextexface_normal= -gess_normal1;
			min_dist_vertex =  size1 + dot;
		}
		else
		{
			vextexface_normal= gess_normal1;
			min_dist_vertex = size1 - dot;
		}
		col_point[nPoints] = vexterface_point;
		col_normal[nPoints] = vextexface_normal;		
		nPoints++;
	}
*/
	//edge-edge collision
	poly_edge edges[12];
	edges[0].i0=0;edges[0].i1=1;
	edges[1].i0=1;edges[1].i1=2;
	edges[2].i0=2;edges[2].i1=3;
	edges[3].i0=3;edges[3].i1=0;

	edges[4].i0=4;edges[4].i1=5;
	edges[5].i0=5;edges[5].i1=6;
	edges[6].i0=6;edges[6].i1=7;
	edges[7].i0=7;edges[7].i1=4;

	edges[8].i0=3;edges[8].i1=7;
	edges[9].i0=2;edges[9].i1=4;

	edges[10].i0=0;edges[10].i1=6;
	edges[11].i0=1;edges[11].i1=5;

	real min_dist_edges=REAL_MAX;
	real min_dist_edges2=REAL_MAX;
	real t,s;
//	real t2,s2;
	vector3D edge_point, edge_point2;
	vector3D edge_normal, edge_normal2;
	int e1=-1,e2=-1;
	int e12=-1,e22=-1;
	int hits=0;

	for(int i=0; i < 12; i++)
	{
		vector3D p0=vertex1[edges[i].i0];
		vector3D p1=vertex1[edges[i].i1];
		vector3D pm = (p0+p1)*.5f;
		for(int j=0; j < 12; j++)
		{
			vector3D q0=vertex2[edges[j].i0];
			vector3D q1=vertex2[edges[j].i1];
			vector3D qm = (q0+q1)*.5f;
			real discr_pq=Dot(pm-p.center,qm-q.center);
			if( discr_pq > Epsilon)continue;

			real dist = sqrDistanceSegs(p0, p1, q0, q1, t, s);
			if( dist < min_dist_edges)
			{
				min_dist_edges2=min_dist_edges;
				edge_point2 = edge_point; 
				edge_normal2 = edge_normal;
				e12 = e1;
				e22 = e2;

				min_dist_edges=dist;
				edge_point = p0 + t*(p1-p0); 
				edge_normal = CrossProduct(p1-p0, q1-q0);
				edge_normal.normalize();
				e1 = i;
				e2 = j;
				hits++;
			}
			else
			if( dist < min_dist_edges2)
			{
				//we need to store 2 nearest pair of edges to make sure
				min_dist_edges2=dist;
				edge_point2 = p0 + t*(p1-p0); 
				edge_normal2 = CrossProduct(p1-p0, q1-q0);
				edge_normal2.normalize();
				e12 = i;
				e22 = j;
				hits++;
			}
		}
	}

	if( Dot(p.center-q.center,edge_normal) < 0.0f)
	{
		edge_normal=-edge_normal;
	}

	real dotX = fabs(Dot( edge_point  - q.center, q.getXaxis()));
	real dotY = fabs(Dot( edge_point  - q.center, q.getYaxis()));
	real dotZ = fabs(Dot( edge_point  - q.center, q.getZaxis()));
	if( dotX < (q.size.getX()+ ColMargin) && 
		dotY < (q.size.getY()+ ColMargin) &&
		dotZ < (q.size.getZ()+ ColMargin))
	{
		dotX = fabs(Dot( edge_point  - p.center, p.getXaxis()));
		dotY = fabs(Dot( edge_point  - p.center, p.getYaxis()));
		dotZ = fabs(Dot( edge_point  - p.center, p.getZaxis()));
		if( dotX < (p.size.getX()+ ColMargin) && 
			dotY < (p.size.getY()+ ColMargin) &&
			dotZ < (p.size.getZ()+ ColMargin))
		{
			//inside
			col_point[nPoints] = edge_point;
			col_normal[nPoints] = edge_normal;	
			nPoints++;
   			//draw collion info
			edge_p0 = vertex1[edges[e1].i0];
			edge_p1 = vertex1[edges[e1].i1];
			edge_q0 = vertex2[edges[e2].i0];
			edge_q1 = vertex2[edges[e2].i1];			
		}
	}
	if( hits >=2)
	{
		if( Dot(p.center-q.center,edge_normal2) < 0.0f)
		{
			edge_normal2=-edge_normal2;
		}

		// if second nearest point is inside the boxes, it is the "true" collision point
		dotX = fabs(Dot( edge_point2  - q.center, q.getXaxis()));
		dotY = fabs(Dot( edge_point2  - q.center, q.getYaxis()));
		dotZ = fabs(Dot( edge_point2  - q.center, q.getZaxis()));
		if( dotX < (q.size.getX()+ ColMargin) && 
			dotY < (q.size.getY()+ ColMargin) &&
			dotZ < (q.size.getZ()+ ColMargin))
		{
			dotX = fabs(Dot( edge_point2  - p.center, p.getXaxis()));
			dotY = fabs(Dot( edge_point2  - p.center, p.getYaxis()));
			dotZ = fabs(Dot( edge_point2  - p.center, p.getZaxis()));
			if( dotX < (p.size.getX()+ ColMargin) && 
				dotY < (p.size.getY()+ ColMargin) &&
				dotZ < (p.size.getZ()+ ColMargin))
			{
				//inside
				col_point[nPoints] = edge_point2;
				col_normal[nPoints] = edge_normal2;	
				nPoints++;
			}
		}
	}

	assert(nPoints < 16);
	//draw collion info
	for( int i=0; i < 4; i++)
	{
		last_col_point[i] = col_point[i];
		last_col_normal[i] = col_normal[i];
	}
	return true;
}


