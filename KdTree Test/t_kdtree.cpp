//
//

#include "stdafx.h"
#include "t_kdtree.h"
#include "t_sse.h"

int g_test = 0;
int g_normalMiss = 0;
int g_edgeMiss = 0;

/*---------------------------------------------------------------------------*/ 
static float CalcDistance(const TVector& a, const TVector& b, const TVector& orig, float* u)
{
	TVector d = b - a;

	*u = ( (orig.x-a.x) * d.x + (orig.y-a.y) * d.y  + (orig.z-a.z) * d.z) / (d.x * d.x + d.y * d.y + d.z * d.z);

	if(0 < (*u) && (*u) < 1)
	{
		/* 점이 Edge 의 유효 범위에 들었다. */ 
		TVector p = a + d*(*u);

		return TLength(&(orig - p));
	}
	else
	{
		/* 점이 선분의 범위를 넘었으므로 윈점과 거리 측정 */ 
		*u = 0;

		return TLength(&(orig - a));
	}
}

/*---------------------------------------------------------------------------*/ 
static __m128 CalcDistance4(const TPoint4& a, const TPoint4& b, const TPoint4& orig, __m128* u)
{
	TPoint4 d = b - a;

	*u = ( (orig.x-a.x) * d.x + (orig.y-a.y) * d.y  + (orig.z-a.z) * d.z) / (d.x * d.x + d.y * d.y + d.z * d.z);

	__m128 mask = (g_zero4 < *u) & (*u < g_one4);

	*u = mask & *u;

	/* 점이 Edge 의 유효 범위에 들었다. */ 
	TPoint4 dist(orig - (a + (d * (*u))));

	return TPoint4Length(&dist);
}

/*---------------------------------------------------------------------------*/ 
static __m256 CalcDistance8(const TPoint8& a, const TPoint8& b, const TPoint8& orig, __m256* u)
{
	TPoint8 d = b - a;

	*u = ( (orig.x-a.x) * d.x + (orig.y-a.y) * d.y  + (orig.z-a.z) * d.z) / (d.x * d.x + d.y * d.y + d.z * d.z);

	__m256 mask = (_mm256_setzero_ps() < *u) & (*u < g_one8);

	*u = mask & *u;

	/* 점이 Edge 의 유효 범위에 들었다. */ 
	TPoint8 dist(orig - (a + (d * (*u))));

	return TPoint8Length(&dist);
}

/*---------------------------------------------------------------------------*/ 
/*                                                                           */ 
/*---------------------------------------------------------------------------*/ 
TAABB::TAABB()
{
	ResetAABB();
}

/*---------------------------------------------------------------------------*/ 
TAABB::TAABB(const float* mini, const float* maxi)
{
	minimum[0] = mini[0];
	minimum[1] = mini[1];
	minimum[2] = mini[2];

	maximum[0] = maxi[0];
	maximum[1] = maxi[1];
	maximum[2] = maxi[2];
}

/*---------------------------------------------------------------------------*/ 
void TAABB::ResetAABB()
{
	minimum[0] = FLT_MAX;
	minimum[1] = FLT_MAX;
	minimum[2] = FLT_MAX;

	maximum[0] = -FLT_MAX;
	maximum[1] = -FLT_MAX;
	maximum[2] = -FLT_MAX;
}

/*---------------------------------------------------------------------------*/ 
void TAABB::AddPoint(const float* p)
{
	minimum[0] = min(minimum[0], p[0]);
	minimum[1] = min(minimum[1], p[1]);
	minimum[2] = min(minimum[2], p[2]);

	maximum[0] = max(maximum[0], p[0]);
	maximum[1] = max(maximum[1], p[1]);
	maximum[2] = max(maximum[2], p[2]);
}

/*---------------------------------------------------------------------------*/ 
void TAABB::AddAABB(const TAABB& aabb)
{
	AddPoint(aabb.minimum);
	AddPoint(aabb.maximum);
}

/*---------------------------------------------------------------------------*/ 
TAABB TAABB::SplitLeft(int n, float split) const
{
	TVector temp(maximum);

	temp[n] = split;

	return TAABB(minimum, temp);
}

/*---------------------------------------------------------------------------*/ 
TAABB TAABB::SplitRight(int n, float split) const
{
	TVector temp(minimum);

	temp[n] = split;

	return TAABB(temp, maximum);
}

/*---------------------------------------------------------------------------*/ 
bool TAABB::HitTest(const float* orig, const float* dir, HitResult* result) const
{
	float mini[3] = { minimum[0] - orig[0], minimum[1] - orig[1], minimum[2] - orig[2] };
	float maxi[3] = { maximum[0] - orig[0], maximum[1] - orig[1], maximum[2] - orig[2] };

	return HitTest(dir, mini, maxi, result);
}

/*---------------------------------------------------------------------------*/ 
bool TAABB::HitTest(const float* dir, const float* mini, const float* maxi, HitResult* result) const
{
	float dist = FLT_MAX;

	float delta;

	if(dir[0] != 0)
	{
		delta = mini[0] / dir[0];
		if(	delta > 0
			&& delta < dist
			&& mini[1] <= dir[1] * delta && dir[1] * delta <= maxi[1]
			&& mini[2] <= dir[2] * delta && dir[2] * delta <= maxi[2] )
		{
			dist = min(dist, delta);
		}

		delta = maxi[0] / dir[0];
		if(	delta > 0
			&& delta < dist
			&& mini[1] <= dir[1] * delta && dir[1] * delta <= maxi[1]
			&& mini[2] <= dir[2] * delta && dir[2] * delta <= maxi[2] )
		{
			dist = min(dist, delta);
		}
	}

	if(dir[1] != 0)
	{
		delta = mini[1] / dir[1];
		if(	delta > 0
			&& delta < dist
			&& mini[0] <= dir[0] * delta && dir[0] * delta <= maxi[0]
			&& mini[2] <= dir[2] * delta && dir[2] * delta <= maxi[2] )
		{
			dist = min(dist, delta);
		}

		delta = maxi[1] / dir[1];
		if(	delta > 0
			&& delta < dist
			&& mini[0] <= dir[0] * delta && dir[0] * delta <= maxi[0]
			&& mini[2] <= dir[2] * delta && dir[2] * delta <= maxi[2] )
		{
			dist = min(dist, delta);
		}
	}

	if(dir[2] != 0)
	{
		delta = mini[2] / dir[2];
		if(	delta > 0
			&& delta < dist
			&& mini[0] <= dir[0] * delta && dir[0] * delta <= maxi[0]
			&& mini[1] <= dir[1] * delta && dir[1] * delta <= maxi[1] )
		{
			dist = min(dist, delta);
		}

		delta = maxi[2] / dir[2];
		if(	delta > 0
			&& delta < dist
			&& mini[0] <= dir[0] * delta && dir[0] * delta <= maxi[0]
			&& mini[1] <= dir[1] * delta && dir[1] * delta <= maxi[1] )
		{
			dist = min(dist, delta);
		}
	}

	if(dist != FLT_MAX)
	{
		result->t = dist;
		return true;
	}
	else
	{
		return false;
	}
}

/*---------------------------------------------------------------------------*/ 
bool TAABB::IntersectTest(const TAABB& aabb) const
{
	if(aabb.maximum[0] < minimum[0] || maximum[0] < aabb.minimum[0])
		return false;
	if(aabb.maximum[1] < minimum[1] || maximum[1] < aabb.minimum[1])
		return false;
	if(aabb.maximum[2] < minimum[2] || maximum[2] < aabb.minimum[2])
		return false;

	return true;
}

/*---------------------------------------------------------------------------*/ 
TAABB TAABB::operator+(const float* pos0) const
{
	TAABB aabb(minimum, maximum);

	aabb.minimum[0] += pos0[0];
	aabb.minimum[1] += pos0[1];
	aabb.minimum[2] += pos0[2];

	aabb.maximum[0] += pos0[0];
	aabb.maximum[1] += pos0[1];
	aabb.maximum[2] += pos0[2];

	return aabb;
}

/*---------------------------------------------------------------------------*/ 
TVector TAABB::CalcCenter() const
{
	return ( TVector(maximum) + TVector(minimum) ) / 2;
}

/*---------------------------------------------------------------------------*/ 
float TAABB::CalcRadius() const
{
	return TLength(&(TVector(maximum[0]-minimum[0], maximum[1]-minimum[1], maximum[2]-minimum[2])));
}

/*---------------------------------------------------------------------------*/ 
TVector TAABB::CalcSize() const
{
	return ( TVector(maximum) - TVector(minimum) );
}

/*---------------------------------------------------------------------------*/ 
float TAABB::CalcArea() const
{
	TVector Size = CalcSize();
	return (Size.x * Size.y + Size.y * Size.z + Size.z * Size.x) * 2;
}

/*---------------------------------------------------------------------------*/ 
/*                                                                           */ 
/*---------------------------------------------------------------------------*/ 
TTriangle::TTriangle(const TVector& p0, const TVector& p1, const TVector& p2)
{
	pos0 = p0;
	pos1 = p1;
	pos2 = p2;
}

/*---------------------------------------------------------------------------*/ 
TAABB TTriangle::GetAABB() const
{
	TAABB aabb;

	aabb.AddPoint(pos0);
	aabb.AddPoint(pos1);
	aabb.AddPoint(pos2);

	return aabb;
}

/*---------------------------------------------------------------------------*/ 
bool TTriangle::IntersectTest(const TAABB& aabb) const
{
	++g_test;

	// we will do separating axis test.
	TVector pointsAABB[8] = {	TVector(aabb.minimum[0], aabb.minimum[1], aabb.minimum[2]),
									TVector(aabb.minimum[0], aabb.minimum[1], aabb.maximum[2]),
									TVector(aabb.minimum[0], aabb.maximum[1], aabb.minimum[2]),
									TVector(aabb.minimum[0], aabb.maximum[1], aabb.maximum[2]),
									TVector(aabb.maximum[0], aabb.minimum[1], aabb.minimum[2]),
									TVector(aabb.maximum[0], aabb.minimum[1], aabb.maximum[2]),
									TVector(aabb.maximum[0], aabb.maximum[1], aabb.minimum[2]),
									TVector(aabb.maximum[0], aabb.maximum[1], aabb.maximum[2]), };
	
	TVector pointsTriangle[3] = { pos0, pos1, pos2 };

	// try triangle's normal plain.
	TVector edge1 = pos1 - pos0;
    TVector edge2 = pos2 - pos0;

	TVector normal;
	TCross(&normal, &edge1, &edge2);

	float minAABB = FLT_MAX;
	float maxAABB = -FLT_MAX;

	for(int k=0; k<8; ++k)
	{
		float pos0 = TDot(&normal, &pointsAABB[k]);
		minAABB = min(minAABB, pos0);
		maxAABB = max(maxAABB, pos0);
	}

	float pTriangle = TDot(&normal, &pointsTriangle[0]);

	if(pTriangle < minAABB || maxAABB < pTriangle) 
	{
		++g_normalMiss;
		return false;
	}

	// try all edge pairs.
	TVector axis[3] = { TVector(1,0,0), TVector(0,1,0), TVector(0,0,1) };
	TVector edge[3] = { edge1, edge2, pos1 - pos2 };

	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			TVector cross;
			TCross(&cross, &axis[i], &edge[j]);

			float minAABB = FLT_MAX;
			float maxAABB = -FLT_MAX;

			for(int k=0; k<8; ++k)
			{
				float pos0 = TDot(&cross, &pointsAABB[k]);
				minAABB = min(minAABB, pos0);
				maxAABB = max(maxAABB, pos0);
			}

			float minTriangle = FLT_MAX;
			float maxTriangle = -FLT_MAX;

			for(int k=0; k<3; ++k)
			{
				float pos0 = TDot(&cross, &pointsTriangle[k]);
				minTriangle = min(minTriangle, pos0);
				maxTriangle = max(maxTriangle, pos0);
			}

			if(maxTriangle < minAABB || maxAABB < minTriangle)
			{
				++g_edgeMiss;
				return false;
			}
		}
	}

	return true;
}

/*---------------------------------------------------------------------------*/ 
bool TTriangle::HitTest(const TVector& orig, const TVector& dir, HitResult* result) const
{
	return HitTest(pos0, pos1, pos2, orig, dir, result);
}

bool TTriangle::HitTest(const TVector& orig, const TVector& dir, const TVector& scale, HitResult* result) const
{
	return HitTest(pos0 * scale, pos1 * scale, pos2 * scale, orig, dir, result);
}

/*---------------------------------------------------------------------------*/ 
bool TTriangle::HitTest(const TVector& p0, const TVector& p1, const TVector& p2, const TVector& orig, const TVector& dir, HitResult* result) const
{
	int u,v,w;
	TVector edge1 = p1 - p0;
    TVector edge2 = p2 - p0;

	TVector normal;
	TCross(&normal, &edge1, &edge2);

	if(abs(normal.x) < abs(normal.y))
	{
		// x<y
		if(abs(normal.z) < abs(normal.x))
		{
			// z<x<y
			u = 0;
			v = 2;
			w = 1;
		}
		else
		{
			// x<=z && x<y
			if(abs(normal.z) < abs(normal.y))
			{
				// x<=z<y
				u = 0;
				v = 2;
				w = 1;
			}
			else
			{
				// x<=y<=z
				u = 0;
				v = 1;
				w = 2;
			}

		}
	}
	else
	{
		// y<=x
		if(abs(normal.z) < abs(normal.y))
		{
			// z<y<=x
			u = 1;
			v = 2;
			w = 0;
		}
		else
		{
			// y<=z && y<=x
			if(abs(normal.z) < abs(normal.x))
			{
				// y<=z<x
				u = 1;
				v = 2;
				w = 0;
			}
			else
			{
				// y<=x<=z
				u = 0;
				v = 1;
				w = 2;
			}
		}
	}

	float sign = 1.0f;
	for(int i=0; i<w; ++i) sign *= -1.0f;

	float nw = normal[w];
	float nu = normal[u] / nw;
	float nv = normal[v] / nw;
	float pu = p0[u];
	float pv = p0[v];
	float np = (nu*pu + nv*pv + p0[w]);
	float e0u = sign * edge1[u] / nw;
	float e0v = sign * edge1[v] / nw;
	float e1u = sign * edge2[u] / nw;
	float e1v = sign * edge2[v] / nw;
	int ci = w;

	w = ci;
	u = w == 0 ? 1 : 0;
	v = w == 2 ? 1 : 2;

	float ou = orig[u];
	float ov = orig[v];
	float ow = orig[w];
	float du = dir[u];
	float dv = dir[v];
	float dw = dir[w];

	float dett = np -(ou*nu+ov*nv+ow);
	float det = du*nu+dv*nv+dw;
	float Du = du*dett - (pu-ou)*det;
	float Dv = dv*dett - (pv-ov)*det;
	float detu = (e1v*Du - e1u*Dv);
	float detv = (e0u*Dv - e0v*Du);
	
	float tmpdet0 = det - detu - detv;
	if(tmpdet0 * detu > 0 && tmpdet0 * detv > 0  && detv * detu > 0)
	{
		float rdet = 1/det;
		result->t = dett * rdet;
		result->u = detu * rdet;
		result->v = detv * rdet;

		return result->t > 0;
	}
	else
	{
		return false;
	}
}

/*---------------------------------------------------------------------------*/ 
bool TTriangle::OcclusionTest(const TVector& orig, const TVector& dir) const
{
	HitResult result;

	return HitTest(orig, dir, &result);
}

/*---------------------------------------------------------------------------*/ 
bool TTriangle::NearestTest(const TVector& orig, float radius, HitResult* result) const
{
	TVector p0 = pos0;
	TVector p1 = pos1;
	TVector p2 = pos2;

	float u0, u1, u2;
	float d0 = ::CalcDistance(p0, p1, orig, &u0);
	float d1 = ::CalcDistance(p1, p2, orig, &u1);
	float d2 = ::CalcDistance(p2, p0, orig, &u2);

	if(d0 < d2)
	{
		if(d0 < d1)
		{
			if(d0 < radius)
			{
				result->u = u0;
				result->v = 0;
				result->t = d0;

				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			if(d1 < radius)
			{
				result->u = 1 - u1;
				result->v = u1;
				result->t = d1;

				return true;
			}
			else
			{
				return false;
			}
		}
	}
	else
	{
		if(d1 < d2)
		{
			if(d1 < radius)
			{
				result->u = 1 - u1;
				result->v = u1;
				result->t = d1;

				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			if(d2 < radius)
			{
				result->u = 0;
				result->v = 1 - u2;
				result->t = d2;

				return true;
			}
			else
			{
				return false;
			}
		}
	}
}

/*---------------------------------------------------------------------------*/ 
