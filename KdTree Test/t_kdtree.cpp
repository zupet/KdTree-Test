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
/*                                                                           */ 
/*---------------------------------------------------------------------------*/ 
TTriangle::TTriangle(const TVector& p0, const TVector& p1, const TVector& p2)
{
	pos0 = p0;
	pos1 = p1;
	pos2 = p2;
}

/*---------------------------------------------------------------------------*/ 
TTriangle::THit::THit(const TTriangle & triangle)
{
	TVector edge1 = triangle.pos1 - triangle.pos0;
    TVector edge2 = triangle.pos2 - triangle.pos0;

	TVector normal;
	TCross(&normal, &edge1, &edge2);

	int u,v,w;
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
	nu = normal[u] / nw;
	nv = normal[v] / nw;
	pu = triangle.pos0[u];
	pv = triangle.pos0[v];
	np = (nu*pu + nv*pv + triangle.pos0[w]);
	e0u = sign * edge1[u] / nw;
	e0v = sign * edge1[v] / nw;
	e1u = sign * edge2[u] / nw;
	e1v = sign * edge2[v] / nw;

	ci = w;
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
bool TTriangle::THit::HitTest(const TVector& orig, const TVector& dir, HitResult* result) const
{
	int u, v, w;
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
bool TTriangle::THit::OcclusionTest(const TVector& orig, const TVector& dir) const
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
__m128	TTriangle::THit::HitTest4(__m128 mask, const TPoint4& orig, const TVector& d, HitResult4* result) const
{
	int u, v, w;
	w = ci;
	u = w == 0 ? 1 : 0;
	v = w == 2 ? 1 : 2;

	__m128 nu = _mm_load_ps1(&this->nu);
	__m128 np = _mm_load_ps1(&this->np);
	__m128 nv = _mm_load_ps1(&this->nv);
	__m128 pu = _mm_load_ps1(&this->pu);
	__m128 pv = _mm_load_ps1(&this->pv);
	__m128 e0u = _mm_load_ps1(&this->e0u);
	__m128 e0v = _mm_load_ps1(&this->e0v);
	__m128 e1u = _mm_load_ps1(&this->e1u);
	__m128 e1v = _mm_load_ps1(&this->e1v);

	__m128 ou = orig[u];
	__m128 ov = orig[v];
	__m128 ow = orig[w];
	__m128 du = _mm_load_ps1(&d[u]);
	__m128 dv = _mm_load_ps1(&d[v]);
	__m128 dw = _mm_load_ps1(&d[w]);

	__m128 dett = np -(ou*nu+ov*nv+ow);
	__m128 det = du*nu+dv*nv+dw;
	__m128 Du = du*dett - (pu-ou)*det;
	__m128 Dv = dv*dett - (pv-ov)*det;
	__m128 detu = (e1v*Du - e1u*Dv);
	__m128 detv = (e0u*Dv - e0v*Du);

	__m128 tmpdet0 = det - detu - detv;

	__m128 detMask = _mm_xor_ps(_mm_xor_ps(tmpdet0, detv) | _mm_xor_ps(detv, detu), g_one4) > _mm_setzero_ps();

	mask = mask & detMask;

	__m128 rdet = _mm_rcp_ps(det);

	result->t = dett * rdet;
	result->u = detu * rdet;
	result->v = detv * rdet;

	return mask & (result->t > _mm_setzero_ps());
}

/*---------------------------------------------------------------------------*/ 
__m128 TTriangle::THit::OcclusionTest4(__m128 mask, const TPoint4& orig, const TVector& dir) const
{
	HitResult4 result;
	return HitTest4(mask, orig, dir, &result);
}

/*---------------------------------------------------------------------------*/ 
__m128 TTriangle::NearestTest4(__m128 mask, const TPoint4& orig, __m128 radius, HitResult4* result) const
{
	TPoint4 p0(pos0);
	TPoint4 p1(pos1);
	TPoint4 p2(pos2);

	__m128 u0, u1, u2;

	__m128 d0 = ::CalcDistance4(p0, p1, orig, &u0);
	__m128 d1 = ::CalcDistance4(p1, p2, orig, &u1);
	__m128 d2 = ::CalcDistance4(p2, p0, orig, &u2);

	__m128 minimum = _mm_min_ps(d0, _mm_min_ps(d1, d2));

	__m128 minMask;
	
	result->u = u0;
	result->v = g_zero4;
	result->t = d0;

	minMask = (minimum == d1);
	result->u = _mm_merge_ps(minMask, result->u, (g_one4 - u1));
	result->v = _mm_merge_ps(minMask, result->v, u1);
	result->t = _mm_merge_ps(minMask, result->t, d1);

	minMask = (minimum == d2);
	result->u = _mm_merge_ps(minMask, result->u, g_zero4);
	result->v = _mm_merge_ps(minMask, result->v, g_one4 - u2);
	result->t = _mm_merge_ps(minMask, result->t, d2);

	return mask & (result->t < radius);
}

/*---------------------------------------------------------------------------*/ 
__m256 TTriangle::THit::HitTest8(__m256 mask, const TPoint8& orig, const TVector& d, HitResult8* result) const
{
	int u, v, w;
	w = ci;
	u = w == 0 ? 1 : 0;
	v = w == 2 ? 1 : 2;

	__m256 nu = _mm256_broadcast_ss(&this->nu);
	__m256 np = _mm256_broadcast_ss(&this->np);
	__m256 nv = _mm256_broadcast_ss(&this->nv);
	__m256 pu = _mm256_broadcast_ss(&this->pu);
	__m256 pv = _mm256_broadcast_ss(&this->pv);
	__m256 e0u = _mm256_broadcast_ss(&this->e0u);
	__m256 e0v = _mm256_broadcast_ss(&this->e0v);
	__m256 e1u = _mm256_broadcast_ss(&this->e1u);
	__m256 e1v = _mm256_broadcast_ss(&this->e1v);

	__m256 ou = orig[u];
	__m256 ov = orig[v];
	__m256 ow = orig[w];
	__m256 du = _mm256_broadcast_ss(&d[u]);
	__m256 dv = _mm256_broadcast_ss(&d[v]);
	__m256 dw = _mm256_broadcast_ss(&d[w]);

	__m256 dett = np -(ou*nu+ov*nv+ow);
	__m256 det = du*nu+dv*nv+dw;
	__m256 Du = du*dett - (pu-ou)*det;
	__m256 Dv = dv*dett - (pv-ov)*det;
	__m256 detu = (e1v*Du - e1u*Dv);
	__m256 detv = (e0u*Dv - e0v*Du);

	__m256 tmpdet0 = det - detu - detv;

	__m256 detMask = _mm256_xor_ps(_mm256_xor_ps(tmpdet0, detv) | _mm256_xor_ps(detv, detu), g_one8) > _mm256_setzero_ps();

	mask = mask & detMask;

	__m256 rdet = _mm256_rcp_ps(det);

	result->t = dett * rdet;
	result->u = detu * rdet;
	result->v = detv * rdet;

	return mask & (result->t > _mm256_setzero_ps());
/**/ 
}

/*---------------------------------------------------------------------------*/ 
__m256 TTriangle::THit::OcclusionTest8(const __m256& mask, const TPoint8& orig, const TVector& dir) const
{
	HitResult8 result;
	return HitTest8(mask, orig, dir, &result);
}

/*---------------------------------------------------------------------------*/ 
__m256 TTriangle::NearestTest8(const __m256& mask, const TPoint8& orig, const __m256& radius, HitResult8* result) const
{
	TPoint8 p0(pos0);
	TPoint8 p1(pos1);
	TPoint8 p2(pos2);

	__m256 u0, u1, u2;

	__m256 d0 = ::CalcDistance8(p0, p1, orig, &u0);
	__m256 d1 = ::CalcDistance8(p1, p2, orig, &u1);
	__m256 d2 = ::CalcDistance8(p2, p0, orig, &u2);

	__m256 minimum = _mm256_min_ps(d0, _mm256_min_ps(d1, d2));

	__m256 minMask;
	
	result->u = u0;
	result->v = _mm256_setzero_ps();
	result->t = d0;

	minMask = (minimum == d1);
	result->u = _mm256_merge_ps(minMask, result->u, (g_one8 - u1));
	result->v = _mm256_merge_ps(minMask, result->v, u1);
	result->t = _mm256_merge_ps(minMask, result->t, d1);

	minMask = (minimum == d2);
	result->u = _mm256_merge_ps(minMask, result->u, _mm256_setzero_ps());
	result->v = _mm256_merge_ps(minMask, result->v, g_one8 - u2);
	result->t = _mm256_merge_ps(minMask, result->t, d2);

	return mask & (result->t < radius);
}

/*---------------------------------------------------------------------------*/ 
