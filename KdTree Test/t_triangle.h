//
//

#if !defined(_T_TRIANGLE_H_)
#define _T_TRIANGLE_H_

#include "t_sse.h"
#include "t_vector.h"

/*---------------------------------------------------------------------------*/ 
struct TTriangle
{
	TTriangle() {}

	TTriangle(const TVector& p0, const TVector& p1, const TVector& p2);

	struct HitResult
	{
		HitResult() :t(FLT_MAX) {}
		inline float GetDist() { return t; }
		float u, v, t;
	};

	bool HitTest(const TVector& orig, const TVector& dir, HitResult* result) const;
	bool HitTest(const TVector& orig, const TVector& dir, const TVector& scale, HitResult* result) const;
	bool OcclusionTest(const TVector& orig, const TVector& dir) const;
	bool IntersectTest(const TAABB& aabb) const;
	bool NearestTest(const TVector& orig, float radius, HitResult* result) const;

	TAABB GetAABB() const;

	TVector pos0;
	TVector pos1;
	TVector pos2;

private:
	bool HitTest(const TVector& p0, const TVector& p1, const TVector& p2, const TVector& orig, const TVector& dir, HitResult* result) const;
};

#endif
