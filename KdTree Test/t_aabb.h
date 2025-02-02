//
//

#if !defined(_T_AABB_H_)
#define _T_AABB_H_

#include "t_sse.h"
#include "t_vector.h"

/*---------------------------------------------------------------------------*/ 
struct TAABB
{
	struct HitResult
	{
		HitResult() :t(FLT_MAX) {}
		inline float GetDist() { return t; }
		float t;
	};

	TAABB();
	TAABB(const float* mini, const float* maxi);

	void ResetAABB();
	void AddPoint(const float* p);
	void AddAABB(const TAABB& aabb);

	TAABB SplitLeft(int n, float split) const;
	TAABB SplitRight(int n, float split) const;

	bool IntersectTest(const TAABB& aabb) const;

	bool HitTest(const float* orig, const float* dir, HitResult* result) const;
	bool HitTest(const float* dir, const float* mini, const float* maxi, HitResult* result) const;

	TAABB operator+(const float pos[3]) const;

	float minimum[3];
	float maximum[3];

	TVector CalcCenter() const;
	float CalcRadius() const;
	TVector CalcSize() const;
	float CalcArea() const;
};

typedef	std::vector<TAABB> aabb_list;
typedef	aabb_list::iterator aabb_list_iter;

#endif
