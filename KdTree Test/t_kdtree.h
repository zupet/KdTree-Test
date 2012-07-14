//
//

#if !defined(_T_KDTREE_H_)
#define _T_KDTREE_H_

#include "t_sse.h"

typedef unsigned int	uint;

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

	void		ResetAABB();
	void		AddPoint(const float* p);
	void		AddAABB(const TAABB& aabb);

	TAABB		SplitLeft(int n, float split) const;
	TAABB		SplitRight(int n, float split) const;

	bool		IntersectTest(const TAABB& aabb) const;

	bool		HitTest(const float* orig, const float* dir, HitResult* result) const;

	TAABB		operator+(const float pos[3]) const;

	float		minimum[3];
	float		maximum[3];

	D3DXVECTOR3	CalcCenter() const;
	float		CalcRadius() const;
	D3DXVECTOR3	CalcSize() const;
};

typedef	std::vector<TAABB>		aabb_list;
typedef	aabb_list::iterator		aabb_list_iter;

/*---------------------------------------------------------------------------*/ 
struct TTriangle
{
	struct HitResult
	{
		HitResult() :t(FLT_MAX) {}
		inline float GetDist() { return t; }
		float u, v, t;
	};

	struct HitResult4
	{
		HitResult4() : t(g_fltMax4) {}
		inline __m128 GetDist() { return t; }
		inline void MergeResult(const __m128& mask, const HitResult4& result)
		{
			u = _mm_merge_ps(mask, u, result.u);
			v = _mm_merge_ps(mask, v, result.v);
			t = _mm_merge_ps(mask, t, result.t);
		}
		__m128 u, v, t;
	};

	struct HitResult8
	{
		HitResult8() : t(g_fltMax8) {}
		inline __m256 GetDist() { return t; }
		inline void MergeResult(const __m256& mask, const HitResult8& result)
		{
			u = _mm256_merge_ps(mask, u, result.u);
			v = _mm256_merge_ps(mask, v, result.v);
			t = _mm256_merge_ps(mask, t, result.t);
		}

		__m256 u, v, t;
	};

	struct THit
	{
		THit() {}

		THit(const TTriangle & triangle);

		bool	HitTest(const D3DXVECTOR3& orig, const D3DXVECTOR3& dir, HitResult* result) const;
		bool	OcclusionTest(const D3DXVECTOR3& orig, const D3DXVECTOR3& dir) const;

		__m128	HitTest4(__m128 mask, const TPoint4& orig, const D3DXVECTOR3& dir, HitResult4* result) const;
		__m128	OcclusionTest4(__m128 mask, const TPoint4& orig, const D3DXVECTOR3& dir) const;

		__m256	HitTest8(__m256 mask, const TPoint8& orig, const D3DXVECTOR3& dir, HitResult8* result) const;
		__m256	OcclusionTest8(const __m256& mask, const TPoint8& orig, const D3DXVECTOR3& dir) const;

		float nu; //used to store normal data
		float nv; //used to store normal data
		float np; //used to store vertex data
		float pu; //used to store vertex data
		float pv; //used to store vertex data
		int ci; //used to store edges data
		float e0u; //used to store edges data
		float e0v; //used to store edges data
		float e1u; //used to store edges data
		float e1v; //used to store edges data
	};

	TTriangle() {}

	TTriangle(const D3DXVECTOR3& p0, const D3DXVECTOR3& p1, const D3DXVECTOR3& p2);

	TAABB	GetAABB() const;
	bool	IntersectTest(const TAABB& aabb) const;

	bool	NearestTest(const D3DXVECTOR3& orig, float radius, HitResult* result) const;
	__m128	NearestTest4(__m128 mask, const TPoint4& orig, __m128 radius, HitResult4* result) const;
	__m256	NearestTest8(const __m256& mask, const TPoint8& orig, const __m256& radius, HitResult8* result) const;

	D3DXVECTOR3 pos0;
	D3DXVECTOR3 pos1;
	D3DXVECTOR3 pos2;
};

/*---------------------------------------------------------------------------*/ 
struct TKdEvent
{
	enum EVENT_STATE { STATE_OUT, STATE_ON, STATE_IN };

	TKdEvent(float split, EVENT_STATE state, uint triangle)
	{
		m_split = split;
		m_data = (state << 30) + ((DWORD)triangle & 0x3FFFFFFF);
	}

	EVENT_STATE GetState() const
	{
		return (TKdEvent::EVENT_STATE)(m_data >> 30);
	}

	UINT GetIndex() const
	{
		return (m_data & 0x3FFFFFFF);
	}

	bool operator<(const TKdEvent& right) const
	{
		if(m_split == right.m_split)
		{
			if(GetState() == right.GetState())
			{
				return GetIndex() < right.GetIndex();
			}
			else
			{
				return GetState() < right.GetState();
			}
		}
		else
		{
			return m_split < right.m_split;
		}
	}

	float		m_split;
	DWORD		m_data;
};

typedef	std::vector<TKdEvent>			kd_event_list;
typedef	kd_event_list::const_iterator	kd_event_list_iter;

/*---------------------------------------------------------------------------*/ 
struct TKdSplit
{
public:
	enum SPLIT_AXIS { SPLIT_X = 0, SPLIT_Y = 1, SPLIT_Z = 2, SPLIT_END = 3 };

	void SetAxis(SPLIT_AXIS axis)
	{
		m_data = (m_data & 0x3FFFFFFF) + (axis << 30);
	}

	void SetChildren(uint children)
	{
		m_data = (m_data & 0xC0000000) + ((DWORD)children & 0x3FFFFFFF);
	}

	void SetCount(uint count)
	{	
		m_count = (UINT)count;
	}

	SPLIT_AXIS GetAxis() const
	{
		return (SPLIT_AXIS)(m_data >> 30);
	}

	UINT GetChildren() const
	{
		return (m_data & 0x3FFFFFFF);
	}

	union {
		float	m_split;
		UINT	m_count;
	};
	DWORD		m_data;
};

/*---------------------------------------------------------------------------*/ 
template<typename TObject>
class TKdTree
{
public:
	struct HitResult
	{
		inline float GetDist() { return result.GetDist(); }
		typename TObject::HitResult		result;
		uint							object;
	};

	struct HitResult4
	{
		inline __m128 GetDist() { return result.GetDist(); }
		inline void MergeResult(__m128 mask, const HitResult4& hitResult)
		{
			result.MergeResult(mask, hitResult.result);
			object = _mm_merge_ps(mask, object, hitResult.object);
		}
		typename TObject::HitResult4	result;
		__m128							object;
	};

	struct HitResult8
	{
		inline __m256 GetDist() { return result.GetDist(); }
		inline void MergeResult(const __m128& mask, const HitResult8& hitResult)
		{
			result.MergeResult(mask, hitResult.result);
			object = _mm_merge_ps(mask, object, hitResult.object);
		}
		typename TObject::HitResult8	result;
		__m256							object;
	};

	TKdTree();

	void			ResetKdTree();

	void			SaveKdTree(LPCTSTR filename);
	bool			LoadKdTree(LPCTSTR filename);

	void			AddObject(const TObject& object);
	TObject&		GetObject(uint offset);

	void			BuildTree(UINT depth=UINT_MAX);
	bool			HitTest(const D3DXVECTOR3& orig, const D3DXVECTOR3& dir, HitResult* result) const;
	bool			OcclusionTest(const D3DXVECTOR3& orig, const D3DXVECTOR3& dir) const;
	bool			NearestTest(const D3DXVECTOR3& orig, float radius, HitResult* result) const;

	__m128			HitTest4(__m128 mask, const TPoint4& orig, const D3DXVECTOR3& dir, HitResult4* result) const;
	__m128			OcclusionTest4(__m128 mask, const TPoint4& orig, const D3DXVECTOR3& dir) const;
	__m128			NearestTest4(__m128 mask, const TPoint4& orig, float radius, HitResult4* result) const;

	__m256			HitTest8(const __m256& mask, const TPoint8& orig, const D3DXVECTOR3& dir, HitResult8* result) const;
	__m256			OcclusionTest8(const __m256& mask, const TPoint8& orig, const D3DXVECTOR3& dir) const;
	__m256			NearestTest8(const __m256& mask, const TPoint8& orig, float radius, HitResult8* result) const;

	const TAABB&	GetAABB() const { return m_aabb; }
	bool			IsEmpty() const { return m_leafList.empty(); }
	uint			GetObjectCount() const { return m_objectList.size(); }

	uint			m_maxLeafCount;

protected:
	void			BuildTree(uint tree, const kd_event_list& xList, const kd_event_list& yList, const kd_event_list& zList, const TAABB& aabb, const aabb_list& aabbList, UINT depth);
	uint			InitKdSplit(uint tree);
	float			CalcMinSAH(TKdSplit::SPLIT_AXIS axis, const kd_event_list& splitList, const TAABB& aabb, float* minSplit);
	float			CalcSAH(uint n_l, uint n_c, uint n_r, float p_l, float p_r);

	typedef	std::vector<TObject>			object_list;
	typedef	typename object_list::iterator	object_list_iter;

	object_list			m_objectList;

	typedef	typename TObject::THit			THit;
	typedef	std::vector<THit>				hit_list;
	typedef	typename hit_list::iterator		hit_list_iter;

	hit_list			m_hitList;

	typedef	std::vector<TKdSplit>			split_list;
	typedef	split_list::iterator			split_list_iter;

	split_list			m_splitList;

	typedef	std::vector<uint>				offset_list;
	typedef	offset_list::iterator			offset_list_iter;

	offset_list			m_leafList;

	TAABB				m_aabb;
};

extern	uint			g_calls;
extern	uint			g_left;
extern	uint			g_right;
extern	uint			g_push;
extern	uint			g_nodes;
extern	uint			g_objects;


#include "t_kdtree.inl"

/*---------------------------------------------------------------------------*/ 

#endif //_T_KDTREE_H_
