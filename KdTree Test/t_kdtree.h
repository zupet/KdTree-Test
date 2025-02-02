//
//

#if !defined(_T_KDTREE_H_)
#define _T_KDTREE_H_

#include "t_sse.h"
#include "t_aabb.h"
#include "t_triangle.h"

typedef unsigned int	uint;

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
	bool			HitTest(const TVector& orig, const TVector& dir, HitResult* result) const;
	bool			HitTest(const TVector& orig, const TVector& dir, const TVector& scale, HitResult* result) const;
	bool			OcclusionTest(const TVector& orig, const TVector& dir) const;
	bool			NearestTest(const TVector& orig, float radius, HitResult* result) const;

	const TAABB&	GetAABB() const { return m_aabb; }
	bool			IsEmpty() const { return m_leafList.empty(); }
	uint			GetObjectCount() const { return m_objectList.size(); }

	uint			m_maxLeafCount;

protected:
	void			BuildTree(uint tree, const kd_event_list& xList, const kd_event_list& yList, const kd_event_list& zList, const TAABB& aabb, const aabb_list& aabbList, UINT depth);
	float			CalcMinSAH(TKdSplit::SPLIT_AXIS axis, const kd_event_list& splitList, const TAABB& aabb, float* minSplit);
	float			CalcSAH(uint n_l, uint n_c, uint n_r, float p_l, float p_r);

	typedef	std::vector<TObject>			object_list;
	typedef	typename object_list::iterator	object_list_iter;

	object_list			m_objectList;

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
