//
//

#if !defined(_T_BVHTREE_H_)
#define _T_BVHTREE_H_

#include "t_sse.h"
#include "t_aabb.h"
#include "t_triangle.h"

typedef unsigned int	uint;

/*---------------------------------------------------------------------------*/ 
struct TVolume : public TAABB
{
public:

	void SetChildren(uint children)
	{
		m_children = children;
	}

	uint GetChildren() const
	{
		return m_children;
	}

	void SetAsLeaf()
	{
		m_data = 0x80000000 + (m_data & 0x7fffffff);
	}

	bool IsLeaf()
	{
		return m_data & 0x80000000 ? true : false;
	}

	void SetCount(uint count)
	{	
		m_data = (m_data & 0x80000000) + (count & 0x7fffffff);
	}

	uint GetCount()
	{	
		return m_data & 0x7fffffff;
	}

	uint m_children;
	uint m_data;
};

/*---------------------------------------------------------------------------*/ 
template<typename TObject>
class TBVHTree
{
public:
	struct HitResult
	{
		inline float GetDist() { return result.GetDist(); }
		typename TObject::HitResult		result;
		uint							object;
	};

	TBVHTree();

	void			ResetTree();

	void			SaveTree(LPCTSTR filename) {}
	bool			LoadTree(LPCTSTR filename) { return false; }

	void			AddObject(const TObject& object);
	TObject&		GetObject(uint offset);

	void			BuildTree(UINT depth=UINT_MAX);
	bool			HitTest(const TVector& orig, const TVector& dir, HitResult* result) const;
	bool			HitTest(const TVector& orig, const TVector& dir, const TVector& scale, HitResult* result) const;

	const TAABB&	GetAABB() const { return m_aabb; }
	bool			IsEmpty() const { return m_leafList.empty(); }
	uint			GetObjectCount() const { return m_objectList.size(); }

	uint			m_maxLeafCount;

protected:
	typedef	std::vector<TObject>			object_list;
	typedef	typename object_list::iterator	object_list_iter;

	typedef	std::vector<TVolume>			volume_list;
	typedef	volume_list::iterator			volume_list_iter;

	typedef	std::vector<uint>				offset_list;
	typedef	offset_list::iterator			offset_list_iter;

	void			BuildTree(uint tree, const aabb_list& aabbList, UINT depth);
	float			CalcMinSAH(uint axis, offset_list& sorted_list);
	float			CalcSAH(uint n_l, uint n_c, uint n_r, float p_l, float p_r);

	object_list			m_objectList;

	volume_list			m_volumeList;

	offset_list			m_leafList;

	TAABB				m_aabb;
};

extern	uint			g_calls;
extern	uint			g_left;
extern	uint			g_right;
extern	uint			g_push;
extern	uint			g_nodes;
extern	uint			g_objects;


#include "t_bvhtree.inl"

/*---------------------------------------------------------------------------*/ 

#endif
