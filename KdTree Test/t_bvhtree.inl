
//
//


/*---------------------------------------------------------------------------*/ 
/*                                                                           */ 
/*---------------------------------------------------------------------------*/ 
template<class TObject>
TBVHTree<TObject>::TBVHTree()
{
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TBVHTree<TObject>::ResetTree()
{
	object_list	empty_object_list;
	split_list	empty_split_list;
	offset_list	empty_offset_list;
	

	m_objectList.swap(empty_object_list);
	m_volumeList.swap(empty_split_list);
	m_leafList.swap(empty_offset_list);

	m_aabb.ResetAABB();
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TBVHTree<TObject>::AddObject(const TObject& triangle)
{
	m_objectList.push_back(triangle);
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
TObject& TBVHTree<TObject>::GetObject(uint offset)
{
	return m_objectList[offset];
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TBVHTree<TObject>::BuildTree(UINT depth)
{
	uint size = (uint)m_objectList.size();
	
	aabb_list	aabbList;
	aabbList.reserve(size);
	for(uint i=0; i<size; ++i)
	{
		const TAABB& triangle = m_objectList[i].GetAABB();
		aabbList.push_back(triangle);
		m_aabb.AddAABB(triangle);
	}

	m_volumeList.resize(2);
	m_leafList.clear();
	m_maxLeafCount = 0;

	BuildTree(0, aabbList, depth);
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TBVHTree<TObject>::BuildTree(uint tree, const aabb_list& aabbList, UINT depth)
{
	float minRatio = 1.0f;
	TVolume::SPLIT_AXIS minAxis = TVolume::SPLIT_END;
	float minSplit = 0;

	if(depth > 0)
	{
		float ratio[3];
		float split[3];
		ratio[0] = CalcMinSAH(0, xList, split+0);
		ratio[1] = CalcMinSAH(1, yList, split+1);
		ratio[2] = CalcMinSAH(2, zList, split+2);

		for(int i=0; i<3; ++i)
		{
			if(ratio[i] < minRatio)
			{
				minRatio = ratio[i];
				minAxis = (TVolume::SPLIT_AXIS)i;
				minSplit = split[i];
			}
		}
	}

	if(minAxis != TVolume::SPLIT_END)
	{
		uint children = (uint)m_volumeList.size();

		m_volumeList[tree].SetAxis(minAxis);
		m_volumeList[tree].SetChildren(children);
		m_volumeList[tree].m_volume = minSplit;

		bvh_event_list xTemp, yTemp, zTemp;
		xTemp.reserve(xList.size());
		yTemp.reserve(yList.size());
		zTemp.reserve(zList.size());

		TAABB leftAABB = aabb.SplitLeft(minAxis, minSplit);
		TAABB rightAABB = aabb.SplitRight(minAxis, minSplit);

		m_volumeList.resize(m_volumeList.size() + 2);

		xTemp.clear(); yTemp.clear(); zTemp.clear();
		for(bvh_event_list_iter iter=xList.begin(); iter!=xList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(bound.minimum[minAxis] < minSplit || bound.maximum[minAxis] == minSplit)
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(leftAABB))
					xTemp.push_back(*iter);
			}
		}
		for(bvh_event_list_iter iter=yList.begin(); iter!=yList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(bound.minimum[minAxis] < minSplit || bound.maximum[minAxis] == minSplit)
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(leftAABB))
					yTemp.push_back(*iter);
			}
		}
		for(bvh_event_list_iter iter=zList.begin(); iter!=zList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(bound.minimum[minAxis] < minSplit || bound.maximum[minAxis] == minSplit)
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(leftAABB))
					zTemp.push_back(*iter);
			}
		}
		BuildTree(children+0, xTemp, yTemp, zTemp, aabb.SplitLeft(minAxis, minSplit), aabbList, depth-1);

		xTemp.clear(); yTemp.clear(); zTemp.clear();
		for(bvh_event_list_iter iter=xList.begin(); iter!=xList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(minSplit < bound.maximum[minAxis] || minSplit == bound.minimum[minAxis])
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(rightAABB))
					xTemp.push_back(*iter);
			}
		}
		for(bvh_event_list_iter iter=yList.begin(); iter!=yList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(minSplit < bound.maximum[minAxis] || minSplit == bound.minimum[minAxis])
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(rightAABB))
					yTemp.push_back(*iter);
			}
		}
		for(bvh_event_list_iter iter=zList.begin(); iter!=zList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(minSplit < bound.maximum[minAxis] || minSplit == bound.minimum[minAxis])
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(rightAABB))
					zTemp.push_back(*iter);
			}
		}
		BuildTree(children+1, xTemp, yTemp, zTemp, aabb.SplitRight(minAxis, minSplit), aabbList, depth-1);
	}
	else
	{
		offset_list objectList;

		objectList.reserve(xList.size() + yList.size() + zList.size());

		for(bvh_event_list_iter iter=xList.begin(); iter!=xList.end(); ++iter)
		{
			objectList.push_back(iter->GetIndex());
		}
		for(bvh_event_list_iter iter=yList.begin(); iter!=yList.end(); ++iter)
		{
			objectList.push_back(iter->GetIndex());
		}
		for(bvh_event_list_iter iter=zList.begin(); iter!=zList.end(); ++iter)
		{
			objectList.push_back(iter->GetIndex());
		}

		std::sort(objectList.begin(), objectList.end());
		offset_list_iter end = std::unique(objectList.begin(), objectList.end());

		uint count = (uint)(end - objectList.begin());

		m_volumeList[tree].SetAxis(TVolume::SPLIT_END);
		m_volumeList[tree].SetChildren((uint)m_leafList.size());
		m_volumeList[tree].SetCount(count);

		m_leafList.insert(m_leafList.end(), objectList.begin(), end);

		m_maxLeafCount = max(m_maxLeafCount, (uint)(end - objectList.begin()));
	}
}

template<class TObject>
float TBVHTree<TObject>::CalcMinSAH(uint axis, offset_list& sorted_list)
{
	return 0;
}

template<class TObject>
float TBVHTree<TObject>::CalcSAH(uint n_l, uint n_c, uint n_r, float p_l, float p_r)
{
	return 12500000 + min(p_l*(n_l+n_c) + p_r*n_r, p_l*n_l + p_r*(n_r+n_c));
}
