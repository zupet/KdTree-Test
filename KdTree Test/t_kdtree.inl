//
//


/*---------------------------------------------------------------------------*/ 
struct TTraversalInfo
{
	float		t_min, t_max;
	TKdSplit	split;
};

struct TNearestInfo
{
	float	distance;
	uint	offset;
};

/*---------------------------------------------------------------------------*/ 
/*                                                                           */ 
/*---------------------------------------------------------------------------*/ 
template<class TObject>
TKdTree<TObject>::TKdTree()
{
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TKdTree<TObject>::ResetKdTree()
{
	object_list	empty_object_list;
	split_list	empty_split_list;
	offset_list	empty_offset_list;
	

	m_objectList.swap(empty_object_list);
	m_splitList.swap(empty_split_list);
	m_leafList.swap(empty_offset_list);

	m_aabb.ResetAABB();
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TKdTree<TObject>::SaveKdTree(LPCTSTR filename)
{
	if(IsEmpty() == false)
	{
		FILE* file;
		if(_tfopen_s(&file, filename, _T("wb")) == 0)
		{
			fwrite(&m_aabb, sizeof(m_aabb), 1, file);

			UINT size = (UINT)m_objectList.size();
			fwrite(&size, sizeof(size), 1, file);
			fwrite(&m_objectList[0], sizeof(m_objectList[0]), size, file);

			size = (UINT)m_splitList.size();
			fwrite(&size, sizeof(size), 1, file);
			fwrite(&m_splitList[0], sizeof(m_splitList[0]), size, file);

			size = (UINT)m_leafList.size();
			fwrite(&size, sizeof(size), 1, file);
			fwrite(&m_leafList[0], sizeof(m_leafList[0]), size, file);

			fclose(file);
		}
	}
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
bool TKdTree<TObject>::LoadKdTree(LPCTSTR filename)
{
	ResetKdTree();

	FILE* file;
	if(_tfopen_s(&file, filename, _T("rb")) == 0)
	{
		fread(&m_aabb, sizeof(m_aabb), 1, file);

		UINT size;
		fread(&size, sizeof(size), 1, file);
		m_objectList.resize(size);
		fread(&m_objectList[0], sizeof(m_objectList[0]), size, file);

		fread(&size, sizeof(size), 1, file);
		m_splitList.resize(size);
		fread(&m_splitList[0], sizeof(m_splitList[0]), size, file);

		fread(&size, sizeof(size), 1, file);
		m_leafList.resize(size);
		fread(&m_leafList[0], sizeof(m_leafList[0]), size, file);

		fclose(file);

		return true;
	}
	return false;
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TKdTree<TObject>::AddObject(const TObject& triangle)
{
	m_objectList.push_back(triangle);
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
TObject& TKdTree<TObject>::GetObject(uint offset)
{
	return m_objectList[offset];
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TKdTree<TObject>::BuildTree(UINT depth)
{
	uint size = (uint)m_objectList.size();
	
	kd_event_list	xList;
	kd_event_list	yList;
	kd_event_list	zList;
	aabb_list	aabbList;
	
	xList.reserve(size * 2);
	yList.reserve(size * 2);
	zList.reserve(size * 2);
	aabbList.reserve(size);

	for(uint i=0; i<size; ++i)
	{
		const TAABB& triangle = m_objectList[i].GetAABB();

		aabbList.push_back(triangle);

		if(triangle.minimum[0] != triangle.maximum[0])
		{
			xList.push_back(TKdEvent(triangle.minimum[0], TKdEvent::STATE_IN, i));
			xList.push_back(TKdEvent(triangle.maximum[0], TKdEvent::STATE_OUT, i));
		}
		else
		{
			xList.push_back(TKdEvent(triangle.maximum[0], TKdEvent::STATE_ON, i));
		}

		if(triangle.minimum[1] != triangle.maximum[1])
		{
			yList.push_back(TKdEvent(triangle.minimum[1], TKdEvent::STATE_IN, i));
			yList.push_back(TKdEvent(triangle.maximum[1], TKdEvent::STATE_OUT, i));
		}
		else
		{
			yList.push_back(TKdEvent(triangle.maximum[1], TKdEvent::STATE_ON, i));
		}

		if(triangle.minimum[2] != triangle.maximum[2])
		{
			zList.push_back(TKdEvent(triangle.minimum[2], TKdEvent::STATE_IN, i));
			zList.push_back(TKdEvent(triangle.maximum[2], TKdEvent::STATE_OUT, i));
		}
		else
		{
			zList.push_back(TKdEvent(triangle.maximum[2], TKdEvent::STATE_ON, i));
		}

		m_aabb.AddAABB(triangle);
	}

	sort(xList.begin(), xList.end());
	sort(yList.begin(), yList.end());
	sort(zList.begin(), zList.end());

	m_splitList.resize(2);
	m_leafList.clear();
	m_maxLeafCount = 0;

	BuildTree(0, xList, yList, zList, m_aabb, aabbList, depth);
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
void TKdTree<TObject>::BuildTree(uint tree, const kd_event_list& xList, const kd_event_list& yList, const kd_event_list& zList, const TAABB& aabb, const aabb_list& aabbList, UINT depth)
{
	float minRatio = 1.0f;
	TKdSplit::SPLIT_AXIS minAxis = TKdSplit::SPLIT_END;
	float minSplit = 0;

	if(depth > 0)
	{
		float ratio[3];
		float split[3];
		ratio[0] = CalcMinSAH(TKdSplit::SPLIT_X, xList, aabb, split+0);
		ratio[1] = CalcMinSAH(TKdSplit::SPLIT_Y, yList, aabb, split+1);
		ratio[2] = CalcMinSAH(TKdSplit::SPLIT_Z, zList, aabb, split+2);

		for(int i=0; i<3; ++i)
		{
			if(ratio[i] < minRatio)
			{
				minRatio = ratio[i];
				minAxis = (TKdSplit::SPLIT_AXIS)i;
				minSplit = split[i];
			}
		}
	}

	if(minAxis != TKdSplit::SPLIT_END)
	{
		uint children = (uint)m_splitList.size();

		m_splitList[tree].SetAxis(minAxis);
		m_splitList[tree].SetChildren(children);
		m_splitList[tree].m_split = minSplit;

		kd_event_list xTemp, yTemp, zTemp;
		xTemp.reserve(xList.size());
		yTemp.reserve(yList.size());
		zTemp.reserve(zList.size());

		TAABB leftAABB = aabb.SplitLeft(minAxis, minSplit);
		TAABB rightAABB = aabb.SplitRight(minAxis, minSplit);

		m_splitList.resize(m_splitList.size() + 2);

		xTemp.clear(); yTemp.clear(); zTemp.clear();
		for(kd_event_list_iter iter=xList.begin(); iter!=xList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(bound.minimum[minAxis] < minSplit || bound.maximum[minAxis] == minSplit)
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(leftAABB))
					xTemp.push_back(*iter);
			}
		}
		for(kd_event_list_iter iter=yList.begin(); iter!=yList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(bound.minimum[minAxis] < minSplit || bound.maximum[minAxis] == minSplit)
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(leftAABB))
					yTemp.push_back(*iter);
			}
		}
		for(kd_event_list_iter iter=zList.begin(); iter!=zList.end(); ++iter)
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
		for(kd_event_list_iter iter=xList.begin(); iter!=xList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(minSplit < bound.maximum[minAxis] || minSplit == bound.minimum[minAxis])
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(rightAABB))
					xTemp.push_back(*iter);
			}
		}
		for(kd_event_list_iter iter=yList.begin(); iter!=yList.end(); ++iter)
		{
			const TAABB& bound = aabbList[iter->GetIndex()];
			if(minSplit < bound.maximum[minAxis] || minSplit == bound.minimum[minAxis])
			{
				if(m_objectList[iter->GetIndex()].IntersectTest(rightAABB))
					yTemp.push_back(*iter);
			}
		}
		for(kd_event_list_iter iter=zList.begin(); iter!=zList.end(); ++iter)
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

		for(kd_event_list_iter iter=xList.begin(); iter!=xList.end(); ++iter)
		{
			objectList.push_back(iter->GetIndex());
		}
		for(kd_event_list_iter iter=yList.begin(); iter!=yList.end(); ++iter)
		{
			objectList.push_back(iter->GetIndex());
		}
		for(kd_event_list_iter iter=zList.begin(); iter!=zList.end(); ++iter)
		{
			objectList.push_back(iter->GetIndex());
		}

		std::sort(objectList.begin(), objectList.end());
		offset_list_iter end = std::unique(objectList.begin(), objectList.end());

		uint count = (uint)(end - objectList.begin());

		m_splitList[tree].SetAxis(TKdSplit::SPLIT_END);
		m_splitList[tree].SetChildren((uint)m_leafList.size());
		m_splitList[tree].SetCount(count);

		m_leafList.insert(m_leafList.end(), objectList.begin(), end);

		m_maxLeafCount = max(m_maxLeafCount, (uint)(end - objectList.begin()));
	}
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
bool TKdTree<TObject>::HitTest(const TVector& orig, const TVector& dir, HitResult* hitResult) const
{
	++g_calls;

	float o[3] = { orig.x, orig.y, orig.z };

	float inv[3];
	int order[3][2];

	float t_max = FLT_MAX;
	float t_min = 0;

	for(int axis=0; axis<3; ++axis)
	{
		float temp = dir[axis] ? 1/dir[axis] : (dir[axis] < 0 ? -FLT_MAX : FLT_MAX);
		inv[axis] = temp;

		order[axis][0] = 0 < temp ? 0 : 1;
		order[axis][1] = 0 < temp ? 1 : 0;

		float left	= (m_aabb.minimum[axis] - o[axis]) * inv[axis];
		float right	= (m_aabb.maximum[axis] - o[axis]) * inv[axis];

		t_max = min(t_max, max(left, right));
	}

	if(0 <= t_max)
	{
		const TKdSplit*	splitList	= &m_splitList[0];
		const uint*		leafList	= &m_leafList[0];
		const TObject*	objectList	= &m_objectList[0];

		uint traversal = 0;
		TTraversalInfo traversalStack[100];

		uint mailbox = 0;
		uint mailboxStack[8] = { 
				(uint)-1, (uint)-1, (uint)-1, (uint)-1, 
				(uint)-1, (uint)-1, (uint)-1, (uint)-1, 
		};

		TKdSplit kdSplit = splitList[0];

		while(true)
		{
loop:
			TKdSplit::SPLIT_AXIS axis = kdSplit.GetAxis();
			if(axis != TKdSplit::SPLIT_END)
			{
				++g_nodes;

				float t_at_split = (kdSplit.m_split - o[axis]) * inv[axis];
				if(t_max <= t_at_split)
				{
					++g_left;
					kdSplit = splitList[kdSplit.GetChildren() + order[axis][0]];
				}
				else if(t_at_split <= t_min)
				{
					++g_right;
					kdSplit = splitList[kdSplit.GetChildren() + order[axis][1]];
				}
				else
				{
					++g_push;
					traversalStack[traversal].t_max = t_max;
					traversalStack[traversal].t_min = t_at_split;
					traversalStack[traversal].split = splitList[kdSplit.GetChildren() + order[axis][1]];
					++traversal;

					t_max = t_at_split;
					kdSplit = splitList[kdSplit.GetChildren() + order[axis][0]];
				}
			}
			else
			{
				if(t_min < hitResult->GetDist())
				{
					const uint* candidate = leafList + kdSplit.GetChildren();

					for(UINT i=0; i<kdSplit.m_count; ++i)
					{
						int index = candidate[i];

						if(	mailboxStack[0] == index || mailboxStack[1] == index
							|| mailboxStack[2] == index || mailboxStack[3] == index
							|| mailboxStack[4] == index || mailboxStack[5] == index
							|| mailboxStack[6] == index || mailboxStack[7] == index
							)
						{
							continue;
						}
						else
						{
							mailboxStack[mailbox++ & (8-1)] = index;
						}

						++g_objects;

						TObject::HitResult testResult;
						if(objectList[index].HitTest(orig, dir, &testResult) && (testResult.GetDist() < hitResult->GetDist()))
						{
							hitResult->object	= index;
							hitResult->result	= testResult;
						}
					}
				}
				while(traversal)
				{
					--traversal;
					t_min = traversalStack[traversal].t_min;

					if(t_min < hitResult->GetDist())
					{
						t_max = traversalStack[traversal].t_max;
						kdSplit = traversalStack[traversal].split;

						goto loop;
					}
				}

				break;
			}
		}
		
		return hitResult->GetDist() != FLT_MAX;
	}
	return false;
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
bool TKdTree<TObject>::HitTest(const TVector& orig, const TVector& dir, const TVector& scale, HitResult* hitResult) const
{
	++g_calls;

	float o[3] = { orig.x, orig.y, orig.z };

	float inv[3];
	int order[3][2];

	float t_max = FLT_MAX;
	float t_min = 0;

	for(int axis=0; axis<3; ++axis)
	{
		float temp = dir[axis] ? 1/dir[axis] : (dir[axis] < 0 ? -FLT_MAX : FLT_MAX);
		inv[axis] = temp;

		order[axis][0] = 0 < temp ? 0 : 1;
		order[axis][1] = 0 < temp ? 1 : 0;

		float left	= (m_aabb.minimum[axis] * scale[axis] - o[axis]) * inv[axis];
		float right	= (m_aabb.maximum[axis] * scale[axis] - o[axis]) * inv[axis];

		t_max = min(t_max, max(left, right));
	}

	if(0 <= t_max)
	{
		const TKdSplit*	splitList	= &m_splitList[0];
		const uint*		leafList	= &m_leafList[0];
		const TObject*	objectList	= &m_objectList[0];

		uint traversal = 0;
		TTraversalInfo traversalStack[100];

		uint mailbox = 0;
		uint mailboxStack[8] = { 
				(uint)-1, (uint)-1, (uint)-1, (uint)-1, 
				(uint)-1, (uint)-1, (uint)-1, (uint)-1, 
		};

		TKdSplit kdSplit = splitList[0];

		while(true)
		{
loop:
			TKdSplit::SPLIT_AXIS axis = kdSplit.GetAxis();
			if(axis != TKdSplit::SPLIT_END)
			{
				++g_nodes;

				float t_at_split = (kdSplit.m_split * scale[axis] - o[axis]) * inv[axis];
				if(t_max <= t_at_split)
				{
					++g_left;
					kdSplit = splitList[kdSplit.GetChildren() + order[axis][0]];
				}
				else if(t_at_split <= t_min)
				{
					++g_right;
					kdSplit = splitList[kdSplit.GetChildren() + order[axis][1]];
				}
				else
				{
					++g_push;
					traversalStack[traversal].t_max = t_max;
					traversalStack[traversal].t_min = t_at_split;
					traversalStack[traversal].split = splitList[kdSplit.GetChildren() + order[axis][1]];
					++traversal;

					t_max = t_at_split;
					kdSplit = splitList[kdSplit.GetChildren() + order[axis][0]];
				}
			}
			else
			{
				if(t_min < hitResult->GetDist())
				{
					const uint* candidate = leafList + kdSplit.GetChildren();

					for(UINT i=0; i<kdSplit.m_count; ++i)
					{
						int index = candidate[i];

						if(	mailboxStack[0] == index || mailboxStack[1] == index
							|| mailboxStack[2] == index || mailboxStack[3] == index
							|| mailboxStack[4] == index || mailboxStack[5] == index
							|| mailboxStack[6] == index || mailboxStack[7] == index
							)
						{
							continue;
						}
						else
						{
							mailboxStack[mailbox++ & (8-1)] = index;
						}

						++g_objects;

						TObject::HitResult testResult;
						if(objectList[index].HitTest(orig, dir, scale, &testResult) && (testResult.GetDist() < hitResult->GetDist()))
						{
							hitResult->object	= index;
							hitResult->result	= testResult;
						}
					}
				}
				while(traversal)
				{
					--traversal;
					t_min = traversalStack[traversal].t_min;

					if(t_min < hitResult->GetDist())
					{
						t_max = traversalStack[traversal].t_max;
						kdSplit = traversalStack[traversal].split;

						goto loop;
					}
				}

				break;
			}
		}
		
		return hitResult->GetDist() != FLT_MAX;
	}
	return false;
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
bool TKdTree<TObject>::OcclusionTest(const TVector& orig, const TVector& dir) const
{
	if(m_leafList.empty())
	{
		return false;
	}

	float o[3] = { orig.x, orig.y, orig.z };
	float inv[3];

	inv[0] = dir.x ? 1/dir.x : (dir.x < 0 ? -FLT_MAX : FLT_MAX);
	inv[1] = dir.y ? 1/dir.y : (dir.y < 0 ? -FLT_MAX : FLT_MAX);
	inv[2] = dir.z ? 1/dir.z : (dir.z < 0 ? -FLT_MAX : FLT_MAX);

	float x_max = dir.x ? max((m_aabb.minimum[0] - o[0]) * inv[0], (m_aabb.maximum[0] - o[0]) * inv[0]) : FLT_MAX;
	float y_max = dir.y ? max((m_aabb.minimum[1] - o[1]) * inv[1], (m_aabb.maximum[1] - o[1]) * inv[1]) : FLT_MAX;
	float z_max = dir.z ? max((m_aabb.minimum[2] - o[2]) * inv[2], (m_aabb.maximum[2] - o[2]) * inv[2]) : FLT_MAX;

	float t_max = min(x_max, min(y_max, z_max));
	float t_min = 0;

	if(0 <= t_max)
	{
		const TKdSplit*	splitList	= &m_splitList[0];
		const uint*		leafList	= &m_leafList[0];
		const TObject*	objectList	= &m_objectList[0];

		uint offset = 0;
		uint traversal = 0;
		TTraversalInfo traversalStack[100];

		int order[3][2];

		uint mailbox = 0;
		uint mailboxStack[8] = { 
				(uint)-1, (uint)-1, (uint)-1, (uint)-1, 
				(uint)-1, (uint)-1, (uint)-1, (uint)-1, 
		};

		for(int axis=0; axis<3; ++axis)
		{
			if(0 < inv[axis])
			{
				order[axis][0] = 0;
				order[axis][1] = 1;
			}
			else
			{
				order[axis][0] = 1;
				order[axis][1] = 0;
			}
		}

		while(true)
		{
			TKdSplit::SPLIT_AXIS axis = splitList[offset].GetAxis();
			UINT children = splitList[offset].GetChildren();
			if(axis != TKdSplit::SPLIT_END)
			{
				float t_at_split = (splitList[offset].m_split - o[axis]) * inv[axis];
				if(t_max <= t_at_split)
				{
					offset = children + order[axis][0];
				}
				else if(t_at_split <= t_min)
				{
					offset = children + order[axis][1];
				}
				else
				{
					traversalStack[traversal].t_max = t_max;
					traversalStack[traversal].t_min = t_at_split;
					traversalStack[traversal].offset = children + order[axis][1];
					++traversal;

					t_max = t_at_split;
					offset = children + order[axis][0];
				}
			}
			else
			{
				const uint* candidate = leafList + children;

				for(UINT i=0; i<splitList[offset].m_count; ++i)
				{
					int index = candidate[i];

					if(	mailboxStack[0] == index || mailboxStack[1] == index
						|| mailboxStack[2] == index || mailboxStack[3] == index
						|| mailboxStack[4] == index || mailboxStack[5] == index
						|| mailboxStack[6] == index || mailboxStack[7] == index
						)
					{
						continue;
					}
					else
					{
						mailboxStack[mailbox++ & (8-1)] = index;
					}

					if(hitList[index].OcclusionTest(orig, dir) == true)
					{
						return true;
					}
				}
				if(traversal)
				{
					--traversal;
					t_max = traversalStack[traversal].t_max;
					t_min = traversalStack[traversal].t_min;
					offset = traversalStack[traversal].offset;
				}
				else
				{
					break;
				}
			}	
		}
	}
	return false;
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
bool TKdTree<TObject>::NearestTest(const TVector& orig, float radius, HitResult* hitResult) const
{
	if(!m_leafList.empty())
	{
		const TKdSplit*	splitList	= &m_splitList[0];
		const uint*		leafList	= &m_leafList[0];
		const TObject*	objectList	= &m_objectList[0];

		uint offset = 0;
		float distance = 0;

		uint traversal = 0;
		TNearestInfo nearestStack[100];

		while(true)
		{
			TKdSplit::SPLIT_AXIS axis	= splitList[offset].GetAxis();
			UINT children = splitList[offset].GetChildren();
			if(axis != TKdSplit::SPLIT_END)
			{
				if(distance < radius)
				{
					float split = splitList[offset].m_split;

					if(orig[axis]+radius < split)
					{
						offset		= children+0;
						distance	= orig[axis] - split;
					}
					else if(split < orig[axis]-radius)
					{
						offset		= children+1;
						distance	= split - orig[axis];
					}
					else
					{
						nearestStack[traversal].offset = children+1;
						nearestStack[traversal].distance = split - orig[axis];
						++traversal;

						offset		= children+0;
						distance	= orig[axis] - split;
					}
				}
				else if(traversal)
				{
					--traversal;
					offset		= nearestStack[traversal].offset;
					distance	= nearestStack[traversal].distance;
				}
				else
				{
					return hitResult->GetDist() != FLT_MAX;
				}
			}
			else
			{
				const uint* candidate = leafList + children;

				for(UINT i=0; i<splitList[offset].m_count; ++i)
				{
					TObject::HitResult testResult;
					if(objectList[candidate[i]].NearestTest(orig, radius, &testResult) && testResult.GetDist() < radius)
					{
						hitResult->object = candidate[i];
						hitResult->result = testResult;
						radius = testResult.GetDist();
					}
				}
				if(traversal)
				{
					--traversal;
					offset		= nearestStack[traversal].offset;
					distance	= nearestStack[traversal].distance;
				}
				else
				{
					return hitResult->GetDist() != FLT_MAX;
				}
			}	
		}
		return hitResult->GetDist() != FLT_MAX;
	}
	return false;
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
float TKdTree<TObject>::CalcMinSAH(TKdSplit::SPLIT_AXIS axis, const kd_event_list& splitList, const TAABB& aabb, float* minSplit)
{
	float minRatio = 1.0f;

	kd_event_list_iter begin, end, iter;

	/* Z Axis */ 
	uint n_l = 0;
	uint n_r = 0;
	/* aabb.minimum[axis] 보다 작은 구간 체크 */ 
	for(begin=splitList.begin(); begin!=splitList.end() && begin->m_split < aabb.minimum[axis]; ++begin)
	{
		if(begin->GetState() == TKdEvent::STATE_OUT)	{ --n_l; --n_r;	}
		if(begin->GetState() == TKdEvent::STATE_IN)		{ ++n_l; ++n_r;	}
	}
	/* aabb.minimum[axis] 경계선 위 체크 */ 
	n_l = n_r;
	for(; begin!=splitList.end() && begin->m_split==aabb.minimum[axis]; ++begin)
	{
		if(begin->GetState() == TKdEvent::STATE_OUT)	{ --n_l; --n_r;	}
		if(begin->GetState() == TKdEvent::STATE_ON)		{ ++n_l;		}
		if(begin->GetState() == TKdEvent::STATE_IN)		{ ++n_l; ++n_r;	}
	}
	/* aabb 내부 체크 */ 
	for(end=begin; end!=splitList.end() && end->m_split<aabb.maximum[axis]; ++end)
	{
		if(end->GetState() == TKdEvent::STATE_ON)		{ ++n_r;		}
		if(end->GetState() == TKdEvent::STATE_IN)		{ ++n_r;		}
	}
	/* aabb.maximum[axis] 경계선 위 체크 */ 
	for(iter=end; iter!=splitList.end() && iter->m_split==aabb.maximum[axis]; ++iter)
	{
		if(iter->GetState() == TKdEvent::STATE_ON)		{ ++n_r;		}
	}

	float sah = CalcSAH(n_l, 0, n_r, 0, aabb.maximum[axis] - aabb.minimum[axis]);
	/* split 을 찾자 */ 
	for(iter=begin; iter!=end; )
	{
		uint n_c = 0;
		uint temp = 0;
		float split = iter->m_split;
		for(; iter!=end && iter->m_split==split; ++iter)
		{
			if(iter->GetState() == TKdEvent::STATE_OUT) --n_r;
			if(iter->GetState() == TKdEvent::STATE_ON) ++n_c;
			if(iter->GetState() == TKdEvent::STATE_IN) ++temp;
		}
		/* SAH 계산을 한다. */ 
		float ratio = CalcSAH(n_l, n_c, n_r, split - aabb.minimum[axis], aabb.maximum[axis] - split) / sah;
		if(ratio < minRatio)
		{
			minRatio = ratio;
			*minSplit = split;
		}
		/* 새로운 삼각형을 추가 */ 
		n_l += n_c + temp;
	}

	return minRatio;
}

/*---------------------------------------------------------------------------*/ 
template<class TObject>
float TKdTree<TObject>::CalcSAH(uint n_l, uint n_c, uint n_r, float p_l, float p_r)
{
	/*
	const float	k_t = traverse time; 224 clock
	const float k_i = intersect time; 29 clock
	return k_t + k_i * min(p_l*(n_l+n_c) + p_r*n_r, p_l*n_l + p_r*(n_r+n_c));
	*/

	//return 224 + 29 * min(p_l*(n_l+n_c) + p_r*n_r, p_l*n_l + p_r*(n_r+n_c));
	//return 29 + 224 * min(p_l*(n_l+n_c) + p_r*n_r, p_l*n_l + p_r*(n_r+n_c));
	return 12500000 + min(p_l*(n_l+n_c) + p_r*n_r, p_l*n_l + p_r*(n_r+n_c));
}

/*---------------------------------------------------------------------------*/ 
