//
//

#if !defined(_T_SSE_H_)
#define _T_SSE_H_

/*----------------------------------------------------------------------------*/ 
__forceinline	__m128	operator+(__m128 l, __m128 r)	{ return _mm_add_ps(l,r);		}
__forceinline	__m128	operator-(__m128 l, __m128 r)	{ return _mm_sub_ps(l,r);		}
__forceinline	__m128	operator*(__m128 l, __m128 r)	{ return _mm_mul_ps(l,r);		}
__forceinline	__m128	operator/(__m128 l, __m128 r)	{ return _mm_div_ps(l,r);		}
__forceinline	__m128	operator&(__m128 l, __m128 r)	{ return _mm_and_ps(l,r);		}
__forceinline	__m128	operator|(__m128 l, __m128 r)	{ return _mm_or_ps(l,r);		}
__forceinline	__m128	operator<(__m128 l, __m128 r)	{ return _mm_cmplt_ps(l,r);		}
__forceinline	__m128	operator>(__m128 l, __m128 r)	{ return _mm_cmpgt_ps(l,r);		}
__forceinline	__m128	operator<=(__m128 l, __m128 r)	{ return _mm_cmple_ps(l,r);		}
__forceinline	__m128	operator>=(__m128 l, __m128 r)	{ return _mm_cmpge_ps(l,r);		}
__forceinline	__m128	operator!=(__m128 l, __m128 r)	{ return _mm_cmpneq_ps(l,r);	}
__forceinline	__m128	operator==(__m128 l, __m128 r)	{ return _mm_cmpeq_ps(l,r);		}

__forceinline	__m128	_mm_merge_ps(__m128 m, __m128 l, __m128 r)
{
	return _mm_andnot_ps(m, l) | (m & r);
}

/*----------------------------------------------------------------------------*/ 
struct TPoint4
{
	TPoint4() {}
	TPoint4(const D3DXVECTOR3& a) :x(_mm_set1_ps(a.x)), y(_mm_set1_ps(a.y)), z(_mm_set1_ps(a.z)) {}
	TPoint4(__m128 a, __m128 b, __m128 c) :x(a), y(b), z(c) {}
	TPoint4(const __m128* a) :x(a[0]), y(a[1]), z(a[2]) {}
	TPoint4(const D3DXVECTOR3& a, const D3DXVECTOR3& b, const D3DXVECTOR3& c, const D3DXVECTOR3& d) :x(_mm_set_ps(a.x,b.x,c.x,d.x)), y(_mm_set_ps(a.y,b.y,c.y,d.y)), z(_mm_set_ps(a.z,b.z,c.z,d.z)) {}

	operator __m128* ()				{ return &x; }
    operator const __m128* () const	{ return &x; }

	TPoint4 operator+(const TPoint4& r) const	{ return TPoint4(x+r.x, y+r.y, z+r.z);	}
	TPoint4 operator-(const TPoint4& r) const	{ return TPoint4(x-r.x, y-r.y, z-r.z);	}
	TPoint4 operator*(__m128 r) const			{ return TPoint4(x * r, y * r, z * r);	}
	TPoint4 operator/(__m128 r) const			{ return TPoint4(x / r, y / r, z / r);	}

	__m128 operator[](int index) const			{ return _val[index];					}

	union
	{
		struct
		{
				__m128 x, y, z;
		};
		struct
		{
				__m128 _val[3];
		};
	};


};

__forceinline TPoint4* TPoint4Cross(TPoint4* result, const TPoint4* l, const TPoint4* r)
{
	result->x = (l->y * r->z) - (l->z * r->y);
	result->y = (l->z * r->x) - (l->x * r->z);
	result->z = (l->x * r->y) - (l->y * r->x);

	return result;
}

__forceinline __m128 TPoint4Dot(const TPoint4* l, const TPoint4* r)
{
	return (l->x * r->x) + (l->y * r->y) + (l->z * r->z);
}

__forceinline TPoint4* TPoint4Normalize(TPoint4* result, const TPoint4* l)
{
	__m128 rec_len = _mm_rsqrt_ps( (l->x * l->x) + (l->y * l->y) + (l->z * l->z) );

	result->x = l->x * rec_len;
	result->y = l->y * rec_len;
	result->z = l->z * rec_len;

	return result;
}

__forceinline __m128 TPoint4Length(const TPoint4* l)
{
	return _mm_sqrt_ps( (l->x * l->x) + (l->y * l->y) + (l->z * l->z) );
}

__forceinline TPoint4* TPoint4Merge(TPoint4* result, __m128 mask, const TPoint4* l, const TPoint4* r)
{
	result->x = _mm_merge_ps(mask, l->x, r->x);
	result->y = _mm_merge_ps(mask, l->y, r->y);
	result->z = _mm_merge_ps(mask, l->z, r->z);

	return result;
}

/*----------------------------------------------------------------------------*/ 
extern __m128	g_zero4;
extern __m128	g_one4;
extern __m128	g_fltMax4;
extern __m128	g_mask4;
extern __m128	g_epsilon4;

/*----------------------------------------------------------------------------*/ 
__forceinline	__m256	operator+(__m256 l, __m256 r)	{ return _mm256_add_ps(l,r);				}
__forceinline	__m256	operator-(__m256 l, __m256 r)	{ return _mm256_sub_ps(l,r);				}
__forceinline	__m256	operator*(__m256 l, __m256 r)	{ return _mm256_mul_ps(l,r);				}
__forceinline	__m256	operator/(__m256 l, __m256 r)	{ return _mm256_div_ps(l,r);				}
__forceinline	__m256	operator&(__m256 l, __m256 r)	{ return _mm256_and_ps(l,r);				}
__forceinline	__m256	operator|(__m256 l, __m256 r)	{ return _mm256_or_ps(l,r);					}
__forceinline	__m256	operator<(__m256 l, __m256 r)	{ return _mm256_cmp_ps(l,r,_CMP_LT_OQ);		}
__forceinline	__m256	operator>(__m256 l, __m256 r)	{ return _mm256_cmp_ps(l,r,_CMP_GT_OQ);		}
__forceinline	__m256	operator<=(__m256 l, __m256 r)	{ return _mm256_cmp_ps(l,r,_CMP_LE_OQ);		}
__forceinline	__m256	operator>=(__m256 l, __m256 r)	{ return _mm256_cmp_ps(l,r,_CMP_GE_OQ);		}
__forceinline	__m256	operator!=(__m256 l, __m256 r)	{ return _mm256_cmp_ps(l,r,_CMP_NEQ_OS);	}
__forceinline	__m256	operator==(__m256 l, __m256 r)	{ return _mm256_cmp_ps(l,r,_CMP_EQ_OS);		}

__forceinline	__m256	_mm256_merge_ps(__m256 m, __m256 l, __m256 r)
{
	return _mm256_blendv_ps(l, r, m);
}

/*---------------------------------------------------------------------------*/ 
struct TPoint8
{
	TPoint8() {}
	TPoint8(const D3DXVECTOR3& p) :x(_mm256_broadcast_ss(&p.x)), y(_mm256_broadcast_ss(&p.y)), z(_mm256_broadcast_ss(&p.z)) {}
	TPoint8(const __m256& a, const __m256& b, const __m256& c) :x(a), y(b), z(c) {}
	TPoint8(const __m256* a) :x(a[0]), y(a[1]), z(a[2]) {}
	TPoint8(const D3DXVECTOR3& a, const D3DXVECTOR3& b, const D3DXVECTOR3& c, const D3DXVECTOR3& d, const D3DXVECTOR3& e, const D3DXVECTOR3& f, const D3DXVECTOR3& g, const D3DXVECTOR3& h)
	:x(_mm256_set_ps(a.x,b.x,c.x,d.x,e.x,f.x,g.x,h.x))
	,y(_mm256_set_ps(a.y,b.y,c.y,d.y,e.y,f.y,g.y,h.y))
	,z(_mm256_set_ps(a.z,b.z,c.z,d.z,e.z,f.z,g.z,h.z))
	{
	}

	TPoint8 operator+(const TPoint8& r) const	{ return TPoint8(x+r.x, y+r.y, z+r.z);	}
	TPoint8 operator-(const TPoint8& r) const	{ return TPoint8(x-r.x, y-r.y, z-r.z);	}
	TPoint8 operator*(const __m256& r) const	{ return TPoint8(x*r, y*r, z*r);		}

    operator const __m256* () const	{ return &x; }

	__m256	x;
	__m256	y;
	__m256	z;
};

__forceinline TPoint8*		TPoint8Cross(TPoint8* result, const TPoint8* l, const TPoint8* r)
{
	result->x = (l->y * r->z) - (l->z * r->y);
	result->y = (l->z * r->x) - (l->x * r->z);
	result->z = (l->x * r->y) - (l->y * r->x);

	return result;
}

__forceinline __m256		TPoint8Dot(const TPoint8* l, const TPoint8* r)
{
	return (l->x * r->x) + (l->y * r->y) + (l->z * r->z);
}

__forceinline TPoint8*		TPoint8Merge(TPoint8* result, const __m256& mask, const TPoint8* l, const TPoint8* r)
{
	result->x = _mm256_merge_ps(mask, l->x, r->x);
	result->y = _mm256_merge_ps(mask, l->y, r->y);
	result->z = _mm256_merge_ps(mask, l->z, r->z);

	return result;
}

__forceinline TPoint8* TPoint8Normalize(TPoint8* result, const TPoint8* l)
{
	__m256 rec_len = _mm256_rsqrt_ps( (l->x * l->x) + (l->y * l->y) + (l->z * l->z) );

	result->x = l->x * rec_len;
	result->y = l->y * rec_len;
	result->z = l->z * rec_len;

	return result;
}

__forceinline __m256 TPoint8Length(const TPoint8* l)
{
	return _mm256_sqrt_ps( (l->x * l->x) + (l->y * l->y) + (l->z * l->z) );
}

/*----------------------------------------------------------------------------*/ 
extern __m256	g_zero8;
extern __m256	g_one8;
extern __m256	g_fltMax8;
extern __m256	g_mask8;
extern __m256	g_epsilon8;

/*----------------------------------------------------------------------------*/ 

#endif //_T_SSE_H_