//
//

#include "stdafx.h"
#include "t_sse.h"


/*---------------------------------------------------------------------------*/ 
__m128	g_zero4		= _mm_setzero_ps();
__m128	g_one4		= _mm_set1_ps(1.0f);
__m128	g_fltMax4	= _mm_set1_ps(FLT_MAX);
__m128	g_mask4		= (g_one4 == g_one4);
__m128	g_epsilon4	= _mm_set1_ps(0.0001f);

/*---------------------------------------------------------------------------*/ 
__m256	g_zero8		= _mm256_setzero_ps();
__m256	g_one8		= _mm256_set1_ps(1.0f);
__m256	g_fltMax8	= _mm256_set1_ps(FLT_MAX);
__m256	g_mask8		= g_one8 == g_one8;
__m256	g_epsilon8	= _mm256_set1_ps(0.0001f);
