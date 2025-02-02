// KdTree Test.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "KdTree Test.h"

#include "t_kdtree.h"
#include "t_bvhtree.h"

#define TEST_LOOP 100

#ifdef PROFILE
#include "ittnotify.h"
#pragma comment(lib, "libittnotify.lib")
#endif //PROFILE

#define _MM_DENORMALS_ZERO_ON     0x0040

/*---------------------------------------------------------------------------*/ 
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);

void LoadBunny();
void SetBitmapSize(int width, int height);
void PickTriangle(HWND hWnd, int x, int y);

/*---------------------------------------------------------------------------*/ 
typedef	std::vector<TVector>	vector_list;
typedef	vector_list::iterator		vector_list_iter;

typedef	std::vector<TTriangle>		triangle_list;
typedef	triangle_list::iterator		triangle_list_iter;

vector_list					g_raylList;

struct TriangleNormal
{
	TVector n0;
	TVector n1;
	TVector n2;
};

typedef std::vector<TriangleNormal>	normal_list;
typedef	normal_list::iterator		normal_list_iter;

normal_list					g_normalList;

triangle_list				g_triangleList;

TKdTree<TTriangle>			g_kdTree;
TBVHTree<TTriangle>			g_bvhTree;

int					g_width;
int					g_height;
BYTE				g_bitmapInfoBuffer[sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * 256];
BITMAPINFO*			g_bitmapInfo = (BITMAPINFO*)g_bitmapInfoBuffer;
std::vector<BYTE>	g_bitmap;

uint		g_left = 0;
uint		g_right = 0;
uint		g_push = 0;
uint		g_calls = 0;
uint		g_nodes = 0;
uint		g_objects = 0;


int GetMSB(DWORD_PTR dwordPtr)
{
	if(dwordPtr)
	{
		int result = 1;
#if defined(_WIN64)
		if(dwordPtr & 0xFFFFFFFF00000000) { result += 32; dwordPtr &= 0xFFFFFFFF00000000; }
		if(dwordPtr & 0xFFFF0000FFFF0000) { result += 16; dwordPtr &= 0xFFFF0000FFFF0000; }
		if(dwordPtr & 0xFF00FF00FF00FF00) { result += 8;  dwordPtr &= 0xFF00FF00FF00FF00; }
		if(dwordPtr & 0xF0F0F0F0F0F0F0F0) { result += 4;  dwordPtr &= 0xF0F0F0F0F0F0F0F0; }
		if(dwordPtr & 0xCCCCCCCCCCCCCCCC) { result += 2;  dwordPtr &= 0xCCCCCCCCCCCCCCCC; }
		if(dwordPtr & 0xAAAAAAAAAAAAAAAA) { result += 1; }
#else
		if(dwordPtr & 0xFFFF0000) { result += 16; dwordPtr &= 0xFFFF0000; }
		if(dwordPtr & 0xFF00FF00) { result += 8;  dwordPtr &= 0xFF00FF00; }
		if(dwordPtr & 0xF0F0F0F0) { result += 4;  dwordPtr &= 0xF0F0F0F0; }
		if(dwordPtr & 0xCCCCCCCC) { result += 2;  dwordPtr &= 0xCCCCCCCC; }
		if(dwordPtr & 0xAAAAAAAA) { result += 1; }
#endif
		return result;
	}
	else
	{
		return 0;
	}
}

extern int g_test;
extern int g_normalMiss;
extern int g_edgeMiss;

#define OSXSAVEFlag (1UL<<27)  
#define AVXFlag     ((1UL<<28)|OSXSAVEFlag)  
#define FMAFlag     ((1UL<<12)|AVXFlag|OSXSAVEFlag)  
#define CLMULFlag   ((1UL<< 1)|AVXFlag|OSXSAVEFlag)  
#define VAESFlag    ((1UL<<25)|AVXFlag|OSXSAVEFlag)  
  
bool SimdDetectFeature(unsigned int idFeature)  
{  
    int reg[4];
    __cpuidex(reg, 1, 0);  
    if((reg[2] & idFeature) != idFeature)  
        return false;  
    return true;  
} 

/*---------------------------------------------------------------------------*/ 
int APIENTRY _tWinMain(HINSTANCE hInstance, HINSTANCE, LPTSTR, int nCmdShow)
{
	// 테스트 정밀도를 위해 설정
	DWORD_PTR processMask, systemMask;
	GetProcessAffinityMask(GetCurrentProcess(), &processMask, &systemMask);
	SetProcessAffinityMask(GetCurrentProcess(), 1i64 << (GetMSB(processMask) - 1) );

	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);

	//unsigned int currentControl;
	//_controlfp_s(&currentControl, _PC_24, _MCW_PC);

	// vTune 권고사항. SSE 에서 Flush-To-Zero 와 Denormals-To-Zero Flag 를 켜준다.
	//_mm_setcsr(_mm_getcsr() | _MM_FLUSH_ZERO_ON);
	//_mm_setcsr(_mm_getcsr() | _MM_DENORMALS_ZERO_ON);

	__int64 freq, start, end;
	QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
	char temp[256];

	LoadBunny();
	
	g_kdTree.BuildTree(20);
	g_kdTree.SaveKdTree(_T("bunny.kdt"));
	g_kdTree.LoadKdTree(_T("bunny.kdt"));

	//g_bvhTree.BuildTree(20);
	//g_bvhTree.SaveTree(_T("bunny.kdt"));
	//g_bvhTree.LoadTree(_T("bunny.kdt"));

	SetBitmapSize(800, 800);

	QueryPerformanceCounter((LARGE_INTEGER*)&start);

#ifdef PROFILE
	__itt_resume();
#endif //PROFILE

	TVector scale(1.0f, 1.0f, 1.0f);
	for(int iter=0; iter<TEST_LOOP; ++iter)
	{

		/* Single 1 Ray Tracing */

		TVector light;
		TNormalize(&light, &TVector(1, 1, 1));

		BYTE* line = &g_bitmap[0];

		for (int y = 0; y < g_height; ++y)
		{
			for (int x = 0; x < g_width; ++x)
			{
				TKdTree<TTriangle>::HitResult result;
				TVector orig((float)x, (float)y, -500.0f);
				TVector dir(0, 0, 1);

				//if(g_kdTree.HitTest(orig, dir, scale, &result))
				if (g_kdTree.HitTest(orig, dir, &result))
				{
					TVector normal = g_normalList[result.object].n0 * (1 - result.result.u - result.result.v)
						+ g_normalList[result.object].n1 * result.result.u
						+ g_normalList[result.object].n2 * result.result.v;

					float intensity = min(1.0f, max(0.0f, TDot(&normal, &light)));

					line[x] = (BYTE)(intensity * 255.0f);
				}
				else
				{
					line[x] = 0;
				}
			}
			line += g_width;
		}

	}

#ifdef PROFILE
	__itt_pause();
	return 0;
#endif //PROFILE

	QueryPerformanceCounter((LARGE_INTEGER*)&end);

	SetPriorityClass(GetCurrentProcess(), NORMAL_PRIORITY_CLASS);

	sprintf_s(temp, sizeof(temp), "%f sec (%.3f M rays/sec)\r\nleft %d, right %d, push %d, \r\ncalls %d, nodes %d, objects %d, \r\nnormalmiss %d, edgemiss %d", (end - start) / double(freq), TEST_LOOP * double(freq) * (g_width * g_height) / (end - start) / 1000000, g_left, g_right, g_push, g_calls, g_nodes, g_objects, g_normalMiss, g_edgeMiss);
	MessageBoxA(NULL, temp, temp, MB_OK);

	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_KDTREETEST));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_KDTREETEST);
	wcex.lpszClassName	= _T("KdTree Class");
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	RegisterClassEx(&wcex);

	HWND hWnd = CreateWindow(_T("KdTree Class"), _T("KdTree Test"), WS_OVERLAPPEDWINDOW,
	  CW_USEDEFAULT, 0, 1024, 850, NULL, NULL, hInstance, NULL);

	if (!hWnd)
	{
	  return FALSE;
	}

	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);

	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}

	return (int) msg.wParam;
}

/*---------------------------------------------------------------------------*/ 
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
	case WM_LBUTTONDOWN:
		PickTriangle(hWnd, LOWORD(lParam),HIWORD(lParam));
		break;

	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);

		if(!g_bitmap.empty())
		{
			SetDIBitsToDevice(hdc, 0, 0, g_width, g_height, 0, 0, 0, g_height, &g_bitmap[0], g_bitmapInfo, DIB_RGB_COLORS); 
		}
		EndPaint(hWnd, &ps);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

/*----------------------------------------------------------------------------*/ 
void LoadBunny()
{
	FILE* file;
	fopen_s(&file, "bunny.ply", "rt");
	if(file)
	{
		int count = 69451;

		std::vector<TVector>	vertexList;
		g_triangleList.reserve(count);
		g_normalList.reserve(count);

		vertexList.reserve(35947);

		for(int i=0; i<35947; ++i)
		{
			float x, y, z, confidence, intensity;
			fscanf_s(file, "%f %f %f %f %f", &x, &y, &z, &confidence, &intensity);
			vertexList.push_back(TVector(220 + x * 2200, 220 * 2 - y * 2200, z * 2200) * 2);
		}

		std::vector<TVector>	normalList;

		normalList.resize(35947, TVector(0,0,0));

		struct Face
		{
			int p0, p1, p2;
		};

		std::vector<Face> faceList;
		

		for(int i=0; i<count; ++i)
		{
			int number, p0, p1, p2;
			fscanf_s(file, "%d %d %d %d", &number, &p0, &p1, &p2);
			g_triangleList.push_back(TTriangle(vertexList[p0], vertexList[p1], vertexList[p2]));
			g_kdTree.AddObject(TTriangle(vertexList[p0], vertexList[p1], vertexList[p2]));
			//g_bvhTree.AddObject(TTriangle(vertexList[p0], vertexList[p1], vertexList[p2]));
			Face face;
			face.p0 = p0;
			face.p1 = p1;
			face.p2 = p2;
			faceList.push_back(face);

			TVector normal;
			TNormalize(&normal, TCross(&normal, &(vertexList[p0]-vertexList[p1]), &(vertexList[p0]-vertexList[p2])));

			normalList[p0] += normal;
			normalList[p1] += normal;
			normalList[p2] += normal;
		}

		for (int i=0; i<35947; ++i)
		{
			TNormalize(&normalList[i], &normalList[i]);
		}

		for (int i=0; i<count; ++i)
		{
			Face& face = faceList[i];

			TriangleNormal normal;
			normal.n0 = normalList[face.p0];
			normal.n1 = normalList[face.p1];
			normal.n2 = normalList[face.p2];
			g_normalList.push_back(normal);
		}

		fclose(file);
	}
}

/*---------------------------------------------------------------------------*/ 
void SetBitmapSize(int width, int height)
{
	g_width = width;
	g_height = height;

	g_bitmapInfo->bmiHeader.biSize			= sizeof(BITMAPINFOHEADER);
	g_bitmapInfo->bmiHeader.biWidth			= g_width;
	g_bitmapInfo->bmiHeader.biHeight		= -g_height;
	g_bitmapInfo->bmiHeader.biPlanes		= 1;
	g_bitmapInfo->bmiHeader.biBitCount		= 8;
	g_bitmapInfo->bmiHeader.biCompression	= BI_RGB;
	g_bitmapInfo->bmiHeader.biSizeImage		= 0;
	g_bitmapInfo->bmiHeader.biXPelsPerMeter	= 100;
	g_bitmapInfo->bmiHeader.biYPelsPerMeter	= 100;
	g_bitmapInfo->bmiHeader.biClrUsed		= 256;
	g_bitmapInfo->bmiHeader.biClrImportant	= 0;

	for(int i=0; i<256; ++i)
	{
		g_bitmapInfo->bmiColors[i].rgbRed = i;
		g_bitmapInfo->bmiColors[i].rgbGreen = i;
		g_bitmapInfo->bmiColors[i].rgbBlue = i;
	}

	g_bitmap.clear();

	g_bitmap.resize(width * height, 0);
}

/*---------------------------------------------------------------------------*/ 
void PickTriangle(HWND hWnd, int x, int y)
{
	HDC hdc = GetDC(hWnd);

	HPEN hRedPen = CreatePen(PS_SOLID, 0, RGB(255,0,0));
	HPEN hPenOld = (HPEN)SelectObject(hdc, (HGDIOBJ)hRedPen);

	TVector orig((float)x, (float)y, -500.0f);
	TVector dir(0,0,1);

	TKdTree<TTriangle>::HitResult result;
	if(g_kdTree.HitTest(orig, dir, &result))
	{

		TTriangle triangle = g_kdTree.GetObject(result.object);
		
		TVector p0 = triangle.pos0;
		TVector p1 = triangle.pos1;
		TVector p2 = triangle.pos2;

		MoveToEx(hdc, (int)p0.x, (int)p0.y, NULL);
		LineTo(hdc, (int)p1.x, (int)p1.y);
		LineTo(hdc, (int)p2.x, (int)p2.y);
		LineTo(hdc, (int)p0.x, (int)p0.y);
	}

	SelectObject(hdc, (HGDIOBJ)hPenOld);
	DeleteObject((HGDIOBJ)hRedPen);

	ReleaseDC(hWnd, hdc);
}

/*----------------------------------------------------------------------------*/ 
