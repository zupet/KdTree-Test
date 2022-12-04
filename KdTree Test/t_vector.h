#pragma once

struct TVector
{
	TVector() {}
	TVector(const float v[3]) : x(v[0]), y(v[1]), z(v[2]) {}
	TVector(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

	TVector operator+ (const TVector& v) const
	{
		return TVector(x + v.x, y + v.y, z + v.z);
	}
	TVector operator- (const TVector& v) const
	{
		return TVector(x - v.x, y - v.y, z - v.z);
	}
	TVector operator+= (const TVector& v)
	{
		x += v.x; y += v.y; z += v.z; return *this;
	}
	float& operator[] (int index)
	{
		return *(&x + index);
	}
	const float& operator[] (int index) const
	{
		return *(&x + index);
	}
	TVector operator* (float v) const
	{
		return TVector(x * v, y * v, z * v);
	}
	TVector operator/ (float v) const
	{
		return TVector(x / v, y / v, z / v);
	}
	operator const float* () const
	{
		return &x;
	}

	float x, y, z;
};

inline float TDot(const TVector* l, const TVector* r)
{
	return l->x * r->x + l->y * r->y + l->z * r->z;
}
inline float TLength(const TVector* l)
{
	return sqrtf(TDot(l, l));
}
inline TVector* TCross(TVector* v, const TVector* l, const TVector* r)
{
	*v = TVector(l->y * r->z - l->z * r->y, l->z*r->x - l->x*r->z, l->x*r->y - l->y*r->x);
	return v;
}
inline TVector* TNormalize(TVector* v, const TVector* l)
{
	*v = *l / TLength(l);
	return v;
}

