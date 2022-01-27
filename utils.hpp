#pragma once
#define _USE_MATH_DEFINES
#include <random>
#include <iostream>
#include <algorithm>
#include <math.h>   
#include <vector> 
#include <array>
#include <cfloat>
using namespace std;
std:: default_random_engine generator;
std::uniform_real_distribution<double> distr(0.0,1.0);
double uniform_rand() { return distr(generator); }
inline float clamp(float lo, float hi, float v) { return max(lo, min(hi, v)); }
inline double clamp(double x) { return x<0 ? 0 : x>1 ? 1 : x; }
inline int toInt(double x) { return int(pow(clamp(x),1/2.2)*255+.5); }

struct Vec {
	double x = 0, y = 0, z = 0;
	double operator[](int idx) const { return idx == 0 ? x : idx == 1 ? y : z; }
	Vec(double x_=0, double y_=0, double z_=0) { x=x_; y=y_; z=z_; }
	Vec operator+(const Vec &b) const { return Vec(x+b.x,y+b.y,z+b.z); }
	Vec operator-(const Vec &b) const { return Vec(x-b.x,y-b.y,z-b.z); }
	Vec operator * (const Vec &v) const { return Vec(x * v.x, y * v.y, z * v.z); }
	Vec operator*(double b) const { return Vec(x*b,y*b,z*b); }
	friend Vec operator * (const float &r, const Vec &v) { return Vec(v.x * r, v.y * r, v.z * r); }
	Vec operator/(double b) const { return Vec(x/b,y/b,z/b); }
	Vec mult(const Vec &b) const { return Vec(x*b.x,y*b.y,z*b.z); }
	float l2_norm() const { return sqrt(x*x+y*y+z*z);}
	Vec norm() const { return  *this * (1/l2_norm()); }
	double dot(const Vec &b) const { return x*b.x+y*b.y+z*b.z; }
	int max_dim() const { return x > y && x > z ? x : y > z ? y : z; }
	Vec operator%(const Vec&b) const { return Vec(y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x); }
	static Vec e_min(const Vec& a, const Vec& b) { return Vec(min(a.x,b.x),min(a.y,b.y),min(a.z,b.z)); }
	static Vec e_max(const Vec& a, const Vec& b) { return Vec(max(a.x,b.x),max(a.y,b.y),max(a.z,b.z)); }
    bool operator ==(Vec b) const{ return x == b.x && y == b.y && z == b.z; }
    bool operator !=(Vec b) const{ return !(*this == b);}
    friend ostream & operator << (ostream &os, const Vec &v) { return os << v.x << ", " << v.y << ", " << v.z; }
};