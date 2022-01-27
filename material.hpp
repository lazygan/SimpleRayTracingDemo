#pragma once
#include "utils.hpp"
#define M_PI 3.1415926535
struct Material{
    Vec e; 
	Material(Vec e = Vec(0,0,0),Vec ks = Vec(1,1,1)) : e(e) {}
	virtual float pdf(const Vec &wi, const Vec &wo, const Vec &n) {return 1;}
	virtual Vec sample(const Vec &wo, const Vec &n)  {return Vec();}
	virtual Vec eval(const Vec &wi, const Vec &wo, const Vec &n) {return Vec();}
	virtual Vec refract(const Vec &wi, const Vec &n) { return Vec(); }
	Vec toWorld(const Vec &a, const Vec &n){
       Vec B, C;
       if (fabs(n.x) > fabs(n.y)){
           float invLen = 1.0f / sqrt(n.x * n.x + n.z * n.z);
           C =Vec(n.z * invLen, 0.0f, -n.x *invLen);
       } else {
           float invLen = 1.0f / std::sqrt(n.y * n.y + n.z * n.z);
           C =Vec(0.0f, n.z * invLen, -n.y *invLen);
       }
       B =C % n;
       return B * a.x   + C * a.y  + n * a.z ;
    }
	Vec reflect(const Vec &wi, const Vec &n) const { return wi * -1 + n* wi.dot(n) * 2; }
	Vec calc_refract(const Vec &wi, const Vec &n, float ior) const {
        float cosi = clamp(-1, 1, wi.dot(n)), etai = 1, etat = ior;
        Vec n_ = n, i = wi * -1;
		if(cosi < 0 ){ swap(etai,etat); n_ = n * -1; i = i* -1;}
        float eta = etai / etat, k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : i * eta  + n_ * (eta * cosi - sqrtf(k));
    }
	float fresnel(const Vec &wi, const Vec &n, const float &ior) const {
        float kr, cosi = clamp(-1, 1,wi.dot(n)), etai = 1, etat = ior;
        if (cosi < 0) {  std::swap(etai, etat); }
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        if (sint >= 1) kr = 1;
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            float Rp = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            kr = (Rs * Rs + Rp * Rp) / 2; // 折射比例
        }
		return kr;
    }
};

struct Diffuse : Material{
	Vec Kd;
	Diffuse(Vec Kd, Vec e = Vec(0,0,0)) :Kd(Kd), Material(e){}
	float pdf(const Vec &wi, const Vec &wo, const Vec &n) override { return 0.5f / M_PI; }
	Vec sample(const Vec &wo, const Vec &n) override {
		float x_1 = uniform_rand(), x_2 = uniform_rand();
		float z = std::fabs(1.0f - 2.0f * x_1);
		float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
		Vec localRay(r*std::cos(phi), r*std::sin(phi), z);
		return toWorld(localRay, n);
	}
	Vec eval(const Vec &wi, const Vec &wo, const Vec &n) override {
		if(wo.dot(n) < 0) return Vec();
		return Kd / M_PI;
	}
};

struct Specular :  Material {
	Vec Ks;
	Specular(Vec Ks = Vec(1,1,1)) : Ks(Ks), Material() {}
	float pdf(const Vec &wi, const Vec &wo, const Vec &n) override { return 1; }
	Vec sample(const Vec &wo, const Vec &n) override { 
		return reflect(wo,n).norm(); 
	}
	Vec eval(const Vec &wi, const Vec &wo, const Vec &n) override {
		if(wo.dot(n) < 0) return Vec();
		if((reflect(wi,n).norm()- wo).l2_norm() < 1e-6){ return Ks; } 
		return Vec();
	}
};

struct Refraction : Material {
	float ior;
	Vec Ks;
	Refraction(float ior = 1.5,Vec Ks = Vec(1,1,1)) : Ks(Ks),ior(ior),Material(){ }
	float pdf(const Vec &wi, const Vec &wo, const Vec &n) override { return 1; }
	Vec refract(const Vec &wi, const Vec &n) override { 
		return  calc_refract(wi,n,ior);
	}
};