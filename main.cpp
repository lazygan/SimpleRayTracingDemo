#include "OBJ_Loader.hpp"
#include "utils.hpp"
#include "material.hpp"
using namespace std;

struct Ray {
	Vec o, d, d_inv;
	Ray(Vec o_, Vec d_) : o(o_), d(d_) { d_inv = { 1/d.x, 1/d.y, 1/d.z } ; }
};
struct Bounds {
	Vec p_min = { DBL_MAX,DBL_MAX,DBL_MAX } ;
	Vec p_max = { DBL_MIN,DBL_MIN,DBL_MIN } ;
	Bounds() = default;
	Bounds(Vec p) : p_min(p), p_max(p) { }
	Bounds(Vec p1, Vec p2) { p_min = Vec::e_min(p1,p2); p_max = Vec::e_max(p1,p2); }
	Bounds Union(const Bounds& b2) {
		Bounds ret;
		ret.p_min = Vec::e_min(this->p_min, b2.p_min);
		ret.p_max = Vec::e_max(this->p_max, b2.p_max);
		return ret;
	}
	bool IntersectP(const Ray& ray) {
		const Vec& origin = ray.o, invDir = ray.d_inv;
		bool dirIsNeg[3] = { ray.d.x >= 0, ray.d.y >= 0, ray.d.z >= 0 } ;
		float tEnter = FLT_MIN, tExit = FLT_MAX;
		for (int i = 0; i < 3; i++) {
			float min = (p_min[i] - origin[i]) * invDir[i];
			float max = (p_max[i] - origin[i]) * invDir[i];
			if (!dirIsNeg[i])  swap(min, max);
			tEnter = std::max(min, tEnter);
			tExit = std::min(max, tExit);
		}
		return tEnter <= tExit&& tExit >= 0;
	}
	Vec centroid() { return p_min * 0.5  + p_max * 0.5 ; }
} ;
struct Object;
struct Intersection {
	double  t = 0.0f;
	Vec x, n;
	Object* obj = nullptr;
};
namespace Lights{
	vector<Object*> light_objs;
	void regist_lights(Object* obj){ light_objs.push_back(obj);}
};
struct Object {
	Bounds b;
	Material* m;
	float area;
	Object(Material *m): m(m){
		if(m->e.l2_norm() > 0)
			Lights::regist_lights(this); 
	}
	virtual Intersection intersect(const Ray& r) { return Intersection(); }
	virtual Bounds bounds() { return b; }
	virtual void sample(Intersection &pos, float &pdf){};
};
namespace Lights{
	void sample_on_lights(Intersection &pos, float &pdf){ 
		float emit_area_sum = 0;
		for (uint32_t k = 0; k < light_objs.size(); ++k) {
				emit_area_sum += light_objs[k]->area;
		}
		float p = uniform_rand() * emit_area_sum;
		emit_area_sum = 0;
		for (uint32_t k = 0; k < light_objs.size(); ++k) {
			emit_area_sum += light_objs[k]->area;
			if (p <= emit_area_sum){
				light_objs[k]->sample(pos, pdf);
				break;
			}
		}
	}
};
struct BVHBuildNode {
	Bounds bounds;
	BVHBuildNode *left = nullptr;
	BVHBuildNode *right = nullptr;
	Object* obj = nullptr;
};
struct BVHAccel {
	BVHBuildNode* root;
	BVHAccel(vector<Object*> objs) { root = build(objs); }
	BVHBuildNode* build(vector<Object*>objs) {
		BVHBuildNode* node = new BVHBuildNode();
		Bounds bounds;
		auto l= objs.begin(), m= l + (objs.size() / 2), r= objs.end();
		for (int i = 0; i < objs.size(); ++i) bounds = bounds.Union(objs[i]->bounds());
		if (objs.size() == 1) {
			node->bounds = objs[0]->bounds();
			node->obj = objs[0];
			node->left = nullptr;
			node->right = nullptr;
			return node;
		} else if (objs.size() == 2) {
			node->left = build({ objs[0] });
			node->right = build({ objs[1] });
			node->bounds = node->left->bounds.Union(node->right->bounds);
			return node;
		} else {
			Bounds cb;
			for (int i = 0; i < objs.size(); ++i) cb =cb.Union(objs[i]->bounds().centroid());
			int dim = (cb.p_max - cb.p_min).max_dim();
			if(dim == 0) {
				std::sort(l, r, [](auto f1, auto f2) {
					return f1->bounds().centroid().x < f2->bounds().centroid().x;
				});
			} else if( dim == 1) {
				std::sort(l, r, [](auto f1, auto f2) {
					return f1->bounds().centroid().y < f2->bounds().centroid().y;
				});
			} else {
				std::sort(l, r, [](auto f1, auto f2) {
					return f1->bounds().centroid().z < f2->bounds().centroid().z;
				});
			}
			auto lp= std::vector<Object*>(l, m);
			auto rp= std::vector<Object*>(m, r);
			node->left = build(lp);
			node->right = build(rp);
			node->bounds = node->left->bounds.Union(node->right->bounds);
		}
		return node;
	}
	Intersection intersect(BVHBuildNode* node, const Ray& ray) const {
		Intersection isect;
		if (!node->bounds.IntersectP(ray))  return isect; 
		if (!node->left && !node->right) return node->obj->intersect(ray);
		Intersection hit1 = intersect(node->left, ray);
		Intersection hit2 = intersect(node->right, ray);
		if( hit1.t && hit2.t) return hit1.t< hit2.t? hit1 : hit2; 
		else if(hit1.t) return hit1; 
		else if(hit2.t) return hit2;
		return isect;
	}
};
struct Triangle : Object {
	Vec v0, v1, v2, n;
	Triangle(Vec v0_, Vec v1_, Vec v2_,Material* m):
			v0(v0_), v1(v1_), v2(v2_), Object(m){
		Vec e2 = v2 - v0, e1 = v1-v0;
		n = (e1 % e2).norm();
		b = Bounds(v0, v1).Union(Bounds(v2));
		area = (e2 % e1).l2_norm() * 0.5;
	}
	Intersection intersect(const Ray &r) override {
		Vec o = r.o, d  = r.d;
		Vec e1 = v1 - v0, e2 = v2 - v0;
		Vec s0 = o- v0, s1 = d%e2, s2 = s0%e1;
		Vec s = Vec(s2.dot(e2), s1.dot(s0), s2.dot(r.d)) / s1.dot(e1);
		double t = s.x, u = s.y, v = s.z;
		if (t >=0 && u >= 0 && v >=0 && (u+v) <= 1)  return { t , r.o+r.d*t , n, this };
		return Intersection();
	}
	void sample(Intersection &pos, float &pdf) override {
		float x = sqrt(uniform_rand()), y = uniform_rand();
        pos.x= v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        pos.n= this->n;
		pos.obj = this;
        pdf = 1.0f / area;
	}
} ;
struct MeshTriangle : Object {
	std::vector<Triangle> triangles;
	BVHAccel* bvh = nullptr;
	MeshTriangle(string filename,Vec trans, float scale, Material* m)
	: Object(m) {
		objl::Loader loader;
		loader.LoadFile(filename);
		auto mesh = loader.LoadedMeshes[0];
		for (int i = 0; i < mesh.Vertices.size(); i += 3) {
			std::array<Vec,3> face_vertices;
			for (int j = 0; j < 3; j++) {
				auto vert = Vec(mesh.Vertices[i + j].Position.X,
					mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z);
				face_vertices[j] =vert * scale + trans ;
			}
			triangles.emplace_back(face_vertices[0], 
				face_vertices[1], face_vertices[2],m);
		}
		std::vector<Object*> objs;
		for (auto& tri : triangles)  objs.push_back(&tri); 
		for (auto obj : objs) b = b.Union(obj->bounds());
		bvh = new BVHAccel(objs);
		Bounds b = bvh->root->bounds;
	}
	Intersection intersect(const Ray & r) { return bvh->intersect(bvh->root,r); }
};
struct Scene {
	BVHAccel* bvh = nullptr;
	vector<Object*> objs;
	float RussianRoulette = 0.8;
	Scene() { 
		Material* red = new Diffuse(Vec(0.75,0.25,0.25));
		Material* green = new Diffuse(Vec(0.25, 0.75, 0.25));
		Material* white = new Diffuse(Vec(0.75, 0.75, 0.75));
		Material* yellow= new Diffuse(Vec(0.75f, 0.75f, 0.0f));
		Material* mirr= new Specular();
		Material* glass= new Refraction();
		Material* light = new Diffuse(Vec(), Vec(25,25,25));
		float ul = 50;
		Vec llu(-ul,ul,ul),lru(-ul,ul,-ul),lrd(-ul,-ul,-ul),lld(-ul,-ul,ul);
		Vec rlu(ul,ul,ul), rru(ul,ul,-ul), rrd(ul,-ul,-ul), rld(ul,-ul,ul);
		objs = {
			new Triangle(lru,llu,lrd, red), new Triangle(lld,lrd,llu, red),//LEFT
			new Triangle(rru,rrd,rlu, green), new Triangle(rld,rlu,rrd, green),//RIGHT
			new Triangle(llu,lru,rlu, white), new Triangle(rru,rlu,lru, white),//TOP
			new Triangle(lld,rld,lrd, white), new Triangle(rrd,lrd,rld, white),//BOTTOM
			new Triangle(lrd,rrd,lru, white), new Triangle(rru,lru,rrd, white),//BACK
			new MeshTriangle("./bunny.obj",Vec(0,-60,0), 300.0, glass),
		};
		llu.x *= 0.3, llu.z *= 0.3, rlu.x *= 0.3, rlu.z *= 0.3;
		lru.x *= 0.3, lru.z *= 0.3, rru.x *= 0.3, rru.z *= 0.3;
		objs.push_back(new Triangle(llu,lru,rlu, light));
		objs.push_back(new Triangle(rru,rlu,lru, light));
		bvh = new BVHAccel(objs); 
	}
	Intersection intersect(const Ray &r) { return bvh->intersect(bvh->root,r); }
	Vec radiance(const Ray &r,Intersection inter = Intersection()) {
		if(!inter.t) inter = intersect(r);
		if (!inter.t) return Vec();
		Material* m= inter.obj->m;
		Vec x=inter.x, n = inter.n, wo =(r.d * -1).norm();
		if(m->e.l2_norm() > 0) return m->e; // 发光物直接返回
		Vec L_dir, L_indir;
		float pdf_light = 0.0f;
		Intersection inter_;
		Lights::sample_on_lights( inter_,pdf_light);
		Material* m_= inter_.obj->m;
		Vec x_ = inter_.x, ws = (x_-x).norm(), n_= inter_.n;
		if((intersect(Ray(x,ws)).x - x_).l2_norm() < 1e-3)  // 如果没有遮挡,求得直接光照
			L_dir = m_->e * m->eval(wo,ws,n) * ws.dot(n)*(ws*-1.0).dot(n_) / (((x_-x).l2_norm() * (x_-x).l2_norm()) * pdf_light); 
		if(uniform_rand() < Scene::RussianRoulette) { // 间接光照,俄罗斯轮盘赌决定是否放弃追踪
				Vec wi = m->sample(wo,n);
				Intersection i = intersect(Ray(x,wi));
				if(i.obj && i.obj->m->e.l2_norm() < 1e-3) // 如果采样到光源, 则返回
					L_indir = radiance(Ray(x,wi),i) * m->eval(wi,wo,n)*wi.dot(n) / (m->pdf(wi,wo,n)*Scene::RussianRoulette);
		}
		Vec refra_wi = m->refract(wo,n);
		if(refra_wi != Vec()){
			if(uniform_rand() < Scene::RussianRoulette) { //折射光计算
				return L_dir + L_indir + radiance(Ray(x,refra_wi)) / Scene::RussianRoulette;
			}
		} 
		return L_dir + L_indir;
	}
};
int main(int argc, char *argv[]) {
	Scene scene;
	int w= 1024, h= 760, samps = 4;
	Ray cam(Vec(0,0,250), Vec(0,0.0,-1).norm());
	Vec cx=Vec(w*.5135/h), cy=(cx%cam.d).norm()*.5135, r, *c=new Vec[w*h];
	for (int y=0; y<h; y++) {
		fprintf(stderr,"\rRendering (%d spp) %5.2f%%",samps*4,100.*y/(h-1));
		for (unsigned short x=0; x<w; x++) {
			for (int sy=0, i=(h-y-1)*w+x; sy<2; sy++)  
			        for (int sx=0; sx<2; sx++, r=Vec()) {
				for (int s=0; s<samps; s++) {
					double r1=2*uniform_rand(), dx=r1<1 ? sqrt(r1)-1: 1-sqrt(2-r1);
					double r2=2*uniform_rand(), dy=r2<1 ? sqrt(r2)-1: 1-sqrt(2-r2);
					Vec d = cx*( ( (sx+.5 + dx)/2 + x)/w - .5) +
					                    cy*( ( (sy+.5 + dy)/2 + y)/h - .5) + cam.d;
					r = r + scene.radiance(Ray(cam.o+d*140,d.norm())) * (1./samps);
				}
				c[i] = c[i] + Vec(clamp(r.x),clamp(r.y),clamp(r.z))*.25;
			}
		}
	}
	FILE *f = fopen("image_.ppm", "w");
	fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
	for (int i=0; i<w*h; i++)
	    fprintf(f,"%d %d %d ", toInt(c[i].x), toInt(c[i].y), toInt(c[i].z));
}
