#ifndef RT_GEOM_H
#define RT_GEOM_H

#include <iostream>
#include <vector>
#include <math.h>
#include <random>


constexpr int WIDTH = 700;
constexpr int HEIGHT = 700;


struct Point3d;
double dot(const Point3d &A, const Point3d &B);
Point3d operator * (double a, const Point3d &A);


struct Point3d
{
    double x, y, z;
    Point3d(double x = -1000, double y = -1000, double z = -1000):x(x), y(y), z(z) {};
    Point3d(std::vector<double> &v) : x(v[0]), y(v[1]), z(v[2]){};
    Point3d(Point3d const &copy) {
        this->x = copy.x;
        this->y = copy.y;
        this->z = copy.z;
    };

    Point3d operator * (double a) const {
        Point3d res(*this);
        res.x *= a;
        res.y *= a;
        res.z *= a;
        return res;
    };
    
    Point3d norm() const {
        return *this / sqrt(dot(*this, *this));
    }
    
    
    Point3d reflect(const Point3d &N) const {
        return *this - 2 * dot(*this, N) * N;
    }
    
    Point3d refract(Point3d &a_normal, const double a_matIOR) { 
        const Point3d &ray_dir = *this;
        float eta = 1.0 / a_matIOR; 
        float cos_theta = -dot(a_normal, ray_dir);
        if(cos_theta < 0)
        {
            cos_theta *= -1.0;
            a_normal = -1.0 * a_normal;
            eta = 1.0 / eta;
        }
        
        float k = 1.0f - eta * eta * (1.0 - cos_theta * cos_theta);
        if(k >= 0.0f) {
            return eta * ray_dir + (eta * cos_theta - sqrt(k)) * a_normal;
        } else {
            return Point3d(0, 0, 0);
        } 
}   
    
    Point3d operator / (double a) const {
        Point3d res(*this);
        res.x /= a;
        res.y /= a;
        res.z /= a;
        return res;
    };

    Point3d operator + (Point3d const &obj) const {
        Point3d res;
        res.x = x + obj.x;
        res.y = y + obj.y;
        res.z = z + obj.z;
        return res;
    }
    
    Point3d operator - (Point3d const &obj) const{
        return *this + (obj * (-1.));
    }
};


Point3d operator * (double a, const Point3d &A) 
{
    Point3d res(A);
    res.x *= a;
    res.y *= a;
    res.z *= a;
    return res;
}


double dot(const Point3d &A, const Point3d &B) 
{   
    return A.x * B.x + A.y * B.y + B.z * A.z;
}


double xdist(const Point3d &A)
{
    return dot(A, A);
}


struct Ray {
    Ray() {}
    Ray(const Point3d &a, const Point3d &b) { A = a; B = b; }
    Ray(const Ray &r) : A(r.A), B(r.B){}
    
    Point3d origin() const     { return A; }
    Point3d direction() const { return B; }
    Point3d point_at_parameter(double t) const { return A + B * t; }

    Point3d A;
    Point3d B;
};


struct Material
{
    int specularity;
    double ref;
    double dif_par;
    double refract_index;
    double refraction;
    uint32_t color;
    
    //Конструктор
    Material (const int specularity = 32, const double ref = 0, const double dif_par = 1, 
              const double refract_index = 1, const double refraction = 0, const uint32_t color = 0) : 
    specularity(specularity), ref(ref), dif_par(dif_par), refract_index(refract_index), 
    refraction(refraction), color(color) {}
};


struct Hyperboloid
{
    Point3d center;
    double height;
    Material material;
    double k;
    
    Hyperboloid(const Point3d &center, double height, const Material &material, double k = 50) :
        center(center), height(height), material(material), k(k) {}
        
    double intersect(const Ray &, Point3d &) const;
};


double Hyperboloid::intersect(const Ray &ray, Point3d &N) const
{
    Point3d dir = ray.direction();
    Point3d oc = ray.origin() - center;

    double a = dir.x * dir.x + dir.z * dir.z - dir.y * dir.y;
    dir.y *= -1;
    double b = 2.0 * dot(oc, dir);
    double c = dot(oc, oc) - 2 * oc.y * oc.y - this->k * this->k;
    
    double discriminant = b * b - 4 * a * c;
    double t;
    
    if(discriminant < 0){
        t =  -1.0;
    } else {
        t = (-b - sqrt(discriminant)) / (2.0 * a);
    }
    
    dir = ray.point_at_parameter(t);
    if (abs(dir.y - this->center.y) > this->height) {
        t = -1;
    } else { 
        Point3d point = (dir - center);
        point.y *= -1;
        N = point.norm();
    }
    
    return t;
}


struct Sphere
{
    Point3d center;
    double radius; 
    Material material;
    
    double intersect(const Ray &ray) const;
    
    Sphere(const Sphere &sp) {
        center = sp.center;
        radius = sp.radius;
        material = sp.material;
    }
    
    Sphere (const Point3d &center, const double radius, const Material &material = Material()) : 
    center(center), radius(radius), material(material) {}
};


double Sphere::intersect(const Ray &ray) const
{
    Point3d oc = ray.origin() - center;
    double a = dot(ray.direction(), ray.direction());
    double b = 2.0 * dot(oc, ray.direction());
    double c = dot(oc, oc) - radius * radius;
    double discriminant = b * b - 4 * a * c;
    if(discriminant < 0){
        return -1.0;
    } else {
        return (-b - sqrt(discriminant)) / (2.0 * a);
    }
}

struct Light
{
    Point3d position;
    double intensity;
    
    Light (Point3d &pos, double intensity = 0) : position(pos), intensity(intensity) {};
    Light(double x = 0, double y = 0, double z = 0, double intensity = 0) : position(Point3d(x, y, z)), intensity(intensity) {}
};


struct Surface
{
    Material material;
    double y, width, length;
    
    Surface() : y(120), width(600), length(900) {}
    Surface(const Material &material, double y) : material(material), y(y) {}
    
    double intersect(const Ray &ray) const
    {
        double step = ray.direction().y;
        double length = this->y - ray.origin().y;
        double t = length / step;
        
        if (abs(ray.point_at_parameter(t).x - 256) > this->width || abs(ray.point_at_parameter(t).z - 256) > this->length) {
            return -1;
        } else {
            return t;
        }
    }
    
    void update_color(const Point3d& hit) {
        this->material.color = (int(floor(hit.x / 70)) + int(floor(hit.z / 70))) & 1 ? 0x006FDCF7 : 0x00222222;
    }
    
};


struct Cylinder
{
    double y1, y2, r;
    Material material;
    double x, z;
    
    Cylinder(){}
    
    Cylinder(double y1, double y2, double r, const Material &material = Material()) : 
        y1(y1), y2(y2), r(r), material(material), x(260), z(200) {}
        
    Cylinder(const Cylinder &P) {
        this->material = P.material;
        this->r = P.r;
        this->y1 = P.y1;
        this->y2 = P.y2;
        this->x = P.x;
        this->z = P.z;
    }

    
    double intersect(const Ray &, Point3d &) const; 
    double circle_intersect(const Ray &, Point3d &) const;         
};


double Cylinder::circle_intersect(const Ray &ray, Point3d &N) const
{
    double step = ray.direction().y;
    double length = this->y2 - ray.origin().y;
    double t = -1, cur_t = length / step;
    
    double a = xdist(Point3d(this->x, this->y2, this->z) - ray.point_at_parameter(cur_t));
    if (cur_t > 0 && a <= this->r * this->r) {
        N = Point3d(0, 1, 0);
        t = cur_t;
    }
    
    return t;
}


double Cylinder::intersect(const Ray &ray, Point3d &N) const
{
    double t = this->circle_intersect(ray, N);
    
    Point3d oc = ray.origin() - Point3d(this->x, 0, this->z);
    oc.y = 0;
    Point3d dir = ray.direction();
    dir.y = 0;
    double a = dot(dir, dir);
    double b = 2.0 * dot(oc, dir);
    double c = dot(oc, oc) - this->r * this->r;
    double discriminant = b * b - 4 * a * c;
    double cur_t;
    
    if(discriminant < 0){
        cur_t =  -1.0;
    } else {
        cur_t = (-b - sqrt(discriminant)) / (2.0 * a);
    }
    
    
    if (cur_t > 0 && (t < 0 || cur_t < t)) {
        Point3d point = ray.point_at_parameter(cur_t);
        if (point.y >= this->y1 && point.y <= this->y2) {
            t = cur_t;
            N = (point - Point3d(this->x, point.y, this->z)).norm();
        }
    }
    
    return t;
}


struct Pyramid
{
    Point3d p1, p2, p3, p4;
    Material material;
    
    Pyramid(){}
    
    Pyramid(const Point3d &p1, const Point3d &p2, const Point3d &p3, const Point3d &p4, const Material &material = Material()) : 
        p1(p1), p2(p2), p3(p3), p4(p4), material(material) {}
        
    Pyramid(const Pyramid &P) {
        this->material = P.material;
        this->p1 = P.p1;
        this->p2 = P.p2;
        this->p3 = P.p3;
        this->p4 = P.p4;
    }

    double intersect(const Ray &, Point3d &) const;         
};


double squere(const Point3d &p1, const Point3d &p2, const Point3d &p3)
{
    Point3d a = p2 - p1;
    Point3d b = p3 - p1;
    Point3d res = Point3d(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    
    return 0.5 * sqrt(dot(res, res));
}


bool is_in(const Point3d &p1, const Point3d &p2, const Point3d &p3, const Point3d &p4)
{
    return abs(squere(p1, p2, p3) - (squere(p4, p2, p3) + squere(p1, p4, p3) + squere(p1, p2, p4))) < 1e-3;
}


double simple_intersect(const Ray &ray, const Point3d &p1, const Point3d &p2, const Point3d &p3, double &A, double &B, double &C) 
{
    double D = p1.x * (p2.y * p3.z - p3.y * p2.z) + p2.x * (p3.y * p1.z - p1.y * p3.z) + p3.x * (p1.y * p2.z - p2.y * p1.z);
    D *= -1;
    A = p1.y * (p2.z - p3.z) + p2.y * (p3.z - p1.z) + p3.y * (p1.z - p2.z);
    B = p1.z * (p2.x - p3.x) + p2.z * (p3.x - p1.x) + p3.z * (p1.x - p2.x);
    C = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    
    double a = A * ray.direction().x + B * ray.direction().y + C * ray.direction().z;
    if (abs(a) < 1e-3) return -1;
    
    double t = -1 * (A * ray.origin().x + B * ray.origin().y + C * ray.origin().z + D) / a;
    if (is_in(p1, p2, p3, ray.point_at_parameter(t))) {
        return t;
    } else {
        return -1;
    }
}


double Pyramid::intersect(const Ray &ray, Point3d &N) const
{
    Point3d cur_N;
    double t = -1, cur_t;
    
    cur_t = simple_intersect(ray, p1, p2, p3, cur_N.x, cur_N.y, cur_N.z);
    if (cur_t > 0) {
        N = cur_N;
        t = cur_t;
    }
    
    cur_t = simple_intersect(ray, p1, p2, p4, cur_N.x, cur_N.y, cur_N.z);
    if (cur_t > 0 && (t < 0 || cur_t < t)) {
        N = cur_N;
        t = cur_t;
    }
    
    cur_t = simple_intersect(ray, p1, p4, p3, cur_N.x, cur_N.y, cur_N.z);
    if (cur_t > 0 && (t < 0 || cur_t < t)) {
        N = cur_N;
        t = cur_t;
    }
    
    cur_t = simple_intersect(ray, p4, p2, p3, cur_N.x, cur_N.y, cur_N.z);
    if (cur_t > 0 && (t < 0 || cur_t < t)) {
        N = cur_N;
        t = cur_t;
    }
    
    return t;
}


struct Background
{
    uint32_t *data;
    int width, height;
    Sphere sphere;
    
    Background(const char* filename);
    uint32_t get_color(const Ray &ray) const;
};


Background::Background(const char* filename) : sphere(Sphere(Point3d(WIDTH / 2, HEIGHT / 2, -WIDTH / tan(M_PI / 4)), 5000))
{
    FILE* f = fopen(filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f);

    this->width = *(int*)&info[18];
    this->height = *(int*)&info[22];

    int size = this->width * this->height;
    this->data = new uint32_t[size]; 
    
    fread(this->data, sizeof(uint32_t), size, f); 
    fclose(f);
}


uint32_t Background::get_color(const Ray &ray) const
{   
    Point3d point = (ray.point_at_parameter(this->sphere.intersect(ray)) - this->sphere.center).norm();
    double u = 0.5 + atan2(point.z, point.x) / (2 * M_PI);
    double v = 0.5 - asin(point.y) / M_PI;
    
    int a = u * this->width;
    int b = v * this->height;
    
    return this->data[a + b * this->width]; // background color   
}


#endif //RT_GEOM_H
