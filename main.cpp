#include <iostream>
#include <cstdint>
#include <omp.h>

#include <string>
#include <vector>
#include <unordered_map>

#include "Bitmap.h"
#include "geom.h"

const uint32_t BACKGROUND  = 0x00ff5555;
const int SURFACE_ID = 100;
const int PYRAMID_ID = 200;
const int CYLINDER_ID = 300;
const int HYPER_ID = 400;


std::vector<uint32_t> output_image(WIDTH * HEIGHT);
std::vector<Sphere> objects;
std::vector<Pyramid> pyramids;
std::vector<Cylinder> cylinders;
std::vector<Hyperboloid> hyperboloids;

Surface surface;


Background bg_img("../sky.bmp");

Point3d camera(WIDTH / 2, HEIGHT / 2, -WIDTH / tan(M_PI / 4)); 


void render(std::string &outFilePath)
{
  SaveBMP(outFilePath.c_str(), output_image.data(), WIDTH, HEIGHT);
  std::cout << "end." << std::endl;
}


int find_visible_sphere(Ray &ray) 
{
    double t = -1.;
    int obj_id = -1;
    int cur_id = 0;
    
    for (const Sphere &obj : objects) {
        double cur_t = obj.intersect(ray);
        if (cur_t >= 0. && (t > cur_t || t < 0.)) {
            obj_id = cur_id;
            t = cur_t;
        }
        cur_id++;
    }
    
    double cur_t = surface.intersect(ray);
    if (cur_t >= 0 && (t < 0 || cur_t < t)) {
        obj_id = SURFACE_ID;
        t = cur_t;
    }
    
    Point3d cur_N;
    for (const Pyramid &obj : pyramids) {
        cur_t = obj.intersect(ray, cur_N);
        if (cur_t >= 0 && (t < 0 || cur_t < t)) {
            obj_id = PYRAMID_ID;
            t = cur_t;
        }
    }
    
    for (const Cylinder &obj : cylinders) {
        cur_t = obj.intersect(ray, cur_N);
        if (cur_t >= 0 && (t < 0 || cur_t < t)) {
            obj_id = CYLINDER_ID;
            t = cur_t;
        }
    }
    
    for (const Hyperboloid &obj : hyperboloids) {
        cur_t = obj.intersect(ray, cur_N);
        if (cur_t >= 0 && (t < 0 || cur_t < t)) {
            obj_id = HYPER_ID;
            t = cur_t;
        }
    }
    
    return obj_id;
}


int find_visible_sphere_plus(Ray &ray, Material &material, Point3d &point, Point3d &N) 
{
    double t = -1.;
    int obj_id = -1;
    int cur_id = 0;
    Point3d center;
    
    for (const Sphere &obj : objects) {
        double cur_t = obj.intersect(ray);
        if (cur_t >= 0. && (t > cur_t || t < 0.)) {
            obj_id = cur_id;
            t = cur_t;
            material = obj.material;
            center = obj.center;
        }
        cur_id++;
    }
    
    point = ray.point_at_parameter(t);
    N = (point - center).norm();    
    
    double cur_t = surface.intersect(ray);
    if (cur_t >= 0 && (t < 0 || cur_t < t)) {
        N = Point3d(0, 1, 0);
        point = ray.point_at_parameter(cur_t);
        surface.update_color(point);
        material = surface.material;
        obj_id = SURFACE_ID;
        t = cur_t;
    }
    
    Point3d cur_N;
    
    for (const Pyramid &obj : pyramids) {
        cur_t = obj.intersect(ray, cur_N);
        if (cur_t >= 0 && (t < 0 || cur_t < t)) {
            N = cur_N.norm();
            point = ray.point_at_parameter(cur_t);
            material = obj.material;
            obj_id = PYRAMID_ID;
            t = cur_t;
        }
    }
    
    
    for (const Cylinder &obj : cylinders) {
        cur_t = obj.intersect(ray, cur_N);
        if (cur_t >= 0 && (t < 0 || cur_t < t)) {
            N = cur_N;
            point = ray.point_at_parameter(cur_t);
            material = obj.material;
            obj_id = CYLINDER_ID;
            t = cur_t;
        }
    }
    
    for (const Hyperboloid &obj : hyperboloids) {
        cur_t = obj.intersect(ray, cur_N);
        if (cur_t >= 0 && (t < 0 || cur_t < t)) {
            N = cur_N;
            point = ray.point_at_parameter(cur_t);
            material = obj.material;
            obj_id = HYPER_ID;
            t = cur_t;
        }
    }
    
    return obj_id;
}


uint32_t update_color(uint32_t color, double k)
{
    uint32_t r = (color & 0x00FF0000) >> 16;
    uint32_t g = (color & 0x0000FF00) >> 8;
    uint32_t b = color & 0x000000FF;
    
    uint32_t max = std::max(r, std::max(g, b));
    k = std::min(k, 255. / max);
    
    r = uint32_t(floor(r * k));
    g = uint32_t(floor(g * k));
    b = uint32_t(floor(b * k));
     
    
    return (r << 16) + (g << 8) + b;
}


uint32_t sum_color(uint32_t A, uint32_t B) 
{
    uint32_t r = (A & 0x00FF0000) >> 16;
    uint32_t g = (A & 0x0000FF00) >> 8;
    uint32_t b = A & 0x000000FF;
    
    uint32_t r2 = (B & 0x00FF0000) >> 16;
    uint32_t g2 = (B & 0x0000FF00) >> 8;
    uint32_t b2 = B & 0x000000FF;
    
    r = std::min(r + r2, uint32_t(255));
    g = std::min(g + g2, uint32_t(255));
    b = std::min(b + b2, uint32_t(255));
     
    return (r << 16) + (g << 8) + b;  
}


double ambient_light()
{
    return 0.1;
}


double specular_light(const std::vector<Light> &lights, const Point3d &point, const Point3d &N, const Material &material, int obj_id)
{
    double specularStrength = 0.9;
    Point3d view_dir = (camera - point).norm();
    double specular_light_intensity = 0.;
    
    for (const Light &light : lights) {
        Ray view_ray(camera, point - camera);
        
        Point3d xpoint = dot(point - light.position, N) < 0 ? point - N * 1e-3 : point + N * 1e-3;
        Ray ray(light.position, xpoint - light.position);
        
        
        if (find_visible_sphere(ray) != obj_id) {
            continue;
        }
             
        Point3d reflect_dir = (point - light.position).reflect(N).norm();
        specular_light_intensity += specularStrength * pow(std::max(dot(view_dir, reflect_dir), 0.0), material.specularity) * light.intensity;
    }
    
    return specular_light_intensity;
}


double diffusion_light(const std::vector<Light> &lights, const Point3d &point, const Point3d &N, const Material &material, int obj_id)
{    
    double diffuse_light_intensity = 0.;
    
    for (const Light &light : lights) {
        Point3d xpoint = dot(light.position - point, N) < 0 ? point - N * 1e-2 : point + N * 1e-2;
        Ray ray(xpoint, light.position - xpoint);
        
        int inters = find_visible_sphere(ray);
        if (inters > 0 && inters != obj_id) {
            continue;
        }
        
        Point3d dir = (light.position - xpoint).norm();
        diffuse_light_intensity += light.intensity * std::max(0., dot(dir, N));
    }
    return diffuse_light_intensity * material.dif_par;
}


uint32_t ray_cast(const Point3d &camera, const std::vector<Light> &lights, const Point3d &pxl_pos, int depth);


uint32_t antialias(const Point3d &camera, const std::vector<Light> &lights, const Point3d &pxl_pos, int depth)
{
    return ray_cast(camera, lights, pxl_pos, depth + 1);

    // uint32_t r = 0, g = 0, b = 0;
    // for (int i = -1; i <= 1; i++) {
    //         uint32_t A = ray_cast(camera, lights, pxl_pos + Point3d(i * 5 * 1e-2, 0, 0), depth + 1);
    //         r += (A & 0x00FF0000) >> 16;
    //         g += (A & 0x0000FF00) >> 8;
    //         b += A & 0x000000FF;
    // }
    // 
    // return ((r / 3) << 16) + ((g / 3) << 8) + (b / 3);
}


uint32_t ray_cast(const Point3d &camera, const std::vector<Light> &lights, const Point3d &pxl_pos, int depth)
{
    if (depth > 7) {
        return 0;
    }
    
    Ray ray(camera, pxl_pos - camera);
    uint32_t color;
    double brightness = 0.;
    
    Material material;
    Point3d point;
    Point3d N;
    
    int obj_id = find_visible_sphere_plus(ray, material, point, N);
    color = bg_img.get_color(ray);
    
    brightness = ambient_light();
    if (obj_id < 0) {
        return color;
    }
    
    
    brightness += diffusion_light(lights, point, N, material, obj_id);
    brightness += specular_light(lights, point, N, material, obj_id);
    
    uint32_t reflect_color = 0;
    uint32_t refract_color = 0;
    
    if (material.ref > 0) {
        Point3d ray_dir = ray.direction().reflect(N).norm();
        point = dot(ray_dir, N) < 0 ? point - N * 1e-3 : point + N * 1e-3;
        
        Ray reflect_ray(point, ray_dir);
        reflect_color = antialias(point, lights, reflect_ray.point_at_parameter(1), depth + 1); 
    }
    
    if (material.refraction > 0) {
        Point3d refract_dir = ray.direction().norm().refract(N, material.refract_index).norm();
        if (dot(refract_dir, refract_dir) > 0) {
            point = dot(refract_dir, N) < 0 ? point - N * 1e-3 : point + N * 1e-3;
            Ray refract_ray(point, refract_dir);
            refract_color = antialias(point, lights, refract_ray.point_at_parameter(1), depth + 1);
        }
    }
    
    return sum_color(sum_color(update_color(material.color, brightness), update_color(reflect_color, material.ref)), update_color(refract_color, material.refraction));
}


void ray_tracing(const Point3d &camera, const std::vector<Light> &lights) 
{
    int x, y;
    #pragma omp parallel for default(shared) private(x, y)
    for (x = 0; x < WIDTH; ++x) {
        for (y = 0; y < HEIGHT; ++y) {
            uint32_t color = antialias(camera, lights, Point3d(x, y, 0), 0);
            output_image[WIDTH * y + x] = color;
        }
    }
}


int main(int argc, char **argv)
{
    std::unordered_map<std::string, std::string> cmdLineParams;

    for(int i=0; i < argc; i++)
    {
        std::string key(argv[i]);

        if(key.size() > 0 && key[0]=='-')
        {
            if(i != argc - 1) // not last argument
            {
                cmdLineParams[key] = argv[i+1];
                i++;
            } else {
                cmdLineParams[key] = "";
            }
        }
    }

    std::string outFilePath = "zout.bmp";
    if(cmdLineParams.find("-out") != cmdLineParams.end())
        outFilePath = cmdLineParams["-out"];

    int sceneId = 0;
    if(cmdLineParams.find("-scene") != cmdLineParams.end())
        sceneId = atoi(cmdLineParams["-scene"].c_str());
             
    int num_threads = 1;
    if(cmdLineParams.find("-threads") != cmdLineParams.end())         
        num_threads = atoi(cmdLineParams["-threads"].c_str());
        
    omp_set_num_threads(num_threads);
    
    if (sceneId == 2) {
        Material blick_blue(64, 0, 1, 1, 0, 0x00663319);
        Hyperboloid hyper(Point3d(200, 220, 400), 100, blick_blue);
        hyperboloids.push_back(hyper);
        
        std::vector<Light> lights;
        Point3d pos = camera + Point3d(-200, 200, 100);
        lights.push_back(Light(pos, 0.7));
     
        ray_tracing(camera, lights);
        render(outFilePath);
        return 0;
    } 
    
    if (sceneId == 3) {
        return 0;
    }


    Material glass(256, 0.05, 0.01, 1.6, 0.85, 0x00111111);
    Material mirror(256, 1, 0.1, 1, 0, 0x00FFFFFF);
    Material blick_blue(64, 0.2, 0.8, 1, 0, 0x00663319);
    Material pink(64, 0.2, 0.8, 1, 0, 0x00663399);
    Material blue(16, 0, 1, 1, 0, 0x00aaaaaa);
    Material monno_white(512, 1.0, 0, 1.2, 0, 0x00111111);

    
    objects.push_back(Sphere({400, 310, 90}, 80, glass));
    objects.push_back(Sphere({110, 276, -10}, 40, pink));
    objects.push_back(Sphere({140, 236, 20}, 40, blick_blue));
    objects.push_back(Sphere({230, 400, 0}, 50, mirror));
    
    Pyramid pyramid(Point3d(350, 180, 0), Point3d(550, 180, 20), Point3d(450, 180, -80), Point3d(450 ,360, -40), monno_white);
    Cylinder cylinder(120, 220, 60, blue);
    
    pyramids.push_back(pyramid);
    cylinders.push_back(cylinder);
    
    Point3d pos = camera + Point3d(0, 150, 50);
    std::vector<Light> lights;
    lights.push_back(Light(856, 356,  -60, 0.7));
    lights.push_back(Light(pos, 0.4));
    
    ray_tracing(camera, lights);
    render(outFilePath);
}


