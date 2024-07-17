//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

float vectors_cos(const Vector3f &vec1, const Vector3f &vec2)
{
    return (dotProduct(vec1, vec2) / std::sqrt(std::pow(vec1.x - vec2.x, 2) + std::pow(vec1.y - vec2.y, 2) + std::pow(vec1.z - vec2.z, 2)));
}

float point_distance(const Vector3f &vec1, const Vector3f &vec2)
{
    return std::sqrt(std::pow(vec1.x - vec2.x, 2) + std::pow(vec1.y - vec2.y, 2) + std::pow(vec1.z - vec2.z, 2));
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO: Implement Path Tracing Algorithm here

    // find intersection of ray and scene
    Intersection intersection_with_scene = this->intersect(ray);
    // printf("depth: %d, ray origin: (%.6f, %.6f, %.6f), ray direction: (%.6f, %.6f, %.6f), after intersection with scene\n", depth, ray.origin.x, ray.origin.y, ray.origin.z, ray.direction.x, ray.direction.y, ray.direction.z);
    if(intersection_with_scene.happened)
    {
        if(intersection_with_scene.m->hasEmission())
        {
            if(depth == 0)
            {
                return intersection_with_scene.m->getEmission();
            }
            else{
                return Vector3f(0, 0, 0);
            }
        }
        // printf("depth: %d, intersection with scene happen\n", depth);
        auto p = intersection_with_scene.coords;
        // direct from light source
        Intersection inter;
        float light_pdf;
        sampleLight(inter, light_pdf);
        auto x = inter.coords;
        // compute if there are obstacle between x and intersection_p
        // printf("depth: %d, after ray emit intersection\n", depth);
        bool has_obstacle = false;

        // Ray light_to_obj(x,  p - x);
        // Intersection light_to_scene = this->intersect(light_to_obj);
        // if(light_to_scene.happened && (light_to_scene.distance - point_distance(x, p) > -EPSILON)){
        //     has_obstacle = false;
        // }else{
        //     has_obstacle = true;
        // }

        Ray ray_emit(p + EPSILON, x - p);
        Intersection ray_emit_intersection = this->intersect(ray_emit);
        float p_x_distance = point_distance(p, x);
        float p_obstacle_distance = point_distance(p, ray_emit_intersection.coords);
        if(ray_emit_intersection.happened && p_x_distance - p_obstacle_distance > EPSILON)
        {
            has_obstacle = true;
        }

        Vector3f L_dir(0, 0, 0);
        if(!has_obstacle)
        {
            // compute effect direct from light source
            // Vector3f wo(-ray.direction.x, -ray.direction.y, -ray.direction.z);
            Vector3f wo = ray.direction;
            // Vector3f wi = p - x;
            Vector3f wi = x - p;
            // Vector3f minus_wi = -wi;
            // auto f_r = intersection_with_scene.m->eval(wi, wo, intersection_with_scene.normal);
            auto f_r = intersection_with_scene.m->eval(wo, wi, intersection_with_scene.normal);
            L_dir = inter.emit * f_r * vectors_cos(-wi, inter.normal) * vectors_cos(wi, intersection_with_scene.normal) / (std::pow(x.x - p.x, 2) + std::pow(x.y - p.y, 2) + std::pow(x.z - p.z, 2)) / light_pdf;
        }

        // indirect from light source
        Vector3f L_indir(0, 0, 0);
        // RR
        // float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float ksi = get_random_float();
        // printf("depth: %d, compute probability\n", depth);
        if(ksi > this->RussianRoulette)
        {
            return L_dir;
        }
        else
        {
            // Vector3f wo(-ray.direction.x, -ray.direction.y, -ray.direction.z);
            Vector3f wo = ray.direction;
            auto wi = intersection_with_scene.m->sample(ray.direction, intersection_with_scene.normal);
            // wi = -wi;
            // Ray reflect_ray(p, -wi);
            Ray reflect_ray(p + EPSILON, wi);
            Intersection reflect_ray_intersection = this->intersect(reflect_ray);
            if(reflect_ray_intersection.happened)
            {
                if(!reflect_ray_intersection.m->hasEmission())
                {
                    // recursively compute input ray
                    // Ray input_ray(p, wi);
                    // auto pdf = intersection_with_scene.m->pdf(wi, wo, intersection_with_scene.normal);
                    auto pdf = intersection_with_scene.m->pdf(wo, wi, intersection_with_scene.normal);
                    // auto f_r = reflect_ray_intersection.m->eval(wi, wo, intersection_with_scene.normal);
                    auto f_r = reflect_ray_intersection.m->eval(wo, wi, intersection_with_scene.normal);
                    L_indir = this->castRay(reflect_ray, depth + 1) * f_r * vectors_cos(wi, intersection_with_scene.normal) / pdf / this->RussianRoulette;
                }
            }
        }
        return L_dir + L_indir;
        //  RussianRoulette
    }
    else
    {
        // printf("depth: %d, intersection with scene not happened\n", depth);
        return Vector3f(0,0,0);
    }
}