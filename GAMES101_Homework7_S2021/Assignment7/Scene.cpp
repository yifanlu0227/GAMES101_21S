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

// 没有加速结构的光线与三角形求交
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    // shade(p, wo)
    //      sampleLight(inter , pdf_light)
    //      Get x, ws, NN, emit from inter
    //      Shoot a ray from p to x
    //      If the ray is not blocked in the middle
    //      L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws, NN) / |x-p|^2 / pdf_light
    //
    //      L_indir = 0
    //      wi = sample(wo, N)
    //      Trace a ray r(p, wi)
    //      If ray r hit a non-emitting object at q
    //      L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N) / pdf(wo, wi, N) / RussianRoulette
    //      Return L_dir + L_indir

    // get intersection
    Intersection inter = intersect(ray);

    if(!inter.happened)
        return Vector3f(0.f,0.f,0.f);

    // Note: if hit the light, return the emission intensity!
    if(inter.m->hasEmission())
        return inter.m->getEmission();

    Vector3f L_dir(0.f, 0.f, 0.f);
    Vector3f L_indir(0.f, 0.f, 0.f);

    // get direct light 
    Intersection light;
    float pdf;
    sampleLight(light, pdf);

    Vector3f wo = - ray.direction; // direction from intersection p
    Vector3f N = normalize(inter.normal); // 朝内
    Vector3f ws = normalize(light.coords - inter.coords); // direction from intersection p
    Vector3f NN = normalize(light.normal); // 朝内

    float light_distance2 = dotProduct(light.coords - inter.coords, light.coords - inter.coords);
    float light_distance = sqrt(light_distance2);

    // if blocked
    Ray ray_to_light(inter.coords, ws);
    Intersection block_inter = intersect(ray_to_light);
    if(block_inter.distance < light_distance - 1e-3) // Note: check if blocked
        L_dir += 0;
    else
        L_dir += light.emit * inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / light_distance2 / pdf; // wo is useless

    // get indirect light 
    float RR = get_random_float();

    if(RR < RussianRoulette){
        // uniform sample from hemisphere
        Vector3f wi = inter.m->sample(wo, N); // wo is useless
        Ray ray_out(inter.coords, wi); // wi is from p

        Intersection new_inter = intersect(ray_out);
        if(new_inter.happened && !new_inter.m->hasEmission()){ // Note: must be non-emitting object 
            float pdf_hemi = inter.m->pdf(wo, wi, N); // wo is useless. Set pdf_hemi = 1/2pi is also ok.
            L_indir += castRay(ray_out, depth+1) * inter.m->eval(wo, wi, N) * dotProduct(wi, N) / pdf_hemi / RussianRoulette; // wo is useless
        }
    }

    return L_dir + L_indir;
}