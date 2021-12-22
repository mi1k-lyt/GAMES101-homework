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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir = { 0,0,0 };
    Vector3f L_indir = { 0,0,0 };

    Intersection intersection = Scene::intersect(ray);//光线打到物体的碰撞信息
    
    if (!intersection.happened) {       
        return {};
    }
    if (intersection.m->hasEmission()) {//直接碰到光源      
        return intersection.m->getEmission();
    }
  
    /*               直接光照                 */
    
    Intersection light_sample;
    float pdf_light = 0.0f;
    sampleLight(light_sample, pdf_light);
    //求|x-p|^2  用obj_light_distance表示
    Vector3f o_l_dis = light_sample.coords - intersection.coords;
    float obj_light_distance = o_l_dis.dis();
    //求wo 既光线到物体的方向 ；wi为ray的dir
    Vector3f light_dir = o_l_dis.normalized();
    //这里判断光线从物体到光源之间是否有遮挡物
    Ray obj2light(intersection.coords, light_dir);
    Intersection light_insect = Scene::intersect(obj2light);
    //如果没有遮挡物的话 计算直接光源
    if (light_insect.distance - o_l_dis.norm() >= -0.00007f) {
        L_dir = light_sample.emit
              * intersection.m->eval(ray.direction, light_dir,  intersection.normal)
              * dotProduct(light_dir, intersection.normal)
              * dotProduct(-light_dir, light_sample.normal)//注意这里一定是负的light_dir 具体看讲义16的P41的图 大坑！！
              / obj_light_distance
              / pdf_light;
    }

    /*              间接光照                    */
   
    //如果没有通过轮盘赌，则不计算间接光照
    if (get_random_float() > RussianRoulette)
        return L_dir;
    
    Vector3f wo = intersection.m->sample(ray.direction, intersection.normal).normalized();
    Ray obj2other(intersection.coords, wo);
    Intersection other_insect = Scene::intersect(obj2other);
    
    if (other_insect.happened && !other_insect.m->hasEmission()) {
        L_indir = castRay(obj2other, depth+1)
                * intersection.m->eval(ray.direction, wo, intersection.normal)
                * dotProduct(wo, intersection.normal)
                / intersection.m->pdf(ray.direction, wo, intersection.normal)
                / RussianRoulette;
    }

    return L_dir + L_indir;

}