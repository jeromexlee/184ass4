#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // TODO Part 2, task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }

  // You'll want to adjust this code.
  // Right now we just return a single node containing all primitives.
  BVHNode *node = new BVHNode(bbox);
  node->prims = new vector<Primitive *>(prims);  
  if (prims.size() <= max_leaf_size){
    node->l = NULL;
    node->r = NULL;
  }
  else {
    Vector3D extent = bbox.extent;
    int axis = 0;
    if(extent.x >= extent.y && extent.x >= extent.z){
      axis = 0;
    }
    else if(extent.y >= extent.x && extent.y >= extent.z){
      axis = 1;
    }
    else{
      axis = 2;
    }
    vector<Primitive *> lp = vector<Primitive *>();
    vector<Primitive *> rp = vector<Primitive *>();
    double split = (centroid_box.max[axis] + centroid_box.min[axis])/2;
    for (Primitive *p : prims){
      Vector3D c = p->get_bbox().centroid();
      if(split >= c[axis]){
        lp.push_back(p);
      }
      else{
        rp.push_back(p);
      }
    }
    if(lp.size() > 0){
      node->l = construct_bvh(lp, max_leaf_size);  
    }
    if(rp.size() > 0){
      node->r = construct_bvh(rp, max_leaf_size);      
    }
  }
  return node;

}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {

  // TODO Part 2, task 3:
  // Implement BVH intersection.
  // Currently, we just naively loop over every primitive.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if(!node->bb.intersect(ray, t0, t1)) return false;
  if(node->isLeaf()){
      for (Primitive *p : *(node->prims)) {
      total_isects++;
      if (p->intersect(ray)) 
        return true;
    }
    return false;
  }
  bool b1 = intersect(ray,node->l);
  bool b2 = intersect(ray,node->r);
  return b1 || b2;

}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {

  // TODO Part 2, task 3:
  // Implement BVH intersection.
  // Currently, we just naively loop over every primitive.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if(!node->bb.intersect(ray, t0, t1)) return false;
  if (node->isLeaf()){
    bool hit = false;
    for (Primitive *p : *(node->prims)) {
      total_isects++;
      if (p->intersect(ray, i)) 
        hit = true;
    }
    return hit;
  }
  bool b1 = intersect(ray,i,node->l);
  bool b2 = intersect(ray,i,node->r);
  return b1 || b2;

}

}  // namespace StaticScene
}  // namespace CGL
