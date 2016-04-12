#include "lenscamera.h"
#include "fstream"
#include "image.h"
#include <ctime>
// #include <String>

using namespace std;

namespace CGL {


/****** Helpers ******/
  

// Extract the R, G, or B channel out of an RGBA color stored in a single 32bit integer
static uint32_t red_channel(uint32_t color) {
    return (255 & (color >> (0)));
}

static uint32_t green_channel(uint32_t color) {
    return (255 & (color >> (8)));
}

static uint32_t blue_channel(uint32_t color) {
    return (255 & (color >> (16)));
}

// Convert from millimeters to meters
static const double scale = .001;


/****** LensElement functions ******/


bool LensElement::pass_through(Ray &r, double &prev_ior) const {
  // Part 1 Task 1: Implement this. It takes r and passes it through this lens element.
  // printf("%f\n",(r.o.z));
  // printf("Center: %f, Radius: %f, ior: %f, aperture: %f\n",center,radius,ior,aperture);
  Vector3D hit_p;
  if (radius == 0){
    double t =(r.o.z - center)/r.d.z;
    hit_p = r.o + t*r.d;
    if(sqrt(pow(hit_p.x,2)+pow(hit_p.y,2)) > aperture/2){
      return false;
    }

    return true;
  }
 
  if(intersect(r,&hit_p)){
    if(sqrt(pow(hit_p.x,2)+pow(hit_p.y,2)) > aperture/2) {
      // printf("555-----> False1 TAT\n");
      return false;
    }
    // printf("LOL---->True =v=\n");
    if(refract(r, hit_p, prev_ior)){
      // if(radius > 0)
      prev_ior = ior;
    } else{
      return false;
    }

    // printf("%f,%f,%f\n",hit_p.x,hit_p.y,hit_p.z );
    return true;
  }
  else {
    // printf("555-----> False2 TAT\n");
    return false;
  }
  
}
bool LensElement::intersect(const Ray &r, Vector3D *hit_p) const {
  // Part 1 Task 1: Implement this. It intersects r with this spherical lens elemnent 
  // (or aperture diaphragm). You'll want to reuse some sphere intersect code.
  Vector3D o = Vector3D(0,0,center);
  double a = dot(r.d,r.d),
         b = 2*dot(r.o - o, r.d),
         c = dot(r.o-o,r.o-o) - pow(radius,2),
         delta = b*b-4*a*c,
         t1 = 0,
         t2 = 0,
         t = 0;
  
  if(delta >= 0){
    t2 = (-b+sqrt(delta))/(2*a);
    t1 = (-b-sqrt(delta))/(2*a);
    if(r.d.z <0){
      if (radius<0){
        t = t1;
      } else{
        t = t2;
      }
    }
    else {
      if (radius<0){
        t = t2;
      } else{
        t = t1;
      }
    }
    if(t>=r.min_t && t<=r.max_t){

    }
    else return false;
    *hit_p = r.o+t*r.d;
    // r.max_t = t;
    return true;
  }
  else {
    return false;
  }
  
}
bool LensElement::refract(Ray& r, const Vector3D& hit_p, const double& prev_ior) const {
  // Part 1 Task 1: Implement this. It refracts the Ray r with this lens element or 
  // does nothing at the aperture element.
  // You'll want to consult your refract function from the previous assignment.
  Matrix3x3 o2w;
  Vector3D normal;
  // printf("%f,%f,%f\n", hit_p.x,hit_p.y,hit_p.z);
  if(r.d.z <0){
    if(radius<0){
      normal = hit_p - Vector3D(0,0,center);
    }
    else{
      normal = Vector3D(0,0,center) - hit_p;
    }
  }
  else{
    if(radius<0){
      normal = Vector3D(0,0,center) - hit_p;
    }
    else{
      
      normal = hit_p - Vector3D(0,0,center);
    }
  }
  make_coord_space(o2w, normal);
  Matrix3x3 w2o = o2w.T();
  Vector3D wi;
  

  // printf("%f,%f\n", r.d.z,wo.z);
  double n = 0.0;
  if(r.d.z<0){
    n = prev_ior/ior;
  }
  else {
    n = ior/prev_ior;
  }
  Vector3D wo = w2o * (-r.d);  
  wi = Vector3D(-sin_theta(wo)*cos_phi(wo)*n,
                 -sin_theta(wo)*sin_phi(wo)*n,
                 -sqrt(1 - pow(sin_theta(wo),2)*(n*n)));
  if(sin_theta(wo)*n>= 1)
    return false;
  else{
    r.d = o2w*wi;
    r.o = hit_p;
    return true;
  }

}






/****** Lens functions ******/



void Lens::parse_lens_file(std::string filename) {

  ifstream infile(filename);
  string line;
  double z_coord = 0;
  double z_ap;
  vector<LensElement> backwards;
  elts.clear();
  bool first = true;
  while (getline(infile, line)) {
    if (first) {
      cout << "[Lens] Loading lens file " << line << endl;
      first = false;
    }
    if (line[0] == '#')
      continue;
    stringstream ss(line);
    LensElement lens;
    double offset;
    ss >> lens.radius >> offset >> lens.ior >> lens.aperture;
    lens.center = z_coord;
    if (!lens.radius) {
      z_ap = z_coord;
    }
    z_coord += offset;
    backwards.push_back(lens);
  }
  for (int i = backwards.size() - 1; i >= 0; --i) {
    LensElement l = backwards[i];
    l.center = (l.center - z_ap) + l.radius;
    if (i) l.ior = backwards[i-1].ior;
    else l.ior = 1;
    if (!l.ior) l.ior = 1;
    elts.push_back(l);
    if (!l.radius)
      ap_i = elts.size()-1;
    // cout << "Lens element edge first " << (l.center - l.radius) << " " 
    //   << l.radius << " " << l.center << " " << l.ior << " " << l.aperture << endl;
  }
  double c = elts.front().center, r = elts.front().radius, a = elts.front().aperture * .5;
  back_elt = c - (r>0?1:-1) * sqrt(r*r-a*a);
  ap_radius = ap_original = elts[ap_i].aperture;

  // Get infinity and close focus depths, also get focal length.
  set_focus_params();
  // Focus at infinity to start.
  sensor_depth = infinity_focus;
       
}


void Lens::set_focus_params() {;

  // Part 1 Task 2: Implement th  is. 
  // After this function is called, the three variables
  // infinity_focus, near_focus, and focal_length
  // should be set correctly.
  Ray r = Ray(Vector3D(0.001,0,-9999999),Vector3D(0,0,1));
  Ray ro = Ray(Vector3D(0.001,0,-9999999),Vector3D(0,0,1));
  std::vector<Vector3D> trace;
  trace_backwards(r,&trace);
  Vector3D fp = r.o + (-r.o.x/r.d.x)*r.d;
  infinity_focus = fp.z;
  Vector3D pp = r.o+((ro.o.x-r.o.x)/r.d.x)*r.d;
  focal_length = abs(pp.z-fp.z);
  std::vector<Vector3D> trace2;
  ro = Ray(Vector3D(0,0,-5*focal_length),Vector3D(0.00001,0,1));
  trace_backwards(ro,&trace2);
  Vector3D nf = ro.o + (-ro.o.x/ro.d.x)*ro.d;
  near_focus = nf.z;
  // printf("%f,%f,%f\n",fp.x,fp.y,fp.z );
  std::string s = "focus_depth";
  s.append(std::to_string(random_uniform()));
  s.append(".txt");
  ofstream fout(s);
  for(double i = infinity_focus; i <= near_focus; i+=(near_focus- infinity_focus)/100){
    fout << i << " " << focus_depth(i) << endl;
  }
  fout.close();




  cout << "[Lens] Infinity focus depth is " << infinity_focus << endl;
  cout << "[Lens] Close focus depth is " << near_focus << endl;
  cout << "[Lens] True focal length is " << focal_length << endl;
}




bool Lens::trace(Ray &r, std::vector<Vector3D> *trace) const {
  // Part 1 Task 1: Implement this. It traces a ray from the sensor out into the world.
  double prev_ior = 1.0;
  // bool b = true;
  for(LensElement el : elts){
    if(el.pass_through(r,prev_ior)){
      // prev_ior = el.ior;
      
      // return false;
      trace->push_back(r.o);
    } else{
       return false;
    }
    
  }
  // elts[0].pass_through(r,prev_ior);

  // printf("12312312323\n");
  return true;
}

bool Lens::trace_backwards(Ray &r, std::vector<Vector3D> *trace) const {
  // Part 1 Task 1: Implement this. It traces a ray from the world backwards through 
  // the lens towards the sensor.
  double prev_ior = 1.0;
  for (int i = elts.size()-1;i>=0;i--){
    if(i != 0){
      prev_ior = elts[i-1].ior;
    } else{
      prev_ior = 1.0;
    }
    if(elts[i].pass_through(r,prev_ior)){
      // prev_ior = elts[i+1].ior;
      trace->push_back(r.o);
    }
  }
  

  //  for(LensElement el : elts){
  //   printf("%f\n",el.ior );
  // }

  // elts[elts.size()-7].pass_through(r,prev_ior);
  // trace->push_back(r.o);
  return true;
}

float Lens::focus_depth(float d) const {

  // Part 1 Task 2: Implement this. Should find the conjugate of a ray
  // starting from the sensor at depth d.
  Ray r = Ray(Vector3D(0,0,d),Vector3D(0.02,0,-1).unit());
  std::vector<Vector3D> trace1;
  trace(r,&trace1);
  Vector3D fd = r.o+(-r.o.x/r.d.x)*r.d;
  return fd.z;
}

Vector3D Lens::back_lens_sample() const {

  // Part 1 Task 2: Implement this. Should return a point randomly sampled
  // on the back element of the lens (the element closest to the sensor)
  double theta = 2*M_PI*random_uniform();
  double r = elts[0].aperture*0.5*sqrt(random_uniform());

  return Vector3D(r*cos(theta),r*sin(theta),elts[0].center - elts[0].radius);

}



/****** LensCamera functions ******/


LensCamera::LensCamera(): pt(NULL) {
  string path = string(__FILE__).substr(0,string(__FILE__).find_last_of('/')+1) + "../lenses/";
  static const vector<string> lens_files = {"dgauss.50mm.dat", "wide.22mm.dat", "telephoto.250mm.dat", "fisheye.10mm.dat"};
  for (string lens_file : lens_files)
    lenses.emplace_back(path + lens_file);

  mount_lens(0);
}


Ray LensCamera::generate_ray(double x, double y, int& rays_tried, double & coss) const {

  Ray r = Ray(Vector3D(),Vector3D() );
  if (lens_ind >= 0) {
    // Part 1 Task 2: Implement this. It generates a ray from sensor pixel (x,y)
    // pointing toward the back element of the lens (use back_lens_sample) and traces
    // it through the Lens (using your "trace" function)
    const Lens& l = lenses[lens_ind];
    double film_d = sqrt(24*24+36*36);
    double screen_d = sqrt(screenW*screenW + screenH*screenH);
    double film_w = film_d * screenW / screen_d;
    double film_h = film_d * screenH / screen_d;
    Vector3D sensor_point(-(x-0.5)*film_w, -(y-0.5)*film_h, l.sensor_depth);
    for(rays_tried = 1; rays_tried<=20;rays_tried++){
      Vector3D bls = l.back_lens_sample();
    // printf("%f,%f,%f\n",bls.x,bls.y,bls.z );
    // printf("%f\n",l.elts[0].aperture/2);
      r = Ray(sensor_point,(bls - sensor_point).unit());
      std::vector<Vector3D> trace1;
      coss = pow(r.d.z,4);
      if(l.trace(r,&trace1)){
        // r.d = Vector3D(0,0,1);
        break;
      }
      else{
        coss = 0;
      }

    }
    // l.trace(r,&trace1);


    /***** end of your code ******/


    // This code converts the ray you traced through the lens into world coordinates.
    r.o = pos + c2w * r.o * scale;
    r.d = (c2w * r.d).unit();

  } else {

    // Generate ray for a pinhole camera. Same as in the previous assignment.
    x = 2*(x-.5); y = 2*(y-.5);
    r = Ray(pos,(c2w*Vector3D(x*tan(radians(hFov)*.5),y*tan(radians(vFov)*.5),-1)).unit());
    rays_tried = 1;
    coss = 1;

  }

  r.min_t = nClip; r.max_t = fClip;
  return r;
}



void LensCamera::move_sensor(float delta) {
  if (lens_ind < 0) return;
  curr_lens().sensor_depth += delta;
  cout << "[LensCamera] Sensor plane moved to " << curr_lens().sensor_depth
       << ", focus now at " << lenses[lens_ind].focus_depth(lenses[lens_ind].sensor_depth) << endl;
}

void LensCamera::stop_down(float ratio) {
  float ap = curr_lens().ap_radius * ratio;
  if (ap > curr_lens().ap_original) ap = curr_lens().ap_original;
  curr_lens().ap_radius = ap;
  cout << "[LensCamera] Aperture is now " << curr_lens().ap_radius << "mm" << endl;
}

void LensCamera::mount_lens(int i) {
  lens_ind = i;
  if (i >= 0) {
    cout << "[LensCamera] Switched to lens #" << (i+1) 
         << " with focal length " << curr_lens().focal_length << "mm" << endl;
  } else {
    cout << "[LensCamera] Switched to pinhole camera" << endl;
  }
}



// A dummy function to demonstrate how to work with the image buffer.
// Calculates the average value of the green color channel in the image.
// You'll have to remember your 2D array indexing in order to take differences
// of neighboring pixels in a more sophisticated metric function.
static double mean_green(const ImageBuffer& ib) {
  double sum = 0;
  for (int i = 0; i < ib.w * ib.h; ++i) {
      sum += green_channel(ib.data[i]);
  }
  double mean = sum / (ib.w * ib.h);
  
  return mean;
}

// static double cal_variance(const ImageBuffer& ib,uint32_t (*fn) (uint32_t)) {
//   double sum = 0;
//   for (int i = 0; i < ib.w * ib.h; ++i) {
//       sum += fn(ib.data[i]);
//   }
//   double mean = sum / (ib.w * ib.h);
//   sum = 0;
//   for (int i = 0; i < ib.w * ib.h; ++i) {
//       sum += pow(fn(ib.data[i])-mean,2);
//   }
  
//   return sum / (ib.w * ib.h);
// }

double LensCamera::focus_metric(const ImageBuffer& ib) const {

  // Part 2 Task 1: Implement this. Design a metric to judge how "in-focus"
  // the image patch stored in the provided ImageBuffer is.
  double sumr=0,sumg=0,sumb=0,
         meanr=0,meang=0,meanb=0;
  for (int i = 0; i < ib.w * ib.h; ++i) {
      sumr += red_channel(ib.data[i]);
      sumg += green_channel(ib.data[i]);
      sumb += blue_channel(ib.data[i]);
  }
  meanr = sumr/ (ib.w * ib.h); meang = sumg/(ib.w * ib.h); meanb = sumb/(ib.w * ib.h);
  sumr=0; sumg=0; sumb=0;
  for (int i = 0; i < ib.w * ib.h; ++i) {
      sumr += pow(red_channel(ib.data[i])-meanr,2);
      sumg += pow(green_channel(ib.data[i])-meang,2);
      sumb += pow(blue_channel(ib.data[i])-meanb,2);
  }
  return (sumr + sumg + sumb)/(ib.w * ib.h);


  // return cal_variance(ib,green_channel)+cal_variance(ib,red_channel)+cal_variance(ib,blue_channel); //  A meaningless standin
}


void LensCamera::autofocus() {


  // Part 2 Task 2: Implement this. Design a global search using your 
  // focus metric to set the sensor to be at the depth where the 
  // render cell is most "in focus". Provided code shows how to 
  // move the sensor, request a render of the cell, and evaluate the focus metric.

  // This call ensures that your pathtracer is rendering at high enough quality.
  // Increase samples per pixel to 16 and samples per light to 16.
  pt->bump_settings();

  // Example code. Nothing to do with your actual implementation except to 
  // demonstrate functionality.
  ImageBuffer ib;
  double a = -INF_D;
  double c = 0;
  double best_value;
  double nf = curr_lens().near_focus;
  double inf = curr_lens().infinity_focus;
  double b = (nf - inf)/15;
  double C = sqrt(36*36 + 24*24) / sqrt(screenW*screenW + screenH*screenH);
  double fnum = curr_lens().focal_length/curr_lens().ap_radius;
  b = min(C*fnum,b);
  double s1 = inf;
  double s2 = nf;
  int i = 0;


  std::string s = "part2lens";
  s.append(std::to_string(lens_ind+1));
  s.append(".txt");
  ofstream fout(s);

  std::clock_t start;
    double duration;

    start = std::clock();
  //Speed upped method 

  while((s2-s1)>b){
    for(curr_lens().sensor_depth = s1; curr_lens().sensor_depth <= s2; curr_lens().sensor_depth += ((s2-s1)/8)){
      pt->raytrace_cell(ib);
      c = focus_metric(ib);  

      cout << "[LensCamera] The variance is " << c << " for sensor depth " << curr_lens().sensor_depth << endl;
      fout << curr_lens().sensor_depth << " " << c << endl;
      if(c>a){
        a = c;
        best_value = curr_lens().sensor_depth;
      }
    }
    if(best_value + ((s2-s1)/8) < nf){
      s2 = best_value + ((s2-s1)/8);
    } else {
      s2 = best_value;
    }
    if (best_value - ((s2-s1)/8) > inf){
      s1 = best_value - ((s2-s1)/8);
    } else {
      s1 = best_value;
    }
    i++;
    if(i == 3){
      break;
    }
  }

  //regular method
  // for(curr_lens().sensor_depth  = inf; curr_lens().sensor_depth <= nf; curr_lens().sensor_depth += b){
  //   pt->raytrace_cell(ib);
  //   c = focus_metric(ib);
  //   cout << "[LensCamera] The variance is " << c << " for sensor depth " << curr_lens().sensor_depth << endl;
  //   fout << curr_lens().sensor_depth << " " << c << endl;
  //   if(c>a){
  //     a = c;
  //     best_value = curr_lens().sensor_depth;
  //   }
  // }
  duration = (std::clock() - start )/ (double) CLOCKS_PER_SEC;
  std::cout<<"[LensCamera] The method takes "<< duration << "second." << endl;
  fout.close();
  curr_lens().sensor_depth = best_value;
  cout << "[LensCamera] The best sensor depth is " << best_value << endl;
  pt->raytrace_cell(ib);

  
}





void LensCamera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;

  file << lens_ind << endl;
  for (Lens &lens : lenses) {
    file << lens.sensor_depth << " ";
  }
  file << endl;

  cout << "[LensCamera] Dumped settings to " << filename << endl;
}

void LensCamera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;

  file >> lens_ind;
  for (Lens &lens : lenses) {
    file >> lens.sensor_depth;
  }

  cout << "[LensCamera] Loaded settings from " << filename << endl;
}


} // namespace CGL

