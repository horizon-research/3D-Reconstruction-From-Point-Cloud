struct Point {
  double vec[3];
  double normal[3];
  int color[3];
  double U;
  double V;

  Point() { vec[0]= vec[1] = vec[2] = 0; }
  Point (double x, double y, double z) { vec[0]=x; vec[1]=y; vec[2]=z;  }
  Point (double x, double y, double z, double nx, double ny, double nz, int r, int g, int b, double u, double v)
   { 
     vec[0] = x; vec[1] = y; vec[2] = z;  
     normal[0] = nx; normal[1] = ny; normal[2] = nz;
     color[0] = r; color[1] = g; color[2] = b;
     U = u; V = v;
   }
  Point (double x, double y, double z, double nx, double ny, double nz, int r, int g, int b)
  { 
    vec[0] = x; vec[1] = y; vec[2] = z;  
    normal[0] = nx; normal[1] = ny; normal[2] = nz;
    color[0] = r; color[1] = g; color[2] = b;
  }

  double x() const { return vec[ 0 ]; }
  double y() const { return vec[ 1 ]; }
  double z() const { return vec[ 2 ]; }

  double& x() { return vec[ 0 ]; }
  double& y() { return vec[ 1 ]; }
  double& z() { return vec[ 2 ]; }

  double nx() const { return normal[ 0 ]; }
  double ny() const { return normal[ 1 ]; }
  double nz() const { return normal[ 2 ]; }

  double& nx() { return normal[ 0 ]; }
  double& ny() { return normal[ 1 ]; }
  double& nz() { return normal[ 2 ]; }

  int r() const { return color[ 0 ]; }
  int g() const { return color[ 1 ]; }
  int b() const { return color[ 2 ]; }

  int& r() { return color[ 0 ]; }
  int& g() { return color[ 1 ]; }
  int& b() { return color[ 2 ]; }

  double u() const {return U;}
  double v() const {return V;}

  double& u() {return U;}
  double& v() {return V;}

  // string detail() const {
  //   string res = x + " " + y + " " + z + " " + nx + " " + ny + " " + nz + ' ' + r + ' ' + g + ' ' + b;
  //   return res;
  // }

  bool operator==(const Point& p) const
  {
    return (x() == p.x()) && (y() == p.y()) && (z() == p.z())  ;
  }

  bool  operator!=(const Point& p) const { return ! (*this == p); }
}; //end of class

struct Construct_coord_iterator {
  typedef  const double* result_type;
  const double* operator()(const Point& p) const
  { return static_cast<const double*>(p.vec); }

  const double* operator()(const Point& p, int)  const
  { return static_cast<const double*>(p.vec+3); }
};
