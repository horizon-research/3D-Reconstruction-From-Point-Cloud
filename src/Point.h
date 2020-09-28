struct Point {
    double ver[3];
    double normal[3];
    int color[3];
    double U;
    double V;

    Point() { ver[0]= ver[1] = ver[2] = 0; }
    Point (double x, double y, double z) { ver[0]=x; ver[1]=y; ver[2]=z; }
    Point (double x, double y, double z, double nx, double ny, double nz, int r, int g, int b, double u, double v)
    {
       ver[0] = x; ver[1] = y; ver[2] = z;
       normal[0] = nx; normal[1] = ny; normal[2] = nz;
       color[0] = r; color[1] = g; color[2] = b;
       U = u; V = v;
    }
    
    Point (double x, double y, double z, double nx, double ny, double nz, int r, int g, int b)
    {
        ver[0] = x; ver[1] = y; ver[2] = z;
        normal[0] = nx; normal[1] = ny; normal[2] = nz;
        color[0] = r; color[1] = g; color[2] = b;
    }

    double x() const { return ver[0]; }
    double y() const { return ver[1]; }
    double z() const { return ver[2]; }

    double& x() { return ver[0]; }
    double& y() { return ver[1]; }
    double& z() { return ver[2]; }

    double nx() const { return normal[0]; }
    double ny() const { return normal[1]; }
    double nz() const { return normal[2]; }

    double& nx() { return normal[0]; }
    double& ny() { return normal[1]; }
    double& nz() { return normal[2]; }

    int r() const { return color[0]; }
    int g() const { return color[1]; }
    int b() const { return color[2]; }

    int& r() { return color[0]; }
    int& g() { return color[1]; }
    int& b() { return color[2]; }

    double u() const {return U;}
    double v() const {return V;}

    double& u() {return U;}
    double& v() {return V;}

    std::string detail() const
    {
        std::string res;
        for(int i = 0; i < 3; ++i) res += std::to_string(ver[i]) + ' ';
        for(int i = 0; i < 3; ++i) res += std::to_string(normal[i]) + ' ';
        for(int i = 0; i < 2; ++i) res += std::to_string(color[i]) + ' ';
        res += std::to_string(color[2]);
        return res;
    }

    std::string detailWithUV() const
    {
        std::string res;
        for(int i = 0; i < 3; ++i) res += std::to_string(ver[i]) + ' ';
        for(int i = 0; i < 3; ++i) res += std::to_string(normal[i]) + ' ';
        for(int i = 0; i < 3; ++i) res += std::to_string(color[i]) + ' ';
        res += std::to_string(U) + ' ';
        res += std::to_string(V);
        return res;
    }

    bool operator==(const Point& p) const
    {
        return (x() == p.x()) && (y() == p.y()) && (z() == p.z());
    }

    bool operator!=(const Point& p) const { return ! (*this == p); }

}; //end of struct

struct Construct_coord_iterator {
    typedef  const double* result_type;
    const double* operator()(const Point& p) const
    { return static_cast<const double*>(p.ver); }

    const double* operator()(const Point& p, int)  const
    { return static_cast<const double*>(p.ver+3); }
};

// Note: This is implemented for std::set
// Not intended for ordering
// Idea from https://stackoverflow.com/questions/1114856/stdset-with-user-defined-type-how-to-ensure-no-duplicates
struct point_set_comparator {
    bool operator()(const Point& a, const Point& b) const
    {
        return !(a == b);
    }
};
