struct Quaternion {
    double x;
    double y;
    double z;
    double w;

    Quaternion (double X, double Y, double Z, double W)
    {
      double d = sqrt( X * X + Y * Y + Z * Z + W * W);
      if(d != 0){
        x = X/d;
        y = Y/d;
        z = Z/d;
        w = W/d;
      }
    }

    Quaternion(Vector_3 p1, Vector_3 p2){
      double X = p1.y()*p2.z() - p1.z()*p2.y();
      double Y = p1.z()*p2.x() - p1.x()*p2.z();;
      double Z = p1.x()*p2.y() - p1.y()*p2.x();;
      double W = 1.0 + p1.x() * p2.x() + p1.y() * p2.y() + p1.z() * p2.z();
      double d = sqrt( X * X + Y * Y + Z * Z + W * W);
      if(d != 0){
        x = X/d;
        y = Y/d;
        z = Z/d;
        w = W/d;
      }
      
    }

    double X() const { return x; }
    double Y() const { return y; }
    double Z() const { return z; }
    double W() const { return w; }

    Transform3 translate(){
      return Transform3 (w*w + x*x - y*y -z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y,
                        2*x*y + 2*w*z, w*w - x*x+ y*y- z*z, 2*y*z - 2*w*x,
                        2*x*z - 2*w*y, 2*y*z + 2*w*x, w*w - x*x - y*y + z*z,
                        1.0);
    }


}; //end of struct