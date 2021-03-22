#define ARDUINO 1

#include "PrintHelpers.hpp"

#ifndef ARDUINO
#include <iostream>
#include <cmath>
#endif

class quaternion{
    public:
        quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {};
        quaternion(quaternion *q):  w(q->w), x(q->x), y(q->y), z(q->z) {};
        quaternion():  w(0), x(0), y(0), z(0) {};
        ~quaternion(){};

        float get_w(){return w;}
        float get_x(){return x;}
        float get_y(){return y;}
        float get_z(){return z;}

        quaternion ham(quaternion &q1);
        quaternion conj(){return quaternion(w, -x, -y, -z);};

    protected:
        float w = 0;
        float x = 0;
        float y = 0;
        float z = 0;
#ifndef ARDUINO
    friend std::ostream& operator<<(std::ostream& os, quaternion& dt);
#endif
};

#ifndef ARDUINO
std::ostream& operator<<(std::ostream& os, quaternion& q1)
{
    os << "(" << q1.w << ", " << q1.x << ", " << q1.y << ", " << q1.z << ")";
    return os;
}
#else

// inline Print &operator <<(Print &strm, const int obj)
// {
//     strm.print(obj); return strm;
// }

// inline Print &operator <<(Print &strm, const float obj)
// {
//     strm.print(obj); return strm;
// }

// inline Print &operator <<(Print &strm, const char *obj)
// {
//     strm.print(obj); return strm;
// }

// inline Print &operator <<(Print &strm, const char obj)
// {
//     strm.print(obj); return strm;
// }

// template<int rows, int cols, class MemT>
inline Print &operator<<(Print &strm, const quaternion &q1)
{
    strm << "(" << q1.get_w() << ", " << q1.get_x() << ", " << q1.get_y() << ", " << q1.get_z() << ")";
    return strm;
}
#endif

quaternion quaternion::ham(quaternion &q1){
    return quaternion(-q1.get_x() * x - q1.get_y() * y - q1.get_z() * z + q1.get_w() * w,
                     q1.get_x() * w + q1.get_y() * z - q1.get_z() * y + q1.get_w() * x,
                     -q1.get_x() * z + q1.get_y() * w + q1.get_z() * x + q1.get_w() * y,
                     q1.get_x() * y - q1.get_y() * x + q1.get_z() * w + q1.get_w() * z);
}


class rot: public quaternion{
    private:
        float angle = 0;
    public:
        rot(float angle, float* vector):quaternion(){
            set_angle_vector(angle, vector);
        }

        rot():quaternion(1, 0, 0, 0){
            this->angle = 0;
        }

        set_angle_vector(float angle, float* vector){
            this->angle = angle;
            set_vector(vector);
        }

        set_vector(float* vector){
            w = cos(this->angle/2);
            float s2 = sin(this->angle/2);
            float len = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
            x = vector[0]*s2/len;
            y = vector[1]*s2/len;
            z = vector[2]*s2/len;
        }

        set_vector(float x, float y, float z){
            w = cos(this->angle/2);
            float s2 = sin(this->angle/2);
            float len = sqrt(x*x + y*y + z*z);
            this->x = x*s2/len;
            this->y = y*s2/len;
            this->z = z*s2/len;
        }

        set_angle(float angle){
          this->angle = angle;
          float s2 = sin(this->angle/2);
          float len = sqrt(x*x + y*y + z*z);
          
          this->w = cos(this->angle/2);

          // rescale by len
          this->x = x*s2/len;
          this->y = y*s2/len;
          this->z = z*s2/len;
        }

        quaternion rotate(quaternion &q){
            quaternion temp = conj();
            quaternion t2 = q.ham(temp);
            return ham(t2);
        }
    
};
