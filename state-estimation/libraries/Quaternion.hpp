#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>


class Quaternion {
    public:
        // Members
        float w, x, y, z;
        
        // Constructors
        Quaternion();
        Quaternion(float w, float x, float y, float z);
    
        // Methods
        float norm() const;
        Quaternion conjugate() const;
        Quaternion inverse() const;
        Quaternion operator+(const Quaternion& q) const;
        Quaternion operator-(const Quaternion& q) const;
        Quaternion operator*(const Quaternion& q) const;
        Quaternion operator*(float scalar) const;
        Quaternion operator/(float scalar) const;
                
        void normalize();
        
        static Quaternion from_rotvec(const float *rotvec);
        static float* to_euler(const Quaternion quat);
};

#endif