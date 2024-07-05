#include "Quaternion.hpp"

// Default constructor
Quaternion::Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}

// Parameterized constructor
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {
    
}

// Norm of the quaternion
float Quaternion::norm() const {
    return sqrt(w * w + x * x + y * y + z * z);
}

// Conjugate of the quaternion
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

// Inverse of the quaternion
Quaternion Quaternion::inverse() const {
    float normSq = norm();
    normSq = normSq * normSq;
    return conjugate() / normSq;
}

// Quaternion addition
Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

// Quaternion subtraction
Quaternion Quaternion::operator-(const Quaternion& q) const {
    return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
}

// Quaternion multiplication
Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

// Quaternion scalar multiplication
Quaternion Quaternion::operator*(float scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

// Quaternion scalar division
Quaternion Quaternion::operator/(float scalar) const {
    return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
}

std::ostream& operator<<(std::ostream& out, const Quaternion& q) {
    out << "w: " << q.w << ", x: " << q.x << ", y: " << q.y << ", z: " << q.z;
    return out;
}

// Normalize the quaternion
void Quaternion::normalize() {
    float n = norm();
    if (n > 0) {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }
}

Quaternion Quaternion::from_rotvec(const float *rotvec) {
    float angle = sqrt(rotvec[0] * rotvec[0] + rotvec[1] * rotvec[1] + rotvec[2] * rotvec[2]);
    float scale;
    if(angle < 0.001) {
            float angle2 = angle * angle;
            scale = 0.5 - angle2 / 48 + angle2 * angle2 / 3840;
    } else {
        scale = sin(angle / 2) / angle;
    }
    Quaternion quat = Quaternion(
        cos(angle / 2.0),
        scale * rotvec[0],
        scale * rotvec[1],
        scale * rotvec[2]
        );
    return quat;
}

float* Quaternion::to_euler(const Quaternion quat) {
    float euler[3] = {0, 0, 0};
    double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    euler[0] = yaw;
    euler[1] = pitch;
    euler[2] = roll;
    return euler;
}