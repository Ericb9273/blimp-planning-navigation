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

float* Quaternion::to_euler(const Quaternion& quat) {
    float euler[3] = {0.0, 0.0, 0.0};
    /*
    double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    double roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
    double pitch;
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    
    euler[0] = yaw;
    euler[1] = pitch;
    euler[2] = roll;
    return euler;
    */
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    const double w2 = quat.w * quat.w;
    const double x2 = quat.x * quat.x;
    const double y2 = quat.y * quat.y;
    const double z2 = quat.z * quat.z;
    const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
    const double abcd = quat.w * quat.x + quat.y * quat.z;
    const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
    const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
    if (abcd > (0.5-eps)*unitLength)
    {
        yaw = 2 * atan2(quat.y, quat.w);
        pitch = pi;
        roll = 0;
    }
    else if (abcd < (-0.5+eps)*unitLength)
    {
        yaw = -2 * ::atan2(quat.y, quat.w);
        pitch = -pi;
        roll = 0;
    }
    else
    {
        const double adbc = quat.w * quat.z - quat.x * quat.y;
        const double acbd = quat.w * quat.y - quat.x * quat.z;
        yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
        pitch = ::asin(2*abcd/unitLength);
        roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
    }
    euler[0] = roll;
    euler[1] = pitch;
    euler[2] = yaw;
    return euler;
}