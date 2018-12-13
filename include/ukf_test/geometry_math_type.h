#ifndef GEOMETRY_MATH_TYPE_H_
#define GEOMETRY_MATH_TYPE_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

void get_dcm_from_q(Eigen::Matrix3d &dcm, const Eigen::Quaterniond &q) {
    float a = q.w();
    float b = q.x();
    float c = q.y();
    float d = q.z();
    float aSq = a*a;
    float bSq = b*b;
    float cSq = c*c;
    float dSq = d*d;
    dcm(0, 0) = aSq + bSq - cSq - dSq; 
    dcm(0, 1) = 2 * (b * c - a * d);
    dcm(0, 2) = 2 * (a * c + b * d);
    dcm(1, 0) = 2 * (b * c + a * d);
    dcm(1, 1) = aSq - bSq + cSq - dSq;
    dcm(1, 2) = 2 * (c * d - a * b);
    dcm(2, 0) = 2 * (b * d - a * c);
    dcm(2, 1) = 2 * (a * b + c * d);
    dcm(2, 2) = aSq - bSq - cSq + dSq;
}

void get_q_from_dcm(Eigen::Quaterniond &q, const Eigen::Matrix3d &dcm) {
    float t = dcm.trace();
    if ( t > 0.0f ) {
        t = sqrt(1.0f + t);
        q.w() = 0.5f * t;
        t = 0.5f / t;
        q.x() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(0,2) - dcm(2,0)) * t;
        q.z() = (dcm(1,0) - dcm(0,1)) * t;
    } else if (dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2)) {
        t = sqrt(1.0f + dcm(0,0) - dcm(1,1) - dcm(2,2));
        q.x() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(0,2) + dcm(2,0)) * t;
    } else if (dcm(1,1) > dcm(2,2)) {
        t = sqrt(1.0f - dcm(0,0) + dcm(1,1) - dcm(2,2));
        q.y() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(0,2) - dcm(2,0)) * t;
        q.x() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(2,1) + dcm(1,2)) * t;
    } else {
        t = sqrt(1.0f - dcm(0,0) - dcm(1,1) + dcm(2,2));
        q.z() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(1,0) - dcm(0,1)) * t;
        q.x() = (dcm(0,2) + dcm(2,0)) * t;
        q.y() = (dcm(2,1) + dcm(1,2)) * t;
    }
}

void get_euler_from_R(Eigen::Vector3d &e, const Eigen::Matrix3d &R) {
   float phi = atan2f(R(2, 1), R(2, 2));
   float theta = asinf(-R(2, 0));
   float psi = atan2f(R(1, 0), R(0, 0));
   float pi = M_PI;

   if (fabsf(theta - pi/2.0f) < 1.0e-3) {
       phi = 0.0f;
       psi = atan2f(R(1, 2), R(0, 2));
   } else if (fabsf(theta + pi/2.0f) < 1.0e-3) {
       phi = 0.0f;
       psi = atan2f(-R(1, 2), -R(0, 2));
   }
   e(0) = phi;
   e(1) = theta;
   e(2) = psi;
}

void get_euler_from_q(Eigen::Vector3d &e, const Eigen::Quaterniond &q) {
    Eigen::Matrix3d temp_R;
    get_dcm_from_q(temp_R, q);
    get_euler_from_R(e, temp_R);
}

void get_q_from_euler(Eigen::Quaterniond &q, const Eigen::Vector3d &e) {

    float cosPhi_2 = float(cos(e(0) / float(2.0)));
    float cosTheta_2 = float(cos(e(1) / float(2.0)));
    float cosPsi_2 = float(cos(e(2) / float(2.0)));
    float sinPhi_2 = float(sin(e(0) / float(2.0)));
    float sinTheta_2 = float(sin(e(1) / float(2.0)));
    float sinPsi_2 = float(sin(e(2) / float(2.0)));
    q.w() = cosPhi_2 * cosTheta_2 * cosPsi_2 +
            sinPhi_2 * sinTheta_2 * sinPsi_2;
    q.x() = sinPhi_2 * cosTheta_2 * cosPsi_2 -
            cosPhi_2 * sinTheta_2 * sinPsi_2;
    q.y() = cosPhi_2 * sinTheta_2 * cosPsi_2 +
            sinPhi_2 * cosTheta_2 * sinPsi_2;
    q.z() = cosPhi_2 * cosTheta_2 * sinPsi_2 -
            sinPhi_2 * sinTheta_2 * cosPsi_2;

}

void get_dcm_from_euler(Eigen::Matrix3d &R, const Eigen::Vector3d &e) {
    // Eigen::Quaterniond temp_q;
    // get_q_from_euler(temp_q, e);
    // get_dcm_from_q(R, temp_q);
        float cosPhi = float(cos(e(0)));
        float sinPhi = float(sin(e(0)));
        float cosThe = float(cos(e(1)));
        float sinThe = float(sin(e(1)));
        float cosPsi = float(cos(e(2)));
        float sinPsi = float(sin(e(2)));

        R(0, 0) = cosThe * cosPsi;
        R(0, 1) = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
        R(0, 2) = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

        R(1, 0) = cosThe * sinPsi;
        R(1, 1) = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
        R(1, 2) = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

        R(2, 0) = -sinThe;
        R(2, 1) = sinPhi * cosThe;
        R(2, 2) = cosPhi * cosThe;

}

void get_eR_from_two_R(Eigen::Vector3d &res, const Eigen::Matrix3d &Rd, const Eigen::Matrix3d &R) {
    Eigen::Matrix3d temp_eR;
    temp_eR = (R.transpose()*Rd - Rd.transpose()*R) / 2.0f;
    res(0) = temp_eR(2,1);
    res(1) = temp_eR(0,2);
    res(2) = temp_eR(1,0);
}

void get_R_from_R_cha_e(Eigen::Matrix3d &res, const Eigen::Matrix3d &R, const Eigen::Vector3d &de) {
    Eigen::Matrix3d Omega_cha;
    Omega_cha.setIdentity();
    Omega_cha(0,1) = -de(2);
    Omega_cha(0,2) = de(1);
    Omega_cha(1,0) = de(2);
    Omega_cha(1,2) = -de(0);
    Omega_cha(2,0) = -de(1);
    Omega_cha(2,1) = de(0);
    res = R * Omega_cha;
    Eigen::Vector3d axis_temp;
    axis_temp(0) = std::sqrt(res(0,0)*res(0,0) + res(1,0)*res(1,0) + res(2,0)*res(2,0));
    axis_temp(1) = std::sqrt(res(0,1)*res(0,1) + res(1,1)*res(1,1) + res(2,1)*res(2,1));
    axis_temp(2) = std::sqrt(res(0,2)*res(0,2) + res(1,2)*res(1,2) + res(2,2)*res(2,2));

    for (int i = 0; i < 3 ; i++) {
        for (int j = 0; j < 3; j++) {
            res(i,j) = res(i,j)/axis_temp(j);
        }
    }
}

#endif
