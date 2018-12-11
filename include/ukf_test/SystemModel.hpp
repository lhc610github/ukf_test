#ifndef SYSTEMMODEL_HPP_
#define SYSTEMMODEL_HPP_

#include <kalman/SystemModel.hpp>
#include <iostream>
#include "geometry_math_type.h"

#define ONE_G 9.871

namespace Test1 {

template<typename T>
class State : public Kalman::Vector<T, 21> {
    public:
        KALMAN_VECTOR(State, T, 21)

        //! X-position
        static constexpr size_t X = 0;
        //! Y-position
        static constexpr size_t Y = 1;
        //! Z-position
        static constexpr size_t Z = 2;
        //! X-velocity
        static constexpr size_t VX = 3;
        //! Y-velocity
        static constexpr size_t VY = 4;
        //! Z-velocity
        static constexpr size_t VZ = 5;
        // static constexpr size_t AX = 6;
        // static constexpr size_t AY = 7;
        // static constexpr size_t AZ = 8;
        //! W-orientation
        // static constexpr size_t QW = 6;
        //! X-orientation
        static constexpr size_t QX = 6;
        //! Y-orientation
        static constexpr size_t QY = 7;
        //! Z-orientation
        static constexpr size_t QZ = 8;
        //  static constexpr size_t WX = 12;
        // //! Y-orientation
        // static constexpr size_t WY = 13;
        // //! Z-orientation
        // static constexpr size_t WZ = 14;       //! accelerator bias
        static constexpr size_t baX = 9;
        static constexpr size_t baY = 10;
        static constexpr size_t baZ = 11;
        // //! gyro bias
        static constexpr size_t bwX = 12;
        static constexpr size_t bwY = 13;
        static constexpr size_t bwZ = 14;
        
        static constexpr size_t naX = 15;
        static constexpr size_t naY = 16;
        static constexpr size_t naZ = 17;
        // //! gyro bias
        static constexpr size_t nwX = 18;
        static constexpr size_t nwY = 19;
        static constexpr size_t nwZ = 20;

        T x()        const { return (*this)[ X ]; }
        T y()        const { return (*this)[ Y ]; }
        T z()        const { return (*this)[ Z ]; }
        T vx()       const { return (*this)[ VX ]; }
        T vy()       const { return (*this)[ VY ]; }
        T vz()       const { return (*this)[ VZ ]; }
        // T ax()       const { return (*this)[ AX ]; }
        // T ay()       const { return (*this)[ AY ]; }
        // T az()       const { return (*this)[ AZ ]; }
        // T qw()       const { return (*this)[ QW ]; }
        T qx()       const { return (*this)[ QX ]; }
        T qy()       const { return (*this)[ QY ]; }
        T qz()       const { return (*this)[ QZ ]; }
        // T wx()       const { return (*this)[ WX ]; }
        // T wy()       const { return (*this)[ WY ]; }
        // T wz()       const { return (*this)[ WZ ]; }
        T bax()      const { return (*this)[ baX ]; }
        T bay()      const { return (*this)[ baY ]; }
        T baz()      const { return (*this)[ baZ ]; }
        T bwx()      const { return (*this)[ bwX ]; }
        T bwy()      const { return (*this)[ bwY ]; }
        T bwz()      const { return (*this)[ bwZ ]; }
        T nax()      const { return (*this)[ naX ]; }
        T nay()      const { return (*this)[ naY ]; }
        T naz()      const { return (*this)[ naZ ]; }
        T nwx()      const { return (*this)[ nwX ]; }
        T nwy()      const { return (*this)[ nwY ]; }
        T nwz()      const { return (*this)[ nwZ ]; }

        T& x()        { return (*this)[ X ]; }
        T& y()        { return (*this)[ Y ]; }
        T& z()        { return (*this)[ Z ]; }
        T& vx()       { return (*this)[ VX ]; }
        T& vy()       { return (*this)[ VY ]; }
        T& vz()       { return (*this)[ VZ ]; }
        // T& ax()       { return (*this)[ AX ]; }
        // T& ay()       { return (*this)[ AY ]; }
        // T& az()       { return (*this)[ AZ ]; }
        // T& qw()       { return (*this)[ QW ]; }
        T& qx()       { return (*this)[ QX ]; }
        T& qy()       { return (*this)[ QY ]; }
        T& qz()       { return (*this)[ QZ ]; }
        // T& wx()       { return (*this)[ WX ]; }
        // T& wy()       { return (*this)[ WY ]; }
        // T& wz()       { return (*this)[ WZ ]; }
        T& bax()      { return (*this)[ baX ]; }
        T& bay()      { return (*this)[ baY ]; }
        T& baz()      { return (*this)[ baZ ]; }
        T& bwx()      { return (*this)[ bwX ]; }
        T& bwy()      { return (*this)[ bwY ]; }
        T& bwz()      { return (*this)[ bwZ ]; }

        T& nax()      { return (*this)[ naX ]; }
        T& nay()      { return (*this)[ naY ]; }
        T& naz()      { return (*this)[ naZ ]; }
        T& nwx()      { return (*this)[ nwX ]; }
        T& nwy()      { return (*this)[ nwY ]; }
        T& nwz()      { return (*this)[ nwZ ]; }
};


template<typename T>
class Control : public Kalman::Vector<T, 7> {
    public:
        KALMAN_VECTOR(Control, T, 7)

        //! accelerator
        static constexpr size_t aX = 0;
        static constexpr size_t aY = 1;
        static constexpr size_t aZ = 2;
        //! gyro
        static constexpr size_t wX = 3;
        static constexpr size_t wY = 4;
        static constexpr size_t wZ = 5;

        // static constexpr size_t baX = 6;
        // static constexpr size_t baY = 7;
        // static constexpr size_t baZ = 8;

        // static constexpr size_t bwX = 9;
        // static constexpr size_t bwY = 10;
        // static constexpr size_t bwZ = 11;

        static constexpr size_t DT = 6;

        T ax()      const { return (*this)[ aX ]; }
        T ay()      const { return (*this)[ aY ]; }
        T az()      const { return (*this)[ aZ ]; }
        T wx()      const { return (*this)[ wX ]; }
        T wy()      const { return (*this)[ wY ]; }
        T wz()      const { return (*this)[ wZ ]; }
        // T bax()      const { return (*this)[ baX ]; }
        // T bay()      const { return (*this)[ baY ]; }
        // T baz()      const { return (*this)[ baZ ]; }
        // T bwx()      const { return (*this)[ bwX ]; }
        // T bwy()      const { return (*this)[ bwY ]; }
        // T bwz()      const { return (*this)[ bwZ ]; }
        T dt()      const { return (*this)[ DT ]; }

        T& ax()      { return (*this)[ aX ]; }
        T& ay()      { return (*this)[ aY ]; }
        T& az()      { return (*this)[ aZ ]; }
        T& wx()      { return (*this)[ wX ]; }
        T& wy()      { return (*this)[ wY ]; }
        T& wz()      { return (*this)[ wZ ]; }
        // T& bax()      { return (*this)[ baX ]; }
        // T& bay()      { return (*this)[ baY ]; }
        // T& baz()      { return (*this)[ baZ ]; }
        // T& bwx()      { return (*this)[ bwX ]; }
        // T& bwy()      { return (*this)[ bwY ]; }
        // T& bwz()      { return (*this)[ bwZ ]; }
        T& dt()      { return (*this)[ DT ]; }

};

template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::SystemModel<State<T>, Control<T>, CovarianceBase> {
    public:
        typedef Test1::State<T> S;
        typedef Test1::Control<T> C;

        SystemModel() {
            // Covariance<StateType> _temp_P;
            // _temp_P.setIdentity();
            // _temp_P(0,0) = T(0.5);
            // _temp_P(1,1) = T(0.5);
            // _temp_P(2,2) = T(0.5);
            // _temp_P(3,3) = T(1.0);
            // _temp_P(4,4) = T(1.0);
            // _temp_P(5,5) = T(1.0);
            // _temp_P(6,6) = T(2.0);
            // _temp_P(7,7) = T(2.0);
            // _temp_P(8,8) = T(2.0);
            // _temp_P(9,9) = T(2.0);
            // _temp_P(10,10) = T(2.0);
            // _temp_P(11,11) = T(2.0);
            // setCovariance(_temp_P);
            // this->P(0,0) = T(0.01);
            // this->P(1,1) = T(0.01);
            // this->P(2,2) = T(0.01);

            // this->P(3,3) = T(0.001);
            // this->P(4,4) = T(0.001);
            // this->P(5,5) = T(0.001);

            // this->P(6,6) = T(0.1);
            // this->P(7,7) = T(0.1);
            // this->P(8,8) = T(0.1);

            // // this->P(9,9) = T(0.00005);
            // // this->P(10,10) = T(0.00005);
            // // this->P(11,11) = T(0.00005);
            // this->P(9,9) = T(0.0000005);
            // this->P(10,10) = T(0.0000005);
            // this->P(11,11) = T(0.0000005);
            // this->P(12,12) = T(0.0000001);
            // this->P(13,13) = T(0.0000001);
            // this->P(14,14) = T(0.0000001);

            // // this->P(15,15) = T(0.0002);
            // // this->P(16,16) = T(0.0002);
            // // this->P(17,17) = T(0.0002);
            // this->P(15,15) = T(0.0000002);
            // this->P(16,16) = T(0.0000002);
            // this->P(17,17) = T(0.0000002);
            // this->P(18,18) = T(0.00000002);
            // this->P(19,19) = T(0.00000002);
            // this->P(20,20) = T(0.00000002);       
        }
        
        void set_conv(T c_P, T c_V, T c_Att, T c_ba, T c_bw, T c_na, T c_nw) {
            this->P(0,0) = c_P;
            this->P(1,1) = c_P;
            this->P(2,2) = c_P;

            this->P(3,3) = c_V;
            this->P(4,4) = c_V;
            this->P(5,5) = c_V;

            this->P(6,6) = c_Att;
            this->P(7,7) = c_Att;
            this->P(8,8) = c_Att;

            this->P(9,9) = c_ba;
            this->P(10,10) = c_ba;
            this->P(11,11) = c_ba;
            this->P(12,12) = c_bw;
            this->P(13,13) = c_bw;
            this->P(14,14) = c_bw;

            this->P(15,15) = c_na;
            this->P(16,16) = c_na;
            this->P(17,17) = c_na;
            this->P(18,18) = c_nw;
            this->P(19,19) = c_nw;
            this->P(20,20) = c_nw;
        }
        
        mutable T ax_out;
        mutable T ay_out;
        mutable T az_out;
        mutable T wx_out;
        mutable T wy_out;
        mutable T wz_out;

        S f(const S& x, const C& u) const {
            S x_;
            // T ax = u.ax() - x.bax();
            // T ay = u.ay() - x.bay();
            // T az = u.az() - x.baz();

            // std::cout <<"x: " << x.transpose() << std::endl;
            // std::cout <<"u: " << u.transpose() << std::endl;
            T ax = -u.ax() - x.bax();
            T ay = -u.ay() - x.bay();
            T az = -u.az() - x.baz();
            // T ax = -u.ax();
            // T ay = -u.ay();
            // T az = -u.az();



            T x_new = x.x() + x.vx() * u.dt() + T(1)/T(2) * ax * u.dt() *u.dt();
            T y_new = x.y() + x.vy() * u.dt() + T(1)/T(2) * ay * u.dt() *u.dt();
            T z_new = x.z() + x.vz() * u.dt() + T(1)/T(2) * az * u.dt() *u.dt();

            // Kalman::SquareMatrix<T, 3> R_;
            Eigen::Vector3d e_;
            e_(0) = x.qx()/T(180)*T(M_PI);
            e_(1) = x.qy()/T(180)*T(M_PI);
            e_(2) = x.qz()/T(180)*T(M_PI);
            // e_(0) = x.qx();
            // e_(1) = x.qy();
            // e_(2) = x.qz();
            Eigen::Matrix3d R_;
            get_dcm_from_euler(R_, e_);



            T ga_x = R_(0, 0) * ax + R_(0, 1) * ay + R_(0, 2) * az;
            T ga_y = R_(1, 0) * ax + R_(1, 1) * ay + R_(1, 2) * az;
            T ga_z = R_(2, 0) * ax + R_(2, 1) * ay + R_(2, 2) * az + T(ONE_G);
            ax_out = ga_x;
            ay_out = ga_y;
            az_out = ga_z;
            // ax_out = const_cast<T*>(&ga_x);
            // ay_out = const_cast<T*>(&ga_y);
            // az_out = const_cast<T*>(&ga_z);
            // x_.ax() = ga_x;
            // x_.ay() = ga_y;
            // x_.az() = ga_z;
            // std::cout << "ba "<< x.bax() <<","<< x.bay() <<","<< x.baz() << std::endl;
            // std::cout << "q  "<< a <<","<< b <<","<< c <<","<<d<< std::endl;
            // std::cout << "ga "<< ga_x <<","<< ga_y <<","<< ga_z << std::endl;
            // std::cout << "a  "<<ax <<","<< ay<<"," << az << std::endl;

            T vx_new = x.vx() + ga_x * u.dt();
            T vy_new = x.vy() + ga_y * u.dt();
            T vz_new = x.vz() + ga_z * u.dt();

            T wx_new = u.wx() - x.bwx();
            T wy_new = u.wy() - x.bwy();
            T wz_new = u.wz() - x.bwz();
            // T wx_new = u.wx();
            // T wy_new = u.wy();
            // T wz_new = u.wz();

            wx_out = wx_new;
            wy_out = wy_new;
            wz_out = wz_new;

            // Kalman::SquareMatrix<T, 3> R_new;
            Eigen::Matrix3d R_new;
            // Eigen::Matrix3d _I;
            // _I.Identity();
            Eigen::Matrix3d Omega_cha;
            Omega_cha(0,0) = 1;
            Omega_cha(1,1) = 1;
            Omega_cha(2,2) = 1;
            Omega_cha(0,1) = -wz_new * u.dt();
            Omega_cha(0,2) = wy_new * u.dt();
            Omega_cha(1,0) = wz_new * u.dt();
            Omega_cha(1,2) = -wx_new * u.dt();
            Omega_cha(2,0) = -wy_new * u.dt();
            Omega_cha(2,1) = wx_new * u.dt();
            
            R_new = R_ * Omega_cha;
            // std::cout << Omega_cha << std::endl;
            // std::cout << R_new << std::endl;
            // R_new(0,0) = R_(0,0);
            // R_new(1,1) = R_(1,1);
            // R_new(2,2) = R_(2,2);

            // R_new(0,1) = R_(0,1) - u.wz() * u.dt();
            // R_new(0,2) = R_(0,2) + u.wy() * u.dt();
            // R_new(1,0) = R_(1,0) + u.wz() * u.dt();
            // R_new(1,2) = R_(1,2) - u.wx() * u.dt();
            // R_new(2,0) = R_(2,0) - u.wy() * u.dt();
            // R_new(2,1) = R_(2,1) + u.wx() * u.dt();

            Kalman::Vector<T, 3> axis_temp;
            axis_temp(0) = std::sqrt(R_new(0,0)*R_new(0,0) + R_new(1,0)*R_new(1,0) + R_new(2,0)*R_new(2,0));
            axis_temp(1) = std::sqrt(R_new(0,1)*R_new(0,1) + R_new(1,1)*R_new(1,1) + R_new(2,1)*R_new(2,1));
            axis_temp(2) = std::sqrt(R_new(0,2)*R_new(0,2) + R_new(1,2)*R_new(1,2) + R_new(2,2)*R_new(2,2));

            for (int i = 0; i < 3 ; i++) {
                for (int j = 0; j < 3; j++) {
                    R_new(i,j) = R_new(i,j)/axis_temp(j);
                }
            }

            Eigen::Vector3d new_e;
            
            get_euler_from_R(new_e, R_new);

            // Kalman::Vector<T, 4> q_new;
            // T _t = R_new.trace();
            // if (_t > T(0.0f) ) {
            //     _t = std::sqrt(T(1) +_t);
            //     q_new(0) = T(0.5) *_t;
            //     _t = T(0.5) /_t;
            //     q_new(1) = (R_new(2,1) - R_new(1,2)) *_t;
            //     q_new(2) = (R_new(0,2) - R_new(2,0)) *_t;
            //     q_new(3) = (R_new(1,0) - R_new(0,1)) *_t;
            // } else if (R_new(0,0) > R_new(1,1) && R_new(0,0) > R_new(2,2)) {
            //     _t = std::sqrt(T(1) + R_new(0,0) - R_new(1,1) - R_new(2,2));
            //     q_new(1) = T(0.5) *_t;
            //     _t = T(0.5) /_t;
            //     q_new(0) = (R_new(2,1) - R_new(1,2)) *_t;
            //     q_new(2) = (R_new(1,0) + R_new(0,1)) *_t;
            //     q_new(3) = (R_new(0,2) + R_new(2,0)) *_t;
            // } else if (R_new(1,1) > R_new(2,2)) {
            //     _t = std::sqrt(T(1) - R_new(0,0) + R_new(1,1) - R_new(2,2));
            //     q_new(2) = T(0.5) *_t;
            //     _t = T(0.5) /_t;
            //     q_new(0) = (R_new(0,2) - R_new(2,0)) *_t;
            //     q_new(1) = (R_new(1,0) + R_new(0,1)) *_t;
            //     q_new(3) = (R_new(2,1) + R_new(1,2)) *_t;
            // } else {
            //     _t = std::sqrt(T(1) - R_new(0,0) - R_new(1,1) + R_new(2,2));
            //     q_new(3) = T(0.5) *_t;
            //     _t = T(0.5) /_t;
            //     q_new(0) = (R_new(1,0) - R_new(0,1)) *_t;
            //     q_new(1) = (R_new(0,2) + R_new(2,0)) *_t;
            //     q_new(2) = (R_new(2,1) + R_new(1,2)) *_t;
            // }

            // T bax_new = x.bax();
            // T bay_new = x.bay();
            // T baz_new = x.baz();
            // T bwx_new = x.bwx();
            // T bwy_new = x.bwy();
            // T bwz_new = x.bwz();

            // T q_new_length = std::sqrt(q_new(0)*q_new(0) + q_new(1)*q_new(1) + q_new(2)*q_new(2) + q_new(3)*q_new(3));
            // q_new(0) = q_new(0)/q_new_length;
            // q_new(1) = q_new(1)/q_new_length;
            // q_new(2) = q_new(2)/q_new_length;
            // q_new(3) = q_new(3)/q_new_length;


            x_.x() = x_new;
            x_.y() = y_new;
            x_.z() = z_new;
            x_.vx() = vx_new;
            x_.vy() = vy_new;
            x_.vz() = vz_new;
            // x_.qw() = x.qw();//q_new(0);
            // x_.qx() = new_e(0);
            // x_.qy() = new_e(1);
            // x_.qz() = new_e(2);
            x_.qx() = new_e(0)/T(M_PI)*T(180);//q_new(1);
            x_.qy() = new_e(1)/T(M_PI)*T(180);//q_new(1);
            x_.qz() = new_e(2)/T(M_PI)*T(180);//q_new(1);
            // x_.wx() = u.wx();
            // x_.wy() = u.wy();
            // x_.wz() = u.wz();
            // x_.qy() = q_new(2);
            // x_.qz() = q_new(3);
            x_.bax() = x.bax() + x.nax()*u.dt();
            x_.bay() = x.bay() + x.nay()*u.dt();
            x_.baz() = x.baz() + x.naz()*u.dt();
            x_.bwx() = x.bwx() + x.nwx()*u.dt();
            x_.bwy() = x.bwy() + x.nwy()*u.dt();
            x_.bwz() = x.bwz() + x.nwz()*u.dt();

            x_.nax() = x.nax();
            x_.nay() = x.nay();
            x_.naz() = x.naz();
            x_.nwx() = x.nwx();
            x_.nwy() = x.nwy();
            x_.nwz() = x.nwz();

            return x_;
        }
};

}

#endif