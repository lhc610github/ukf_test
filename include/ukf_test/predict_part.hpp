#ifndef PREDICT_PART_HPP_
#define PREDICT_PART_HPP_

#include "ukf_test/update_part.hpp"
#include "sensor_msgs/Imu.h"
#include <deque>

typedef float T;
typedef Test1::State<T> S_;

typedef Test1::Control<T> C_;
typedef Test1::SystemModel<T> SM_;

typedef Test1::VioMeasurement<T> VMeas_;
typedef Test1::VioMeasurementModel<T> VM_;

void * start_predict_loop_thread(void *args);
// #include <kalman/UnscentedKalmanFilter.hpp>

// template<typename T, class State, class Control, class SystemModel, class VioMeasurement, class VioModel>
class Filter_predict_part : public Filter_update_part<T, S_, C_, SM_, VMeas_, VM_>{
    public:
        Filter_predict_part(ros::NodeHandle & temp_nh):
        _first_predict(true),
        imu_sequ_index(0),
        loop_num(0),
        nh_predict_(temp_nh) {

            pthread_mutex_init(&imu_data_mutex, NULL);

            this->set_predict_valid(false);
            std::string imu_topic_name;
            int _ctrl_bandwidth, _imu_delay;
            nh_predict_.param<std::string>("imu_topic", imu_topic_name, "/imu");
            nh_predict_.param<std::string>("frame_id", frameId, std::string("imu"));
            nh_predict_.param<int>("imu_rate", imu_rate, 1000);
            nh_predict_.param<int>("predict_rate", predict_rate, 100);
            nh_predict_.param<int>("imu_sequence_num", imu_sequ_num, 800);
            nh_predict_.param<int>("ctrl_bandwidth", _ctrl_bandwidth, 30);
            nh_predict_.param<int>("imu_delay", _imu_delay, 300);
            imu_sub = nh_predict_.subscribe(imu_topic_name, 2, &Filter_predict_part::imu_update_cb, this);

            imu_delay = (int)((float)_imu_delay*(float)imu_rate/1000.0f);
            // _delta_t_max = 1.0f/float(predict_rate);
            // std::cout << "max dt = [" << _delta_t_max << "]" << std::endl;
            // if (_delta_t_max < 0.005f) {
            //     ROS_WARN("predict rate is too high limited to 200Hz");
            // } 

            imu_cal_num = (int)(imu_rate/_ctrl_bandwidth);
            if (imu_cal_num < 10|| imu_cal_num > 100) {
                imu_cal_num = 20;
                ROS_WARN("wrong bandwidth correct to 20");
            }

            ROS_INFO("imu cal num : %d", imu_cal_num);

            if (imu_delay < 0 || imu_delay > imu_sequ_num - imu_cal_num) {
                imu_delay = (imu_sequ_num - imu_cal_num)/2;
                ROS_WARN("wrong imu_delay correct to %d", imu_delay);
            }

            ROS_INFO("imu delay num : %d", imu_delay);


            int _result = pthread_create( &predict_tid, NULL, &start_predict_loop_thread, this);
            if ( _result ) throw _result;
        }

        ~Filter_predict_part() {
            pthread_join(predict_tid, NULL);
        }

        typedef struct imu_data_t {
            ros::Time header;
            double ax;
            double ay;
            double az;
            double wx;
            double wy;
            double wz;
        } imu_data_s;

        typedef struct imu_mean_data_t {
            ros::Time header;
            double ax;
            double ay;
            double az;
            double wx;
            double wy;
            double wz;
            double noise_ax;
            double noise_ay;
            double noise_az;
            double noise_wx;
            double noise_wy;
            double noise_wz;
        } imu_mean_data_s;

        void imu_update_cb(const sensor_msgs::Imu& msg) {
            imu_data_s _imu_data_temp;
            _imu_data_temp.header = msg.header.stamp;
            // _imu_data_temp.header = ros::Time::now();
            _imu_data_temp.ax = msg.linear_acceleration.x;
            _imu_data_temp.ay = msg.linear_acceleration.y;
            _imu_data_temp.az = msg.linear_acceleration.z;
            _imu_data_temp.wx = msg.angular_velocity.x;
            _imu_data_temp.wy = msg.angular_velocity.y;
            _imu_data_temp.wz = msg.angular_velocity.z;
            if (_imu_data_temp.header.toSec() > 100.0f) {
                pthread_mutex_lock(&imu_data_mutex);
                imu_sequ.push_back(_imu_data_temp);

                if (imu_sequ.size() > imu_sequ_num) {
                    do {imu_sequ.pop_front(); }
                    while (imu_sequ.size() > imu_sequ_num);
                    imu_sequ_index = imu_sequ.size();
                } else {
                    imu_sequ_index = imu_sequ.size();
                }
                // std::cout << imu_sequ_index << std::endl;
                loop_num ++;
                if (loop_num >= 10000) {
                    loop_num = 0;
                }
                pthread_mutex_unlock(&imu_data_mutex);
            }
        }
        
        void clear_imu_sequence() {
            imu_sequ.clear();
            imu_sequ_index = 0;
        }

        imu_mean_data_s cal_mean_imu_data() {
            imu_mean_data_s temp_data;
            if (imu_sequ_index < imu_cal_num + imu_delay) {
                temp_data.header = imu_sequ[imu_sequ_index-1].header;
                temp_data.ax = imu_sequ[imu_sequ_index-1].ax;
                temp_data.ay = imu_sequ[imu_sequ_index-1].ay;
                temp_data.az = imu_sequ[imu_sequ_index-1].az;
                temp_data.wx = imu_sequ[imu_sequ_index-1].wx;
                temp_data.wy = imu_sequ[imu_sequ_index-1].wy;
                temp_data.wz = imu_sequ[imu_sequ_index-1].wz;
                temp_data.noise_ax = 0.0f;
                temp_data.noise_ay = 0.0f;
                temp_data.noise_az = 0.0f;
                temp_data.noise_wx = 0.0f;
                temp_data.noise_wy = 0.0f;
                temp_data.noise_wz = 0.0f;
            } else {
                float sum_ax = 0;
                float sum_ay = 0;
                float sum_az = 0;
                float sum_wx = 0;
                float sum_wy = 0;
                float sum_wz = 0;
                for (int i = 1; i <= imu_cal_num; i++) {
                    sum_ax += imu_sequ[imu_sequ_index - imu_delay - i].ax;
                    sum_ay += imu_sequ[imu_sequ_index - imu_delay - i].ay;
                    sum_az += imu_sequ[imu_sequ_index - imu_delay - i].az;
                    sum_wx += imu_sequ[imu_sequ_index - imu_delay - i].wx;
                    sum_wy += imu_sequ[imu_sequ_index - imu_delay - i].wy;
                    sum_wz += imu_sequ[imu_sequ_index - imu_delay - i].wz;
                }
                float m_ax = sum_ax/(float)imu_cal_num;
                float m_ay = sum_ay/(float)imu_cal_num;
                float m_az = sum_az/(float)imu_cal_num;
                float m_wx = sum_wx/(float)imu_cal_num;
                float m_wy = sum_wy/(float)imu_cal_num;
                float m_wz = sum_wz/(float)imu_cal_num;


                temp_data.header = imu_sequ[imu_sequ_index-imu_delay-1].header;
                temp_data.ax = m_ax;
                temp_data.ay = m_ay;
                temp_data.az = m_az;
                temp_data.wx = m_wx;
                temp_data.wy = m_wy;
                temp_data.wz = m_wz;

            }
            return temp_data;
        }

        void start_predict_loop() {
            ros::Rate predict_loop_rate(predict_rate);
            while (ros::ok()) {
                bool _is_predict_this_time = false;
                if (imu_sequ.size() >  imu_cal_num+ imu_delay) {
                    pthread_mutex_lock(&imu_data_mutex);
                    if (_first_predict) {
                        ROS_INFO("first predict   %f", imu_sequ[imu_sequ_index - imu_delay -1].header.toSec());
                        // ROS_INFO("sequ size   %d", imu_sequ.size());
                        // ROS_INFO("imu_sequ size : %d", imu_sequ.size());
                        // for (int i = 0; i < imu_sequ.size() ; i++) {
                        //     ROS_INFO("[%d] time: %.6f",i,imu_sequ[i].header.toSec());
                        // }
                        _last_timestamp = imu_sequ[imu_sequ_index - imu_delay -1].header;
                        _first_predict = false;
                    } else {
                        float dt = (float)(imu_sequ[imu_sequ_index - imu_delay -1].header - _last_timestamp).toSec();
                        // if (dt < 1.0f && dt > 0.0008f) {
                        if (dt < 1.0f && loop_num != last_loop_num){
                            // printf("%.4f\n",dt);
                        // if (_temp_rate > (float)predict_rate - 20.0f && _temp_rate < (float)predict_rate+20.0f) {
                            _last_timestamp = imu_sequ[imu_sequ_index-imu_delay-1].header;
                            last_loop_num = loop_num;
                            if (this->get_update_init_state()) {
                                imu_mean_data_s _temp_mean_imu_data = cal_mean_imu_data();
                                C_ _C;
                                _C.ax() = _temp_mean_imu_data.ax;
                                _C.ay() = _temp_mean_imu_data.ay;
                                _C.az() = _temp_mean_imu_data.az;
                                _C.wx() = _temp_mean_imu_data.wx;
                                _C.wy() = _temp_mean_imu_data.wy;
                                _C.wz() = _temp_mean_imu_data.wz;
                                // _C.dt() = 1.0f/(float)predict_rate;
                                float temp_dt = 1.0f/(float)predict_rate;
                                if (dt > temp_dt/2.0f && dt < 2.0f * temp_dt) {
                                    _C.dt() = dt;
                                } else {
                                    _C.dt() = temp_dt;
                                }
                                this->predict_process(_C, imu_sequ[imu_sequ_index-imu_delay-1].header);
                                this->set_predict_valid(true);
                                this->set_ready_to_update(true);
                                _is_predict_this_time = true;
                            }
                        } else if (dt >= 1.0f) {
                            _first_predict = true;
                            ROS_INFO("time out !,   %f", dt);
                            this->reinit();
                            clear_imu_sequence();
                        }
                    }
                    pthread_mutex_unlock(&imu_data_mutex);
                }
                if (_is_predict_this_time)
                    predict_loop_rate.sleep();
                else
                    usleep(100);
            }
        }


    private:
        ros::NodeHandle nh_predict_;
        ros::Subscriber imu_sub;
        bool _first_predict;
        ros::Time _last_timestamp;
        std::string frameId;
        // float _delta_t_max;

        pthread_t predict_tid;
        pthread_mutex_t imu_data_mutex;

        std::deque<imu_data_s> imu_sequ;
        
        int imu_rate;
        int predict_rate;
        int imu_sequ_num;
        int imu_sequ_index;
        int imu_cal_num;
        int loop_num;
        int last_loop_num;
        int imu_delay;
};

void *
start_predict_loop_thread(void *args) {
    Filter_predict_part *filter_temp = (Filter_predict_part *)args;
    filter_temp->start_predict_loop();
    return NULL;
}

#endif