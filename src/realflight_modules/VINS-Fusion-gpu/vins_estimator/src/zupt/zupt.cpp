#include "zupt.h"

bool Zupt::zuptDetection(double t, const Eigen::Vector3d* const vel_ptr,
                                   const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>* const features_frame_ptr)
{
    bool is_static = true;
    bool is_imu_static = true;
    bool is_parallax_static = true;

    double parallax = -1;
    if (features_frame_ptr != nullptr)
        parallax = inputFeatureAndCheckParallax(t, *features_frame_ptr);

    int cnt = 0;
    for (int i = 0; i < parallax_buf_.size(); i++) {
        if (parallax_buf_[i].second > parallax_static_thr_)
            cnt++;
    }
    double parallax_unstatic_cnt_ratio = (double)cnt / (double)parallax_buf_.size();
    if ( parallax_unstatic_cnt_ratio > unstatic_ratio_thr_ )
        is_parallax_static = false;


    mBuf.lock();
    while (acc_buf_.size() > queue_size_) {
        acc_buf_.pop_front();
        gyr_buf_.pop_front();
        acc_filt_buf_.pop_front();
        gyr_filt_buf_.pop_front();
    }
    mBuf.unlock();

    Eigen::Vector3d high_freq_acc;
    Eigen::Vector3d high_freq_gyr;

    mBuf.lock();
    cnt = 0;
    int total_cnt = 0;
    for (int i = 0; i < acc_buf_.size(); i++) {
        if (acc_buf_[i].first > featureBuf_.back().first) break;

        high_freq_acc = acc_buf_[i].second - acc_filt_buf_[i].second; 
        high_freq_gyr = gyr_buf_[i].second - gyr_filt_buf_[i].second; 

        if (high_freq_acc.norm() > acc_amp_static_thr_ ||
            high_freq_gyr.norm() > gyr_amp_static_thr_ ||
            acc_filt_buf_[i].second.norm() > acc_avg_static_thr_ ||
            gyr_filt_buf_[i].second.norm() > gyr_avg_static_thr_) {
            cnt++;
        }
        total_cnt++;
    }

    double imu_unstatic_cnt_ratio = (double)cnt / (double)total_cnt;
    if ( imu_unstatic_cnt_ratio > unstatic_ratio_thr_ )
        is_imu_static = false;
    else
        // consider vel
        if ( vel_ptr != nullptr && vel_ptr->norm() > vel_static_thr_ )
            is_imu_static = false;
        // don't consider vel
        else
            is_imu_static = true;

    // is_static = is_imu_static || is_parallax_static;
    is_static = is_imu_static;

    mBuf.unlock();


    Eigen::Vector3d acc_raw, gyr_raw;
    Eigen::Quaterniond q_GI;
    getIMURawAndQuaternion(t, acc_raw_buf_, gyr_raw_buf_, quaternion_GI_buf_,
                           &acc_raw, &gyr_raw, &q_GI);

    result_buf_.push_back( ZuptResultInfo(t, is_static, acc_raw, gyr_raw, q_GI) );
    while (result_buf_.size() > feature_frame_size_) {
        result_buf_.pop_front();
    }

    if (ENABLE_ZUPT_DEBUG_LOG) { // DEBUG_ZUPT

    static bool is_first = true;
    mBuf.lock();
    if (is_first) {
        FILE* f = fopen("/root/Fast-Drone-XI35/vins_output/zuptlog.csv", "w");
        fprintf(f, "t,is_static,is_imu_static,is_parallax_static,imu_unstatic_cnt_ratio,parallax_unstatic_cnt_ratio,acc_x,acc_y,acc_z,acc_norm,acc_filt_norm,acc_high_norm,gyr_norm,gyr_filt_norm,gyr_high_norm,parallax,accbuf_size,featurebuf_size,total_cnt,resultbuf_size\n");
        fclose(f);
        is_first = false;
    }
    else {
        FILE* f = fopen("/root/Fast-Drone-XI35/vins_output/zuptlog.csv", "a");
        fprintf(f, "%f,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%ld,%ld,%d,%ld\n", 
                    t, 
                    is_static, 
                    is_imu_static, 
                    is_parallax_static, 
                    imu_unstatic_cnt_ratio,
                    parallax_unstatic_cnt_ratio,
                    acc_buf_.back().second(0), 
                    acc_buf_.back().second(1), 
                    acc_buf_.back().second(2), 
                    acc_buf_.back().second.norm(), 
                    acc_filt_buf_.back().second.norm(),
                    (acc_buf_.back().second - acc_filt_buf_.back().second).norm(),
                    gyr_buf_.back().second.norm(),
                    gyr_filt_buf_.back().second.norm(),
                    (gyr_buf_.back().second - gyr_filt_buf_.back().second).norm(),
                    parallax,
                    acc_buf_.size(),
                    featureBuf_.size(),
                    total_cnt,
                    result_buf_.size());
        fclose(f);
    }
    mBuf.unlock();

    } // DEBUG_ZUPT


    return is_static;
}


void Zupt::getIMURawAndQuaternion(double t,
                                  std::deque<std::pair<double, Eigen::Vector3d>>& acc_raw_buf,
                                  std::deque<std::pair<double, Eigen::Vector3d>>& gyr_raw_buf,
                                  std::deque<std::pair<double, Eigen::Quaterniond>>& q_GI_buf,
                                  Eigen::Vector3d* acc_raw_ptr,
                                  Eigen::Vector3d* gyr_raw_ptr,
                                  Eigen::Quaterniond* q_GI_ptr)
{
    int i, j;
    for (i = 0; i < acc_raw_buf.size(); i++) {
        if (acc_raw_buf[i].first > t) break;
    }
    i -= 1;

    double a;
    double t0, t1;
    if (i < acc_raw_buf.size() - 1) {
        j = i + 1;
        t0 = acc_raw_buf[i].first;
        t1 = acc_raw_buf[j].first;
        a = (t - t0) / (t1 - t0);
    }
    else {
        j = i;
        a = 0;
    }

    *acc_raw_ptr = acc_raw_buf[i].second + a * ( acc_raw_buf[j].second - acc_raw_buf[i].second );
    *gyr_raw_ptr = gyr_raw_buf[i].second + a * ( gyr_raw_buf[j].second - gyr_raw_buf[i].second );
    // *q_GI_ptr = q_GI_buf[i] * Exp(a * Log(q_GI_buf[i].inverse() * q_GI_buf[j]));
    *q_GI_ptr = q_GI_buf[i].second;

    while (acc_raw_buf.size() > raw_meas_queue_size_) {
        acc_raw_buf.pop_front();
        gyr_raw_buf.pop_front();
        q_GI_buf.pop_front();
    }
}


bool Zupt::getResultInfo(double t, ZuptResultInfo* info)
{
    if (result_buf_.size() == 0)
        return false;

    // processMeasurement too slow, lost some static results cuz results queue over size.
    if (t < (result_buf_.front().t_ - eps_))
        return false;

    while (t > (result_buf_.front().t_ + eps_)) {
        result_buf_.pop_front();
        if (result_buf_.size() == 0)
            return false;
    }

    if (abs(t - result_buf_.front().t_) <= eps_) {
        *info = result_buf_.front();
        result_buf_.pop_front();
        return true;
    }
}


void Zupt::inputIMU(double t, const Eigen::Vector3d& linearAcceleration, 
                              const Eigen::Vector3d& angularVelocity,
                              const Eigen::Quaterniond& last_Q)
{
    Eigen::Vector3d acc_filt, gyr_filt;
    Eigen::Vector3d linearAcceleration_minus_g = linearAcceleration + 
                                 last_Q.toRotationMatrix().inverse() * Eigen::Vector3d(0, 0, -9.8066);
    lpf_.process(linearAcceleration_minus_g, angularVelocity, &acc_filt, &gyr_filt);
  
    mBuf.lock();

    acc_raw_buf_.push_back( std::make_pair(t, linearAcceleration) );
    gyr_raw_buf_.push_back( std::make_pair(t, angularVelocity) );
    quaternion_GI_buf_.push_back( std::make_pair(t, last_Q) );

    acc_buf_.push_back( std::make_pair(t, linearAcceleration_minus_g) );
    gyr_buf_.push_back( std::make_pair(t, angularVelocity) );
    acc_filt_buf_.push_back( std::make_pair(t, acc_filt) );
    gyr_filt_buf_.push_back( std::make_pair(t, gyr_filt) );

    mBuf.unlock();
}

double Zupt::inputFeatureAndCheckParallax(double t, 
        const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1> > > >& features_frame)
{
    featureBuf_.push_back(std::make_pair(t, features_frame));

    if (featureBuf_.size() < 2) return 1.0;

    const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1> > > >& fs1 = featureBuf_[featureBuf_.size()-2].second;
    const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1> > > >& fs2 = featureBuf_[featureBuf_.size()-1].second;

    int parallax_cnt = 0;
    double parallax_sum = 0;
    for (auto& feat_in_f2: fs2) {
        int feat_in_f2_id = feat_in_f2.first;
        auto it = std::find_if( fs1.begin(), fs1.end(), 
                                [feat_in_f2_id](const auto& feat_in_f1){ 
                                    return feat_in_f1.first == feat_in_f2_id; }
                              );
        if ( it != fs1.end() ) {
            auto& feat_in_f1 = *it;
            double z1 = feat_in_f1.second[0].second(2);
            double z2 = feat_in_f2.second[0].second(2);

            double u1 = feat_in_f1.second[0].second(0) / z1;
            double v1 = feat_in_f1.second[0].second(1) / z1;
            double u2 = feat_in_f2.second[0].second(0) / z2;
            double v2 = feat_in_f2.second[0].second(1) / z2;

            double du = u1 - u2;
            double dv = v1 - v2;

            double para = du * du + dv * dv;
            parallax_sum += para;
            parallax_cnt++;
        }
    }

    while (featureBuf_.size() > feature_frame_size_) {
        featureBuf_.pop_front();
    }

    while (parallax_buf_.size() > feature_frame_size_) {
        parallax_buf_.pop_front();
    }

    if (parallax_cnt == 0) {
        parallax_buf_.push_back(std::make_pair(t, 1.0));
        return 1.0;
    }
    else {
        double avg_parallax = parallax_sum / parallax_cnt;
        parallax_buf_.push_back(std::make_pair(t, avg_parallax));
        return avg_parallax;
    }


}

 
// -----------------Test Steps--------------------------
// Uncomment #define ZUPT_TEST.
//
// mkdir build
// cd build
// g++ -o zupt -I../ -I/usr/include/eigen3/ ../zupt.cpp && ./zupt
// rosrun plotjuggler plotjuggler
//
// Open result.csv in plotjuggler.
// -----------------------------------------------------

// #define ZUPT_TEST
#ifdef ZUPT_TEST

#include <cmath>
typedef std::pair<double, Eigen::Vector3d> imu_data_t;
typedef std::vector<imu_data_t> imu_data_batch_t;
typedef std::pair<double, Eigen::Quaterniond> qua_data_t;
typedef std::vector<std::pair<double, Eigen::Quaterniond>> qua_data_batch_t;

Eigen::Vector3d sin(double w, double t) { return Eigen::Vector3d( sin(w*t), sin(w*t), sin(w*t) ); }
Eigen::Vector3d cos(double w, double t) { return Eigen::Vector3d( cos(w*t), cos(w*t), cos(w*t) ); }

static void zupt_test()
{
    // generate simulate data
    constexpr double imu_freq = 250.0;
    constexpr double img_freq = 30.0;
    constexpr double move_freq = 1.0;
    constexpr double ba = 0.51;
    constexpr double bg = 0.03;
    constexpr double vel_init_scale = -0.0;
    constexpr double T_sec = 20.0;
    constexpr double static_T_sec = 1.0;
    double t = 0.0;

    Eigen::Vector3d gw(0, 0, -9.8066);
    double delta_w = 1e-3;
    Eigen::Vector3d u(1, 1, 1);
    u /= u.norm();

    int imu_data_size = imu_freq * T_sec;
    double freq_ratio = img_freq / imu_freq;
    Eigen::Vector3d acc_bias(ba, ba, ba);
    Eigen::Vector3d gyr_bias(bg, bg, bg);   
    Eigen::Vector3d vel(vel_init_scale, vel_init_scale, vel_init_scale);   

    imu_data_batch_t acc_sim_data;
    imu_data_batch_t gyr_sim_data;
    imu_data_batch_t vel_sim_data;
    qua_data_batch_t qua_sim_data;

    acc_sim_data.reserve(imu_data_size);
    gyr_sim_data.reserve(imu_data_size);
    vel_sim_data.reserve(imu_data_size);
    qua_sim_data.reserve(imu_data_size);

    double cur_t;
    for (int i = 0, j = 0; i < imu_data_size; i++) {
        cur_t = (double)i / imu_freq;

        Eigen::AngleAxisd ag = Eigen::AngleAxisd(j*delta_w, u);
        Eigen::Quaterniond qi(ag);
        Eigen::Matrix3d Ri = qi.toRotationMatrix();
        qua_sim_data[i] = qua_data_t(cur_t, qi);

        if ( (int)(cur_t/static_T_sec) % 2 == 0 ) {
            acc_sim_data[i] = imu_data_t(cur_t, Ri.inverse()*(-gw) + acc_bias);
            gyr_sim_data[i] = imu_data_t(cur_t, gyr_bias);
        }
        else {
            acc_sim_data[i] = imu_data_t(cur_t, acc_bias + Ri.inverse()*(sin(2.0*M_PI*move_freq, cur_t) - gw));
            gyr_sim_data[i] = imu_data_t(cur_t, gyr_bias + cos(2.0*M_PI*move_freq, cur_t));
            j++;
        }
    }

    for (int i = 0; i < imu_data_size; i++) {
        cur_t = (double)i / imu_freq;
        vel_sim_data[i] = imu_data_t(cur_t, vel);
        vel += acc_sim_data[i].second / imu_freq;
    }


    // simulate
    FILE* f = fopen("result.csv", "w");
    fprintf(f, "# timestamp[s], acc_x[m/s^2], acc_y[m/s^2], acc_z[m/s^2], gyr_x[rad/s^2], gyr_y[rad/s^2], gyr_z[rad/s^2], vel_norm[m/s], get_img[], is_static[]\n");
    Zupt zupt_obj;

    int cnt = 0;
    bool get_img = false;
    bool is_static = true;
    for (int i = 0; i < imu_data_size; i++) {
        cur_t = (double)i / imu_freq;

        zupt_obj.inputIMU(acc_sim_data[i].first, 
                          acc_sim_data[i].second, 
                          gyr_sim_data[i].second,
                          qua_sim_data[i].second);

        if (cnt == 0 || (double)cnt / i < freq_ratio) {
            // is_static = zupt_obj.zuptDetection(cur_t, &vel_sim_data[i].second, nullptr);
            is_static = zupt_obj.zuptDetection(cur_t, nullptr, nullptr);

            get_img = true;
            std::cout << "@";

            cnt++;
        }
        else {
            get_img = false;
            std::cout << "#";
        }

        fprintf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n", cur_t,
                                                acc_sim_data[i].second(0),
                                                acc_sim_data[i].second(1),
                                                acc_sim_data[i].second(2),
                                                gyr_sim_data[i].second(0),
                                                gyr_sim_data[i].second(1),
                                                gyr_sim_data[i].second(2),
                                                vel_sim_data[i].second.norm(),
                                                get_img,
                                                is_static
                                                );
    }
    std::cout << std::endl;

    fclose(f);
}

// for test
int main(int argc, char** argv)
{
    zupt_test();
    return 0;
}

#endif // ZUPT_TEST

