#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <string>

class ImuFilter
{
public:
    ImuFilter(ros::NodeHandle& nh)
    {
        nh.param<std::string>("filter_type", filter_type_, "lpf1");  // lpf1 或 iir2
        nh.param<double>("cutoff_freq", cutoff_freq_, 5.0);          // Hz
        nh.param<double>("sampling_freq", sampling_freq_, 200.0);    // Hz

        imu_sub_ = nh.subscribe("/mavros/imu/data_raw", 10, &ImuFilter::imuCallback, this);
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_filter/data", 10);

        dt_ = 1.0 / sampling_freq_;
        init_ = false;
        ROS_INFO("IMU Filter Node Started. Type: %s, fc=%.2f Hz, fs=%.2f Hz", 
                 filter_type_.c_str(), cutoff_freq_, sampling_freq_);
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        Eigen::Vector3d acc(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);

        Eigen::Vector3d gyr(msg->angular_velocity.x,
                            msg->angular_velocity.y,
                            msg->angular_velocity.z);

        if (!init_)
        {
            acc_prev1_ = acc_prev2_ = acc;
            gyr_prev1_ = gyr_prev2_ = gyr;
            acc_filt_prev1_ = acc_filt_prev2_ = acc;
            gyr_filt_prev1_ = gyr_filt_prev2_ = gyr;
            computeCoeffs();
            init_ = true;
            return;
        }

        Eigen::Vector3d acc_filt, gyr_filt;

        if (filter_type_ == "lpf1")
        {
            acc_filt = a1_ * acc + (1 - a1_) * acc_filt_prev1_;
            gyr_filt = a1_ * gyr + (1 - a1_) * gyr_filt_prev1_;
        }
        else if (filter_type_ == "iir2")
        {
            // 二阶IIR（双二阶Butterworth低通滤波器）
            acc_filt = b0_ * acc + b1_ * acc_prev1_ + b2_ * acc_prev2_
                       - a1_iir_ * acc_filt_prev1_ - a2_iir_ * acc_filt_prev2_;
            gyr_filt = b0_ * gyr + b1_ * gyr_prev1_ + b2_ * gyr_prev2_
                       - a1_iir_ * gyr_filt_prev1_ - a2_iir_ * gyr_filt_prev2_;
        }
        else
        {
            ROS_WARN_THROTTLE(2.0, "Unknown filter type: %s", filter_type_.c_str());
            acc_filt = acc;
            gyr_filt = gyr;
        }

        // ��新状态
        acc_prev2_ = acc_prev1_;
        acc_prev1_ = acc;
        acc_filt_prev2_ = acc_filt_prev1_;
        acc_filt_prev1_ = acc_filt;

        gyr_prev2_ = gyr_prev1_;
        gyr_prev1_ = gyr;
        gyr_filt_prev2_ = gyr_filt_prev1_;
        gyr_filt_prev1_ = gyr_filt;

        // 发布滤波后的IMU数据
        sensor_msgs::Imu out = *msg;
        out.linear_acceleration.x = acc_filt.x();
        out.linear_acceleration.y = acc_filt.y();
        out.linear_acceleration.z = acc_filt.z();

        out.angular_velocity.x = gyr_filt.x();
        out.angular_velocity.y = gyr_filt.y();
        out.angular_velocity.z = gyr_filt.z();

        imu_pub_.publish(out);
    }

    void computeCoeffs()
    {
        double wc = 2.0 * M_PI * cutoff_freq_;
        double T = dt_;
        double b = wc * T;
        a1_ = b / (1.0 + b); // 一阶低通

        if (filter_type_ == "iir2")
        {
            // 二阶 Butterworth 数字滤波器双线性变换系数
            double ita = 1.0 / tan(M_PI * cutoff_freq_ / sampling_freq_);
            double q = sqrt(2.0);
            b0_ = 1.0 / (1.0 + q * ita + ita * ita);
            b1_ = 2.0 * b0_;
            b2_ = b0_;
            a1_iir_ = 2.0 * (1.0 - ita * ita) * b0_;
            a2_iir_ = (1.0 - q * ita + ita * ita) * b0_;
        }

        ROS_INFO("Filter coefficients computed. a1=%.3f (LPF), b0=%.3f, a1_iir=%.3f", 
                 a1_, b0_, a1_iir_);
    }

    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;

    std::string filter_type_;
    double cutoff_freq_;
    double sampling_freq_;
    double dt_;
    bool init_;

    // 一阶低通
    double a1_;

    // 二阶IIR
    double b0_, b1_, b2_, a1_iir_, a2_iir_;

    Eigen::Vector3d acc_prev1_, acc_prev2_;
    Eigen::Vector3d gyr_prev1_, gyr_prev2_;
    Eigen::Vector3d acc_filt_prev1_, acc_filt_prev2_;
    Eigen::Vector3d gyr_filt_prev1_, gyr_filt_prev2_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_filter_node");
    ros::NodeHandle nh("~");
    ImuFilter filter(nh);
    ros::spin();
    return 0;
}

