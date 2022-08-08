#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float32.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>


#include <fstream>

// #define PUB_SURROUND_PTS 1

#define TIME_SYNCHRONIZATION 0
#define WRITE_CSV 1

// int TIME_SYNCHRONIZATION = 0;

Eigen::Matrix<double,4,4> odom_to_matrix(const nav_msgs::Odometry &odom_msg)
{
    Eigen::Matrix<double,4,4> SE3 = Eigen::Matrix<double,4,4>::Zero();

    // quat -> rot matrix 
    tf::Quaternion q1(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    );
    tf::Matrix3x3 m1(q1);

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            SE3(i,j) = m1[i][j];
        }
    }

    // add translation
    SE3(0,3) = odom_msg.pose.pose.position.x;
    SE3(1,3) = odom_msg.pose.pose.position.y;
    SE3(2,3) = odom_msg.pose.pose.position.z;
    SE3(3,3) = 1.0;

    return SE3;
}

std::vector<double> quat_to_rpy(geometry_msgs::Quaternion quat)
{
    tf::Quaternion q1(
        quat.x,
        quat.y,
        quat.z,
        quat.w
    );

    tf::Matrix3x3 m1(q1);

    double roll,pitch,yaw;
    m1.getRPY(roll,pitch,yaw);

    std::vector<double> rpy = {roll,pitch,yaw};

    return rpy;
}



class SlamEvaluator{
    public:
        SlamEvaluator(int time_sync);

        ros::NodeHandle nh;

        ros::Subscriber gazebo_sub, odom_sub;

        ros::Publisher rmse_pub;

        nav_msgs::Odometry odom_truth;
        nav_msgs::Odometry odom_est;

        void upd_odom_truth(const nav_msgs::OdometryConstPtr &msg);
        void upd_odom_est(const nav_msgs::OdometryConstPtr &msg);

        void odom_callback(const nav_msgs::OdometryConstPtr &odom_fake, const nav_msgs::OdometryConstPtr &odom_real);

        void rmse();

        int time_synchronization = 0;


        std::vector<nav_msgs::Odometry> odom_truth_hist;
        std::vector<nav_msgs::Odometry> odom_est_hist;

        std::fstream fs;


};

SlamEvaluator::SlamEvaluator(int time_sync)
{


    // time_synchronization = TIME_SYNCHRONIZATION;



    if(time_sync)
    {
        message_filters::Subscriber<nav_msgs::Odometry> odom_truth_sub(nh,"/unitree_odom_fake",1);
        message_filters::Subscriber<nav_msgs::Odometry> odom_est_sub(nh,"/unitree_odom",1);

        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(3000),odom_truth_sub,odom_est_sub);
        sync.registerCallback(boost::bind(&SlamEvaluator::odom_callback,this,_1,_2));

    }
    else
    {
        gazebo_sub = nh.subscribe<nav_msgs::Odometry>("/unitree_odom_fake",10000,&SlamEvaluator::upd_odom_truth,this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("/unitree_odom",10000,&SlamEvaluator::upd_odom_est,this);
    }


    // message_filters::TimeSynchronizer<nav_msgs::Odometry,nav_msgs::Odometry> sync(odom_truth_sub,odom_est_sub,30000);
    // sync.registerCallback(boost::bind(&SlamEvaluator::odom_callback,this,_1,_2));

    rmse_pub = nh.advertise<std_msgs::Float32>("/rmse",10000);


    std::string CSV_PATH =  "slam_evaluation_kalmann.csv";
    fs.open(CSV_PATH, std::fstream::in | std::fstream::out);
    
    fs << "time_gt" << ", "
        << "x_gt" << ", "
        << "y_gt" << ", "
        << "z_gt" << ", "
        << "roll_gt" << ", "
        << "pitch_gt" << ", "
        << "yaw_gt" << ", "
        
        << "time_est" << ", "
        << "x_est" << ", "
        << "y_est" << ", "
        << "z_est" << ", "
        << "roll_est" << ", "
        << "pitch_est" << ", "
        << "yaw_est" << "\n";
    
    

    


}


void SlamEvaluator::odom_callback(const nav_msgs::OdometryConstPtr &_odom_truth, const nav_msgs::OdometryConstPtr &_odom_est)
{

    std::cout << "odom_callback" << std::endl;
    odom_truth = *_odom_truth;
    odom_est = *_odom_est;    

    odom_truth_hist.push_back(odom_truth);
    odom_est_hist.push_back(odom_est);

}



void SlamEvaluator::upd_odom_truth(const nav_msgs::OdometryConstPtr &msg)
{
    // std::cout << "upd1" << std::endl;
    odom_truth = *msg;
}

void SlamEvaluator::upd_odom_est(const nav_msgs::OdometryConstPtr &msg)
{
    // std::cout << "upd2" << std::endl;
    odom_est = *msg;

    odom_truth_hist.push_back(odom_truth);
    odom_est_hist.push_back(odom_est);


    if(WRITE_CSV)
    {
        std::vector<double> rpy_truth = quat_to_rpy(odom_truth.pose.pose.orientation);
        std::vector<double> rpy_est = quat_to_rpy(odom_est.pose.pose.orientation);


        
        fs << odom_truth.header.stamp << ", "
            << odom_truth.pose.pose.position.x << ", "
            << odom_truth.pose.pose.position.y << ", "
            << odom_truth.pose.pose.position.z << ", "
            << rpy_truth[0] << ", "
            << rpy_truth[1] << ", "
            << rpy_truth[2] << ", "
            
            << odom_est.header.stamp << ", "
            << odom_est.pose.pose.position.x << ", "
            << odom_est.pose.pose.position.y << ", "
            << odom_est.pose.pose.position.z << ", "
            << rpy_est[0] << ", "
            << rpy_est[1] << ", "
            << rpy_est[2] << "\n";


    }

}





void SlamEvaluator::rmse()
{

    // biggest interval will be half of the vector size
    int n_intervals = odom_truth_hist.size()/2;
    

    float rmse = 0.0;


    if(odom_truth_hist.size()>10 && odom_est_hist.size()>10)
    {
        // go over all deltas
        for(int delta=1;delta<n_intervals;delta++)
        {

            float rpe_interval = 0.0;
            int interval_length = 0; 


            // go over whole odometry history
            for(int i=0;i<(odom_truth_hist.size()-delta);i+=delta)
            {

                
                // get Q_i,Q_{i+delta}   |   P_i, P_{i+delta},  Q - ground truth, P - estimated
                nav_msgs::Odometry tf_truth_i = odom_truth_hist[i];
                nav_msgs::Odometry tf_truth_ii = odom_truth_hist[i+delta];

                nav_msgs::Odometry tf_est_i = odom_est_hist[i];
                nav_msgs::Odometry tf_est_ii = odom_est_hist[i+delta];

                // std::cout << "tf_truth_i: " << tf_truth_i.pose.pose.position.x << std::endl;
                // std::cout << "tf_truth_i: " << tf_truth_i.pose.pose.position.y << std::endl;
                // std::cout << "tf_truth_i: " << tf_truth_i.pose.pose.position.z << std::endl;


                // std::cout << "tf_est_i: " << tf_est_i.pose.pose.position.x << std::endl;
                // std::cout << "tf_est_i: " << tf_est_i.pose.pose.position.y << std::endl;
                // std::cout << "tf_est_i: " << tf_est_i.pose.pose.position.z << std::endl;




                // convert everything into homogeneous transformation matrices
                Eigen::Matrix<double,4,4> Q_i = odom_to_matrix(tf_truth_i);
                Eigen::Matrix<double,4,4> Q_ii = odom_to_matrix(tf_truth_ii);

                Eigen::Matrix<double,4,4> P_i = odom_to_matrix(tf_est_i);
                Eigen::Matrix<double,4,4> P_ii = odom_to_matrix(tf_est_ii);

                // std::cout << "Q_i: " << Q_i << std::endl;
                // std::cout << "Q_ii: " << Q_ii << std::endl;
                // std::cout << "P_i: " << P_i << std::endl;
                // std::cout << "P_ii: " << P_ii << std::endl;
                
                // std::cout << "Q_i.inverse() * Q_ii: " << Q_i.inverse() * Q_ii << std::endl;
                // std::cout << "P_i.inverse() * P_ii" << P_i.inverse() * P_ii << std::endl;

                // pose error:
                Eigen::Matrix<double,4,4> RPE = (Q_i.inverse() * Q_ii).inverse() * (P_i.inverse() * P_ii);

                // std::cout << "i: " << i << std::endl;
                // std::cout << "RPE: " << RPE << std::endl;

                rpe_interval += pow(RPE(0,3),2) + pow(RPE(1,3),2) + pow(RPE(2,3),2);
                interval_length++;


            }

            rpe_interval /= interval_length;

            rpe_interval = sqrt(rpe_interval);

            // std::cout << "rpe_interval: " << rpe_interval << std::endl;

            rmse += rpe_interval;

        }

        rmse /= n_intervals;


        std::cout << "rmse: " << rmse << std::endl;

        std_msgs::Float32 rmse_msg;
        rmse_msg.data = rmse;

        rmse_pub.publish(rmse_msg);

    }

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"slam_evaluator");

    SlamEvaluator slam_evaluator(TIME_SYNCHRONIZATION);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        slam_evaluator.rmse();
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}