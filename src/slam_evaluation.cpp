#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float32.h>


#include <cmath>


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



class SlamEvaluator{
    public:
        SlamEvaluator();

        ros::NodeHandle nh;

        ros::Subscriber gazebo_sub, odom_sub;

        ros::Publisher rmse_pub;

        nav_msgs::Odometry odom_truth;
        nav_msgs::Odometry odom_est;

        void upd_odom_truth(const nav_msgs::OdometryConstPtr &msg);
        void upd_odom_est(const nav_msgs::OdometryConstPtr &msg);

        void rmse();

        std::vector<nav_msgs::Odometry> odom_truth_hist;
        std::vector<nav_msgs::Odometry> odom_est_hist;

};
    



SlamEvaluator::SlamEvaluator()
{
    gazebo_sub = nh.subscribe<nav_msgs::Odometry>("/unitree_odom",10000,&SlamEvaluator::upd_odom_truth,this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/loam_odom",10000,&SlamEvaluator::upd_odom_est,this);

    rmse_pub = nh.advertise<std_msgs::Float32>("rmse",10000);

}



void SlamEvaluator::upd_odom_truth(const nav_msgs::OdometryConstPtr &msg)
{
    odom_truth = *msg;
}

void SlamEvaluator::upd_odom_est(const nav_msgs::OdometryConstPtr &msg)
{
    odom_est = *msg;

    odom_truth_hist.push_back(odom_truth);
    odom_est_hist.push_back(odom_est);
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

    SlamEvaluator slam_evaluator;

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        slam_evaluator.rmse();
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}