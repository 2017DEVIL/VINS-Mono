#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

//每个相机都有一个FeatureTracker实例，即trackerData[i]
FeatureTracker trackerData[NUM_OF_CAM];

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;//上一帧相机的时间戳
bool init_pub = 0;

/**
 * @brief   ROS的回调函数，对新来的图像进行特征点追踪，发布
 * @Description readImage()函数对新来的图像使用光流法进行特征点跟踪
 *              追踪的特征点封装成feature_points发布到pub_img的话题下，
 *              图像封装成ptr发布在pub_match下
 * @param[in]   img_msg 输入的图像
 * @return      void
*/
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    //单 判断当前帧是否是第一帧图像
    if(first_image_flag)
    {
        // 更新：不再是第一帧图像
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();//记录第一个图像帧的时间
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }

    // 单 通过时间间隔判断相机数据流是否稳定，有问题则restart
    // last_image_time ,上一帧相机的时间戳
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag); // 复位
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();

    // frequency control
    // 单 发布频率控制
    // 并不是每读入一帧图像，就要发布特征点
    // 判断间隔时间内的发布次数
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        // 时间间隔内的发布频率十分接近设定频率时，更新时间间隔起始时刻，并将数据发布次数置0。类似于重新初始化判定系统。以防累积的数值运算误差
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    //单 将图像格式从8UC1转换成mono8,并通过toCvCopy函数，将ROS图像转换成CV图像

    //OpenCV中，图像是以Mat矩阵的形式存储的，与ROS定义的图像消息的格式不同，需要利用cv_bridge进行转换
    cv_bridge::CvImageConstPtr ptr;

    // 将图像编码8UC1转换为mono8,单色8bit
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;//图像数据的大小端存储模式
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        //cv_bridge将ROS的图像消息转换为OpenCV图像格式时都是通过CvImage类实现的。
        //cv_bridge提供了两种方式转换为CvImage类，分别为复制（copy）和共享（share）
        //   CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
        //                       const std::string& encoding = std::string());
        //   CvImagePtr toCvCopy(const sensor_msgs::Image& source,
        //                       const std::string& encoding = std::string());

        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        if (i != 1 || !STEREO_TRACK)//单目
            //单 readImage()函数读取图像数据进行处理
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec());
        else//双目，只进行直方图均衡化处理
            //单问
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }
    //更新新加入的特征点的全局ID
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

    //1、将特征点id，矫正后归一化平面的3D点(x,y,z=1)，像素2D点(u,v)，像素的速度(vx,vy)，封装成sensor_msgs::PointCloudPtr类型的feature_points实例中,用pub_img发布;
    //2、将图像封装到cv_bridge::cvtColor类型的ptr实例中发布到pub_match

    ////没看
   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p); //特征点的归一化坐标
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i); //特征点的ID
                    u_of_point.values.push_back(cur_pts[j].x);//特征点的像素坐标
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);//特征点的速度
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        if (!init_pub)//第一帧不发布
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);//把特征点发布给后端

        if (SHOW_TRACK)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    //draw speed line
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            //cv::imshow("vis", stereo_img);
            //cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "feature_tracker"); //ros初始化，不是特别明白，有时间再搞
    ros::NodeHandle n("~"); //命名空间为/node_namespace/node_name
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//设置消息日志级别

    ///ros初始化是什么东西？初始化了得到了什么参数


    //读配置文件中的image_topic等参数
    readParameters(n);


    // 读取每个相机实例对应的相机内参，NUM_OF_CAM 经常为1，单目
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    //判断是否加入鱼眼mask来去除边缘噪声
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }
    /*
    //创建话题的subscriber
    ros::Subscriber	subscribe(const	string	&topic,	uint32_t	queue_size,	void(*)(M));
    //第一个参数是订阅话题的名称
    //第二个参数是订阅队列的长度,如果受到的消息都没来得及处理,那么新消息入队旧消息就会出队
    //第三个参数是回调函数指针,指向回调函数来处理接收到的消息
 */

    //订阅话题IMAGE_TOPIC(/cam0/image_raw),执行回调函数img_callback
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    //发布feature，实例feature_points，跟踪的特征点，给后端优化用
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    //发布feature_img，实例ptr，跟踪的特征点图，给RVIZ用和调试用
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    // 通过时间间隔判断相机数据流是否稳定，有问题则restart
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?