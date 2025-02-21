#include "feature_tracker.h"



//FeatureTracker的static成员变量n_id初始化为0
int FeatureTracker::n_id = 0;

//判断跟踪的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;

    // COL:图像的宽，ROW：图像的高

    int img_x = cvRound(pt.x);

    //cvRound()：返回跟参数最接近的整数值，即四舍五入；
    // cvFloor()：返回不大于参数的最大整数值，即向下取整；
    // cvCeil()：返回不小于参数的最小整数值，即向上取整

    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

//去除无法跟踪的特征点
//去除无法跟踪的特征点,status为0的点直接跳过，否则v[j++]=v[i]留下来，最后v.resize(j)根据最新的j安排内存
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);//resize()，设置大小（size）; reserve()，设置容量（capacity）;
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//空的构造函数
FeatureTracker::FeatureTracker() //这个函数的作用是什么？为什么是空的
{
}


/**
 * @brief   对跟踪点进行排序并去除密集点
 * @Description 对跟踪到的特征点，按照被追踪到的次数排序并依次选点
 *              使用mask进行类似非极大抑制，半径为30，去掉密集点，使特征点分布均匀
 * @return      void
*/

void FeatureTracker::setMask()
{
    // 如果是鱼眼镜头直接clone即可，否则创建空白板
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255)); //创建一个mat，每个元素都是255白
    

    // prefer to keep features that are tracked for long time(倾向于留下被追踪时间很长的特征点)
    // 构造(cnt，pts，id)序列，（追踪次数，当前特征点坐标，id）
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    // 为(cnt，pts，id)序列赋值
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    //对光流跟踪到的特征点forw_pts，按照被跟踪到的次数cnt从大到小排序（lambda表达式）
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    //清空cnt，pts，id并重新存入
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();


    // auto可以在声明变量的时候根据变量初始值的类型自动为此变量选择匹配的类型
    for (auto &it : cnt_pts_id)
    {
        //        原始图像上进行mask
        if (mask.at<uchar>(it.second.first) == 255)
        {
            //当前特征点位置对应的mask值为255，则保留当前特征点，将对应的特征点位置pts，id，被追踪次数cnt分别存入
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            //在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}


//添将新检测到的特征点n_pts
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);//新提取的特征点id初始化为-1
        track_cnt.push_back(1);//新提取的特征点被跟踪的次数初始化为1
    }
}

/**
 * @brief   对图像使用光流法进行特征点跟踪
 * @Description createCLAHE() 对图像进行自适应直方图均衡化
 *              calcOpticalFlowPyrLK() LK金字塔光流法
 *              setMask() 对跟踪点进行排序，设置mask
 *              rejectWithF() 通过基本矩阵剔除outliers
 *              goodFeaturesToTrack() 添加特征点(shi-tomasi角点)，确保每帧都有足够的特征点
 *              addPoints()添加新的追踪点
 *              undistortedPoints() 对角点图像坐标去畸变矫正，并计算每个角点的速度
 * @param[in]   _img 输入图像
 * @param[in]   _cur_time 当前时间（图像时间戳）
 * @return      pts_velocity，forw_pts（像素坐标系），更新特征id，跟踪次数
*/
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;


    //单 如果EQUALIZE=1，表示太亮或太暗，进行直方图均衡化处理
    if (EQUALIZE)
    {
        //自适应直方图均衡
        //createCLAHE(double clipLimit, Size tileGridSize)
        //tilesGridSize：图像被分成称为“tiles”（瓷砖、地砖、小方地毯、片状材料、块状材料）的小块，在OpenCV中，tilesGridSize默认为8x8 ，即整个图像被划分为8纵8横共64块。然后对每一个块进行直方图均衡处理
        //https://blog.csdn.net/LaoYuanPython/article/details/120850922?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522164291089816780261911100%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=164291089816780261911100&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-7-120850922.first_rank_v2_pc_rank_v29&utm_term=createCLAHE&spm=1018.2226.3001.4187
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        //如果当前帧的图像数据forw_img为空，说明当前是第一次读入图像数据
        //将读入的图像赋给当前帧forw_img，同时还赋给prev_img、cur_img
        //forw_img、cur_img、prev_img分别表示当前帧、上一帧、上上一帧
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        //否则，说明之前就已经有图像读入，只需要更新当前帧forw_img的数据
        forw_img = img;
    }

    //此时forw_pts还保存的是上一帧图像中的特征点，所以把它清除
    forw_pts.clear();

    //    如果上一帧有特征点
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        //调用cv::calcOpticalFlowPyrLK()对前一帧的特征点cur_pts进行LK金字塔光流跟踪，得到forw_pts
        //status标记了从前一帧cur_img到forw_img特征点的跟踪状态，无法被追踪到的点标记为0
////        光流跟踪用的竟然是未去畸变的像素点
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        //将位于图像边界外的点标记为0
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        //根据status,把跟踪失败的点剔除，还有图像边界的点
        //不仅要从当前帧数据forw_pts中剔除，而且还要从cur_un_pts、prev_pts和cur_pts中剔除
        //prev_pts和cur_pts中的特征点是一一对应的
        //记录特征点id的ids，和记录特征点被跟踪次数的track_cnt也要剔除
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    //光流追踪成功,特征点被成功跟踪的次数就加1
    //数值代表被追踪的次数，数值越大，说明被追踪的就越久
    for (auto &n : track_cnt)
        n++;

    //PUB_THIS_FRAME=1 需要发布特征点
    if (PUB_THIS_FRAME)
    {
        //通过基本矩阵剔除outliers

        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;

        //setMask()保证相邻的特征点之间要相隔30个像素,设置mask
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;

        //计算是否需要提取新的特征点
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            /**
 *void cv::goodFeaturesToTrack(    在mask中不为0的区域检测新的特征点
 *   InputArray  image,              输入图像
 *   OutputArray     corners,        存放检测到的角点的vector
 *   int     maxCorners,             返回的角点的数量的最大值
 *   double  qualityLevel,           角点质量水平的最低阈值（范围为0到1，质量最高角点的水平为1），小于该阈值的角点被拒绝
 *   double  minDistance,            返回角点之间欧式距离的最小值
 *   InputArray  mask = noArray(),   和输入图像具有相同大小，类型必须为CV_8UC1,用来描述图像中感兴趣的区域，只在感兴趣区域中检测角点
 *   int     blockSize = 3,          计算协方差矩阵时的窗口大小
 *   bool    useHarrisDetector = false,  指示是否使用Harris角点检测，如不指定则使用shi-tomasi算法
 *   double  k = 0.04                Harris角点检测需要的k值
 *)
 */
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;

        //添将新检测到的特征点n_pts添加到forw_pts中，id初始化-1,track_cnt初始化为1
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;

    //根据不同的相机模型去畸变矫正和转换到归一化坐标系上，计算速度
    undistortedPoints();
    prev_time = cur_time;
}


/**
 * @brief   通过F矩阵去除outliers
 * @Description 将图像坐标转换为归一化坐标
 *              cv::findFundamentalMat()计算F矩阵
 *              reduceVector()去除outliers
 * @return      void
*/
void FeatureTracker::rejectWithF()
{

    if (forw_pts.size() >= 8)// 当前帧（追踪上）特征点数量足够多
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;

            //根据不同的相机模型将二维坐标转换到三维坐标
            //对于PINHOLE（针孔相机）可将像素坐标转换到归一化平面并去畸变
            //对于CATA（卡特鱼眼相机）将像素坐标投影到单位圆内
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;

        //调用cv::findFundamentalMat对un_cur_pts和un_forw_pts计算F矩阵,并且剔除外点
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

//更新特征点id
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}


//读取相机内参
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


//显示去畸变矫正后的图像  name为图像帧名称
void FeatureTracker::showUndistortion(const string &name)
{
    ////为什么+600
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;

            ////liftProjective是什么意思
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));


        ////???????
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


//得到无畸变的归一化平面坐标，以及他们的速度
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());

    //像素坐标转到归一化平面并去畸变
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        //根据不同的相机模型将二维坐标转换到归一化相机三维坐标系
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)// 2.2 通过id判断不是最新的点
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;// 当前帧-地图点上一帧
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else// 之前没出现过，先放进去但是速度为0
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else//如果是最新的点，则速度为0
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else// 如果地图是空的，速度是0
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
