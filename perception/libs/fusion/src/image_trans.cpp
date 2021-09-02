#include "fusions/image_trans.h"

inline double Image_trans::fx() { return P_(0, 0); }
inline double Image_trans::fy() { return P_(1, 1); }
inline double Image_trans::cx() { return P_(0, 2); }
inline double Image_trans::cy() { return P_(1, 2); }
inline double Image_trans::Tx() { return P_(0, 3); }
inline double Image_trans::Ty() { return P_(1, 3); }

cv::Mat_<double> Image_trans::get_D() { return D_; }
cv::Matx33d Image_trans::get_R() { return R_; }
cv::Matx33d Image_trans::get_K() { return K_; }
cv::Matx34d Image_trans::get_P() { return P_; }
cv::Matx44d Image_trans::get_E() { return E_; }
cv::Size Image_trans::get_dst_size() { return dst_size_; }

Image_trans::Image_trans(int width, int height, cv::Mat K, cv::Mat D, cv::Mat E)
{
    cv::Size dst_size(width, height); // 从左上角按照dst_size大小切
    dst_size_ = dst_size;

    cv::Mat R_temp, P_temp;
    R_temp = cv::Mat_<double>::eye(3, 3);
    P_temp = cv::Mat_<double>::zeros(3, 4);

    K_ = K.clone();
    D_ = D.clone();
    R_ = R_temp.clone();
    P_ = P_temp.clone();
    E_ = E.clone();

    cv::Mat cameraExtrinsicMat(E_);
    invRt = cameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
    cv::Mat invT = -invRt.t() * (cameraExtrinsicMat(cv::Rect(3, 0, 1, 3)));
    invTt = invT.t();

    cache_ = true;
}

Image_trans::~Image_trans(){};

bool Image_trans::projectLidar3dToPixel(const pcl::PointXYZ cloud_point)
{
    assert(initialized());

    cv::Mat point(1, 3, CV_64F);
    double fp[3];
    fp[0] = cloud_point.x;
    fp[1] = cloud_point.y;
    fp[2] = cloud_point.z;
    if (fp[0] <= 0)
    {
        return false;
    }
    for (int i = 0; i < 3; i++)
    {
        point.at<double>(i) = invTt.at<double>(i);
        for (int j = 0; j < 3; j++)
        {
            point.at<double>(i) += double(fp[j]) * invRt.at<double>(j, i);
        }
    }
    if (point.at<double>(2) <= 1)
    {
        return false;
    }
    if (std::isnan(point.at<double>(0)) || std::isnan(point.at<double>(1)) || std::isnan(point.at<double>(2)))
        return false;
    double tmpx = point.at<double>(0) / point.at<double>(2);
    double tmpy = point.at<double>(1) / point.at<double>(2);

    cv::Point2d imagepoint;
    imagepoint.x = tmpx;
    imagepoint.y = tmpy;
    cv::Mat cameraMat(K_);
    imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
    imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);
    int px = int(imagepoint.x + 0.5);
    int py = int(imagepoint.y + 0.5);

    if (0 <= px && px < dst_size_.width && 0 <= py && py < dst_size_.height)
    {
        Point.x = px;
        Point.y = py;
        Point.z = point.at<double>(2);
        return true;
    }
    return false;
}

bool Image_trans::projectLidar3dToRoiPixel(const pcl::PointXYZ cloud_point, const cv::Rect bbox)
{
    assert(initialized());

    cv::Mat point(1, 3, CV_64F);
    double fp[3];
    fp[0] = cloud_point.x;
    fp[1] = cloud_point.y;
    fp[2] = cloud_point.z;
    if (fp[0] <= 0)
    {
        return false;
    }
    for (int i = 0; i < 3; i++)
    {
        point.at<double>(i) = invTt.at<double>(i);
        for (int j = 0; j < 3; j++)
        {
            point.at<double>(i) += double(fp[j]) * invRt.at<double>(j, i);
        }
    }
    if (point.at<double>(2) <= 1)
    {
        return false;
    }
    if (std::isnan(point.at<double>(0)) || std::isnan(point.at<double>(1)) || std::isnan(point.at<double>(2)))
        return false;
    double tmpx = point.at<double>(0) / point.at<double>(2);
    double tmpy = point.at<double>(1) / point.at<double>(2);

    cv::Point2d imagepoint;
    imagepoint.x = tmpx;
    imagepoint.y = tmpy;
    cv::Mat cameraMat(K_);
    imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
    imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);
    int px = int(imagepoint.x + 0.5);
    int py = int(imagepoint.y + 0.5);

    if (bbox.x <= px && px < bbox.x + bbox.width && bbox.y <= py && py < bbox.y + bbox.height)
    {
        Point.x = px;
        Point.y = py;
        Point.z = point.at<double>(2);
        return true;
    }
    return false;
}

cv::Point3d Image_trans::get_Point()
{
    return Point;
}

cv::Point2d Image_trans::project3dToPixel(const cv::Point3d &xyz)
{
    assert(initialized());
    assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane
    cv::Point2d uv_rect;
    uv_rect.x = (fx() * xyz.x + Tx()) / xyz.z + cx();
    uv_rect.y = (fy() * xyz.y + Ty()) / xyz.z + cy();
    return uv_rect;
}

cv::Point3d Image_trans::projectPixelTo3dRay(const cv::Point2d &uv_rect)
{
    assert(initialized());

    cv::Point3d ray;
    ray.x = (uv_rect.x - cx() - Tx()) / fx();
    ray.y = (uv_rect.y - cy() - Ty()) / fy();
    ray.z = 1.0;
    return ray;
}

void Image_trans::rectifyImage(const sensor_msgs::Image &msg, cv::Mat &raw, cv::Mat &undistort)
{
    assert(initialized());

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    raw = cv_ptr->image;

    cv::Mat R_ = cv::Mat_<double>::eye(3, 3);
    cv::Matx33d P_binned;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P_binned(i, j) = P_(i, j);
        }
    }

    cv::Size binned_resolution = dst_size_;
    cv::Mat full_map1, full_map2;
    cv::initUndistortRectifyMap(K_, D_, R_, P_binned, binned_resolution,
                                CV_16SC2, full_map1, full_map2);
    cv::remap(raw, undistort, full_map1, full_map2, cv::INTER_LINEAR);

    return;
}

cv::Point2d Image_trans::rectifyPoint(const cv::Point2d &uv_raw)
{
    assert(initialized());

    /// @todo cv::undistortPoints requires the point data to be float, should allow double
    cv::Point2f raw32 = uv_raw, rect32;
    const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
    cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
    cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
    return rect32;
}

cv::Point2d Image_trans::unrectifyPoint(const cv::Point2d &uv_rect)
{
    assert(initialized());
    // Convert to a ray
    cv::Point3d ray = projectPixelTo3dRay(uv_rect);

    // Project the ray on the image
    cv::Mat R_ = cv::Mat_<double>::eye(3, 3);
    cv::Mat r_vec, t_vec = cv::Mat_<double>::zeros(3, 1);
    cv::Rodrigues(R_.t(), r_vec);
    std::vector<cv::Point2d> image_point;
    cv::projectPoints(std::vector<cv::Point3d>(1, ray), r_vec, t_vec, K_, D_, image_point);

    return image_point[0];
}

cv::Rect Image_trans::rectifyRoi(const cv::Rect &roi_raw)
{
    assert(initialized());

    /// @todo Actually implement "best fit" as described by REP 104.

    // For now, just unrectify the four corners and take the bounding box.
    cv::Point2d rect_tl = rectifyPoint(cv::Point2d(roi_raw.x, roi_raw.y));
    cv::Point2d rect_tr = rectifyPoint(cv::Point2d(roi_raw.x + roi_raw.width, roi_raw.y));
    cv::Point2d rect_br = rectifyPoint(cv::Point2d(roi_raw.x + roi_raw.width,
                                                   roi_raw.y + roi_raw.height));
    cv::Point2d rect_bl = rectifyPoint(cv::Point2d(roi_raw.x, roi_raw.y + roi_raw.height));

    cv::Point roi_tl(std::ceil(std::min(rect_tl.x, rect_bl.x)),
                     std::ceil(std::min(rect_tl.y, rect_tr.y)));
    cv::Point roi_br(std::floor(std::max(rect_tr.x, rect_br.x)),
                     std::floor(std::max(rect_bl.y, rect_br.y)));

    return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}

cv::Rect Image_trans::unrectifyRoi(const cv::Rect &roi_rect)
{
    assert(initialized());

    /// @todo Actually implement "best fit" as described by REP 104.

    // For now, just unrectify the four corners and take the bounding box.
    cv::Point2d raw_tl = unrectifyPoint(cv::Point2d(roi_rect.x, roi_rect.y));
    cv::Point2d raw_tr = unrectifyPoint(cv::Point2d(roi_rect.x + roi_rect.width, roi_rect.y));
    cv::Point2d raw_br = unrectifyPoint(cv::Point2d(roi_rect.x + roi_rect.width,
                                                    roi_rect.y + roi_rect.height));
    cv::Point2d raw_bl = unrectifyPoint(cv::Point2d(roi_rect.x, roi_rect.y + roi_rect.height));

    cv::Point roi_tl(std::floor(std::min(raw_tl.x, raw_bl.x)),
                     std::floor(std::min(raw_tl.y, raw_tr.y)));
    cv::Point roi_br(std::ceil(std::max(raw_tr.x, raw_br.x)),
                     std::ceil(std::max(raw_bl.y, raw_br.y)));

    return cv::Rect(roi_tl.x, roi_tl.y, roi_br.x - roi_tl.x, roi_br.y - roi_tl.y);
}