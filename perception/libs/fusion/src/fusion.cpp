#include "fusions/fusion.h"

fusion::fusion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat image, Image_trans *trans,
               std::vector<cv::Rect> bboxes) : cloud_(cloud), image_(image), bboxes_(bboxes), trans_(trans)
{
}

fusion::~fusion() {}

void fusion::process()
{
    for (int i = 0; i < bboxes_.size(); i++)
    {
        GroupType group;
        UVPointCloud group_cloud;
        cv::Rect bbox = bboxes_[i];
        for (int j = 0; j < cloud_->points.size(); j++)
        {
            cv::Point3d pixel;
            bool in = trans_->projectLidar3dToRoiPixel(cloud_->points[j], bbox);
            if (in)
            {
                pixel = trans_->get_Point();
                UVPoint full_point;
                full_point.x = cloud_->points[j].x;
                full_point.y = cloud_->points[j].y;
                full_point.z = cloud_->points[j].z;
                full_point.u = pixel.x;
                full_point.v = pixel.y;
                full_point.d = pixel.z;
                group_cloud.points.push_back(full_point);
            }
        }
        group.cloud = group_cloud;
        groups.push_back(group);
    }
    for (int i = 0; i < groups.size(); i++)
    {
        std::vector<float> distance_candidates;
        for (int j = 0; j < groups[i].cloud.points.size(); j++)
        {
            float distance_candidate = groups[i].cloud.points[j].d;
            distance_candidates.push_back(distance_candidate);
        }
        float g_distance = getMode(distance_candidates);
        groups[i].distance = g_distance;
    }
}

float fusion::getShortest(const std::vector<float> &candidates)
{
    if (candidates.empty())
        return 0.0f;

    return *std::min_element(candidates.begin(), candidates.end());
}

float fusion::getMedian(const std::vector<float> &candidates)
{
    size_t num_candidates = candidates.size();

    if (num_candidates == 0)
    {
        return 0.0f;
    }
    else if (num_candidates == 1)
    {
        return candidates.at(0);
    }

    /* Sort candidate by its distances */
    std::vector<float> sorted_candidates(candidates);
    std::sort(sorted_candidates.begin(), sorted_candidates.end());

    float median;
    if (num_candidates % 2 == 0)
    {
        median = (sorted_candidates.at(num_candidates / 2 - 1) + sorted_candidates.at(num_candidates / 2)) / 2;
    }
    else
    {
        median = sorted_candidates.at(num_candidates / 2);
    }
    return median;
}

float fusion::getMode(const std::vector<float> &candidates)
{
    if (candidates.empty())
        return 0.0f;

    /*
    Express histogram by std::map.
    key (first) of map = bin (category) of histgram (bin means integer distance of candidates here)
    value (second) of map = candidate's real value which belong to that category
  */
    std::map<int, std::vector<float>> histogram;

    /* Categorize each candidates into histogram */
    for (const auto &sample : candidates)
    {
        int bin = static_cast<float>(sample);
        histogram[bin].push_back(sample);
    }

    /* Search the most common bin */
    size_t max_num_candidates = 0;
    int max_bin = 0;
    for (const auto &bin : histogram)
    {
        if (max_num_candidates < bin.second.size())
        {
            max_bin = bin.first;
            max_num_candidates = bin.second.size();
        }
    }

    /* Calculate median value of candidates which belong to most common bin */
    return getMedian(histogram[max_bin]);
}

std::vector<cv::Point3d> fusion::get_PointsImage()
{
    std::vector<cv::Point3d> pixels;
    for (int j = 0; j < cloud_->points.size(); j++)
    {
        cv::Point3d pixel;
        bool in = trans_->projectLidar3dToPixel(cloud_->points[j]);
        if (in)
        {
            pixel = trans_->get_Point();
            pixels.push_back(pixel);
        }
    }
    return pixels;
}

std::vector<GroupType> fusion::get_Groups()
{
    return groups;
}

std::vector<float> fusion::get_Distances()
{
    std::vector<float> distances;
    for (int i = 0; i < groups.size(); i++)
    {
        float distance = groups[i].distance;
        distances.push_back(distance);
    }
    return distances;
}