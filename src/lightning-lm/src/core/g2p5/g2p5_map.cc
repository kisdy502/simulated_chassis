//
// Created by xiang on 25-6-23.
//

#include "core/g2p5/g2p5_map.h"

#include <malloc.h>
#include <cstdlib>
#include <execution>

namespace lightning::g2p5 {

bool G2P5Map::Init(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x, const float &temp_max_y) {
    ReleaseResources();
    min_x_ = temp_min_x;
    min_y_ = temp_min_y;
    max_x_ = temp_max_x;
    max_y_ = temp_max_y;

    grid_size_x_ = ceil((max_x_ - min_x_) / grid_reso_);
    grid_size_y_ = ceil((max_y_ - min_y_) / grid_reso_);

    if (grid_size_x_ <= 0 || grid_size_y_ <= 0) {
        return false;
    }

    max_x_ = min_x_ + grid_size_x_ * grid_reso_;
    max_y_ = min_y_ + grid_size_y_ * grid_reso_;
    grids_ = new SubGrid *[grid_size_x_];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        grids_[xi] = new SubGrid[grid_size_y_];
    }
    return true;
}

std::shared_ptr<G2P5Map> G2P5Map::MakeDeepCopy() {
    std::shared_ptr<G2P5Map> ret(new G2P5Map(options_));
    ret->min_x_ = min_x_;
    ret->min_y_ = min_y_;
    ret->max_x_ = max_x_;
    ret->max_y_ = max_y_;
    ret->grid_size_x_ = grid_size_x_;
    ret->grid_size_y_ = grid_size_y_;

    ret->grids_ = new SubGrid *[grid_size_x_];
    for (int xi = 0; xi < grid_size_x_; ++xi) {
        ret->grids_[xi] = new SubGrid[grid_size_y_];
        for (int yi = 0; yi < grid_size_y_; ++yi) {
            ret->grids_[xi][yi] = this->grids_[xi][yi];
        }
    }

    return ret;
}

bool G2P5Map::Resize(const float &temp_min_x, const float &temp_min_y, const float &temp_max_x,
                     const float &temp_max_y) {
    if (IsEmpty()) {
        const float min_x = std::floor(temp_min_x / grid_reso_) * grid_reso_;
        const float min_y = std::floor(temp_min_y / grid_reso_) * grid_reso_;
        return Init(min_x, min_y, temp_max_x, temp_max_y);
    }

    // Move the origin by whole subgrids, exactly matching the data offset.
    const int left = std::max(0, static_cast<int>(std::ceil((min_x_ - temp_min_x) / grid_reso_)));
    const int bottom = std::max(0, static_cast<int>(std::ceil((min_y_ - temp_min_y) / grid_reso_)));
    const int right = std::max(0, static_cast<int>(std::ceil((temp_max_x - max_x_) / grid_reso_)));
    const int top = std::max(0, static_cast<int>(std::ceil((temp_max_y - max_y_) / grid_reso_)));
    if (left == 0 && bottom == 0 && right == 0 && top == 0) {
        return true;
    }

    const int width = grid_size_x_ + left + right;
    const int height = grid_size_y_ + bottom + top;
    auto **new_grids = new SubGrid *[width];
    for (int x = 0; x < width; ++x) {
        new_grids[x] = new SubGrid[height];
    }
    for (int x = 0; x < grid_size_x_; ++x) {
        for (int y = 0; y < grid_size_y_; ++y) {
            new_grids[x + left][y + bottom] = grids_[x][y];
        }
        delete[] grids_[x];
    }

    delete[] grids_;
    grids_ = new_grids;
    min_x_ -= left * grid_reso_;
    min_y_ -= bottom * grid_reso_;
    grid_size_x_ = width;
    grid_size_y_ = height;
    max_x_ = min_x_ + grid_size_x_ * grid_reso_;
    max_y_ = min_y_ + grid_size_y_ * grid_reso_;

    return true;
}

G2P5Map::~G2P5Map() {
    if (grids_ != nullptr) {
        for (int xi = 0; xi < grid_size_x_; ++xi) {
            delete[] grids_[xi];
        }
        delete[] grids_;
        grids_ = nullptr;
    }
}

void G2P5Map::SetHitPoint(const float &px, const float &py, const bool &if_hit, float height) {
    if (grids_ == nullptr) {
        return;
    }

    if (px < min_x_ || px > max_x_ || py < min_y_ || py > max_y_) {
        return;
    }

    int x_index = floor((px - min_x_) / options_.resolution_);
    int y_index = floor((py - min_y_) / options_.resolution_);

    UpdateCell(Vec2i(x_index, y_index), if_hit, height);
}

void G2P5Map::UpdateCell(const Vec2i &point_index, const bool &if_hit, float height) {
    if (grids_ == nullptr) {
        return;
    }

    int x_index = point_index.x();
    int y_index = point_index.y();
    int xi = (x_index >> SUB_GRID_SIZE);
    int yi = (y_index >> SUB_GRID_SIZE);

    if (xi < 0 || xi > (grid_size_x_ - 1) || yi < 0 || yi > (grid_size_y_ - 1)) {
        return;
    }

    int sub_index_i = x_index - (xi << SUB_GRID_SIZE);
    int sub_index_j = y_index - (yi << SUB_GRID_SIZE);

    if (sub_index_i < 0 || sub_index_i > (sub_grid_width_ - 1) || sub_index_j < 0 ||
        sub_index_j > (sub_grid_width_ - 1)) {
        return;
    }

    grids_[xi][yi].SetGridHitPoint(if_hit, sub_index_i, sub_index_j, height);
}

void G2P5Map::SetMissPoint(const float &point_x, const float &point_y, const float &laser_origin_x,
                           const float &laser_origin_y, float height, float lidar_height) {
    if (grids_ == nullptr) {
        return;
    }

    int point_x_index = floor((point_x - min_x_) / options_.resolution_);
    int point_y_index = floor((point_y - min_y_) / options_.resolution_);

    int xi_lidar, yi_lidar;
    if (!GetDataIndex(laser_origin_x, laser_origin_y, xi_lidar, yi_lidar)) {
        return;
    }

    float k = 0;
    int sign = 1;
    int diff_y = point_y_index - yi_lidar;
    int diff_x = point_x_index - xi_lidar;

    /// 整数的直线填充算法
    if (diff_y == 0 && diff_x == 0) {
        return;
    }

    std::vector<Vec2i> updated_pts;
    std::vector<float> heights;

    if (std::abs(diff_y) > std::abs(diff_x)) {
        if (diff_y == 0) {
            return;
        }

        k = float(diff_x) / diff_y;
        float dh = (lidar_height - height) / diff_y;

        sign = diff_y > 0 ? 1 : -1;
        for (int j = sign; j != diff_y; j += sign) {
            int i = float(j * k);
            int x_index = xi_lidar + i;
            int y_index = yi_lidar + j;

            updated_pts.emplace_back(Vec2i(x_index, y_index));
            heights.emplace_back(lidar_height - j * dh);
        }
    } else {
        if (diff_x == 0) {
            return;
        }

        k = float(diff_y) / diff_x;
        sign = diff_x > 0 ? 1 : -1;

        float dh = (lidar_height - height) / diff_x;

        for (int i = sign; i != diff_x; i += sign) {
            int j = float(i * k);
            int x_index = xi_lidar + i;
            int y_index = yi_lidar + j;

            updated_pts.emplace_back(Vec2i(x_index, y_index));
            heights.emplace_back(lidar_height - i * dh);
        }
    }

    for (int i = 0; i < updated_pts.size(); ++i) {
        UpdateCell(updated_pts[i], false, heights[i]);
    }
}

bool G2P5Map::GetDataIndex(const float x, const float y, int &x_index, int &y_index) {
    if (x > max_x_ || x < min_x_ || y > max_y_ || y < min_y_) {
        return false;
    }
    x_index = floor((x - min_x_) / options_.resolution_);
    y_index = floor((y - min_y_) / options_.resolution_);

    return true;
}

void G2P5Map::ReleaseResources() {
    if (grids_ != nullptr) {
        for (int xi = 0; xi < grid_size_x_; ++xi) {
            delete[] grids_[xi];
        }
        delete[] grids_;
        grids_ = nullptr;
    }
    min_x_ = min_y_ = 10000;
    max_x_ = max_y_ = -10000;

    grid_size_x_ = grid_size_y_ = 0;
    malloc_trim(0);
}

nav_msgs::msg::OccupancyGrid G2P5Map::ToROS() {
    nav_msgs::msg::OccupancyGrid occu_map;
    int image_width = grid_size_x_ * sub_grid_width_;
    int image_height = grid_size_y_ * sub_grid_width_;
    occu_map.info.resolution = static_cast<nav_msgs::msg::MapMetaData::_resolution_type>(options_.resolution_);
    occu_map.info.width = static_cast<nav_msgs::msg::MapMetaData::_width_type>(image_width);
    occu_map.info.height = static_cast<nav_msgs::msg::MapMetaData::_width_type>(image_height);
    occu_map.info.origin.position.x = min_x_;
    occu_map.info.origin.position.y = min_y_;

    int grid_map_size = occu_map.info.width * occu_map.info.height;
    occu_map.data.resize(grid_map_size);

    std::fill(occu_map.data.begin(), occu_map.data.end(), -1);
    int tmp_area = 0;

    for (int bxi = 0; bxi < grid_size_x_; ++bxi) {
        for (int byi = 0; byi < grid_size_y_; ++byi) {
            if (grids_[bxi][byi].IsEmpty()) {
                continue;
            }

            for (int sxi = 0; sxi < sub_grid_width_; ++sxi) {
                for (int syi = 0; syi < sub_grid_width_; ++syi) {
                    int x = (bxi << SUB_GRID_SIZE) + sxi;
                    int y = (byi << SUB_GRID_SIZE) + syi;

                    if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
                        unsigned int hit_cnt = 0, visit_cnt = 0;
                        grids_[bxi][byi].GetHitAndVisit(sxi, syi, hit_cnt, visit_cnt);

                        float occ = (visit_cnt > 3) ? (float)hit_cnt / (float)visit_cnt : -1;

                        /// 注意这里有转置符号
                        if (occ < 0) {
                            continue;
                        } else if (occ > options_.occupancy_ratio_) {
                            tmp_area++;
                            occu_map.data[MapIdx(image_width, x, y)] = 100;
                        } else {
                            // Expand free space within this row only.
                            const int first_x = std::max(0, x - 1);
                            const int last_x = std::min(image_width - 1, x + 1);
                            for (int neighbor = first_x; neighbor <= last_x; ++neighbor) {
                                const int extend = MapIdx(image_width, neighbor, y);
                                if (occu_map.data[extend] < 0) {  //-1
                                    tmp_area++;
                                    occu_map.data[extend] = 0;
                                }
                            }
                        }
                    } else {
                        continue;
                    }
                }
            }
        }
    }
    return occu_map;
}

cv::Mat G2P5Map::ToCV() {
    int image_width = grid_size_x_ * sub_grid_width_;
    int image_height = grid_size_y_ * sub_grid_width_;

    // 定义要映射的颜色值
    cv::Vec3b black_color = cv::Vec3b(0, 0, 0);
    cv::Vec3b white_color = cv::Vec3b(255, 255, 255);
    cv::Vec3b other_color = cv::Vec3b(127, 127, 127);

    cv::Mat image(image_height, image_width, CV_8UC3, other_color);

    int image_height_1 = image_height - 1;
    int image_width_1 = image_width - 1;
    int index_y_min, index_y_max, index_x_min, index_x_max;

    for (int bxi = 0; bxi < grid_size_x_; ++bxi) {
        for (int byi = 0; byi < grid_size_y_; ++byi) {
            if (grids_[bxi][byi].IsEmpty()) {
                continue;
            }

            for (int sxi = 0; sxi < sub_grid_width_; ++sxi) {
                for (int syi = 0; syi < sub_grid_width_; ++syi) {
                    int x = (bxi << SUB_GRID_SIZE) + sxi;
                    int y = (byi << SUB_GRID_SIZE) + syi;

                    if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
                        unsigned int hit_cnt = 0, visit_cnt = 0;
                        grids_[bxi][byi].GetHitAndVisit(sxi, syi, hit_cnt, visit_cnt);

                        float occ = visit_cnt ? (hit_cnt == 0 ? 0 : (float)hit_cnt / (float)visit_cnt) : -1;
                        // assert(occ <= 1.0);

                        /// 注意这里有转置符号
                        if (occ < 0) {
                            continue;
                        } else if (occ > options_.occupancy_ratio_) {  // 0.49
                            image.at<cv::Vec3b>(y, x) = black_color;
                        } else {
                            index_y_min = std::max(0, y - 1);
                            index_y_max = std::min(image_height_1, y + 1);
                            index_x_min = std::max(0, x - 1);
                            index_x_max = std::min(image_width_1, x + 1);
                            for (int extend_y = index_y_min; extend_y <= index_y_max; extend_y++) {
                                for (int extend_x = index_x_min; extend_x <= index_x_max; extend_x++) {
                                    if (image.at<cv::Vec3b>(extend_y, extend_x) == other_color) {
                                        image.at<cv::Vec3b>(extend_y, extend_x) = white_color;
                                    }
                                }
                            }
                        }
                    } else {
                        continue;
                    }
                }
            }
        }
    }

    /// cv的需要 flip y 才能转到俯视视角
    cv::Mat image_flip;
    cv::flip(image, image_flip, 1);
    return image_flip;
}

}  // namespace lightning::g2p5