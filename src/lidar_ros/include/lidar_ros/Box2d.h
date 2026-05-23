#pragma once //防止头文件被多次包含的方式，确保编译过程中头文件只被包含一次。
#include <vector>
#include <string>
#include <math.h>
#include <stdio.h> 
#include <limits>
#include "vec2d.h"

// 矩形Box2d 是世界坐标系下的一个矩形
class Box_2d {
 public:
  Box_2d(const Vec2d &center, const double heading, const double length,
        const double width, const double speed);
  double cos_heading() const { return cos_heading_; }
  double sin_heading() const { return sin_heading_; }  
  double heading() const { return heading_; }  
  double max_x() const { return max_x_; }
  double min_x() const { return min_x_; }
  double max_y() const { return max_y_; }
  double min_y() const { return min_y_; }
  double half_length() const { return half_length_; }
  double half_width() const { return half_width_; }
  double length() const { return length_; }
  double width() const { return width_; }
  double center_x() const { return center_.x(); }
  double center_y() const { return center_.y(); }
  // std::hypot(): 计算两数的欧几里得距离（即斜边长度）的函数。函数确保了计算过程中不会发生溢出或下溢，比直接使用 std::sqrt(x*x + y*y) 更为安全和准确
  double half_diagonal() const { return std::hypot(length_, width_)/2;} //返回矩形框对角线长度的一半
  double speed() const { return speed_;}
  
  void InitCorners();
  bool HasOverlap(const Box_2d &box) const;     //矩形框碰撞检测
  double DistanceTo(const Vec2d &point) const;  //矩形框到点的距离
  double DistanceTo(const Box_2d &box) const;   //矩形框到矩形框的距离(中心距离)


  std::vector<Vec2d> corners_;                 //矩形框的四个角点
 
 private:
  Vec2d center_;
  double length_ = 0.0;       //长
  double width_ = 0.0;        //宽
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;      //朝向，弧度制
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
  double speed_ = 0.0;

  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};
