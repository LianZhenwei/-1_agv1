#pragma once //防止头文件被多次包含的方式，确保编译过程中头文件只被包含一次。

#include <cmath>
#include <string>

class Vec2d {
 public:
  //! 构造函数，接收 x 和 y 坐标。
  // 使用 constexpr 关键字意味着如果可能，表达式将在编译时计算。
  // noexcept 关键字表示这个函数不会抛出异常。
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}
  double x() const { return x_; }
  double y() const { return y_; }
 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

 struct point2
 {
   double x;
   double y;
 };



