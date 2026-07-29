#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace Polybolos {
namespace Fusion {

/// Small dense row-major matrix (no Eigen - deterministic, header-only).
class Mat {
public:
 Mat() = default;
 Mat(int rows, int cols, double fill = 0.0)
 : rows_(rows), cols_(cols), data_(static_cast<size_t>(rows * cols), fill) {
 if (rows < 0 || cols < 0) {
 throw std::invalid_argument("Mat dims");
 }
 }

 static Mat identity(int n) {
 Mat m(n, n, 0.0);
 for (int i = 0; i < n; ++i) {
 m(i, i) = 1.0;
 }
 return m;
 }

 static Mat diag(const std::vector<double>& d) {
 Mat m(static_cast<int>(d.size()), static_cast<int>(d.size()), 0.0);
 for (size_t i = 0; i < d.size(); ++i) {
 m(static_cast<int>(i), static_cast<int>(i)) = d[i];
 }
 return m;
 }

 int rows() const { return rows_; }
 int cols() const { return cols_; }

 double& operator()(int r, int c) { return data_[static_cast<size_t>(r * cols_ + c)]; }
 double operator()(int r, int c) const {
 return data_[static_cast<size_t>(r * cols_ + c)];
 }

 std::vector<double> col(int c) const {
 std::vector<double> out(static_cast<size_t>(rows_));
 for (int r = 0; r < rows_; ++r) {
 out[static_cast<size_t>(r)] = (*this)(r, c);
 }
 return out;
 }

 void setCol(int c, const std::vector<double>& v) {
 for (int r = 0; r < rows_ && r < static_cast<int>(v.size()); ++r) {
 (*this)(r, c) = v[static_cast<size_t>(r)];
 }
 }

 Mat transpose() const {
 Mat t(cols_, rows_);
 for (int r = 0; r < rows_; ++r) {
 for (int c = 0; c < cols_; ++c) {
 t(c, r) = (*this)(r, c);
 }
 }
 return t;
 }

 Mat operator+(const Mat& o) const {
 Mat out(rows_, cols_);
 for (size_t i = 0; i < data_.size(); ++i) {
 out.data_[i] = data_[i] + o.data_[i];
 }
 return out;
 }

 Mat operator-(const Mat& o) const {
 Mat out(rows_, cols_);
 for (size_t i = 0; i < data_.size(); ++i) {
 out.data_[i] = data_[i] - o.data_[i];
 }
 return out;
 }

 Mat operator*(const Mat& o) const {
 if (cols_ != o.rows_) {
 throw std::invalid_argument("Mat mul dims");
 }
 Mat out(rows_, o.cols_, 0.0);
 for (int r = 0; r < rows_; ++r) {
 for (int k = 0; k < cols_; ++k) {
 const double aik = (*this)(r, k);
 for (int c = 0; c < o.cols_; ++c) {
 out(r, c) += aik * o(k, c);
 }
 }
 }
 return out;
 }

 Mat operator*(double s) const {
 Mat out = *this;
 for (double& v : out.data_) {
 v *= s;
 }
 return out;
 }

 /// Invert square matrix via Gauss-Jordan (deterministic).
 Mat inverse() const {
 if (rows_ != cols_) {
 throw std::invalid_argument("inverse non-square");
 }
 const int n = rows_;
 Mat a = *this;
 Mat inv = identity(n);
 for (int col = 0; col < n; ++col) {
 int pivot = col;
 double best = std::fabs(a(col, col));
 for (int r = col + 1; r < n; ++r) {
 const double v = std::fabs(a(r, col));
 if (v > best) {
 best = v;
 pivot = r;
 }
 }
 if (best < 1e-15) {
 throw std::runtime_error("singular matrix");
 }
 if (pivot != col) {
 for (int c = 0; c < n; ++c) {
 std::swap(a(col, c), a(pivot, c));
 std::swap(inv(col, c), inv(pivot, c));
 }
 }
 const double diag = a(col, col);
 for (int c = 0; c < n; ++c) {
 a(col, c) /= diag;
 inv(col, c) /= diag;
 }
 for (int r = 0; r < n; ++r) {
 if (r == col) {
 continue;
 }
 const double f = a(r, col);
 for (int c = 0; c < n; ++c) {
 a(r, c) -= f * a(col, c);
 inv(r, c) -= f * inv(col, c);
 }
 }
 }
 return inv;
 }

 double trace() const {
 double t = 0.0;
 const int n = std::min(rows_, cols_);
 for (int i = 0; i < n; ++i) {
 t += (*this)(i, i);
 }
 return t;
 }

 std::vector<double> diagVec() const {
 const int n = std::min(rows_, cols_);
 std::vector<double> d(static_cast<size_t>(n));
 for (int i = 0; i < n; ++i) {
 d[static_cast<size_t>(i)] = (*this)(i, i);
 }
 return d;
 }

private:
 int rows_ = 0;
 int cols_ = 0;
 std::vector<double> data_;
};

inline Mat operator*(double s, const Mat& m) { return m * s; }

} // namespace Fusion
} // namespace Polybolos
