#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace rpp
{
namespace trajectory_optimization
{
namespace minco_spline
{

class BandedSystem
{
public:
  void create(int n, int lower_bw, int upper_bw)
  {
    n_ = n;
    lower_bw_ = lower_bw;
    upper_bw_ = upper_bw;
    data_.assign(static_cast<size_t>(n_) * static_cast<size_t>(lower_bw_ + upper_bw_ + 1), 0.0);
  }

  void reset() { std::fill(data_.begin(), data_.end(), 0.0); }

  double& operator()(int i, int j) { return data_[static_cast<size_t>(i - j + upper_bw_) * n_ + j]; }

  const double& operator()(int i, int j) const { return data_[static_cast<size_t>(i - j + upper_bw_) * n_ + j]; }

  void factorizeLU()
  {
    for (int k = 0; k <= n_ - 2; ++k)
    {
      const int i_max = std::min(k + lower_bw_, n_ - 1);
      const double pivot = operator()(k, k);

      for (int i = k + 1; i <= i_max; ++i)
      {
        if (operator()(i, k) != 0.0)
        {
          operator()(i, k) /= pivot;
        }
      }

      const int j_max = std::min(k + upper_bw_, n_ - 1);
      for (int j = k + 1; j <= j_max; ++j)
      {
        const double c = operator()(k, j);
        if (c != 0.0)
        {
          for (int i = k + 1; i <= i_max; ++i)
          {
            const double l = operator()(i, k);
            if (l != 0.0)
            {
              operator()(i, j) -= l * c;
            }
          }
        }
      }
    }
  }

  template <typename EigenMat>
  void solve(EigenMat& b) const
  {
    for (int j = 0; j <= n_ - 1; ++j)
    {
      const int i_max = std::min(j + lower_bw_, n_ - 1);
      for (int i = j + 1; i <= i_max; ++i)
      {
        const double l = operator()(i, j);
        if (l != 0.0)
        {
          b.row(i) -= l * b.row(j);
        }
      }
    }

    for (int j = n_ - 1; j >= 0; --j)
    {
      b.row(j) /= operator()(j, j);
      const int i_min = std::max(0, j - upper_bw_);
      for (int i = i_min; i <= j - 1; ++i)
      {
        const double u = operator()(i, j);
        if (u != 0.0)
        {
          b.row(i) -= u * b.row(j);
        }
      }
    }
  }

  template <typename EigenMat>
  void solveAdj(EigenMat& b) const
  {
    for (int j = 0; j <= n_ - 1; ++j)
    {
      b.row(j) /= operator()(j, j);
      const int i_max = std::min(j + upper_bw_, n_ - 1);
      for (int i = j + 1; i <= i_max; ++i)
      {
        const double u = operator()(j, i);
        if (u != 0.0)
        {
          b.row(i) -= u * b.row(j);
        }
      }
    }

    for (int j = n_ - 1; j >= 0; --j)
    {
      const int i_min = std::max(0, j - lower_bw_);
      for (int i = i_min; i <= j - 1; ++i)
      {
        const double l = operator()(j, i);
        if (l != 0.0)
        {
          b.row(i) -= l * b.row(j);
        }
      }
    }
  }

private:
  int n_{ 0 };
  int lower_bw_{ 0 };
  int upper_bw_{ 0 };
  std::vector<double> data_;
};

template <int Dim>
struct QuinticPiece
{
  using Vec = Eigen::Matrix<double, Dim, 1>;

  double T{ 0.0 };
  Eigen::Matrix<double, Dim, 6> c{ Eigen::Matrix<double, Dim, 6>::Zero() };

  Vec pos(double t) const
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    const double t5 = t4 * t;
    return c.col(0) + c.col(1) * t + c.col(2) * t2 + c.col(3) * t3 + c.col(4) * t4 + c.col(5) * t5;
  }

  Vec vel(double t) const
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t2 * t2;
    return c.col(1) + 2.0 * c.col(2) * t + 3.0 * c.col(3) * t2 + 4.0 * c.col(4) * t3 + 5.0 * c.col(5) * t4;
  }

  Vec acc(double t) const
  {
    const double t2 = t * t;
    const double t3 = t2 * t;
    return 2.0 * c.col(2) + 6.0 * c.col(3) * t + 12.0 * c.col(4) * t2 + 20.0 * c.col(5) * t3;
  }

  Vec jerk(double t) const
  {
    const double t2 = t * t;
    return 6.0 * c.col(3) + 24.0 * c.col(4) * t + 60.0 * c.col(5) * t2;
  }

  Vec at(double t, int order) const
  {
    if (order <= 0)
      return pos(t);
    if (order == 1)
      return vel(t);
    if (order == 2)
      return acc(t);
    if (order == 3)
      return jerk(t);

    // order 4/5 for completeness
    if (order == 4)
    {
      return 24.0 * c.col(4) + 120.0 * c.col(5) * t;
    }
    if (order == 5)
    {
      return 120.0 * c.col(5);
    }
    return Vec::Zero();
  }
};

using QuinticPiece2D = QuinticPiece<2>;

template <int Dim>
class PiecewiseQuinticTrajectory
{
public:
  using Piece = QuinticPiece<Dim>;
  using Vec = Eigen::Matrix<double, Dim, 1>;

  void clear() { pieces_.clear(); }
  void reserve(size_t n) { pieces_.reserve(n); }
  void push_back(const Piece& p) { pieces_.push_back(p); }

  bool empty() const { return pieces_.empty(); }
  size_t size() const { return pieces_.size(); }
  const Piece& operator[](size_t i) const { return pieces_[i]; }

  double totalTime() const
  {
    double sum = 0.0;
    for (const auto& p : pieces_)
      sum += p.T;
    return sum;
  }

  Vec pos(double t) const { return eval(t, &Piece::pos); }
  Vec vel(double t) const { return eval(t, &Piece::vel); }
  Vec acc(double t) const { return eval(t, &Piece::acc); }
  Vec jerk(double t) const { return eval(t, &Piece::jerk); }
  Vec at(double t, int order) const { return evalOrder(t, order); }

private:
  template <typename Fn>
  Vec eval(double t, Fn fn) const
  {
    if (pieces_.empty())
    {
      return Vec::Zero();
    }

    double local_t = t;
    for (size_t i = 0; i < pieces_.size(); ++i)
    {
      const auto& p = pieces_[i];
      if (i == pieces_.size() - 1 || local_t <= p.T)
      {
        const double clamped_t = std::max(0.0, std::min(local_t, p.T));
        return (p.*fn)(clamped_t);
      }
      local_t -= p.T;
    }

    return (pieces_.back().*fn)(pieces_.back().T);
  }

  Vec evalOrder(double t, int order) const
  {
    if (pieces_.empty())
    {
      return Vec::Zero();
    }

    double local_t = t;
    for (size_t i = 0; i < pieces_.size(); ++i)
    {
      const auto& p = pieces_[i];
      if (i == pieces_.size() - 1 || local_t <= p.T)
      {
        const double clamped_t = std::max(0.0, std::min(local_t, p.T));
        return p.at(clamped_t, order);
      }
      local_t -= p.T;
    }

    return pieces_.back().at(pieces_.back().T, order);
  }

private:
  std::vector<Piece> pieces_;
};

using PiecewiseQuinticTrajectory2D = PiecewiseQuinticTrajectory<2>;

// Piecewise quintic minimum-jerk (DimD) with PVA boundary and intermediate position constraints.
template <int Dim>
class MinJerkOpt
{
public:
  using Vec = Eigen::Matrix<double, Dim, 1>;
  using MatPVA = Eigen::Matrix<double, Dim, 3>;

  void reset(const MatPVA& head_pva, const MatPVA& tail_pva, int piece_num)
  {
    n_ = std::max(1, piece_num);
    head_pva_ = head_pva;
    tail_pva_ = tail_pva;

    t1_.resize(n_);
    t2_.resize(n_);
    t3_.resize(n_);
    t4_.resize(n_);
    t5_.resize(n_);

    A_.create(6 * n_, 6, 6);
    coeffs_.resize(6 * n_, Dim);
  }

  void generate(const Eigen::MatrixXd& intermediate_positions, const Eigen::VectorXd& segment_times)
  {
    t1_ = segment_times;
    t2_ = t1_.cwiseProduct(t1_);
    t3_ = t2_.cwiseProduct(t1_);
    t4_ = t2_.cwiseProduct(t2_);
    t5_ = t4_.cwiseProduct(t1_);

    A_.reset();
    coeffs_.setZero();

    // Head PVA at t=0
    A_(0, 0) = 1.0;
    A_(1, 1) = 1.0;
    A_(2, 2) = 2.0;
    coeffs_.row(0) = head_pva_.col(0).transpose();
    coeffs_.row(1) = head_pva_.col(1).transpose();
    coeffs_.row(2) = head_pva_.col(2).transpose();

    // Intermediate constraints and continuity.
    for (int i = 0; i < n_ - 1; ++i)
    {
      // jerk continuity
      A_(6 * i + 3, 6 * i + 3) = 6.0;
      A_(6 * i + 3, 6 * i + 4) = 24.0 * t1_(i);
      A_(6 * i + 3, 6 * i + 5) = 60.0 * t2_(i);
      A_(6 * i + 3, 6 * i + 9) = -6.0;
      // snap continuity
      A_(6 * i + 4, 6 * i + 4) = 24.0;
      A_(6 * i + 4, 6 * i + 5) = 120.0 * t1_(i);
      A_(6 * i + 4, 6 * i + 10) = -24.0;

      // position at end of piece i equals given intermediate point
      A_(6 * i + 5, 6 * i + 0) = 1.0;
      A_(6 * i + 5, 6 * i + 1) = t1_(i);
      A_(6 * i + 5, 6 * i + 2) = t2_(i);
      A_(6 * i + 5, 6 * i + 3) = t3_(i);
      A_(6 * i + 5, 6 * i + 4) = t4_(i);
      A_(6 * i + 5, 6 * i + 5) = t5_(i);
      if (intermediate_positions.rows() > 0)
      {
        coeffs_.row(6 * i + 5) = intermediate_positions.row(i);
      }

      // position continuity
      A_(6 * i + 6, 6 * i + 0) = 1.0;
      A_(6 * i + 6, 6 * i + 1) = t1_(i);
      A_(6 * i + 6, 6 * i + 2) = t2_(i);
      A_(6 * i + 6, 6 * i + 3) = t3_(i);
      A_(6 * i + 6, 6 * i + 4) = t4_(i);
      A_(6 * i + 6, 6 * i + 5) = t5_(i);
      A_(6 * i + 6, 6 * i + 6) = -1.0;
      // velocity continuity
      A_(6 * i + 7, 6 * i + 1) = 1.0;
      A_(6 * i + 7, 6 * i + 2) = 2.0 * t1_(i);
      A_(6 * i + 7, 6 * i + 3) = 3.0 * t2_(i);
      A_(6 * i + 7, 6 * i + 4) = 4.0 * t3_(i);
      A_(6 * i + 7, 6 * i + 5) = 5.0 * t4_(i);
      A_(6 * i + 7, 6 * i + 7) = -1.0;
      // acceleration continuity
      A_(6 * i + 8, 6 * i + 2) = 2.0;
      A_(6 * i + 8, 6 * i + 3) = 6.0 * t1_(i);
      A_(6 * i + 8, 6 * i + 4) = 12.0 * t2_(i);
      A_(6 * i + 8, 6 * i + 5) = 20.0 * t3_(i);
      A_(6 * i + 8, 6 * i + 8) = -2.0;
    }

    // Tail PVA at end of last segment.
    const int last = n_ - 1;

    A_(6 * n_ - 3, 6 * last + 0) = 1.0;
    A_(6 * n_ - 3, 6 * last + 1) = t1_(last);
    A_(6 * n_ - 3, 6 * last + 2) = t2_(last);
    A_(6 * n_ - 3, 6 * last + 3) = t3_(last);
    A_(6 * n_ - 3, 6 * last + 4) = t4_(last);
    A_(6 * n_ - 3, 6 * last + 5) = t5_(last);

    A_(6 * n_ - 2, 6 * last + 1) = 1.0;
    A_(6 * n_ - 2, 6 * last + 2) = 2.0 * t1_(last);
    A_(6 * n_ - 2, 6 * last + 3) = 3.0 * t2_(last);
    A_(6 * n_ - 2, 6 * last + 4) = 4.0 * t3_(last);
    A_(6 * n_ - 2, 6 * last + 5) = 5.0 * t4_(last);

    A_(6 * n_ - 1, 6 * last + 2) = 2.0;
    A_(6 * n_ - 1, 6 * last + 3) = 6.0 * t1_(last);
    A_(6 * n_ - 1, 6 * last + 4) = 12.0 * t2_(last);
    A_(6 * n_ - 1, 6 * last + 5) = 20.0 * t3_(last);

    coeffs_.row(6 * n_ - 3) = tail_pva_.col(0).transpose();
    coeffs_.row(6 * n_ - 2) = tail_pva_.col(1).transpose();
    coeffs_.row(6 * n_ - 1) = tail_pva_.col(2).transpose();

    A_.factorizeLU();
    A_.solve(coeffs_);
  }

  const Eigen::MatrixXd& getCoeffs() const { return coeffs_; }

  PiecewiseQuinticTrajectory<Dim> getTrajectory() const
  {
    PiecewiseQuinticTrajectory<Dim> traj;
    traj.reserve(static_cast<size_t>(n_));
    for (int i = 0; i < n_; ++i)
    {
      QuinticPiece<Dim> piece;
      piece.T = t1_(i);
      for (int k = 0; k < 6; ++k)
      {
        for (int d = 0; d < Dim; ++d)
        {
          piece.c(d, k) = coeffs_(6 * i + k, d);
        }
      }
      traj.push_back(piece);
    }
    return traj;
  }

  double getEnergy() const
  {
    double energy = 0.0;
    for (int seg = 0; seg < n_; ++seg)
    {
      const double T = t1_(seg);
      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T2 * T2;
      const double T5 = T4 * T;

      const Vec c3 = coeffs_.row(6 * seg + 3).transpose();
      const Vec c4 = coeffs_.row(6 * seg + 4).transpose();
      const Vec c5 = coeffs_.row(6 * seg + 5).transpose();

      const Vec a = 6.0 * c3;
      const Vec b = 24.0 * c4;
      const Vec c = 60.0 * c5;

      energy += a.squaredNorm() * T;
      energy += a.dot(b) * T2;
      energy += (2.0 * a.dot(c) + b.squaredNorm()) * (T3 / 3.0);
      energy += b.dot(c) * (T4 / 2.0);
      energy += c.squaredNorm() * (T5 / 5.0);
    }
    return energy;
  }

  Eigen::MatrixXd getEnergyPartialGradByCoeffs() const
  {
    Eigen::MatrixXd gdC(6 * n_, Dim);
    gdC.setZero();

    for (int seg = 0; seg < n_; ++seg)
    {
      const double T = t1_(seg);
      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T2 * T2;
      const double T5 = T4 * T;

      const Vec c3 = coeffs_.row(6 * seg + 3).transpose();
      const Vec c4 = coeffs_.row(6 * seg + 4).transpose();
      const Vec c5 = coeffs_.row(6 * seg + 5).transpose();

      const Vec a = 6.0 * c3;
      const Vec b = 24.0 * c4;
      const Vec c = 60.0 * c5;

      const Vec dE_da = 2.0 * a * T + b * T2 + 2.0 * c * (T3 / 3.0);
      const Vec dE_db = a * T2 + 2.0 * b * (T3 / 3.0) + c * (T4 / 2.0);
      const Vec dE_dc = 2.0 * a * (T3 / 3.0) + b * (T4 / 2.0) + 2.0 * c * (T5 / 5.0);

      gdC.row(6 * seg + 3) = (6.0 * dE_da).transpose();
      gdC.row(6 * seg + 4) = (24.0 * dE_db).transpose();
      gdC.row(6 * seg + 5) = (60.0 * dE_dc).transpose();
    }

    return gdC;
  }

  Eigen::VectorXd getEnergyPartialGradByTimesExplicit() const
  {
    Eigen::VectorXd gdT(n_);
    gdT.setZero();

    for (int seg = 0; seg < n_; ++seg)
    {
      const double T = t1_(seg);
      const double T2 = T * T;
      const double T3 = T2 * T;
      const double T4 = T2 * T2;

      const Vec c3 = coeffs_.row(6 * seg + 3).transpose();
      const Vec c4 = coeffs_.row(6 * seg + 4).transpose();
      const Vec c5 = coeffs_.row(6 * seg + 5).transpose();

      const Vec a = 6.0 * c3;
      const Vec b = 24.0 * c4;
      const Vec c = 60.0 * c5;

      gdT(seg) = a.squaredNorm() + 2.0 * a.dot(b) * T + (2.0 * a.dot(c) + b.squaredNorm()) * T2 + 2.0 * b.dot(c) * T3 +
                 c.squaredNorm() * T4;
    }

    return gdT;
  }

  // Given gradient w.r.t. coefficients, compute gradients w.r.t. intermediate points and segment times (implicit part).
  void adjointPropagate(const Eigen::MatrixXd& gradC, Eigen::MatrixXd& gradIntermediatePositions, Eigen::VectorXd& gradTimesImplicit) const
  {
    gradIntermediatePositions.resize(std::max(0, n_ - 1), Dim);
    gradIntermediatePositions.setZero();
    gradTimesImplicit.resize(n_);
    gradTimesImplicit.setZero();

    if (n_ <= 0)
      return;

    Eigen::MatrixXd lambda = gradC;
    A_.solveAdj(lambda);

    for (int i = 0; i < n_ - 1; ++i)
    {
      gradIntermediatePositions.row(i) = lambda.row(6 * i + 5);
    }

    for (int seg = 0; seg < n_; ++seg)
    {
      const double T = t1_(seg);
      const double T2 = t2_(seg);
      const double T3 = t3_(seg);
      const double T4 = t4_(seg);

      auto add = [&](int row, int col, double dA) {
        gradTimesImplicit(seg) += -dA * lambda.row(row).dot(coeffs_.row(col));
      };

      if (seg < n_ - 1)
      {
        const int i = seg;
        add(6 * i + 3, 6 * i + 4, 24.0);
        add(6 * i + 3, 6 * i + 5, 120.0 * T);

        add(6 * i + 4, 6 * i + 5, 120.0);

        add(6 * i + 5, 6 * i + 1, 1.0);
        add(6 * i + 5, 6 * i + 2, 2.0 * T);
        add(6 * i + 5, 6 * i + 3, 3.0 * T2);
        add(6 * i + 5, 6 * i + 4, 4.0 * T3);
        add(6 * i + 5, 6 * i + 5, 5.0 * T4);

        add(6 * i + 6, 6 * i + 1, 1.0);
        add(6 * i + 6, 6 * i + 2, 2.0 * T);
        add(6 * i + 6, 6 * i + 3, 3.0 * T2);
        add(6 * i + 6, 6 * i + 4, 4.0 * T3);
        add(6 * i + 6, 6 * i + 5, 5.0 * T4);

        add(6 * i + 7, 6 * i + 2, 2.0);
        add(6 * i + 7, 6 * i + 3, 6.0 * T);
        add(6 * i + 7, 6 * i + 4, 12.0 * T2);
        add(6 * i + 7, 6 * i + 5, 20.0 * T3);

        add(6 * i + 8, 6 * i + 3, 6.0);
        add(6 * i + 8, 6 * i + 4, 24.0 * T);
        add(6 * i + 8, 6 * i + 5, 60.0 * T2);
      }
      else
      {
        const int last = n_ - 1;
        add(6 * n_ - 3, 6 * last + 1, 1.0);
        add(6 * n_ - 3, 6 * last + 2, 2.0 * T);
        add(6 * n_ - 3, 6 * last + 3, 3.0 * T2);
        add(6 * n_ - 3, 6 * last + 4, 4.0 * T3);
        add(6 * n_ - 3, 6 * last + 5, 5.0 * T4);

        add(6 * n_ - 2, 6 * last + 2, 2.0);
        add(6 * n_ - 2, 6 * last + 3, 6.0 * T);
        add(6 * n_ - 2, 6 * last + 4, 12.0 * T2);
        add(6 * n_ - 2, 6 * last + 5, 20.0 * T3);

        add(6 * n_ - 1, 6 * last + 3, 6.0);
        add(6 * n_ - 1, 6 * last + 4, 24.0 * T);
        add(6 * n_ - 1, 6 * last + 5, 60.0 * T2);
      }
    }
  }

  void attachGrad(Eigen::MatrixXd& gradIntermediatePositions, Eigen::VectorXd& gradTimes) const
  {
    const Eigen::MatrixXd gdC = getEnergyPartialGradByCoeffs();
    Eigen::VectorXd gdT_imp;
    adjointPropagate(gdC, gradIntermediatePositions, gdT_imp);
    gradTimes = getEnergyPartialGradByTimesExplicit() + gdT_imp;
  }

private:
  int n_{ 1 };
  MatPVA head_pva_{ MatPVA::Zero() };
  MatPVA tail_pva_{ MatPVA::Zero() };

  Eigen::VectorXd t1_, t2_, t3_, t4_, t5_;
  BandedSystem A_;
  Eigen::MatrixXd coeffs_;
};

using MinJerkOpt2D = MinJerkOpt<2>;
using MinJerkOpt3D = MinJerkOpt<3>;

}  // namespace minco_spline
}  // namespace trajectory_optimization
}  // namespace rpp
