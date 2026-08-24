#include "openarmx_deploy/arm_kinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace openarmx_deploy {

namespace {

using Vec3 = std::array<double, 3>;
// 3x3 旋转矩阵, row-major: {r00,r01,r02, r10,r11,r12, r20,r21,r22}
using Mat3 = std::array<double, 9>;

Mat3 mat3Mul(const Mat3& a, const Mat3& b) {
  Mat3 r{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) s += a[i * 3 + k] * b[k * 3 + j];
      r[i * 3 + j] = s;
    }
  }
  return r;
}

Mat3 rotX(double a) {
  double c = std::cos(a), s = std::sin(a);
  return {1, 0, 0, 0, c, -s, 0, s, c};
}
Mat3 rotY(double a) {
  double c = std::cos(a), s = std::sin(a);
  return {c, 0, s, 0, 1, 0, -s, 0, c};
}
Mat3 rotZ(double a) {
  double c = std::cos(a), s = std::sin(a);
  return {c, -s, 0, s, c, 0, 0, 0, 1};
}

Mat3 rotAxis(const Vec3& axis, double a) {
  // Rodrigues 旋转公式; axis 为 URDF 中已归一化的单位向量
  double c = std::cos(a), s = std::sin(a), C = 1.0 - c;
  double x = axis[0], y = axis[1], z = axis[2];
  return {
      c + x * x * C,       x * y * C - z * s,   x * z * C + y * s,
      y * x * C + z * s,   c + y * y * C,       y * z * C - x * s,
      z * x * C - y * s,   z * y * C + x * s,   c + z * z * C,
  };
}

Mat3 rpyToRot(const Vec3& rpy) {
  // URDF rpy: 固定轴 XYZ 顺序 R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return mat3Mul(mat3Mul(rotZ(rpy[2]), rotY(rpy[1])), rotX(rpy[0]));
}

Vec3 sub3(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

double norm3(const Vec3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 3x3 线性方程求解 (Cramer 规则 + 伴随矩阵); DLS 阻尼保证 A 良态
bool solve3x3(const Mat3& A, const Vec3& b, Vec3& x) {
  double det = A[0] * (A[4] * A[8] - A[5] * A[7])
             - A[1] * (A[3] * A[8] - A[5] * A[6])
             + A[2] * (A[3] * A[7] - A[4] * A[6]);
  if (std::abs(det) < 1e-12) return false;
  double id = 1.0 / det;

  // 伴随矩阵 (逆 = adj / det)
  Mat3 adj;
  adj[0] = (A[4] * A[8] - A[5] * A[7]);
  adj[1] = (A[2] * A[7] - A[1] * A[8]);
  adj[2] = (A[1] * A[5] - A[2] * A[4]);
  adj[3] = (A[5] * A[6] - A[3] * A[8]);
  adj[4] = (A[0] * A[8] - A[2] * A[6]);
  adj[5] = (A[2] * A[3] - A[0] * A[5]);
  adj[6] = (A[3] * A[7] - A[4] * A[6]);
  adj[7] = (A[1] * A[6] - A[0] * A[7]);
  adj[8] = (A[0] * A[4] - A[1] * A[3]);

  x[0] = id * (adj[0] * b[0] + adj[1] * b[1] + adj[2] * b[2]);
  x[1] = id * (adj[3] * b[0] + adj[4] * b[1] + adj[5] * b[2]);
  x[2] = id * (adj[6] * b[0] + adj[7] * b[1] + adj[8] * b[2]);
  return true;
}

}  // namespace

ArmKinematics::ArmKinematics(Side side) {
  if (side == Side::kRight) {
    joints_ = {{
        {{-0.0875, 0.00016852, 0.31809},
         {0, 0, 0}, {1, 0, 0}},
        {{-0.067, 0.0229, 0}, {0, 0, 0}, {0, 1, 0}},
        {{0, -0.0229, -0.07825}, {0, 0, 0}, {0, 0, 1}},
        {{0, 0.0005, -0.1625}, {0, 0, 0}, {0, -1, 0}},
        {{0, 0, -0.1049}, {0, 0, 0}, {0, 0, 1}},
        {{0, 9.50288425331097e-05, -0.12049999999997},
         {0, 0, 0}, {0, 1, 0}},
        {{-0.0199, 0, 0}, {0, 0, 0}, {-1, 0, 0}},
    }};
    lower_ = {-3.23, -0.376, -3.13, 0.0, -2.6, -0.5246, -1.54};
    upper_ = {1.65, 2.86, 0.0, 2.43, 0.0, 0.5641, 1.54};
  } else {
    joints_ = {{
        {{0.0875, 0.00016852, 0.31809}, {0, 0, 0}, {-1, 0, 0}},
        {{0.067, 0.0229, 0}, {0, 0, 0}, {0, 1, 0}},
        {{0, -0.0229, -0.07825}, {0, 0, 0}, {0, 0, -1}},
        {{0, 0.0005, -0.1625}, {0, 0, 0}, {0, 1, 0}},
        {{0, 0, -0.1049}, {0, 0, 0}, {0, 0, -1}},
        {{0, 0, -0.1205}, {0, 0, 0}, {0, 1, 0}},
        {{0.0199, 9.503e-05, 0}, {0, 0, 0}, {1, 0, 0}},
    }};
    lower_ = {-1.6417, -2.8723, -3.136, 0.0, -2.61, -0.549, -1.54};
    upper_ = {3.2402, 0.38, 0.0, 2.43, 0.0, 0.544, 1.55};
  }
}

void ArmKinematics::forward(const std::vector<double>& q,
                            std::array<double, 3>& pos,
                            std::array<double, 9>& R) const {
  if (q.size() != 7) {
    throw std::invalid_argument("ArmKinematics::forward expects 7 joint angles");
  }
  Mat3 Racc{1, 0, 0, 0, 1, 0, 0, 0, 1};
  Vec3 p{0, 0, 0};

  for (int i = 0; i < 7; ++i) {
    const auto& j = joints_[i];
    // T_child = SE3(rpy, xyz) * SE3(axis, q): R 累乘, 平移在父系内累加
    Mat3 Ri = mat3Mul(rpyToRot(j.rpy), rotAxis(j.axis, q[i]));
    p[0] += Racc[0] * j.xyz[0] + Racc[1] * j.xyz[1] + Racc[2] * j.xyz[2];
    p[1] += Racc[3] * j.xyz[0] + Racc[4] * j.xyz[1] + Racc[5] * j.xyz[2];
    p[2] += Racc[6] * j.xyz[0] + Racc[7] * j.xyz[1] + Racc[8] * j.xyz[2];
    Racc = mat3Mul(Racc, Ri);
  }

  pos = p;
  R = Racc;
}

std::array<double, 3> ArmKinematics::forwardPos(const std::vector<double>& q) const {
  std::array<double, 3> pos;
  std::array<double, 9> R;
  forward(q, pos, R);
  return pos;
}

bool ArmKinematics::inversePosition(const std::array<double, 3>& target_pos,
                                    const std::vector<double>& q_init,
                                    std::vector<double>& out_q) const {
  std::vector<double> q = q_init;
  if (q.size() != 7) q.assign(7, 0.0);

  const int max_iter = 300;
  const double eps = 5e-3;      // 位置收敛阈值 (m) — 数值雅可比 + 7DOF冗余, 5mm 足够抓取精度
  const double damper = 0.01;   // DLS 阻尼
  const double max_step = 0.5;  // 单次迭代最大关节增量 (rad)
  const double h = 1e-6;        // 数值雅可比扰动

  double err_norm = 1e9;
  for (int it = 0; it < max_iter; ++it) {
    Vec3 p = forwardPos(q);
    Vec3 err = sub3(target_pos, p);
    err_norm = norm3(err);
    if (err_norm < eps) {
      out_q = q;
      return true;
    }

    // 数值雅可比 J (3x7, row-major), 中心差分
    std::array<double, 21> J{};
    for (int j = 0; j < 7; ++j) {
      std::vector<double> qp = q, qm = q;
      qp[j] += h;
      qm[j] -= h;
      Vec3 pp = forwardPos(qp);
      Vec3 pm = forwardPos(qm);
      double inv = 1.0 / (2.0 * h);
      J[0 * 7 + j] = (pp[0] - pm[0]) * inv;
      J[1 * 7 + j] = (pp[1] - pm[1]) * inv;
      J[2 * 7 + j] = (pp[2] - pm[2]) * inv;
    }

    // DLS: dq = J^T (J J^T + λI)^-1 err, 只需解 3x3
    Mat3 A{};
    for (int i = 0; i < 3; ++i) {
      for (int k = 0; k < 3; ++k) {
        double s = 0.0;
        for (int j = 0; j < 7; ++j) s += J[i * 7 + j] * J[k * 7 + j];
        A[i * 3 + k] = s + (i == k ? damper : 0.0);
      }
    }
    Vec3 step;
    if (!solve3x3(A, err, step)) break;

    std::array<double, 7> dq{};
    for (int j = 0; j < 7; ++j) {
      dq[j] = J[0 * 7 + j] * step[0] + J[1 * 7 + j] * step[1] +
              J[2 * 7 + j] * step[2];
    }

    // 步长裁剪
    double n = 0.0;
    for (int j = 0; j < 7; ++j) n += dq[j] * dq[j];
    n = std::sqrt(n);
    if (n > max_step) {
      double f = max_step / n;
      for (int j = 0; j < 7; ++j) dq[j] *= f;
    }

    // 积分 + 限位裁剪
    for (int j = 0; j < 7; ++j) {
      q[j] = std::max(lower_[j], std::min(upper_[j], q[j] + dq[j]));
    }
  }

  out_q = q;
  return false;
}

}  // namespace openarmx_deploy
