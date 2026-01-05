#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

bool EnsureDir(const std::string &path) {
  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    return (info.st_mode & S_IFDIR) != 0;
  }
  return mkdir(path.c_str(), 0755) == 0;
}

} // namespace

int main(int argc, char **argv) {
  std::string output_dir = "documents/solutions";
  if (argc > 1) {
    output_dir = argv[1];
  }
  if (!EnsureDir(output_dir)) {
    std::cerr << "Failed to create output directory: " << output_dir << "\n";
    return 1;
  }

  const std::string csv_path = output_dir + "/df_quaternion.csv";
  const std::string txt_path = output_dir + "/df_quaternion.txt";

  std::ofstream csv(csv_path);
  if (!csv.is_open()) {
    std::cerr << "Failed to open CSV file: " << csv_path << "\n";
    return 1;
  }

  const double dt = 0.02;
  const double t_end = 2.0 * M_PI;
  const double g = 9.81;
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);

  Eigen::Vector4d quat_min(std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity());
  Eigen::Vector4d quat_max(-std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity());

  csv << "t,x,y,z,w\n";

  for (int i = 0;; ++i) {
    double t = i * dt;
    if (t > t_end + 1e-9) {
      break;
    }

    const double s = std::sin(t);
    const double c = std::cos(t);
    const double d = 1.0 + s * s;

    const double x = 10.0 * c / d;
    const double y = 10.0 * s * c / d;
    const double z = 10.0;

    const double vx = -10.0 * s * (3.0 - s * s) / (d * d);
    const double vy = 10.0 * (1.0 - 3.0 * s * s) / (d * d);

    const double f = -10.0 * s * (3.0 - s * s);
    const double fp = -30.0 * c * c * c;
    const double ax = (fp * d - 2.0 * f * 2.0 * s * c) / (d * d * d);

    const double g_term = 10.0 * (1.0 - 3.0 * s * s);
    const double gp = -60.0 * s * c;
    const double ay = (gp * d - 2.0 * g_term * 2.0 * s * c) / (d * d * d);

    const Eigen::Vector3d a(ax, ay, 0.0);

    const double psi = std::atan2(vy, vx);
    const Eigen::Vector3d b1d(std::cos(psi), std::sin(psi), 0.0);

    Eigen::Vector3d b3 = (a + g * e3).normalized();
    Eigen::Vector3d b2 = b3.cross(b1d).normalized();
    Eigen::Vector3d b1 = b2.cross(b3).normalized();

    Eigen::Matrix3d R;
    R.col(0) = b1;
    R.col(1) = b2;
    R.col(2) = b3;

    Eigen::Quaterniond q(R);
    q.normalize();
    if (q.w() < 0.0) {
      q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
    }

    quat_min = quat_min.cwiseMin(Eigen::Vector4d(q.x(), q.y(), q.z(), q.w()));
    quat_max = quat_max.cwiseMax(Eigen::Vector4d(q.x(), q.y(), q.z(), q.w()));

    csv << std::fixed << std::setprecision(2) << t << ", "
        << std::setprecision(7) << q.x() << ", " << q.y() << ", " << q.z() << ", "
        << q.w() << "\n";
  }

  csv.close();

  std::ofstream txt(txt_path);
  if (!txt.is_open()) {
    std::cerr << "Failed to open TXT file: " << txt_path << "\n";
    return 1;
  }

  txt << "Quadrotor differential flatness (Q3)\n\n";
  txt << "Trajectory:\n";
  txt << "  x = 10 cos(t) / (1 + sin^2(t))\n";
  txt << "  y = 10 sin(t) cos(t) / (1 + sin^2(t))\n";
  txt << "  z = 10\n\n";
  txt << "Velocity and acceleration (analytic):\n";
  txt << "  vx = -10 sin(t) (3 - sin^2(t)) / (1 + sin^2(t))^2\n";
  txt << "  vy = 10 (1 - 3 sin^2(t)) / (1 + sin^2(t))^2\n";
  txt << "  ax = (fp * d - 4 f sin(t) cos(t)) / d^3, fp = -30 cos^3(t), f = -10 sin(t) (3 - sin^2(t))\n";
  txt << "  ay = (gp * d - 4 g sin(t) cos(t)) / d^3, gp = -60 sin(t) cos(t), g = 10 (1 - 3 sin^2(t))\n\n";
  txt << "Attitude construction (FLU):\n";
  txt << "  psi = atan2(vy, vx)\n";
  txt << "  b3 = (a + g e3) / ||a + g e3||\n";
  txt << "  b1d = [cos(psi), sin(psi), 0]^T\n";
  txt << "  b2 = (b3 x b1d) / ||b3 x b1d||\n";
  txt << "  b1 = b2 x b3\n";
  txt << "  R = [b1 b2 b3], q = quat(R), normalized with w >= 0\n\n";
  txt << "Output files:\n";
  txt << "  CSV: " << csv_path << "\n\n";
  txt << "Quaternion component ranges:\n";
  txt << "  min [x y z w] = [" << std::setprecision(7) << quat_min.x() << ", "
      << quat_min.y() << ", " << quat_min.z() << ", " << quat_min.w() << "]\n";
  txt << "  max [x y z w] = [" << quat_max.x() << ", " << quat_max.y() << ", "
      << quat_max.z() << ", " << quat_max.w() << "]\n";

  txt.close();

  std::cout << "Wrote outputs to: " << output_dir << "\n";
  std::cout << "CSV: " << csv_path << "\n";
  std::cout << "TXT: " << txt_path << "\n";

  return 0;
}
