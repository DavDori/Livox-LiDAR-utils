#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cmath>  // for M_PI or define your own DEG2RAD
#include <functional>
#include <memory>
#include <random>  // for std::random_device, std::normal_distribution, std::generate_canonical
#include <string>

#include "livox_lidar_utils/pcg_random.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG2RAD(deg) ((deg) * static_cast<float>(M_PI) / 180.0f)

class LivoxRenoise : public rclcpp::Node {
 public:
  LivoxRenoise() : Node("livox_renoise_node") {
    declare_parameter("topic.in", "/livox/lidar");
    declare_parameter("topic.out", "/livox/lidar_ds");
    declare_parameter("sigma.range_m", 0.1);
    declare_parameter("sigma.angle_deg", 0.5);

    topic_in_ = get_parameter("topic.in").as_string();
    topic_out_ = get_parameter("topic.out").as_string();
    sigma_range_ = get_parameter("sigma.range_m").as_double();
    sigma_angle_ = DEG2RAD(get_parameter("sigma.angle_deg").as_double());

    cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        topic_in_, 10,
        std::bind(&LivoxRenoise::pointcloudCallback, this,
                  std::placeholders::_1));

    cloud_pub_ =
        create_publisher<livox_ros_driver2::msg::CustomMsg>(topic_out_, 10);

    std::random_device rd;
    uint64_t seed = static_cast<uint64_t>(rd()) << 32 | rd();
    rng_ = pcg32_fast(seed);

    dist_range_ = std::normal_distribution<double>(0.0, sigma_range_);
    dist_angle_ = std::normal_distribution<double>(0.0, sigma_angle_);

    RCLCPP_INFO(this->get_logger(), "LivoxRenoise initialized.");
    printParams();
  }

 private:
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_sub_;
  std::string topic_in_, topic_out_;
  double sigma_range_, sigma_angle_;
  pcg32_fast rng_;  // Fast 32-bit PCG engine
  std::normal_distribution<double> dist_range_, dist_angle_;

  inline void fill_normal_array(Eigen::ArrayXf &out, float sigma,
                                pcg32_fast &rng) {
    const size_t N = out.size();
    for (size_t i = 0; i < N; i += 2) {
      float u1 = std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);
      float u2 = std::uniform_real_distribution<float>(0.0f, 1.0f)(rng);

      const float r = std::sqrt(-2.0f * std::log(u1)) * sigma;
      const float theta = 2.0f * static_cast<float>(M_PI) * u2;

      out(i) = r * std::cos(theta);
      if (i + 1 < N) out(i + 1) = r * std::sin(theta);
    }
  }

  void pointcloudCallback(
      const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    using namespace Eigen;
    const size_t N = msg->points.size();
    if (N == 0) return;

    // Extract xyz
    MatrixXf xyz(3, N);
    for (size_t i = 0; i < N; ++i) {
      xyz(0, i) = msg->points[i].x;
      xyz(1, i) = msg->points[i].y;
      xyz(2, i) = msg->points[i].z;
    }

    ArrayXf r = xyz.colwise().norm().array();

    ArrayXf azim(N), elev(N);
    for (size_t i = 0; i < N; ++i) {
      azim(i) = std::atan2(xyz(1, i), xyz(0, i));
      elev(i) = std::asin(std::clamp(xyz(2, i) / r(i), -1.0f, 1.0f));
    }

    // Fill noise using PCG + Box-Muller
    ArrayXf noise_r(N), noise_azim(N), noise_elev(N);
    fill_normal_array(noise_r, static_cast<float>(sigma_range_), rng_);
    fill_normal_array(noise_azim, static_cast<float>(sigma_angle_), rng_);
    fill_normal_array(noise_elev, static_cast<float>(sigma_angle_), rng_);

    ArrayXf r_noisy = r + noise_r;
    ArrayXf azim_noisy = azim + noise_azim;
    ArrayXf elev_noisy = elev + noise_elev;

    ArrayXf cos_elev = elev_noisy.cos();
    ArrayXf x = r_noisy * cos_elev * azim_noisy.cos();
    ArrayXf y = r_noisy * cos_elev * azim_noisy.sin();
    ArrayXf z = r_noisy * elev_noisy.sin();

    // Rebuild CustomMsg
    livox_ros_driver2::msg::CustomMsg noisy_msg;
    noisy_msg.header = msg->header;
    noisy_msg.timebase = msg->timebase;
    noisy_msg.point_num = msg->point_num;
    noisy_msg.lidar_id = msg->lidar_id;
    noisy_msg.rsvd = msg->rsvd;
    noisy_msg.points.reserve(N);

    for (size_t i = 0; i < N; ++i) {
      livox_ros_driver2::msg::CustomPoint noisy_pt = msg->points[i];
      noisy_pt.x = x(i);
      noisy_pt.y = y(i);
      noisy_pt.z = z(i);
      noisy_msg.points.emplace_back(std::move(noisy_pt));
    }

    cloud_pub_->publish(std::move(noisy_msg));
  }

  void printParams() {
    std::ostringstream general_params;
    general_params << "\n--- Parameters ---\n"
                   << "  - CustomMsg input topic: " << topic_in_ << "\n"
                   << "  - CustomMsg output topic: " << topic_out_ << "\n"
                   << "  - Sigma range (m): " << sigma_range_ << "\n"
                   << "  - Sigma angle (rad): " << sigma_angle_ << "\n";
    RCLCPP_INFO(this->get_logger(), "%s", general_params.str().c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxRenoise>());
  rclcpp::shutdown();
  return 0;
}
