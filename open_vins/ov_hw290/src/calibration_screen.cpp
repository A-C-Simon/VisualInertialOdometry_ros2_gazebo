#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QProcess>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <array>
#include <cmath>
#include <csignal>
#include <deque>
#include <set>
#include <unistd.h>

// All ROS callbacks and widget updates run on the Qt thread.
class CalibrationScreen : public QWidget {
 public:
  explicit CalibrationScreen(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    executor_.add_node(node_);
    clock_.start();
    const auto target = node_->declare_parameter<std::string>("target_image", "");
    target_.load(QString::fromStdString(target));
    if (target_.isNull()) throw std::runtime_error("Cannot load target_image");
    image_path_ = QString::fromStdString(target);
    output_root_ = QString::fromStdString(node_->declare_parameter<std::string>("output_root", "/tmp"));
    seconds_ = node_->declare_parameter<int>("record_seconds", 90);
    tag_size_ = node_->declare_parameter<double>("tag_size_m", 0.040);
    if (seconds_ < 1 || seconds_ > 600 || tag_size_ <= 0)
      throw std::runtime_error("Invalid recording duration or measured tag size");
    setWindowTitle("HW290 screen calibration");
    setStyleSheet("background: #222; color: white;");
    panel_ = new QWidget(this);
    panel_->setFixedWidth(420);
    auto layout = new QVBoxLayout(panel_);
    auto previews = new QHBoxLayout;
    for (auto &label : views_) {
      label = new QLabel("Waiting for camera", panel_);
      label->setFixedSize(190, 145);
      label->setAlignment(Qt::AlignCenter);
      previews->addWidget(label);
    }
    layout->addLayout(previews);
    status_ = new QLabel(panel_);
    status_->setWordWrap(true);
    layout->addWidget(status_);
    auto help = new QLabel("Measure a complete black tag edge at this display size. Keep the grid visible in both previews. Esc returns to the chat and stops an active recording.", panel_);
    help->setWordWrap(true);
    layout->addWidget(help);
    auto measured = new QDoubleSpinBox(panel_);
    measured->setRange(1.0, 200.0);
    measured->setDecimals(1);
    measured->setSingleStep(0.1);
    measured->setPrefix("Measured tag edge: ");
    measured->setSuffix(" mm");
    measured->setValue(tag_size_ * 1000.0);
    layout->addWidget(measured);
    connect(measured, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double mm) { tag_size_ = mm / 1000.0; });
    prepare_ = new QPushButton(QString("Prepare %1 second recording").arg(seconds_), panel_);
    layout->addWidget(prepare_);
    connect(prepare_, &QPushButton::clicked, this, [this, measured] {
      armed_ = true;
      stable_since_ = -1;
      prepare_->setEnabled(false);
      measured->setEnabled(false);
    });
    recorder_.setProcessChannelMode(QProcess::MergedChannels);
    connect(&recorder_, &QProcess::readyReadStandardOutput, this, [this] {
      recorder_log_ += recorder_.readAllStandardOutput();
    });
    connect(&recorder_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
            [this](int, QProcess::ExitStatus) {
      writeLog();
      if (!finishing_) {
        failed_ = true;
        border(false, "Recorder stopped unexpectedly. Return to chat to inspect the log.");
      }
    });
    for (int camera = 0; camera < 2; ++camera) {
      subscriptions_[camera] = node_->create_subscription<sensor_msgs::msg::Image>(
          "/cam" + std::to_string(camera) + "/image_raw", rclcpp::SensorDataQoS(),
          [this, camera](sensor_msgs::msg::Image::ConstSharedPtr message) {
        if (message->encoding != "mono8" || message->width == 0 || message->height == 0 ||
            message->step < message->width || message->data.size() < message->step * size_t(message->height)) return;
        cv::Mat image(message->height, message->width, CV_8UC1,
                      const_cast<uint8_t*>(message->data.data()), message->step);
        images_[camera] = image.clone();
        image_times_[camera] = clock_.elapsed();
        stamps_[camera] = message->header.stamp.sec + 1e-9 * message->header.stamp.nanosec;
      });
    }
    imu_ = node_->create_subscription<sensor_msgs::msg::Imu>("/imu0", rclcpp::SensorDataQoS().keep_last(200),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
      const auto &a = message->linear_acceleration;
      const auto &g = message->angular_velocity;
      if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
          !std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) return;
      imu_times_.push_back(clock_.elapsed());
    });
    connect(&timer_, &QTimer::timeout, this, [this] { tick(); });
    timer_.start(50);
    border(false, "RED: hold the rig steady. Waiting for camera and IMU.");
  }

  ~CalibrationScreen() override { stopRecorder(); }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);
    const int side = std::min(height(), width() - 460);
    painter.fillRect(rect(), QColor("#222"));
    if (side > 0) painter.drawPixmap(QRect(0, (height() - side) / 2, side, side), target_);
  }
  void resizeEvent(QResizeEvent*) override {
    panel_->move(width() - panel_->width() - 10, 10);
  }
  void keyPressEvent(QKeyEvent *event) override {
    if (event->key() == Qt::Key_Escape) close();
    else QWidget::keyPressEvent(event);
  }

 private:
  void border(bool green, const QString &text) {
    const auto color = green ? "#16c94b" : "#f43b3b";
    for (auto label : views_) label->setStyleSheet(QString("border: 5px solid %1; background: black;").arg(color));
    status_->setText(text);
  }
  void writeLog() {
    if (run_dir_.isEmpty()) return;
    QFile file(run_dir_ + "/recorder.log");
    if (file.open(QIODevice::WriteOnly)) file.write(recorder_log_);
  }
  void stopRecorder() {
    finishing_ = true;
    if (recorder_.state() != QProcess::NotRunning) {
      ::kill(recorder_.processId(), SIGINT);
      if (!recorder_.waitForFinished(5000)) {
        recorder_.terminate();
        if (!recorder_.waitForFinished(2000)) {
          recorder_.kill();
          recorder_.waitForFinished(2000);
        }
      }
    }
    if (recorder_.isOpen()) recorder_log_ += recorder_.readAllStandardOutput();
    writeLog();
  }
  void startRecorder() {
    run_dir_ = output_root_ + "/screen_calibration_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    if (!QDir().mkpath(run_dir_)) {
      failed_ = true;
      border(false, "Cannot create the recording directory.");
      return;
    }
    QFile::copy(image_path_, run_dir_ + "/displayed_target.png");
    QFile yaml(run_dir_ + "/target.yaml");
    if (!yaml.open(QIODevice::WriteOnly)) {
      failed_ = true;
      border(false, "Cannot save measured target dimensions.");
      return;
    }
    yaml.write(QString("target_type: aprilgrid\ntagCols: 6\ntagRows: 6\ntagSize: %1\ntagSpacing: 0.3\n")
                   .arg(tag_size_, 0, 'g', 10).toUtf8());
    yaml.close();
    recorder_.start("ros2", {"bag", "record", "-o", run_dir_ + "/sensors_bag",
                    "/cam0/image_raw", "/cam1/image_raw", "/cam0/camera_info", "/cam1/camera_info", "/imu0"});
    recorder_start_ = clock_.elapsed();
  }
  void tick() {
    if (!rclcpp::ok()) { close(); return; }
    // Drain all queued callbacks. One spin_some call per 50 ms GUI tick can
    // process only one sample per subscription and falsely report a 20 Hz IMU.
    executor_.spin_all(std::chrono::milliseconds(5));
    const auto now = clock_.elapsed();
    while (!imu_times_.empty() && now - imu_times_.front() > 2000) imu_times_.pop_front();
    const bool streams = image_times_[0] >= 0 && image_times_[1] >= 0 &&
        now - image_times_[0] < 800 && now - image_times_[1] < 800 &&
        !imu_times_.empty() && now - imu_times_.back() < 500;
    const double rate = imu_times_.size() >= 2 ?
        (imu_times_.size() - 1) * 1000.0 / std::max<qint64>(1, imu_times_.back() - imu_times_.front()) : 0;
    if (now - last_detection_ >= 200) {
      last_detection_ = now;
      std::array<std::set<int>, 2> ids;
      for (int camera = 0; camera < 2; ++camera) {
        if (images_[camera].empty()) continue;
        std::vector<int> found;
        std::vector<std::vector<cv::Point2f>> corners;
        auto parameters = cv::aruco::DetectorParameters::create();
        parameters->markerBorderBits = 2;  // Kalibr's generated target has a two-bit black border.
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
        cv::aruco::detectMarkers(images_[camera], dictionary_, corners, found, parameters);
        if (found.size() < 7) {
          // OpenCV 4.5's AprilTag quad search misses some dim, distorted views.
          // Keep the raw recording intact; this fallback is for preview only.
          parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
          cv::aruco::detectMarkers(images_[camera], dictionary_, corners, found, parameters);
          if (found.size() < 7) {
            cv::Mat contrast;
            cv::createCLAHE(2.0)->apply(images_[camera], contrast);
            cv::aruco::detectMarkers(contrast, dictionary_, corners, found, parameters);
          }
        }
        for (int id : found) if (id >= 0 && id < 36) ids[camera].insert(id);
        tag_counts_[camera] = ids[camera].size();
        cv::Mat preview;
        cv::cvtColor(images_[camera], preview, cv::COLOR_GRAY2RGB);
        if (!found.empty()) cv::aruco::drawDetectedMarkers(preview, corners, found);
        QImage qt(preview.data, preview.cols, preview.rows, preview.step, QImage::Format_RGB888);
        views_[camera]->setPixmap(QPixmap::fromImage(qt.copy()).scaled(180, 135, Qt::KeepAspectRatio));
      }
      common_tags_ = 0;
      for (int id : ids[0]) if (ids[1].count(id)) ++common_tags_;
    }
    if (failed_ || finishing_) return;
    const bool ready = streams && rate >= 80 && rate <= 120 &&
        std::abs(stamps_[0] - stamps_[1]) < 0.005 &&
        tag_counts_[0] >= 7 && tag_counts_[1] >= 7 && common_tags_ >= 4;
    if (now - last_log_ >= 3000) {
      last_log_ = now;
      RCLCPP_INFO(node_->get_logger(), "preview: common_tags=%d imu_rate=%.1f streams=%s ready=%s",
                  common_tags_, rate, streams ? "yes" : "no", ready ? "yes" : "no");
    }
    if (recording_since_ >= 0) {
      if (!streams) {
        failed_ = true;
        stopRecorder();
        border(false, "RED: sensor data stopped. Recording saved but incomplete. Return to chat.");
        showNormal();
        return;
      }
      const int left = seconds_ - int((now - recording_since_) / 1000);
      if (left <= 0) {
        stopRecorder();
        border(false, "FINISHED: stop moving. Saved in " + run_dir_ + ". Return to chat.");
        showNormal();
      } else {
        border(true, QString("GREEN: move slowly through different distances, positions and tilts. %1 seconds left. %2 common tags, IMU %3 Hz.%4")
            .arg(left).arg(common_tags_).arg(rate, 0, 'f', 1)
            .arg(common_tags_ < 4 ? " Bring the full grid back into view." : ""));
      }
      return;
    }
    if (recorder_start_ >= 0) {
      const bool subscribed = recorder_log_.contains("Subscribed to topic '/cam0/image_raw'") &&
          recorder_log_.contains("Subscribed to topic '/cam1/image_raw'") &&
          recorder_log_.contains("Subscribed to topic '/imu0'");
      if (now - recorder_start_ > 20000 && !subscribed) {
        failed_ = true;
        stopRecorder();
        border(false, "RED: recorder did not subscribe to the sensors. Return to chat.");
      } else if (subscribed && ready) {
        if (countdown_since_ < 0) countdown_since_ = now;
        const int remaining = 5 - int((now - countdown_since_) / 1000);
        if (remaining <= 0) recording_since_ = now;
        border(false, QString("RED: hold steady. Start moving when green, in %1 seconds.").arg(std::max(0, remaining)));
      } else {
        countdown_since_ = -1;
        border(false, "RED: hold steady with the grid visible. Waiting for recorder and sensors.");
      }
      return;
    }
    border(false, QString("RED: left %1 tags, right %2 tags, IMU %3 Hz. %4")
        .arg(tag_counts_[0]).arg(tag_counts_[1]).arg(rate, 0, 'f', 1)
        .arg(tag_counts_[0] < 7 || tag_counts_[1] < 7 ?
             "Aim straight at the grid and move closer until at least 7 tags appear in each view. Keep the complete grid visible." :
             (armed_ ? "Hold steady while I check the image." :
              QString("Click Prepare; hold still until green, then move for %1 seconds.").arg(seconds_))));
    if (armed_ && ready) {
      if (stable_since_ < 0) stable_since_ = now;
      if (now - stable_since_ >= 3000) startRecorder();
    } else stable_since_ = -1;
  }
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 2> subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  std::array<cv::Mat, 2> images_;
  std::array<qint64, 2> image_times_{{-1, -1}};
  std::array<double, 2> stamps_{{0, 0}};
  std::array<int, 2> tag_counts_{{0, 0}};
  std::deque<qint64> imu_times_;
  QWidget *panel_;
  std::array<QLabel*, 2> views_;
  QLabel *status_;
  QPushButton *prepare_;
  QPixmap target_;
  QString image_path_, output_root_, run_dir_;
  QByteArray recorder_log_;
  QProcess recorder_;
  QElapsedTimer clock_;
  QTimer timer_;
  qint64 last_detection_ = -1000, last_log_ = -3000, stable_since_ = -1, recorder_start_ = -1,
         countdown_since_ = -1, recording_since_ = -1;
  int seconds_ = 90, common_tags_ = 0;
  double tag_size_ = 0.040;
  bool armed_ = false, finishing_ = false, failed_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("hw290_calibration_screen");
    CalibrationScreen window(node);
    window.showFullScreen();
    QTimer::singleShot(500, &window, [&window] {
      window.raise();
      window.activateWindow();
    });
    const int result = app.exec();
    rclcpp::shutdown();
    return result;
  } catch (const std::exception &error) {
    fprintf(stderr, "Calibration screen: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
