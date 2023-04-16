#include <QMainWindow>
#include <QPixmap>
#include <QThread>
#include <QTimer>
#include <QtGui/QFont>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include <chrono>
#include <memory>
// uic -o untitled.h untitled.ui
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "untitled.h"

class RosControllerThread : public rclcpp::Node, protected QThread {
   public:
	RosControllerThread() : Node("cpp_robot_controller") {
		RCLCPP_INFO(this->get_logger(), "created");
		publisherTwist = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
		publisherSetPose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/set_pose", 1);
	}

	void ImageEventSetup(std::function<void(sensor_msgs::msg::Image::SharedPtr)> event) {
		subscriberImage = this->create_subscription<sensor_msgs::msg::Image>("/depth_camera/image_raw", 10, event);
	}

	void run() {
		rclcpp::spin(this_shared);
	}

	void Start(std::shared_ptr<RosControllerThread> shared) {
		this_shared = shared;
		start();
	}

	~RosControllerThread() {
		rclcpp::shutdown();
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherTwist;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisherSetPose;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriberImage;
	std::shared_ptr<RosControllerThread> this_shared;
};

class WindowManager : protected Ui_MainWindow {
   public:
	WindowManager(int argc, char** argv) {
		RosProcess(argc, argv);
		WindowProcess();
	}

	void WindowProcess() {
		setupUi(&window);

		QObject::connect(pushButton, &QPushButton::clicked, [this]() {
			// auto message = geometry_msgs::msg::Twist();
			// message.angular.z = 2.0;
			// ros->publisherTwist->publish(message);

			auto posMessage = geometry_msgs::msg::PoseWithCovarianceStamped();
			posMessage.pose.pose.position.x = 5;
			posMessage.pose.pose.position.y = 5;
			ros->publisherSetPose->publish(posMessage);
		});

		QObject::connect(pushButton_2, &QPushButton::clicked, [this]() {
			auto message = geometry_msgs::msg::Twist();
			ros->publisherTwist->publish(message);
		});

		window.show();
	}

	void ImageUpdate(const sensor_msgs::msg::Image::SharedPtr imageMsg) {
		QImage image(&imageMsg->data[0], imageMsg->width, imageMsg->height, imageMsg->step, QImage::Format_RGB888);
		label->setPixmap(QPixmap::fromImage(image));
	}

	void RosProcess(int argc, char** argv) {
		rclcpp::init(argc, argv);
		ros = std::make_shared<RosControllerThread>();
		ros->ImageEventSetup(std::bind(&WindowManager::ImageUpdate, this, std::placeholders::_1));
		ros->Start(ros);
	}

	~WindowManager() {
	}
	std::shared_ptr<RosControllerThread> ros;
	QMainWindow window;
};

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	WindowManager wm(argc, argv);
	return app.exec();
}

// using namespace std::chrono_literals;

// class MinimalPublisher : public rclcpp::Node {
//    public:
// 	MinimalPublisher()
// 	    : Node("cpp_topic_publisherTwist_spiral") {
// 		publisherTwist_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
// 		i = 0.0;
// 		timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::publish_message, this));
// 	}

//    private:
// 	void publish_message() {
// 		auto message = geometry_msgs::msg::Twist();
// 		message.linear.x = 4.0;
// 		message.angular.z = 2.0 + i;
// 		RCLCPP_INFO(this->get_logger(), "Sending - Linear Velocity : '%f', Angular Velocity : '%f'", message.linear.x, message.angular.z);
// 		publisherTwist_->publish(message);
// 		i += 0.1;
// 	}
// 	rclcpp::TimerBase::SharedPtr timer_;
// 	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherTwist_;
// 	float i;
// };

// class ControlWindow : public rclcpp::Node {
//    public:
// 	ControlWindow(int argc, char** argv) : Node("cpp_robot_move"), app_(argc, argv) {
// 		RCLCPP_INFO(this->get_logger(), "created");
// 		publisherTwist_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/vehicle_blue/cmd_vel", 1);
// 		subscriberImage_ = this->create_subscription<sensor_msgs::msg::Image>("/camera", 10, std::bind(&ControlWindow::ImageViwer, this, _1));
// 		WindowGui();
// 		// rclcpp::shutdown();
// 	}

//    private:
// 	void ImageViwer(const sensor_msgs::msg::Image::SharedPtr image) const {
// 		RCLCPP_INFO(this->get_logger(), "resim geldi");
// 		(void)image;
// 	}

// 	void WindowGui() {
// 		QWidget window;
// 		window.setWindowTitle("Hello, World!");
// 		window.setGeometry(100, 100, 400, 200);

// 		// QLabel label(&window);
// 		// label.setText("Hello, World!");
// 		// label.setFont(QFont("Arial", 30));

// 		QPushButton stopButton("stop turn!", &window);
// 		stopButton.setGeometry(150, 50, 100, 50);

// 		QPushButton turnButton("robot turn!", &window);
// 		turnButton.setGeometry(150, 100, 100, 50);

// 		// QObject::connect(&button, &QPushButton::clicked, [&label]() {
// 		// 	label.setText("Goodbye, World!");
// 		// });

// 		QObject::connect(&turnButton, &QPushButton::clicked, [this]() {
// 			auto message = geometry_msgs::msg::Twist();
// 			message.angular.z = 2.0;
// 			publisherTwist_->publish(message);
// 		});

// 		QObject::connect(&stopButton, &QPushButton::clicked, [this]() {
// 			publisherTwist_->publish(geometry_msgs::msg::Twist());
// 		});

// 		window.show();
// 		app_.exec();
// 	}

// 	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisherTwist_;
// 	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriberImage_;
// 	QApplication app_;
// };

// int main(int argc, char* argv[]) {
// 	rclcpp::init(argc, argv);
// 	rclcpp::spin(std::make_shared<ControlWindow>(argc, argv));
// 	rclcpp::shutdown();
// 	return 0;
// }

// int main(int argc, char* argv[]) {
// 	QApplication app(argc, argv);

// 	return app.exec();
// }

// ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"

/*
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("cpp_topic_subscriber_spiral")
    {
      subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1, std::bind(&MinimalSubscriber::subscribe_message, this, _1));
    }

  private:
    void subscribe_message(const geometry_msgs::msg::Twist::SharedPtr message) const
    {
      RCLCPP_INFO(this->get_logger(), "Recieved - Linear Velocity : '%f', Angular Velocity : '%f'", message->linear.x, message->angular.z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}*/