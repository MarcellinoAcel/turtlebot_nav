#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "turtle_nav2pos/convertion.hpp"
#include "irobot_create_msgs/msg/audio_note_vector.hpp"

using std::placeholders::_1;

class Bv_nav : public rclcpp::Node
{
public:
	Bv_nav() : Node("bv_nav")
	{
		client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
			this, "navigate_to_pose");

		audio_pub_ = this->create_publisher<irobot_create_msgs::msg::AudioNoteVector>(
			"/cmd_audio", 10);

		// Example: start the two-goal sequence after 2 seconds
		timer_ = this->create_wall_timer(
			std::chrono::seconds(2),
			std::bind(&Bv_nav::start_two_goals, this));

		RCLCPP_INFO(this->get_logger(), "System Ready");
	}

	// ----------------------------------------------------------------------
	// START SEQUENCE (replace this with your own condition)
	// ----------------------------------------------------------------------
	void start_two_goals()
	{
		timer_->cancel(); // run only once

		current_goal_id = 1;
		second_goal_pending = true;

		RCLCPP_INFO(this->get_logger(), "Starting goal sequence...");

		send_goal(24.29, -7.64, conv.toRad(0)); // FIRST GOAL
	}

	// ----------------------------------------------------------------------
	// SEND GOAL
	// ----------------------------------------------------------------------
	void send_goal(double x, double y, double theta)
	{
		if (!this->client_ptr_->wait_for_action_server())
		{
			RCLCPP_ERROR(this->get_logger(), "Action server not available");
			return;
		}

		auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
		goal_msg.pose.pose.position.x = x;
		goal_msg.pose.pose.position.y = y;
		goal_msg.pose.pose.orientation.z = theta;
		goal_msg.pose.header.frame_id = "map";

		auto send_goal_options =
			rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
		send_goal_options.goal_response_callback =
			std::bind(&Bv_nav::goal_response_callback, this, _1);
		send_goal_options.result_callback =
			std::bind(&Bv_nav::get_result_callback, this, _1);

		client_ptr_->async_send_goal(goal_msg, send_goal_options);

		RCLCPP_INFO(this->get_logger(),
					"Sending goal: x=%.2f, y=%.2f, θ=%.2f deg",
					x, y, conv.toDeg(theta));
	}

private:
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
	rclcpp::Publisher<irobot_create_msgs::msg::AudioNoteVector>::SharedPtr audio_pub_;

	Convertion conv;

	int current_goal_id = 0;
	bool second_goal_pending = false;

	rclcpp::TimerBase::SharedPtr timer_;

	// ----------------------------------------------------------------------
	// BEEP FUNCTION
	// ----------------------------------------------------------------------
	void play_beep()
	{
		irobot_create_msgs::msg::AudioNoteVector beep_msg;
		irobot_create_msgs::msg::AudioNote note;

		note.frequency = 500;
		note.max_runtime.sec = 1;
		note.max_runtime.nanosec = 0;

		beep_msg.notes.push_back(note);
		audio_pub_->publish(beep_msg);

		RCLCPP_INFO(this->get_logger(), "Beep!");
	}

	// ----------------------------------------------------------------------
	// GOAL CALLBACKS
	// ----------------------------------------------------------------------
	void goal_response_callback(
		const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
	{
		if (!goal_handle)
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
		else
			RCLCPP_INFO(this->get_logger(), "Goal accepted");
	}

	void get_result_callback(
		const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
	{
		if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
		{
			RCLCPP_ERROR(this->get_logger(), "Goal failed or canceled");
			return;
		}

		RCLCPP_INFO(this->get_logger(), "Goal succeeded");

		if (current_goal_id == 1)
		{
			// First goal → 1 beep
			play_beep();
			RCLCPP_INFO(this->get_logger(), "Finished Goal 1 (1 beep)");

			if (second_goal_pending)
			{
				second_goal_pending = false;
				current_goal_id = 2;

				send_goal(-8.9, 9.2, conv.toRad(0)); // SECOND GOAL
			}
		}
		else if (current_goal_id == 2)
		{
			// Second goal → 2 beeps
			play_beep();
			play_beep();
			RCLCPP_INFO(this->get_logger(), "Finished Goal 2 (2 beeps)");
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Bv_nav>());
	rclcpp::shutdown();
	return 0;
}
