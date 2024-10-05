#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class PeriodicTaskExecutor : public rclcpp::Node
{
public:
    PeriodicTaskExecutor()
    : Node("periodic_task_node"),
      task_1_deadline_(rclcpp::Duration::from_seconds(1.0)),  // Deadline for Task 1 (1 second)
      task_2_deadline_(rclcpp::Duration::from_seconds(2.0)),  // Deadline for Task 2 (2 seconds)
      task_3_deadline_(rclcpp::Duration::from_seconds(5.0))   // Deadline for Task 3 (5 seconds)
    {
        // Declare the "use_sim_time" parameter if not already declared
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", false);
        }
        // Retrieve the "use_sim_time" parameter value
        this->get_parameter("use_sim_time", use_sim_time_);

        // Initialize the release times for all tasks
        task_1_release_time_ = this->now();
        task_2_release_time_ = this->now();
        task_3_release_time_ = this->now();

        // Set up Task 1 to run at 1 Hz (once per second)
        task_1_id_ = 1;
        task_1_counter_ = 0;
        task_1_timer_ = this->create_wall_timer(1s, std::bind(&PeriodicTaskExecutor::execute_task_1, this));

        // Set up Task 2 to run at 0.5 Hz (once every 2 seconds)
        task_2_id_ = 2;
        task_2_counter_ = 0;
        task_2_timer_ = this->create_wall_timer(2s, std::bind(&PeriodicTaskExecutor::execute_task_2, this));

        // Set up Task 3 to run at 0.2 Hz (once every 5 seconds)
        task_3_id_ = 3;
        task_3_counter_ = 0;
        task_3_timer_ = this->create_wall_timer(5s, std::bind(&PeriodicTaskExecutor::execute_task_3, this));
    }

private:
    rclcpp::Time task_1_release_time_, task_2_release_time_, task_3_release_time_;
    int task_1_id_, task_2_id_, task_3_id_;
    int task_1_counter_, task_2_counter_, task_3_counter_;
    rclcpp::Duration task_1_deadline_, task_2_deadline_, task_3_deadline_;
    bool use_sim_time_;
    rclcpp::TimerBase::SharedPtr task_1_timer_;
    rclcpp::TimerBase::SharedPtr task_2_timer_;
    rclcpp::TimerBase::SharedPtr task_3_timer_;

    void execute_task_1()
    {
        task_1_counter_++;
        auto end_time = this->now();
        auto task_1_response_time = end_time - task_1_release_time_;

        if (task_1_response_time > task_1_deadline_) {
            RCLCPP_WARN(this->get_logger(), "Task %d, Job %d missed its deadline!", task_1_id_, task_1_counter_);
        }

        log_execution(task_1_id_, task_1_counter_, task_1_release_time_, end_time);
        task_1_release_time_ = this->now();  // Update release time for the next job
    }

    void execute_task_2()
    {
        task_2_counter_++;
        auto end_time = this->now();
        auto task_2_response_time = end_time - task_2_release_time_;

        if (task_2_response_time > task_2_deadline_) {
            RCLCPP_WARN(this->get_logger(), "Task %d, Job %d missed its deadline!", task_2_id_, task_2_counter_);
        }

        log_execution(task_2_id_, task_2_counter_, task_2_release_time_, end_time);
        task_2_release_time_ = this->now();  // Update release time for the next job
    }

    void execute_task_3()
    {
        task_3_counter_++;
        auto end_time = this->now();
        auto task_3_response_time = end_time - task_3_release_time_;

        if (task_3_response_time > task_3_deadline_) {
            RCLCPP_WARN(this->get_logger(), "Task %d, Job %d missed its deadline!", task_3_id_, task_3_counter_);
        }

        log_execution(task_3_id_, task_3_counter_, task_3_release_time_, end_time);
        task_3_release_time_ = this->now();  // Update release time for the next job
    }

    void log_execution(int task_id, int job_num, rclcpp::Time start_time, rclcpp::Time end_time)
    {
        auto response_time = end_time - start_time;
        RCLCPP_INFO(this->get_logger(), 
                    "Task %d, Job %d, Started at %.4f, Finished at %.4f, Response Time: %.4f seconds",
                    task_id, job_num, start_time.seconds(), end_time.seconds(), response_time.seconds());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PeriodicTaskExecutor>());
    rclcpp::shutdown();
    return 0;
}
