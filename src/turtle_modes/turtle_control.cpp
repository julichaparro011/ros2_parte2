#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "turtlesim/msg/pose.hpp"

#include "turtle_modes/srv/set_mode.hpp"
#include "turtle_modes/action/follow3_points.hpp"

using namespace std::chrono_literals;

class TurtleModeController : public rclcpp::Node
{
public:
  using SetMode = turtle_modes::srv::SetMode;
  using Follow3Points = turtle_modes::action::Follow3Points;
  using GoalHandleFollow3Points = rclcpp_action::ServerGoalHandle<Follow3Points>;

  TurtleModeController()
  : Node("turtle_mode_controller")
  {
    // Pub cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // Sub pose
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      [this](const turtlesim::msg::Pose::SharedPtr msg){
        last_pose_ = *msg;
        have_pose_ = true;
      }
    );

    // Pub modo actual
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/current_mode", 10);
    publish_mode_("MANUAL");

    // Servicio set_mode
    set_mode_srv_ = this->create_service<SetMode>(
      "/set_mode",
      std::bind(&TurtleModeController::on_set_mode, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Acción follow_3_points
    action_server_ = rclcpp_action::create_server<Follow3Points>(
      this,
      "/follow_3_points",
      std::bind(&TurtleModeController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TurtleModeController::handle_cancel, this, std::placeholders::_1),
      std::bind(&TurtleModeController::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "TurtleModeController listo. Servicio: /set_mode | Accion: /follow_3_points");
  }

private:
  enum Mode : uint8_t { MANUAL=0, CIRCLE=1, TRAJECTORY=2 };

  // ------------------ Utilidades ------------------
  static double wrap_pi(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void publish_stop_()
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    cmd_pub_->publish(t);
  }

  void publish_mode_(const std::string & name)
  {
    std_msgs::msg::String msg;
    msg.data = name;
    mode_pub_->publish(msg);
  }

  void switch_to_manual_(const std::string & reason)
  {
    // Detener círculo si estaba activo
    if (circle_timer_) circle_timer_.reset();

    // Si había acción corriendo, se aborta desde el hilo de ejecución (allá revisa flags)
    mode_.store(MANUAL);
    publish_stop_();
    publish_mode_("MANUAL");

    RCLCPP_INFO(get_logger(), "-> MANUAL (%s)", reason.c_str());
  }

  // ------------------ Servicio SetMode ------------------
  void on_set_mode(const std::shared_ptr<SetMode::Request> req,
                   std::shared_ptr<SetMode::Response> res)
  {
    uint8_t requested = req->mode;

    // Si cambian de modo mientras hay una trayectoria activa, marcamos preempt
    if (trajectory_active_.load() && requested != TRAJECTORY) {
      preempt_requested_.store(true);
    }

    if (requested == MANUAL) {
      switch_to_manual_("Servicio SetMode");
      res->ok = true;
      res->message = "Modo MANUAL activado";
      return;
    }

    if (requested == CIRCLE) {
      // Activar círculo
      mode_.store(CIRCLE);
      clockwise_.store(req->clockwise);

      // Importante: en círculo publicamos cmd_vel periódicamente
      if (!circle_timer_) {
        circle_timer_ = this->create_wall_timer(
          50ms, std::bind(&TurtleModeController::circle_step_, this)
        );
      }

      publish_mode_("CIRCLE");
      res->ok = true;
      res->message = std::string("Modo CIRCLE activado, sentido: ") + (req->clockwise ? "horario" : "antihorario");
      RCLCPP_INFO(get_logger(), "-> CIRCLE (%s)", req->clockwise ? "horario" : "antihorario");
      return;
    }

    if (requested == TRAJECTORY) {
      // Ojo: la trayectoria se arranca enviando un GOAL a la acción, no desde el servicio.
      mode_.store(TRAJECTORY);
      publish_mode_("TRAJECTORY");
      res->ok = true;
      res->message = "Modo TRAJECTORY activado. Envia un goal a /follow_3_points para iniciar.";
      RCLCPP_INFO(get_logger(), "-> TRAJECTORY (esperando goal de accion)");
      return;
    }

    res->ok = false;
    res->message = "Modo invalido. Use 0=MANUAL, 1=CIRCLE, 2=TRAJECTORY";
  }

  void circle_step_()
  {
    if (mode_.load() != CIRCLE) return;

    geometry_msgs::msg::Twist t;
    const double v = 1.5;          // puedes ajustar
    const double w = 1.0;          // puedes ajustar
    t.linear.x = v;
    t.angular.z = clockwise_.load() ? -w : +w;

    cmd_pub_->publish(t);
  }

  // ------------------ Acción Follow3Points ------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Follow3Points::Goal> goal)
  {
    (void)goal;
    // Aceptamos siempre; podrías validar aquí si no hay pose aún.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollow3Points> /*goal_handle*/)
  {
    // Permitimos cancelación
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollow3Points> goal_handle)
  {
    // Ejecutar en hilo aparte
    std::thread{std::bind(&TurtleModeController::execute_trajectory_, this, goal_handle)}.detach();
  }

  void execute_trajectory_(const std::shared_ptr<GoalHandleFollow3Points> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Follow3Points::Feedback>();
    auto result = std::make_shared<Follow3Points::Result>();

    // Forzamos modo trayectoria
    mode_.store(TRAJECTORY);
    publish_mode_("TRAJECTORY");

    trajectory_active_.store(true);
    preempt_requested_.store(false);

    // Esperar pose
    rclcpp::Rate wait_rate(20);
    while (rclcpp::ok() && !have_pose_) {
      wait_rate.sleep();
    }

    const double k_ang = (goal->angular_gain > 0.0f) ? goal->angular_gain : 4.0;
    const double v_cmd = (goal->linear_speed > 0.0f) ? goal->linear_speed : 1.5;
    const double tol  = (goal->pos_tolerance > 0.0f) ? goal->pos_tolerance : 0.2;

    rclcpp::Rate rate(30);

    for (int i = 0; i < 3; ++i) {
      const auto & p = goal->points[i];

      while (rclcpp::ok()) {
        // Cancelación
        if (goal_handle->is_canceling()) {
          publish_stop_();
          result->success = false;
          result->message = "Trayectoria cancelada por el usuario";
          goal_handle->canceled(result);

          trajectory_active_.store(false);
          switch_to_manual_("Accion cancelada");
          return;
        }

        // Cambio de modo externo (preempt)
        if (preempt_requested_.load()) {
          publish_stop_();
          result->success = false;
          result->message = "Trayectoria abortada: cambio de modo solicitado";
          goal_handle->abort(result);

          trajectory_active_.store(false);
          switch_to_manual_("Preempt (cambio de modo)");
          return;
        }

        // Si alguien dejó CIRCLE activo por error, igual aquí mandamos cmd_vel,
        // pero lo recomendado es NO correr teleop al mismo tiempo que autónomo.
        auto pose = last_pose_;

        double dx = p.x - pose.x;
        double dy = p.y - pose.y;
        double dist = std::sqrt(dx*dx + dy*dy);

        feedback->current_segment = static_cast<uint8_t>(i + 1);
        feedback->distance_to_goal = static_cast<float>(dist);
        goal_handle->publish_feedback(feedback);

        if (dist < tol) {
          // Llegó a este punto
          break;
        }

        double desired = std::atan2(dy, dx);
        double e_ang = wrap_pi(desired - pose.theta);

        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = k_ang * e_ang;

        // Solo avanza si está más o menos orientado
        if (std::fabs(e_ang) < 0.35) {
          // velocidad proporcional y limitada
          double v = std::min<double>(v_cmd, 1.0 * dist + 0.2);
          cmd.linear.x = v;
        } else {
          cmd.linear.x = 0.0;
        }

        cmd_pub_->publish(cmd);
        rate.sleep();
      }
    }

    // Finalizó los 3 puntos
    publish_stop_();
    result->success = true;
    result->message = "Trayectoria completada (3 puntos). Volviendo a MANUAL.";
    goal_handle->succeed(result);

    trajectory_active_.store(false);
    switch_to_manual_("Trayectoria terminada");
  }

  // ------------------ ROS objects ------------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_srv_;

  rclcpp_action::Server<Follow3Points>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr circle_timer_;

  // ------------------ Estado ------------------
  turtlesim::msg::Pose last_pose_;
  std::atomic<bool> have_pose_{false};

  std::atomic<uint8_t> mode_{MANUAL};
  std::atomic<bool> clockwise_{false};

  std::atomic<bool> trajectory_active_{false};
  std::atomic<bool> preempt_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleModeController>());
  rclcpp::shutdown();
  return 0;
}
