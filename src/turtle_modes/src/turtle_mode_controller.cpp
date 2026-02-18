// Juliana Chaparro Álvarez y Jerson Nicolás Guzmán Muñoz
// -----------------------------------------------------------
// Este programa en C++ permite controlar turtlesim con 3 modos:
//
// 0) MANUAL:
//    - No publica cmd_vel (para que teleop/rqt tengan el control)
//    - Solo detiene la tortuga al entrar en MANUAL
//
// 1) CIRCLE:
//    - Publica cmd_vel periódicamente (timer) para describir un círculo
//    - Permite dirección: horario (-1) / antihorario (+1)
//
// 2) TRAJECTORY:
//    - Acción cancelable para ir a 3 puntos 
//    - Al finalizar o cancelar, vuelve a MANUAL
//
// Interfaces usadas (ya compiladas en turtle_modes_interfaces):
//   - srv/SetMode.srv
//   - srv/GetMode.srv
//   - action/FollowTrajectory.action
// ---------------------------------------------------------

#include <cmath>
#include <chrono>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#include "turtle_modes_interfaces/srv/set_mode.hpp"
#include "turtle_modes_interfaces/srv/get_mode.hpp"
#include "turtle_modes_interfaces/action/follow_trajectory.hpp"

using namespace std::chrono_literals;

class TurtleModeController : public rclcpp::Node
{
public:
  using SetMode = turtle_modes_interfaces::srv::SetMode;
  using GetMode = turtle_modes_interfaces::srv::GetMode;
  using FollowTrajectory = turtle_modes_interfaces::action::FollowTrajectory;
  using GoalHandleFollow = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

  // Modos del sistema
  enum Mode : uint8_t { MANUAL = 0, CIRCLE = 1, TRAJECTORY = 2 };

  TurtleModeController()
  : Node("turtle_mode_controller"),
    mode_(MANUAL),
    circle_dir_(+1),
    stop_requested_(false),
    pose_received_(false)
  {
    
    this->declare_parameter<double>("circle_linear", 2.0);   // velocidad lineal para el círculo
    this->declare_parameter<double>("circle_angular", 2.0);  // velocidad angular para el círculo
    this->declare_parameter<double>("ctrl_rate_hz", 30.0);   // frecuencia control trayectoria
    this->declare_parameter<double>("linear_gain", 1.5);     // ganancia lineal (trayectoria)
    this->declare_parameter<double>("angular_gain", 6.0);    // ganancia angular (trayectoria)
    this->declare_parameter<double>("pos_tol", 0.15);        // tolerancia para "llegué al punto"

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      [this](const turtlesim::msg::Pose::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(pose_mtx_);
        pose_ = *msg;
        pose_received_ = true;
      });

    // -------------------------
    // Servicios
    // -------------------------
    set_mode_srv_ = this->create_service<SetMode>(
      "/turtle_modes/set_mode",
      std::bind(&TurtleModeController::on_set_mode, this,
                std::placeholders::_1, std::placeholders::_2));

    get_mode_srv_ = this->create_service<GetMode>(
      "/turtle_modes/get_mode",
      std::bind(&TurtleModeController::on_get_mode, this,
                std::placeholders::_1, std::placeholders::_2));

    // -------------------------
    // Action server (trayectoria cancelable)
    // -------------------------
    traj_action_server_ = rclcpp_action::create_server<FollowTrajectory>(
      this,
      "/turtle_modes/follow_trajectory",
      std::bind(&TurtleModeController::handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&TurtleModeController::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&TurtleModeController::handle_accepted, this,
                std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Listo. Servicios: /turtle_modes/set_mode, /turtle_modes/get_mode | Action: /turtle_modes/follow_trajectory");
  }

private:

  static double norm_angle(double a)
  {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    cmd_pub_->publish(t);
  }

 
  turtlesim::msg::Pose get_pose_copy()
  {
    std::lock_guard<std::mutex> lk(pose_mtx_);
    return pose_;
  }

  // Detiene cualquier "modo activo" (timer del círculo, acción de trayectoria, etc.)
  void stop_any_active_mode()
  {
    // Señal para abortar/terminar loops de trayectoria si están corriendo
    stop_requested_.store(true);

    // Si el círculo está activo, cancelamos su timer
    if (circle_timer_) {
      circle_timer_->cancel();
      circle_timer_.reset();
    }

    // Stop “físico” inmediato
    publish_stop();
  }

  // -----------------------------------------------------------
  // Servicio: SetMode
  // -----------------------------------------------------------
  void on_set_mode(const std::shared_ptr<SetMode::Request> req,
                   std::shared_ptr<SetMode::Response> res)
  {
    // Validación básica del modo
    if (req->mode > 2) {
      res->ok = false;
      res->message = "Modo invalido. Usa 0=MANUAL, 1=CIRCLE, 2=TRAJECTORY";
      return;
    }

    // Antes de cambiar, detenemos lo que esté corriendo
    stop_any_active_mode();

    // Actualiza dirección del círculo si el usuario envía algo distinto de 0
    if (req->circle_dir != 0) {
      circle_dir_ = (req->circle_dir < 0) ? -1 : +1;
    }

    mode_ = static_cast<Mode>(req->mode);

    // --------- MODO MANUAL ---------
    if (mode_ == MANUAL) {
      // En manual NO publicamos cmd_vel continuamente.
      // El usuario controla con teleop o rqt.
      res->ok = true;
      res->message = "Modo MANUAL activo (usa teleop/rqt).";
      return;
    }

    // --------- MODO CIRCLE ---------
    if (mode_ == CIRCLE) {
      // Limpia el stop_requested porque vamos a iniciar el lazo del círculo
      stop_requested_.store(false);

      const double v = this->get_parameter("circle_linear").as_double();
      const double w = this->get_parameter("circle_angular").as_double();

      // Timer que publica cmd_vel cada 50 ms
      circle_timer_ = this->create_wall_timer(
        50ms,
        [this, v, w]()
        {
          // Si el modo cambió, no hacemos nada
          if (mode_ != CIRCLE) return;

          geometry_msgs::msg::Twist t;
          t.linear.x = v;
          t.angular.z = static_cast<double>(circle_dir_) * w;
          cmd_pub_->publish(t);
        });

      res->ok = true;
      res->message = (circle_dir_ > 0) ? "Modo CIRCLE antihorario activo." : "Modo CIRCLE horario activo.";
      return;
    }

    // --------- MODO TRAJECTORY ---------
    // En trajectory: se ejecuta mediante Action.
    // Aquí solo dejamos el modo seleccionado.
    res->ok = true;
    res->message = "Modo TRAJECTORY seleccionado. Envia goal a /turtle_modes/follow_trajectory (start=true).";
  }

  // -----------------------------------------------------------
  // Servicio: GetMode
  // -----------------------------------------------------------
  void on_get_mode(const std::shared_ptr<GetMode::Request>,
                   std::shared_ptr<GetMode::Response> res)
  {
    (void)res;
    res->mode = static_cast<uint8_t>(mode_);
    res->circle_dir = static_cast<int8_t>(circle_dir_);
  }

  // -----------------------------------------------------------
  // Action server: FollowTrajectory (goal / cancel / execute)
  // -----------------------------------------------------------

  // Decide si acepta el goal
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowTrajectory::Goal> goal)
  {
    // Rechazar si aún no hay pose (turtlesim no ha publicado /turtle1/pose)
    if (!pose_received_) {
      RCLCPP_WARN(get_logger(), "Aun no recibo /turtle1/pose. Espera y vuelve a enviar el goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Rechazar si goal no inicia
    if (!goal->start) {
      RCLCPP_WARN(get_logger(), "Goal recibido con start=false. Rechazado.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Decide si acepta la cancelación
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollow>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Cuando se acepta, ejecutamos en un hilo (no bloquear executor)
  void handle_accepted(const std::shared_ptr<GoalHandleFollow> goal_handle)
  {
    std::thread([this, goal_handle]() { execute_trajectory(goal_handle); }).detach();
  }

  // Ejecución real del movimiento por 3 puntos
  void execute_trajectory(const std::shared_ptr<GoalHandleFollow> goal_handle)
  {
    // Cambiar a modo trayectoria y habilitar ejecución
    stop_requested_.store(false);
    mode_ = TRAJECTORY;

    auto result = std::make_shared<FollowTrajectory::Result>();
    auto feedback = std::make_shared<FollowTrajectory::Feedback>();

    // -------------------------
    // Define 3 waypoints (EDITA AQUÍ si quieres)
    // -------------------------
    // Coordenadas típicas turtlesim: aprox [0..11]
    std::vector<std::pair<double,double>> waypoints = {
      {2.0, 2.0},
      {9.0, 2.0},
      {9.0, 9.0}
    };

    // Lee parámetros del controlador
    const double rate_hz = this->get_parameter("ctrl_rate_hz").as_double();
    const double k_lin  = this->get_parameter("linear_gain").as_double();
    const double k_ang  = this->get_parameter("angular_gain").as_double();
    const double tol    = this->get_parameter("pos_tol").as_double();

    rclcpp::Rate rate(rate_hz);

    // Recorre cada waypoint
    for (size_t i = 0; i < waypoints.size() && rclcpp::ok(); ++i) {

      const double gx = waypoints[i].first;
      const double gy = waypoints[i].second;

      // Control hasta llegar al punto i
      while (rclcpp::ok()) {

        // 1) Cancelación del usuario
        if (goal_handle->is_canceling()) {
          publish_stop();
          result->success = false;
          result->message = "Trayectoria cancelada por el usuario.";
          goal_handle->canceled(result);
          mode_ = MANUAL;
          return;
        }

        // 2) Abortado por cambio de modo (servicio set_mode)
        if (stop_requested_.load()) {
          publish_stop();
          result->success = false;
          result->message = "Trayectoria abortada por cambio de modo.";
          goal_handle->abort(result);
          mode_ = MANUAL;
          return;
        }

        // Pose actual
        auto p = get_pose_copy();

        // Error en posición
        const double dx = gx - p.x;
        const double dy = gy - p.y;
        const double dist = std::sqrt(dx*dx + dy*dy);

        // Feedback (progreso)
        feedback->current_waypoint = static_cast<uint32_t>(i + 1);
        feedback->distance_to_goal = static_cast<float>(dist);
        goal_handle->publish_feedback(feedback);

        // Condición de llegada
        if (dist < tol) {
          publish_stop();
          break; // pasa al siguiente waypoint
        }

        // Control de orientación: apunta al objetivo
        const double target = std::atan2(dy, dx);
        const double e = norm_angle(target - p.theta);

        // Ley de control simple
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = std::min(2.0, k_lin * dist);              // limita velocidad lineal
        cmd.angular.z = std::clamp(k_ang * e, -4.0, 4.0);         // limita velocidad angular

        cmd_pub_->publish(cmd);

        rate.sleep();
      }
    }

    // Si llegó a todos los puntos:
    publish_stop();
    result->success = true;
    result->message = "Trayectoria completada. Volviendo a modo MANUAL.";
    goal_handle->succeed(result);

    // Volver a manual al finalizar
    mode_ = MANUAL;
  }

  // -----------------------------------------------------------
  // ROS entities
  // -----------------------------------------------------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

  rclcpp::Service<SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<GetMode>::SharedPtr get_mode_srv_;

  rclcpp_action::Server<FollowTrajectory>::SharedPtr traj_action_server_;

  // Timer para el modo CIRCLE
  rclcpp::TimerBase::SharedPtr circle_timer_;

  // -----------------------------------------------------------
  // Estado interno
  // -----------------------------------------------------------
  Mode mode_;
  int circle_dir_;                 // -1 horario, +1 antihorario

  std::atomic_bool stop_requested_; // para abortar trayectoria si cambia el modo

  std::mutex pose_mtx_;
  turtlesim::msg::Pose pose_;
  bool pose_received_;
};

// -----------------------------------------------------------
// main
// -----------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleModeController>());
  rclcpp::shutdown();
  return 0;
}

