import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from mavros_msgs.msg import Waypoint, State
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import math
import requests


class MissionModeNode(Node):

    def __init__(self):
        super().__init__('mission_mode_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.get_logger().info("Nodo missione avviato!")

        self.setpoint_timer = self.create_timer(
            0.1, self.publish_position_setpoint)

        self.obstacle_timer = self.create_timer(
            1.0, self.publish_obstacle_position)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        self.obstacle_pub = self.create_publisher(
            PoseStamped, '/obstacle_position', qos_profile)

        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_callback, qos_profile_sensor_data)

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        self.obstacle_pos_sub = self.create_subscription(
            PoseStamped, '/obstacle_position', self.obstacle_pos_callback, qos_profile_sensor_data)

        self.wp_push_client = self.create_client(
            WaypointPush, '/mavros/mission/push')
        while not self.wp_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio /mavros/mission/push non disponibile, in attesa...')

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio /mavros/set_mode non disponibile, in attesa...')

        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')

        # GPS coordinate
        self.lat = 37.4144411
        self.lon = -121.9959840
        self.alt = 10.0

        self.current_state = State()
        self.setpoint = PoseStamped()
        self.current_position = None
        self.reference_position = PoseStamped().pose.position
        self.obstacle_position = None
        self.scostamento = 20.0
        self.count = 0

        self.send_mission(self.lat, self.lon, self.alt)
        self.service_check()

    def send_mission(self, lat, lon, alt):
        # Setting waypoint coordinates
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 16  # NAV_WAYPOINT
        wp.is_current = True
        wp.autocontinue = True
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt

        # Pushing the waypoint in MAVROS service
        wp_push_req = WaypointPush.Request()
        wp_push_req.start_index = 0
        wp_push_req.waypoints.append(wp)

        future = self.wp_push_client.call_async(wp_push_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Mission pushed')
            self.set_auto_mission_mode()
        else:
            self.get_logger().error("Error in pushing the mission")

    def service_check(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0) or not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for services")
        self.get_logger().info("Services available")
        self.arm_drone()

    def state_callback(self, msg):
        if msg.mode != self.current_state.mode:
            self.current_state = msg
            self.get_logger().info(f"Stato attuale: {self.current_state.mode}")

    def local_pos_callback(self, msg):
        self.current_position = msg.pose.position

    def obstacle_pos_callback(self, msg):
        self.obstacle_position = msg.pose.position

    def publish_position_setpoint(self):
        if self.current_position is None or self.obstacle_position is None:
            return

        if self.found_obstacle():
            self.get_logger().info("Ostacolo rilevato!")
            self.reference_position = self.current_position

            # Passa alla modalità OFFBOARD solo se non è già in modalità OFFBOARD
            if self.current_state.mode != "OFFBOARD" :
                self.change_mode("OFFBOARD")

            # Invia una richiesta al server Flask per ottenere nuove coordinate
            new_x, new_y = self.request_new_coordinates(
                self.reference_position.x, self.reference_position.y)

            # Crea un nuovo setpoint con le coordinate aggiornate
            self.setpoint = PoseStamped()
            self.setpoint.pose.position.x = new_x
            self.setpoint.pose.position.y = new_y
            self.setpoint.pose.position.z = self.alt
            self.setpoint_pub.publish(self.setpoint)
        else:
            # Continua a pubblicare il setpoint corrente per mantenere la modalità OFFBOARD
            if self.current_position is not None:
                self.setpoint_pub.publish(self.setpoint)

            # Controlla se il drone ha raggiunto il setpoint
            if self.reached_setpoint():
                # Passa alla modalità AUTO.MISSION solo se non è già in modalità AUTO.MISSION
                if self.current_state.mode != "AUTO.MISSION":
                    self.change_mode("AUTO.MISSION")

    def request_new_coordinates(self, x, y):
        try:
            response = requests.post("http://localhost:5000/update_coordinates", json={
                "X": x,
                "Y": y
            })
            data = response.json()
            return data["new_x"], data["new_y"]
        except Exception as e:
            self.get_logger().error(
                f"Errore nel richiedere nuove coordinate: {e}")
            return x, y

    def found_obstacle(self):
        if self.current_position is None or self.setpoint is None:
            return False

        distance = math.sqrt(
            (self.obstacle_position.x - self.current_position.x) ** 2 +
            (self.obstacle_position.y - self.current_position.y) ** 2 +
            (self.obstacle_position.z - self.current_position.z) ** 2
        )
        if distance < 5.0:
            self.reference_position = self.current_position
            return True
        return False

    def publish_obstacle_position(self):
        obstacle_msg = PoseStamped()
        obstacle_msg.pose.position.x = 15.0
        obstacle_msg.pose.position.y = 30.0
        obstacle_msg.pose.position.z = 10.0
        self.obstacle_pub.publish(obstacle_msg)

    def reached_setpoint(self):
        if self.current_position is None or self.setpoint is None:
            return False

        tolerance = 1.0
        dx = abs(self.current_position.x - self.setpoint.pose.position.x)
        dy = abs(self.current_position.y - self.setpoint.pose.position.y)
        dz = abs(self.current_position.z - self.setpoint.pose.position.z)

        if dx < tolerance and dy < tolerance and dz < tolerance:
            return True
        return False

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Drone armato con successo')
        else:
            self.get_logger().info('Impossibile armare il drone')

    def set_auto_mission_mode(self):
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = "AUTO.MISSION"
        future = self.set_mode_client.call_async(set_mode_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info('Modalità AUTO.MISSION impostata con successo!')
        else:
            self.get_logger().error('Errore nell\'impostazione della modalità AUTO.MISSION.')

    def change_mode(self, mode):
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            req = SetMode.Request()
            req.custom_mode = mode
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.mode_change_callback)

    def mode_change_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"Modalità impostata con successo!")
            else:
                self.get_logger().warn(f"Cambio modalità fallito!")
        except Exception as e:
            self.get_logger().error(f"Errore nel cambio modalità: {e}")


def main(args=None):
    rclpy.init(args=args)
    mission_mode_node = MissionModeNode()
    rclpy.spin(mission_mode_node)
    mission_mode_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
