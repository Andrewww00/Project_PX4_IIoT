import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from mavros_msgs.msg import Waypoint, State
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import math


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

        # Timer per pubblicare i setpoint di posizione a 10 Hz
        self.setpoint_timer = self.create_timer(
            0.1, self.publish_position_setpoint)

        # Timer per pubblicare la posizione dell'ostacolo ogni 1 secondo
        self.obstacle_timer = self.create_timer(
            1.0, self.publish_obstacle_position)

        # Publisher per setpoint di posizione sul topic /mavros/setpoint_position/local
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # Publisher per la posizione dell'ostacolo
        self.obstacle_pub = self.create_publisher(
            PoseStamped, '/obstacle_position', qos_profile)

        # Subscriber per ottenere la posizione attuale del drone
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_callback, qos_profile_sensor_data)

        # Subscriber per lo stato del drone
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        # Subscriber per ottenere la posizione dell'ostacolo
        self.obstacle_pos_sub = self.create_subscription(
            PoseStamped, '/obstacle_position', self.obstacle_pos_callback, qos_profile_sensor_data)

        # Client per caricare i waypoint
        self.wp_push_client = self.create_client(
            WaypointPush, '/mavros/mission/push')
        while not self.wp_push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio /mavros/mission/push non disponibile, in attesa...')

        # Client per impostare la modalità del drone
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio /mavros/set_mode non disponibile, in attesa...')

        # Client per il servizio di arming
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')

        # Coordinate GPS della missione

        self.lat = 37.4144411 #47.3986402
        self.lon = -121.9959840 #8.5462568
        self.alt = 10.0

        self.current_state = State()
        self.setpoint = PoseStamped()
        self.current_position = None
        self.reference_position = PoseStamped().pose.position  # Inizializzazione della posizione di riferimento
        self.obstacle_position = None
        self.scostamento = 20.0  # Spostamento verso destra per evitare l'ostacolo
        self.count = 0 

        self.send_mission()
        self.service_check()

    def send_mission(self):
        # Creazione di un waypoint per la missione
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = 16  # NAV_WAYPOINT
        wp.is_current = True
        wp.autocontinue = True
        wp.z_alt = self.alt
        wp.x_lat = self.lat
        wp.y_long = self.lon

        # Invia il waypoint al servizio di MAVROS
        wp_push_req = WaypointPush.Request()
        wp_push_req.start_index = 0
        wp_push_req.waypoints.append(wp)

        future = self.wp_push_client.call_async(wp_push_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Missione caricata con successo!')
            # Imposta la modalità AUTO.MISSION dopo aver caricato la missione
            self.set_auto_mission_mode()
        else:
            self.get_logger().error('Errore nel caricamento della missione.')

    def service_check(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0) or not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Attendo i servizi...')
        self.get_logger().info("Servizi disponibili")
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
        
        if self.found_obstacle() and self.count==0:
            self.count +=1
            self.get_logger().info("Ostacolo rilevato!")
            self.reference_position = self.current_position
            self.change_mode("OFFBOARD")

        # Imposta i nuovi setpoint di posizione basati sulla posizione di riferimento
        self.setpoint.pose.position.x = self.reference_position.x + self.scostamento
        self.setpoint.pose.position.y = self.reference_position.y
        self.setpoint.pose.position.z = self.reference_position.z

        self.setpoint_pub.publish(self.setpoint)

        if self.reached_setpoint():
            self.set_auto_mission_mode()
        
    
    def found_obstacle(self):
        if self.current_position is None or self.setpoint is None:
            return False
        
        distance = math.sqrt(
            (self.obstacle_position.x - self.current_position.x)**2 +
            (self.obstacle_position.y - self.current_position.y)**2 +
            (self.obstacle_position.z - self.current_position.z)**2
        )
        if distance < 5.0:
            # Aggiorna la reference_position quando viene rilevato l'ostacolo
            self.reference_position = self.current_position  
            return True
        return False


    def publish_obstacle_position(self):
        # Crea un messaggio di tipo PoseStamped per la posizione dell'ostacolo
        obstacle_msg = PoseStamped()
        obstacle_msg.pose.position.x = 15.0
        obstacle_msg.pose.position.y = 30.0
        obstacle_msg.pose.position.z = 10.0  
        self.obstacle_pub.publish(obstacle_msg)

    def reached_setpoint(self):
        if self.current_position is None or self.setpoint is None:
            return False

        # Tolleranza per considerare il setpoint raggiunto
        tolerance = 1.0
        dx = abs(self.current_position.x - self.setpoint.pose.position.x)
        dy = abs(self.current_position.y - self.setpoint.pose.position.y)
        dz = abs(self.current_position.z - self.setpoint.pose.position.z)

        # Controlla se il drone è vicino abbastanza al setpoint
        if dx < tolerance and dy < tolerance and dz < tolerance:
            self.get_logger().info("Setpoint raggiunto!")
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
