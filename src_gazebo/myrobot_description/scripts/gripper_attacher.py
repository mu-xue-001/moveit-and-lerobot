import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState, ModelState
import tf2_ros
import subprocess

class GripperAttacher(Node):
    def __init__(self):
        super().__init__('gripper_attacher')
        
        self.attached = False
        self.offset = [0.0, 0.0, 0.12]
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(ContactsState, '/gazebo/contacts', self.contact_cb, 10)
        self.follow_timer = None
        
        self.get_logger().info('Gripper Attacher Started')
    
    def contact_cb(self, msg):
        for contact in msg.states:
            col1 = contact.collision1_name
            col2 = contact.collision2_name
            if ('gripper' in col1 and 'pipe' in col2) or ('pipe' in col1 and 'gripper' in col2):
                if not self.attached:
                    self.get_logger().info('Contact detected! Attaching...')
                    self.attached = True
                    self.follow_timer = self.create_timer(0.05, self.follow)
                return
        
        if self.attached and not msg.states:
            self.get_logger().info('Detaching...')
            self.attached = False
            if self.follow_timer:
                self.follow_timer.cancel()
                self.follow_timer = None
    
    def follow(self):
        if not self.attached:
            return
        try:
            t = self.tf_buffer.lookup_transform('world', 'gripper_left_finger_link', rclpy.time.Time())
            cmd = ['gz', 'model', '-m', 'metal_pipe', 
                   '-x', str(t.transform.translation.x + self.offset[0]),
                   '-y', str(t.transform.translation.y + self.offset[1]),
                   '-z', str(t.transform.translation.z + self.offset[2])]
            subprocess.run(cmd, capture_output=True, timeout=0.1)
        except:
            pass

def main():
    rclpy.init()
    node = GripperAttacher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()