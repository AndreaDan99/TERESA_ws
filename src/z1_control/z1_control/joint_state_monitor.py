from sensor_msgs.msg import JointState


class JointStateMonitor:
    def __init__(self, node, joint_names):
        self.node = node
        self.joint_names = joint_names
        self.positions = {j: 0.0 for j in joint_names}
        self.velocities = {j: 0.0 for j in joint_names}
        self.ready = False

        self.sub = node.create_subscription(
            JointState,
            "/joint_states",
            self._cb,
            10
        )

    def _cb(self, msg: JointState):
        # robusto anche se arriva un subset
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        updated = False

        for j in self.joint_names:
            if j in name_to_idx:
                i = name_to_idx[j]
                if i < len(msg.position):
                    self.positions[j] = msg.position[i]
                    updated = True
                if i < len(msg.velocity):
                    self.velocities[j] = msg.velocity[i]

        if updated and not self.ready:
            self.ready = True
            self.node.get_logger().info("JointStateMonitor ready.")

    def get_positions(self):
        return [self.positions[j] for j in self.joint_names]

    def get_velocities(self):
        return [self.velocities[j] for j in self.joint_names]
