import py_trees
import time

# --- Mock action/condition behaviors ---

class Wait(py_trees.behaviour.Behaviour):
    def __init__(self, delay_sec):
        super().__init__(name=f"Wait({delay_sec}s)")
        self.delay_sec = delay_sec
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if time.time() - self.start_time >= self.delay_sec:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

class Offboard(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Offboard")
        self.offboard_mode = False

    def update(self):
        if not self.offboard_mode:
            print("Switching to Offboard mode")
            self.offboard_mode = True
        return py_trees.common.Status.SUCCESS

class Arm(py_trees.behaviour.Behaviour):
    def __init__(self, drone_interface):
        super().__init__(name="Arm")
        self.drone_interface = drone_interface
        self.armed = False

    def update(self):
        if not self.armed:
            self.drone_interface.arm()
            self.armed = True
            print("Arming drone (via interface)")
        return py_trees.common.Status.SUCCESS
    

class Disarm(py_trees.behaviour.Behaviour):
    def __init__(self, drone_interface):
        super().__init__(name="Disarm")
        self.drone_interface = drone_interface
        self.disarmed = False

    def update(self):
        if not self.disarmed:
            self.drone_interface.disarm()
            self.disarmed = True
            print("Disarming drone (via interface)")
        return py_trees.common.Status.SUCCESS

class Fly_flame_pattern(py_trees.behaviour.Behaviour):
    def __init__(self, target_location):
        super().__init__(name="FlyToTarget")
        self.target = target_location

    def update(self):
        print(f"Flying to {self.target}")
        return py_trees.common.Status.SUCCESS

class Return_to_base(py_trees.behaviour.Behaviour):
    def update(self):
        print("Returning to base")
        return py_trees.common.Status.SUCCESS

class Land(py_trees.behaviour.Behaviour):
    def __init__(self, drone_interface):
        super().__init__(name="Land")
        self.drone_interface = drone_interface
        self.landed = False

    def update(self):
        if not self.landed:
            self.drone_interface.land()
            self.landed = True
            print("Landing (via interface)")
        return py_trees.common.Status.SUCCESS

# Example drone interface stub (replace with your ROS2 node or interface)
class DroneInterface:
    def arm(self):
        # Here you would call your ROS2 node's arm() method
        print("DroneInterface.arm() called")

def create_drone_behaviour_tree(start_delay, base_pose, offset, drone_interface=None):
    """Creates a behavior tree for a drone with specific offset and delay."""
    scan_pose = (
        base_pose[0] + offset[0],
        base_pose[1] + offset[1],
        base_pose[2]
    )

    root = py_trees.composites.Sequence("DroneMission")

    children = [
        Wait(start_delay),
        Arm(),
        Offboard(),
        Fly_flame_pattern(scan_pose),
        Return_to_base(),
        Disarm(drone_interface),
        Land()
    ]
    # If you want to use the Arm behaviour, insert it after Wait
    if drone_interface is not None:
        children.insert(1, Arm(drone_interface))

    root.add_children(children)
    return root


if __name__ == "__main__":
    drones = {
        "drone1": {"delay": 0.0, "offset": (0, 0)},
        "drone2": {"delay": 5.0, "offset": (10, 0)},
        "drone3": {"delay": 10.0, "offset": (20, 0)},
    }

    base_pose = (0, 0, 10)  # Shared base scan altitude

    trees = {}
    for drone_name, params in drones.items():
        bt = create_drone_behaviour_tree(
            start_delay=params["delay"],
            base_pose=base_pose,
            offset=params["offset"]
        )
        trees[drone_name] = py_trees.trees.BehaviourTree(bt)

    # Tick each tree (very basic tick loop)
    while True:
        all_done = True
        for name, tree in trees.items():
            status = tree.tick()
            if status != py_trees.common.Status.SUCCESS:
                all_done = False
        if all_done:
            break
        time.sleep(0.5)