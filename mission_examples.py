from drone_controller import (
    FlightController, Position3D, DroneState, GripperState
)

def example_1_simple_transport():
    """
    Simple package transport from point A to point B
    """
    print("\n" + "="*60)
    print("EXAMPLE 1: Simple Package Transport")
    print("="*60)
    
    drone = FlightController()
    drone.home_position = Position3D(0, 0, 0)
    
    warehouse = Position3D(0, 0, 0.5)
    delivery_point = Position3D(20, 15, 0.5)
    
    drone.autonomous_transport_mission(warehouse, delivery_point)

def example_2_multi_pickup():
    """
    Multiple pickups and deliveries from same area
    """
    print("\n" + "="*60)
    print("EXAMPLE 2: Multi-Point Delivery Mission")
    print("="*60)
    
    drone = FlightController()
    drone.home_position = Position3D(0, 0, 0)
    
    delivery_points = [
        (Position3D(5, 5, 0.5), Position3D(10, 10, 0.5)),
        (Position3D(10, 10, 0.5), Position3D(15, 5, 0.5)),
        (Position3D(15, 5, 0.5), Position3D(5, 15, 0.5)),
    ]
    
    for idx, (pickup, dropoff) in enumerate(delivery_points, 1):
        print(f"\n--- Delivery {idx}/3 ---")
        drone.autonomous_transport_mission(pickup, dropoff)
        
        if drone.battery_percent < 20:
            print(f"\nBattery low ({drone.battery_percent:.1f}%). Returning home.")
            break

def example_3_obstacle_avoidance():
    """
    Navigate through area with obstacles
    """
    print("\n" + "="*60)
    print("EXAMPLE 3: Obstacle Avoidance Mission")
    print("="*60)
    
    drone = FlightController()
    drone.home_position = Position3D(0, 0, 0)

    from drone_controller import Obstacle
    drone.sensors.obstacles = [
        Obstacle(Position3D(10, 8, 1.0), radius=2.0, danger_level=0.8),
        Obstacle(Position3D(15, 12, 1.2), radius=1.5, danger_level=0.7),
    ]
    
    pickup = Position3D(5, 5, 0.5)
    dropoff = Position3D(20, 20, 0.5)
    
    print("\nEnvironment obstacles detected:")
    for i, obs in enumerate(drone.sensors.obstacles, 1):
        print(f"  Obstacle {i}: {obs.position.x:.1f}, {obs.position.y:.1f} (danger: {obs.danger_level:.1f})")
    
    drone.autonomous_transport_mission(pickup, dropoff)

def example_4_gripper_operations():
    """
    Detailed gripper control and object manipulation
    """
    print("\n" + "="*60)
    print("EXAMPLE 4: Advanced Gripper Operations")
    print("="*60)
    
    drone = FlightController()
    
    print("\nGripper State: OPEN")
    print(f"  Servo Angle: {drone.gripper.servo_angle}°")
    print(f"  Grip Force: {drone.gripper.grip_force}%")
    print(f"  Object Detected: {drone.gripper.detect_object()}")
    
    print("\nClosing gripper...")
    drone.gripper.set_detection(True)
    if drone.gripper.close():
        print(f"  State: {drone.gripper.state.name}")
        print(f"  Servo Angle: {drone.gripper.servo_angle}°")
        print(f"  Grip Force: {drone.gripper.grip_force}%")
        print(f"  Object Detected: {drone.gripper.detect_object()}")

    print("\nOpening gripper...")
    if drone.gripper.open():
        print(f"  State: {drone.gripper.state.name}")
        print(f"  Servo Angle: {drone.gripper.servo_angle}°")
        print(f"  Grip Force: {drone.gripper.grip_force}%")

def example_5_battery_management():
    """
    Demonstrate battery awareness and automatic return-to-home
    """
    print("\n" + "="*60)
    print("EXAMPLE 5: Battery Management and RTH")
    print("="*60)
    
    drone = FlightController()
    drone.home_position = Position3D(0, 0, 0)
    drone.battery_percent = 30  
    
    print(f"\nInitial Battery: {drone.battery_percent:.1f}%")
    print(f"Home Position: {drone.home_position.x:.1f}, {drone.home_position.y:.1f}")
    
    if drone.arm() and drone.takeoff(1.0):
        print("\nDrone taking off...")
        drone.position = Position3D(0, 0, 1.0)
        
        drone.battery_percent = 15
        drone.update_battery()
        
        if drone.battery_percent < 20:
            print(f"\n[WARNING] Battery critical: {drone.battery_percent:.1f}%")
            print("Initiating Return-To-Home sequence...")
            drone.state = DroneState.RETURNING
            drone.position = drone.home_position
            
            if drone.land():
                print("\n[SUCCESS] Landed safely at home")
                print(f"Final Battery: {drone.battery_percent:.1f}%")

def example_6_emergency_procedures():
    """
    Emergency procedures and failsafe mechanisms
    """
    print("\n" + "="*60)
    print("EXAMPLE 6: Emergency Procedures")
    print("="*60)
    
    drone = FlightController()
    drone.home_position = Position3D(0, 0, 0)
    drone.position = Position3D(15, 15, 5.0)  
    
    print(f"\nDrone Position: {drone.position.x:.1f}, {drone.position.y:.1f}, {drone.position.z:.1f}")
    print(f"Distance from Home: {drone.position.distance_to(drone.home_position):.2f} meters")
    

    print("\n[EMERGENCY] Motor failure detected!")
    print("Executing emergency descent...")

    if drone.gripper.state != GripperState.OPEN:
        drone.gripper.open()
        print("  - Payload released")

    descent_rate = 0.5 
    while drone.position.z > 0:
        drone.position.z -= descent_rate * 0.1
        if int(drone.position.z * 10) % 10 == 0:
            print(f"  - Altitude: {drone.position.z:.1f}m")
    
    print(f"\n[SUCCESS] Emergency landing complete at {drone.position.x:.1f}, {drone.position.y:.1f}")
    drone.state = DroneState.LANDED

def run_all_examples():
    """Run all mission examples"""
    examples = [
        example_1_simple_transport,
        example_2_multi_pickup,
        example_3_obstacle_avoidance,
        example_4_gripper_operations,
        example_5_battery_management,
        example_6_emergency_procedures,
    ]
    
    for example_func in examples:
        try:
            example_func()
        except Exception as e:
            print(f"\n[ERROR] in {example_func.__name__}: {e}")
        
        input("\nPress Enter to continue to next example...")

if __name__ == '__main__':
    print("\n" + "#"*60)
    print("# COEX Drone 4 - Mission Examples")
    print("#"*60)
    print("\nAvailable Examples:")
    print("  1. Simple Package Transport")
    print("  2. Multi-Point Delivery Mission")
    print("  3. Obstacle Avoidance Mission")
    print("  4. Advanced Gripper Operations")
    print("  5. Battery Management and RTH")
    print("  6. Emergency Procedures")
    
    example_1_simple_transport()
