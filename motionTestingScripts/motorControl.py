import can
import struct

node_id = 3  # must match `<odrv>.axis0.config.can.node_id`. The default is 0.
bus = can.interface.Bus("can1", bustype="socketcan")

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None):
    pass

# Put axis into closed loop control state
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07),  # 0x07: Set_Axis_State
    data=struct.pack('<I', 8),  # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))

# Wait for axis to enter closed loop control by scanning heartbeat messages
for msg in bus:
    if msg.arbitration_id == (node_id << 5 | 0x01):  # 0x01: Heartbeat
        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        if state == 8:  # 8: AxisState.CLOSED_LOOP_CONTROL
            break       

running = True

# Loop for user input
while running:
    try:
        user_input = input("Enter velocity (-20 to 20), or 'q' to quit: ")
        if user_input.lower() == 'q':
            running = False
            user_input = 0
        
        velocity = float(user_input)
        velocity = max(-20.0, min(20.0, velocity))  # Clamp value between -20 and 20
        
        bus.send(can.Message(
            arbitration_id=(node_id << 5 | 0x0d),  # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', velocity, 0.0),  # velocity, torque feedforward
            is_extended_id=False
        ))
        print(f"Sent velocity command: {velocity:.3f} turns/s")
    
    except ValueError:
        print("Invalid input. Please enter a number between -20 and 20, or 'q' to quit.")


bus.shutdown()