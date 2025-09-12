import csc376_franky

csc376_gripper = csc376_franky.Gripper("192.168.1.107")
gripper_state = csc376_gripper.state

print(f"Current width: {gripper_state.width}")
print(f"Is gripper grasped: {gripper_state.is_grasped}")
