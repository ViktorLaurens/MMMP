new_pose = input("Set arm pose to (or 0 to exit): ")
new_pose = tuple([float(x) for x in new_pose.split()])
if not new_pose:
    print("Exiting...")
print(f"\nSet Pose: {new_pose}")