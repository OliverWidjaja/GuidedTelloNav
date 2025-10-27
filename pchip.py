from scipy.interpolate import pchip_interpolate


WAYPOINTS = [
    [0.0, 0.0, 0.5, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.5, 0.0, 1.25, 0.0]
]

def waypoint_interpolation(start, end, num_points):
    """Generate interpolated waypoints between start and end"""
    waypoints = []
    for i in range(1, num_points + 1):
        ratio = i / (num_points + 1)
        x = start[0] + (end[0] - start[0]) * ratio
        y = start[1] + (end[1] - start[1]) * ratio
        z = start[2] + (end[2] - start[2]) * ratio
        yaw = start[3] + (end[3] - start[3]) * ratio
        waypoints.append([x, y, z, yaw])
    return waypoints

traj = waypoint_interpolation(WAYPOINTS[0], WAYPOINTS[1], 2)

print(traj)