import rps.robotarium as robotarium
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np

# N = number of robots
N = 1
# Robotarium object
initial_conditions = np.array(np.mat('0.01;0.01;0'))
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions,
                          sim_in_real_time=True)

# -------------------- bellow is some graifx stuff from Maben

# Define goal points by removing orientation from poses - modified to lie outside the shown area
arrayExtra = np.array([2.5, 0, 0])
arrayExtra.shape = (3, 1)
goal_points = arrayExtra

# --- Create single integrator pose controller
position_controller = create_si_position_controller()
# --- Create converter that takes single integrator
#    velocity vector gives the equivalent unicycle velocity vector
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# get current position and pose so that the graphics lines coming below
#  can draw the initial position and pose
x = r.get_poses()

# Plotting Parameters
CM = np.random.rand(N, 3)  # Random Colors
goal_marker_size_m = 0.2
robot_marker_size_m = 0.15
marker_size_goal = determine_marker_size(r, goal_marker_size_m)
marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r, 0.1)
line_width = 5

# Create Goal Point Markers
# Text with goal identification
goal_caption = ['G{0}'.format(ii) for ii in range(goal_points.shape[1])]
# Arrow for desired orientation
goal_orientation_arrows = [
    r.axes.arrow(goal_points[0, ii], goal_points[1, ii], goal_marker_size_m * np.cos(goal_points[2, ii]),
                 goal_marker_size_m * np.sin(goal_points[2, ii]), width=0.02, length_includes_head=True,
                 color=CM[ii, :], zorder=-2)
    for ii in range(goal_points.shape[1])]
# Plot text for caption
goal_points_text = [r.axes.text(goal_points[0, ii], goal_points[1, ii], goal_caption[ii], fontsize=font_size, color='k',
                                fontweight='bold', horizontalalignment='center', verticalalignment='center', zorder=-3)
                    for ii in range(goal_points.shape[1])]
goal_markers = [
    r.axes.scatter(goal_points[0, ii], goal_points[1, ii], s=marker_size_goal, marker='s', facecolors='none',
                   edgecolors=CM[ii, :], linewidth=line_width, zorder=-3)
    for ii in range(goal_points.shape[1])]
robot_markers = [
    r.axes.scatter(x[0, ii], x[1, ii], s=marker_size_robot, marker='o', facecolors='none', edgecolors=CM[ii, :],
                   linewidth=line_width)
    for ii in range(goal_points.shape[1])]

# ----  choose lane radius and the number of simulation steps
laneRadius = float(0.4)  # --------------------------------------------this one is actually pretty important for
# the stuff bellow too, it's the radius of the circle


# ----  draw infinity track using two circular track
circle = plt.Circle((-laneRadius, 0), laneRadius, color='r', fill=False, linewidth=line_width)
circle2 = plt.Circle((laneRadius, 0), laneRadius, color='r', fill=False, linewidth=line_width)
r.axes.add_artist(circle)
r.axes.add_artist(circle2)

# -------------------------grafix stuff i got from Maben


#  Waypoints are points along the infinity symbol that the robot will go to
# The more waypoints the slower but more accurate the robot will be
numOfWaypoint = 30000

# An array of zeroes, the number of zeroes is defined by the number of waypoints
zeroArr = np.zeros(numOfWaypoint, dtype=np.float64)
# Equally spacing the elements in zeroArr between 0 and 2pi
# Meaning they will be spaced equally along a circle
for i in range(numOfWaypoint):
    zeroArr[i] = 2 * np.pi * i / numOfWaypoint
# Make the array of waypoints into an array of columns were every column has two elements
waypoints = np.zeros((2, 2 * numOfWaypoint), dtype=np.float64)

# this code defines the waypoints around the left circle
for i in range(numOfWaypoint):
    angle = zeroArr[i]
    # The i in numOfWaypoints is to offset the circle, bellow it is + numOfWaypoints to offset it even more
    waypoints[0, i] = -1*laneRadius + laneRadius * np.cos(angle)
    waypoints[1, i] = laneRadius * np.sin(angle)

# and this code defines the waypoints around the right circle
for i in range(numOfWaypoint):
    angle = zeroArr[i]
    waypoints[0, i + numOfWaypoint] = laneRadius - laneRadius * np.cos(angle)
    waypoints[1, i + numOfWaypoint] = laneRadius * np.sin(angle)

currentWaypoint = waypoints[:, 0]
# shape tells the python compiler that this array is a column vector
waypointIndex = 0
currentWaypoint.shape = (2, 1)
# this is the number that specifies how close the bot has to get to the waypoint before it has to go to the next
# waypoint
pointChangeDistance = 0.01
# this is needed later

#  initialize the simulation
r.step()
# number of steps, basically how many times the code runs
count = 0
countMax = 2000

while count < countMax:
    # adds one to the amount of times the loop is supposed to run
    count = count + 1

    # Get poses of agents
    x = r.get_poses()

    # -------------- this bellow is some grafix stuff that i need and just copied from Maben
    for i in range(x.shape[1]):
        robot_markers[i].set_offsets(x[:2, i].T)
    for j in range(goal_points.shape[1]):
        goal_markers[j].set_sizes([determine_marker_size(r, goal_marker_size_m)])
    # -------------- and that's the end of the grafixc stuff

    #  gets the position from the robot (as declared before, robot = x)
    xBot = float(x.item(0))
    yBot = float(x.item(1))
    posiBot = float(x.item(2))  # this one spesifies the angle of the robot, aka what way it is facing
    # but i'm never actually going to use that variable in my code, just thought you'd want to know what it was

    # get the current waypoint
    currentXPoint = currentWaypoint[0]
    currentYPoint = currentWaypoint[1]

    #  this code checks how close the bot is to the waypoint it is targeting
    distanceSq = np.square(xBot - currentXPoint) + np.square(yBot - currentYPoint)
    #   and if that distance is close or equal to the distance where it is supposed to change the waypoint target
    if distanceSq <= pointChangeDistance:
        #   adds one to the waypoint index
        waypointIndex = waypointIndex + 1
        #  if we have reached the last column of the waypoints array
        #     then go back to the zeroth column
        if waypointIndex >= 2 * numOfWaypoint:
            waypointIndex = 0
        #   indicates what waypoint the bot is supposed to target
        currentWaypoint = waypoints[:, waypointIndex]
        currentWaypoint.shape = (2, 1)

    #   makes a column vector of the current position of the robot
    botPos = np.array([xBot, yBot])
    botPos.shape = (2, 1)

    # getting the velocity from the position controller by supplying the position
    #  and the waypoint that the bot is targeting
    vel = position_controller(botPos, currentWaypoint)
    #  "convert the rectilinear velocity vector into
    #   the tangential velocity and turning rate
    #   needed for steering as a unicycle"
    uni_vel = si_to_uni_dyn(vel, x)
    # Set the velocity command
    r.set_velocities(0, uni_vel)

    # Iterate the simulation
    r.step()

# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
