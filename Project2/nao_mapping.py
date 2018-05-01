'''L by L meters surrounding environment of a mobile robot presented as a grid.

The data is acquired using camera mounted in the front of the Nao robot's head x, y and z.

The camera detects landmarks within its visual field.

Range data further than 40 inches is disregarded since it may contain false measurements.

Detected landmarks inside the local grid are directly used for computing the local gridself.

The boundaries where the landmarks reside are interpolated as walls of the local grid.

The desired end points input to the Bresenham's line algorithm to indicate free and occupied cells.

Bresenham's line algorithm determines which cells on the 2D grid should be selected to form a close approximation to a straight line between camera origin and end point.

The end points on the border of the local grid map are used to determine free cells and end points positioned inside the grid are used to determine free and occupied cells.

For having a consistent local map it is needed to continuously integrate a new local grid into the previous local grid map.

This is done by rotation and translation of the previous grid map on the new computed grid.

The difference between the robot poses in two consecutive frames is used to compute the transformation matrix.

The odometry data provides robot's pose.

The robot pose is defined by (x, y, \theta) where x and y are coordinates relative to some known start point and \theta is the robot's heading.

After transformation, it is needed to update the map values.

Free and occupancy counters are defined.

Counter values are updated using Reflectation mapping policy: C = hit /(hit + miss).

Where hit represents the number of times a beam ended in that cell while miss stands for the number of times a beam passed through that grid cell.

---------------------------------------------------------------
Inputs are: laser range data (L) and estimated odometry (X)
Output is: local occupancy grid map
The functions are:
init - initialize mapping
mp - build current local grid map from laser data
prc - load and pre-process data
shw - show grid map, laser points, robot locations, and current robot direction
trns - transform map t-1 on map t
upd - update current local map
