## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

* We are adding another state in our state machine for planning which kicks in after the quad is armed and provides us a path from start to goal (if available).
* `plan_path` method creates a grid of size (north_max - north_min, east_max - east_min) from the csv
* we iterate through the csv and mark different cells in the grid as obsticales if our flying height is below the actual obstacle height
* we return this grid along with north and east offsets for passing grid postions to astar instead of local postion
* for `astar` we are using euclidian distance as heuristic

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

Reading first line from the csv will return us first element with 2 columns

```
row = ['lat0 37.792480', 'lon0 -122.397450']
```

We split each value by space and get element at first index and then convert it to float

Setting this (lat, lng) as our home ensures that our planning environment is matching that of the csv
while we use `global_to_local()` method to get local positions

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

We retrieve current global position using `self._longitude self._latitude self._altitude` instance vars and use `global_to_local` method

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

We use `global_to_local` again

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

* We add these states to our `Actions`

```
NORTHEAST = (-1, 1, math.sqrt(2))
NORTHWEST = (-1, -1, math.sqrt(2))
SOUTHEAST = (1, 1, math.sqrt(2))
SOUTHWEST = (1, -1, math.sqrt(2))
```

* Remove necessary diagonal motions when there are obstacles or out of bounds

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

I'm using colinearity. But colinearity is causing zigzags as it can't distinguish if point1 and point 3 are collinear. Bresenham might work well here

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


