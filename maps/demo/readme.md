# Maps test demo

## Description
The 'maps.processor' class will be used to display the desired area to the user. The 'maps.planner' class will fit a path based on the 'water axis' for the input start and end points within that area, providing the relevant waypoints.

## Dependencies
The following toolboxes are required:
- [Mapping Toolbox](https://de.mathworks.com/products/mapping.html) 

## Setup
Follow the steps below to run this demo:
- Add +maps as your 'run' class.
- Replace 'pwd' with the path of the folder where your .shp files are stored.
- Enter the category you want to extract in the input bar Desirename. The code will classify according to the name of the .shp file, and eventually classify all .shp files of the same category name into one pgon_memory
> Note：

> It is recommended not to use Depth areas with multiple layers and unsurveyed areas as 'depare' input (e.g., .7V7ZEEK1, 7V7RUP03), as this will result in unpredictable runtime and unknown errors.

> Please select connected .000 files as input, otherwise the correct route cannot be determined.
``` Matlab
addpath( 'pwd'); % Set the +maps (namespace) path
folder = 'pwd';  % Set the folder path for reading .shp files
desirename=["depare","bridge","wtwaxs","lndare"]; %give the desirename %"notmrk"
```
- 'maps.processor(desirename, folder)' creates an instance of the 'maps.processor' class with the parameters 'desirename' and 'folder'.
- 'p.plot()' calls the 'plot' method of the 'maps.processor' instance, which is used to visualize the specified area.
``` Matlab
p = maps.processor(desirename, folder);
p.plot();
```
- This line creates an instance of the 'maps.planner' class, named pl. It uses the pgon_memory property of the previously created 'maps.processor' instance (p) as an input parameter. The 'pgon_memory' contains the necessary data about the desire area for the planner to work with.
```Matlab
pl = maps.planner(p.pgon_memory);
```
- The 'generate_random_points' method of the 'maps.planner' instance (pl) to automatically generate random start and end points for testing purposes.
```Matlab
% Defines the given starting and ending points
% given_point1 = [~, ~];
% given_point2 = [~, ~];
[given_point1, given_point2]=pl.generate_random_points(); % generate start(given point 1) and end points(given point 2) for testing
```
- This line calls the 'plan_path' method of the 'maps.planner' instance (pl), using the given_point1 and given_point2 as input parameters. The 'plan_path' method computes the best path.
```Matlab
pl = pl.plan_path(given_point1, given_point2);
```
- This line calls the 'plot_path' method of the 'maps.planner' instance (pl) with the '1'. The argument 1 indicates that the method should plot the full graph with all relevant information, including nodes, edges, waypoints, the start point, the end point, and the best path. 
```Matlab
pl.plot_path(1);
```
## Results
to be added
