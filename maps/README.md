# Maps Module

## About
 This module allows users to customize maps, and the algorithm can automatically generate waypoints and depths based on user-defined start and end points.
 
 They are two different functional codes, please review their readme and use them separately.
 
 - **`maps/demo/0002shp_tool/readme.md`** Since MATLAB can't directly read and modify `.000` files, this method is used to display them in MATLAB. You can use `0002shp.py` to batch extract `.shp` files.
 - **`maps/demo/readme.md`** You can use the map class's processor to read the map files in `maps/demo/.shp`. (You can also use `0002shp.py` to extract `.shp` files for the ENC area you're interested in.) This allows you to select the area and features you want to visualize from the `.shp` files and process the point, line, and polygon data. After entering the starting and ending points, the Planner module of the map class can generate routes and depths.
 
 Create the ENC map you need in MATLAB for easy Autobarge simulation!

## Dependencies
- **maps/demo/0002shp_tool/0002shp.py**: Python version >= 3.6, Install GDAL Package.
- **maps/demo/maps_test_demo.m**: N/A

## Contact
Email: [Zhongbi Luo](mailto:zhongbi.luo@kuleuven.be)
