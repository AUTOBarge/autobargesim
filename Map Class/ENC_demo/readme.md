# Shpread
# Getting Started
Matlab
## Pre-requisite
[Mapping Toolbox](https://de.mathworks.com/products/mapping.html) 
# Explain and Run
- Add ENC_class/ENC.m as your 'run' function.
- Replace 'pwd' with the path of the folder where your .shp files are stored.
``` Matlab
folder = 'pwd';  % Set the folder path!!!!!!!!!!!!!!!
files = dir(fullfile(folder, '*.shp'));  % Get information of all .shp files in the folder
desirename=["depare","bridge","wtwaxs","lndare","notmrk"]; %give the desirename
```
- Enter the category you want to extract in the input bar Desirename. The code will classify according to the name of the .shp file, and eventually classify all .shp files of the same category name into one pgon_memory
> For example, the code will automatically classify the files in the .shp folder with "notmrk" as one category, and store them in the form of points.
``` Matlab
pgon_memory_5 = ENC_function.shape_multi(desirename,files,folder,desirename(5));
```
- Each time, provide any number of category names of the 'S-57 standard' you want to analyze and place them in "desirename". Then apply the function ENC_function.shape_multi in sequence to obtain their file data.
> Note the consistency between the desirename in your folder and the provided desirename.
- Select the category you need to plot. For points, lines, and polygons, there are examples of polt.
```Matlab
%%%%polt
```
