# 0002shp
# Requirements and Installation
- Python version >= 3.6.
- Install [GDAL](https://gdal.org/index.html) Package.
# About
This is a tool for converting .000 files to .shp files. The provided .000 files are from the [Flanders region](https://www.visuris.be/Vaarkaarten) and are named by their built-in names. To avoid legal issues, it is not convenient to provide .000 files here. Please download them after obtaining the necessary permissions on your own. To use this, you must be able to use the tool to view the river ENC, and then select the area of interest for extraction. The author used [Open CPN](https://opencpn.org/) to read these maps. You can also extract .000 files from the countries and regions you are interested in.
In addition, the extraction names of the files are strictly extracted according to the names defined by [S-57](http://www.s-57.com/). If you don't know the meaning of each name, you can look them up.
# Explain and Run
- Enter the name of the .000 file you want to extract "input_name", and determine the file path "output_dir" where you want to save it. Fill in the name of the Layer you want to extract from the .000 file into "desirenames". Add the current .000 file path to 'input_file = f"{input_name}.000"'
``` Python
input_name = "7V7ZEEK1"
output_dir = "./output"
desirenames = ["lndare", "bridge", "wtwaxs", "depare", "notmrk"]
# Open .000
input_file = f"{input_name}.000"
```
- The code will provide the names of the Layers present in the current .000 file for you to refer and select.
``` Python
for i in range(layer_count):
    layer = dataset.GetLayerByIndex(i)
    layer_name = layer.GetName()
    print(layer_name)
```

