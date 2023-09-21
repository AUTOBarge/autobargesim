from osgeo import ogr
ogr.UseExceptions()
# Input and output file paths
input_name = "7V7ZEEK1"
output_dir = "./output"
desirenames = ["lndare", "bridge", "wtwaxs", "depare", "notmrk"]
# Open .000
input_file = f"{input_name}.000"
dataset = ogr.Open(input_file)
# Get layer count
layer_count = dataset.GetLayerCount()
print(layer_count)

output_count = 0
for i in range(layer_count):
    layer = dataset.GetLayerByIndex(i)
    layer_name = layer.GetName()
    print(layer_name)
# Traversal layer

for desirename in desirenames:
    for i in range(layer_count):
        layer = dataset.GetLayerByIndex(i)
        layer_name = layer.GetName()
        print(layer_name)
        # Determine if the layer name contains "---" features
        if desirename in layer_name.lower():
            print("find target layer:", layer_name)
            # Create drivers for.shp files
            driver = ogr.GetDriverByName("ESRI Shapefile")

            # Create drivers for.shp files
            output_file = f"{output_dir}/output_{input_name}_{desirename}.shp"
            output_dataset = driver.CreateDataSource(output_file)
            output_layer = output_dataset.CreateLayer(layer_name, layer.GetSpatialRef(), ogr.wkbUnknown)

            # Copy field
            layer_defn = layer.GetLayerDefn()
            for j in range(layer_defn.GetFieldCount()):
                field_defn = layer_defn.GetFieldDefn(j)
                if field_defn.GetType() == ogr.OFTString or field_defn.GetType() == ogr.OFTInteger or field_defn.GetType() == ogr.OFTReal:
                    output_layer.CreateField(field_defn)

            # Replication element
            for feature in layer:
                output_layer.CreateFeature(feature)

            # Clearing resources
            output_dataset = None
            print("saved target layer in output.shp")
            output_count = output_count + 1
            break

        else:
            print("No target layer")
        # Clearing resources
print(f"output: {output_count} .shp files")
dataset = None