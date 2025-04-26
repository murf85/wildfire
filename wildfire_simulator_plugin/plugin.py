from PyQt5.QtWidgets import (
    QLabel,
    QVBoxLayout,
    QComboBox,
    QPushButton,
    QWidget,
    QDockWidget,
    QFileDialog,
    QAction,
    QSlider,
    QMessageBox
)
from PyQt5.QtCore import Qt
from qgis.core import (
    QgsProject,
    QgsRasterLayer,
    QgsVectorLayer,
    QgsGeometry,
    QgsWkbTypes,
    QgsFeature,
    QgsProcessingFeedback,
    QgsVectorLayerTemporalProperties,
    QgsDataSourceUri,
    QgsVectorLayer,
    QgsFeature,
    QgsGeometry,
    QgsPointXY,
    QgsProject,
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsMessageLog,
    Qgis,
    QgsTemporalNavigationObject,
)
from qgis.gui import QgsMapToolIdentifyFeature
from qgis.gui import QgsMapToolEmitPoint, QgsRubberBand
from qgis.PyQt.QtGui import QColor
import json
import os
import processing
import rasterio
import numpy
import math
import subprocess
import threading
import csv
import numpy as np

#########################
# CONSTANTS
#########################

FUELS = {
    1: 10,   # temperate or sub-polar needleleaf forest -> FM10
    2: 13,   # sub-polar taiga -> FM13
    5: 9,    # temperate or sub-polar broadleaf deciduous forest -> FM9
    6: 8,    # forest foliage temperate or sub-polar -> FM8
    8: 141,  # temperate or sub-polar shrubland -> SH1
    10: 101, # temperate or sub-polar grassland -> GR1
    11: 93,  # sub-polar or polar shrubland-lichen-moss -> NB3
    12: 103, # sub-polar or polar grassland-lichen-moss -> GR3
    13: 99,  # sub-polar or polar barren-lichen-moss -> NB9
    14: 98,  # wetland -> NB8
    15: 93,  # cropland -> NB3
    16: 99,  # barren lands -> NB9
    17: 91,  # urban -> NB1
    18: 98,  # water -> NB8
    19: 92,  # snow and ice -> NB2
}

#########################
# PLUGIN CLASSES
#########################

class WFSPlugin:
    def __init__(self, iface):
        self.iface = iface
        self.dock_widget = None
        self.map_tool = None
        self.rubber_band = None
        self.selected_region = None
        self.ignited_region = None

    def initGui(self):
        self.action = QAction('Wildfire Simulator', self.iface.mainWindow())
        self.action.triggered.connect(self.run)
        self.iface.addToolBarIcon(self.action)

    def unload(self):
        self.iface.removeToolBarIcon(self.action)

    def run(self):
        if not self.dock_widget:
            self.dock_widget = WFSDockWidget(self)
            self.iface.addDockWidget(Qt.LeftDockWidgetArea, self.dock_widget)
        self.dock_widget.show()

class WFSDockWidget(QDockWidget):
    def __init__(self, plugin):
        super(WFSDockWidget, self).__init__(plugin.iface.mainWindow())
        self.plugin = plugin
        self.setWindowTitle("Wildfire Simulator")
        self.iface = plugin.iface

        # Initialize wind speed and direction with default values
        self.resolution = 50
        self.wind_speed = 30 # Default wind speed
        self.wind_direction = 0 # Default wind direction

        # Set up rubber bands
        self.plugin.rubber_band = QgsRubberBand(self.plugin.iface.mapCanvas(), QgsWkbTypes.PolygonGeometry)
        self.plugin.rubber_band.setColor(QColor(Qt.green))
        self.plugin.rubber_band.setWidth(2)

        self.plugin.fire_origin_rubber_band = QgsRubberBand(self.plugin.iface.mapCanvas(), QgsWkbTypes.PolygonGeometry)
        self.plugin.fire_origin_rubber_band.setColor(QColor(Qt.red))
        self.plugin.fire_origin_rubber_band.setWidth(2)

        # Set up map selection tools
        self.map_tool = RegionSelectionTool(self.iface, self, self.plugin.rubber_band, is_ignited_region=False)
        self.fire_origin_map_tool = RegionSelectionTool(self.iface, self, self.plugin.fire_origin_rubber_band, is_ignited_region=True)

        self.layout = QVBoxLayout()

        self.refresh_button = QPushButton("Refresh Layers")
        self.refresh_button.clicked.connect(self.refresh)
        self.layout.addWidget(self.refresh_button)

        # --- DTM layer dropdown ---
        self.dtm_label = QLabel("Select Elevation Layer:")
        self.layout.addWidget(self.dtm_label)
        self.dtm_selector = QComboBox()
        self.populate_raster_layers(self.dtm_selector)
        self.layout.addWidget(self.dtm_selector)

        # --- Landcover layer dropdown ---
        self.landcover_label = QLabel("Select Landcover Layer:")
        self.layout.addWidget(self.landcover_label)
        self.landcover_selector = QComboBox()
        self.populate_raster_layers(self.landcover_selector)
        self.layout.addWidget(self.landcover_selector)

        # --- Buttons ---
        self.select_button = QPushButton("Select Simulation Area")
        self.select_button.clicked.connect(self.activate_selection)
        self.layout.addWidget(self.select_button)

        self.confirm_selection_button = QPushButton("Confirm Simulation Area")
        self.confirm_selection_button.clicked.connect(self.confirm_drawn_area)
        self.layout.addWidget(self.confirm_selection_button)

        self.fire_origin_button = QPushButton("Select Ignited Area")
        self.fire_origin_button.clicked.connect(self.activate_fire_origin_selection)
        # self.fire_origin_button.setEnabled(False)  # Disabled by default
        self.layout.addWidget(self.fire_origin_button)

        self.confirm_ignited_button = QPushButton("Confirm Ignited Area")
        self.confirm_ignited_button.clicked.connect(self.confirm_ignited_region)
        # self.confirm_ignited_button.setEnabled(False)  # Disabled by default
        self.layout.addWidget(self.confirm_ignited_button)

        self.clear_button = QPushButton("Clear Selected Areas")
        self.clear_button.clicked.connect(self.clear_selection)
        self.layout.addWidget(self.clear_button)

        # --- Wind Speed Slider ---
        self.wind_speed_label = QLabel(f"Wind Speed: {self.wind_speed} km/h")
        self.layout.addWidget(self.wind_speed_label)

        self.wind_speed_slider = QSlider(Qt.Horizontal)
        self.wind_speed_slider.setMinimum(0)  # Minimum wind speed
        self.wind_speed_slider.setMaximum(100)  # Maximum wind speed
        self.wind_speed_slider.setValue(self.wind_speed)  # Default value
        self.wind_speed_slider.valueChanged.connect(self.update_wind_speed)
        self.layout.addWidget(self.wind_speed_slider)

        # --- Wind Direction Slider ---
        self.wind_direction_label = QLabel(f"Wind Direction: {self.wind_direction}°")
        self.layout.addWidget(self.wind_direction_label)

        self.wind_direction_slider = QSlider(Qt.Horizontal)
        self.wind_direction_slider.setMinimum(0)  # Minimum wind direction (0°)
        self.wind_direction_slider.setMaximum(360)  # Maximum wind direction (360°)
        self.wind_direction_slider.setValue(self.wind_direction)  # Default value
        self.wind_direction_slider.valueChanged.connect(self.update_wind_direction)
        self.layout.addWidget(self.wind_direction_slider)

        self.resolution_label = QLabel(f"Resolution: {self.resolution}m")
        self.layout.addWidget(self.resolution_label)
        self.resolution_slider = QSlider(Qt.Horizontal)
        self.resolution_slider.setMinimum(1)  # min resolution
        self.resolution_slider.setMaximum(1000)  # Max resolution
        self.resolution_slider.setValue(self.resolution)  # Default value
        self.resolution_slider.valueChanged.connect(self.update_resolution)
        self.layout.addWidget(self.resolution_slider)

        self.convert_button = QPushButton("Prepare Simulation Scenario")
        self.convert_button.clicked.connect(self.convert_to_json)
        self.layout.addWidget(self.convert_button)
        '''
        self.cadmium_button = QPushButton("Start Simulation")
        self.cadmium_button.clicked.connect(self.run_cadmium)
        self.layout.addWidget(self.cadmium_button)

        self.cancel_cadmium_button = QPushButton("End Simulation")
        self.cancel_cadmium_button.clicked.connect(self.end_cadmium_run)
        self.layout.addWidget(self.cancel_cadmium_button)
        self.cancel_cadmium_button.hide()

        self.cadmium_proc = None
        '''
        container = QWidget()
        container.setLayout(self.layout)
        self.setWidget(container)
        
    def refresh(self):
        self.dtm_selector.clear()
        self.landcover_selector.clear()
        self.populate_raster_layers(self.dtm_selector)
        self.populate_raster_layers(self.landcover_selector)

    def populate_raster_layers(self, selector):
        """Populate the dropdown with GeoTIFF (raster) layers."""
        selector.clear()
        layers = QgsProject.instance().mapLayers().values()
        for layer in layers:
            if isinstance(layer, QgsRasterLayer):
                ds = layer.dataProvider().dataSourceUri().lower()
                if ds.endswith('.tif') or ds.endswith('.tiff'):
                    selector.addItem(layer.name(), layer)

    def update_wind_speed(self, value):
        """Update the wind speed when the slider is moved."""
        self.wind_speed = value
        self.wind_speed_label.setText(f"Wind Speed: {self.wind_speed} km/h")
        print(f"Wind speed updated to: {self.wind_speed} m/min")

    def update_wind_direction(self, value):
        """Update the wind direction when the slider is moved."""
        self.wind_direction = value
        self.wind_direction_label.setText(f"Wind Direction: {self.wind_direction}°")
        print(f"Wind direction updated to: {self.wind_direction}°")

    def update_resolution(self, value):
        """Update the resolution when the slider is moved."""
        self.resolution = value
        self.resolution_label.setText(f"Resolution: {self.resolution}m")
        print(f"Resolution updated to: {self.resolution}m")

    def activate_selection(self):
        """Activate the polygon drawing tool."""
        self.plugin.iface.mapCanvas().setMapTool(self.map_tool)
        print("Draw tool activated. Draw a polygon on the map.")

    def activate_fire_origin_selection(self):
        """Activate the fire origin selection tool."""
        if self.plugin.selected_region:
            self.plugin.iface.mapCanvas().setMapTool(self.fire_origin_map_tool)
            print("Fire origin drawing tool activated. Draw a polygon within the selected region.")
        else:
            print("No selected region. Please draw a selected region first.")

    def confirm_drawn_area(self):
        """Confirm the drawn polygon and set it as the selected region."""
        if self.map_tool.points:
            self.plugin.selected_region = QgsGeometry.fromPolygonXY([self.map_tool.points])
            print("Selected region confirmed.")
            # Enable the fire origin button since a selected region exists
        #     self.fire_origin_button.setEnabled(True)
        # else:
        #     print("No polygon drawn. Please draw a polygon first.")
        #     # Disable the fire origin button if no selected region exists
        #     self.fire_origin_button.setEnabled(False)
    
    def confirm_ignited_region(self):
        """Confirm the drawn polygon and set it as the ignited region."""
        if self.fire_origin_map_tool.points:
            self.plugin.ignited_region = QgsGeometry.fromPolygonXY([self.fire_origin_map_tool.points])
            print("Ignited region confirmed.") 
            # self.confirm_ignited_button.setEnabled(False)  # Disable the button after confirmation
        else:
            print("No polygon drawn. Please draw a polygon first.")
    
    def activate_fire_origin_selection(self):
        """Activate the fire origin selection tool."""
        if self.plugin.selected_region:
            self.plugin.iface.mapCanvas().setMapTool(self.fire_origin_map_tool)
            # self.confirm_ignited_button.setEnabled(True)  # Enable the button
            print("Fire origin drawing tool activated. Draw a polygon within the selected region.")
        else:
            print("No selected region. Please draw a selected region first.")

    def clear_selection(self):
        """Clear the current selection and rubber band."""
        if self.plugin.rubber_band:
            self.plugin.selected_region = None
            self.plugin.rubber_band.reset()
            self.plugin.rubber_band = None
            self.plugin.iface.mapCanvas().refresh()
        if self.plugin.fire_origin_rubber_band:
            self.plugin.ignited_region = None
            self.plugin.fire_origin_rubber_band.reset()
            self.plugin.fire_origin_rubber_band = None
            self.plugin.iface.mapCanvas().refresh()
        self.map_tool.points = []
        self.fire_origin_map_tool.points = []
        self.plugin.iface.mapCanvas().setMapTool(None)
        # self.fire_origin_button.setEnabled(False)
        # self.confirm_ignited_button.setEnabled(False)  # Disable the button
        print("Selection cleared!")

    def clip_ignited_region(self, json_file_path):
        """Clip the ignited region as a binary raster and resample it to match the slope raster."""
        if not self.plugin.ignited_region:
            print("No ignited region to clip.")
            return None

        # Create an in-memory mask layer from the ignited region polygon
        mask_layer = createTemporaryPolygonLayer(self.plugin.ignited_region)

        # Create a binary raster with the same dimensions as the slope raster
        dtm_layer = self.dtm_selector.currentData()
        if not dtm_layer:
            print("No dtm layer selected.")
            return None

        # Determine output file path for the clipped ignited region raster
        
        ignited_raster_path_temp = os.path.splitext(json_file_path)[0] + f"_ignited_temp.tif"
        ignited_raster_path = os.path.splitext(json_file_path)[0] + f"_ignited.tif"

        # Use the GDAL Clip algorithm to clip the raster using the mask
        params = {
            "INPUT": dtm_layer.source(),
            "MASK": mask_layer,
            "CROP_TO_CUTLINE": True,
            "OUTPUT": ignited_raster_path_temp
        }

        # Run the clip operation
        processing.run("gdal:cliprasterbymasklayer", params)

        # Open the slope raster to get its dimensions and transform
        with rasterio.open(dtm_layer.source()) as slope_src:
            slope_height = slope_src.height
            slope_width = slope_src.width
            slope_transform = slope_src.transform
            slope_crs = slope_src.crs

        # Open the clipped ignited raster
        with rasterio.open(ignited_raster_path_temp) as ignited_src:
            ignited_data = ignited_src.read(1)
            ignited_transform = ignited_src.transform
            ignited_crs = ignited_src.crs

            # Create an empty array for the resampled ignited raster
            resampled_ignited_data = numpy.zeros((slope_height, slope_width), dtype=numpy.uint8)

            # Resample the ignited raster to match the slope raster
            rasterio._warp._reproject(
                source=ignited_data,
                destination=resampled_ignited_data,
                src_transform=ignited_transform,
                src_crs=ignited_crs,
                dst_transform=slope_transform,
                dst_crs=slope_crs,
                resampling=rasterio._warp.Resampling.nearest
            )

            # Save the resampled ignited raster
            with rasterio.open(ignited_raster_path, 'w', driver='GTiff', height=slope_height, width=slope_width,
                            count=1, dtype=resampled_ignited_data.dtype, crs=slope_crs, transform=slope_transform) as dst:
                dst.write(resampled_ignited_data, 1)

        return ignited_raster_path

    def convert_to_json(self):
        """Clip the selected GeoTIFFs to the drawn area, process them, and output JSON."""
        dtm_layer = self.dtm_selector.currentData()
        landcover_layer = self.landcover_selector.currentData()

        if dtm_layer and landcover_layer and self.plugin.selected_region and self.plugin.selected_region.isGeosValid():
            # json_file_path, _ = QFileDialog.getSaveFileName(self, "Save JSON File", "", "JSON Files (*.json);;All Files (*)")
            # if not json_file_path:
            #     print("No JSON file path provided.")
            #     return

            # Create an in-memory mask layer from the drawn polygon
            mask_layer = createTemporaryPolygonLayer(self.plugin.selected_region)

            root = os.path.dirname(os.path.abspath(__file__))
            slope_path = os.path.join(root, "slope.tif")
            aspect_path = os.path.join(root, "aspect.tif")
            json_file_path = os.path.join(root, "map.json")

            # Determine output file paths for the clipped rasters
            clipped_dtm_path = os.path.splitext(json_file_path)[0] + "_dtm.tif"
            clipped_land_path = os.path.splitext(json_file_path)[0] + "_landcover.tif"

            # Use the GDAL Clip algorithm to clip the rasters using the mask
            params_dtm = {
                "INPUT": dtm_layer.source(),
                "MASK": mask_layer,
                "CROP_TO_CUTLINE": True,
                "OUTPUT": clipped_dtm_path
            }
            params_land = {
                "INPUT": landcover_layer.source(),
                "MASK": mask_layer,
                "CROP_TO_CUTLINE": True,
                "OUTPUT": clipped_land_path
            }

            processing.run("gdal:cliprasterbymasklayer", params_dtm)
            processing.run("gdal:cliprasterbymasklayer", params_land)

            # Clip the ignited region as a binary raster
            ignited_raster_path = self.clip_ignited_region(json_file_path)

            # Calculate slope and aspect
            feedback = QgsProcessingFeedback()
            processing.run("qgis:slope", {
                "INPUT": dtm_layer,
                "Z_FACTOR": 1.0,
                "OUTPUT": slope_path, 
            }, feedback=feedback)
            processing.run("qgis:aspect", {
                "INPUT": dtm_layer,
                "Z_FACTOR": 1.0,
                "OUTPUT": aspect_path, 
            }, feedback=feedback)

            slope_layer = QgsRasterLayer(slope_path, "slope")
            aspect_layer = QgsRasterLayer(aspect_path, "aspect")
            if not slope_layer.isValid():
                print("Failed to load slope layer!")
                return
            if not aspect_layer.isValid():
                print("Failed to load aspect layer!")
                return
            QgsProject.instance().addMapLayer(slope_layer, False)
            QgsProject.instance().addMapLayer(aspect_layer, False)

            # Prepare paths for further processing
            paths = {
                "aspect": aspect_path,
                "slope": slope_path,
                "elevation": dtm_layer.source(),
                "land": clipped_land_path,
                "ignited": ignited_raster_path,
                "json": json_file_path,
            }

            # Resample and align the landcover raster to match the slope and aspect rasters
            with rasterio.open(paths["slope"]) as slope_src:
                slope_transform = slope_src.transform
                slope_crs = slope_src.crs
                slope_width = slope_src.width
                slope_height = slope_src.height

            with rasterio.open(paths["land"]) as land_src:
                land_data = land_src.read(1)
                land_transform = land_src.transform
                land_crs = land_src.crs

                # Resample landcover to match slope raster
                resampled_land_data = numpy.empty((slope_height, slope_width), dtype=numpy.float32)
                rasterio._warp._reproject(
                    source=land_data,
                    destination=resampled_land_data,
                    src_transform=land_transform,
                    src_crs=land_crs,
                    dst_transform=slope_transform,
                    dst_crs=slope_crs,
                    resampling=rasterio._warp.Resampling.nearest
                )

                # Save the resampled landcover raster
                resampled_land_path = os.path.splitext(json_file_path)[0] + "_landcover_resampled.tif"
                with rasterio.open(resampled_land_path, 'w', driver='GTiff', height=slope_height, width=slope_width,
                                count=1, dtype=resampled_land_data.dtype, crs=slope_crs, transform=slope_transform) as dst:
                    dst.write(resampled_land_data, 1)

                paths["land"] = resampled_land_path

            dump_json(paths, self)
            print("JSON conversion completed.")
        else:
            print("No valid layers or selected region. Cannot convert to JSON.")
    '''
    def on_cadmium_finish_running(self, csv_path):
        uri = f"file:///{csv_path}?delimiter=,&xField=x&yField=y&crs=EPSG:2959"
        layer = QgsVectorLayer(uri, "ignition", "delimitedtext")
        if not layer.isValid():
            print("Failed to load layer!")
            return
        QgsProject.instance().addMapLayer(layer)
        props = layer.temporalProperties()
        props.setIsActive(True)
        props.setAccumulateFeatures(True)
        props.setMode(QgsVectorLayerTemporalProperties.TemporalMode.ModeFeatureDateTimeInstantFromField)
        props.setStartField("time")

    def run_cadmium(self):
        if self.cadmium_proc:
            return
        root = os.path.dirname(os.path.abspath(__file__))
        simulator = os.path.join(root, "bin/wildfire")
        map_json = os.path.join(root, "config/map.json")
        map_csv = os.path.join(root, "ignition.csv")
        def callback():
            self.cadmium_proc = subprocess.Popen([simulator, map_json, map_csv])
            self.cancel_cadmium_button.show()
            self.cadmium_proc.wait()
            self.end_cadmium_run()
            self.on_cadmium_finish_running(map_csv)
            return
        self.cadmium_button.hide()
        thread = threading.Thread(target=callback)
        thread.start()
    
    def end_cadmium_run(self):
        if not self.cadmium_proc:
            return
        self.cancel_cadmium_button.hide()
        self.cadmium_proc.kill()
        self.cadmium_button.show()
        self.cadmium_proc = None
    '''
class RegionSelectionTool(QgsMapToolEmitPoint):
    def __init__(self, iface, plugin, rubber_band, is_ignited_region=False):
        super(RegionSelectionTool, self).__init__(iface.mapCanvas())
        self.points = []  # List to store clicked QgsPointXY objects
        self.plugin = plugin
        self.rubber_band = rubber_band
        self.is_ignited_region = is_ignited_region  # Flag to indicate if this is for the ignited region

    def canvasPressEvent(self, event):
        point = self.toMapCoordinates(event.pos())

        # If this is for the ignited region, enforce the selected region constraint
        if self.is_ignited_region:
            if not self.plugin.selected_region or not self.plugin.selected_region.contains(point):
                print("Point is outside the selected region. Ignoring.")
                return

        # Automatically close the polygon if the user clicks near the first point
        if len(self.points) > 2 and self.is_near_first_point(point):
            self.points.append(self.points[0])
            if self.is_ignited_region:
                self.plugin.ignited_region = QgsGeometry.fromPolygonXY([self.points])
                print("Polygon closed and ignited region set.")
                print(f"Ignited region geometry: {self.plugin.ignited_region.asWkt()}")  # Debug print
            else:
                self.plugin.selected_region = QgsGeometry.fromPolygonXY([self.points])
                print("Polygon closed and selected region set.")
            self.highlight_region()
            return

        # Add the new point to the list
        self.points.append(point)
        self.highlight_region()

    def is_near_first_point(self, point, tolerance=50):
        """Check if the current point is near the first point (within a certain tolerance)."""
        first_point = self.points[0]
        dist = math.sqrt((first_point.x() - point.x())**2 + (first_point.y() - point.y())**2)
        return dist <= tolerance

    def highlight_region(self):
        """Highlight the drawn polygon on the map."""
        self.rubber_band.reset()
        for p in self.points:
            self.rubber_band.addPoint(p)

    def clear_highlight(self):
        """Clear the drawn polygon from the map."""
        self.rubber_band.reset()
        self.plugin.iface.mapCanvas().refresh()

#########################
# HELPER FUNCTIONS
#########################

def createTemporaryPolygonLayer(geometry):
    """
    Creates an in-memory vector layer (Polygon) with the given geometry.
    This layer will be used as a mask for clipping.
    """
    crs = QgsProject.instance().crs().authid()
    mem_layer = QgsVectorLayer(f"Polygon?crs={crs}", "mask", "memory")
    prov = mem_layer.dataProvider()
    feat = QgsFeature()
    feat.setGeometry(geometry)
    prov.addFeatures([feat])
    mem_layer.updateExtents()
    return mem_layer

def read_raster(path):
    """Read a raster file and return its data and metadata."""
    with rasterio.open(path) as src:
        data = src.read(1)  # Read the first band
        transform = src.transform
        crs = src.crs
        return data, transform, crs

def dump_json(paths, widget):
    """Read raster data and populate the JSON file."""
    # Read raster data
    slope_data, slope_transform, _ = read_raster(paths['slope'])
    aspect_data, _, _ = read_raster(paths['aspect'])
    elevation_data, _,_ = read_raster(paths['elevation'])
    landcover_data, _, _ = read_raster(paths['land'])
    ignited_data, _, _ = read_raster(paths['ignited']) if paths.get('ignited') else (None, None, None)

    resolution = widget.resolution

    # Get dimensions
    height, width = slope_data.shape

    # Initialize JSON structure
    data = {
        "cells": {
            "default": {
                "delay": "inertial"
            }
        }
    }

    shift = 0

    for row in range(0, height, resolution):
        for col in range(int(-0.5*resolution), int(width+0.5*resolution), resolution):
            if shift % 2:
                col = int(col+(0.5*resolution))

            if col >= width:
                continue
            if col < 0:
                continue

            slope_value = slope_data[row][col]
            aspect_value = aspect_data[row][col]
            landcover_value = landcover_data[row][col]
            ignited_value = ignited_data[row][col] if ignited_data is not None else 0
            elevation_value = elevation_data[row][col]
           

            # Skip invalid values
            if elevation_value <= -9999.0:
                continue

            # Map landcover value to fuel model
            try:
                fuel = FUELS[int(landcover_value)]
            except KeyError:
                continue

            # Get cell coordinates
            x, y = slope_transform * (col, row)
            cell_name = f"{int(x)}_{int(y)}"

            # Set ignited to true or false based on the ignited raster
            ignited = bool(ignited_value) if ignited_data is not None else False

            # Add cell to JSON
            data["cells"][cell_name] = {
                "neighborhood": {},
                "state": {
                    #"slope": float(slope_value),
                    #"aspect": float(aspect_value),
                    "fuelModelNumber": fuel,
                    "windDirection": widget.wind_direction,
                    "windSpeed": widget.wind_speed,
                    "x": float(x),
                    "y": float(y),
                    "elevation": float(elevation_value),
                    "ignited": ignited  # Set as true or false
                }
            }
            data["cells"][cell_name]["neighborhood"][cell_name] = 0
            
            neighborhood = [(1,0.5),(1,-0.5),(-1,0.5),(-1,-0.5),(0,1),(0,-1)]
            for neighbor in neighborhood:

                r = row + int(neighbor[0] * resolution)
                c = col + int(neighbor[1] * resolution)

                # Skip out-of-bounds neighbors
                if c < 0 or r < 0 or c >= width or r >= height:
                    continue
                neighbor_elevation_value = elevation_data[r][c]
                if neighbor_elevation_value < -9998.0:
                    continue
                    
                landcover_value = landcover_data[r][c]
                
                try:
                    fuel = FUELS[int(landcover_value)]
                except KeyError:
                    continue
                

                # Get neighbor coordinates
                neighbor_x, neighbor_y = slope_transform * (c, r)
        
                neighbor_name = f"{int(neighbor_x)}_{int(neighbor_y)}"

                # Add neighbor to neighborhood
                if neighbor == (0,1) or neighbor == (0,-1):
                    data["cells"][cell_name]["neighborhood"][neighbor_name] = resolution
                else:
                    data["cells"][cell_name]["neighborhood"][neighbor_name] = 1.118*resolution
               
        shift += 1

    # Write JSON to file
    with open(paths['json'], "w") as f:
        json.dump(data, f, indent=4)
    print(f"JSON file saved to: {paths['json']}")
