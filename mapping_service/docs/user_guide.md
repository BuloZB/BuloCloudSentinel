# Persistent Mapping Module User Guide

## Overview

The Persistent Mapping Module allows you to generate orthomosaic images and 3D terrain meshes from drone imagery. This guide will walk you through the process of creating a mapping project, uploading images, processing them, and viewing the results.

## Getting Started

### Creating a Project

1. Navigate to the Maps tab in the SentinelWeb interface.
2. Click the "New Project" button.
3. Enter a name and description for your project.
4. Click "Create" to create the project.

### Uploading Images

1. Select your project from the project list.
2. Click the "Upload Images" button.
3. Select the images you want to upload. You can select multiple images at once.
4. Click "Upload" to upload the images.

**Note:** For best results, use geotagged images with GPS coordinates. The module supports JPEG and TIFF formats.

### Processing Images

1. After uploading images, click the "Process" button.
2. Configure the processing options:
   - **Orthophoto Resolution:** The resolution of the orthomosaic in cm/pixel.
   - **Mesh Size:** The number of faces in the 3D mesh.
   - **Point Cloud Quality:** The quality of the point cloud (low, medium, high).
   - **Feature Quality:** The quality of feature extraction (low, medium, high).
3. Click "Start Processing" to start the processing job.

**Note:** Processing can take several minutes to hours depending on the number and size of images, and the selected processing options.

### Viewing Results

1. After processing is complete, you can view the results in the project details page.
2. The following assets will be available:
   - **Orthomosaic:** A high-resolution aerial image of the mapped area.
   - **3D Mesh:** A 3D model of the mapped area.
   - **Digital Surface Model (DSM):** A height map of the mapped area including objects.
   - **Digital Terrain Model (DTM):** A height map of the mapped area excluding objects.
3. Click on an asset to view it in the map viewer.

## Map Viewer

The map viewer allows you to explore the mapping results in 2D and 3D.

### Controls

- **Pan:** Click and drag to pan the map.
- **Zoom:** Use the mouse wheel or the zoom controls to zoom in and out.
- **Rotate:** Right-click and drag to rotate the 3D view.
- **Tilt:** Middle-click and drag to tilt the 3D view.

### Layers

The map viewer supports multiple layers:

- **Orthomosaic:** The 2D aerial image.
- **3D Mesh:** The 3D model of the mapped area.
- **Digital Surface Model:** A height map of the mapped area including objects.
- **Digital Terrain Model:** A height map of the mapped area excluding objects.

You can toggle layers on and off using the layer control panel.

### Measurement Tools

The map viewer includes measurement tools:

- **Distance:** Measure the distance between points.
- **Area:** Measure the area of a polygon.
- **Height:** Measure the height of a point.

To use a measurement tool:

1. Click the measurement tool button.
2. Click on the map to place measurement points.
3. Double-click to complete the measurement.

### Timeline

If you have multiple versions of a mapping project, you can use the timeline to view different versions:

1. Click the timeline button to show the timeline.
2. Drag the timeline slider to select a date.
3. The map will update to show the mapping data from the selected date.

## Best Practices

### Image Capture

For best results, follow these guidelines when capturing images:

- **Overlap:** Ensure at least 70% forward overlap and 60% side overlap between images.
- **Altitude:** Maintain a consistent altitude throughout the flight.
- **Angle:** Capture images at nadir (straight down) for orthomosaic generation.
- **Lighting:** Avoid strong shadows and reflections by flying on overcast days or during mid-day.
- **Resolution:** Use the highest resolution available on your camera.
- **Geotags:** Ensure your images are geotagged with accurate GPS coordinates.

### Processing

For optimal processing results:

- **Image Selection:** Remove blurry or poorly exposed images before processing.
- **Resolution:** Choose an appropriate resolution based on your needs. Higher resolution requires more processing time.
- **Quality Settings:** Higher quality settings produce better results but require more processing time.
- **Area Coverage:** Process smaller areas at a time for faster results.

## Troubleshooting

### Common Issues

#### Images Not Uploading

- Ensure the images are in a supported format (JPEG, TIFF).
- Check that the images are not too large. The maximum file size is 50MB per image.
- Verify your internet connection is stable.

#### Processing Fails

- Check that your images have sufficient overlap.
- Ensure the images are properly geotagged.
- Try processing with lower quality settings.
- Reduce the number of images being processed.

#### Poor Quality Results

- Ensure your images have sufficient overlap.
- Check that your images are properly exposed and in focus.
- Try processing with higher quality settings.
- Ensure your images are captured at a consistent altitude and angle.

### Getting Help

If you encounter issues not covered in this guide, please contact support at support@bulo.cloud.
