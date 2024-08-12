# CaptureSamples

The CaptureSamples Gazebo plugin was developed for the platform Ubuntu 20.04 Focal, 
Gazebo version Gazebo Garden

Its purpose is to prepare training data consisting of a sequence of images and
poses of some objects in the images. The 3D geometry of the objects is known.

## Install and build

Install Gazebo Garden as described in https://gazebosim.org/docs/latest/getstarted/
The further description uses ~/workspace/capture_samples directory for the plugin 
installation, but it can be an arbitrary directory 

~~~
mkdir ~/workspace
cd ~/workspace
git clone https://github.com/sdarkhovsky/capture_samples.git
cd ~/workspace/capture_samples
mkdir build
cd build
cmake ..
make
~~~

This will generate the `CaptureSamples` library under `build`.

## Run

Add the library to the path:
~~~
cd ~/workspace/capture_samples
export GZ_GUI_PLUGIN_PATH=`pwd`/build
~~~

The plugin functionality is demonstrated on the examples of two worlds.
We refer to the first world as depot_foldable_chair. It contains the models Depot and 
foldable_chair (https://app.gazebosim.org/home). Its SDF file is at
~/workspace/capture_samples/depot_foldable_chair_base_dir

We refer to the second world as shapes. It contains the stock Gazebosim models 'box' 
and 'capsule'. Its SDF file is at ~/workspace/capture_samples/shapes_base_dir

You are welcome to create your own models and worlds by the examples. 


To run the depot_foldable_chair example:
~~~
cd ~/workspace/capture_samples
export SCENE_CAPTURE_PATH=`pwd`/depot_foldable_chair_base_dir
gz sim -v 4 $SCENE_CAPTURE_PATH/depot_foldable_chair.sdf
~~~

To run the shapes example:
~~~
cd ~/workspace/capture_samples
export SCENE_CAPTURE_PATH=`pwd`/shapes_base_dir
gz sim -v 4 $SCENE_CAPTURE_PATH/capture_samples_shapes.sdf
~~~

The Capture Samples and other plugins are automatically loaded in the right panel
of the Gazebo GUI.

## Create trajectory of models. 

### Define waypoints of the trajectory for one or mode models.

1. Click on a model to select it.

2. Click on the left button "Capture pose of the selected model" in the Capture Samples
plugin panel.

3. Click on the Translate Mode or Rotate Mode buttons in the Transform Control plugin panel
Move the model along arrows or arcs.

Repeat the steps 1-3 as many times as necessary.

To create trajectories of multiple models repeat the steps 1-3 for each model for each
waypoint.

### Generate the trajectory

Click on the right button "Generate Scene Capture Parameters file from the captured poses"
in the Capture Samples plugin panel.
The randomized trajectory is interpolated from the waypoints and stored in the file
$SCENE_CAPTURE_PATH/SceneCaptureParams.xml.

## Create images of the scenes with moving models. 

Click "Run the simulation" button in the World control panel.
The trajectories are replayed. The screenshot images are captured automatically
as the models move along the trajectories.
The images are stored in the directory $SCENE_CAPTURE_PATH/pictures

The images at $SCENE_CAPTURE_PATH/pictures together with the model poses
in $SCENE_CAPTURE_PATH/SceneCaptureParams.xml can be used as training data.
