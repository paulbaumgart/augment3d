# augment3d

Perform data augmentation on your point clouds, to help your neural networks generalize better.


### Installing Prerequisites

augment3d relies on the [PointCloudLibrary](http://pointclouds.org/documentation/), version 1.7 or later.

On Linux: see [Prebuilt PCL binaries for Linux](http://pointclouds.org/downloads/linux.html)

On Mac, using [homebrew](https://brew.sh/):

    brew install homebrew/science/pcl


### Building

    mkdir build
    cd build
    cmake ..
    make augment3d


### Running

    build/augment3d -reflect|-rotate|-scale input_file output_dir

Options:

    -reflect
        augment input point cloud data by reflecting it over a plane through the origin
    -reflect_normal
        vector normal to the reflection plane
        default: "0,1,0"

    -rotate
        augment input point cloud data by rotating it along an axis
    -rotate_axis
        rotation axis
        default: "0,1,0"
    -rotate_from
        rotation start angle, in degrees
        default: -90
    -rotate_to
        rotation end angle, in degrees
        default: 90
    -rotate_steps
        number of rotation steps, linearly interpolated between rotate_from and rotate_to (must be 2 or more)
        default: 5

    -scale
        augment input point cloud data by scaling its dimensions
    -scale_from
        x,y,z values from which to start scaling
        default: "1,1,1"
    -scale_to
        x,y,z values at which to stop scaling
        default: "2,2,2"
    -scale_steps
        number of scaling steps, linearly interpolated between scale_from and scale_to (must be 2 or more)
        default: 2


### Unit Tests

    cd build
    make unittests && ./unittests
