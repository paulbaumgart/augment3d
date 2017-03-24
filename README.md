# augment3d

Perform data augmentation on your point clouds, to help your neural networks generalize better.

### Prerequisites

    brew install homebrew/science/pcl

### Building

    mkdir build
    cd build
    cmake ..
    make augment3d


### Running

    build/augment3d -reflect|-rotate|-scale input_file output_dir

Options:

    -reflect (augment input point cloud data by reflecting it over a plane
      through the origin) type: bool default: false
    -reflect_normal (vector normal to the reflection plane) type: string
      default: "0,1,0"

    -rotate (augment input point cloud data by rotating it along an axis)
      type: bool default: false
    -rotate_axis (rotation axis) type: string default: "0,1,0"
    -rotate_from (rotation start angle, in degrees) type: double default: -90
    -rotate_steps (number of rotation steps, linearly interpolated between
      rotate_from and rotate_to(must be 2 or more)) type: int32 default: 5
    -rotate_to (rotation end angle, in degrees) type: double default: 90
    
    -scale (augment input point cloud data by scaling its dimensions)
      type: bool default: false
    -scale_from (x,y,z values from which to start scaling) type: string
      default: "1,1,1"
    -scale_steps (number of scaling steps, linearly interpolated between
      scale_from and scale_to(must be 2 or more)) type: int32 default: 2
    -scale_to (x,y,z values at which to stop scaling) type: string
      default: "2,2,2"



### Testing

    cd build
    make unittests && ./unittests
