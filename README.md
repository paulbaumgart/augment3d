# augment3d

**Work in progress!**

### Prerequisites

    brew install boost --c++11
    brew install homebrew/science/pcl

### Building

    mkdir build
    cd build
    cmake ..
    make augment3d


### Running

    cd build
    ./augment3d <input_file>


### Testing

    cd build
    make unittests && ./unittests
