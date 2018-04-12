# CS - Library: Gridmaps
This library contains implementations of two-dimensional gridmaps. The code is open-source ([BSD License](LICENSE)) and has been tested under Ubuntu 16.04 with ROS Kinetic. Please note that this project is part of ongoing research and that there will be changes in the future.

## Structure
This package consists of several subfolders:

* [static\_maps](cslibs_gridmaps/include/cslibs_gridmaps/static_maps/) and [dynamic\_maps](cslibs_gridmaps/include/cslibs_gridmaps/dynamic_maps/) contain map implementations for maps with *static* and *dynamic* size, respectively. Chunking allows *dynamic* maps to efficiently grow over time.
* [conversion](cslibs_gridmaps/include/cslibs_gridmaps/static_maps/conversion/) contains methods to convert static gridmaps from and to ROS messages of type ``nav_msgs::OccupancyGrid``.
* [serialization](cslibs_gridmaps/include/cslibs_gridmaps/serialization/) contains methods to serialize and de-serialize gridmaps using *YAML*.

## Usage

### Dependencies
This library depends on the following packages of our research group:

* [cslibs\_indexed\_storage](https://github.com/cogsys-tuebingen/cslibs_indexed_storage)
* [cslibs\_math](https://github.com/cogsys-tuebingen/cslibs_math)
* [cslibs\_utility](https://github.com/cogsys-tuebingen/cslibs_utility)

### Examples
Exemplary usage of these maps can be found in [cslibs\_mapping](https://github.com/cogsys-tuebingen/cslibs_mapping/tree/master/src/mapper).

## Contributing
[Contribution guidelines for this project](CONTRIBUTING.md)
