# Welcome!
This project is originally designed for self study about computer graphics, however, during the studying process, some useful libraries are developed, I will move them out when they become mature, I will list some operational libraries in the following paragraphs.
# Marching Cubes 33
This code is currently located in the directory: ~/hello_mc33/normal_implement. The dvega68 under the directory ~/hello_mc33 is cloned from dvega68 github user's implementation, and the without_table is for a crazy attempt about implementing interpolate trianges in cubes without direct mc33 tables, but with some rules(maybe will be continued in the future).
## Build
Currently, this project is just for self use, but if you want to compile it locally, here is the hint.
```text
1. download libigl and compile it to binary library
2. download Eigen
3. install vcpkg, and use it to install packages: GTest/Boost/Protobuf.
4. modify the CMakeLists.txt, change the CMAKE_TOOLCHAIN_FILE to your installed vcpkg path, change the eigen and libigl path.
```
I'm sure the above-mentioned steps will be sufficient for build the library in your local environment. If you have problems, feel ease to contact us. Email: hanwein2@gmail.com.
## Usage
We provide two interfaces in mc33.h, given a signed_distance file, it will generate obj type triangle mesh. For generate signed distance file, we also provide interfaces in signed_distances_iface.h, given a obj type triangle mesh, it will generate signed distance file. We save signed distance file by protobuf, you can check the proto file under the ~/hello_mc33/normal_implement/proto directory. Here is an example about using signed distance file to generate and save mesh file as obj.
```c++
std::string sd_path = "./signed_distances.pb";
std::string save_path = "./proto_bunny.obj";
generate_mesh(sd_path, save_path);
```
We also build a **execute binary**, which is integrated in a visualized tool. You can find it [here](https://github.com/hanxiaowein1/charles_mesh/releases/tag/v0.2). In this visualized tool, you can choose a signed distance file, generate mesh and save it in directory.
## Output Examples
![mc33 bunny](images/mc33_20x20x20.png)
*<p align="center">Generated with 20\*20\*20 signed distance</p>*

## Core Ideas
### Generate all tables
Use original 33 cases from mc33 paper, then rotate it along the directions x/y/z, so we will rotate it 4 * 4 * 4 times, flip it along the planes xy/xz/yz, and finally invert it signed distance(turn positive into negative, vice versa). After such operations, we are be able to get all marching cubes33 tables.
### Cache
Notice that adjacent cubes have shared edges.
```text
  Vertices:            Edges:               Faces:
    3 ___________2        _____2______         ____________
   /|           /|      /|           /|      /|           /|
  / |          / |     B |          A |     / |    2     / |
7/___________6/  |    /_____6_____ /  |    /___________ /  |
|   |        |   |   |   3        |   1   |   |     4  |   |
|   |        |   |   |   |        |   |   | 3 |        | 1 |     z
|   0________|___1   |   |_____0__|___|   |   |_5______|___|     |
|  /         |  /    7  /         5  /    |  /         |  /      |____y
| /          | /     | 8          | 9     | /      0   | /      /
4/___________5/      |/_____4_____|/      |/___________|/      x
```
Based on the index of vertex/edge/face of the above text. We can imagine that if you along the x+ direction, and the cube is not the first cube, it will share same face 4 which is the face 5 of (x - 1) cube. Based on this observation, we can reduce significantly duplicate computation. Currectly we use c++ unordered_map as the type of our storage, and use (z,y,x,edge_index) as the key, of course if it missed, it will search it on duplicate edge on other cubes. If nothing hitted, it will finally compute and generate a new value in the cache.
## Issues
Although we write unit tests for nearly all functions, it's still hard to write tests for original case configs. Currently there are still problems, we're working on it. However, if you can provide suggestions to detect the configuration issues, we would be very grateful!