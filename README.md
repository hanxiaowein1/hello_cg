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
Based on the index of vertex/edge/face of the above text. We can imagine that if you along the x+ direction, and the cube is not the first cube, it will share same face 4 which is the face 5 of (x - 1) cube. Based on this observation, we can reduce significantly duplicate computation. Currectly we use c++ unordered_map as the type of our storage, and use (z,y,z,edge_index) as the key, of course if it missed, it will search it on duplicate edge on other cubes. If nothing hitted, it will finally compute and generate a new value in the cache.
## Issues
Although we write unit tests for nearly all functions, it's still hard to write tests for original case configs. Currently there is still problems, we're working on it. However, if you can provide suggestions to detect the configuration issues, we would be very grateful!