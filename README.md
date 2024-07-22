# AMRELmp: Automatic Mountain Road Extraction from LiDAR data (multi-platform)

AMREL is a multi-platform software tool to automatically extract roads
from large LiDAR data sets of mountainous areas.
The road extraction is performed in two steps:

* seeds production using LiDAR derived digital terrain model (DTM) tiles,
* road detection using LiDAR raw data (ground 3D points).

A separate step transforms ASCII MNT tiles (.asc) and ground point tiles (.xyz)
into internal NVM and TIL formats.

![Figure](resources/steps/example.png?raw=true)

Authors: Philippe Even and Phuc Ngo, LORIA/ADAGIo, Université de Lorraine.

Reference : Even, P. and Ngo, P., 2021.
[Automatic forest road extraction from LiDAR data of mountainous areas.](https://doi.org/10.1007/978-3-030-76657-3_6)
In International Joint Conference of Discrete Geometry Mathematical Morphology,
Uppsala, Sweden, May 24-17, 2021 (Springer LNCS 12708), pp. 93-106.
[hal-03144147](https://hal.archives-ouvertes.fr/hal-03144147).

## QUICK SETUP / COMPILATION GUIDE
### windows

1. download and install visual studio 2019 and cmake (required)

2. compile and install libraries --
Go to the "tools" directory, then execute: `buildLibs.bat`
then `copyLibraryFiles.bat`

3. generate project files -- Run: `GenerateProjectFiles.bat`

4. compile "AMREL.sln" within visual studio
(use the release configuration for performance reasons),
or compile and run by: `AMREL.bat` (compiles both DEBUG and RELEASE),
then simple run by: `AMREL\binaries\AMREL\Release\AMREL.exe`

Always run from "resources" folder (already configured for visual studio).

### linux

1. compile and install libraries --
Go to the "tools" directory, then execute `buildLibs.sh`
then `copyLibraryFiles.sh`

2. generate project files -- Run `GenerateProjectFiles.sh`

3. compile and start program: `AMREL.sh`

The application can be compiled in debug mode on linux with:
`make config="debug"` (can cause heavy performance issues).

### MacOs

1. compile and install libraries --
Go to the "tools" directory, then execute (in sudo mode) `bash buildLibs.sh`
then `bash copyLibraryFiles.sh`

2. generate project files -- Run `bash GenerateProjectFiles.sh`

3. compile and start program with xCode: `AMREL.xcodeproj`

## HOW TO USE

Extraction results are sent to `resources/steps` directory.
The application is controlled from `resources/config/AMREL.ini` file,
that contains several options:

### NewLidar

When set to `yes`, AMREL only transforms LiDAR inputs into internal NVM
and TIL formats.
It reads ASCII-formatted DTM maps (.asc files) found by default in the
`resources/asc` directory, then XYZ-formatted ground point tiles (.xyz files)
found by default in the `resources/xyz` directory.
Caution: DTM and point tiles must share the same area.
Mostly 500 m x 500 m wide tiles have been tested.
Ascii headers should contain `ncols`, `nrows`, `xllcenter`, `yllcenter`,
`cellsize` and `nodata_value` specifications.

When set to `no`, AMREL extracts roads from internal format tiles (first
a selection of seeds in NVM tiles, then the extraction of forest roads
from TIL point files. Input tiles are listed in the specified tile set
input (`TileSet` option).
The result is a PNG image (`roads.png`) stored into
the `resources/steps` directory.

### TileSet

This option provides the name of an input file, that lists the names
of the tiles to be processed. This file is stored into the
`resources/tilesets` directory, and has a ".txt" suffix.

An example of NVM and TIL files (Gris-Mouton area) are available in
[AMRELtest GitHub repository](https::github.com/evenp/AMRELtest):

1) Copy the files from **AMRELtest/Data/nvm/** directory
into `resources/nvm` directory.

2) Copy the files from **AMRELtest/Data/til/eco/** directory
into `resources/til/eco` directory.

3) Sets the option to grismouton (tile set file is already available
in the `resources/tilesets` directory).

### CloudAccess

This option sets the access speed to the points stored into the TIL files:
'top' is fast, but consumes a huge memory size to store the index table;
'eco' uses much less memory, but is quite slower;
'mid' is a good compromise (recommended).
In practice, it should match the point cloud density.

### SawingPadSize

This option sets the size of the groups of DTM tiles, that are iterately
processed to select seeds. It is recommended when processing large LiDARs
to avoid memory saturation. A pad size of 7 can be considered as a good
compromise between required memory and execution time.

When set to 0, all the tiles are processed at a single stage.

### AsdBufferSize

This option sets the size of the groups of point tiles, that are iterately
processed to extract roads. It is recommended when processing large LiDARs
to avoid memory saturation. A pad size of 5 can be considered as a good
compromise between required memory and execution time.

When set to 0, all the tiles are processed at a single stage.

### AmrelStep

When set to 'all' (the default), both seed selection and road extraction
steps are run sequentially. It is possible to run each step separately,
provided that their required input is available.

When set to 'sawing', only the seed selection step is processed.

When set to 'asd', only the road extraction step is processed.

It is also possible to process separately the sub-steps of the seed
selection steps to check the produced outputs:

* production of a slope-shaded map ('shade'),
* Sobel filtering to get gradient maps ('sobel'),
* straight edge extraction using FBSD tool ('fbsd'),
* disposition of seeds along the longest edges ('seeds').

### OutputImage

When set to 'yes', this option produces a PNG image of the last processed step.
The image is stored into the 'resources/steps' directory.

### ColorImage

By default, gray-level PNG images are output. When set to 'yes', this option
assigns a distinct color to each element of the output set when possible
(for instance to each extracted road section).

### DtmBack

When set to 'yes', this option displays the hill-shaded DTM in the output
PNG image background.

### BlackRoads

In the output PNG image, white roads are drawn over a black background.
When set to 'yes', this option draws black roads on a white background.

### DtmDir

This option can be used to specify the directory containing input
ASCII-formatted DTM files. An absolute path should be provided.

### PointDir

This option can be used to specify the directory containing input
XYZ-formatted point files. An absolute path should be provided.

## OTHER CONTROL MODE

A Unix-style command line mode is also provided. Arguments are described
in [AMREL GitHub repository](https::github.com/evenp/AMREL) README file.
