SMU Notes
=========

You need opencv, ffmpeg, and sdl2.
I haven't worked out the exact minimal install, should be something like:

```bash
brew install ffmpeg
brew install opencv --devel --with-ffmpeg
brew install sdl2
```

CVDrone
===

To run:

```
cd build/linux
make
```

To run on of the sample programs from (home dir of project):
```
cd src/
cp ../sample/sampleFileName.cpp .
./build.sh sampleFileName.cpp
./sampleFileName.a
```

The samples directory is filled with awesome uses for the library. Make a backup of _src/main.cpp_ and then copy them into _src/_ as _main.cpp_

```
mv src/main.cpp src/main.cpp.old
cp samples/[file] src/main.cpp
cd build/linux
make clean
make
```

CVDrone Readme.txt
===

 CV Drone (= OpenCV + AR.Drone)
 Copyright (C) 2014 puku0x
 https://github.com/puku0x/cvdrone


INTRODUCTION
  CV Drone is free software; you can redistribute it and/or
  modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file cvdrone-license-LGPL.txt.
   (2) The BSD-style license that is included with this library in
       the file cvdrone-license-BSD.txt.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
  cvdrone-license-LGPL.txt and cvdrone-license-BSD.txt for more details.

HOW TO INSTALL
  Please unzip "cvdrone-master.zip" into an arbitrary directory.

HOW TO UNINSTALL
  Please delete the cvdrone folder.

BEFORE YOU BUILD
  You should install Visual Studio before you build CV Drone.
  CV Drone supports VC++2008/2010/2012/2013.
  To download VS, please see http://www.microsoft.com/visualstudio/eng/downloads .

HOW TO USE
  1. Open \build\vs20xx\test.sln
  2. Press F7 to build.
  3. Press F5 (or Ctrl+F5) to run.
  4. You can play around with OpenCV. Sample codes are in "src\samples".

FOR AR.DRONE 1.0 USERS
  Please update your AR.Drone's firmware to 1.11.5.

FOR AR.DRONE 2.0 USERS
  Please update your AR.Drone's firmware to 2.4.8.

FOR VS2010 USERS
  You can not build CV Drone by VS2010 after you installed VS2012.
  To build VS2010,
    1) You should install "Visual Studio 2010 SP1".  [Recommended]
    or,
    2) You should uninstall ".Net Framework 4.5" and re-install "4.0".

LIBRARY DEPENDENCIES
  CV Drone uses following libraries.
  - OpenCV 3.0.0 Alpha <BSD license>
    http://opencv.org/
  - FFmpeg 2.2.3 <LGPL v2.1 license>
    http://www.ffmpeg.org/
  - stdint.h/inttypes.h for Microsoft Visual Studio r26
    https://code.google.com/p/msinttypes/
  - POSIX Threads for Win32 2.9.1 <LGPL v2.1 license>
    http://www.sourceware.org/pthreads-win32/

  Marker-based AR sample uses following libraries adding to the above.
  - GLUT for Win32 3.7.6
    http://user.xmission.com/~nate/glut.html
  - MarkerDetector
    https://github.com/MasteringOpenCV/code/tree/master/Chapter2_iPhoneAR/Example_MarkerBasedAR/Example_MarkerBasedAR

  License files for each library can be found in the 'licenses' folder.

Thank you.
