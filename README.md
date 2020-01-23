# CSci 5552 Spring 2020 Course Code Repository
This repository will be updated with starter code for each assignment.
After each assignment is due, it will be updated to include instructor solutions.
## Compilation Guide
### Linux
First install all required dependencies.  This repository requires Eigen3, CMake, and Python 3 (including the Matplotlib package).  On Ubuntu install with:
```
sudo apt-get install cmake libeigen3-dev python3-matplotlib
```

Next clone this repository into a local directory, and ensure you have the latest version:
```
git clone https://github.umn.edu/davi1510/CSci5552Spring2020.git
cd CSci5552Spring2020
git pull
```

Finally, create a build directory, run cmake, and build:
```
mkdir build
cd build
cmake ..
make
```

Executables and libraries will appear in directories matching their location in the source tree.  In example, the homework 0 test executable will appear at `build/src/homework/hw0/hw0_test`.

### Windows
First download and install all required dependencies:
* Eigen 3: <http://eigen.tuxfamily.org>
  * Eigen 3 is a header only library, and does not need to be installed, just downloaded.  Extract the headers somewhere convenient.
* Python 3: <https://www.python.org/downloads/>
  * Make sure you check the "Add Python to Path" box in the installer
  * Post-install, open an administrative command prompt and run `pip3 install matplotlib`
* Git: <https://git-scm.com/downloads>
  * I'd recommend choosing a different editor during the install process unless you're already familiar with vim.
* CMake: <https://cmake.org/download/>
  * Generally choose the `Windows win64-x64 Installer`
* Visual Studio: <https://visualstudio.microsoft.com/>
  * Select Visual Studio Community 2019 from the drop-down list.
  
Next open the command prompt and navigate to where you want the code to download. Then run:
```
git clone https://github.umn.edu/davi1510/CSci5552Spring2020.git
cd CSci5552Spring2020
mkdir build
```

Finally, open CMake (cmake-gui).
* Click "Browse Source..." and select the CSci5552Spring2020 directory.
* Click "Browse Build..." and select the build directory inside the CSci5552Spring2020 directory.
* Click "Configure" and select the version of Visual Studio you have installed.  To finish the configuration:
  * In the main panel click in the empty value column next to "EIGEN3\_INCLUDE\_DIR".
  * Click on the "..." that appears to the right.
  * Browse to where you downloaded and extracted the Eigen headers and select that directory.
* Click "Generate", then "Open Project"

### Mac (OS X)
First download and install all initial dependencies:
* Xcode: Install from App Store
* Homebrew: <https://brew.sh>
  * `/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`

Next install more dependencies using brew:
* CMake: Run `brew install cmake`
* Eigen3: Run `brew install eigen`
* Python 3: Run `brew install python`
  * After run `pip3 install matplotlib` to get the Python plotting code.
  
Next clone this repository into a local directory, and ensure you have the latest version:
```
git clone https://github.umn.edu/davi1510/CSci5552Spring2020.git
cd CSci5552Spring2020
git pull
```

Finally, create a build directory, run cmake, and build:
```
mkdir build
cd build
cmake ..
make
```

Executables and libraries will appear in directories matching their location in the source tree.  In example, the homework 0 test executable will appear at `build/src/homework/hw0/hw0_test`.
