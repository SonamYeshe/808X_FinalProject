# 808X_FinalProject
Final project of software development course. Turtlebot can explore an unknown world autonomously.
Everytime 

[![Build Status](https://travis-ci.org/SonamYeshe/808X_FinalProject.svg?branch=master)](https://travis-ci.org/SonamYeshe/808X_FinalProject)
[![Coverage Status](https://coveralls.io/repos/github/SonamYeshe/808X_FinalProject/badge.svg?branch=master)](https://coveralls.io/github/SonamYeshe/808X_FinalProject?branch=master)
---

## License
GNU General Public License v3.0

Permissions of this strong copyleft license are conditioned on making available complete source code of licensed works and modifications, which include larger works using a licensed work, under the same license. Copyright and license notices must be preserved. Contributors provide an express grant of patent rights.

## Disclaimer 
Jiawei Ge(SonamYeshe), hereby disclaims all copyright interest in the program `walker' (which makes passes at compilers) written by Jiawei Ge(SonamYeshe).

 (signature of Jiawei Ge), 14 November 2017

 Jiawei Ge

## Dependencies
1. ROS kinetic
2. Gazebo 7.0.0
3. RViz version 1.12.13 (kinetic)
4. Travis CI
5. Coveralls.io
6. ROS packages: stage_ros, rviz, slam_gmapping, move_base.

## Backlog and Sprint google spreadsheet
Backlog: https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1Yvd1-wER0aqfgY3TEJp908nOtQcfq_xC7EuNYKqGUYw/edit?usp=sharing

Sprint: https://docs.google.com/a/terpmail.umd.edu/document/d/1eefyruMA5AlBzoHlFuDsTt1-XJztjlX6wcgxyxrt5PQ/edit?usp=sharing

## UML design
UML Class Diagram for frontier:
https://drive.google.com/a/terpmail.umd.edu/file/d/1SPhiz5TIXWj1EcMbA_ZcNvGMOR5UEW9L/view?usp=sharing

UML Class Diagram for navigation:
https://drive.google.com/a/terpmail.umd.edu/file/d/18OaFoiMPZx2ztfEumjQeb-R7rY_oi3jj/view?usp=sharing

UML Activity Diagram for frontier:
https://drive.google.com/a/terpmail.umd.edu/file/d/1x55BHgdjT7aoJMb38gkIQyQikabvKMTB/view?usp=sharing

UML Activity Diagram for navigation:
https://drive.google.com/a/terpmail.umd.edu/file/d/161zBalcv-rCE0iNJbwVsMLsyn225hA_Y/view?usp=sharing

## Demo video
Video: https://www.youtube.com/upload
PPT: https://docs.google.com/a/terpmail.umd.edu/presentation/d/1kRmysa4oBWb_NNsi8qdF4qPpMcRfTe3Escjh5DqZTek/edit?usp=sharing

## Build

```
$ cd <path to catkin_ws/src>
$ git clone https://github.com/SonamYeshe/808X_FinalProject.git
change repository name to finalproject
$ cd ..
$ catkin_make
```

## Run the demo
```
$ roslaunch finalproject nav_stack.launch
```

## Doxygen
```
$ sudo apt install doxygen
$ doxygen finalproject.conf
```
The useful output is a index.html file under ../doc/html.

## Test
```
$ cd <path to catkin_ws>
$ catkin_make test
```

## Cpplint
```
$ cd <path to catkin_ws/src>
$ wget https://raw.githubusercontent.com/google/styleguide/gh-pages/cpplint/cpplint.py
$ chmod +x cpplint.py
$ roscd finalproject/
$ ../cpplint.py $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
```

## Cppcheck
```
$ roscd finalproject/
$ cppcheck --enable=all --std=c++11 --includes-file=cppcheck.txt --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```
