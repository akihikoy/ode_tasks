#!/bin/bash
#\file    sync.sh
#\brief   Sync files from original (private use only).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.31, 2015
rsync -azv -L ${HOME}/prg/testl/ros_sandbox/ode1/{CMakeLists.txt,Makefile,config,include,launch,mainpage.dox,manifest.xml,msg,scripts,src,srv} ode1/
