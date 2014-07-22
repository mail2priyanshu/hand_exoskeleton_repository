# Makefile for the ReNeu Hand Exoskeleton Project
# Author: Priyanshu Agarwal
# The University of Texas at Austin
# Created: Feb 12th, 2014

prefix := $(shell rtai-config --prefix)

ifeq ($(prefix),)
$(error Please add <rtai-install>/bin to your PATH variable)
endif

# Library Directories
LIBDIRS  = -L/usr/realtime/lib 
LIBDIRS += -L/usr/lib
LIBDIRS += -L/usr/local/lib 
LIBDIRS += -L/usr/local/
#LIBDIRS += -L./lib/
#LIBDIRS += -L/usr/X11R6/lib #for GLUT

# Static Libraries
#STATICLIBS = /home/reneu-t1600/phasespace/libowlsock.so
#STATICLIBS+= /usr/local/lib/libnlopt.a
#STATICLIBS +=-Llib/libdxl.a
STATICLIBS += libEposCmd.so

# Archive Libraries
#ARCHLIBS = ../lib/libdxl.a

# Include Directories
#INCLUDEDIRS = -I/home/reneu-t1600/phasespace/include
#INCLUDEDIRS += -I/home/reneu-t1600/comedi_programs/include/eigen-eigen-5097c01bcdc4/Eigen
#INCLUDEDIRS += -I/home/reneu-t1600/phasespace/include/libowlsock
INCLUDEDIRS += -I /usr/include/
INCLUDEDIRS += -I ../include
INCLUDEDIRS += -I /usr/local/
#INCLUDEDIRS += -I/usr/local/include/
#INCLUDEDIRS += -I/usr/realtime/include/ # rtai headers - check if you need the 'realtime' part
#INCLUDEDIRS += -I/usr/src/comedilib/c++/include/
#INCLUDEDIRS = -I/usr/src/rtai/addons/cpp/
#INCLUDEDIRS += -I./include/
#INCLUDEDIRS += -I./src/Externals/eigen3/
#INCLUDEDIRS += -I/usr/X11R6/include  #for GLUT
#INCLUDEDIRS += -I/home/rtai/boost/boost_1_47_0/

# Object Files
OBJFILES = NiFpga.o
OBJFILES += exo_finger.o
#OBJFILES += gnuplot_i.o

CFLAGS 	= -O3 -I./src -I/usr/local/Eigen/Dense

CC = $(shell rtai-config --cxx)
LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_CFLAGS += -D_GNU_SOURCE
LXRT_CFLAGS += -Wl,-rpath /usr/realtime/lib
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

all:: gnuplot_i.o NiFpga.o exo_finger.o IFE_controller

gnuplot_i.o: ./gnuplot/gnuplot_i.c
	gcc $(CFLAGS) -c -o gnuplot_i.o ./gnuplot/gnuplot_i.c

NiFpga.o: NiFpga.c


exo_finger.o: exo_finger.cc
	$(CC) $(LXRT_CFLAGS) -c $(INCLUDEDIRS) -o $@ $< -lm
#PID_motor_control.o: PID_motor_control.c
#	$(CC) $(LXRT_CFLAGS) $(INCLUDEDIRS) -ldl NiFpga.o -o $@ $< $(LXRT_LDFLAGS) -lkcomedilxrt -lm $(LIBDIRS) $(STATICLIBS)

#exoskeleton_controller.o: exoskeleton_controller.cc
#	$(CC) -c $(LXRT_CFLAGS) $(INCLUDEDIRS) -ldl NiFpga.o -o $@ $(LXRT_LDFLAGS) -lkcomedilxrt -lm $(LIBDIRS) $(STATICLIBS) -dxl

#exoskeleton_kinematics_solver: exoskeleton_kinematics_solver.cc NiFpga.o exo_finger.o gnuplot_i.o
#	$(CC) $(LXRT_CFLAGS) $(INCLUDEDIRS) $(OBJFILES) gnuplot_i.o -o $@ $< $(LXRT_LDFLAGS) -lm $(LIBDIRS) $(STATICLIBS) $(ARCHLIBS) -ldl

IFE_controller: IFE_controller.cc NiFpga.o exo_finger.o gnuplot_i.o
	$(CC) $(LXRT_CFLAGS) $(INCLUDEDIRS) $(OBJFILES) gnuplot_i.o -o $@ $< $(LXRT_LDFLAGS) -lm $(LIBDIRS) $(STATICLIBS) $(ARCHLIBS) -ldl

#-o $@
clean::
	rm -f *.o IFE_controller modules.order Module.symvers

.PHONY: clean
