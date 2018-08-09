ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src

bin_PROGRAMS =  rtspserver

rtspserver_SOURCES = tests/testVideoCapture.cpp
rtspserver_CPPFLAGS = -std=c++11 -g -Wall -D__STDC_CONSTANT_MACROS
rtspserver_LDFLAGS = -Lsrc -llivemediastreamer -lBasicUsageEnvironment -lUsageEnvironment -lliveMedia -lgroupsock -lavcodec -lavformat -lavutil -lswresample -lswscale -lncnn
rtspserver_DEPENDENCIES = src/liblivemediastreamer.la
