/*
 *  V4LCapture.cpp - Capture based on V4L2
 * 
 *  Copyright (C) 2016  Fundació i2CAT, Internet i Innovació digital a Catalunya
 *
 *  This file is part of media-streamer.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Authors: David Cassany <david.cassany@i2cat.net>  
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "V4LCapture.hh"
#include "../../AVFramedQueue.hh"
#include<opencv2/opencv.hpp>



#include<stdio.h>
#include<unistd.h>
#include<sys/socket.h>
#include<string.h>
#include<errno.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<sys/types.h>
#define SERVER_PORT 9999
#include <time.h>
#include <sys/time.h>


V4LCapture::V4LCapture() : HeadFilter(1, REGULAR, true), forceFormat(true), 
    frameCount(0), durationCount(std::chrono::microseconds(0))
{
	printf("---func(%s)---line(%d)---\n", __FUNCTION__, __LINE__);
    oStreamInfo = new StreamInfo (VIDEO);
    oStreamInfo->video.codec = RAW;
    oStreamInfo->video.pixelFormat = YUV420P;
    
    currentTime = std::chrono::high_resolution_clock::now();
    lastTime = currentTime;
    
    //cap = new cv::VideoCapture("rtsp://admin:a12345678@192.168.1.12/h264/ch1/sub/av_stream");
    //cap->set(cv::CAP_PROP_BUFFERSIZE, 3);

    //printf("CAP_PROP_BUFFERSIZE=%f---\n", cap->get(cv::CAP_PROP_BUFFERSIZE));
    initializeEventMap();


#if 0
    struct sockaddr_in server_sock;
    int sockfd = socket(AF_INET,SOCK_STREAM,0);
    bzero(&server_sock,sizeof(server_sock));
    server_sock.sin_family=AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &server_sock.sin_addr);
    server_sock.sin_port=htons(SERVER_PORT);

    int ret=::connect(sockfd,(struct sockaddr *)&server_sock,sizeof(server_sock));
    if(ret<0)
    {
        printf("connect()\n");
    }
    printf("connect success %d\n", sockfd);
#endif

}

V4LCapture::~V4LCapture()
{
    printf("---func(%s)---line(%d)---\n", __FUNCTION__, __LINE__);
}

bool V4LCapture::configure(std::string device, unsigned width, unsigned height, unsigned fps, std::string format, bool fFormat)
{
	printf("---func(%s)---line(%d)---\n", __FUNCTION__, __LINE__);
    forceFormat = fFormat;
    frameDuration = std::chrono::microseconds((int)(std::micro::den/fps));
        struct sockaddr_in server_sock;
    sockfd = socket(AF_INET,SOCK_STREAM,0);
    bzero(&server_sock,sizeof(server_sock));
    server_sock.sin_family=AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &server_sock.sin_addr);
    server_sock.sin_port=htons(SERVER_PORT);
    //setsockopt();

    int ret=::connect(sockfd,(struct sockaddr *)&server_sock,sizeof(server_sock));
    if(ret<0)
    {
        printf("connect()\n");
    }
    printf("connect success %d\n", sockfd);

    return true;
}

bool V4LCapture::specificWriterConfig(int /*writerID*/) 
{
    printf("---func(%s)---line(%d)---\n", __FUNCTION__, __LINE__);
    return true;
};

bool V4LCapture::doProcessFrame(std::map<int, Frame*> &dstFrames, int& ret)
{   
    VideoFrame* frame = dynamic_cast<VideoFrame*> (dstFrames.begin()->second);
    std::chrono::microseconds diff = std::chrono::duration_cast<std::chrono::microseconds>
        (std::chrono::high_resolution_clock::now() - wallclock);
        
    if (std::abs(diff.count()) > frameDuration.count()/TOLERANCE_FACTOR){
//        utils::warningMsg("Resetting wallclock");
        wallclock = std::chrono::high_resolution_clock::now();
    }
    
    wallclock += frameDuration;
    
    if (!readFrame(frame)){
        frame->setConsumed(false);
    } else {
        frame->setConsumed(true);
    }
    
    
    currentTime = std::chrono::high_resolution_clock::now();
    
    frame->setPresentationTime(std::chrono::duration_cast<std::chrono::microseconds>(
        currentTime.time_since_epoch()));
    frame->setDecodeTime(frame->getPresentationTime());
    
    ret = (std::chrono::duration_cast<std::chrono::microseconds> (
            wallclock - currentTime)).count();
            
    if (!frame->getConsumed()){
        return false;
    }

    //NOTE: this is due to buggy driver paranoia, we set the frameDuration to the experimented frameDuration
    int avgFrameTime =
        getAvgFrameDuration(std::chrono::duration_cast<std::chrono::microseconds>(currentTime - lastTime));
    if (std::abs(frameDuration.count() - avgFrameTime) > 
        frameDuration.count()/(TOLERANCE_FACTOR*2)) {
        
        utils::warningMsg("Current fps set to " + std::to_string((float) std::micro::den/avgFrameTime));
        frameDuration = std::chrono::microseconds(avgFrameTime);
    }
    lastTime = currentTime;
    
    return true;
}

FrameQueue* V4LCapture::allocQueue(ConnectionData cData)
{
	printf("---func(%s)---line(%d)---\n", __FUNCTION__, __LINE__);
	return VideoFrameQueue::createNew(cData, oStreamInfo, DEFAULT_RAW_VIDEO_FRAMES);
}


bool V4LCapture::readFrame(VideoFrame *dstFrame)
{
#if 0
    if (cap == NULL)
        printf( "Video capture is NUKK\n");
    cv::Mat frame;
    cap->read(frame);

    if(frame.type() == CV_8UC3) {
        //alignFce2(frame);
        cv::Mat yuvImg;
        cv::cvtColor(frame, yuvImg, CV_BGR2YUV_I420);
	//cv::resize(yuvImg, yuvImg, cv::Size(),0.5,0.5);

        memcpy(dstFrame->getDataBuf(), yuvImg.data, 
               yuvImg.rows * yuvImg.cols);

        dstFrame->setSize(640, 480);
        dstFrame->setPixelFormat(YUV420P);
    }
    else
	    printf("NOooooooooo CV_8UC3, %d\n", cap->isOpened ());
#endif

    int needRcv = 921600; //640*480*3/2;
    unsigned char *buf = dstFrame->getDataBuf();
    char prebuf[4]= {0x11,0x22,0x33,0x44};
    int ret = write(sockfd, prebuf, 4);
    if (ret != 4) {
	    printf("request failed");
	    return false;
    }

    while(needRcv > 0){
        //printf("start capture---(%d)%d\n", needRcv, sockfd);
        ret = read(sockfd, buf, needRcv);
        buf += ret;
        needRcv -= ret;
    }

    dstFrame->setSize(640 * 2, 480);
    dstFrame->setPixelFormat(YUV420P);
    return true;
}

int V4LCapture::getAvgFrameDuration(std::chrono::microseconds duration)
{
    if (frameCount <= FRAME_AVG_COUNT){
        if (frameCount > 0){
            durationCount += duration;
        }
        frameCount++;
        return frameDuration.count();
    }
    
    durationCount -= durationCount/FRAME_AVG_COUNT;
    durationCount += duration;
    return (durationCount/FRAME_AVG_COUNT).count();
}

void V4LCapture::doGetState(Jzon::Object &filterNode)
{
    filterNode.Add("status", "capture");
    
    filterNode.Add("device", device);
    filterNode.Add("width", (int) 1920);
    filterNode.Add("height", (int) 1080);
    filterNode.Add("fps", (int) (std::micro::den/frameDuration.count()));
    filterNode.Add("format", utils::getPixTypeAsString(YUV420P));
}

bool V4LCapture::configureEvent(Jzon::Node* params)
{
    utils::infoMsg("V4LCapture configure event");
    if (!params->Has("device") || !params->Has("width") ||
        !params->Has("height") || !params->Has("fps")) {
        return false;
    }
    
    if (!params->Get("width").IsNumber() || 
        !params->Get("height").IsNumber() ||
        !params->Get("fps").IsNumber() ||
        (params->Has("forceformat") && !params->Get("forceformat").IsBool())) {
        return false;
    }
    
    return configure(params->Get("device").ToString(), 
              params->Get("width").ToInt(), 
              params->Get("height").ToInt(),
              params->Get("fps").ToInt(), 
              params->Has("format") ? params->Get("format").ToString() : "YUYV",
              params->Has("forceformat") ? params->Get("forceformat").ToBool() : true);    
}

void V4LCapture::initializeEventMap()
{
	printf("---func(%s)---line(%d)---\n", __FUNCTION__, __LINE__);
    eventMap["configure"] = std::bind(&V4LCapture::configureEvent, this, std::placeholders::_1);
}
