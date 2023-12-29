/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_ROS_H
#define ROSCPP_ROS_H

#include "ros/time.h"                           //Time class포함, 시간과 관련된 헤더파일
#include "ros/rate.h"                           //Rate class포함, loop frequence와 관련된 헤더파일
#include "ros/console.h"                        //콘솔 출력과 로그에 관한 헤더파일
#include "ros/assert.h"                         //assertion 관련 매크로에 관한 헤더파일

#include "ros/common.h"                         //일반적인 정의, 매크로를 정의한 헤더파일
#include "ros/types.h"                          //타입 정의, 선언과 ROS Messages의 데이터타입을 정의한 헤더파일
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/single_subscriber_publisher.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "ros/service.h"
#include "ros/init.h"
#include "ros/master.h"
#include "ros/this_node.h"
#include "ros/param.h"
#include "ros/topic.h"
#include "ros/names.h"

#endif
