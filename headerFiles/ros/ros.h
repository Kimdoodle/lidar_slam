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
#include "ros/rate.h"                           //Rate class포함, loop frequency와 관련된 헤더파일
#include "ros/console.h"                        //콘솔 출력과 로그에 관한 헤더파일
#include "ros/assert.h"                         //assertion 관련 매크로에 관한 헤더파일

#include "ros/common.h"                         //일반적인 정의, 매크로를 정의한 헤더파일
#include "ros/types.h"                          //타입 정의, 선언과 ROS Messages의 데이터타입을 정의한 헤더파일
#include "ros/node_handle.h"                    //노드 간 통신과 관련된 헤더파일
#include "ros/publisher.h"                      //Publisher의 생성과 관리를 담당하는 헤더파일
#include "ros/single_subscriber_publisher.h"    //단일 Subscriber - Publisher일 때를 메커니즘을 관리하는 헤더파일
#include "ros/service_server.h"                 //Service server생성에 관련된 헤더파일
#include "ros/subscriber.h"                     //Subscriber 생성, 관리에 관련된 헤더파일
#include "ros/service.h"                        //ROS Service의 기본 헤더파일
#include "ros/init.h"                           //ROS 환경의 실행에 관련된 함수, 매크로를 포함한 헤더파일
#include "ros/master.h"                         //ROS Master와의 상호작용에 관련된 헤더파일
#include "ros/this_node.h"                      //현재 노드의 정보와 관련된 함수나 클래스를 정의한 헤더파일
#include "ros/param.h"                          //ROS parameter의 접근, 관리하는 함수를 정의한 헤더파일
#include "ros/topic.h"                          //Publisher-Subscriber사이 주고받는 정보인 topic에 관한 헤더파일
#include "ros/names.h"                          //ROS 이름에 관한 헤더파일. 각 엔티티는 계층적인 식별자를 가짐

#endif
