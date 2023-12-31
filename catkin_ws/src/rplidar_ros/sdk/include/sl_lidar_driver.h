/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 */
 /*
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

/*
 *  LiDAR 센서 드라이버 헤더파일.
 *      - namespace sl을 사용한다.
 *
 *  1. LidarScanMode 구조체
 *      - sl_u16타입의 모드 ID
 *      - sl_u18타입의 answer타입 ID
 *      - float타입의 측정 간격(us_per_sample), 최대거리(max_distance)
 *      - char[]의 scan_mode 스캔 모드 이름
 *
 *  2. Result 구조체: 정상작동여부나 오류코드 확인용
 *      - sl_result(unsigned int) 타입의 error코드
 *      - 코드에 대한 value값
 *      - 정상작동이면 err값을 SL_RESULT_OK로 설정(생성자)
 *      - 오류라면 오류 코드를 err값으로 설정(생성자)
 *      - Conversion operator를 통해 bool값이나 sl_result로 변환이 가능함
 *      - operator를 통해 value값이나 내부 원소에 접근이 가능함
 *
 *  3. ENUM
 *      - LIDARTechnologyType: LiDAR센서의 작동방식을 지정한 enum.
 *      - LIDARMajorType: LiDAR센서의 모델명을 지정한 enum.
 *      - LIDARInterfaceType: LiDAR센서의 연결방식을 지정한 enum.
 *      - MotorCtrlSupport: 모터를 제어하는 방식을 지정한 enum.
 *      - ChannelType: 연결된 채널의 방식을 지정한 enum.
 *
 *  4. SlamtecLidarTimingDesc 구조체
 *      - Timing과 관련된 정보를 포함
 *      - sample_duration_uS: 샘플 지속시간, 마이크로초 단위
 *      - native_baudrate: 기본 보드레이트
 *      - linkage_delay_uS: 연결지연시간, 마이크로초 단위
 *      - native_interface_type: 연결방식, enum
 *      - native_timestamp_support: timestamp포함 여부, bool
 *
 *  5. IChannel 클래스: 연결 채널(communication channel)의 추상 인터페이스
 *      - open(): 연결 수립. 성공 시 true반환
 *      - close(): 연결 종료
 *      - flush(): 모든 데이터를 remote endpoint로 전송
 *      - waitForData(size, timeoutInMs, actualReady)
 *        : timeoutInMs시간 내 size크기의 데이터를 기다리는 함수.
 *          수신 시 actualReady에 저장함
 *      - waitForDataExt(size_hint, timeoutInMs)
 *        : size_hint크기의 데이터가 수신 가능해질때까지 timeoutInMs동안 대기.
 *      - write(data, size): size크기의 data를 endpoint로 전송, 성공한 바이트 수 반환.
 *      - read(buffer, size): size크기의 data를 buffer에 저장, 성공한 바이트 수 반환.
 *      - clearReadCache(): read Cache를 초기화하는 함수.
 *      - getChannelType(): 채널의 타입을 반환하는 함수
 *
 *     ISerialPortChannel 클래스: 시리얼 포트(serial port)통신 특화 클래스.
 *      - IChannel클래스를 상속함.
 *      - setDTR(dtr): 시리얼 포트 통신에서 사용되는 제어 신호 DTR를 설정/해제하는데 사용함
 *
 *     createSerialPortChannel(device, baudrate)
 *      - 기기의 포트 정보(device)와 baudrate로 연결을 수립하는 함수
 *     createTcpChannel(ip, port)
 *      - ip주소와 port번호로 TCP 연결을 수립하는 함수
 *     createUdpChannel(ip, port)
 *      - ip주소와 port번호로 UDP 연결을 수립하는 함수
 *
 *  6. LidarMotorInfo 구조체
 *      - motorCtrlSupport: 지원되는 모터 제어방식, enum
 *      - desired_speed: 모터의 희망 속도를 설정, sl_u16타입
 *      - max_speed: 모터의 최대 속도를 설정, sl_u16타입
 *      - min_speed: 모터의 최소 속도를 설정, sl_u16타입
 *
 *  7. ILidarDriver 클래스
 *      - LiDAR센서와 상호작용하기위한 클래스
 *      - connect(channel): 수립된 채널을 매개변수로 센서와 연결
 *      - disconnect(): 연결 해제
 *      - isConnected(): 연결 여부 확인, bool값 반환
 *
 *      - reset(timeoutInMs): 시스템 초기화
 *      - getAllSupportedScanModes(outModes, timeoutInMs): 지원하는 스캔 모드 반환
 *      - getTypicalScanMode(outMode, timeoutInMs): 일반적인 스캔 모드 반환
 *
 *      - startScan(force, useTypicalScan, options, outUsedScanMode): 스캔 시작
 *      - startScanExpress(force, scanMode, options, outUsedScanMode, timeout)
 *        : 특정 모드로 스캔 시작
 *      - getHealth(health, timeout): LiDAR센서의 self-protection mode여부 확인
 *      - getDeviceInfo(info, timeout): LiDAR센서의 장치 정보 반환
 *      - checkMotorCtrlSupport(motorCtrlSupport, timeout): 특정 모터제어 방식 지원여부 확인
 *      - getFrequency(scanMode, nodes, count, frequency): 주어진 data로 frequency계산
 *      - setLidarIpConf(conf, timeout): LPX, S2E시리즈 센서의 고정 IP주소 설정
 *      - getLidarIpConf(conf, timeout): LPX, S2E시리즈 센서의 고정 IP주소 반환
 *      - getDeviceMacAddr(macAddrArray, timeoutInMs): 센서의 MAC주소 반환
 *      - stop(timeout): 대기상태로 전환
 *      - grabScanDataHq(nodebuffer, count, timeout): 이전에 수신한 완전한 스캔 데이터 대기
 *      - gradScanDataHqWithTimeStamp(nodebuffer, count, timestamp_uS, timeout)
 *        : timeStamp를 포함하여 이전에 수신한 완전한 스캔 데이터 대기
 *      - ascendScanData(nodebuffer, count): 스캔 데이터를 각도 값에 따라 정렬
 *      - getScanDataWithIntervalHq(nodebuffer, count): 완전한 데이터 여부에 상관없이 스캔 포인트 반환
 *      - setMotorSpeed(speed): 모터 속도 설정
 *      - getMotorInfo(motorInfo, timeoutInMs): 속도 정보를 포함한 모터 정보 반환
 *      - negotiateSerialBaudRate(requiredBaudRate, baudRateDetected): 새로운 baudrate사용 설정
 *      - getLIDARTechnologyType(devInfo): 센서의 측정방법 정보 반환
 *      - getLIDARMajorType(devInfo): 센서 모델의 시리즈 정보 반환
 *      - getModeNameDescriptionString(out_description, fetchAliasName, devInfo, timeout)
 *        : 센서의 모델명 정보 반환
 *
 *
 *      드라이버 설정 예시 #######
 *      Result<ISerialChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
 *      assert((bool)channel);
 *      assert(*channel);
 *
 *      auto lidar = createLidarDriver();
 *      assert((bool)lidar);
 *      assert(*lidar);
 *
 *      auto res = (*lidar)->connect(*channel);
 *      assert(SL_IS_OK(res));
 *
 *      sl_lidar_response_device_info_t deviceInfo;
 *      res = (*lidar)->getDeviceInfo(deviceInfo);
 *      assert(SL_IS_OK(res));
 *
 *      printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
 *              deviceInfo.model,
 *              deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
 *              deviceInfo.hardware_version);
 *
 *      delete *lidar;
 *      delete *channel;
 */

#pragma once

#ifndef __cplusplus
#error "The Slamtec LIDAR SDK requires a C++ compiler to be built"
#endif

#include <vector>
#include <map>
#include <string>

#ifndef DEPRECATED
    #ifdef __GNUC__
        #define DEPRECATED(func) func __attribute__ ((deprecated))
    #elif defined(_MSC_VER)
        #define DEPRECATED(func) __declspec(deprecated) func
    #else
        #pragma message("WARNING: You need to implement DEPRECATED for this compiler")
        #define DEPRECATED(func) func
    #endif
#endif


#include "sl_lidar_cmd.h"

#include <string>

    namespace sl {

#ifdef DEPRECATED
#define DEPRECATED_WARN(fn, replacement) do { \
        static bool __shown__ = false; \
        if (!__shown__) { \
            printDeprecationWarn(fn, replacement); \
            __shown__ = true; \
        } \
    } while (0)
#endif

    /**
    * Lidar scan mode
    */
    struct LidarScanMode
    {
        // Mode id
        sl_u16  id;

        // Time cost for one measurement (in microseconds)
        float   us_per_sample;

        // Max distance in this scan mode (in meters)
        float   max_distance;

        // The answer command code for this scan mode
        sl_u8   ans_type;

        // The name of scan mode (padding with 0 if less than 64 characters)
        char    scan_mode[64];
    };

    template <typename T>
    struct Result
    {
        sl_result err;
        T value;
        Result(const T& value)
            : err(SL_RESULT_OK)
            , value(value)
        {
        }

        Result(sl_result err)
            : err(err)
            , value()
        {
        }

        operator sl_result() const
        {
            return err;
        }

        operator bool() const
        {
            return SL_IS_OK(err);
        }

        T& operator* ()
        {
            return value;
        }

        T* operator-> ()
        {
            return &value;
        }
    };

    enum LIDARTechnologyType {
        LIDAR_TECHNOLOGY_UNKNOWN = 0,
        LIDAR_TECHNOLOGY_TRIANGULATION = 1,
        LIDAR_TECHNOLOGY_DTOF = 2,
        LIDAR_TECHNOLOGY_ETOF = 3,
        LIDAR_TECHNOLOGY_FMCW = 4,
    };

    enum LIDARMajorType {
        LIDAR_MAJOR_TYPE_UNKNOWN = 0,
        LIDAR_MAJOR_TYPE_A_SERIES = 1,
        LIDAR_MAJOR_TYPE_S_SERIES = 2,
        LIDAR_MAJOR_TYPE_T_SERIES = 3,
        LIDAR_MAJOR_TYPE_M_SERIES = 4,
        LIDAR_MAJOR_TYPE_C_SERIES = 6,
    };

    enum LIDARInterfaceType {
        LIDAR_INTERFACE_UART = 0,
        LIDAR_INTERFACE_ETHERNET = 1,
        LIDAR_INTERFACE_USB = 2,
        LIDAR_INTERFACE_CANBUS = 5,


        LIDAR_INTERFACE_UNKNOWN = 0xFFFF,
    };

    struct SlamtecLidarTimingDesc {

        sl_u32  sample_duration_uS;
        sl_u32  native_baudrate;
        
        sl_u32  linkage_delay_uS;

        LIDARInterfaceType native_interface_type;

        bool    native_timestamp_support;
    };

    /**
    * Abstract interface of communication channel
    */
    class IChannel
    {
    public:
        virtual ~IChannel() {}

    public:
        /**
        * Open communication channel (return true if succeed)
        */
        virtual bool open() = 0;

        /**
        * Close communication channel
        */
        virtual void close() = 0;

        /**
        * Flush all written data to remote endpoint
        */
        virtual void flush() = 0;

        /**
        * Wait for some data
        * \param size Bytes to wait
        * \param timeoutInMs Wait timeout (in microseconds, -1 for forever)
        * \param actualReady [out] actual ready bytes
        * \return true for data ready
        */
        virtual bool waitForData(size_t size, sl_u32 timeoutInMs = -1, size_t* actualReady = nullptr) = 0;


        /**
        * Wait for some data
        * \param size_hint Byte count may available to retrieve without beening blocked 
        * \param timeoutInMs Wait timeout (in microseconds, -1 for forever)
        * \return RESULT_OK if there is data available for receiving
        *         RESULT_OPERATION_TIMEOUT if the given timeout duration is exceed
        *         RESULT_OPERATION_FAIL if there is something wrong with the channel
        */
        virtual sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs = 1000) = 0;


        /**
        * Send data to remote endpoint
        * \param data The data buffer
        * \param size The size of data buffer (in bytes)
        * \return Bytes written (negative for write failure)
        */
        virtual int write(const void* data, size_t size) = 0;

        /**
        * Read data from the chanel
        * \param buffer The buffer to receive data
        * \param size The size of the read buffer
        * \return Bytes read (negative for read failure)
        */
        virtual int read(void* buffer, size_t size) = 0;

        /**
        * Clear read cache
        */
        virtual void clearReadCache() = 0;

        virtual int getChannelType() = 0;

    private:

    };

    /**
    * Abstract interface of serial port channel
    */
    class ISerialPortChannel : public IChannel
    {
    public:
        virtual ~ISerialPortChannel() {}

    public:
        virtual void setDTR(bool dtr) = 0;
    };

    /**
    * Create a serial channel
    * \param device Serial port device
    *                   e.g. on Windows, it may be com3 or \\.\com10
    *                   on Unix-Like OS, it may be /dev/ttyS1, /dev/ttyUSB2, etc
    * \param baudrate Baudrate
    *                   Please refer to the datasheet for the baudrate (maybe 115200 or 256000)
    */
    Result<IChannel*> createSerialPortChannel(const std::string& device, int baudrate);

    /**
    * Create a TCP channel
    * \param ip IP address of the device
    * \param port TCP port
    */
    Result<IChannel*> createTcpChannel(const std::string& ip, int port);

    /**
    * Create a UDP channel
    * \param ip IP address of the device
    * \param port UDP port
    */
    Result<IChannel*> createUdpChannel(const std::string& ip, int port);

    enum MotorCtrlSupport
    {
        MotorCtrlSupportNone = 0,
        MotorCtrlSupportPwm = 1,
        MotorCtrlSupportRpm = 2,
    };

    enum ChannelType{
        CHANNEL_TYPE_SERIALPORT = 0x0,
        CHANNEL_TYPE_TCP = 0x1,
        CHANNEL_TYPE_UDP = 0x2,
    };

        /**
    * Lidar motor info
    */
    struct LidarMotorInfo
    {
        MotorCtrlSupport motorCtrlSupport;

        // Desire speed
        sl_u16 desired_speed;

        // Max speed 
        sl_u16 max_speed;

        // Min speed
        sl_u16 min_speed;
    };

    class ILidarDriver
    {
    public:
        virtual ~ILidarDriver() {}

    public:
        /**
        * Connect to LIDAR via channel
        * \param channel The communication channel
        *                    Note: you should manage the lifecycle of the channel object, make sure it is alive during lidar driver's lifecycle
        */
        virtual sl_result connect(IChannel* channel) = 0;

        /**
        * Disconnect from the LIDAR
        */
        virtual void disconnect() = 0;
        
        /**
        * Check if the connection is established
        */
        virtual bool isConnected() = 0;

    public:
        enum
        {
            DEFAULT_TIMEOUT = 2000
        };

    public:
        /// Ask the LIDAR core system to reset it self
        /// The host system can use the Reset operation to help LIDAR escape the self-protection mode.
        ///
        ///  \param timeout       The operation timeout value (in millisecond)
        virtual sl_result reset(sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;

        /// Get all scan modes that supported by lidar
        virtual sl_result getAllSupportedScanModes(std::vector<LidarScanMode>& outModes, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;

        /// Get typical scan mode of lidar
        virtual sl_result getTypicalScanMode(sl_u16& outMode, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;

        /// Start scan
        ///
        /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
        /// \param useTypicalScan   Use lidar's typical scan mode or use the compatibility mode (2k sps)
        /// \param options          Scan options (please use 0)
        /// \param outUsedScanMode  The scan mode selected by lidar
        virtual sl_result startScan(bool force, bool useTypicalScan, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr) = 0;

        /// Start scan in specific mode
        ///
        /// \param force            Force the core system to output scan data regardless whether the scanning motor is rotating or not.
        /// \param scanMode         The scan mode id (use getAllSupportedScanModes to get supported modes)
        /// \param options          Scan options (please use 0)
        /// \param outUsedScanMode  The scan mode selected by lidar
        virtual sl_result startScanExpress(bool force, sl_u16 scanMode, sl_u32 options = 0, LidarScanMode* outUsedScanMode = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Retrieve the health status of the RPLIDAR
        /// The host system can use this operation to check whether RPLIDAR is in the self-protection mode.
        ///
        /// \param health        The health status info returned from the RPLIDAR
        ///
        /// \param timeout       The operation timeout value (in millisecond) for the serial port communication     
        virtual sl_result getHealth(sl_lidar_response_device_health_t& health, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Get the device information of the RPLIDAR include the serial number, firmware version, device model etc.
        /// 
        /// \param info          The device information returned from the RPLIDAR
        /// \param timeout       The operation timeout value (in millisecond) for the serial port communication  
        virtual sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Check whether the device support motor control
        /// Note: this API will disable grab.
        /// 
        /// \param motorCtrlSupport Return the result.
        /// \param timeout          The operation timeout value (in millisecond) for the serial port communication. 
        virtual sl_result checkMotorCtrlSupport(MotorCtrlSupport& motorCtrlSupport, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Calculate LIDAR's current scanning frequency from the given scan data
        /// Please refer to the application note doc for details
        /// Remark: the calcuation will be incorrect if the specified scan data doesn't contains enough data
        ///
        /// \param scanMode      Lidar's current scan mode
        /// \param nodes         Current scan's measurements
        /// \param count         The number of sample nodes inside the given buffer
        virtual sl_result getFrequency(const LidarScanMode& scanMode, const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, float& frequency) = 0;

		///Set LPX and S2E series lidar's static IP address
		///
		/// \param conf             Network parameter that LPX series lidar owned
		/// \param timeout          The operation timeout value (in millisecond) for the ethernet udp communication
		virtual sl_result setLidarIpConf(const sl_lidar_ip_conf_t& conf, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;
       
        ///Get LPX and S2E series lidar's static IP address
        ///
        /// \param conf             Network parameter that LPX series lidar owned
        /// \param timeout          The operation timeout value (in millisecond) for the ethernet udp communication
        virtual sl_result getLidarIpConf( sl_lidar_ip_conf_t& conf, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;
  // 
		/////Get LPX series lidar's MAC address
		///
		/// \param macAddrArray         The device MAC information returned from the LPX series lidar
        ///                             Notice: the macAddrArray must point to a valid buffer with at least 6 bytes length
        ///                                     Otherwise, buffer overwrite will occur
		virtual sl_result getDeviceMacAddr(sl_u8* macAddrArray, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;

        /// Ask the LIDAR core system to stop the current scan operation and enter idle state. The background thread will be terminated
        ///
        /// \param timeout       The operation timeout value (in millisecond) for the serial port communication 
        virtual sl_result stop(sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

        /// Wait and grab a complete 0-360 degree scan data previously received. 
        /// The grabbed scan data returned by this interface always has the following charactistics:
        ///
        /// 1) The first node of the grabbed data array (nodebuffer[0]) must be the first sample of a scan, i.e. the start_bit == 1
        /// 2) All data nodes are belong to exactly ONE complete 360-degrees's scan
        /// 3) Note, the angle data in one scan may not be ascending. You can use API ascendScanData to reorder the nodebuffer.
        ///
        /// \param nodebuffer     Buffer provided by the caller application to store the scan data
        ///
        /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
        ///                       Once the interface returns, this parameter will store the actual received data count.
        ///
        /// \param timeout        Max duration allowed to wait for a complete scan data, nothing will be stored to the nodebuffer if a complete 360-degrees' scan data cannot to be ready timely.
        ///
        /// The interface will return SL_RESULT_OPERATION_TIMEOUT to indicate that no complete 360-degrees' scan can be retrieved withing the given timeout duration. 
        ///
        /// \The caller application can set the timeout value to Zero(0) to make this interface always returns immediately to achieve non-block operation.
        virtual sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;



        /// Wait and grab a complete 0-360 degree scan data previously received with timestamp support.
        /// 
        /// The returned timestamp belongs to the first data point of the scan data (begining of the scan).
        /// Its value is represented based on the current machine's time domain with the unit of microseconds (uS).
        /// 
        /// If the currently connected LIDAR supports hardware timestamp mechanism, this timestamp will use 
        /// the actual data emitted by the LIDAR device and remap it to the current machine's time domain. 
        /// 
        /// For other models that do not support hardware timestamps, this data will be deducted through estimation, 
        /// and there may be a slight deviation from the actual situation. 
        ///
        /// The grabbed scan data returned by this interface always has the following charactistics:
        ///
        /// 1) The first node of the grabbed data array (nodebuffer[0]) must be the first sample of a scan, i.e. the start_bit == 1
        /// 2) All data nodes are belong to exactly ONE complete 360-degrees's scan
        /// 3) Note, the angle data in one scan may not be ascending. You can use API ascendScanData to reorder the nodebuffer.
        ///
        /// \param nodebuffer     Buffer provided by the caller application to store the scan data
        ///
        /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
        ///                       Once the interface returns, this parameter will store the actual received data count.
        ///
        /// \param timestamp_uS   The reference used to store the timestamp value.
        /// \param timeout        Max duration allowed to wait for a complete scan data, nothing will be stored to the nodebuffer if a complete 360-degrees' scan data cannot to be ready timely.
        ///
        /// The interface will return SL_RESULT_OPERATION_TIMEOUT to indicate that no complete 360-degrees' scan can be retrieved withing the given timeout duration. 
        ///
        /// \The caller application can set the timeout value to Zero(0) to make this interface always returns immediately to achieve non-block operation.
        virtual sl_result grabScanDataHqWithTimeStamp(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, sl_u64 & timestamp_uS, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;


        /// Ascending the scan data according to the angle value in the scan.
        ///
        /// \param nodebuffer     Buffer provided by the caller application to do the reorder. Should be retrived from the grabScanData
        ///
        /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
        ///                       Once the interface returns, this parameter will store the actual received data count.
        /// The interface will return SL_RESULT_OPERATION_FAIL when all the scan data is invalid. 
        virtual sl_result ascendScanData(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t count) = 0;

        /// Return received scan points even if it's not complete scan
        ///
        /// \param nodebuffer     Buffer provided by the caller application to store the scan data
        ///
        /// \param count          Once the interface returns, this parameter will store the actual received data count.
        ///
        /// The interface will return SL_RESULT_OPERATION_TIMEOUT to indicate that not even a single node can be retrieved since last call. 
        virtual sl_result getScanDataWithIntervalHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count) = 0;
        /// Set lidar motor speed
        /// The host system can use this operation to set lidar motor speed.
        ///
        /// \param speed        The speed value set to lidar
        ///
        ///Note: The function will stop scan if speed is DEFAULT_MOTOR_SPEED.
        virtual sl_result setMotorSpeed(sl_u16 speed = DEFAULT_MOTOR_SPEED) = 0;
        
        /// Get the motor information of the RPLIDAR include the max speed, min speed, desired speed.
        /// 
        /// \param motorInfo          The motor information returned from the RPLIDAR
        virtual sl_result getMotorInfo(LidarMotorInfo &motorInfo, sl_u32 timeoutInMs = DEFAULT_TIMEOUT) = 0;
    

        /// Ask the LIDAR to use a new baudrate for serial communication
        /// The target LIDAR system must support such feature to work.
        /// This function does NOT check whether the target LIDAR works with the requiredBaudRate or not.
        /// In order to verifiy the result, use getDeviceInfo or other getXXXX functions instead.
        /// 
        /// \param requiredBaudRate   The new baudrate required to be used. It MUST matches with the baudrate of the binded channel.
        /// \param baudRateDetected   The actual baudrate detected by the LIDAR system
        virtual sl_result negotiateSerialBaudRate(sl_u32 requiredBaudRate, sl_u32* baudRateDetected = NULL) = 0;



        /// Get the technology of the LIDAR's measurement system
        /// 
        /// 
        /// \param devInfo   The device info used to deduct the result
        ///                  If NULL is specified, a driver cached version of the connected LIDAR will be used
        virtual LIDARTechnologyType getLIDARTechnologyType(const sl_lidar_response_device_info_t* devInfo = nullptr) = 0;
        
        
        /// Get the Major Type (Series Info) of the LIDAR
        /// 
        /// 
        /// \param devInfo   The device info used to deduct the result
        ///                  If NULL is specified, a driver cached version of the connected LIDAR will be used
        virtual LIDARMajorType getLIDARMajorType(const sl_lidar_response_device_info_t* devInfo = nullptr) = 0;


        /// Get the Model Name of the LIDAR
        /// The result will be somthing like: "A1M8" or "S1M1" or "A3M1-R1"
        /// 
        /// \param out_description   The output string that contains the generated model name
        ///                          
        /// \param fetchAliasName    If set to true, a communication will be taken to ask if there is any Alias name availabe
        /// \param devInfo           The device info used to deduct the result
        ///                          If NULL is specified, a driver cached version of the connected LIDAR will be used
        /// \param timeout           The timeout value used by potential data communication
        virtual sl_result getModelNameDescriptionString(std::string& out_description, bool fetchAliasName = true, const sl_lidar_response_device_info_t* devInfo = nullptr, sl_u32 timeout = DEFAULT_TIMEOUT) = 0;

};

    /**
    * Create a LIDAR driver instance
    *
    * Example
    * Result<ISerialChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);
    * assert((bool)channel);
    * assert(*channel);
    *
    * auto lidar = createLidarDriver();
    * assert((bool)lidar);
    * assert(*lidar);
    *
    * auto res = (*lidar)->connect(*channel);
    * assert(SL_IS_OK(res));
    *
    * sl_lidar_response_device_info_t deviceInfo;
    * res = (*lidar)->getDeviceInfo(deviceInfo);
    * assert(SL_IS_OK(res));
    *
    * printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
    *        deviceInfo.model,
    *        deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
    *        deviceInfo.hardware_version);
    *
    * delete *lidar;
    * delete *channel;
    */
    Result<ILidarDriver*> createLidarDriver();
}
