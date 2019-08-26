
/* main.cpp */

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include <signal.h>

#include "rplidar.h"

using namespace rp::standalone::rplidar;

/*
 * SIGINTが発生したかどうかを示すフラグ
 */
volatile sig_atomic_t gSigintHandled = 0;

/*
 * RPlidarのデバイスの情報を出力
 */
void PrintRPlidarDeviceInfo(
    const rplidar_response_device_info_t& deviceInfo)
{
    std::cout << "Serial number: ";
    std::for_each(
        std::begin(deviceInfo.serialnum),
        std::end(deviceInfo.serialnum),
        [](_u8 x) { std::cout << std::setw(2)
                              << std::setfill('0')
                              << std::hex
                              << std::uppercase
                              << static_cast<int>(x)
                              << std::dec; });
    std::cout << '\n';

    std::cout << "Firmware version: "
              << static_cast<int>(deviceInfo.firmware_version >> 8)
              << "." << std::setw(2) << std::setfill('0')
              << static_cast<int>(deviceInfo.firmware_version & 0xFF) << '\n';
    std::cout << "Hardware version: "
              << static_cast<int>(deviceInfo.hardware_version) << '\n';
}

/*
 * RPlidarのデバイスの状態を出力
 */
void PrintRPlidarHealthInfo(
    const rplidar_response_device_health_t& healthInfo)
{
    const char* healthStatus =
        (healthInfo.status == RPLIDAR_STATUS_OK) ?
            "RPLIDAR_STATUS_OK" :
        (healthInfo.status == RPLIDAR_STATUS_WARNING) ?
            "RPLIDAR_STATUS_WARNING" :
        (healthInfo.status == RPLIDAR_STATUS_ERROR) ?
            "RPLIDAR_STATUS_ERROR" : "UNKNOWN";

    std::cout << "RPlidar health status: "
              << healthStatus << '\n';
    std::cout << "RPlidar error code: "
              << healthInfo.error_code << '\n';
}

/*
 * RPlidarのスキャンデータを出力
 */
void PrintRPlidarMeasurementNodeHqInfo(
    const rplidar_response_measurement_node_hq_t& measurementNode)
{
    char syncBit =
        (measurementNode.flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? 'S' : ' ';
    float angleDeg = measurementNode.angle_z_q14 * 90.0f / (1 << 14);
    float distMeter = measurementNode.dist_mm_q2 / 1000.0f / (1 << 2);

    std::cout << "Scan data sync: " << syncBit << ", "
              << "theta (deg): " << angleDeg << ", "
              << "dist (m): " << distMeter << ", "
              << "quality: " << static_cast<int>(measurementNode.quality) << '\n';
}

/*
 * RPlidarのスキャンモードを出力
 */
void PrintRPlidarScanMode(
    const RplidarScanMode& scanMode)
{
    std::cout << "Scan mode id: " << static_cast<int>(scanMode.id) << ", "
              << "sample duration (us): " << scanMode.us_per_sample << ", "
              << "max distance (m): " << scanMode.max_distance << ", "
              << "ans type: " << static_cast<int>(scanMode.ans_type) << ", "
              << "scan mode: " << scanMode.scan_mode << '\n';
}

/*
 * SIGINTシグナルのハンドラ
 */
void SigintHandler(int sig)
{
    (void)sig;

    gSigintHandled = 1;
}

int main(int argc, char** argv)
{
    const std::string defaultPortName = "/dev/ttyUSB0";
    const int availableBaudrates[] = { 115200, 256000 };
    
    u_result opResult;
    int statusCode = EXIT_SUCCESS;
    const size_t numOfNodes = RPlidarDriver::MAX_SCAN_NODES;
    size_t actualNumOfNodes;
    std::string portName = defaultPortName;
    int comBaudrate = availableBaudrates[0];
    
    /* パラメータ(ポート名)の取得 */
    if (argc > 1)
        portName = argv[1];
    
    /* パラメータ(ボーレート)の取得 */
    if (argc > 2)
        comBaudrate = std::stoi(argv[2], nullptr, 10);

    if (std::find(std::begin(availableBaudrates),
                  std::end(availableBaudrates),
                  comBaudrate) ==
        std::end(availableBaudrates)) {
        std::cerr << "Invalid baudrate specified" << '\n'
                  << "Available baudrate: 115200, 256000" << '\n';
        return EXIT_FAILURE;
    }

    /* RPlidarドライバの作成 */
    RPlidarDriver* pDriver =
        RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    if (pDriver == nullptr) {
        std::cerr << "RPlidarDriver::CreateDriver() failed" << '\n';
        return EXIT_FAILURE;
    }
    
    /* RPlidarに接続 */
    opResult = pDriver->connect(portName.c_str(),
                                static_cast<_u32>(comBaudrate));
    
    if (IS_FAIL(opResult)) {
        std::cerr << "RPlidarDriver::connect() failed" << '\n';
        statusCode = EXIT_FAILURE;
        goto Cleanup;
    }

    /* デバイスの情報を取得 */
    rplidar_response_device_info_t deviceInfo;
    opResult = pDriver->getDeviceInfo(deviceInfo);

    if (IS_FAIL(opResult)) {
        std::cerr << "RPlidarDriver::getDeviceInfo() failed" << '\n';
        statusCode = EXIT_FAILURE;
        goto Cleanup;
    }
    
    /* デバイスの情報を表示 */
    PrintRPlidarDeviceInfo(deviceInfo);

    /* デバイスの状態を取得 */
    rplidar_response_device_health_t healthInfo;
    opResult = pDriver->getHealth(healthInfo);

    if (IS_FAIL(opResult)) {
        std::cerr << "RPlidarDriver::getHealth() failed" << '\n';
        statusCode = EXIT_FAILURE;
        goto Cleanup;

    }

    /* デバイスの状態を表示 */
    PrintRPlidarHealthInfo(healthInfo);

    if (healthInfo.status == RPLIDAR_STATUS_ERROR) {
        std::cerr << "RPlidar internal error detected" << '\n';
        statusCode = EXIT_FAILURE;
        goto Cleanup;
    }

    /* SIGINTシグナルを捕捉 */
    struct sigaction sigact;
    std::memset(&sigact, 0, sizeof(sigact));
    sigact.sa_handler = SigintHandler;
    sigemptyset(&sigact.sa_mask);
    sigaction(SIGINT, &sigact, nullptr);

    /* RPlidarのモータを動作 */
    opResult = pDriver->startMotor();

    if (IS_FAIL(opResult)) {
        std::cerr << "RPlidarDriver::startMotor() failed" << '\n';
        statusCode = EXIT_FAILURE;
        goto Cleanup;
    }

    /* RPlidarのデータ取得を開始 */
    RplidarScanMode scanMode;

    opResult = pDriver->startScan(false, true, 0, &scanMode);

    if (IS_FAIL(opResult)) {
        std::cerr << "RPlidarDriver::startScan() failed" << '\n';
        statusCode = EXIT_FAILURE;
        goto Cleanup;
    }

    /* RPlidarのスキャンモードを表示 */
    PrintRPlidarScanMode(scanMode);

    /* RPlidarのスキャンデータを取得 */
    actualNumOfNodes = numOfNodes;
    rplidar_response_measurement_node_hq_t measuredNodes[numOfNodes];

    while (true) {
        /* SIGINTシグナルが発生したら終了 */
        if (gSigintHandled == 1)
            break;

        /* RPlidarのスキャンデータを取得 */
        opResult = pDriver->grabScanDataHq(measuredNodes, actualNumOfNodes);

        if (IS_FAIL(opResult)) {
            std::cerr << "RPlidarDriver::grabScanDataHq() failed" << '\n';
            break;
        }

        /* RPlidarのスキャンデータを昇順に並べ替え */
        opResult = pDriver->ascendScanData(measuredNodes, actualNumOfNodes);

        /* RPlidarのスキャンデータを表示 */
        for (int i = 0; i < static_cast<int>(actualNumOfNodes); ++i)
            PrintRPlidarMeasurementNodeHqInfo(measuredNodes[i]);
    }

Cleanup:
    /* RPlidarのデータ取得を終了 */
    pDriver->stop();

    /* RPlidarのモータを停止 */
    pDriver->stopMotor();

    /* RPlidarとの接続を終了 */
    pDriver->disconnect();

    /* RPlidarドライバを破棄 */
    RPlidarDriver::DisposeDriver(pDriver);
    pDriver = nullptr;

    return statusCode;
}

