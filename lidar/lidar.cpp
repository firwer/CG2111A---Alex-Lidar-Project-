/**
 * @file lidar.cpp
 * @brief RPLidar control and data communication
 * @author Group B04-4A
 *
 * Example usage: ./lidar /dev/ttyUSB0 http://<server ip>:4000/data
 */

#include <curl/curl.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <signal.h>

#include "Include/rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x) ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0) usleep(ms * 1000);
}
#endif

using namespace rp::standalone::rplidar;

/* CG2111A */
#define DEF_MARGIN 20
#define DISP_RING_ABS_DIST 100
#define DISP_FULL_DIST 16000
#define DISP_DEFAULT_DIST 1000
#define DISP_MIN_DIST 1000
#define PI 3.14159265

/* Project specific */
#define CANVA_WIDTH 600
#define CANVA_HEIGHT 600
#define SUCCESS 0
#define ERROR 1

volatile sig_atomic_t flag = 0;

/**
 * @brief Handle SIGINT (Ctrl + C) signal to terminate the program
 * 
 * @param sig The signal sent by the system (we won't use it though - we will only send SIGINT anyway)
 */
void sig_handler(int sig) {
    flag = 1;
}

/**
 * @brief Send the data collected to the web server by HTTP requests
 *
 * @param values The values to be sent, as an array of tuples
 * @param url The URL for the POST route in the server
 */
void send_data_to_server(std::vector<std::tuple<float, float, int> > values, std::string url) {
    std::string body;
    std::stringstream stream;
    stream << "data=[";
    for (unsigned int i = 0; i < values.size(); i++) {
        stream << "[" << std::get<0>(values[i]) << "," << std::get<1>(values[i]) << ","
               << std::get<2>(values[i]) << "]";
        if (i != values.size() - 1) stream << ",";
    }
    stream << "]";
    body = stream.str();

    CURL* curl;
    CURLcode res;
    curl = curl_easy_init();
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
        res = curl_easy_perform(curl);
        if (res != CURLE_OK)
            std::cerr << "Error: curl failed: " << curl_easy_strerror(res) << std::endl;
        curl_easy_cleanup(curl);
    }
}

/**
 * @brief Capture data from lidar
 *
 * @param drv The lidar driver
 * @param url The URL for the POST route in the server
 * @return u_result The result of the capture
 */
u_result capture_data(RPlidarDriver* drv, std::string url) {
    u_result ans;

    rplidar_response_measurement_node_t nodes[360 * 2];
    size_t count = _countof(nodes);

    float angle, dist;
    _u8 quality;

    std::vector<std::tuple<float, float, int> > values;

    std::cout << "Waiting for data...\n";

    // fetch exactly one 0-360 degrees' scan
    ans = drv->grabScanData(nodes, count);

    if (IS_OK(ans)) {
        drv->ascendScanData(nodes, count);

        for (int pos = 0; pos < (int)count; ++pos) {
            quality = (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
            dist = nodes[pos].distance_q2 / 4.0f;

            float distScale = 400 / (float)DISP_DEFAULT_DIST;
            float distPixel = dist * distScale;
            float rad = (float)(angle * PI / 180.0);

            float endptX = (CANVA_WIDTH / 2) + (distPixel * sin(rad));
            float endptY = (CANVA_HEIGHT / 2) - (distPixel * cos(rad));

            int brightness = (quality << 1) + 128;
            if (brightness > 255) brightness = 255;

            // std::cout << endptX << "\t" << endptY << "\t" << brightness << "\n";
            values.push_back(std::make_tuple(endptX, endptY, brightness));
        }

        std::cout << "Scan complete\n";
        send_data_to_server(values, url);
        std::cout << "Sent data to server\n";
    }

    return ans;
}

/**
 * @brief Get the lidar health status
 *
 * @param drv The lidar driver
 * @return int Either SUCCESS (0) or ERROR (1) depending on whether lidar is healthy or not
 */
bool lidar_is_healthy(RPlidarDriver* drv) {
    rplidar_response_device_health_t healthinfo;
    u_result op_result = drv->getHealth(healthinfo);

    if (IS_OK(op_result)) {
        std::cout << "RPLidar health status: ";
        switch (healthinfo.status) {
            case RPLIDAR_STATUS_OK:
                std::cout << "OK.\n";
                return true;
            case RPLIDAR_STATUS_WARNING:
                std::cout << "Warning.\n";
                return true;
            case RPLIDAR_STATUS_ERROR:
                std::cerr << "Error: " << healthinfo.error_code << "\n";
                return false;
        }
    }

    // op_result is not okay :(
    std::cerr << "Error: cannot retrieve the lidar health code: " << op_result << "\n";
    return false;
}

int main(const int argc, const char* argv[]) {
    const char* opt_com_path = NULL;
    const char* server_posturl = NULL;
    _u32 opt_com_baudrate = 115200;

    // Handle command line arguments
    if (argc < 3) {
        std::cerr << "Error: serial port of the RPLidar and server URL missing\n";
        return ERROR;
    }
    opt_com_path = argv[1];
    server_posturl = argv[2];

    // Initialize driver
    RPlidarDriver* drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        std::cerr << "Error: insufficent memory\n";
        return ERROR;
    }

    // Connect driver
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        std::cerr << "Error: cannot bind to the specified serial port " << opt_com_path << "\n";
        return ERROR;
    }

    // Get device info
    rplidar_response_device_info_t devinfo;
    u_result op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT)
            std::cerr << "Error: operation time out.\n";
        else
            std::cerr << "Error: unexpected error, code: " << op_result << "\n";
        return ERROR;
    }

    do {
        std::cout << "Getting LIDAR health info\n";
        if (!lidar_is_healthy(drv)) {
            std::cerr << "Error: lidar is not healthy\n";
            return ERROR;
        }

        drv->startMotor();
        // Start the scan. Also scan when motor is not running
        if (IS_FAIL(drv->startScan(true))) {
            std::cerr << "Error: cannot start the scan operation.\n";
            break;
        }
        if (IS_FAIL(capture_data(drv, server_posturl))) {
            std::cerr << "Error: capture data failed.\n";
            break;
        }
        
        if (flag) {
            std::cout << "SIGINT received. Terminating...\n";
            break;
        }
    } while (1);

    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
}
