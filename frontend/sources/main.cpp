/* Copyright 2015 Sony Corporation */
/* Sony Confidential               */


/**
 * @file main.cpp
 * @brief Interface mavlink commands from FCU to camera.
 * @author Bharat Tak
*/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <utime.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <poll.h>
#include <limits.h>

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <inttypes.h>
#include <fstream>
#include <time.h>
#include <sys/time.h>

#include <mavconn/interface.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "socket.hpp"

using mavconn::MAVConnInterface;
using mavconn::Framing;
using mavlink::mavlink_message_t;

#define NUMBER_OF_CAMERA_PARAMETERS 4
#define OPTCMP(value, str, COMMAND) if (!strcmp(str, argv[1])) { value = COMMAND; }

struct camera_request
{
    char command[100];
    char action[100];
    uint8_t value = 0;
    uint8_t param_type;
    uint16_t param_index;
    boost::posix_time::ptime timestamp;
};

struct camera_responce
{
    char command[100];
    char action[100];
    uint8_t value = 0;
    int result = -1;
    uint8_t param_type;
    uint16_t param_index;
    boost::posix_time::ptime timestamp;
};

void get_camera_parameter(std::string param_id_str, SocketClient *main_client)
{
    camera_request *request = new camera_request{};
    strcpy(request->command, param_id_str.c_str());
    snprintf(request->action, sizeof(request->action), "GET");
    request->param_type = static_cast<uint8_t>(mavlink::common::MAV_PARAM_TYPE::UINT8);
    request->timestamp = boost::posix_time::microsec_clock::local_time();
    main_client->write(request, sizeof(*request));
    delete request;
}

uint8_t set_camera_parameter(std::string param_id_str, SocketClient *main_client, uint8_t data) {
    camera_request *request = new camera_request{};
    strcpy(request->command, param_id_str.c_str());
    snprintf(request->action, sizeof(request->action), "SET");
    request->value = data;
    request->param_type = static_cast<uint8_t>(mavlink::common::MAV_PARAM_TYPE::UINT8);
    request->timestamp = boost::posix_time::microsec_clock::local_time();
    main_client->write(request, sizeof(*request));
    delete request;
}

/* Handle responce from camera func and send output to Mavlink
 */
void handle_responce(camera_responce *responce, MAVConnInterface::Ptr fcu_link)
{
    printf("Command: %s\n\t Action: %s\n\t Result: %d\n", responce->command, responce->action, responce->result);

    if (!std::strcmp(responce->action, "GET")) {
        // Emit the value of a parameter. The inclusion of param_count and param_index
        // in the message allows the recipient to keep track of received parameters and
        // allows them to re-request missing parameters after a loss or timeout.
        mavlink::common::msg::PARAM_EXT_VALUE camera_param_msg;
        memset(&camera_param_msg, 0, sizeof(mavlink::common::msg::PARAM_EXT_VALUE));

        // Parameter id, terminated by NULL if the length is less than 16 human-readable chars
        // and WITHOUT null termination (NULL) byte if the length is exactly 16 chars -
        // applications have to provide 16+1 bytes storage if the ID is stored as string
        mavlink::set_string(camera_param_msg.param_id, std::string(responce->command));

        // Total number of parameters
        camera_param_msg.param_count = NUMBER_OF_CAMERA_PARAMETERS;
        mavlink::mavlink_param_union_t union_value;
        union_value.param_uint8 = responce->value;
        memcpy(&camera_param_msg.param_value, &union_value, sizeof(float));
        camera_param_msg.param_type = responce->param_type;
        camera_param_msg.param_index = responce->param_index; //EXPOSURE_MODE_INDEX;

        auto mi = camera_param_msg.get_message_info();
        mavlink::mavlink_message_t valueMsg;
        mavlink::MsgMap map(valueMsg);
        camera_param_msg.serialize(map);
        mavlink::mavlink_finalize_message(&valueMsg, fcu_link->get_system_id(), fcu_link->get_component_id(),
                                          mi.min_length, mi.length, mi.crc_extra);
        fcu_link->send_message(&valueMsg);

    } else if (!std::strcmp(responce->action, "SET")) {
        mavlink::common::msg::PARAM_EXT_ACK ack;

        ack.param_result = responce->result == 0 ? static_cast<uint8_t>(mavlink::common::PARAM_ACK::ACCEPTED) :
                           static_cast<uint8_t>(mavlink::common::PARAM_ACK::FAILED);
        mavlink::set_string(ack.param_id, std::string(responce->command));

        mavlink::mavlink_param_union_t union_value;
        union_value.param_uint8 = responce->value;
        memcpy(&ack.param_value, &union_value, sizeof(float));

        ack.param_type = responce->param_type;
        auto mi = ack.get_message_info();

        mavlink::mavlink_message_t ackMsg;
        mavlink::MsgMap ack_map(ackMsg);
        ack.serialize(ack_map);
        mavlink::mavlink_finalize_message(&ackMsg, fcu_link->get_system_id(),
                                          fcu_link->get_component_id(), mi.min_length, mi.length, mi.crc_extra);
        fcu_link->send_message(&ackMsg);

    } else if (!std::strcmp(responce->action, "TRIGGER")) {
        if (!std::strcmp(responce->command, "TRIGGER_SURVEY")) {
            mavlink::common::msg::CAMERA_IMAGE_CAPTURED cic;
            memset(&cic, 0, sizeof(mavlink::common::msg::CAMERA_IMAGE_CAPTURED));
            cic.camera_id = 1; /*< Camera ID (1 for first, 2 for second, etc.) */
            cic.image_index = 0; /*< Zero based index of this image (image count since armed -1) */
            cic.capture_result = (responce->result == 0); /*< Boolean indicating success (1) or failure (0) while capturing this image. */
            auto mi = cic.get_message_info();
            mavlink::mavlink_message_t ackMsg;
            mavlink::MsgMap ack_map(ackMsg);
            cic.serialize(ack_map);
            mavlink::mavlink_finalize_message(&ackMsg, fcu_link->get_system_id(),
                                              fcu_link->get_component_id(), mi.min_length, mi.length, mi.crc_extra);
            fcu_link->send_message(&ackMsg);

        } else if (!std::strcmp(responce->command, "TRIGGER_TEST_IMAGE")) {
            mavlink::common::msg::COMMAND_ACK ack;
            ack.command = responce->param_index;

            // Fill in the ack. Should be cleaned up when we have proper return from handle_camera_trigger()
            ack.result = (responce->result == 0) ? static_cast<uint16_t>(mavlink::common::MAV_RESULT::ACCEPTED) :
                         static_cast<uint16_t>(mavlink::common::MAV_RESULT::FAILED);
            ack.progress = 100; // 100% not used

            auto mi = ack.get_message_info();
            mavlink::mavlink_message_t ackMsg;
            mavlink::MsgMap ack_map(ackMsg);
            ack.serialize(ack_map);
            mavlink::mavlink_finalize_message(&ackMsg, fcu_link->get_system_id(),
                                              fcu_link->get_component_id(), mi.min_length, mi.length, mi.crc_extra);
            fcu_link->send_message(&ackMsg);
        }
    }
}


/* Called by libmavconn. Triggered everytime a mavlink message
 * is received from FCU. This is the place to parse received mavlink messages and call
 * handling functions.
 */
void mavlink_callback(const mavlink_message_t *mmsg, Framing framing, MAVConnInterface::Ptr fcu_link, SocketClient *main_client)
{
    // Handle Message ID
    switch (mmsg->msgid)
    {
    case mavlink::common::msg::HEARTBEAT::MSG_ID:
    {
        // Emit a HB every time we get a HB from the FCU
        if (mmsg->sysid == 1) {
            mavlink::common::msg::HEARTBEAT hb;
            memset(&hb, 0, sizeof(mavlink::common::msg::HEARTBEAT)); // Set everthing to zero for now
            auto mi = hb.get_message_info();

            mavlink::mavlink_message_t hbMsg;
            mavlink::MsgMap hb_map(hbMsg);
            hb.serialize(hb_map);
            mavlink::mavlink_finalize_message(&hbMsg, fcu_link->get_system_id(),
                                              fcu_link->get_component_id(), mi.min_length, mi.length, mi.crc_extra);
            fcu_link->send_message(&hbMsg);
        }

        break;
    }

    case mavlink::common::msg::CAMERA_TRIGGER::MSG_ID:
    {
        mavlink::common::msg::CAMERA_TRIGGER trig;
        mavlink::MsgMap map(mmsg);
        trig.deserialize(map);
        std::cout << trig.to_yaml() << std::endl;

        camera_request *request = new camera_request{};
        strcpy(request->command, "TRIGGER_SURVEY");
        snprintf(request->action, sizeof(request->action), "TRIGGER");
        request->timestamp = boost::posix_time::microsec_clock::local_time();
        main_client->write(request, sizeof(*request));
        delete request;

        break;
    }

    case mavlink::common::msg::COMMAND_LONG::MSG_ID:
    {
        mavlink::common::msg::COMMAND_LONG cmd;
        mavlink::MsgMap map(mmsg);
        cmd.deserialize(map);

        if (cmd.command == static_cast<uint16_t>(mavlink::common::MAV_CMD::IMAGE_START_CAPTURE)) {
            std::cout << cmd.to_yaml() << std::endl;

            camera_request *request = new camera_request{};
            strcpy(request->command, "TRIGGER_TEST_IMAGE");
            snprintf(request->action, sizeof(request->action), "TRIGGER");
            request->param_index = cmd.command;
	        request->timestamp = boost::posix_time::microsec_clock::local_time();
            main_client->write(request, sizeof(*request));
            delete request;
        }

        break;
    }

    // Request all parameters of this component. After this request, all parameters are emitted.
    case mavlink::common::msg::PARAM_EXT_REQUEST_LIST::MSG_ID:
    {
        mavlink::common::msg::PARAM_EXT_REQUEST_LIST req_list;
        mavlink::MsgMap map(mmsg);
        req_list.deserialize(map);
        std::cout << req_list.to_yaml() << std::endl;
        get_camera_parameter("EXPOSURE_MODE", main_client);
        get_camera_parameter("ISO", main_client);
        get_camera_parameter("SHUTTERSPD", main_client);
        get_camera_parameter("APERTURE", main_client);
        get_camera_parameter("WHITE_BALANCE", main_client);
        get_camera_parameter("EXPOSURE_COMP", main_client);
        get_camera_parameter("COMPR_SETTING", main_client);

        break;
    }

    // Request to read the value of a parameter with the either the param_id string id or param_index.
    case mavlink::common::msg::PARAM_EXT_REQUEST_READ::MSG_ID:
    {
        //printf("PARAM_EXT_REQUEST_READ\n");
        mavlink::common::msg::PARAM_EXT_REQUEST_READ read;
        mavlink::MsgMap map(mmsg);
        read.deserialize(map);
        std::cout << read.to_yaml() << std::endl;

        if (read.param_index == -1) {
            get_camera_parameter(mavlink::to_string(read.param_id), main_client);
        } else {
            std::cout << "Requesting camera parameters using param_index is currently not supported" << std::endl;
        }

        break;
    }

    case mavlink::common::msg::PARAM_EXT_SET::MSG_ID:
    {
        int ret = -1;
        mavlink::common::msg::PARAM_EXT_SET set;
        mavlink::MsgMap map(mmsg);
        set.deserialize(map);
        std::cout << set.to_yaml() << std::endl;

        mavlink::mavlink_param_union_t union_value;
        memcpy(&union_value, &set.param_value, sizeof(float));

        uint8_t data = union_value.param_uint8;
        set_camera_parameter(mavlink::to_string(set.param_id), main_client, data);

        break;
    }

    default:
        // printf("Warning, did not handle message id %i\n",message.msgid);
        break;
    }
}

/* Handle camera requests */
void start_camera_server(SocketServer *serverport)
{
    /*
    CameraFunc* camera_func = new CameraFunc(serverport);
    camera_func->start();

    delete camera_func;
    */
}

/*
 * This is the entry point for the executable. The function establishes
 * connection first with the camera and configures it's settings, then
 * with FCU via mavlink. Then it waits forever as libmavconn calls mavlink_callback
 * from another thread to trigger images/setting-changes etc.
 */
int main(int argc, char **argv)
{
    char socket_name[100];
    snprintf(socket_name, 100, "mavlink2cam");
    printf("%s\n", "banana");
    SocketClient *main_client = new SocketClient(socket_name);

    while (false == main_client->connect()) {
        fprintf(stderr, "Cannot connect camera server: %s\n", strerror(errno));
        sleep(1);
        /*
        SocketServer *serverport = new SocketServer(socket_name);
        if (0 == fork()) {
            delete main_client;
            start_camera_server(serverport);
            delete serverport;
            exit(0);
        }
        delete serverport;
        
        if (false == main_client->connect()) {
            fprintf(stderr, "Cannot connect camera server: %s\n", strerror(errno));
            exit(0);
        }
        */
    }

    // Send camera initialization request with main_client
    // char command[100];
    // snprintf(command, 100, "INITIALIZE");
    // main_client->write(command, sizeof(command));
    camera_request *initialize = new camera_request{};
    snprintf(initialize->command, 100, "INITIALIZE");
    main_client->write(initialize, sizeof(*initialize));
    delete initialize;

    // Get responce of initialization
    camera_responce *responce = new camera_responce{};
    main_client->read(responce, sizeof(*responce));

    // Configure connection to FCU
    std::string fcu_url, gcs_url;
    std::string fcu_protocol;
    int system_id, component_id;
    int tgt_system_id, tgt_component_id;

    MAVConnInterface::Ptr fcu_link;
    fcu_url = "serial:///dev/ttyUSB0:57600";
    gcs_url = "udp://@";
    fcu_protocol = "v2.0";
    system_id = 1;
    component_id = static_cast<uint8_t>(mavlink::common::MAV_COMPONENT::COMP_ID_CAMERA);
    tgt_system_id = 1;
    tgt_component_id = 1;

    printf("%d  %s\n", argc, argv[1]);
    if (argc > 1) {
        for (int i = 1; i < (argc); i++) {
            if (!strcmp("-d", argv[i]) && ((i + 1) < argc)) {
                // Update mavlink connection device
                printf("%s\n", argv[i + 1]);
                fcu_url = std::string("serial://") + std::string(argv[i + 1]);
                printf("Setting Mavlink Device to %s\n", fcu_url.c_str());
                break;
            } else if (!strcmp("-sitl", argv[i])) {
                fcu_url = "udp://127.0.0.1:14540@";
                printf("%s\n", "Starting in SITL mode at 127.0.0.1:14540");
            }
        }
    }

    try {
        fcu_link = MAVConnInterface::open_url(fcu_url, system_id, component_id);
        // may be overridden by URL
        fcu_link->set_system_id(system_id);
        fcu_link->set_component_id(component_id);
        printf("%s\n", "Connected");
    }

    catch (mavconn::DeviceError &ex) {
        printf("FCU: %s\n", ex.what());
        return 0;
    }

    if (fcu_protocol == "v1.0") {
        fcu_link->set_protocol_version(mavconn::Protocol::V10);

    } else if (fcu_protocol == "v2.0") {
        fcu_link->set_protocol_version(mavconn::Protocol::V20);

    } else {
        printf("Unknown FCU protocol: \"%s\", should be: \"v1.0\" or \"v2.0\". Used default v1.0.\n", fcu_protocol.c_str());
        fcu_link->set_protocol_version(mavconn::Protocol::V10);
    }

    fcu_link->message_received_cb = std::bind(&mavlink_callback, std::placeholders::_1, std::placeholders::_2, fcu_link, main_client);
    fcu_link->port_closed_cb = []() {
        printf("FCU connection closed, application will be terminated.\n");
        return 0;
    };

    int read;
    while(1) {
        // Handle responce from server and send back mavlink messages
        read = main_client->read(responce, sizeof(*responce));
        if (read != 0) {
            handle_responce(responce, fcu_link);
        }
    }

    return 0;
}
