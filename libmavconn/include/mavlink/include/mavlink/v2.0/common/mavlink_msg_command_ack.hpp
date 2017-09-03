// MESSAGE COMMAND_ACK support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief COMMAND_ACK message
 *
 * Report status of a command. Includes feedback whether the command was executed.
 */
struct COMMAND_ACK : mavlink::Message {
    static constexpr msgid_t MSG_ID = 77;
    static constexpr size_t LENGTH = 4;
    static constexpr size_t MIN_LENGTH = 3;
    static constexpr uint8_t CRC_EXTRA = 143;
    static constexpr auto NAME = "COMMAND_ACK";


    uint16_t command; /*< Command ID, as defined by MAV_CMD enum. */
    uint8_t result; /*< See MAV_RESULT enum */
    uint8_t progress; /*< WIP: Needs to be set when MAV_RESULT is MAV_RESULT_IN_PROGRESS, values from 0 to 100 for progress percentage, 255 for unknown progress. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  command: " << command << std::endl;
        ss << "  result: " << +result << std::endl;
        ss << "  progress: " << +progress << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << command;                       // offset: 0
        map << result;                        // offset: 2
        map << progress;                      // offset: 3
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> command;                       // offset: 0
        map >> result;                        // offset: 2
        map >> progress;                      // offset: 3
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
