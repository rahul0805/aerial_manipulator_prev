// MESSAGE PARAM_COMMIT_TRANSACTION support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief PARAM_COMMIT_TRANSACTION message
 *
 * Request to end the current parameter transaction. This message will have effect only if a transaction was previously opened using the PARAM_START_TRANSACTION message. The response will contain the same message but with a response attached to it. The response can either be a success/failure or and in progress in case the receiving side takes some time to apply the parameters.
 */
struct PARAM_COMMIT_TRANSACTION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 329;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 121;
    static constexpr auto NAME = "PARAM_COMMIT_TRANSACTION";


    uint8_t target_system; /*<  System ID */
    uint8_t target_component; /*<  Component ID */
    uint8_t param_action; /*<  Commit or cancel an ongoing transaction. */
    uint16_t transaction_id; /*<  Identifier for a specific transaction. */
    uint8_t response; /*<  Message acceptance response (sent back to GS). */


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
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  param_action: " << +param_action << std::endl;
        ss << "  transaction_id: " << transaction_id << std::endl;
        ss << "  response: " << +response << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << transaction_id;                // offset: 0
        map << target_system;                 // offset: 2
        map << target_component;              // offset: 3
        map << param_action;                  // offset: 4
        map << response;                      // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> transaction_id;                // offset: 0
        map >> target_system;                 // offset: 2
        map >> target_component;              // offset: 3
        map >> param_action;                  // offset: 4
        map >> response;                      // offset: 5
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
