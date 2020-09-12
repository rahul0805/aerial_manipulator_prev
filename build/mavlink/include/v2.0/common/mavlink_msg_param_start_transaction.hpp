// MESSAGE PARAM_START_TRANSACTION support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief PARAM_START_TRANSACTION message
 *
 * Request to start a new parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The response (ack) will contain the same message but with a response attached to it.
 */
struct PARAM_START_TRANSACTION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 328;
    static constexpr size_t LENGTH = 6;
    static constexpr size_t MIN_LENGTH = 6;
    static constexpr uint8_t CRC_EXTRA = 160;
    static constexpr auto NAME = "PARAM_START_TRANSACTION";


    uint8_t target_system; /*<  System ID */
    uint8_t target_component; /*<  Component ID */
    uint8_t param_transport; /*<  Possible transport layers to set and get parameters via mavlink during a parameter transaction. */
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
        ss << "  param_transport: " << +param_transport << std::endl;
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
        map << param_transport;               // offset: 4
        map << response;                      // offset: 5
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> transaction_id;                // offset: 0
        map >> target_system;                 // offset: 2
        map >> target_component;              // offset: 3
        map >> param_transport;               // offset: 4
        map >> response;                      // offset: 5
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
