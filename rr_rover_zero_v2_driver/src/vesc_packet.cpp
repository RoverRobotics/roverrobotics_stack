// -*- mode:c++; fill-column: 100; -*-

#include "rr_rover_zero_v2_driver/vesc_packet.h"

#include <cassert>
#include <iterator>

#include <boost/range/begin.hpp>
#include <boost/range/distance.hpp>
#include <boost/range/end.hpp>

#include "rr_rover_zero_v2_driver/vesc_packet_factory.h"
#include "rr_rover_zero_v2_driver/datatypes.h"

namespace rr_rover_zero_v2_driver
{

  VescFrame::VescFrame(int payload_size)
  {
    assert(payload_size >= 0 && payload_size <= 1024);

    if (payload_size < 256)
    {
      // single byte payload size
      frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + payload_size));
      *frame_->begin() = 2;
      *(frame_->begin() + 1) = payload_size;
      payload_.first = frame_->begin() + 2;
    }
    else
    {
      // two byte payload size
      frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + 1 + payload_size));
      *frame_->begin() = 3;
      *(frame_->begin() + 1) = payload_size >> 8;
      *(frame_->begin() + 2) = payload_size & 0xFF;
      payload_.first = frame_->begin() + 3;
    }

    payload_.second = payload_.first + payload_size;
    *(frame_->end() - 1) = 3;
  }

  VescFrame::VescFrame(const BufferRangeConst &frame, const BufferRangeConst &payload)
  {
    /* VescPacketFactory::createPacket() should make sure that the input is valid, but run a few cheap
     checks anyway */
    assert(boost::distance(frame) >= VESC_MIN_FRAME_SIZE);
    assert(boost::distance(frame) <= VESC_MAX_FRAME_SIZE);
    assert(boost::distance(payload) <= VESC_MAX_PAYLOAD_SIZE);
    assert(std::distance(frame.first, payload.first) > 0 &&
           std::distance(payload.second, frame.second) > 0);

    frame_.reset(new Buffer(boost::begin(frame), boost::end(frame)));
    payload_.first = frame_->begin() + std::distance(frame.first, payload.first);
    payload_.second = frame_->begin() + std::distance(frame.first, payload.second);
  }

  VescPacket::VescPacket(const std::string &name, int payload_size, int payload_id) : VescFrame(payload_size), name_(name)
  {
    assert(payload_id >= 0 && payload_id < 256);
    assert(boost::distance(payload_) > 0);
    *payload_.first = payload_id;
    // printf("pay load id %d ", payload_id);
  }

  VescPacket::VescPacket(const std::string &name, boost::shared_ptr<VescFrame> raw) : VescFrame(*raw), name_(name)
  {
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketFWVersion::VescPacketFWVersion(boost::shared_ptr<VescFrame> raw) : VescPacket("FWVersion", raw)
  {
  }

  int VescPacketFWVersion::fwMajor() const
  {
    return *(payload_.first + 1);
  }

  int VescPacketFWVersion::fwMinor() const
  {
    return *(payload_.first + 2);
  }

  REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)

  VescPacketRequestFWVersion::VescPacketRequestFWVersion() : VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
  {
    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketValues::VescPacketValues(boost::shared_ptr<VescFrame> raw) : VescPacket("Values", raw)
  {
  }

  double VescPacketValues::temp_mos() const //double16 1e1
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 1)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 2)));
    return static_cast<double>(v) / 10.0;
  }
  double VescPacketValues::temp_motor() const //double16 1e1
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 3)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 4)));
    return static_cast<double>(v) / 10.0;
  }
  double VescPacketValues::current_motor() const //double32 1e2
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 5)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 6)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 7)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 8)));
    return static_cast<double>(v) / 100.0;
  }
  double VescPacketValues::current_in() const //double32 1e2
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 9)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 10)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 11)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 12)));
    return static_cast<double>(v) / 100.0;
  }
  double VescPacketValues::id() const //double32 1e2
  {
  int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 13)) << 24) +
                                   (static_cast<uint32_t>(*(payload_.first + 14)) << 16) +
                                   (static_cast<uint32_t>(*(payload_.first + 15)) << 8) +
                                   static_cast<uint32_t>(*(payload_.first + 16)));
  return static_cast<double>(v) / 100.0;
  }
  double VescPacketValues::iq() const //double32 1e2
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 17)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 18)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 19)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 20)));
    return static_cast<double>(v) / 100.0;
  }
  double VescPacketValues::duty_now() const //double16 1e3
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 21)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 22)));
    return static_cast<double>(v) / 1000.0;
  }
  double VescPacketValues::rpm() const //double32 1e0
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 23)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 24)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 25)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 26)));
    return static_cast<double>(v);
  }
  double VescPacketValues::v_in() const //double16 1e1
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 27)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 28)));
    return static_cast<double>(v) / 10.0;
  }

  double VescPacketValues::amp_hours() const //double32 1e4
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 29)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 30)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 31)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 32)));
    return static_cast<double>(v) / 10000.0;
  }
  double VescPacketValues::amp_hours_charged() const //double32 1e4
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 33)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 34)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 35)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 36)));
    return static_cast<double>(v) / 10000;
  }
  double VescPacketValues::watt_hours() const //double32 1e4
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 37)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 38)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 39)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 40)));
    return static_cast<double>(v) / 10000;
  }
  double VescPacketValues::watt_hours_charged() const //double32 1e4
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 41)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 42)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 43)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 44)));
    return static_cast<double>(v) / 10000;
  }
  int VescPacketValues::tachometer() const //int32
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 45)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 46)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 47)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 48)));
    return static_cast<int>(v);
  }
  int VescPacketValues::tachometer_abs() const //int32
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 49)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 50)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 51)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 52)));
    return static_cast<int>(v);
  }

  int VescPacketValues::fault_code() const //int 8
  {
    return static_cast<int>(*(payload_.first + 53));
  }

  double VescPacketValues::position() const //double32 1e6
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 54)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 55)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 56)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 57)));
    return static_cast<double>(v) / 1000000.0;
  }

  uint VescPacketValues::vesc_id() const //uint 8
  {
    return static_cast<uint>(*(payload_.first + 58));
  }
  double VescPacketValues::temp_mos_1() const //double 16 1e1
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 59)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 60)));
    return static_cast<double>(v) / 10.0;
  }

  double VescPacketValues::temp_mos_2() const //double 16 1e1
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 61)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 62)));
    return static_cast<double>(v) / 10.0;
  }
  double VescPacketValues::temp_mos_3() const //double 16 1e1
  {
    int16_t v = static_cast<int16_t>((static_cast<uint16_t>(*(payload_.first + 63)) << 8) +
                                     static_cast<uint16_t>(*(payload_.first + 64)));
    return static_cast<double>(v) / 10.0;
  }

  double VescPacketValues::vd() const //double32 1e3
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 65)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 66)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 67)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 68)));
    return static_cast<double>(v) / 1000.0;
  }
  double VescPacketValues::vq() const //double32 1e3
  {
    int32_t v = static_cast<int32_t>((static_cast<uint32_t>(*(payload_.first + 69)) << 24) +
                                     (static_cast<uint32_t>(*(payload_.first + 70)) << 16) +
                                     (static_cast<uint32_t>(*(payload_.first + 71)) << 8) +
                                     static_cast<uint32_t>(*(payload_.first + 72)));
    return static_cast<double>(v) / 1000.0;
  }

  REGISTER_PACKET_TYPE(COMM_GET_VALUES, VescPacketValues)

  VescPacketRequestValues::VescPacketRequestValues() : VescPacket("RequestFWVersion", 1, COMM_GET_VALUES)
  {
    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  VescPacketRequestValues::VescPacketRequestValues(int canid) : VescPacket("CanRequestFWVersion", 3, COMM_FORWARD_CAN)
  {
    *(payload_.first + 1) = static_cast<uint8_t>(canid);
    *(payload_.first + 2) = static_cast<uint8_t>(COMM_GET_VALUES);
    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }
  /*------------------------------------------------------------------------------------------------*/

  VescPacketSetDuty::VescPacketSetDuty(double duty) : VescPacket("SetDuty", 5, COMM_SET_DUTY)
  {
    /** @todo range check duty */

    int32_t v = static_cast<int32_t>(duty * 100000.0);

    *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  VescPacketSetDuty::VescPacketSetDuty(double duty, int m2_address) : VescPacket("CanSetDuty", 7, COMM_FORWARD_CAN)
  {
    int32_t v = static_cast<int32_t>(duty * 100000.0);
    *(payload_.first + 1) = static_cast<uint8_t>(m2_address);
    *(payload_.first + 2) = static_cast<uint8_t>(COMM_SET_DUTY);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 5) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 6) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);
    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketSetCurrent::VescPacketSetCurrent(double current) : VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
  {
    int32_t v = static_cast<int32_t>(current * 1000.0);

    *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake) : VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
  {
    int32_t v = static_cast<int32_t>(current_brake * 1000.0);

    *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketSetRPM::VescPacketSetRPM(double rpm) : VescPacket("SetRPM", 5, COMM_SET_RPM)
  {
    int32_t v = static_cast<int32_t>(rpm);

    *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  VescPacketSetRPM::VescPacketSetRPM(double rpm, int m2_address) : VescPacket("CanSetRPM", 7, COMM_FORWARD_CAN)
  {
    int32_t v = static_cast<int32_t>(rpm);
    *(payload_.first + 1) = static_cast<uint8_t>(m2_address);
    *(payload_.first + 2) = static_cast<uint8_t>(COMM_SET_RPM);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 5) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 6) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);
    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketSetPos::VescPacketSetPos(double pos) : VescPacket("SetPos", 5, COMM_SET_POS)
  {
    /** @todo range check pos */

    int32_t v = static_cast<int32_t>(pos * 1000000.0);

    *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
    *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
    *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
    *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

  /*------------------------------------------------------------------------------------------------*/

  VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos) : VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
  {
    /** @todo range check pos */

    int16_t v = static_cast<int16_t>(servo_pos * 1000.0);

    *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
    *(payload_.first + 2) = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);

    VescFrame::CRC crc_calc;
    crc_calc.process_bytes(&(*payload_.first), boost::distance(payload_));
    uint16_t crc = crc_calc.checksum();
    *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
    *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
  }

} // namespace rr_rover_zero_v2_driver
