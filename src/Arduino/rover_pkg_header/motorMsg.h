#ifndef _ROS_rover_pkg_motorMsg_h
#define _ROS_rover_pkg_motorMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rover_pkg
{

  class motorMsg : public ros::Msg
  {
    public:
      typedef int16_t _speed_type;
      _speed_type speed;
      typedef int16_t _turn_type;
      _turn_type turn;
      typedef int8_t _type_type;
      _type_type type;
      typedef int8_t _delay_type;
      _delay_type delay;

    motorMsg():
      speed(0),
      turn(0),
      type(0),
      delay(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        int16_t real;
        uint16_t base;
      } u_turn;
      u_turn.real = this->turn;
      *(outbuffer + offset + 0) = (u_turn.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_turn.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->turn);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      union {
        int8_t real;
        uint8_t base;
      } u_delay;
      u_delay.real = this->delay;
      *(outbuffer + offset + 0) = (u_delay.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->delay);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        int16_t real;
        uint16_t base;
      } u_turn;
      u_turn.base = 0;
      u_turn.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_turn.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->turn = u_turn.real;
      offset += sizeof(this->turn);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      union {
        int8_t real;
        uint8_t base;
      } u_delay;
      u_delay.base = 0;
      u_delay.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->delay = u_delay.real;
      offset += sizeof(this->delay);
     return offset;
    }

    virtual const char * getType() override { return "rover_pkg/motorMsg"; };
    virtual const char * getMD5() override { return "030f44a689105ea221a9fb2deeb091ab"; };

  };

}
#endif
