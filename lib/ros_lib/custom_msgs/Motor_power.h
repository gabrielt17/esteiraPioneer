#ifndef _ROS_custom_msgs_Motor_power_h
#define _ROS_custom_msgs_Motor_power_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class Motor_power : public ros::Msg
  {
    public:
      typedef int16_t _leftPW_type;
      _leftPW_type leftPW;
      typedef int16_t _rightPW_type;
      _rightPW_type rightPW;

    Motor_power():
      leftPW(0),
      rightPW(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_leftPW;
      u_leftPW.real = this->leftPW;
      *(outbuffer + offset + 0) = (u_leftPW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftPW.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->leftPW);
      union {
        int16_t real;
        uint16_t base;
      } u_rightPW;
      u_rightPW.real = this->rightPW;
      *(outbuffer + offset + 0) = (u_rightPW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightPW.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rightPW);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_leftPW;
      u_leftPW.base = 0;
      u_leftPW.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftPW.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->leftPW = u_leftPW.real;
      offset += sizeof(this->leftPW);
      union {
        int16_t real;
        uint16_t base;
      } u_rightPW;
      u_rightPW.base = 0;
      u_rightPW.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightPW.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rightPW = u_rightPW.real;
      offset += sizeof(this->rightPW);
     return offset;
    }

    virtual const char * getType() override { return "custom_msgs/Motor_power"; };
    virtual const char * getMD5() override { return "b0b15ff14f868ca04ffdc9e80d354840"; };

  };

}
#endif
