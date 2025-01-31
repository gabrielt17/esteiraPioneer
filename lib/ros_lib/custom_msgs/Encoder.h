#ifndef _ROS_custom_msgs_Encoder_h
#define _ROS_custom_msgs_Encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace custom_msgs
{

  class Encoder : public ros::Msg
  {
    public:
      typedef float _leftRPM_type;
      _leftRPM_type leftRPM;
      typedef float _rightRPM_type;
      _rightRPM_type rightRPM;

    Encoder():
      leftRPM(0),
      rightRPM(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_leftRPM;
      u_leftRPM.real = this->leftRPM;
      *(outbuffer + offset + 0) = (u_leftRPM.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftRPM.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftRPM.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftRPM.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftRPM);
      union {
        float real;
        uint32_t base;
      } u_rightRPM;
      u_rightRPM.real = this->rightRPM;
      *(outbuffer + offset + 0) = (u_rightRPM.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightRPM.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightRPM.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightRPM.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightRPM);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_leftRPM;
      u_leftRPM.base = 0;
      u_leftRPM.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftRPM.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftRPM.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftRPM.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftRPM = u_leftRPM.real;
      offset += sizeof(this->leftRPM);
      union {
        float real;
        uint32_t base;
      } u_rightRPM;
      u_rightRPM.base = 0;
      u_rightRPM.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightRPM.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightRPM.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightRPM.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightRPM = u_rightRPM.real;
      offset += sizeof(this->rightRPM);
     return offset;
    }

    virtual const char * getType() override { return "custom_msgs/Encoder"; };
    virtual const char * getMD5() override { return "d45b400ddf8168bc65c052f58bac5088"; };

  };

}
#endif
