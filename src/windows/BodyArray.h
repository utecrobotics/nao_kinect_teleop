#ifndef _ROS_kinect_msgs_BodyArray_h
#define _ROS_kinect_msgs_BodyArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Bool.h"

namespace kinect_msgs
{

  class BodyArray : public ros::Msg
  {
    public:
      uint8_t body_length;
      geometry_msgs::Vector3 st_body;
      geometry_msgs::Vector3 * body;
      std_msgs::Bool left_hand;
      std_msgs::Bool right_hand;

    BodyArray():
      body_length(0), body(NULL),
      left_hand(),
      right_hand()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = body_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < body_length; i++){
      offset += this->body[i].serialize(outbuffer + offset);
      }
      offset += this->left_hand.serialize(outbuffer + offset);
      offset += this->right_hand.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t body_lengthT = *(inbuffer + offset++);
      if(body_lengthT > body_length)
        this->body = (geometry_msgs::Vector3*)realloc(this->body, body_lengthT * sizeof(geometry_msgs::Vector3));
      offset += 3;
      body_length = body_lengthT;
      for( uint8_t i = 0; i < body_length; i++){
      offset += this->st_body.deserialize(inbuffer + offset);
        memcpy( &(this->body[i]), &(this->st_body), sizeof(geometry_msgs::Vector3));
      }
      offset += this->left_hand.deserialize(inbuffer + offset);
      offset += this->right_hand.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "kinect_msgs/BodyArray"; };
    const char * getMD5(){ return "4ba581328c819cf4ed602618be227db9"; };

  };

}
#endif