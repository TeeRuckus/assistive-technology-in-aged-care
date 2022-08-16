// Generated by gencpp from file fallen_analyser/cords.msg
// DO NOT EDIT!


#ifndef FALLEN_ANALYSER_MESSAGE_CORDS_H
#define FALLEN_ANALYSER_MESSAGE_CORDS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>

namespace fallen_analyser
{
template <class ContainerAllocator>
struct cords_
{
  typedef cords_<ContainerAllocator> Type;

  cords_()
    : xCord()
    , yCord()  {
    }
  cords_(const ContainerAllocator& _alloc)
    : xCord(_alloc)
    , yCord(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float32_<ContainerAllocator>  _xCord_type;
  _xCord_type xCord;

   typedef  ::std_msgs::Float32_<ContainerAllocator>  _yCord_type;
  _yCord_type yCord;





  typedef boost::shared_ptr< ::fallen_analyser::cords_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fallen_analyser::cords_<ContainerAllocator> const> ConstPtr;

}; // struct cords_

typedef ::fallen_analyser::cords_<std::allocator<void> > cords;

typedef boost::shared_ptr< ::fallen_analyser::cords > cordsPtr;
typedef boost::shared_ptr< ::fallen_analyser::cords const> cordsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fallen_analyser::cords_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fallen_analyser::cords_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fallen_analyser::cords_<ContainerAllocator1> & lhs, const ::fallen_analyser::cords_<ContainerAllocator2> & rhs)
{
  return lhs.xCord == rhs.xCord &&
    lhs.yCord == rhs.yCord;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fallen_analyser::cords_<ContainerAllocator1> & lhs, const ::fallen_analyser::cords_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fallen_analyser

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fallen_analyser::cords_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fallen_analyser::cords_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fallen_analyser::cords_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fallen_analyser::cords_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fallen_analyser::cords_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fallen_analyser::cords_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fallen_analyser::cords_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6466c8bc21dcd864939639eb1c9ad609";
  }

  static const char* value(const ::fallen_analyser::cords_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6466c8bc21dcd864ULL;
  static const uint64_t static_value2 = 0x939639eb1c9ad609ULL;
};

template<class ContainerAllocator>
struct DataType< ::fallen_analyser::cords_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fallen_analyser/cords";
  }

  static const char* value(const ::fallen_analyser::cords_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fallen_analyser::cords_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Float32 xCord\n"
"std_msgs/Float32 yCord\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float32\n"
"float32 data\n"
;
  }

  static const char* value(const ::fallen_analyser::cords_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fallen_analyser::cords_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.xCord);
      stream.next(m.yCord);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cords_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fallen_analyser::cords_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fallen_analyser::cords_<ContainerAllocator>& v)
  {
    s << indent << "xCord: ";
    s << std::endl;
    Printer< ::std_msgs::Float32_<ContainerAllocator> >::stream(s, indent + "  ", v.xCord);
    s << indent << "yCord: ";
    s << std::endl;
    Printer< ::std_msgs::Float32_<ContainerAllocator> >::stream(s, indent + "  ", v.yCord);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FALLEN_ANALYSER_MESSAGE_CORDS_H