// Generated by gencpp from file custom_msg/PointArray_msg.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSG_MESSAGE_POINTARRAY_MSG_H
#define CUSTOM_MSG_MESSAGE_POINTARRAY_MSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace custom_msg
{
template <class ContainerAllocator>
struct PointArray_msg_
{
  typedef PointArray_msg_<ContainerAllocator> Type;

  PointArray_msg_()
    : array()  {
    }
  PointArray_msg_(const ContainerAllocator& _alloc)
    : array(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Point_<ContainerAllocator> >> _array_type;
  _array_type array;





  typedef boost::shared_ptr< ::custom_msg::PointArray_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msg::PointArray_msg_<ContainerAllocator> const> ConstPtr;

}; // struct PointArray_msg_

typedef ::custom_msg::PointArray_msg_<std::allocator<void> > PointArray_msg;

typedef boost::shared_ptr< ::custom_msg::PointArray_msg > PointArray_msgPtr;
typedef boost::shared_ptr< ::custom_msg::PointArray_msg const> PointArray_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msg::PointArray_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msg::PointArray_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msg::PointArray_msg_<ContainerAllocator1> & lhs, const ::custom_msg::PointArray_msg_<ContainerAllocator2> & rhs)
{
  return lhs.array == rhs.array;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msg::PointArray_msg_<ContainerAllocator1> & lhs, const ::custom_msg::PointArray_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_msg::PointArray_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msg::PointArray_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msg::PointArray_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msg::PointArray_msg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msg::PointArray_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msg::PointArray_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msg::PointArray_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "11842a6e8a5f0ee08a3522f9fd5f1fd5";
  }

  static const char* value(const ::custom_msg::PointArray_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x11842a6e8a5f0ee0ULL;
  static const uint64_t static_value2 = 0x8a3522f9fd5f1fd5ULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msg::PointArray_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msg/PointArray_msg";
  }

  static const char* value(const ::custom_msg::PointArray_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msg::PointArray_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point[] array \n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::custom_msg::PointArray_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msg::PointArray_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.array);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointArray_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msg::PointArray_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msg::PointArray_msg_<ContainerAllocator>& v)
  {
    s << indent << "array[]" << std::endl;
    for (size_t i = 0; i < v.array.size(); ++i)
    {
      s << indent << "  array[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.array[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSG_MESSAGE_POINTARRAY_MSG_H
