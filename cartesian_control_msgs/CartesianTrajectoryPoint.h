// Generated by gencpp from file cartesian_control_msgs/CartesianTrajectoryPoint.msg
// DO NOT EDIT!


#ifndef CARTESIAN_CONTROL_MSGS_MESSAGE_CARTESIANTRAJECTORYPOINT_H
#define CARTESIAN_CONTROL_MSGS_MESSAGE_CARTESIANTRAJECTORYPOINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Accel.h>
#include <cartesian_control_msgs/CartesianPosture.h>

namespace cartesian_control_msgs
{
template <class ContainerAllocator>
struct CartesianTrajectoryPoint_
{
  typedef CartesianTrajectoryPoint_<ContainerAllocator> Type;

  CartesianTrajectoryPoint_()
    : time_from_start()
    , pose()
    , twist()
    , acceleration()
    , jerk()
    , posture()  {
    }
  CartesianTrajectoryPoint_(const ContainerAllocator& _alloc)
    : time_from_start()
    , pose(_alloc)
    , twist(_alloc)
    , acceleration(_alloc)
    , jerk(_alloc)
    , posture(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Duration _time_from_start_type;
  _time_from_start_type time_from_start;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::geometry_msgs::Accel_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;

   typedef  ::geometry_msgs::Accel_<ContainerAllocator>  _jerk_type;
  _jerk_type jerk;

   typedef  ::cartesian_control_msgs::CartesianPosture_<ContainerAllocator>  _posture_type;
  _posture_type posture;





  typedef boost::shared_ptr< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> const> ConstPtr;

}; // struct CartesianTrajectoryPoint_

typedef ::cartesian_control_msgs::CartesianTrajectoryPoint_<std::allocator<void> > CartesianTrajectoryPoint;

typedef boost::shared_ptr< ::cartesian_control_msgs::CartesianTrajectoryPoint > CartesianTrajectoryPointPtr;
typedef boost::shared_ptr< ::cartesian_control_msgs::CartesianTrajectoryPoint const> CartesianTrajectoryPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator1> & lhs, const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return lhs.time_from_start == rhs.time_from_start &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist &&
    lhs.acceleration == rhs.acceleration &&
    lhs.jerk == rhs.jerk &&
    lhs.posture == rhs.posture;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator1> & lhs, const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cartesian_control_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "02d556067c148166af2dabae6251c00f";
  }

  static const char* value(const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x02d556067c148166ULL;
  static const uint64_t static_value2 = 0xaf2dabae6251c00fULL;
};

template<class ContainerAllocator>
struct DataType< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartesian_control_msgs/CartesianTrajectoryPoint";
  }

  static const char* value(const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duration time_from_start\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Twist twist\n"
"geometry_msgs/Accel acceleration\n"
"# A more suitable datatype would be good, see https://github.com/ros/common_msgs/issues/137\n"
"geometry_msgs/Accel jerk\n"
"CartesianPosture posture\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Accel\n"
"# This expresses acceleration in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: cartesian_control_msgs/CartesianPosture\n"
"# Posture joint names may reflect a subset of all available joints (empty posture definitions are\n"
"# also possible). The length of posture_joint_names and posture_joint_values have to be equal.\n"
"\n"
"string[] posture_joint_names\n"
"float64[] posture_joint_values\n"
;
  }

  static const char* value(const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time_from_start);
      stream.next(m.pose);
      stream.next(m.twist);
      stream.next(m.acceleration);
      stream.next(m.jerk);
      stream.next(m.posture);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CartesianTrajectoryPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartesian_control_msgs::CartesianTrajectoryPoint_<ContainerAllocator>& v)
  {
    s << indent << "time_from_start: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.time_from_start);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Accel_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
    s << indent << "jerk: ";
    s << std::endl;
    Printer< ::geometry_msgs::Accel_<ContainerAllocator> >::stream(s, indent + "  ", v.jerk);
    s << indent << "posture: ";
    s << std::endl;
    Printer< ::cartesian_control_msgs::CartesianPosture_<ContainerAllocator> >::stream(s, indent + "  ", v.posture);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTESIAN_CONTROL_MSGS_MESSAGE_CARTESIANTRAJECTORYPOINT_H
