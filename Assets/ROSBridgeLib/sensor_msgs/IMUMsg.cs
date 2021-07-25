using UnityEngine;
using SimpleJSON;
using System;

namespace ROSBridgeLib
{
    namespace sensor_msgs
    {
        public class IMUMsg : ROSBridgeMsg
        {
            std_msgs.HeaderMsg _header;
            geometry_msgs.QuaternionMsg _orientation;
            geometry_msgs.Vector3Msg _angular_velocity;
            geometry_msgs.Vector3Msg _linear_acceleration;

            Nullable<Quaternion> orientation = null;
            Nullable<Vector3> angular_velocity = null;
            Nullable<Vector3> linear_acceleration = null;

            double[] _orientation_covariance;        // Row major about x, y, z axes
            double[] _angular_velocity_covariance;   // Row major about x, y, z axes
            double[] _linear_velocity_covariance;    // Row major x, y, z 

            public IMUMsg(JSONNode msg)
            {
                _header = new std_msgs.HeaderMsg(msg["header"]);
                _orientation = new geometry_msgs.QuaternionMsg(msg["orientation"]);
                _angular_velocity = new geometry_msgs.Vector3Msg(msg["angular_velocity"]);
                _linear_acceleration = new geometry_msgs.Vector3Msg(msg["linear_acceleration"]);

                JSONArray temp = msg["orientation_covariance"].AsArray;
                _orientation_covariance = new double[temp.Count];
                for (int i = 0; i < _orientation_covariance.Length; i++)
                {
                    _orientation_covariance[i] = temp[i].AsDouble;
                }

                temp = msg["angular_velocity_covariance"].AsArray;
                _angular_velocity_covariance = new double[temp.Count];
                for (int i = 0; i < _angular_velocity_covariance.Length; i++)
                {
                    _angular_velocity_covariance[i] = temp[i].AsDouble;
                }

                temp = msg["linear_acceleration_covariance"].AsArray;
                _linear_velocity_covariance = new double[temp.Count];
                for (int i = 0; i < _linear_velocity_covariance.Length; i++)
                {
                    _linear_velocity_covariance[i] = temp[i].AsDouble;
                }
            }

            public static string getMessageType()
            {
                return "geometry_msgs/Quaternion";
            }

            public std_msgs.HeaderMsg GetHeader()
            {
                return _header;
            }

            public Quaternion GetOrientation()
            {
                if (!orientation.HasValue)
                {
                    orientation = new Quaternion(_orientation.GetX(), _orientation.GetY(), _orientation.GetZ(), _orientation.GetW());
                }
                return orientation.Value;
            }

            public Vector3 GetAngularVelocity()
            {
                if (!angular_velocity.HasValue)
                {
                    angular_velocity = new Vector3((float)_angular_velocity.GetX(), (float)_angular_velocity.GetY(), (float)_angular_velocity.GetZ());
                }
                return angular_velocity.Value;
            }
            
            public Vector3 GetLinearAcceleration()
            {
                if (!linear_acceleration.HasValue)
                {
                    linear_acceleration = new Vector3((float)_linear_acceleration.GetX(), (float)_linear_acceleration.GetY(), (float)_linear_acceleration.GetZ());
                }
                return linear_acceleration.Value;
            }

            public double[] GetOrientationCovariance()
            {
                return (double[])_orientation_covariance.Clone();
            }

            public double[] GetAngularVelocityCovariance()
            {
                return (double[])_angular_velocity_covariance.Clone();
            }

            public double[] GetLinearAccelerationCovariance()
            {
                return (double[])_linear_velocity_covariance.Clone();
            }
        }
    }
}
