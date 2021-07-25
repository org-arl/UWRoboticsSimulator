using SimpleJSON;

namespace ROSBridgeLib
{
    namespace sensor_msgs
    {
        /// <summary>
        /// Navigation Satellite fix for any Global Navigation Satellite System
        /// </summary>
        /// <see cref="http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html"/>
        /// <seealso cref="http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatStatus.html"/>
        /// <remarks>Specified using the WGS 84 reference ellipsoid</remarks>
        public class NavSatFixMsg : ROSBridgeMsg
        {
            /// <value>
            /// header.stamp specifies the ROS time for this measurement (the
            ///        corresponding satellite time may be reported using the
            ///        sensor_msgs/TimeReference message).
            /// </value>
            std_msgs.HeaderMsg _header;

            /// <summary>
            /// Satellite fix status information:
            /// Whether to output an augmented fix is determined by both the fix type and the last time differential corrections were received.
            /// A fix is valid when status >= STATUS_FIX.
            /// </summary>
            /// <see cref="http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatStatus.html"/>
            public enum NavSatStatus
            {
                NO_FIX = -1,        // unable to fix position
                FIX = 0,            // unaugmented fix
                SBAS_FIX = 1,        // with satellite-based augmentation
                GBAS_FIX = 2         // with ground-based augmentation
            }
            NavSatStatus _status;

            /// <summary>
            /// Satellite fix status information:
            /// Defines which Global Navigation Satellite System signals were used by the receiver.
            /// </summary>
            /// <see cref="http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatStatus.html"/>
            public enum NavSatService
            {
                GPS = 1,
                GLONASS = 2,
                COMPASS = 4,      // includes BeiDou.
                GALILEO = 8
            }
            NavSatService _service;

            /// <value> Latitude [degrees]. Positive is north of equator; negative is south.</value>
            double _latitude;
            /// <value> Longitude [degrees]. Positive is east of prime meridian; negative is west.</value>
            double _longitude;
            /// <value> Altitude [m]. Positive is above the WGS 84 ellipsoid</value>
            /// <remarks> (quiet NaN if no altitude is available).</remarks>
            double _altitude;

            /// <value>
            /// Position covariance [m^2] defined relative to a tangential plane
            /// through the reported position. The components are East, North, and
            /// Up (ENU), in row-major order.
            /// </value>
            /// <remarks> Beware: this coordinate system exhibits singularities at the poles.</remarks>
            double[] _position_covariance;

            /// <summary>
            /// If the covariance of the fix is known, fill it in completely. If the
            /// GPS receiver provides the variance of each measurement, put them
            /// along the diagonal. If only Dilution of Precision is available,
            /// estimate an approximate covariance from that.
            /// </summary>
            public enum PositionCovarianceType
            {
                UNKNOWN = 0,
                APPROXIMATED = 1,
                DIAGONAL_KNOWN = 2,
                KNOWN = 3
            }
            PositionCovarianceType _position_covariance_type;


            public NavSatFixMsg(JSONNode msg)
            {
                _header = new std_msgs.HeaderMsg(msg["header"]);
                _status = (NavSatStatus)msg["status"]["status"].AsInt;
                _service = (NavSatService)msg["status"]["service"].AsInt;
                _latitude = msg["latitude"].AsFloat;
                _longitude = msg["longitude"].AsFloat;
                _altitude = msg["altitude"].AsFloat;

                JSONArray temp_covar_array = msg["position_covariance"].AsArray;
                _position_covariance = new double[temp_covar_array.Count];
                for (int i = 0; i < _position_covariance.Length; i++)
                {
                    _position_covariance[i] = temp_covar_array[i].AsDouble;
                }

                _position_covariance_type = (PositionCovarianceType)msg["position_covariance_type"].AsInt;
            }

            public static string getMessageType()
            {
                return "sensor_msgs/NavSatFix";
            }

            public std_msgs.HeaderMsg GetHeader()
            {
                return _header;
            }

            public NavSatStatus GetStatus()
            {
                return _status;
            }

            public NavSatService GetService()
            {
                return _service;
            }

            public double GetLatitude()
            {
                return _latitude;
            }

            public double GetLongitude()
            {
                return _longitude;
            }

            public double GetAltitude()
            {
                return _altitude;
            }

            public double[] GetPositionCovaraince()
            {
                return (double[])_position_covariance.Clone();
            }

            public PositionCovarianceType GetPositionCovarianceType()
            {
                return _position_covariance_type;
            }

            public override string ToString()
            {
                return string.Format("Latitude: {0}, Longitude: {1}, Altitude: {2}", _latitude, _longitude, _altitude);
            }
        }
    }
}
