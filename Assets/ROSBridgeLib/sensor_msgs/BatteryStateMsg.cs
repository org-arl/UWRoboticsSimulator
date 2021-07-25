using SimpleJSON;

namespace ROSBridgeLib
{
    namespace sensor_msgs
    {
        public class BatteryStateMsg : ROSBridgeMsg
        {
            // Power supply status constants
            public enum PowerSupplyStatus
            {
                UNKNOWN = 0,
                CHARGING = 1,
                DISCHARGING = 2,
                NOT_CHARGING = 3,
                FULL = 4
            }

            // Power supply health constants
            public enum PowerSupplyHealth
            {
                UNKNOWN = 0,
                GOOD = 1,
                OVERHEAT = 2,
                DEAD = 3,
                OVERVOLTAGE = 4,
                UNSPEC_FAILURE = 5,
                COLD = 6,
                WATCHDOG_TIMER_EXPIRE = 7,
                SAFETY_TIMER_EXPIRE = 8
            }

            // Power supply technology (chemistry) constants
            public enum PowerSupplyTechnology
            {
                UNKNOWN = 0,
                NIMH = 1,
                LION = 2,
                LIPO = 3,
                LIFE = 4,
                NICD = 5,
                LIMN = 6
            }

            std_msgs.HeaderMsg _header;
            float _voltage;          // Voltage in Volts (Mandatory)
            float _current;          // Negative when discharging (A)  (If unmeasured NaN)
            float _charge;           // Current charge in Ah  (If unmeasured NaN)
            float _capacity;         // Capacity in Ah (last full capacity)  (If unmeasured NaN)
            float _design_capacity;  // Capacity in Ah (design capacity)  (If unmeasured NaN)
            float _percentage;       // Charge percentage on 0 to 1 range  (If unmeasured NaN)
            PowerSupplyStatus _power_supply_status;     // The charging status as reported. Values defined above
            PowerSupplyHealth _power_supply_health;     // The battery health metric. Values defined above
            PowerSupplyTechnology _power_supply_technology; // The battery chemistry. Values defined above
            bool _present;           // True if the battery is present

            float[] _cell_voltage;     // An array of individual cell voltages for each cell in the pack
                                       // If individual voltages unknown but number of cells known set each to NaN
            string _location;          // The location into which the battery is inserted. (slot number or plug)
            string _serial_number;     // The best approximation of the battery serial number

            public BatteryStateMsg(JSONNode msg)
            {
                _header = new std_msgs.HeaderMsg(msg["header"]);
                _voltage = msg["voltage"].AsFloat;
                _current = msg["current"].AsFloat;
                _charge = msg["charge"].AsFloat;
                _capacity = msg["capacity"].AsFloat;
                _design_capacity = msg["design_capacity"].AsFloat;
                _percentage = msg["percentage"].AsFloat;
                _power_supply_status = (PowerSupplyStatus)msg["power_supply_status"].AsInt;
                _power_supply_health = (PowerSupplyHealth)msg["power_supply_health"].AsInt;
                _power_supply_technology = (PowerSupplyTechnology)msg["power_supply_technology"].AsInt;
                _present = msg["present"].AsBool;

                JSONArray cell_voltages_temp = msg["cell_voltage"].AsArray;
                _cell_voltage = new float[cell_voltages_temp.Count];
                for (int i = 0; i < _cell_voltage.Length; i++)
                {
                    _cell_voltage[i] = cell_voltages_temp[i].AsFloat;
                }

                _location = msg["location"].Value;
                _serial_number = msg["serial_number"].Value;
            }

            public static string getMessageType()
            {
                return "sensor_msgs/BatteryState Message";
            }

            public std_msgs.HeaderMsg GetHeader()
            {
                return _header;
            }

            public float GetVoltage()
            {
                return _voltage;
            }

            public float GetCurrent()
            {
                return _current;
            }

            public float GetCharge()
            {
                return _charge;
            }

            public float GetCapacity()
            {
                return _capacity;
            }

            public float GetDesignCapacity()
            {
                return _design_capacity;
            }

            public float GetPercentage()
            {
                return _percentage;
            }

            public PowerSupplyStatus GetPowerSupplyStatus()
            {
                return _power_supply_status;
            }

            public PowerSupplyHealth GetPowerSupplyHealth()
            {
                return _power_supply_health;
            }

            public PowerSupplyTechnology GetPowerSupplyTechnology()
            {
                return _power_supply_technology;
            }

            public bool IsPresent()
            {
                return _present;
            }

            public float[] GetCellVoltages()
            {
                return (float[])_cell_voltage.Clone();
            }

            public string GetLocation()
            {
                return _location;
            }

            public string GetSerialNumber()
            {
                return _serial_number;
            }

            override public string ToString()
            {
                return string.Format("Battery Present: {0}\n\tVoltage: {1}\n\tPercentage: {2}", _present, _voltage, _percentage);
            }
        }
    }
}
