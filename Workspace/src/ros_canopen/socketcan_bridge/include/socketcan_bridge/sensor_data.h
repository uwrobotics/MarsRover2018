// #ifndef SENSOR_DATA
// #define SENSOR_DATA
//
// #include <ros/ros.h>
// #include <vector>
// #include <string>
// #include <can_msgs/Frame.h>
//
// namespace socketcan_bridge
// {
//     class SensorData // TODO Use this class for sending limit switch data
//     {
//         public:
//             SensorData();
//             SensorData(uint8_t therm, uint8_t curr, uint8_t limit);
//             void setThermistors(float value, uint8_t id);
//             void setCurrentSensors(float value, uint8_t id);
//             void setScienceContainer(uint32_t sciLimSwitch);
//             void setScienceContainer(science_msgs::Sensor sensor, uint8_t id);
//             void setScienceContainer(science_msgs::UVSensor sensor, uint8_t id);
//             std::vector<float> getThermistors(void);
//             std::vector<float> getCurrentSensors(void);
//             science_msgs::Sci_Container getScienceContainer(void);
//         private:
//             std::vector<float> thermistors_;
//             std::vector<float> currentSensors_;
//             science_msgs::Sci_Container container_;
//             std::vector<science_msgs::Sensor> tempSensors_;
//             std::vector<science_msgs::Sensor> humiditySensors_;
//     };
// };
//
// #endif
