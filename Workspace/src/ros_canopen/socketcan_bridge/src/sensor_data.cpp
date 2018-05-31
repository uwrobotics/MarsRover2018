
#include <socketcan_bridge/sensor_data.h>
#include <vector>
#include <limits.h>
#include <can_msgs/Frame.h>

namespace socketcan_bridge // TODO Use this class for sending limit switch data
{
	SensorData::SensorData(){
		thermistors_.resize(0);
    	currentSensors_.resize(0);
    	// limitSwitches_.resize(0);
        tempSensors_.resize(3);
        humiditySensors_.resize(3);
	}

    SensorData::SensorData(uint8_t limitSwitchSize, uint8_t currentSensorSize, uint8_t thermistorSize)
    {
    	thermistors_.resize(thermistorSize);
    	currentSensors_.resize(currentSensorSize);
    	// limitSwitches_.resize(limitSwitchSize);
        tempSensors_.resize(3);
        humiditySensors_.resize(3);
    }

    void SensorData::setThermistors(float value, uint8_t id)
    {
    	thermistors_[id] = value;
    }

    void SensorData::setCurrentSensors(float value, uint8_t id)
    {
    	currentSensors_[id] = value;
    }

    void SensorData::setScienceContainer(uint32_t sciLimSwitch)
    {
        container_.limitSwitch = sciLimSwitch;
    }

    void SensorData::setScienceContainer(science_msgs::Sensor sensor, uint8_t id)
    {
        if (id%2 == 0)
        {
            tempSensors_[id/2-3].data = sensor.data;
            tempSensors_[id/2-3].stamp = sensor.stamp;
            container_.tempSensors = tempSensors_;
        }
        else
        {
            humiditySensors_[id/2-2].data = sensor.data;
            humiditySensors_[id/2-2].stamp = sensor.stamp;
            container_.humiditySensors = humiditySensors_;
        }
    }

    void SensorData::setScienceContainer(science_msgs::UVSensor sensor, uint8_t id)
    {
        if (id == 3)
        {
            container_.uvSensor.data = sensor.data;
            container_.uvSensor.stamp = sensor.stamp;
        }
        else if (id == 4)
        {
            container_.gasSensor.data = sensor.data;
            container_.gasSensor.stamp = sensor.stamp;
        }
    }
    // void SensorData::setLimitSwitches(uint32_t value, uint8_t id)
    // {
    // 	limitSwitches_[id] = value;
    // }

    std::vector <float> SensorData::getThermistors(void)
    {
    	return thermistors_;
    }

    std::vector <float> SensorData::getCurrentSensors(void)
    {
    	return currentSensors_;
    }

    science_msgs::Sci_Container SensorData::getScienceContainer(void)
    {
        return container_;
    }

    // std::vector <uint8_t> SensorData::getLimitSwitches(void)
    // {
    // 	return limitSwitches_;
    // }
};
