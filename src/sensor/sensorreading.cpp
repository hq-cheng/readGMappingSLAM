#include "sensor/sensorreading.h"

namespace Gmapping {

    SensorReading::SensorReading( const Sensor* s, double time ) {
        m_sensor = s;
        m_time = time;
    }

    SensorReading::~SensorReading() {
    }

} // end namespace