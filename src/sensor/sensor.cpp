#include "sensor/sensor.h"

namespace Gmapping {

    Sensor::Sensor( const string& name ) {
        m_name = name;
    }

    Sensor::~Sensor() {
    }

} // end namespace