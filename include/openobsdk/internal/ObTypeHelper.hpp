#pragma once

#include <iostream>
#include <sstream>
#include "openobsdk//h/ObTypes.h"

// enum
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBPermissionType &data);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBFormat &data);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBSensorType &data);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBStreamType &data);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBFrameType &data);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBFrameMetadataType &data);

// struct
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBDepthWorkMode &data);
OB_EXTENSION_INTERNAL_API std::ostream &operator<<(std::ostream &os, const OBProtocolVersion &data);
