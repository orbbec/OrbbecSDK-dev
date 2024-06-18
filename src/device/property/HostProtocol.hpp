#pragma once

#include "ISourcePort.hpp"
#include <sstream>

namespace libobsensor {
namespace protocol {

#define HP_REQUEST_MAGIC 0x4d47   // MG
#define HP_RESPONSE_MAGIC 0x4252  // BR

#define MAX_PACKET_SIZE 4096 * 2
#define MAX_STRUCTURED_DATA_SIZE 1008  // 1024 -sizeof(ProtocolHeader) -sizeof(property_id) -sizeof(error_code)
#define FLASH_PAGE_SIZE 256
#define FLASH_BLOCK_SIZE 0x1000064k

#define HP_REQ_HEADER_SIZE sizeof(ReqHeader)
#define HP_RESP_HEADER_SIZE sizeof(RespHeader)

#define HP_NOT_READY_RETRIES 2

#define PROTOCOL_PROPERTY_ID_TYPE_SIZE 4  // sizeof(uint32_t)
#define PROTOCOL_CMD_VERSION_TYPE_SIZE 2  // sizeof(uint16_t)
#define PROTOCOL_VERSION_TYPE_SIZE 4      // sizeof(uint16_t)

enum HpOpCodes {
    OPCODE_GET_PROPERTY        = 1,
    OPCODE_SET_PROPERTY        = 2,
    OPCODE_GET_STRUCTURE_DATA  = 3,
    OPCODE_SET_STRUCTURE_DATA  = 4,
    OPCODE_HEARTBEAT_AND_STATE = 5,

    OPCODE_INIT_READ_RAW_DATA   = 17,
    OPCODE_READ_RAW_DATA        = 18,
    OPCODE_FINISH_READ_RAW_DATA = 19,

    // v1.1 protocol introduction
    OPCODE_GET_COMMAND_VERSION     = 26,  // v1.1 control command version number, determines how the control command parses data content
    OPCODE_GET_STRUCTURE_DATA_V1_1 = 27,  // v1.1 structure type data reading
    OPCODE_SET_STRUCTURE_DATA_V1_1 = 28,  // v.1.1 Structure type data setting

    OPCODE_INIT_READ_STRUCT_DATA_LIST    = 29,    // v1.1 Initialize structure type list data reading
    OPCODE_READ_STRUCT_DATA_LIST         = 30,    // v1.1 Structure type list data reading
    OPCODE_FINISH_READ_STRUCT_DATA_LIST  = 31,    // v1.1 ends structure type list data reading
    OPCODE_INIT_WRITE_STRUCT_DATA_LIST   = 32,    // v1.1 Initialize structure type list data writing
    OPCODE_WRITE_STRUCT_DATA_LIST        = 33,    // v1.1 Structure type list data writing
    OPCODE_FINISH_WRITE_STRUCT_DATA_LIST = 34,    // v1.1 ends writing structure type list data
    OPCODE_GET_PROTOCOL_VERSION          = 0xfe,  // Get the protocol version number supported by the firmware
};

// Command operation hpStatus code -> used for program hpStatus return
enum HpStatusCode {
    HP_STATUS_OK = 0,

    HP_STATUS_NO_DEVICE_FOUND,
    HP_STATUS_CONTROL_TRANSFER_FAILED,  // Transmission error
    HP_STATUS_DATA_TRANSFER_BUSY,       // Data transfer busy

    // Request error
    HP_STATUS_REQUEST_DATA_SIZE_ERROR,

    // Device response error
    HP_STATUS_DEVICE_RESPONSE_BAD_MAGIC,
    HP_STATUS_DEVICE_RESPONSE_WRONG_ID,
    HP_STATUS_DEVICE_RESPONSE_WRONG_OPCODE,
    HP_STATUS_DEVICE_RESPONSE_WRONG_DATA_SIZE,
    HP_STATUS_DEVICE_RESPONSE_ERROR,            // error code=0x0001~0x7fff
    HP_STATUS_DEVICE_RESPONSE_WARNING,          // error code=0x8000~0xfffe
    HP_STATUS_DEVICE_RESPONSE_ERROR_UNKNOWN,    // error code = 0xffff
    HP_STATUS_DEVICE_RESPONSE_DATA_SIZE_ERROR,  //
    HP_STATUS_DEVICE_RESPONSE_CMD_VERSION_UNMATCHED,
    HP_STATUS_UNKNOWN_ERROR = 0xffff,
};

// Status code returned by device command response -> used for communication between SDK and device
enum HpRespErrorCode {
    HP_RESP_OK = 0,

    // error
    HP_RESP_ERROR_INVALID_REQUEST     = 0x0001,
    HP_RESP_ERROR_UNSUPPORTED_REQUEST = 0x0002,
    HP_RESP_ERROR_DEVICE_BUSY         = 0x0003,
    HP_RESP_ERROR_ACCESS_FORBIDDEN    = 0x0004,

    // warning
    HP_RESP_WARNING_REPETITIVE_REQUEST = 0x8000,

    // unknown
    HP_RESP_ERROR_UNKNOWN = 0xffff,
};

struct HpStatus {
    HpStatusCode    statusCode    = HP_STATUS_OK;
    HpRespErrorCode respErrorCode = HP_RESP_OK;
    std::string     msg;
};

inline std::ostream &operator<<(std::ostream &os, const HpStatus &s) {
    return (os << "HpStatusCode: " << s.statusCode << ", respErrorCode: " << s.respErrorCode << ", msg: " << s.msg);
}

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;
    uint16_t sizeInHalfWords;  // bytes/2
    uint16_t opcode;
    uint16_t requestId;
} ReqHeader;

typedef struct {
    uint16_t magic;
    uint16_t sizeInHalfWords;  // bytes/2
    uint16_t opcode;
    uint16_t requestId;  // device response requestId same as request requestId, used for matching request and response
    uint16_t errorCode;
} RespHeader;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
} GetPropertyReq;

typedef struct {
    int32_t cur;  // int32_t means 4 bytes data, type is depends on propertyId
    int32_t max;
    int32_t min;
    int32_t def;
    int32_t step;
} PropertyData;

typedef struct {
    RespHeader   header;
    PropertyData data;
} GetPropertyResp;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
    int32_t   value;  // int32_t means 4 bytes data, type is depends on propertyId
} SetPropertyReq;

typedef struct {
    RespHeader header;
} SetPropertyResp;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
} GetStructureDataReq;

typedef struct {
    RespHeader header;
// if gnu compiler is used, ignore the zero-length array warning
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
    uint8_t data[0];
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

} GetStructureDataResp;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
// if gnu compiler is used, ignore the zero-length array warning
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
    uint8_t data[0];
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
} SetStructureDataReq;

typedef struct {
    RespHeader header;
} SetStructureDataResp;

typedef struct {
    ReqHeader header;
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
    uint8_t data[0];
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
} GetCmdVerDataResp;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
    uint16_t  cmdVer;
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
    uint8_t data[0];
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
} SetStructureDataV11Req;

typedef struct {
    RespHeader header;
} SetStructureDataV11Resp;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
} GetStructureDataV11Req;

typedef struct {
    RespHeader header;
    uint16_t   cmdVer;
    // if gnu compiler is used, ignore the zero-length array warning
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
    uint8_t data[0];
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
} GetStructureDataV11Resp;

typedef struct {
    RespHeader header;
    uint16_t   cmdVer;
    uint32_t   dataSize;
} InitStructureDataListResp;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
    uint32_t  offset;
    uint32_t  size;
} GetStructureDataListReq;

typedef struct {
    ReqHeader header;
    uint32_t  propertyId;
    uint32_t  offset;
    uint32_t  size;
} ReadRawData;

typedef struct {
    ReqHeader header;
    uint32_t  dataSize;
} GetReadDataResp;

#pragma pack(pop)

GetPropertyReq      *initGetPropertyReq(uint8_t *dataBuf, uint32_t propertyId);
SetPropertyReq      *initSetPropertyReq(uint8_t *dataBuf, uint32_t propertyId, uint32_t value);
GetStructureDataReq *initGetStructureDataReq(uint8_t *dataBuf, uint32_t propertyId);
SetStructureDataReq *initSetStructureDataReq(uint8_t *dataBuf, uint32_t propertyId, const uint8_t *data, uint16_t dataSize);
GetPropertyReq      *initGetCmdVersionReq(uint8_t *dataBuf, uint32_t propertyId);

GetStructureDataV11Req *initGetStructureDataV11Req(uint8_t *dataBuf, uint32_t propertyId);
GetStructureDataV11Req *initGetStructureDataListV11Req(uint8_t *dataBuf, uint32_t propertyId);

GetPropertyReq          *initStartGetStructureDataList(uint8_t *dataBuf, uint32_t propertyId);
GetStructureDataListReq *initReadStructureDataList(uint8_t *dataBuf, uint32_t propertyId, uint32_t offset, uint32_t dataSize);
GetPropertyReq          *initFinishGetStructureDataList(uint8_t *dataBuf, uint32_t propertyId);
GetPropertyReq          *initGetRawData(uint8_t *dataBuf, uint32_t propertyId, uint32_t cmd);
ReadRawData             *initReadRawData(uint8_t *dataBuf, uint32_t propertyId, uint32_t offset, uint32_t size);

GetPropertyResp         *parseGetPropertyResp(uint8_t *dataBuf, uint16_t dataSize);
SetPropertyResp         *parseSetPropertyResp(uint8_t *dataBuf, uint16_t dataSize);
GetStructureDataResp    *parseGetStructureDataResp(uint8_t *dataBuf, uint16_t dataSize);
GetStructureDataV11Resp *parseGetStructureDataV11Resp(uint8_t *dataBuf, uint16_t dataSize);
uint16_t                 getStructureDataSize(const GetStructureDataResp *resp);
SetStructureDataResp    *parseSetStructureDataResp(uint8_t *dataBuf, uint16_t dataSize);
GetCmdVerDataResp       *parseGetCmdVerDataResp(uint8_t *dataBuf, uint16_t dataSize);
GetReadDataResp         *parseGetReadDataResp(uint8_t *dataBuf, uint16_t dataSize);
InitStructureDataListResp *parseInitStructureDataListResp(uint8_t* dataBuf, uint16_t dataSize);

HpStatus execute(const std::shared_ptr<IVendorDataPort> &dataPort, uint8_t *reqData, uint16_t reqDataSize, uint8_t *respData, uint16_t *respDataSize);
bool     checkStatus(HpStatus stat, bool throwException = true);

}  // namespace protocol
}  // namespace libobsensor
