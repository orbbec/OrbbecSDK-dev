#include "G330HostProtocol.hpp"
#include "common/logger/Logger.hpp"

namespace libobsensor {
namespace g2r {

uint8_t getByteOffset(uint8_t data) {
    uint8_t mask = 0x01;

    int i;

    for(i = 0; i < 8; i++) {
        if(data & (mask << i)) {
            break;
        }
    }

    return i;
}

uint8_t getByteBit(uint8_t data, uint8_t mask) {
    uint8_t info = 0;

    mask = (0x80) >> (8 - mask - 1);

    uint8_t offset = getByteOffset(mask);

    info = (data & mask) >> offset;

    return info;
}

uint8_t getDWORDOffset(uint32_t data) {
    uint32_t mask = 0x01;

    int i;

    for(i = 0; i < 32; i++) {
        if(data & (mask << i)) {
            break;
        }
    }

    return i;
}
uint8_t getDWORDBit(uint32_t data, uint32_t mask) {
    uint8_t info = 0;

    mask = (0x80000000) >> (32 - mask - 1);

    uint8_t offset = getDWORDOffset(mask);

    info = (data & mask) >> offset;

    return info;
}

uint32_t getBinCRC32(const uint8_t *dataBuf, uint32_t length) {
    int      i      = 0;
    uint32_t oldCRC = 0xFFFFFFFF;
    for(i = 0; i < length; i++) {
        uint8_t  data       = dataBuf[i];
        uint32_t CRC        = oldCRC;
        uint8_t  CRCBit[32] = { 0 };

        CRCBit[0] = getByteBit(data, 6) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 24) ^ getDWORDBit(CRC, 30);
        CRCBit[1] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 1) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 24) ^ getDWORDBit(CRC, 25)
                    ^ getDWORDBit(CRC, 30) ^ getDWORDBit(CRC, 31);
        CRCBit[2] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 2) ^ getByteBit(data, 1) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 24)
                    ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 30) ^ getDWORDBit(CRC, 31);
        CRCBit[3] = getByteBit(data, 7) ^ getByteBit(data, 3) ^ getByteBit(data, 2) ^ getByteBit(data, 1) ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 26)
                    ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 31);
        CRCBit[4] = getByteBit(data, 6) ^ getByteBit(data, 4) ^ getByteBit(data, 3) ^ getByteBit(data, 2) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 24)
                    ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 30);
        CRCBit[5] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 3) ^ getByteBit(data, 1)
                    ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 24) ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 29)
                    ^ getDWORDBit(CRC, 30) ^ getDWORDBit(CRC, 31);
        CRCBit[6] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 2) ^ getByteBit(data, 1)
                    ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 29) ^ getDWORDBit(CRC, 30) ^ getDWORDBit(CRC, 31);
        CRCBit[7] = getByteBit(data, 7) ^ getByteBit(data, 5) ^ getByteBit(data, 3) ^ getByteBit(data, 2) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 24)
                    ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 29) ^ getDWORDBit(CRC, 31);
        CRCBit[8] = getByteBit(data, 4) ^ getByteBit(data, 3) ^ getByteBit(data, 1) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 0) ^ getDWORDBit(CRC, 24)
                    ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 28);
        CRCBit[9] = getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 2) ^ getByteBit(data, 1) ^ getDWORDBit(CRC, 1) ^ getDWORDBit(CRC, 25)
                    ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 29);
        CRCBit[10] = getByteBit(data, 5) ^ getByteBit(data, 3) ^ getByteBit(data, 2) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 2) ^ getDWORDBit(CRC, 24)
                     ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 29);
        CRCBit[11] = getByteBit(data, 4) ^ getByteBit(data, 3) ^ getByteBit(data, 1) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 3) ^ getDWORDBit(CRC, 24)
                     ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 28);
        CRCBit[12] = getByteBit(data, 6) ^ getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 2) ^ getByteBit(data, 1) ^ getByteBit(data, 0)
                     ^ getDWORDBit(CRC, 4) ^ getDWORDBit(CRC, 24) ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 29)
                     ^ getDWORDBit(CRC, 30);
        CRCBit[13] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 5) ^ getByteBit(data, 3) ^ getByteBit(data, 2) ^ getByteBit(data, 1)
                     ^ getDWORDBit(CRC, 5) ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 29) ^ getDWORDBit(CRC, 30)
                     ^ getDWORDBit(CRC, 31);
        CRCBit[14] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 4) ^ getByteBit(data, 3) ^ getByteBit(data, 2) ^ getDWORDBit(CRC, 6)
                     ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 30) ^ getDWORDBit(CRC, 31);
        CRCBit[15] = getByteBit(data, 7) ^ getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 3) ^ getDWORDBit(CRC, 7) ^ getDWORDBit(CRC, 27)
                     ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 29) ^ getDWORDBit(CRC, 31);
        CRCBit[16] = getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 8) ^ getDWORDBit(CRC, 24) ^ getDWORDBit(CRC, 28)
                     ^ getDWORDBit(CRC, 29);
        CRCBit[17] = getByteBit(data, 6) ^ getByteBit(data, 5) ^ getByteBit(data, 1) ^ getDWORDBit(CRC, 9) ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 29)
                     ^ getDWORDBit(CRC, 30);
        CRCBit[18] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 2) ^ getDWORDBit(CRC, 10) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 30)
                     ^ getDWORDBit(CRC, 31);
        CRCBit[19] = getByteBit(data, 7) ^ getByteBit(data, 3) ^ getDWORDBit(CRC, 11) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 31);
        CRCBit[20] = getByteBit(data, 4) ^ getDWORDBit(CRC, 12) ^ getDWORDBit(CRC, 28);
        CRCBit[21] = getByteBit(data, 5) ^ getDWORDBit(CRC, 13) ^ getDWORDBit(CRC, 29);
        CRCBit[22] = getByteBit(data, 0) ^ getDWORDBit(CRC, 14) ^ getDWORDBit(CRC, 24);
        CRCBit[23] = getByteBit(data, 6) ^ getByteBit(data, 1) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 15) ^ getDWORDBit(CRC, 24) ^ getDWORDBit(CRC, 25)
                     ^ getDWORDBit(CRC, 30);
        CRCBit[24] = getByteBit(data, 7) ^ getByteBit(data, 2) ^ getByteBit(data, 1) ^ getDWORDBit(CRC, 16) ^ getDWORDBit(CRC, 25) ^ getDWORDBit(CRC, 26)
                     ^ getDWORDBit(CRC, 31);
        CRCBit[25] = getByteBit(data, 3) ^ getByteBit(data, 2) ^ getDWORDBit(CRC, 17) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 27);
        CRCBit[26] = getByteBit(data, 6) ^ getByteBit(data, 4) ^ getByteBit(data, 3) ^ getByteBit(data, 0) ^ getDWORDBit(CRC, 18) ^ getDWORDBit(CRC, 24)
                     ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 30);
        CRCBit[27] = getByteBit(data, 7) ^ getByteBit(data, 5) ^ getByteBit(data, 4) ^ getByteBit(data, 1) ^ getDWORDBit(CRC, 19) ^ getDWORDBit(CRC, 25)
                     ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 29) ^ getDWORDBit(CRC, 31);
        CRCBit[28] = getByteBit(data, 6) ^ getByteBit(data, 5) ^ getByteBit(data, 2) ^ getDWORDBit(CRC, 20) ^ getDWORDBit(CRC, 26) ^ getDWORDBit(CRC, 29)
                     ^ getDWORDBit(CRC, 30);
        CRCBit[29] = getByteBit(data, 7) ^ getByteBit(data, 6) ^ getByteBit(data, 3) ^ getDWORDBit(CRC, 21) ^ getDWORDBit(CRC, 27) ^ getDWORDBit(CRC, 30)
                     ^ getDWORDBit(CRC, 31);
        CRCBit[30] = getByteBit(data, 7) ^ getByteBit(data, 4) ^ getDWORDBit(CRC, 22) ^ getDWORDBit(CRC, 28) ^ getDWORDBit(CRC, 31);
        CRCBit[31] = getByteBit(data, 5) ^ getDWORDBit(CRC, 23) ^ getDWORDBit(CRC, 29);

        oldCRC = (CRCBit[31] << 31) + (CRCBit[30] << 30) + (CRCBit[29] << 29) + (CRCBit[28] << 28) + (CRCBit[27] << 27) + (CRCBit[26] << 26)
                 + (CRCBit[25] << 25) + (CRCBit[24] << 24) + (CRCBit[23] << 23) + (CRCBit[22] << 22) + (CRCBit[21] << 21) + (CRCBit[20] << 20)
                 + (CRCBit[19] << 19) + (CRCBit[18] << 18) + (CRCBit[17] << 17) + (CRCBit[16] << 16) + (CRCBit[15] << 15) + (CRCBit[14] << 14)
                 + (CRCBit[13] << 13) + (CRCBit[12] << 12) + (CRCBit[11] << 11) + (CRCBit[10] << 10) + (CRCBit[9] << 9) + (CRCBit[8] << 8) + (CRCBit[7] << 7)
                 + (CRCBit[6] << 6) + (CRCBit[5] << 5) + (CRCBit[4] << 4) + (CRCBit[3] << 3) + (CRCBit[2] << 2) + (CRCBit[1] << 1) + CRCBit[0];
    }

    return oldCRC;
}

HpStatus G330HostProtocol::updateFlashFunc(uint32_t offset, void *data, uint32_t dataLen, SetDataCallback callback) {
    HpStatus retStatus;
    if(!tranDataClosed_) {

        // erase
        retStatus = eraseFlashFunc(offset, dataLen);
        if(retStatus.statusCode != HP_STATUS_OK) {
            callback(DATA_TRAN_ERR_UNSUPPORTED, 0);
            return retStatus;
        }
        callback(DATA_TRAN_STAT_TRANSFERRING, 0);

        // write
        retStatus = writeFlashFunc(offset, data, dataLen, [callback](OBDataTranState state, uint8_t percent) {
            if(state != DATA_TRAN_STAT_DONE) {  // pass DATA_TRAN_STAT_DONE after verify success later
                callback(state, percent);
            }
        });
        if(retStatus.statusCode == HP_STATUS_OK && offset != ISP_FLASH_ADDR) {
            uint32_t verifiedCount = 0;
            retStatus              = readFlashFunc(offset, dataLen, [&](OBDataTranState state, OBDataChunk *dataChunk) {
                if(dataChunk->size > 0 && 0 == memcmp((uint8_t *)dataChunk->data, (uint8_t *)data + dataChunk->offset, dataChunk->size)) {
                    verifiedCount += dataChunk->size;
                }
                callback(DATA_TRAN_STAT_VERIFYING, (dataChunk->offset + dataChunk->size) * 100 / dataChunk->fullDataSize);
            });
            const auto verify      = (verifiedCount == dataLen);
            callback(verify ? DATA_TRAN_STAT_DONE : DATA_TRAN_ERR_VERIFY_FAILED, verifiedCount * 100 / dataLen);
        }
        else if(retStatus.statusCode == HP_STATUS_OK && offset == ISP_FLASH_ADDR) {
            callback(DATA_TRAN_STAT_DONE, 100);
        }
    }
    return retStatus;
}

HpStatus G330HostProtocol::writeFlashFunc(uint32_t offset, const void *data, uint32_t dataLen, SetDataCallback callback) {
    HpStatus err_code;
    uint16_t respDataSize;

    uint8_t         percent = 0;
    OBDataTranState state   = DATA_TRAN_STAT_TRANSFERRING;

    uint32_t packet_size = 0;

    bool is_last_packet = false;
    for(uint32_t packet_offset = 0; packet_offset < dataLen; packet_offset += FLASH_PACKET_SIZE) {
        std::lock_guard<std::recursive_mutex> lock(requestMutex_);
        zeroProtocolBuffers();  // reset request and response buffer cache
        is_last_packet = (packet_offset + FLASH_PACKET_SIZE >= dataLen);
        if(is_last_packet) {
            packet_size = dataLen - packet_offset;
        }
        else {
            packet_size = FLASH_PACKET_SIZE;
        }
        percent = (float)(packet_offset + packet_size) * 100.0 / (float)dataLen;

        if(tranDataClosed_) {
            state = DATA_TRAN_STAT_STOPPED;
            callback(state, percent);
            break;
        }

        // init request header
        // length(byte):   2      2     2     2    4   packet_size
        //      reqBuffer_: magic|size|opcode|nId|offset|data
        //
        //  HP_HEADER_SIZE = sizeof(magic|size|opcode|nId|)]
        if(is_last_packet) {
            // for flash write, the data length should be word aligned
            uint16_t wordAlignedSize = (packet_size + 3) / 4 * 4;
            initProtocolHeader(reqBuffer_, wordAlignedSize + 4, OPCODE_WRITE_FLASH);
        }
        else {
            initProtocolHeader(reqBuffer_, packet_size + 4, OPCODE_WRITE_FLASH);
        }
        // init data
        LOG_DEBUG("Update device data: addr=0x{0:x}, len={1}", offset + packet_offset, packet_size);
        *(uint32_t *)(reqBuffer_ + 8) = PREPARE_VAR32_IN_BUFFER((uint32_t)(offset + packet_offset));  // offset
        memcpy(reqBuffer_ + 12, (uint8_t *)data + packet_offset, packet_size);                        // data

        // request
        err_code = executeHostProtocol(reqBuffer_, HP_HEADER_SIZE + packet_size + 4, respBuffer_, NULL,
                                       &respDataSize);  //  size = HP_HEADER_SIZE + packet_size + sizeof(offset)
        if(err_code.statusCode != HP_STATUS_OK) {       // error
            state = DATA_TRAN_ERR_TRAN_FAILED;
        }
        else if(is_last_packet) {  // success & last packet
            percent = 100;
            state   = DATA_TRAN_STAT_DONE;
        }
        else {
            state = DATA_TRAN_STAT_TRANSFERRING;
        }
        callback(state, percent);

        if(err_code.statusCode != HP_STATUS_OK) {  // error: break
            break;
        }
    }

    {
        std::lock_guard<std::recursive_mutex> lock(requestMutex_);
        zeroProtocolBuffers();  // reset request and response buffer cache
        // finished write
        initProtocolHeader(reqBuffer_, 4, OPCODE_FINISH_WRITE_FLASH);  // finish with crc32
        uint32_t crc                  = getBinCRC32((uint8_t *)data, dataLen);
        *(uint32_t *)(reqBuffer_ + 8) = crc;
        err_code                      = executeHostProtocol(reqBuffer_, HP_HEADER_SIZE, respBuffer_, NULL, &respDataSize);
    }

    return err_code;
}
}  // namespace g2r
}  // namespace libobsensor