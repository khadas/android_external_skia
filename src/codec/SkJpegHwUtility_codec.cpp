/*
 * Copyright 2015 Google Inc.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "SkJpegHwUtility_codec.h"
//#define LOG_NDEBUG 0
#define LOG_TAG "SkHwJpegUtility"
#include <utils/Log.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LARGEST_LENGTH (16 * 1024)

static void sk_init_source(HwJpegInputInfo* hwInfo)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    src->next_input_byte = (const unsigned char*)src->fBuffer;
    src->bytes_in_buffer = 0;
    src->cur_offset_instream = 0;
    src->fStream->rewind();
}

static HW_BOOL sk_fill_input_buffer(HwJpegInputInfo *hwInfo)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    size_t bytes = 0;

    if (hwInfo->justcaloutwh &&
            (src->cur_offset_instream + sk_hw_jpeg_source_mgr::kBufferSize > LARGEST_LENGTH)) {
        if (src->cur_offset_instream >= LARGEST_LENGTH) {
            ALOGE("%s src->cur_offset_instream >= LARGEST_LENGTH", __FUNCTION__);
            return false;
        }
        bytes = src->fStream->read(src->fBuffer, LARGEST_LENGTH - src->cur_offset_instream);
    } else {
        bytes = src->fStream->read(src->fBuffer, sk_hw_jpeg_source_mgr::kBufferSize);
    }

    // Note that JPEG is happy with less than the full read,
    // as long as the result is non-zero
    if (bytes == 0) {
        return false;
    }

    src->cur_offset_instream += bytes;
    src->next_input_byte = (const unsigned char*)src->fBuffer;
    src->bytes_in_buffer = bytes;
    ALOGV("%s finish, bytes = %d, cur_offset: = %ld",
          __FUNCTION__, bytes, src->cur_offset_instream);

    return true;
}

static HW_BOOL sk_skip_input_data(HwJpegInputInfo *hwInfo, long num_bytes)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    ALOGV("%s - num_bytes = %ld, cur offset = %ld",
          __FUNCTION__, num_bytes, src->cur_offset_instream);

    if (num_bytes > src->bytes_in_buffer) {
        long bytesToSkip = num_bytes - src->bytes_in_buffer;

        if (hwInfo->justcaloutwh &&
                (bytesToSkip + src->cur_offset_instream > LARGEST_LENGTH)) {
            ALOGE("%s skip over largest length", __FUNCTION__);
            return false;
        }

        while (bytesToSkip > 0) {
            long bytes = (long)src->fStream->skip(bytesToSkip);
            if (bytes <= 0 || bytes > bytesToSkip) {
                ALOGE("%s failure to skip bytes", __FUNCTION__);
                return false;
            }

            src->cur_offset_instream += bytes;
            bytesToSkip -= bytes;
        }
        src->next_input_byte = (const unsigned char*)src->fBuffer;
        src->bytes_in_buffer = 0;
    } else {
        src->next_input_byte += num_bytes;
        src->bytes_in_buffer -= num_bytes;
    }

    ALOGV("%s finish, cur offset = %ld", __FUNCTION__, src->cur_offset_instream);
    return true;
}

static HW_BOOL sk_resync_to_restart(HwJpegInputInfo *hwInfo)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    if (!src->fStream->rewind()) {
        ALOGE("%s failure to rewind", __FUNCTION__);
        return false;
    }

    src->cur_offset_instream = 0;
    src->next_input_byte = (const unsigned char*)src->fBuffer;
    src->bytes_in_buffer = 0;

    return true;
}

static HW_BOOL sk_seek_input_data(HwJpegInputInfo *hwInfo, long byte_offset)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    ALOGV("%s seek to %ld and cur offset %ld",
          __FUNCTION__, byte_offset, src->cur_offset_instream);

    if (byte_offset > src->cur_offset_instream) {
        src->fStream->skip(byte_offset - src->cur_offset_instream);
    } else if (byte_offset < src->cur_offset_instream) {
        src->fStream->rewind();
        src->fStream->skip(byte_offset);
    }

    src->cur_offset_instream = byte_offset;
    src->next_input_byte = (const unsigned char*)src->fBuffer;
    src->bytes_in_buffer = 0;

    return true;
}

/* fill destination with stream */
static int sk_fill_buffer(HwJpegInputInfo *hwInfo, void *des,
                          VPUMemLinear_t *newVpuMem, int w, int h)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    if (des == NULL) {
        ALOGE("%s null param in", __FUNCTION__);
        return -1;
    }

    SkMemoryStream *stream = NULL;
    size_t rdBytes = -1;
    long bytes = 0;
    char *destination = (char*)des;
    size_t *bytesHasIn = NULL;

    if (src->isVpuMem) {
        stream = ((SkJpegVPUMemStream*)(src->fStream))->baseStream;
        bytesHasIn = &(((SkJpegVPUMemStream*)(src->fStream))->bytesInStream);
        bytes = hwInfo->streamCtl.wholeStreamLength - *bytesHasIn;

        if (bytes <= 0) {
            hwInfo->streamCtl.wholeStreamLength = *bytesHasIn;
            goto HANDLE;
        } else if (bytes > hwInfo->streamCtl.wholeStreamLength) {
            return -1;
        }
        destination += *bytesHasIn;
    } else {
        bytes = hwInfo->streamCtl.wholeStreamLength - src->cur_offset_instream;
        stream = src->fStream;
    }

    rdBytes = stream->read(destination, bytes);
    ALOGV("%s real rdbytes: %d", __FUNCTION__, rdBytes);
    if (bytes > 0) {
        if (rdBytes <=0 || (long)rdBytes > bytes) {
            if (NULL != bytesHasIn ) {
                // read error
                *bytesHasIn = -1;
            }
            return -1;
        }
    }
    src->cur_offset_instream += rdBytes;
    // recompute whole length
    if ((long)rdBytes < bytes) {
        if (src->isVpuMem) {
            rdBytes = rdBytes + *bytesHasIn;
            *bytesHasIn = rdBytes;
        }
        hwInfo->streamCtl.wholeStreamLength = rdBytes;
        ALOGV("%s real whole stream length: %d", __FUNCTION__, rdBytes);
    }

HANDLE:
    // if stream getlength is wrong, will go follow codes
    unsigned char* buf = src->fBuffer;
    rdBytes = stream->read(buf, sk_hw_jpeg_source_mgr::kBufferSize);
    ALOGV("%s reReadbyte is %d", __FUNCTION__, rdBytes);
#ifndef UNLIKELY
#define UNLIKELY(x) __builtin_expect((x), false)
#else
#error UNLIKELY has been def
#endif
#ifndef LIKELY
#define LIKELY(x) __builtin_expect((x), true)
#else
#error LIKELY has been def
#endif
    if (rdBytes > 0){
        VPUMemLinear_t tmpVpuMem;
        tmpVpuMem.vir_addr = NULL;
        size_t mallocSize = hwInfo->streamCtl.wholeStreamLength + rdBytes;
        if (rdBytes < sk_hw_jpeg_source_mgr::kBufferSize) {
            mallocSize = (mallocSize + 255) & (~255);
            if (mallocSize < JPEG_INPUT_BUFFER) {
                mallocSize = JPEG_INPUT_BUFFER;
            }

            hw_jpeg_VPUMallocLinear(&tmpVpuMem, mallocSize);
            if (LIKELY(tmpVpuMem.vir_addr != NULL)) {
                memcpy(tmpVpuMem.vir_addr, des, hwInfo->streamCtl.wholeStreamLength);
                memcpy((unsigned char*)tmpVpuMem.vir_addr +
                       hwInfo->streamCtl.wholeStreamLength, buf, rdBytes);
                hwInfo->streamCtl.wholeStreamLength += rdBytes;
                src->cur_offset_instream += rdBytes;
            } else {
                goto ERROR;
            }
        } else {
            size_t tmp = mallocSize;
            if (LIKELY(w > 0 && h > 0)) {
                if (LIKELY(mallocSize < (size_t)(w * h))) {
                    mallocSize = w * h;
                } else {
                    mallocSize *= 4;
                }
            } else {
                mallocSize *= 4;
            }
            mallocSize = (mallocSize + 255) & (~255);
            if (mallocSize < JPEG_INPUT_BUFFER) {
                mallocSize = JPEG_INPUT_BUFFER;
            }
            hw_jpeg_VPUMallocLinear(&tmpVpuMem, mallocSize);
            if (LIKELY(tmpVpuMem.vir_addr != NULL)) {
                unsigned char *mem = (unsigned char*)tmpVpuMem.vir_addr;
                memcpy(mem, des, hwInfo->streamCtl.wholeStreamLength);
                mem += hwInfo->streamCtl.wholeStreamLength;
                memcpy(mem, buf, rdBytes);
                mem += rdBytes;

                rdBytes = stream->read(mem, mallocSize - tmp);
                if (rdBytes > 0) {
                    tmp += rdBytes;
                    do {
                        if (UNLIKELY(tmp >= (size_t)mallocSize)) {
                            mallocSize *= 2;
                            VPUMemLinear_t tmpVpuMem2;
                            tmpVpuMem2.vir_addr = NULL;
                            hw_jpeg_VPUMallocLinear(&tmpVpuMem2, mallocSize);
                            if (tmpVpuMem2.vir_addr != NULL) {
                                mem = (unsigned char*)tmpVpuMem2.vir_addr;
                                memcpy(mem, tmpVpuMem.vir_addr, tmp);
                                mem += tmp;
                                hw_jpeg_VPUFreeLinear(&tmpVpuMem);
                                tmpVpuMem = tmpVpuMem2;
                            } else {
                                //release vpumem
                                hw_jpeg_VPUFreeLinear(&tmpVpuMem);
                                goto ERROR;
                            }
                        } else {
                            break;
                        }
                        rdBytes = stream->read(mem, mallocSize - tmp);
                        if (rdBytes <= 0)
                            break;

                        tmp += rdBytes;
                    } while (true);
                } else {
                    goto ERROR;
                }

                hwInfo->streamCtl.wholeStreamLength = tmp;
            } else {
                goto ERROR;
            }
        }
        *newVpuMem = tmpVpuMem;
        if (tmpVpuMem.vir_addr != NULL && src->isVpuMem) {
            ((SkJpegVPUMemStream*)(src->fStream))
                    ->setNewMemory(newVpuMem, hwInfo->streamCtl.wholeStreamLength);
        }
    }
    src->next_input_byte = (const unsigned char*)src->fBuffer;
    src->bytes_in_buffer = 0;

    return hwInfo->streamCtl.wholeStreamLength;

ERROR:
    if(src->isVpuMem){
        *bytesHasIn = -1;
    }

    return -1;
}

/* fill thumbBuf with thumbmsg if has thumb */
int sk_fill_thumb(HwJpegInputInfo *hwInfo, void *thumbBuf)
{
    if (thumbBuf == NULL) {
        ALOGE("%s nu param in", __FUNCTION__);
        return -1;
    }

    if (!sk_seek_input_data(hwInfo, hwInfo->streamCtl.thumbOffset)) {
        ALOGE("%s seek fail", __FUNCTION__);
        return -1;
    }

    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    size_t bytes = src->fStream->read(thumbBuf, hwInfo->streamCtl.thumbLength);
    if (bytes <= 0 || (long)bytes != hwInfo->streamCtl.thumbLength) {
        ALOGE("%s read fail", __FUNCTION__);
        return -1;
    }
    src->cur_offset_instream += bytes;
    src->next_input_byte = (const unsigned char*)src->fBuffer;
    src->bytes_in_buffer = 0;

    return hwInfo->streamCtl.thumbLength;
}

static HW_BOOL sk_read_1_byte(HwJpegInputInfo *hwInfo, unsigned char *ch)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    if (src->bytes_in_buffer == 0) {
        if (!sk_fill_input_buffer(hwInfo)) {
            return false;
        }
    }

    src->bytes_in_buffer--;
    *ch = *src->next_input_byte++;

    return true;
}

void sk_get_vpumemInst(HwJpegInputInfo *hwInfo, VPUMemLinear_t *thumbBuf)
{
    sk_hw_jpeg_source_mgr *src = (sk_hw_jpeg_source_mgr*) hwInfo->streamCtl.inStream;
    if (!src->isVpuMem)
        return;

    SkJpegVPUMemStream *stream = (SkJpegVPUMemStream*) src->fStream;
    if (thumbBuf != NULL) {
        *thumbBuf = *(stream->getVpuMemInst());
    }
}

///////////////////////////////////////////////////////////////////////////////

sk_hw_jpeg_source_mgr::sk_hw_jpeg_source_mgr(SkMemoryStream *stream, HwJpegInputInfo *hwInfo,
                                             HW_BOOL vpuMem) : fStream(stream)
{
    isVpuMem = vpuMem;
    info = hwInfo;
    init_source = sk_init_source;
    fill_input_buffer = sk_fill_input_buffer;
    skip_input_data = sk_skip_input_data;
    resync_to_restart = sk_resync_to_restart;
    seek_input_data = sk_seek_input_data;
    fill_buffer = sk_fill_buffer;
    fill_thumb = sk_fill_thumb;
    read_1_byte = sk_read_1_byte;
    get_vpumemInst = sk_get_vpumemInst;
}

sk_hw_jpeg_source_mgr::~sk_hw_jpeg_source_mgr()
{
    fStream = NULL;
}

SkJpegVPUMemStream::SkJpegVPUMemStream(SkMemoryStream *stream, size_t *len)
{
    size_t size = *len;
    vpuMem.vir_addr = NULL;
    bytesInStream = 0;
    baseStream = stream;

    size_t mallocSize = (size + 255) & (~255);
    if (mallocSize < LARGEST_LENGTH)
        mallocSize = LARGEST_LENGTH;
    if (mallocSize < JPEG_INPUT_BUFFER) {
        mallocSize = JPEG_INPUT_BUFFER;
    }
    ALOGV("%s in with mallocSize %d",__FUNCTION__, mallocSize);
    hw_jpeg_VPUMallocLinear(&vpuMem, mallocSize);

    if (vpuMem.vir_addr != NULL) {
        stream->rewind();
        bool readApp1 = false, readApp0 = false, readSof = false, error = false;
        unsigned short twoBytes = 0;
        unsigned char *mem = (unsigned char *) vpuMem.vir_addr;
        while (bytesInStream < size && error == false) {
            if ((bytesInStream + 2 > LARGEST_LENGTH)
                    || (bytesInStream + 2 > mallocSize)) {
                error = true;
                break;
            }
            size_t readBytes = stream->read(&twoBytes, 2); // FIX ME if big edian
            ALOGV("%s meet mark: 0x%x, readBytes: %d", __FUNCTION__, twoBytes, readBytes);
            error = (readBytes <= 0);
            if (error)
                break;

            bytesInStream += readBytes;
            memcpy(mem, &twoBytes, sizeof(unsigned short));
            mem += readBytes;
            switch(twoBytes) {
            case 0xD9FF: //EOI
                readApp1 = true;
                readApp0 = true;
                readSof = true;
                break;
            case 0xD8FF: //SOI
                break;
            case 0xE0FF: //APP0
            case 0xE1FF: //APP1
            case 0xC0FF: //SOF0
            case 0xC2FF: //SOF2
                if (twoBytes == 0xC0FF || twoBytes == 0xC2FF) {
                    readSof = true;
                } else if (twoBytes == 0xE0FF) {
                    readApp0 = true;
                } else if (twoBytes == 0xE1FF) {
                    readApp1 = true;
                }
            default:
                if ((bytesInStream + 2 > LARGEST_LENGTH)
                        || (bytesInStream + 2 > mallocSize)) {
                    error = true;
                    break;
                }
                readBytes = stream->read(&twoBytes, 2);
                error = (readBytes <= 0);
                if (error)
                    break;

                bytesInStream += readBytes;
                memcpy(mem, &twoBytes, sizeof(unsigned short));
                mem += readBytes;
                twoBytes = ((twoBytes & 0x000000ff) << 8) | ((twoBytes & 0x0000ff00) >> 8);
                if ((bytesInStream + twoBytes - 2 > LARGEST_LENGTH)
                        || (bytesInStream + twoBytes - 2 > mallocSize)) {
                    error = true;
                    break;
                }
                readBytes = stream->read(mem, twoBytes - 2);
                error = (readBytes <= 0);
                if (error)
                    break;

                bytesInStream += readBytes;
                mem += readBytes;
                break;
            }
            if ((readApp1 || readApp0) && readSof) {
                // just read app0 app1 sofn; optimization for decodeFile()
                // when request size less than thumbnail in file
                break;
            }
        }

        if (error == false && bytesInStream > 0) {
            ALOGV("%s bytesInStream is : %d, size is : %d",
                  __FUNCTION__, bytesInStream, size);
            if (bytesInStream < mallocSize) {
                mem[0] = 0xFF;
                mem[1] = 0xD9; // set end
            }
            if (bytesInStream > size) {
                // wrong base stream length, set to bytesInStream
                *len = bytesInStream;
            }
            this->setMemory(vpuMem.vir_addr, *len, false);
        } else {
            this->setMemory(NULL, 0);
            hw_jpeg_VPUFreeLinear(&vpuMem);
            vpuMem.vir_addr = NULL;
        }
    } else {
        this->setMemory(NULL, 0);
    }
}

SkJpegVPUMemStream::~SkJpegVPUMemStream()
{
    if (vpuMem.vir_addr != NULL) {
        hw_jpeg_VPUFreeLinear(&vpuMem);
    }
}

VPUMemLinear_t* SkJpegVPUMemStream::getVpuMemInst()
{
    return &vpuMem;
}

void SkJpegVPUMemStream::setNewMemory(VPUMemLinear_t *src, size_t size)
{
    if (vpuMem.vir_addr != NULL) {
        hw_jpeg_VPUFreeLinear(&vpuMem);
    }

    vpuMem = *src;
    bytesInStream = size;
    this->setMemory(src->vir_addr, size, false);
}
