/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/******************************* DisplayPort *******************************\
* Module: dp_auxretry.cpp                                                   *
*    Interface implemented by library client.                               *
*                                                                           *
\***************************************************************************/

#include "dp_internal.h"
#include "dp_auxbus.h"
#include "dp_auxretry.h"
#include "dp_messageheader.h"
#include "dp_printf.h"

#include "displayport.h"

using namespace DisplayPort;

namespace {

// Handle different transaction statuses and retry logic
AuxRetry::status handleTransaction(AuxBus::status status, unsigned completed, unsigned size, unsigned& retries) {
    if ((status == AuxBus::success) && (completed == size) && (completed != 0)) {
        return AuxRetry::ack;
    }

    if (status == AuxBus::defer) {
        if (retries) {
            --retries;
            return AuxRetry::defer;
        }
        return AuxRetry::defer;
    }

    if (status == AuxBus::nack) {
        return AuxRetry::nack;
    }

    if (completed == 0) {
        return AuxRetry::unsupportedRegister;
    }

    if (completed < size) {
        if (retries) {
            --retries;
            return AuxRetry::defer;
        }
        return AuxRetry::defer;
    }

    return AuxRetry::ack;
}

} // namespace

//
//    Read a DPCD address.
//        - allows size greater than single transaction/burst size
//        - handles defer retries
//        - handles nacks with incomplete data
//
AuxRetry::status AuxRetry::readTransaction(int address, NvU8* buffer, unsigned size, unsigned retries) {
    unsigned completed;
    AuxBus::status status;

    DP_ASSERT(size <= aux->transactionSize());

    do {
        status = aux->transaction(AuxBus::read, AuxBus::native, address, buffer, size, &completed);
        
        // Got success & requested data. Also size of returned data is expected & non zero.
        AuxRetry::status result = handleTransaction(status, completed, size, retries);
        if (result != AuxRetry::defer) {
            return result;
        }
    } while (retries);

    // Closest approximation is a defer
    return AuxRetry::defer;
}

//
//    Write a DPCD address.
//        - allows size greater than single transaction/burst size
//        - handles defer retries
//        - handles nacks with incomplete data
//
AuxRetry::status AuxRetry::writeTransaction(int address, NvU8* buffer, unsigned size, unsigned retries) {
    unsigned completed;
    AuxBus::status status;

    DP_ASSERT(size <= aux->transactionSize());

    do {
        status = aux->transaction(AuxBus::write, AuxBus::native, address, buffer, size, &completed);

        // Got success & requested data. Also size of returned data is expected & non zero.
        AuxRetry::status result = handleTransaction(status, completed, size, retries);
        if (result != AuxRetry::defer) {
            return result;
        }
    } while (retries);

    // Closest approximation is a defer
    return AuxRetry::defer;
}

//
//    Similar to readTransaction except that it supports reading
//    larger spans than AuxBus::transactionSize()
//
AuxRetry::status AuxRetry::read(int address, NvU8* buffer, unsigned size, unsigned retries) {
    for (unsigned i = 0; i < size; ) {
        unsigned chunkSize = DP_MIN(size - i, aux->transactionSize());
        AuxRetry::status status = readTransaction(address + i, buffer + i, chunkSize, retries);
        if (status != AuxRetry::ack) {
            return status;
        }
        i += chunkSize;
    }
    return AuxRetry::ack;
}

//
//    Similar to writeTransaction except that it supports writing
//    larger spans than AuxBus::transactionSize()
//
AuxRetry::status AuxRetry::write(int address, NvU8* buffer, unsigned size, unsigned retries) {
    for (unsigned i = 0; i < size; ) {
        unsigned chunkSize = DP_MIN(size - i, aux->transactionSize());
        AuxRetry::status status = writeTransaction(address + i, buffer + i, chunkSize, retries);
        if (status != AuxRetry::ack) {
            return status;
        }
        i += chunkSize;
    }
    return AuxRetry::ack;
}

AuxBus::status AuxLogger::transaction(Action action, Type type, int address,
                              NvU8* buffer, unsigned sizeRequested,
                              unsigned* sizeCompleted, unsigned* pNakReason,
                              NvU8 offset, NvU8 nWriteTransactions) {
    AuxBus::status result = bus->transaction(action, type, address, buffer, sizeRequested, sizeCompleted);
    hint[0] = 0;

    //
    // Do the hex dump.
    //   - We can't make library calls
    //   - We need to do this in one printf
    if (result == AuxBus::success) {
        if (type == AuxBus::native &&
            (address == NV_DPCD_MBOX_DOWN_REQ || address == NV_DPCD_MBOX_UP_REP ||
             address == NV_DPCD_MBOX_DOWN_REP || address == NV_DPCD_MBOX_UP_REQ)) {
            unsigned len = *sizeCompleted;
            Buffer storage(buffer, len);
            BitStreamReader reader(&storage, 0, len * 8);
            MessageHeader header;
            DisplayPort::decodeHeader(&reader, &header, Address(1));
            Address::StringBuffer sb;
            DP_USED(sb);
            dpHexDump(hex, sizeof(hex), buffer, header.headerSizeBits / 8);
            dpHexDump(hex_body, sizeof(hex_body), buffer + header.headerSizeBits / 8, len - header.headerSizeBits / 8);

#if defined(_DEBUG) || defined(DEBUG)
            const char* name = "";
            if (header.isTransactionStart && action == write && len > header.headerSizeBits / 8) {
                name = getRequestId(buffer[header.headerSizeBits / 8]);
            }

            DP_PRINTF(DP_NOTICE, "DP-AUX> %s%s%s%s%04Xh hint(to:%s %s%s %s #%d) { %s| %s}",
                      sizeRequested == *sizeCompleted ? "" : "INCOMPLETE ",
                      getStatus(result), getAction(action), getType(type), address,
                      header.address.toString(sb), header.isTransactionStart ? "S" : "",
                      header.isTransactionEnd ? "E" : "", name, header.messageNumber,
                      hex, hex_body);
#endif
            return result;
        }
    } else {
        hex[0] = 0;
    }

    dpHexDump(hex, sizeof(hex), buffer, *sizeCompleted);
    DP_PRINTF(DP_NOTICE, "DP-AUX> %s%s%s%s%04Xh { %s }",
              sizeRequested == *sizeCompleted ? "" : "INCOMPLETE ",
              getStatus(result), getAction(action), getType(type), address, hex);

    return result;
}

AuxBus* DisplayPort::CreateAuxLogger(AuxBus* auxBus) {
    return new AuxLogger(auxBus);
}
