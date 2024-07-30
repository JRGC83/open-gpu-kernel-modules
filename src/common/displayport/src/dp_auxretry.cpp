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
    AuxRetry::status handleTransaction(AuxBus::status s, unsigned completed, unsigned size, unsigned &retries) {
        if ((s == AuxBus::success) && (completed == size) && (completed != 0)) {
            return AuxRetry::ack;
        } else {
            if (s == AuxBus::defer && retries > 0) {
                --retries;
                return AuxRetry::defer;
            }
            if (s == AuxBus::nack) {
                return AuxRetry::nack;
            }
            if (completed == 0) {
                return AuxRetry::unsupportedRegister;
            }
            if (completed < size && retries > 0) {
                --retries;
                return AuxRetry::defer;
            }
        }
        return AuxRetry::defer;
    }
}

AuxRetry::status AuxRetry::readTransaction(int address, NvU8 * buffer, unsigned size, unsigned retries) {
    unsigned completed;
    AuxBus::status s;
    DP_ASSERT(size <= aux->transactionSize());

    do {
        s = aux->transaction(AuxBus::read, AuxBus::native, address, buffer, size, &completed);
        AuxRetry::status status = handleTransaction(s, completed, size, retries);
        if (status != AuxRetry::defer) {
            return status;
        }
    } while(retries > 0);

    return AuxRetry::defer;
}

AuxRetry::status AuxRetry::writeTransaction(int address, NvU8 * buffer, unsigned size, unsigned retries) {
    unsigned completed;
    AuxBus::status s;
    DP_ASSERT(size <= aux->transactionSize());

    do {
        s = aux->transaction(AuxBus::write, AuxBus::native, address, buffer, size, &completed);
        AuxRetry::status status = handleTransaction(s, completed, size, retries);
        if (status != AuxRetry::defer) {
            return status;
        }
    } while(retries > 0);

    return AuxRetry::defer;
}

AuxRetry::status AuxRetry::read(int address, NvU8 * buffer, unsigned size, unsigned retries) {
    for (unsigned i = 0; i < size;) {
        int todo = DP_MIN(size - i, aux->transactionSize());
        status s = readTransaction(address + i, buffer + i, todo, retries);
        if (s != ack) {
            return s;
        }
        i += todo;
    }
    return ack;
}

AuxRetry::status AuxRetry::write(int address, NvU8 * buffer, unsigned size, unsigned retries) {
    for (unsigned i = 0; i < size;) {
        int todo = DP_MIN(size - i, aux->transactionSize());
        status s = writeTransaction(address + i, buffer + i, todo, retries);
        if (s != ack) {
            return s;
        }
        i += todo;
    }
    return ack;
}

AuxBus::status AuxLogger::transaction(Action action, Type type, int address, NvU8 * buffer, unsigned sizeRequested, unsigned * sizeCompleted, unsigned * pNakReason, NvU8 offset, NvU8 nWriteTransactions) {
    AuxBus::status result = bus->transaction(action, type, address, buffer, sizeRequested, sizeCompleted);
    hint[0] = 0;
    if (result == success) {
        if (type == native && (address == NV_DPCD_MBOX_DOWN_REQ || address == NV_DPCD_MBOX_UP_REP || address == NV_DPCD_MBOX_DOWN_REP || address == NV_DPCD_MBOX_UP_REQ)) {
            unsigned len = *sizeCompleted;
            Buffer storage(buffer, len);
            BitStreamReader reader(&storage, 0, len * 8);
            MessageHeader header;
            DisplayPort::decodeHeader(&reader, &header, Address(1));
            Address::StringBuffer sb;
            DP_USED(sb);
            dpHexDump(&hex[0], sizeof(hex), buffer, header.headerSizeBits / 8);
            dpHexDump(&hex_body[0], sizeof(hex), buffer + header.headerSizeBits / 8, len - header.headerSizeBits / 8);
#if defined(_DEBUG) || defined(DEBUG)
            const char * name = "";
            if (header.isTransactionStart && action == write && len > header.headerSizeBits / 8) {
                name = getRequestId(buffer[header.headerSizeBits / 8]);
            }
            DP_PRINTF(DP_NOTICE, "DP-AUX> %s%s%s%s%04Xh hint(to:%s %s%s %s #%d) { %s| %s}", sizeRequested == *sizeCompleted ? "" : "INCOMPLETE ", getStatus(result), getAction(action), getType(type), address, header.address.toString(sb), header.isTransactionStart ? "S" : "", header.isTransactionEnd ? "E" : "", name, header.messageNumber, hex, hex_body);
#endif
            return result;
        }
    } else {
        hex[0] = 0;
    }
    dpHexDump(&hex[0], sizeof(hex), buffer, *sizeCompleted);
    DP_PRINTF(DP_NOTICE, "DP-AUX> %s%s%s%s%04Xh { %s }", sizeRequested == *sizeCompleted ? "" : "INCOMPLETE ", getStatus(result), getAction(action), getType(type), address, hex);
    return result;
}

AuxBus * DisplayPort::CreateAuxLogger(AuxBus * auxBus) {
    return new AuxLogger(auxBus);
}
