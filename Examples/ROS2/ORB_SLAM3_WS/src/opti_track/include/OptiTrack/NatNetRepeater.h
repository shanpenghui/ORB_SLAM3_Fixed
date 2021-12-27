//=============================================================================----
// Copyright © 2016 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================----

#pragma once

//== INCLUDES =================================================================----

#include <winsock.h>

//== GLOBAL DEFINITIONS AND SETTINGS ==========================================----

namespace
{
    const int kMaxAddressLength = 128;
    const int kSubPacketMaxSize = 1400;
}

//== CLASS DEFINITION =========================================================----

class NATNET_API cSlipStream
{
public:
    cSlipStream( const char *Address, int Port );
    ~cSlipStream();

    ///<summary>Output a block of data over UDP.</summary>
    bool Stream( unsigned char* Buffer, int BufferSize );

private:
    char   mAddress[kMaxAddressLength];
    int    mPort;

    SOCKET mSocket;

    bool   StreamPacket( unsigned char *Buffer, int BufferSize );
};
