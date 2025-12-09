// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************

#include "ReactionWheel.h"

//------------------------------------------------------------
// Simple TCP socket communication : 1 server / 1 client
//------------------------------------------------------------

#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/param.h>
#include <netinet/in.h>

struct message_t {
    char dataType;
    char messageID;
    unsigned short length;
    unsigned char data[MAX_DATA_SIZE];
};

static struct message_t ReceivedMsg;
static struct message_t MsgToSend;

static int serverSocket;
static int clientSocket;

static struct sockaddr_in serverAddr; // server's socket address  
static struct sockaddr_in clientAddr; // client's socket address  
static socklen_t clientAddrSize; // size of client address structure  

/** Open a TCP socket for listening
 * @param portnum : listening port number
 * @Note Terminate the current process on error.
 */
void OpenSocketServer(int portnum)
{
    serverSocket = socket(AF_INET, SOCK_STREAM, 0); // create a socket
    if(serverSocket < 0) {
        rt_printf("  ERROR %s : %s\r\n", "opening socket", strerror(errno));
        exit(EXIT_FAILURE);
    }

    // local address initialization  
    bzero(&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(portnum);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    int errcode = bind(serverSocket, &serverAddr, sizeof(serverAddr));
    if(errcode < 0) {
        rt_printf("  ERROR %s : %s\r\n", "binding socket", strerror(errno));
        exit(EXIT_FAILURE);
    }
}

/** Waits for incomming connection from client
 * @return true if connected, false on error
 * @Note This is a blocking call
 */
bool WaitForConnection(void)
{
    int errcode = listen(serverSocket, 1);
    if(errcode < 0) {
        rt_printf("  ERROR %s : %s\r\n", "listening", strerror(errno));
        return(false);
    }

    clientSocket = accept(serverSocket, &clientAddr, &clientAddrSize);
    if(clientSocket < 0) {
        rt_printf("  ERROR %s : %s\r\n", "accepting connection", strerror(errno));
        return(false);
    }
    return(true);
}

/** Close the connected socket
 */
void Disconnect(void)
{
    if(clientSocket > 0) close(clientSocket);
    clientSocket = -1;
}

/** Close the server socket
 */
void CloseSocket(void)
{
    if(serverSocket > 0) close(serverSocket);
    serverSocket = -1;
}

/** Write the stored message to the socket \n
 * Reorders bytes if needed.
 * @Note Only used localy
 */
static void SendMessage(void)
{
    int n;
    int packetSize = PACKET_HEADER_SIZE + MsgToSend.length;
    // bytes reordering host => network
    uint16_t * ptr16b = (void*) MsgToSend.data;
    uint32_t * ptr32b = (void*) MsgToSend.data;
    switch(MsgToSend.dataType) {
    default:
    case EMPTY_TYPE:
    case STRING_TYPE:
        break;
    case INTEGER_TYPE:
        ptr16b[0] = htons(ptr16b[0]);
        break;
    case REAL_TYPE:
        ptr32b[0] = htonl(ptr32b[0]);
        break;
    case INTEGER_ARRAY_TYPE:
        for(n = 0; n < MsgToSend.length / sizeof(short); n++) ptr16b[n] = htons(ptr16b[n]);
        break;
    case REAL_ARRAY_TYPE:
        for(n = 0; n < MsgToSend.length / sizeof(float); n++) ptr32b[n] = htonl(ptr32b[n]);
        break;
    }
    MsgToSend.length = htons(MsgToSend.length);

    int errcode = send(clientSocket, &MsgToSend, packetSize, 0);
    if(errcode < 0) {
        rt_printf("  ERROR %s : %s\r\n", "sending packet", strerror(errno));
    }
}

/** Waits for a message from client and store it.\n
 * @Note This is a blocking call.
 */
void WaitForMessage(void)
{
    int n;
    int packetSize = recv(clientSocket, &ReceivedMsg, MAX_PACKET_SIZE, 0);
    if(packetSize <= 0) {
        ReceivedMsg.dataType = ERROR_TYPE;
        ReceivedMsg.messageID = ERROR_TYPE;
        ReceivedMsg.length = 0;
        return;
    }
    uint16_t * ptr16b = (void*) ReceivedMsg.data;
    uint32_t * ptr32b = (void*) ReceivedMsg.data;

    // bytes reordering network => host
    ReceivedMsg.length = ntohs(ReceivedMsg.length);
    switch(ReceivedMsg.dataType) {
    default:
    case EMPTY_TYPE:
    case STRING_TYPE:
        break;
    case INTEGER_TYPE:
        ptr16b[0] = ntohs(ptr16b[0]);
        break;
    case REAL_TYPE:
        ptr32b[0] = ntohl(ptr32b[0]);
        break;
    case INTEGER_ARRAY_TYPE:
        for(n = 0; n < ReceivedMsg.length / sizeof(short); n++) ptr16b[n] = ntohs(ptr16b[n]);
        break;
    case REAL_ARRAY_TYPE:
        for(n = 0; n < ReceivedMsg.length / sizeof(float); n++) ptr32b[n] = ntohl(ptr32b[n]);
        break;
    }
}

/** Write a message with EMPTY_TYPE data to the socket
 * @param response : response to send.
 */
void WriteResponse(char response)
{
    MsgToSend.messageID = response;
    MsgToSend.dataType = EMPTY_TYPE;
    MsgToSend.length = 0;
    SendMessage();
}

/** Write a message with STRING_TYPE data to the socket
 * @param response : response to send.
 * @param string : zero-terminated text to send.
 */
void WriteString(char response, char *string)
{
    MsgToSend.messageID = response;
    MsgToSend.dataType = STRING_TYPE;
    MsgToSend.length = MIN(strlen(string), MAX_DATA_SIZE - 1);
    memcpy(MsgToSend.data, string, MsgToSend.length);
    SendMessage();
}

/** Write a message with INTEGER_TYPE data to the socket
 * @param response : response to send.
 * @param integer : value to send.
 */
void WriteInteger(char response, short integer)
{
    MsgToSend.messageID = response;
    MsgToSend.dataType = INTEGER_TYPE;
    MsgToSend.length = sizeof(short);
    memcpy(MsgToSend.data, &integer, MsgToSend.length);
    SendMessage();
}

/** Write a message with INTEGER_ARRAY_TYPE data to the socket
 * @param response : response to send.
 * @param array : values to send.
 * @param Nbelements : number of values to send.
 */
void WriteIntegerArray(char response, const short array[], short Nbelements)
{
    MsgToSend.messageID = response;
    MsgToSend.dataType = INTEGER_ARRAY_TYPE;
    MsgToSend.length = (Nbelements * sizeof(short));
    memcpy(MsgToSend.data, array, MsgToSend.length);
    SendMessage();
}

/** Write a message with REAL_TYPE data to the socket
 * @param response : response to send.
 * @param real : value to send.
 */
void WriteReal(char response, float real)
{
    MsgToSend.messageID = response;
    MsgToSend.dataType = REAL_TYPE;
    MsgToSend.length = sizeof(float);
    memcpy(MsgToSend.data, &real, MsgToSend.length);
    SendMessage();
}

/** Write a message with INTEGER_ARRAY_TYPE data to the socket
 * @param response : response to send.
 * @param array : values to send.
 * @param Nbelements : number of values to send.
 */
void WriteRealArray(char response, const float array[], short Nbelements)
{
    MsgToSend.messageID = response;
    MsgToSend.dataType = REAL_ARRAY_TYPE;
    MsgToSend.length = (Nbelements * sizeof(float));
    memcpy(MsgToSend.data, array, MsgToSend.length);
    SendMessage();
}

/** Use with any data type
 * @return the request field of the message
 */
char ReturnRequest(void)
{
    return ReceivedMsg.messageID;
}

/** 
 * @return The type of data received.
 */
char ReturnDataType(void)
{
    return ReceivedMsg.dataType;
}

/** Use when data type == STRING_TYPE
 * @param string : array that receive values.
 * @return number of char received.
 */
int ReturnReceivedString(char *string)
{
    memcpy(string, ReceivedMsg.data, ReceivedMsg.length);
    ReceivedMsg.data[ReceivedMsg.length] = '\0';
    return ReceivedMsg.length;
}

/** Use when data type == INTEGER_TYPE
 * @return received value
 */
short ReturnReceivedInteger(void)
{
    return *((short*) ReceivedMsg.data);
}

/** Use when data type == INTEGER_ARRAY_TYPE
 * @param array : array that receive values.
 * @return number of integer received.
 */
int ReturnReceivedIntegerArray(short int array[])
{
    memcpy(array, ReceivedMsg.data, ReceivedMsg.length);
    return ReceivedMsg.length / sizeof(short);
}

/** Use when data type == REAL_TYPE
 * @return received value
 */
float ReturnReceivedReal(void)
{
    return *((float*) ReceivedMsg.data);
}

/** Use when data type == REAL_ARRAY_TYPE
 * @param array : array that recieve values.
 * @return number of real received.
 */
int ReturnReceivedRealArray(float array[])
{
    memcpy(array, ReceivedMsg.data, ReceivedMsg.length);
    return ReceivedMsg.length / sizeof(float);
}
