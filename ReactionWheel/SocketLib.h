// ***********************************************************
// *                I S A E - S U P A E R O                  *
// *                                                         *
// *               Reaction Wheel Application                *
// *                      Version 2021                       *
// *                                                         *
// ***********************************************************
#ifndef SOCKET_LIB_H
#define SOCKET_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

// Max size of a TCP packet in bytes
#define MAX_PACKET_SIZE 1024
// Size of packet header
#define PACKET_HEADER_SIZE 4
// Max size of data field
#define MAX_DATA_SIZE (MAX_PACKET_SIZE - PACKET_HEADER_SIZE)
// Max number of reals in a  message
#define MAX_REAL_ARRAY_SIZE  (MAX_DATA_SIZE/sizeof(float))
// Max number of integers in a  message
#define MAX_INTEGER_ARRAY_SIZE (MAX_DATA_SIZE/sizeof(short))

// Packets data types declaration
#define ERROR_TYPE          'X'    // Bad message
#define EMPTY_TYPE          'C'    // Request/Response + no data
#define INTEGER_TYPE        'I'    // Request/Response + 1 Short integer
#define REAL_TYPE           'R'    // Request/Response + 1 Single precision real
#define STRING_TYPE         'S'    // Request/Response + String
#define INTEGER_ARRAY_TYPE  'A'    // Request/Response + Short integers array
#define REAL_ARRAY_TYPE     'T'    // Request/Response + Single precision reals array

void OpenSocketServer(int portnum);
bool WaitForConnection(void);
void WaitForMessage(void);
void Disconnect(void);
void CloseSocket(void);

char  ReturnRequest(void);
char  ReturnDataType(void);

int   ReturnReceivedString(char *string);
short ReturnReceivedInteger(void);
int   ReturnReceivedIntegerArray(short int array[]);
float ReturnReceivedReal(void);
int   ReturnReceivedRealArray(float array[]);

void WriteResponse(char response);
void WriteString(char response, char *string);
void WriteInteger(char response, short integer);
void WriteIntegerArray(char response, const short array[], short Nbelements);
void WriteReal(char response, float real);
void WriteRealArray(char response, const float array[], short Nbelements);

#ifdef __cplusplus
}
#endif

#endif
