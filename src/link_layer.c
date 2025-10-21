// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Frame Constants
#define FLAG 0x7E

#define A_CMD 0x03 // Commands sent by transmitter
#define A_RES 0x03 // Responses sent by receiver

#define C_SET 0x03  // Set up
#define C_UA 0x07   // Unnumbered Acknowledgment
#define C_RR1 0x05  // Receiver Ready (positive)
#define C_RR0 0x01  // Receiver Ready (negative)
#define C_REJ1 0x09 // Reject (positive)
#define C_REJ0 0x0D // Reject (negative)
#define C_DISC 0x0B // Disconnect

#define C_DATA0 0x00   // Control field for data frame (sequence 0)
#define C_DATA1 0x40   // Control field for data frame (sequence 1)

// BCC Calculation Macro
#define BCC(a, c) (a ^ c)

//Stuffing Constants
#define ESC 0x7D
#define FLAG_STUFFED 0x5E
#define ESC_STUFFED 0x5D

static int alarmOn = FALSE;
static int alarmCount = 0;

static unsigned char sequenceNr = 0;

static unsigned char byte;

static int timeout;
static int nRetransmissions;

void alarmHandler(int signal)
{
    alarmOn = FALSE;
    alarmCount++;
}

int createSetFrame(unsigned char *frame, unsigned char address, unsigned char control)
{
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = BCC(address, control);
    frame[4] = FLAG;

    return 5; // Frame size
}

unsigned char BCC2(const unsigned char *data, int dataSize)
{
    unsigned char result = 0x00;
    for (int i = 0; i < dataSize; i++)
    {
        result ^= data[i];
    }
    return result;
}

int createDataFrame(unsigned char *frame, const unsigned char *data, int dataSize, unsigned char sequenceNumber)
{

    int framei = 0;

    frame[framei++] = FLAG;
    frame[framei++] = A_CMD;
    frame[framei++] = sequenceNumber;
    frame[framei++] = BCC(A_CMD, sequenceNumber ? C_DATA1 : C_DATA0);

    // Copy data with stuffing
    for (int i = 0; i < dataSize; i++)
    {
        if (data[i] == FLAG)
        {
            frame[framei++] = ESC;
            frame[framei++] = FLAG_STUFFED;
        }
        else if (data[i] == ESC)
        {
            frame[framei++] = ESC;
            frame[framei++] = ESC_STUFFED;
        }
        else
            frame[framei++] = data[i];
    }

    // BCC2 with stuffing if needed
    unsigned char bcc2 = BCC2(data, dataSize);
    if (bcc2 == FLAG)
    {
        frame[framei++] = ESC;
        frame[framei++] = FLAG_STUFFED;
    }
    else if (bcc2 == ESC)
    {
        frame[framei++] = ESC;
        frame[framei++] = ESC_STUFFED;
    }
    else
        frame[framei++] = bcc2;

    // end flag
    frame[framei] = FLAG;

    return framei; // Frame size
}

// State Machine
typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

int isValidControlField(unsigned char c)
{
    return (c == C_SET || c == C_UA || c == C_DISC ||
            c == C_RR0 || c == C_RR1 || c == C_REJ0 || c == C_REJ1);
}

State state = START;
unsigned char recieved_A, recieved_C;

void state_machine(unsigned char byte)
{
    switch (state)
    {
    case START:
        if (byte == FLAG)
            state = FLAG_RCV;
        break;
    case FLAG_RCV:
        if (byte == A_CMD || byte == A_RES)
        {
            state = A_RCV;
            recieved_A = byte;
        }
        else if (byte != FLAG)
            state = FLAG;
        break;
    case A_RCV:
        if (isValidControlField(byte))
        {
            state = C_RCV;
            recieved_C = byte;
        }
        else if (byte == FLAG)
            state = FLAG_RCV;
        else
            state = START;
        break;
    case C_RCV:
        if (byte == BCC(recieved_A, recieved_C))
            state = BCC_OK;
        else if (byte == FLAG)
            state = FLAG_RCV;
        else
            state = START;
        break;
    case BCC_OK:
    if (byte == FLAG)
        state = STOP;
    else
        state = START;
    break;
    case STOP:
        break;
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    timeout = connectionParameters.timeout;
    nRetransmissions = connectionParameters.nRetransmissions;
    if (fd < 0)
        return -1;
    if (connectionParameters.role == LlTx)
    {
        // ALARM SETUP
        (void)signal(SIGALRM, alarmHandler); // Install alarm handler
        alarmOn = FALSE;
        alarmCount = 0;

        unsigned char frame[5];

        // SET FRAME CREATION
        createSetFrame(frame, A_CMD, C_SET);

        while (alarmCount < connectionParameters.nRetransmissions)
        {
            if (writeBytesSerialPort(frame, 5) != 5)
                return -1;

            alarm(connectionParameters.timeout);
            alarmOn = TRUE;

            // WAIT FOR UA
            state = START;
            while (alarmOn && state != STOP)
            {
                // read
                if (readByteSerialPort(&byte) == 1)
                {
                    state_machine(&byte);
                }
            }

            // If STOP is reached, UA is recieved
            if (state == STOP)
            {
                alarm(0);
                return 1; // SUCCESS
            }
        }
        return -1;
    }
    else
    {
        unsigned char frame[5];

        // Wait for SET frame
        state = START;
        while (state != STOP)
        {
            if (readByteSerialPort(&byte) == 1)
            {
                state_machine(byte);
            }
        }
        // Recieved valid SET FRAME
        // UA RESPONSE
        createSetFrame(frame, A_RES, C_UA);
        if (writeBytesSerialPort(frame, 5) != 5)
            return -1;

        return 1; // SUCCESS
    }
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char frame[2 * bufSize + 6];
    int frameSize = createDataFrame(frame, buf, bufSize, sequenceNr);

    alarmCount = 0;

    while (alarmCount < nRetransmissions)
    {
        // Send frame
        if (writeBytesSerialPort(frame, frameSize) != frameSize)
            return -1;

        // Start alarm
        alarm(timeout);
        alarmOn = TRUE;

        // Wait for response (RR or REJ)
        state = START;
        while (alarmOn && state != STOP)
        {
            if (readByteSerialPort(&byte) == 1)
            {
                state_machine(byte);

                if (state == STOP)
                {
                    if ((sequenceNr == 0 && recieved_C == C_RR1) ||
                        (sequenceNr == 1 && recieved_C == C_RR0))
                    {
                        sequenceNr = (sequenceNr + 1) % 2; 
                        alarm(0);
                        return frameSize;
                    }
                    else if ((sequenceNr == 0 && recieved_C == C_REJ0) ||
                             (sequenceNr == 1 && recieved_C == C_REJ1))
                    {
                        // Frame rejected, retry without increasing alarm count
                        alarmCount--;
                        break;
                    }
                }
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
