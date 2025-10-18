// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

//Frame Constants
#define FLAG 0x7E

#define A_CMD 0x03   // Commands sent by transmitter
#define A_RES 0x03   // Responses sent by receiver

#define C_SET 0x03  // Set up
#define C_UA 0x07    // Unnumbered Acknowledgment
#define C_RR1 0x05  // Receiver Ready (positive)
#define C_RR0 0x01  // Receiver Ready (negative)
#define C_REJ1 0x09 // Reject (positive)
#define C_REJ0 0x0D // Reject (negative)
#define C_DISC 0x0B  // Disconnect

#define BCC(a, c) (a ^ c)

static int alarmOn = FALSE;
static int alarmCount = 0;

void alarmHandler(int signal){
    alarmOn = FALSE;
    alarmCount++;
}

int createFrame(unsigned char *frame, unsigned char address, unsigned char control)
{
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = BCC(address, control);
    frame[4] = FLAG;

    return 5; // Frame size
}

//State Machine
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
} State;

State state = START;
unsigned char recieved_A, recieved_C;

void state_machine(unsigned char byte){
    switch(state){
        case START:
        if(byte == FLAG)
        state = FLAG_RCV;
        break;
        case FLAG_RCV:
        if(byte == A_CMD || byte == A_RES){
            state = A_RCV;
            recieved_A = byte;
        }
        else if(byte != FLAG)
        state = FLAG;
        break;
        case A_RCV:
        if(byte == C_SET || byte == C_UA || byte == C_DISC) {
            state = C_RCV;
            recieved_C = byte;
        }
        else if(byte == FLAG)
        state = FLAG_RCV;
        else
        state = START;
        break;
        case C_RCV:
        if(byte == BCC(recieved_A, recieved_C))
        state = BCC_OK;
        else if (byte == FLAG)
        state = FLAG_RCV;
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
    if (fd < 0) return -1;
    if (connectionParameters.role == LlTx){
        //ALARM SETUP
        (void)signal(SIGALRM, alarmHandler); // Install alarm handler
        alarmOn = FALSE;
        alarmCount = 0;

        unsigned char frame[5];
        unsigned char byte;

        //SET FRAME CREATION
        createFrame(frame, A_CMD, C_SET);

        while(alarmCount < connectionParameters.nRetransmissions){
            if(writeBytesSerialPort(frame, 5) != 5) return -1;

            alarm(connectionParameters.timeout);
            alarmOn = TRUE;

            //WAIT FOR UA
            state = START;
            while (alarmOn && state != STOP){
                //read
                if (readByteSerialPort(&byte) == 1){
                    state_machine(&byte);
                }
            }

            //If STOP is reached, UA is recieved
            if(state == STOP){
                alarm(0);
                return 1; //SUCCESS
            }


        }
        return -1;
    }
    else {
        unsigned char byte;
        unsigned char frame[5];

        //Wait for SET frame
        state = START;
        while(state != STOP){
            if(readByteSerialPort(&byte) == 1){
                state_machine(byte);
            }
        }
        //Recieved valid SET FRAME
        //UA RESPONSE
        createFrame(frame, A_RES, C_UA);
        if (writeBytesSerialPort(frame, 5) != 5) return -1;

        return 1; //SUCCESS

    }
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

    return 0;
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
