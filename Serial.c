#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

int fd = 0;



int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


int SerialRx(char *Result, size_t ResultSize)
{
        int bytes;
        if (Result == NULL) return -1;
        if (ResultSize == 0) return -2;
        if (fd == 0) return -3;
        
        bytes = read (fd, Result, ResultSize);
        Result[bytes] = 0; // terminate
        //printf("Received %i chars: >%s<\n",bytes,Result);
        return bytes;
}

int SerialTx(char *Command, size_t CommandSize)
{
        if(Command == NULL) return -1;
        if (CommandSize == 0) return -2;
        if(fd)
        {
        
           write (fd, Command, CommandSize); 
           printf("We've sent: %s with size %i\n",Command, CommandSize); 
           return 0;
        }
        printf("fd wasn't okay\n");
        return -1;
}

int SerialOpen(char *PortName, int speed, int parity, int Blocking)
{
        if (PortName == NULL) return -1;
        fd = open (PortName, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
                printf ("error %d opening %s: %s", errno, PortName, strerror (errno));
                return -1;
        }

        set_interface_attribs (fd, speed, parity);  // set speed to 115,200 bps, 8n1 (no parity)
        set_blocking (fd, Blocking);                // set blocking
 
        return 0;
        
}

int SerialClose(void)
{
        if(fd) close(fd);
        return 0;
}



int SerialTxRx(char *portname, char *DataToSend, char *Result, size_t ResultLength)
{
        char Command[100];
        int receivedlength = 0;
        if(portname==NULL) return -1;
        if(DataToSend==NULL) return -2;
        if(Result==NULL) return -3;

        if (SerialOpen(portname,B4800,0,1))
        {
           printf("Error opening serial\n");
           return -4;
        }
        /*
        if(argc)
        {
                if(argv==NULL)
                {
                        printf("Whoops. no valid string\n");
                        return -4;
                }
                //printf("Argc found\n");
                sprintf(Command,"%s\n",argv);
        }
        else
        {
                //printf("no argc found\n");
                sprintf(Command,"waterlevel?\n");
        }
        */
        sprintf(Command,"%s",DataToSend);
        //printf("Send: >%s<\n",Command);
        SerialTx(Command,strlen(Command));
        usleep(100000);
        Command[0] = 0;
        receivedlength = SerialRx(Command,100);
        if(Result != NULL)
        {
           if(receivedlength > ResultLength) receivedlength = ResultLength;
           snprintf(Result,receivedlength,Command);
           //Result[receivedlength] = 0;
        }
        SerialClose();
        return receivedlength;
}


int SerialTxStirrer(char *portname, unsigned char *DataToSend, size_t Bytes)
{
        char Command[100];
        int receivedlength = 0;
        if(portname==NULL) return -1;
        if(DataToSend==NULL) return -2;

        if (SerialOpen(portname,B9600,0,0)) // no blocking
        {
           printf("Error opening serial\n");
           return -3;
        }
        int i;
        for (i=0;i<Bytes;i++) Command[i] = DataToSend[i];
        SerialTx(Command,Bytes);
        SerialClose();
        return 0;

}

void printdata(char* data, size_t length)
{
    int i;
    for(i=0;i<length;i++)
        printf("%i = 0x%02x\n",i,data[i]);
}

int SerialTxSlow(char *Command, size_t CommandSize)
{
        if(Command == NULL) return -1;
        if (CommandSize == 0) return -2;
        int i = 0;
        int byte;
        if(fd)
        {
            for (i=0;i<CommandSize;i++)
            {
                byte = Command[i];
               write (fd,(void*) byte, 1);
               printf("%i = 0x%02x\n",i,byte); 
               //usleep(5000);
            }
           //printf("We've sent: %s with size %i\n",Command, CommandSize); 
           return 0;
        }
        printf("fd wasn't okay\n");
        return -1;
}


int main(void)
{
    char *portname = "/dev/ttyUSB2";
    char StirrerData[10];
    int receivedlength = 0;
    unsigned char checksum;
    unsigned int RpmSetting;
    if (SerialOpen(portname,B9600,0,0)) // no blocking
    {
       printf("Error opening serial\n");
       return -3;
    }
    int i;
    // get status
/*    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xA1; //get status
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xA1;
    StirrerData[6] = 0x00;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    */
    // 0 rpm
    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xB1;
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xB1;
    StirrerData[6] = 0x00;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    
    RpmSetting = 100;
    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xB1;
    StirrerData[2] = (RpmSetting >> 8) & 0xFF;
    StirrerData[3] = RpmSetting & 0xFF;
    StirrerData[4] = 0x00;
    checksum = (StirrerData[1] + StirrerData[2] + StirrerData[3] + StirrerData[4]) & 0xFF;
    StirrerData[5] = checksum;
    StirrerData[6] = 0x00;
    SerialTxSlow(StirrerData,6);
    
// mirror all bytes:
/*
    StirrerData[0] = 0x7F;
    StirrerData[1] = 0x83;
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0x83;
    StirrerData[6] = 0x00;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    
    RpmSetting = 100;
    StirrerData[0] = 0x7F;
    StirrerData[1] = 0x83;
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x26;
    StirrerData[4] = 0x00;
    checksum = (StirrerData[1] + StirrerData[2] + StirrerData[3] + StirrerData[4]) & 0xFF;
    StirrerData[5] = 0x9B;
    StirrerData[6] = 0x00;
    SerialTxSlow(StirrerData,7);
    */
    SerialClose();
    return 0;
}






