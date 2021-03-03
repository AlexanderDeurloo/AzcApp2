#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>

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
           printf("We've sent: %s   with size %i\n",Command, CommandSize); 
           return 0;
        }
        printf("fd wasn't okay\n");
        return -1;
}

int SerialOpen(char *PortName, int speed, int parity, int Blocking, int reset)
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
        if(reset)
        {
            int DtrFlag;
            DtrFlag = TIOCM_DTR;
            ioctl(fd,TIOCMBIS,&DtrFlag);
            sleep(2);
            ioctl(fd,TIOCMBIC,&DtrFlag);
            sleep(1);
        }
        return 0;
        
}

int SerialClose(void)
{
        if(fd) close(fd);
        return 0;
}



int SerialTxRx(char *portname, char *DataToSend, char *Result, size_t ResultLength, int Blocking)
{
        char Command[100];
        int receivedlength = 0;
        if(portname==NULL) return -1;
        if(DataToSend==NULL) return -2;
        if(Result==NULL) return -3;
        if ( (Blocking < 0) || (Blocking > 1) ) Blocking = 1;
        
        if (SerialOpen(portname,B4800,0,Blocking,0))
        {
           printf("Error opening serial\n");
           return -4;
        }
        sprintf(Command,"%s",DataToSend);
        SerialTx(Command,strlen(Command));
        usleep(100000);
        Command[0] = 0;
        receivedlength = SerialRx(Command,100);
        if(Result != NULL)
        {
           if(receivedlength > ResultLength) receivedlength = ResultLength;
           snprintf(Result,receivedlength,Command);
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

        if (SerialOpen(portname,B9600,0,0,0)) // no blocking
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

