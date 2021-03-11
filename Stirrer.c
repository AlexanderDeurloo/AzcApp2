#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdint.h>

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
        printf("Lets read data\n");

        bytes = read (fd, Result, ResultSize);
        Result[bytes] = 0; // terminate
        printf("Received %i chars: >%s<\n",bytes,Result);
        return bytes;
}


int SerialOpen(char *PortName, int speed, int parity, int Blocking, int reset)
{
        if (PortName == NULL) return -1;
        printf("Lets open the port\n");
        fd = open (PortName, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
                printf ("error %d opening %s: %s", errno, PortName, strerror (errno));
                return -1;
        }

        printf("Lets set the attribs\n");
        set_interface_attribs (fd, speed, parity);  // set speed to 115,200 bps, 8n1 (no parity)
        printf("Lets set blocking\n");
        set_blocking (fd, Blocking);                // set blocking
        if(reset)
        {
            int DtrFlag;
            DtrFlag = TIOCM_DTR;
            ioctl(fd,TIOCMBIS,&DtrFlag);
            sleep(1);
            ioctl(fd,TIOCMBIC,&DtrFlag);
        }
        return 0;
        
}

int SerialClose(void)
{
        if(fd) close(fd);
        return 0;
}




int SerialTxSlow(char *Command, size_t CommandSize)
{
        if(Command == NULL) return -1;
        if (CommandSize == 0) return -2;
        if(fd)
        {
            printf("Lets write data\n");

            int i;
            for (i=0;i<CommandSize;i++)
            {
                write (fd, &Command[i], 1); 
                usleep(50000);
            }
           //printf("We've sent: %s   with size %i\n",Command, CommandSize); 
           return 0;
        }
        printf("fd wasn't okay\n");
        return -1;
}


void StirrerRpmCommand(uint16_t RpmSetting, uint8_t *Command)
{
    unsigned char checksum;
    Command[0] = 0xFE;
    Command[1] = 0xB1;
    Command[2] = (RpmSetting >> 8) & 0xFF;
    Command[3] = RpmSetting & 0xFF;
    Command[4] = 0x00;
    checksum = (Command[1] + Command[2] + Command[3] + Command[4]) & 0xFF;
    Command[5] = checksum;
    return;
}

void PrintRx(char* Buff, int length)
{
    int i;
    if(Buff == NULL) return;
    for (i=0;i<length;i++)
    {
        printf("Buff[%i] = 0x%02x\n",i,Buff[i]);
    }
    return;
}

void main(void)
{
    int ptr;
    ptr = open("/dev/ttyUSB2",O_RDWR);
    if(ptr < 0)
    {
        printf("err open\n");
    }
    write(ptr,"abc",3);
    close(ptr);
}
        
void Oldmain(void)
{
    
    printf("Lets call SerialOpen\n");
    SerialOpen("/dev/ttyUSB2", B9600, 0, 1, 0);
        
    int receivedlength = 0;
    uint8_t StirrerData[10];
    uint8_t Command[100];
    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xA0; //hello
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xA0;

    printf("Lets Tx first data\n");
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("hello = %i\n",receivedlength);
    PrintRx(Command,receivedlength);
    

    sleep(1);
    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xA1; //get status
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xA1;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("status = %i\n",receivedlength);
    PrintRx(Command,receivedlength);
    if(Command[3] != 0x00) // if stirrer is not closed
    {
        sleep(1);
        printf("Stirrer not is closed. Send 0 RPM\n");
        StirrerRpmCommand(0,StirrerData);
        SerialTxSlow(StirrerData,6);
        sleep(1);
        receivedlength = SerialRx(Command,100);
        printf("0 rpm = %i\n",receivedlength);
        PrintRx(Command,receivedlength);

        sleep(1);
        StirrerData[0] = 0xFE;
        StirrerData[1] = 0xA1; //get status
        StirrerData[2] = 0x00;
        StirrerData[3] = 0x00;
        StirrerData[4] = 0x00;
        StirrerData[5] = 0xA1;
        SerialTxSlow(StirrerData,6);
        sleep(1);
        receivedlength = SerialRx(Command,100);
        printf("status = %i\n",receivedlength);
        PrintRx(Command,receivedlength);

    }


    sleep(1);
    StirrerRpmCommand(400,StirrerData);
    SerialTxSlow(StirrerData,6);
    receivedlength = SerialRx(Command,100);
    printf("400 rpm = %i\n",receivedlength);
    PrintRx(Command,receivedlength);

    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xA1; //get status
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xA1;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("status = %i\n",receivedlength);
    PrintRx(Command,receivedlength);

    sleep(4);   // allow speed to increase

    sleep(1);
       StirrerRpmCommand(0,StirrerData);
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("0 rpm = %i\n",receivedlength);
    PrintRx(Command,receivedlength);

    StirrerRpmCommand(1000,StirrerData);
    SerialTxSlow(StirrerData,6);
    receivedlength = SerialRx(Command,100);
    printf("1000 rpm = %i\n",receivedlength);
    PrintRx(Command,receivedlength);

    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xA1; //get status
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xA1;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("status = %i\n",receivedlength);
    PrintRx(Command,receivedlength);

    sleep(10);   // allow speed to increase


    sleep(1);
    StirrerRpmCommand(0,StirrerData);
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("0 rpm = %i\n",receivedlength);
    PrintRx(Command,receivedlength);

    sleep(1);
    StirrerData[0] = 0xFE;
    StirrerData[1] = 0xA1; //get status
    StirrerData[2] = 0x00;
    StirrerData[3] = 0x00;
    StirrerData[4] = 0x00;
    StirrerData[5] = 0xA1;
    SerialTxSlow(StirrerData,6);
    sleep(1);
    receivedlength = SerialRx(Command,100);
    printf("status = %i\n",receivedlength);
    PrintRx(Command,receivedlength);


    
//    SerialTxSlow("abc", 3);
    
    SerialClose();
    
}
