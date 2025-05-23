// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // for exit function

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


class Yoon_UART
{
    private:
        struct termios newtio;
        int ttyfd;

    public:

        Yoon_UART(char *ttyname,int Baudrate);
        // bool YUART_start(unsigned char* buffer_out);
        bool YUART_start(char* buffer_out);
        void YUART_terminate();

};