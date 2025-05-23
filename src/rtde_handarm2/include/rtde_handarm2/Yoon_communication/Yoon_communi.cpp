#include "Yoon_communi.h"

Yoon_UART::Yoon_UART(char *ttyname,int Baudrate)
{

 	// char *ttyname = "/dev/ttyUSB0";
    // char *ttyname = "/dev/cu.usbserial-1440";
	this->ttyfd = open(ttyname, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(this->ttyfd < 0)
	{
		printf( ">> tty Open Fail [%s]\r\n ", ttyname);
        close(this->ttyfd); //close serial port
        exit(1);
	}
	memset( &this->newtio, 0, sizeof(this->newtio) );
	
    if(Baudrate == 115200) newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD | CRTSCTS;   
    else if (Baudrate == 9600) this->newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD | CRTSCTS;
    else
    {
        printf( ">> Baudrate: %d does not supported \r\n ", Baudrate);
        close(this->ttyfd); //close serial port
        exit(1);
    }
	this->newtio.c_iflag = IGNPAR;
	this->newtio.c_oflag = 0;

	//set input mode (non-canonical, no echo,.....)
	this->newtio.c_lflag     = 0;     // LF recive filter unused
	this->newtio.c_cc[VTIME] = 0;     // inter charater timer unused
	this->newtio.c_cc[VMIN]  = 0;     // blocking read until 1 character arrives

	tcflush( this->ttyfd, TCIFLUSH ); // inital serial port
	tcsetattr( this->ttyfd, TCSANOW, &this->newtio ); // setting serial communication
	printf( "## ttyo1 Opened [%s]\r\n", ttyname);
	
}

// bool Yoon_UART::YUART_start(unsigned char* buffer_out)
bool Yoon_UART::YUART_start(char* buffer_out)
{
    // array 계열은 받아오는 변수로 만들때는 함수에 포인터로 선언
    // 그리고 함수에 넣을때는 크기가 명시된 포인터 array가 아닌 일반 array명을 그냥 대입
    // 데이터 복사는 memcpy로 하되 복사해서 대입하는 array의 크기가 복사 당하는 array보다 커야됨 !! 
    // unsigned char buffer[100] = {0};
    char buffer[100] = {0};
    int ret = read(this->ttyfd, buffer, sizeof(buffer));
    if (ret > 0) 
    {
        memcpy(buffer_out,buffer, sizeof(buffer));
        // printf("ret: %d , %s \n",ret,buffer_out);
        // printf("ret: %d \n",ret);
        return true;
    } 
    else 
    {
        //When no data is received, sleep for 50ms to prevent excessive consumption of cpu
        usleep(1000 * 50);
        return false;
    }
    
}

void Yoon_UART::YUART_terminate()
{
    printf("Uart port was terminated \n");
    close(this->ttyfd); //close serial port
}