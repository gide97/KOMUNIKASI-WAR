#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <pthread.h>
#include <thread>
#include <unistd.h>
#include <math.h>
#include "lib/TipeData.h"
#include "lib/KomunikasiArduino.h"
#include "lib/Citra.h"
#include "lib/Algoritma.h"
#include "lib/KomunikasiL.h"
#include "lib/KomunikasiS.h"
#include "lib/InputChar.h"

using namespace inputChar;
using namespace std;
using namespace cv;
using namespace R2CKomunikasiS;
using namespace R2CKomunikasiL;

//--------------CONSTANT VALUE
#define pi (float) 3.1428571428571428571

//--------------CITRA
int indexcambawah,indexcamomni;
VideoCapture cap, cap2;
Size size(640, 480);
Mat frame,jpg,frame_jpg,frame2;
Mat frame_lapangan1 = Mat::zeros(size,CV_8UC1);
Mat frame_garis1 = Mat::zeros(size,CV_8UC1);
Mat frame_bola1 = Mat::zeros(size,CV_8UC1);
Mat frame_gawang1 = Mat::zeros(size,CV_8UC1);
Mat frame_lapangan2 = Mat::zeros(size,CV_8UC1);
Mat frame_garis2 = Mat::zeros(size,CV_8UC1);
Mat frame_bola2 = Mat::zeros(size,CV_8UC1);
Mat frame_gawang2 = Mat::zeros(size,CV_8UC1);
Mat frame_musuh;

int koord_xbola = 320;
int koord_ybola = 240;
float sudut_bola = 0;
int koord_x, koord_y;


//-------------KOMUNIKASI ARDUINO
Serial arduino;
arduinoData mArduinoData;
unsigned char dataMasukArduino[100];
int lengthDataArduino=0;
pcData mPcData;


// --------------------
int heading = 0;
bool flag_bola;


// --------------- KOMUNIKASI ROBOT
#define dataBuff 256
char sendMessage[dataBuff],receivedMessage[dataBuff];
pcData.ID=1;//ROBOT 1

void BaseStationOder(char instruksi)
{
	mPcData.MOTION=1;
	mPcData.KECEPATAN=70;		
	if(instruksi=='w'){
		//printf("\tMaju\n");
		mPcData.HEADING=0;
	}
	else if(instruksi=='a'){
		//printf("\tKiri\n");
		mPcData.HEADING=-90;
	}
	else if(instruksi=='s'){
		//printf("\tMundur\n");
		mPcData.HEADING=180;
	}
	else if(instruksi=='d'){
		//printf("\tKanan\n");
		mPcData.HEADING=90;
	}
	//ALGORITMA ROBOT KE POSISI X Y YANG DIINGINKAN
	else if(instruksi=='p'){

	}
	else mPcData.MOTION=0;

}

//FUNGSI LISTENER UNTUK ROBOT
void thrd_Listen()
{
    R2CKomunikasiL::main();
    char data[dataBuff];

    int count1=0,count2=0,count3=0,count8=0;
    while(1){
        strcpy(receivedMessage,"");
        R2CKomunikasiL::listen(receivedMessage);
        //if(strcmp(data,receivedMessage)==0)continue;
        //strcpy(data,receivedMessage);

        /*
        ID: 1=ROBOT 1
            2=ROBOT 2
        	3=ROBOT 3
        	8=BASE STATION
        

        RUTINITAS BASE STATION MODE CONTROLLING ROBOT (Header "Aa")
        
        	Data yang dikirim adalah:
        	0   1        2             3		       4            5             6
        	Header - ID_pengirim - code_control - ID_penerima - koordinat_x - koordinat_y
        						    w-a-s-d-p
        */

        if(receivedMessage[0]=='A'&&receivedMessage[1]=='a'){//Aa -> BaseStation mode controling manual

			if(receivedMessage[2]=='1'){		// Pesan dari ID sendiri
				if(count1<50)printf("%s\n",receivedMessage);
				else {
					printf("No signal\n");
					count1=50;
				}
			}
	    	else if (receivedMessage[2]=='2'){	// Pesan dari ROBOT2
				printf("%s\n",receivedMessage);
				count1=0;
				count2=0;
	    	}


			else if (receivedMessage[2]=='8'){	// Pesan dari BaseStation
				printf("%s\n",receivedMessage);
				if(receivedMessage[3]!='0')BaseStationOder(receivedMessage[3]);
				count1=0;
				count8=0;
			}	    	
        }

        /* RUTINITAS BASE STaTION MODE BIASA
        	Data yang dikirim adalah:
        	0    1        2            3		  4  			5		   6
        	Header - ID_pengirim - refereeON - ROBOT1_ON - ROBOT2_ON - ROBOT3_ON
		*/
        else if(receivedMessage[0]=='A'&&receivedMessage[1]=='B'){//AB -> BaseStation mode biasa
        	if(receivedMessage[2]=='1'){		// Pesan dari ID sendiri
        		if(count1<50)printf("%s\n",receivedMessage);
        		else {
        			printf("No sigal\n");
        			count1=0;
        		}
        	}
        	else if (receivedMessage[2]=='2'){	// Pesan dari ROBOT2
        		printf("%s\n",receivedMessage);
        		count1=0;
        		count2=0;
        	}

        	else if (receivedMessage[2]=='8'){ // Pesan dari BaseStation
        		printf("%s\n",receivedMessage);
        		count1=0;
        		count2=0;
        	}
        }

    // INDIKATOR KONEKTIVITAS DEVICE
        count1++;
        //count2++;
        //count3++;
        count8++;
        // if(count1>50){//ROBOT 1 TERPUTUS
        // 	count1=50;
        // 	printf("Terputus dari ROBOT 1");
        }
        // else if(count2>50){//ROBOT 2 TERPUTUS
        // 	count2=50;
        // 	printf("Terputus dari ROBOT 2");
        // }
        // else if(count3>50){//ROBOT 3 TERPUTUS
        // 	count=50;
        // 	printf("Terputus dari ROBOT 3");
        // }
        if(count8>50){//BASE STATION TERPUTUS
        	count8=50;
        	printf("Terputus dari BaseStation");
        }
    }
    R2CKomunikasiL::closesocket();
}

/*

void thrd_Send(){
    //FORMAT DATA: HEADER IDKOMPUTER ISIPESAN
    R2CKomunikasiS::main();
    int Sflag=0,posManual=0,ID;
    int x=0;
    int y=0;
    while(1){    
		//JIKA ADA INPUT DARI KETBOARD BASE STATION
        if(kbhit()){
	    char button=getch();
        if(button=='a'||button=='s'||button=='d'||button=='w'){
            strcpy(sendMessage,"");
            strcpy(sendMessage,"Aa");
            sendMessage[2]='1';
            sendMessage[3]=button;
            sendMessage[4]=x;
            sendMessage[5]=y;
			sendMessage[6]='0';
       	    R2CKomunikasiS::send(sendMessage);
            Sflag=0;
            continue;
    	 }
        else if(button=='p'){
		   strcpy(sendMessage,"");
           strcpy(sendMessage,"Aa");
           printf("Masukkan koordinat (ID,x,y) ");
           scanf("%i,%i,%i",&ID,&x,&y);
           printf("koordinat: %i , %i ke ID: %i\n",x,y,ID); 
		   sendMessage[3]='0';
		   if(ID==1){x+='A';y+='A';}	//ditujukan pada robot 1
		   else if(ID==2){x+='a';y+='a';}//ditujukan pada robot 2
		   sendMessage[4]=x;
		   sendMessage[5]=y;
		   sendMessage[6]='1';	
		   R2CKomunikasiS::send(sendMessage);
		   posManual=0; 
           }
        }

		//RUTINITAS KIRIM DATA JIKA TIDAK ADA INPUT DARI KEYBOARD BASE STATION
		//Gak pakai usleep karena akan ada delay jika menginput keyboard
		Sflag++;
		if(Sflag==30000){
		    sendMessage[0]='A';
		    sendMessage[1]='a';
		    sendMessage[2]='1';
		    sendMessage[3]='0';
		    sendMessage[4]=x;
		    sendMessage[5]=y;
			if(posManual<3){sendMessage[6]='1';posManual++;}//'1' maksudnya sebuah perintah, line ini mengirim sendMessage 3x
			else {sendMessage[6]='0';}
		   	R2CKomunikasiS::send(sendMessage);   
			Sflag=0;
		}
    }
    
    R2CKomunikasiS::closesocket();
}

*/
















int main()
{
	 thread communicationL(thrd_Listen);
	 thread communicationS(thrd_Send);

	 communicationS.join();
	 communicationL.join();
}

