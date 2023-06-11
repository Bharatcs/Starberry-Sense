#define pixel_no_threshold 10
#define column_t 1640
#define row_t  1232
#define column_transmit 1232/2
#define row_transmit 164/2
#define pii 3.14159265358
#define fov 9
#define number_of_stars_in_catalog 8874
#define EXPOSURE_TIME_IN_MS 5000
#define COMPRESSION_FACTOR 50
#define ERROR_IN_ARC_SEC 20
#define TEST_MODE 1
#define SPI_CHANNEL 0
#define SPI_CLOCK_SPEED 1000000
#define BUSY 30
#define CLEAR 21
#include <iostream>
#include <wiringPiSPI.h>
#include <bitset>
#include<iostream>
#include <cstdlib>
#include<stdint.h>
#include<stdlib.h>
#include"basic_functions.hpp"
#include"unit_vector/unit_vector_table.hpp"
#include"hash_table/angle_table.hpp"
#include"hash_table/angle_id.hpp"
#include"hash_table/hash.hpp"
#include"matrix.hpp"
#include <raspicam/raspicam_cv.h>
#include <cstdlib>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <exception>
#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <wiringPi.h>
#include<stdbool.h>
#include <pthread.h>
// Author: Bharat Chandra and Mayuresh Sarportdar
// Star Tracker Algorithm
//To compile use makefile
//Structs

struct image
  {
    float x_cen[500];
    float y_cen[500];
    float pix[500];
    int stars;
 };


struct stars_found
  {
    float cen[2];
    float unit_vector[3];
    int primary_vote;
    int secondary_votes;
    int vote_list[number_of_stars_in_catalog + 1];
    int total_vote;
    int star_id;
  };


  struct quaternion
  {
    float q1;
    float q2;
    float q3;
    float q4;
  };


  struct pointing
  {
    float pitch;
    float yaw;
    float roll;
  };
int serial;
int jpeg_size;
unsigned char image_jpeg[(row_t*column_t)+100];

int image[row_t][column_t];
int RG_Matrix[row_t][column_t];

unsigned char kol= -1;
int frames = 0;
uint16_t FRAME_NUMBER;

//Circular QUEUE
typedef struct 
{   uint8_t* array;
    unsigned long int head, tail,num_entries, size; 
} queue;
// Initialise queue
void init_queue(queue *q,unsigned long int max_size){
    q->size = max_size;
    q->array =static_cast<uint8_t *> (malloc(sizeof(uint8_t) * q->size));
    q->num_entries = 0;
    q->head = 0;
    q->tail = 0;
}
// queue empty
bool queue_empty(queue* q){
     return (q->num_entries == 0);
}
// queue full
bool queue_full(queue* q){

        return (q->num_entries == q->size);
}
//enqueue function
bool enqueue(queue* q, uint8_t item)
{
     if(queue_full(q))  { 
         return false;
     }
     q->array[q->tail]= item;
     q->num_entries++;
     q->tail = (q->tail + 1) % q->size;
     return true;
}
//dequeue function
bool dequeue(queue* q,uint8_t* item_return)
{
  if(queue_empty(q))    return false;
    
    *item_return = q->array[q->head];
    q->head=(q->head + 1) % q->size;
    q->num_entries--;
    return true;
}
unsigned long int num_enteries(queue* q)
{
  return q->num_entries;
}
//function to print bytes as binary
void print_binary(uint8_t byte)
{
  for (int i = 7; i >= 0; i--)
  {
    if (byte & (1 << i))
      std::cout << "1";
    else
      std::cout << "0";
  }
  std::cout << std::endl;
}
//get cpu temperature return int celsius
int get_cpu_temp()
{
  FILE *fp;
  char path[1035];
  fp = popen("/opt/vc/bin/vcgencmd measure_temp", "r");
  if (fp == NULL)
  {
    printf("Failed to run command\n" );
    return 0;
  }
  while (fgets(path, sizeof(path)-1, fp) != NULL)
  {
    //printf("%s", path);
  }
  pclose(fp);
  int temp = atoi(&path[5]);
  return temp;
}
// pthread for sharing data between threads
queue q;
pthread_mutex_t lock= PTHREAD_MUTEX_INITIALIZER;

void* data_transmit_spi(void* arg){

int length=0;
unsigned char buf[1800];
unsigned char buf1[1800];
int bytes_send=0;
FILE *fp;
 int fd = wiringPiSPISetupMode(SPI_CHANNEL, SPI_CLOCK_SPEED, 0);
    if (fd == -1) {
        std::cout << "Failed to init SPI communication.\n";
    }
    std::cout << "SPI communication successfully setup.\n";
  while(1)
  { 
    pthread_mutex_lock(&lock);
    if(!queue_empty(&q)){
      
    //print start of frame
  
            if(num_enteries(&q) >= 1800)
            {
              
              length=0;
              for(int i=0;i<1800;i++)
              {
                  dequeue(&q,&buf[i]);
                  length++;
              }
              pthread_mutex_unlock(&lock);
              for(int i=0;i<length;i++)
              {
                buf1[i] = buf[i];
              }
              while(digitalRead(BUSY) == 1);
              //send the data in buf1 to SPI
              wiringPiSPIDataRW(SPI_CHANNEL, buf1, length);
              digitalWrite(CLEAR, HIGH);
              usleep(1000);
              digitalWrite(CLEAR, LOW);
              bytes_send=0;
              for(int i=0;i<length;i++)
              {
                if(buf1[i]==1)
                {
                  bytes_send++;
                }
              }
              //if bytes send is equal is less than 1800 then send the rwmaining bytes from from buf
              for(int i=bytes_send;i<length;i++)
              {
                buf1[i] = buf[bytes_send+i];
              }
              while(digitalRead(BUSY) == 1);
              //send the data in buf1 to SPI
              wiringPiSPIDataRW(SPI_CHANNEL, buf1, length-bytes_send);
              digitalWrite(CLEAR, HIGH);
              usleep(1000);
              digitalWrite(CLEAR, LOW);
              while(digitalRead(BUSY) == 1);
              continue;
            }
            else if(num_enteries(&q)<1800 && num_enteries(&q)>0)
              {
                          length=0;
                          for(int i=0;i<num_enteries(&q);i++)
                          {
                              dequeue(&q,&buf[i]);
                              length++;
                          }
                         pthread_mutex_unlock(&lock);
                          for(int i=0;i<length;i++)
                          {
                            buf1[i] = buf[i];
                          }
                          while(digitalRead(BUSY) == 1);
                          //send the data in buf1 to SPI
                          wiringPiSPIDataRW(SPI_CHANNEL, buf1, length);
                          digitalWrite(CLEAR, HIGH);
                          usleep(1000);
                          digitalWrite(CLEAR, LOW);
                          while(digitalRead(BUSY) == 1);
                          bytes_send=0;
                          for(int i=0;i<length;i++)
                          {
                            if(buf1[i]==1)
                            {
                              bytes_send++;
                            }
                          }
                          //if bytes send is equal is less than 1800 then send the rwmaining bytes from from buf
                          for(int i=bytes_send;i<length;i++)
                          {
                            buf1[i] = buf[bytes_send+i];
                          }
                          while(digitalRead(BUSY) == 1);
                          //send the data in buf1 to SPI
                          wiringPiSPIDataRW(SPI_CHANNEL, buf1, length-bytes_send);
                          while(digitalRead(BUSY) == 1);
                          continue;
              }
           
        }
         else
            {
              pthread_mutex_unlock(&lock);
              usleep(1000);
            }
       

  }
}

main()
{
wiringPiSetup();
//CLEAR
pinMode(CLEAR, OUTPUT);
//BUSY
pinMode(BUSY, INPUT);

digitalWrite(CLEAR, LOW);
struct image image_sky;
int camera_fail =0;
cv::Mat im;                 //store image
cv::Mat im_transmit;        // transmit image
cv::Mat im_camera;          //camera image
raspicam::RaspiCam_Cv Camera;  // initialise camera
try {
 
  Camera.set(CV_CAP_PROP_MODE,4);
  Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1 );
  Camera.set(CV_CAP_PROP_FPS,2);
  Camera.set(CV_CAP_PROP_FRAME_WIDTH ,row_t);
  Camera.set(CV_CAP_PROP_FRAME_HEIGHT ,column_t);
  Camera.set(CV_CAP_PROP_GAIN,800);    //ISO
  Camera.set(CV_CAP_PROP_EXPOSURE,EXPOSURE_TIME_IN_MS*10000); // Exposure time 500ms
  //Camera.setAWB(0); // no automatic white balance
  //Camera.setExposureCompensation(0); // no exposure compensation
  usleep(100);
  Camera.open();
  }
  catch(int i){
  std::cout << "Camera not opened\n";
  camera_fail =1;
  }

if (TEST_MODE)
{
  im = cv::imread("/home/pi/test_image.jpg", CV_LOAD_IMAGE_GRAYSCALE);
  if(! im.data )                              // Check for invalid input
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }
  else
  {
    std::cout << "Image loaded successfully\n";
  }
}

int x,y,n,f;
x=column_t;
y=row_t;


const float error = ((ERROR_IN_ARC_SEC*3.14)/(3600*180));

int file_name=0;
char filename[64];

FRAME_NUMBER=0;

//init queue for 100000000 bytes
init_queue(&q,100000000);
//initailise the spi transmit thread
pthread_t thread_id;
pthread_create(&thread_id, NULL, data_transmit_spi, NULL);
printf ("Newly created thread id is %d\n", thread_id);
int flag=0;

//get current time 
time_t start, end;
time(&start);

while(1)
{
//print start frame
std::cout << "Start Frame" << std::endl;
// if queue is full wait until queue is queue_empty
pthread_mutex_lock(&lock);
printf("queue size is %ld\n",num_enteries(&q));
if(num_enteries(&q) > 99000000) flag=1;
pthread_mutex_unlock(&lock);

while(flag)
{
  pthread_mutex_lock(&lock);
  //print number of bytes in queue
  printf("queue size is %ld \n",num_enteries(&q));
  if(queue_empty(&q)) flag=0;
  pthread_mutex_unlock(&lock);
  //sleep for 10 seconds
  sleep(10);
  
}


//print over check done
//std::cout << "Over check done" << std::endl;
if(camera_fail==0){
    //file_name++;
    //sprintf (filename, "image_%d.jpg",file_name);
Camera.grab(); // Capture frame
Camera.retrieve(im_camera);
    //cv::imwrite(filename,im);
    //frames++;
    //cv::imwrite(filename,im);
    //fp=fopen("out.txt","a+");
    //std::cout <<"image to array\n";
    im=im_camera;
}

//std::cout <<"image to array\n";

for(int i=0;i< row_t; i++)
{
  for(int j=0;j<column_t; j++ )
  {
     image[i][j]=(int)(im.at<uchar>(i,j) );
 
  }
}

//std::cout <<"image to array: Done \n";

int flag,p_v,px;
float pix[2000],pix_sum[2000];
long int star_id,stars;
float x_pix[2000],y_pix[2000];

stars=0;
static int w;
float threshold ;
// threshold determination


// find mean
float pixel_sum=0;
float total_pixels=0;



//std::cout <<"mean\n";
for(int i=0; i< row_t ;i=i+8)
{
    for(int j=0; j < column_t ; j=j+8)
     {
      pixel_sum = pixel_sum + (float) image[i][j];
      total_pixels++;
     }
}



//std::cout <<"Variance\n";

float pixel_mean =pixel_sum /total_pixels;

float pixel_variance=0;
// find variance


for(int i=0; i<row_t ;i=i+8)
{
for(int j=0; j <column_t; j=j+8)
{
   pixel_variance = pixel_variance + pow( (image[i][j] - pixel_mean),2);
}

}

pixel_variance =pixel_variance/total_pixels;


//Set threshold here current 5 sigma
threshold =  ( pixel_mean + 5*sqrt( pixel_variance));

//Region growing matrix

for(int i=0;i< row_t ;i++)
 {
  for(int j=0;j<column_t;j++)
     {
           RG_Matrix[i][j]= 0;
     }
}


std::cout <<"begin RG\n";
for(int i=5;i< row_t-5 ;i++)
{
  if(stars >1500) //To prevent segmentation fault
        break;

   for(int j=5;j< column_t-5;j++)
     {
          p_v=image[i][j];

    // pixels above threshold
         if(p_v > threshold)

      {
            flag=1;
            // old_region
             w=4;

             for(int a=-4;a < 2 ; a++)
               {
                for (int b=-4; b < w+1; b++)
                   {
                     px=RG_Matrix[i+a][j+b];
                     if( px != 0)
                         {
                         flag=0;
                          star_id= px;
                          RG_Matrix[i][j]= px;
                          }
                    }


                w=w-1;
             }



         if(flag==0)
           {

          pix[star_id]=pix[star_id] + 1;
          pix_sum[star_id]=p_v-threshold + pix_sum[star_id];
          x_pix[star_id]=(1+j)*(p_v-threshold) + x_pix[star_id] ;
          y_pix[star_id]=(1+i)*(p_v-threshold) + y_pix[star_id] ;

            }

      //rew region
         else if(flag==1)

           {
          stars=stars+1;
          RG_Matrix[i][j]= stars;
          pix[stars]=1;
          pix_sum[stars]=p_v-threshold;
          x_pix[stars]=(1+j)*(p_v-threshold);
          y_pix[stars]=(1+i)*(p_v-threshold);
           }


       }
   }
}


// now lets begin to remove single eventS
image_sky.stars=0;
for(int i=0;i<stars;i++)
 {
        if((pix[i]>pixel_no_threshold) && (pix[i]<10000))
        {
        image_sky.stars = image_sky.stars+1;
        image_sky.x_cen[image_sky.stars-1] =x_pix[i]/pix_sum[i];
        image_sky.y_cen[image_sky.stars-1]=y_pix[i]/pix_sum[i];
        image_sky.pix[image_sky.stars-1] = pix_sum[i];
        }

        if(image_sky.stars>499) break;
 }
std::cout <<"stars found "<<image_sky.stars<<"\n";

//Sort stars as decreasing order of brightness
float temp;
for(int i=0; i<image_sky.stars; i++)
{
    for(int j=0; j<image_sky.stars; j++)
    {
        if(image_sky.pix[j] < image_sky.pix[j+1])
        {
            temp     = image_sky.pix[j];
            image_sky.pix[j] = image_sky.pix[j+1];
            image_sky.pix[j+1] = temp;
            temp     = image_sky.x_cen[j];
            image_sky.x_cen[j] = image_sky.x_cen[j+1];
            image_sky.x_cen[j+1] = temp;
            temp     = image_sky.y_cen[j];
            image_sky.y_cen[j] = image_sky.y_cen[j+1];
            image_sky.y_cen[j+1] = temp;
        }
    }
}

  struct image captured_frame;
  struct quaternion quat;
  struct pointing pointo;
  struct stars_found star_list[20];

  //Select 20 brightest star and calculate the unit vectors
  for(int i=0;i<image_sky.stars && i<20;i++)
  {

    star_list[i].cen[0]=image_sky.x_cen[i];
    star_list[i].cen[1]=image_sky.y_cen[i];
    cent_to_unitvector(star_list[i].cen,star_list[i].unit_vector);
    star_list[i].star_id=0;
    star_list[i].primary_vote=0;
    star_list[i].secondary_votes=0;
    star_list[i].total_vote=0;
    for(int j=0;j<number_of_stars_in_catalog+1 ;j++)
    {
      star_list[i].vote_list[j]=0;
    }


  }

  int index1,index2;
  float angle;
  for(int i=0;i<image_sky.stars && i<20;i++)
  {

     for(int j=i+1;j<image_sky.stars && j<20;j++)
     {


	     angle = dot_product_cos(star_list[i].unit_vector,star_list[j].unit_vector);

       if(angle > (fov*pii/180) || angle < (0.001))
        { continue;
        }


       index1=hash_index(angle+error);
      
       index2=hash_index(angle-error);


       for(int k=index2; k<= index1; k++)
       {
         star_list[i].vote_list[angle_id[k][0]]= star_list[i].vote_list[angle_id[k][0]] + 1;
         star_list[i].total_vote=star_list[i].total_vote + 1;
         star_list[j].vote_list[angle_id[k][0]]=star_list[j].vote_list[angle_id[k][0]] + 1;
         star_list[j].total_vote=star_list[j].total_vote + 1;
         star_list[i].vote_list[angle_id[k][1]]= star_list[i].vote_list[angle_id[k][1]] + 1;
         star_list[i].total_vote=star_list[i].total_vote + 1;
         star_list[j].vote_list[angle_id[k][1]]=star_list[j].vote_list[angle_id[k][1]] + 1;
         star_list[j].total_vote=star_list[j].total_vote + 1;
       }

    }
  }


  for(int k=0;k< image_sky.stars && k< 20 ;k++)
  {


     int index = 0, maxCount = 0;

     for (int i = 1; i < number_of_stars_in_catalog + 1;i++) {


         if(star_list[k].vote_list[i] > maxCount)
         {
           index= i;
           maxCount =star_list[k].vote_list[i] ;
         }



     }

     star_list[k].primary_vote =  maxCount;
     star_list[k].star_id = index;
  }

  
  for(int i=0;i < image_sky.stars && i< 20; i++)
  {
    for(int j=i+1; j <image_sky.stars && j< 20 ;j++)
    {
      if(star_list[i].star_id==0 || star_list[j].star_id==0)
      {
        continue;
      }

     angle =dot_product_cos(star_list[i].unit_vector,star_list[j].unit_vector)-dot_product_cos_using_id(star_list[i].star_id,star_list[j].star_id) ;

     if(abs(angle) < 0.01 )
     {
       star_list[i].secondary_votes=star_list[i].secondary_votes + 1;
       star_list[j].secondary_votes=star_list[j].secondary_votes + 1;
     }

    }


  }




  // QUEST algorithm

  int l_opt=0;
  float B[3][3]={{0,0,0},{0,0,0},{0,0,0}}, B_t[3][3], vec_mat[3][3], S[3][3], vec1[3],vec2[3];
  float Z[3], sigma,a, p[3]={0,0,0}, b;
  float a_mat[3][3]={{0,0,0},{0,0,0},{0,0,0}}, a_minus_S[3][3], aS_inv[3][3];



  // Generate B matrix
  for(int i=0;i<image_sky.stars  && i<20;i++)
  {
    if (star_list[i].star_id!=0 && star_list[i].secondary_votes > 2 )
    {
      // Generate matrix from vector




      vec1[0] = unit_vector_table[star_list[i].star_id - 1][0];
      vec1[1] = unit_vector_table[star_list[i].star_id - 1][1];
      vec1[2] = unit_vector_table[star_list[i].star_id - 1][2];
      vec2[0] = star_list[i].unit_vector[0];
      vec2[1] = star_list[i].unit_vector[1];
      vec2[2] = star_list[i].unit_vector[2];

      vec_mul_1(vec1,vec2,vec_mat);
      // Add matrix
      mat_add(B,vec_mat,B);
      l_opt++;
    }
  }




  // Generate S matrix
  transpose(B,B_t);
  mat_add(B,B_t,S);

  // Generate Z vector
  Z[0] = B[1][2] - B[2][1];
  Z[1] = B[2][0] - B[0][2];
  Z[2] = B[0][1] - B[1][0];

  // Calculate sigma
  sigma = trace(B);



  // Calculate a
  a = sigma + (float)l_opt;

  a_mat[0][0] = a; a_mat[1][1] = a; a_mat[2][2] = a;

  // Calculate inverse of a-S - incomplete
  mat_sub(a_mat,S,a_minus_S);
  inverse(a_minus_S,aS_inv);


  // Calculate p rodrigues parameter
  mat_vec_mul(aS_inv,Z,p);



  // Calculate quaternion from p vector
  b = 1/(sqrt(vec_mul_2(p,p) + 1));
  quat.q1 = b;
  quat.q2 = b*p[0];
  quat.q3 = b*p[1];
  quat.q4 = b*p[2];
  //if quatrternions are nan, set to 0
  /*if(isnan(quat.q1)) quat.q1 = 0;
  if(isnan(quat.q2)) quat.q2 = 0;
  if(isnan(quat.q3)) quat.q3 = 0;
  if(isnan(quat.q4)) quat.q4 = 0;
  */
  std::cout << "Quaternion: " << quat.q1 << " " << quat.q2 << " " << quat.q3 << " " << quat.q4 << std::endl;
  //Split FRAME_NUMBER into 2 bytes
  uint8_t frame_number_1 = (uint8_t)(FRAME_NUMBER >> 8);
  uint8_t frame_number_2 = (uint8_t)(FRAME_NUMBER & 0xFF);

  //Get rpi cpu temperature
  float cpu_temp = get_cpu_temp();
  //print cpu temp  (for debugging)
  std::cout << "CPU temp: " << cpu_temp << std::endl;
  //convert to 2 bytes
  uint8_t cpu_temp_1 = (uint8_t)(abs(cpu_temp));
  //set MSB to 1 if negative
  if(cpu_temp < 0) cpu_temp_1 = cpu_temp_1 | 0x80;
  //get end time
   time(&end);
   // measure time difference between start and end in seconds unsigned long  INT
    unsigned long int time_diff = (unsigned long int)difftime(end,start);
  //convert to 4 bytes
  uint8_t time_diff_1 = (uint8_t)(time_diff >> 24);
  uint8_t time_diff_2 = (uint8_t)(time_diff >> 16);
  uint8_t time_diff_3 = (uint8_t)(time_diff >> 8);
  uint8_t time_diff_4 = (uint8_t)(time_diff & 0xFF);
  //convert cam_fail to 1 byte
  uint8_t cam_fail_flag = (uint8_t)camera_fail;
  
  //CONVERT quarternions to 32bit unsigned int
  uint32_t q1 = (uint32_t)(abs(quat.q1)*1000000);
  uint32_t q2 = (uint32_t)(abs(quat.q2)*1000000);
  uint32_t q3 = (uint32_t)(abs(quat.q3)*1000000);
  uint32_t q4 = (uint32_t)(abs(quat.q4)*1000000);
  //if negative, set MSB to 1
  if(quat.q1 < 0) q1 = q1 | 0x80000000;
  if(quat.q2 < 0) q2 = q2 | 0x80000000;
  if(quat.q3 < 0) q3 = q3 | 0x80000000;
  if(quat.q4 < 0) q4 = q4 | 0x80000000;
  
  //convert 32bit unsigned int to 4 bytes
  uint8_t q1_1 = (uint8_t)(q1 >> 24);
  uint8_t q1_2 = (uint8_t)(q1 >> 16);
  uint8_t q1_3 = (uint8_t)(q1 >> 8);
  uint8_t q1_4 = (uint8_t)(q1);
  uint8_t q2_1 = (uint8_t)(q2 >> 24);
  uint8_t q2_2 = (uint8_t)(q2 >> 16);
  uint8_t q2_3 = (uint8_t)(q2 >> 8);
  uint8_t q2_4 = (uint8_t)(q2);
  uint8_t q3_1 = (uint8_t)(q3 >> 24);
  uint8_t q3_2 = (uint8_t)(q3 >> 16);
  uint8_t q3_3 = (uint8_t)(q3 >> 8);
  uint8_t q3_4 = (uint8_t)(q3);
  uint8_t q4_1 = (uint8_t)(q4 >> 24);
  uint8_t q4_2 = (uint8_t)(q4 >> 16);
  uint8_t q4_3 = (uint8_t)(q4 >> 8);
  uint8_t q4_4 = (uint8_t)(q4);
  //print the 4 bytes in binary using print binary function
  std::cout << "q1: \n";
  print_binary(q1_1);
  print_binary(q1_2);
  print_binary(q1_3);
  print_binary(q1_4);
  std::cout << "q2: \n";
  print_binary(q2_1);
  print_binary(q2_2);
  print_binary(q2_3);
  print_binary(q2_4);
  std::cout << "q3: \n";
  print_binary(q3_1);
  print_binary(q3_2);
  print_binary(q3_3);
  print_binary(q3_4);
  std::cout << "q4: \n";
  print_binary(q4_1);
  print_binary(q4_2);
  print_binary(q4_3);
  print_binary(q4_4);


  //convert 20 x and y centroid to uint16_t
  uint16_t x_centroid[20];
  uint16_t y_centroid[20];
  for(int i=0;i<20;i++)
  {
    x_centroid[i] = (uint16_t)(image_sky.x_cen[i]*100);
    y_centroid[i] = (uint16_t)(image_sky.y_cen[i]*100);
  }
  //covert the 20 x and y centroid to 2 bytes
  uint8_t x_centroid_1[20];
  uint8_t x_centroid_2[20];
  uint8_t y_centroid_1[20];
  uint8_t y_centroid_2[20];
  for(int i=0;i<20;i++)
  { 
    if(i <image_sky.stars)
    {
    x_centroid_1[i] = (uint8_t)(x_centroid[i] >> 8);
    x_centroid_2[i] = (uint8_t)(x_centroid[i] & 0x00FF);
    y_centroid_1[i] = (uint8_t)(y_centroid[i] >> 8);
    y_centroid_2[i] = (uint8_t)(y_centroid[i] & 0x00FF);
   }
   else
   {
    x_centroid_1[i] = 0;
    x_centroid_2[i] = 0;
    y_centroid_1[i] = 0;
    y_centroid_2[i] = 0;
   }
  }
 // print all the x_centroid and y centrid bytes using print binary
  /*for(int i=0;i<20;i++)
  {
    std::cout << "x_centroid_1[" << i << "]: " << std::endl;
    print_binary(x_centroid_1[i]);
    std::cout << "x_centroid_2[" << i << "]: " << std::endl;
    print_binary(x_centroid_2[i]);
    std::cout << "y_centroid_1[" << i << "]: " << std::endl;
    print_binary(y_centroid_1[i]);
    std::cout << "y_centroid_2[" << i << "]: " << std::endl;
    print_binary(y_centroid_2[i]);
  }
 */

  pthread_mutex_lock(&lock);
// enqeue data in q with header 0x00 0x00 0x00 0x00, frame number 1,2 and cpu temp 1
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,frame_number_1);
  enqueue(&q,frame_number_2);
  enqueue(&q,cpu_temp_1);
  enqueue(&q,time_diff_1);
  enqueue(&q,time_diff_2);
  enqueue(&q,time_diff_3);
  enqueue(&q,time_diff_4);
  enqueue(&q,cam_fail_flag);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
// enqeue data in q with hearder 0x00 0x00 0x00 0x01, quartenions
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,0x01);
  enqueue(&q,q1_1);
  enqueue(&q,q1_2);
  enqueue(&q,q1_3);
  enqueue(&q,q1_4);
  enqueue(&q,q2_1);
  enqueue(&q,q2_2);
  enqueue(&q,q2_3);
  enqueue(&q,q2_4);
  enqueue(&q,q3_1);
  enqueue(&q,q3_2);
  enqueue(&q,q3_3);
  enqueue(&q,q3_4);
  enqueue(&q,q4_1);
  enqueue(&q,q4_2);
  enqueue(&q,q4_3);
  enqueue(&q,q4_4);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
  enqueue(&q,0xff);

// enqueue data in q with hearder 0x00 0x00 0x00 0x02, centroids
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,0x00);
  enqueue(&q,0x02);
  for(int i=0;i<20;i++)
  {
    enqueue(&q,x_centroid_1[i]);
    enqueue(&q,x_centroid_2[i]);
    enqueue(&q,y_centroid_1[i]);
    enqueue(&q,y_centroid_2[i]);
  }
  enqueue(&q,0xff);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
  enqueue(&q,0xff);
//EVERY 600 frames, send image

  if((FRAME_NUMBER+1)%150 == 0)
  {
        //reduce the im size to half and store in im_transmit
        cv::resize(im, im_transmit, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
      // enqeue data in q with hearder 0x00 0x00 0x00 0x03, image
        enqueue(&q,0x00);
        enqueue(&q,0x00);
        enqueue(&q,0x00);
        enqueue(&q,0x03);
        for(int i=0;i<im_transmit.rows;i++)
        {
          for(int j=0;j<im_transmit.cols;j++)
          {
            enqueue(&q,im_transmit.at<uchar>(i,j));
          }
          enqueue(&q,0xff);
          enqueue(&q,0xff);
        }
       enqueue(&q,0xff);
       enqueue(&q,0xff);
       enqueue(&q,0xff);
       enqueue(&q,0xff);
      
  }
 FRAME_NUMBER++;
  pthread_mutex_unlock(&lock);
 //sleep for one second
  sleep(1);

}

Camera.release();

return 0;
}
