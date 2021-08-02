#include "mbed.h"

#include "PinDetect.h"
#include "QEI.h"
#include "eeprom.h"
#include "motor.h"
#include "states.h"
#include <stdio.h>


#define EEPROM_ADDR 0x00
//#define EE_UID_ADDR 0x01             
//#define EE_LT_FLUSHES_ADDR 0x02          
//#define EE_LT_FILM_USED_ADDR 0x03      
//#define EE_LT_BLOCK_ADDR  0x04           


#define SDA PA_10
#define SCL PA_9


/////////////////////////
/////// Buttons /////////
/////////////////////////

PinDetect Flush_Button(PB_4); //PB_4
PinDetect Forward_Button(PB_0);//PB_0
PinDetect Backward_Button(PB_5);
Motor motor(PA_4, PA_5, PA_6, PA_7, PA_0);
QEI encoder(PA_1, PA_3, NC, 2338);
DigitalOut myled (LED2);
DigitalOut Flush_Led(PA_8);


///////////////////////////
/////// Variables /////////
///////////////////////////
int UID = 123;


float JAM_RPM_FORWARD = 30.0;
float JAM_RPM_BACKWARD = 30.0;


int pulses = 0;
int cummulative_pulses = 0;

int LT_FILM_USED = 0;
int LT_FLUSHES = 0;
int LT_BLOCKAGES = 0;
int LT_SERVICES = 0;




//////////////////////////
//////// Bools ///////
/////////////////////////

bool update_film_value = false;
bool update_rpm_value = false;
bool eeprom_reset = false;
bool mode_changed = false;
bool jam_detected = false;
bool update_time = false;
bool update_flush_count = false;

int flush_count = 192000.0;

int timeFlushStarted = 0;

//////////////////////////
////////////RTOS /////////
/////////////////////////

Timer t;

Semaphore motorSema(1);
Semaphore faultSema(1);

void SystemStates_thread(void const *name);
void motorDrive_thread(void const *name);
void fault_thread(void const *name);
void check_thread(void const *name);



Thread t1;
Thread t2;
Thread ledThread;
Thread faultThread;
Thread checkThread;


Timeout flush_end;
Timeout reset_end;

//////////////////////////
//////////Functions///////
/////////////////////////

void start_flush(void);
void end_flush(void);
void end_action(void);
void forward(void);
void backward(void);
void reset_eeprom(void);
void mode_change(void);
void check_forward(void); 
void check_backward(void);

void setup_flush_button(void);
void setup_forward_button(void);
void setup_backward_button(void);
void setup_reset_button(void); 
void setup_mode_button(void); 


void motor_jam_detected(void);

void led2_thread(void);



////////////////////////////////////
//////////BLOCKAGE PARAMETRER///////
////////////////////////////////////

int min_forward_pulse_count = 0;//230; //28 RPM
int min_backward_pulse_count = 0;//200;
    
volatile int pulse_count = 0;
volatile int pulse_count_prev = 0;
volatile int delta_pulse_count = 0;

volatile int block_count = 0;
int max_block_counts = 10;


EEPROM *ep;
Serial pc(USBTX, USBRX);

void setup() 
{
    pc.baud(115200);
    
    printf("Starting...\r\n");
    printf("SystemCoreClock is %d Hz\r\n", SystemCoreClock);


 

    ep = new EEPROM(SDA, SCL, EEPROM_ADDR, EEPROM::T24C64);

    uint8_t initep = 0xFF;
    uint8_t testdata = 0xFF; 

    ep->read((uint32_t)0, (int8_t&)initep);//read address 0 of eeprom
    printf("%d", ep->getError());
    printf("initep %X\n", initep);
    
    if (initep != 0x55) // if address 0 is empty, it is new eeprom
    {


        ep->write((uint32_t)0, 0x55); //if eeprom is new, write 1 to address 0.
        ep->read((uint32_t)0, (int8_t &)testdata);

        printf("initeo %X\n", initep);

        ep->write((uint32_t)0x10,LT_FLUSHES); 
        ep->write((uint32_t)0x20,LT_FILM_USED); 
        ep->write((uint32_t)0x30,LT_BLOCKAGES); 
        
    }
    else 
    {
    
        ep->read((uint32_t)0x10, (int &) LT_FLUSHES);
        printf("Total Number of Flushes = %d\n",LT_FLUSHES);

        ep->read((uint32_t)0x20, (int &) LT_FILM_USED);
        printf("Total amount of Film Flushed/pulses = %d\n",LT_FILM_USED);

        ep->read((uint32_t)0x30, (int &) LT_BLOCKAGES);
        printf("Total Number of Blockages = %d\n",LT_BLOCKAGES);

    }
   
    //Flush_Led = 1;
    myled = 1;
    
    sysState_struct.sysMode = S_RUN;

    setup_flush_button();
    setup_forward_button();
    setup_backward_button();

    t1.start(callback(motorDrive_thread, (void *)"MotorDriveThread"));
    t2.start(callback(SystemStates_thread, (void *)"SystemStateThread"));
    faultThread.start(callback(fault_thread,(void *)"MotorFaultThread"));
    checkThread.start(callback(check_thread,(void *)"SystemFaultThread"));

}



int main() 
{
  setup();

  while (true) 
  {

    if (update_film_value) 
    {
        //Flush_Led = 1;

     //printf("Pulse Difference %d\n", pulses);

      LT_FILM_USED = abs(pulses) + LT_FILM_USED; 
      printf("FILM =  %d\n", LT_FILM_USED);

      ep->write((uint32_t)0x20,LT_FILM_USED); 

      pulses = 0;
      encoder.reset();
      update_film_value = false;
    }

    if(update_flush_count)
    {
     LT_FLUSHES = LT_FLUSHES + 1;
     ep->write((uint32_t)0x10,LT_FLUSHES); 
     printf("FLUSHES =  %d\n", LT_FLUSHES);

     update_flush_count = false;

    }

    if (jam_detected) 
    {
        t.stop();
        t.reset();

        ep->write((uint32_t)0x30,LT_BLOCKAGES); 
        printf("BLOCKAGES =  %d\n", LT_BLOCKAGES); 


        pulses = 0;
        encoder.reset();
        jam_detected = false;

    }

  }

}


void fault_thread(void const *name)
{
    

    while (true) 
    {
            
      if (faultSema.try_acquire_until(5000)) 
      {
        pulse_count = abs(encoder.getPulses());

    

        delta_pulse_count = pulse_count - pulse_count_prev;
        pc.printf("delta_pulse_count = %d\n", delta_pulse_count);
        pulse_count_prev = pulse_count;

        if (gMotorAction == MA_Forward) 
        {
          if (delta_pulse_count < min_forward_pulse_count) 
          {
            block_count++;
            pc.printf("Forward Blockage Count = %d, %d\n",delta_pulse_count, block_count);
            if (block_count > max_block_counts) 
            {

              end_action(); 
              LT_BLOCKAGES = LT_BLOCKAGES + 1; 
              jam_detected = true; 
                      
         
              pc.printf("Blockage");
              block_count = 0;
            }
          }
        }

        if (gMotorAction == MA_Backward) 
        {
          if (delta_pulse_count < min_forward_pulse_count) 
          {
            block_count++;
            pc.printf("Backward Blockage Count = %d, %d\n", delta_pulse_count, block_count);
            if (block_count > max_block_counts) 
            {

                end_action();
                jam_detected = true;           

              pc.printf("Blockage");
              block_count = 0;
            }
          }
        }
      }

      ThisThread::sleep_for(100);

      if (motor._MState == MFORWARD || motor._MState == MBACKWARD) 
      {
        faultSema.release();
      } 
      
      else 
      
      {
      
      }
    }
}


void check_thread(void const *name)
{
  

    while(true)
    {


    }

 
}

void end_action()
{
    if (motor._MState == MFORWARD || motor._MState == MBACKWARD) 
    {

     myled = 0;
     gMotorAction = MA_Brake;
     motorSema.release();
  
     pulses = encoder.getPulses();
     encoder.reset();

     update_film_value = true;  

     flush_end.attach(&end_flush, 0.1); // timeout - duration of flush  
      
    }     
}  

void forward(void) 
{

  if (sysState_struct.sysMode == S_RUN) 
  {
    if (motor._MState == MSTOP) 
    {
      myled = 1;
      block_count = 0;
      gMotorAction = MA_Forward;
      faultSema.release();
      motorSema.release();
      flush_end.attach(&end_action, 10.0); // timeout - duration of flush
    }
  }
}

void backward(void) 
{

  if (sysState_struct.sysMode == S_RUN) 
  {
    if (motor._MState == MSTOP) 
    {
      myled = 1;
      block_count = 0;
      gMotorAction = MA_Backward;
      motorSema.release();
      faultSema.release();
      flush_end.attach(&end_action, 10.0); // timeout - duration of flush
    }
  }
}



void start_flush(void) 
{
  if (sysState_struct.sysMode == S_RUN) 
  {
    if (motor._MState == MSTOP) 
    {
      myled = 1;
      block_count = 0;
      gMotorAction = MA_Forward;
      t.start();
      motorSema.release();
      faultSema.release();

      timeFlushStarted = t.read_ms();
    
      flush_end.attach(&end_flush, 3.0); // timeout - duration of flush - 25cm       
    }
  }
}

void end_flush(void)
{
   static int timelimit = 1500; //12cm
  
  if (motor._MState == MFORWARD ) 
  {
    myled = 0;
    if (t.read_ms() - 0 > timelimit) 
    {
     gMotorAction = MA_Brake;
     motorSema.release();
     t.stop();
     t.reset();

     pulses = encoder.getPulses();
     encoder.reset();

     update_film_value = true;  
     update_flush_count = true;
     
    }
    flush_end.attach(&end_flush, 0.1); // timeout - duration of flush  
    
    }  

}



void motor_jam_detected(void)
{
 jam_detected = true;
}

void setup_flush_button(void) 
{
  Flush_Button.mode(PullUp);
  Flush_Button.attach_asserted(&start_flush);
  Flush_Button.attach_deasserted(&end_flush);
  Flush_Button.setSamplesTillAssert(5); // debounces 10 sample by
  Flush_Button.setAssertValue(0);        // state of the pin
  Flush_Button.setSampleFrequency();     // Defaults to 20ms.
  //Flush_Button.setSamplesTillHeld(100);
}

void setup_forward_button(void) 
{
  Forward_Button.mode(PullUp);
  Forward_Button.attach_asserted(&forward);
  Forward_Button.attach_deasserted(&end_action);
  Forward_Button.setSamplesTillAssert(10); // debounces 10 sample by
  Forward_Button.setAssertValue(0);        // state of the pin
  Forward_Button.setSampleFrequency();     // Defaults to 20ms.
  //Flush_Button.setSamplesTillHeld(100);
}

void setup_backward_button(void) 
{
  Backward_Button.mode(PullUp);
  Backward_Button.attach_asserted(&backward);
  Backward_Button.attach_deasserted(&end_action);
  Backward_Button.setSamplesTillAssert(10); // debounces 10 sample by
  Backward_Button.setAssertValue(0);        // state of the pin
  Backward_Button.setSampleFrequency();     // Defaults to 20ms.
  //Flush_Button.setSamplesTillHeld(100);
}


void SystemStates_thread(void const *name) 
{
  while (true) 
  {
    switch (sysState_struct.sysMode) 
    {
    case S_RUN:

        sysState_struct.sysMode = S_RUN;
           
        break;
        
    case S_SER:

        sysState_struct.sysMode = S_SER;

        break;
   
    default:
        
      printf("Error: Undefined Mode\n");
    }
  }
}

void motorDrive_thread(void const *name) 
{
  while (true) {
    motorSema.acquire();
    switch (gMotorAction) 
    {
    case MA_Forward:
      motor.forward();
      break;
    case MA_Backward:
      motor.backward();
      break;
    case MA_Brake:
      motor.brake();
      break;
    case MA_Stop:
      motor.stop();

      break;
    }
  }
}
