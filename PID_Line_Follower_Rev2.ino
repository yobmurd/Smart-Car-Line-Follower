
float Kp=100,Ki=0,Kd=.24;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0;
float previous_time=0;
int sensor[5]={0, 0, 0, 0, 0};
const int initial_motor_speed=255;
int left_motor_speed = 0;
int right_motor_speed = 0;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

const int pin_left_motor_enable = 6;
const int pin_right_motor_enable = 11;
 
const int pin_left_forward = 4;
const int pin_left_reverse = 5;

const int pin_right_forward = 12;
const int pin_right_reverse = 3;


const int max_speed = 255;

void setup()
{
  pinMode(pin_right_motor_enable,OUTPUT); 
  pinMode(pin_left_motor_enable,OUTPUT); 

  digitalWrite(pin_left_motor_enable,0);
  digitalWrite(pin_right_motor_enable,0);
  
  pinMode(pin_left_forward,OUTPUT);
  pinMode(pin_left_reverse,OUTPUT);
  pinMode(pin_right_forward,OUTPUT);
  pinMode(pin_right_reverse,OUTPUT);

  digitalWrite(pin_left_forward,0);
  digitalWrite(pin_left_reverse,0);
  digitalWrite(pin_right_forward,0);
  digitalWrite(pin_right_reverse,0);
  
 Serial.begin(115200);
}


/*void trace(String s) {
  return;
  Serial.print(s);
}
void trace(int i) {
  trace(String(i));
}*/


void loop()
{
  read_sensor_values();
/*  trace("Sensors: ");
  trace(sensor[0]);
  trace(sensor[1]);
  trace(sensor[2]);
  trace(sensor[3]);
  trace(sensor[4]);
  trace("  Error: ");
  trace(error);*/
  calculate_pid();
  motor_control();
  /*trace(" Left: ");
  trace(left_motor_speed);

  trace(" Right: ");
  trace(right_motor_speed);
  trace("\n");*/
    
}

void read_sensor_values()
{
  sensor[0]=digitalRead(A0);
  sensor[1]=digitalRead(A1);
  sensor[2]=digitalRead(A2);
  sensor[3]=digitalRead(A3);
  sensor[4]=digitalRead(A4);
  
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==1))
    error=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==1))
    error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==1)&&(sensor[4]==0))
    error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==1)&&(sensor[4]==0))
    error=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
    error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
    error=-4;
  //else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[3]==0)&&(sensor[4]==0))
  //  if(error==-4) error=-5;
  //  else error=5;

}

void calculate_pid()
{
    if (error!=previous_error)
    {
      P = error;
      I = I + 1e-6*(error-previous_error)*(micros()-previous_time);
      D = 1e6*(error-previous_error)/(micros()-previous_time);
     previous_error=error;
     previous_time=micros();
     PID_value = Kp*P + Ki*I + Kd*D; 
    }
}

void motor_control()
{
    // Calculating the effective motor speed:
    left_motor_speed = constrain(initial_motor_speed+PID_value, -max_speed,max_speed);
    right_motor_speed = constrain(initial_motor_speed-PID_value, -max_speed,max_speed);

    // set directions
    digitalWrite(pin_left_forward,left_motor_speed > 0);
    digitalWrite(pin_left_reverse,left_motor_speed < 0);

    digitalWrite(pin_right_forward,right_motor_speed > 0);
    digitalWrite(pin_right_reverse,right_motor_speed < 0);
    
    // set speeds
    analogWrite(pin_left_motor_enable,abs(left_motor_speed));
    analogWrite(pin_right_motor_enable,abs(right_motor_speed));
}
