#include <BasicLinearAlgebra.h>
#include "quaternion.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define N 6

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//int send_cmd(int motor, int pwm_value){
////  Serial.println(pwm_value);
//  if (motor >= 3){
//    pwm.setPWM(motor, 0, 2*2550-pwm_value);
//  } else {
//     pwm.setPWM(motor, 0, pwm_value);
//  }
//  return 0;
//}

float pwm_per_rad = 900/0.3905;
int max_val = 4000;
int min_val = 1100;

int send_angle(int motor, float angle){
  int pwm_value = (int) -pwm_per_rad*angle + 2550;
  if (motor >= 3){
    pwm_value = 2*2550-pwm_value;
  }
  
  Serial.println(pwm_value);
  
  if (pwm_value > max_val){
    pwm_value = max_val;
    Serial.println("Max out");
  }
  if (pwm_value < min_val){
    pwm_value = min_val;
    Serial.println("Min out");
  }
  
  pwm.setPWM(motor, 0, pwm_value);
}




float d_avg = 0;

//quaternion *q = new quaternion(0, 1, 0, 0);

// horn length
float h = 24;

// Base dimensions
float adjacent_distance = 34.71; //mm
float pair_distance = 112.96; //mm
float base_side_length = pair_distance + 2*adjacent_distance;

float triangle_height = sqrt(3)*base_side_length/2;
float com = triangle_height/3;
float small_triangle_height = sqrt(3)*adjacent_distance/2;

// Variables for platform dimensions
float hex_length = 68.02 + (10.8)*2/sqrt(3); // side length of the hexagon, plus modified distance to anchors
float bolt_spacing = 19.7; // distance between bolts
float hex_triangle_height = sqrt(3)*hex_length/2;

// Matrix to flip vectors across x axis.
BLA::Matrix<3,3> flip_x = {-1.0, 0, 0,
                           0, 1.0, 0,
                           0, 0, 1.0};


// Rotation matrix 120 degrees around z axis
float c = cos(-2*M_PI/3);
float s = sin(-2*M_PI/3);
BLA::Matrix<3,3> rotation = {c, -s, 0,
                             s, c, 0,
                             0, 0, 1.0};


BLA::Matrix<N,3> b; // location of motors
BLA::Matrix<N,3> p; // location of platform attachment points
BLA::Matrix<N,3> p_rot; // Rotation of platform attachment points by R
BLA::Matrix<N> d = {72.27, 73.35, 72.68, 73.11, 74.77, 73.80}; // lengths of linkages
BLA::Matrix<N> beta = {0., 2.0943951, -2.0943951, 3.14159265, 5.23598776, 1.04719755}; // angles of horns in rad
BLA::Matrix<N,3> l; // linkage vectors. 
BLA::Matrix<N> a;
//BLA::Matrix<N> f;
//BLA::Matrix<N> g;

float e;
float f;
float g;

rot R = rot();
BLA::Matrix<3> T = {0, 0, 71};

// Convenience functions for slicing matricies
template<int row_number>
using row = BLA::Slice<row_number, row_number+1>;
template<int col_number>
using col = BLA::Slice<col_number, col_number+1>;

auto row3 = BLA::Slice<0,3>();
auto col6 = BLA::Slice<0,6>();


void setup() {
  Serial.begin(115200);

  // Setup servos
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(800); 
  Wire.setClock(4000);

  for (int i = 0; i < N; i++){
    d_avg += d(i);
  }
  d_avg /= N;


  // Prototype vector
  b.Submatrix(row<0>(), row3) << adjacent_distance/2, 2*com-small_triangle_height, 0;

  // Rotate prototype vector to get motors 1 & 2
  b.Submatrix(row<2>(), row3) = ~(rotation * ~b.Submatrix(row<0>(), row3));
  b.Submatrix(row<1>(), row3) = ~(rotation * ~b.Submatrix(row<2>(), row3));

  // Motors 3,4, and 5 are the flipped versions of motors 0, 2, and 1 respectively
  b.Submatrix(row<3>(), row3) = ~(flip_x * ~b.Submatrix(row<0>(), row3));
  b.Submatrix(row<4>(), row3) = ~(flip_x * ~b.Submatrix(row<2>(), row3));
  b.Submatrix(row<5>(), row3) = ~(flip_x * ~b.Submatrix(row<1>(), row3));

  Serial << b << '\n';

  // Anchor Matrix
  p.Submatrix(row<0>(), row3) << bolt_spacing/2,hex_triangle_height, 0;

  // Rotate prototype vector to get motors 1 & 2
  p.Submatrix(row<2>(), row3) = ~(rotation * ~p.Submatrix(row<0>(), row3));
  p.Submatrix(row<1>(), row3) = ~(rotation * ~p.Submatrix(row<2>(), row3));

  // Motors 3,4, and 5 are the flipped versions of motors 0, 2, and 1 respectively
  p.Submatrix(row<3>(), row3) = ~(flip_x * ~p.Submatrix(row<0>(), row3));
  p.Submatrix(row<4>(), row3) = ~(flip_x * ~p.Submatrix(row<2>(), row3));
  p.Submatrix(row<5>(), row3) = ~(flip_x * ~p.Submatrix(row<1>(), row3));

  Serial << T << '\n';

//  float z0 = sqrt(d_avg*d_avg + h*h - (p[:,0] - b[:,0])*(p[:,0] - b[:,0]) - (p[:,1] - b[:,1])*(p[:,1] - b[:,1]));
//  T(2) = z0;
  
}

//float offset = 75.0; //(max_val + min_val)/2;
//float amplitude = 3.0; //(max_val - min_val)/2; 
int max_phase = 4000;
BLA::Matrix<N,1> ones = {1,1,1,1,1,1};
float* v = new float[3];
int dir = 1;

void loop() {
  dir *= -1;
  for (int phase = 0; phase < max_phase; phase++){
    v[0] = dir * sin(4*2*M_PI*((float) phase)/max_phase);
    v[1] = cos(4*2*M_PI*((float) phase)/max_phase);
    v[2] = 0;
//    T(2) = offset + amplitude*sin(2*M_PI*((float) phase)/max_phase);
//    Serial.println(offset + amplitude*sin(2*M_PI*phase/max_phase));
//    Serial << T(2) << '\n';
//    Serial.println(phase);
//    Serial.println(4*2*M_PI*((float) phase)/max_phase);
//    Serial.println(dir * sin(4*2*M_PI*((float) phase)/max_phase));
//    Serial.println(
    
    
    R = rot(14*M_PI/180, v);
//    Serial.println("hello1");
    
    for (int i = 0; i < N; i++){
      quaternion p_quat = quaternion(0, p(i, 0), p(i,1), p(i,2)); // p as a quaternion
      p_quat = R.rotate(p_quat);
//      Serial.println("hello2");
      p_rot(i, 0) = p_quat.get_x();
      p_rot(i, 1) = p_quat.get_y();
      p_rot(i, 2) = p_quat.get_z();
  
      //l.Submatrix(BLA::Slice<i, i+1>(), row3) = T + p_rot.Submatrix(BLA::Slice<i, i+1>(), row3) - b.Submatrix(BLA::Slice<i, i+1>(), row3);
    }
  //  
//    Serial.println("hello2");
    
//    Serial.println("hello3");
    l = ones*~T + p_rot - b;
//    Serial.println("hello4");
    
  //   ;//+ p_rot.Submatrix(BLA::Slice<i, i+1>(), row3) - b.Submatrix(BLA::Slice<i, i+1>(), row3);  
    for(int i = 0; i < N; i++){
      e = 2*h*l(i,2);
      f = 2*h*(cos(beta(i))*l(i,0) + sin(beta(i))*l(i,1));
      g = (l(i,0)*l(i,0) + l(i,1)*l(i,1) + l(i,2)*l(i,2)) - d_avg*d_avg + h*h;
      a(i) = asin(g/sqrt(e*e + f*f)) - atan2(f,e);
    }
  
//    Serial << a << '\n';
    for (int i = 0; i < N; i++){
      send_angle(i, a(i));
    }
    delay(10);
  }

} 
