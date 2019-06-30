#include <Servo.h> 
Servo servoX; //set servoX
int pinLB = 9;       // 9 drives left wheel backward
int pinLF = 11;       // 11 drives left wheel forward 

int pinRB = 6;       // 6 drives right wheel back 
int pinRF = 3;       // 3 drives right wheel forward 

int inputPin = A0;    // pin A0 defined as ultrasonic signal reception (ECHO)
int outputPin = A1;   // pin A1 defined as ultrasonic signal transmitter (TRIG)

int Fspeedd = 0;        // forward distance
int Rspeedd = 0;        // Right distance
int Lspeedd = 0;        // Left distance
int directionn = 0;     // Front Left = 8 after = 2 = 4 Right = 6 
                        
int delay_time = 250; // settling time after steering servo motors 

int Fgo = 8;           // Forward 
int Rgo = 6;           // Right 
int Lgo = 4;           // Left 
int Bgo = 2;           // Reverse 

void setup () 
 { 
 Serial.begin (9600);     // Define motor output pins 
 pinMode (pinLB, OUTPUT); // pin 9 (PWM) 
 pinMode (pinLF, OUTPUT); // pin 11 (PWM) 
 pinMode (pinRB, OUTPUT); // pin 6 (PWM) 
 pinMode (pinRF, OUTPUT); // pin 3 (PWM) 

 pinMode (inputPin, INPUT);      // Define ultrasound input pin 
 pinMode (outputPin, OUTPUT);    // Define ultrasonic output pin    


 servoX.attach (5);      // Define x axis servo, pin 5 (PWM) 
 } 
void advance (int a)       // Forward 
   { 
    
    analogWrite (pinRB, 0);    
    analogWrite (pinRF, 150);  // The motor (forward right) action
    analogWrite (pinLB, 0);    
    analogWrite (pinLF, 140);  // The motor (forward left) action 
    delay (a * 40);      
    

   } 

void turnL (int b)          // Turn right
   { 
    analogWrite (pinRB, 0);      
    analogWrite (pinRF, 200);  // The motor (forward right) action
    analogWrite (pinLB, 200);  // The motor (backward left) action
    analogWrite (pinLF, 0); 
    delay (b * 50); 
   } 
void turnR (int c)           // Turn left 
   { 
    analogWrite (pinRB, 200);  // The motor (backward right) action 
    analogWrite (pinRF, 0); 
    analogWrite (pinLB, 0);    
    analogWrite (pinLF, 200);  // The motor (forward left) action
    delay (c * 50); 
   }     
   
void stopp (int d)           // Stop 
   { 
    digitalWrite (pinRB, HIGH); 
    digitalWrite (pinRF, HIGH); 
    digitalWrite (pinLB, HIGH); 
    digitalWrite (pinLF, HIGH); 
    delay (d * 100); 
   } 
void back (int e)            // back
   { 
  
    analogWrite (pinRB, 200);    // The motor (rear right) action 
    analogWrite (pinRF, 0); 
    analogWrite (pinLB, 200);    // The motor (left rear) action 
    analogWrite (pinLF, 0); 
    delay (e * 300);      
  
   } 
  
void detection ()          // test distance of different directions 
   {       
     int delay_time = 250;   // Settling time servo motor after turning 
     ask_pin_F ();              // Read forward distance 
    
    if (Fspeedd <10)           // If the distance is less than 10 cm
     { 
     stopp (1);                 
     back (2);                  
     } 
         
     if (Fspeedd <20)           // If the distance is less than 20cm
     { 
       stopp (1);                 // Clear the output data 
       ask_pin_L ();              // Read from left 
       delay (delay_time);        // Wait for a stable servo motor 
       ask_pin_R ();              // Read from the right   
       delay (delay_time);        // Wait for a stable servo motor   
      
       if (Lspeedd> Rspeedd)     // If the distance is greater than the right from the left 
       { 
        directionn = Rgo;        // Right away 
       } 
      
       if (Lspeedd <= Rspeedd)     // If the left is less than or equal to the distance from the right 
       { 
        directionn = Lgo;        // Turn Left 
       } 
      
       if (Lspeedd <10 && Rspeedd <10)     // If the distance to the left and right are less than 10 cm distance 
       { 
        directionn = Bgo;        // To go after         
       }           
     } 
     else                        // Add as front not less than (greater than) 25 cm      
     { 
       directionn = Fgo;          // Move forward      
     } 
   
   }     
void ask_pin_F ()     // test the distance from the front 
   { 
      servoX.write (90); 
     digitalWrite (outputPin, LOW);     // Let ultrasonic transmitter low voltage 2 μ s 
     delayMicroseconds (2); 
     digitalWrite (outputPin, HIGH);    // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s 
     delayMicroseconds (10); 
     digitalWrite (outputPin, LOW);      // Maintain low voltage ultrasonic transmitter 
     float Fdistance = pulseIn (inputPin, HIGH);    // Read worse time difference 
     Fdistance = Fdistance/5.8/10;         // Time to turn to the distance (unit: cm) 
     Serial.print ("F distance:");        // Output distance (unit: cm) 
     Serial.println (Fdistance);           // Display the distance 
     Fspeedd = Fdistance;                 // Read into the distance Fspeedd (former speed) 
   }   
 void ask_pin_L ()     // Measure the distance from the left 
   { 
     servoX.write (5); 
     delay (delay_time); 
     digitalWrite (outputPin, LOW);     // Let ultrasonic transmitter low voltage 2 μ s 
     delayMicroseconds (2); 
     digitalWrite (outputPin, HIGH);    // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s 
     delayMicroseconds (10); 
     digitalWrite (outputPin, LOW);      // Maintain low voltage ultrasonic transmitter 
     float Ldistance = pulseIn (inputPin, HIGH);    // Read worse time difference 
     Ldistance = Ldistance/5.8/10;         // Time to turn to the distance (unit: cm) 
     Serial.print ("L distance:");         // Output distance (unit: cm) 
     Serial.println (Ldistance);           // Display the distance 
     Lspeedd = Ldistance;                // Read into the distance Lspeedd (left-speed) 
   }   
void ask_pin_R ()     // Measure the distance from the right 
   { 
     servoX.write (177); 
     delay (delay_time); 
     digitalWrite (outputPin, LOW);     // Let ultrasonic transmitter low voltage 2 μ s 
     delayMicroseconds (2); 
     digitalWrite (outputPin, HIGH);    // Let ultrasonic transmitter high voltage 10 μ s, where at least 10 μ s 
     delayMicroseconds (10); 
     digitalWrite (outputPin, LOW);      // Maintain low voltage ultrasonic transmitter 
     float Rdistance = pulseIn (inputPin, HIGH);    // Read worse time difference 
     Rdistance = Rdistance/5.8/10;         // Time to turn to the distance (unit: cm) 
     Serial.print ("R distance:");         // Output distance (unit: cm) 
     Serial.println (Rdistance);           // Display the distance 
     Rspeedd = Rdistance;                // Will read into the distance Rspeedd (Right-speed) 
   }   
  
void loop () 
 { 
   servoX.write (90);    // Let servo motor position ready to return to the pre-prepared next time measurement 
   detection ();          // Measure the angle and direction of judgment to where to move 
    
  if (directionn == 2)    // If directionn (direction) = 2 (reverse)              
  { 
    back (8);                      //    Retrogression (car) 
    turnL (2);                     // Move slightly to the left (to prevent stuck in dead alley) 
    Serial.print ("Reverse");     // Display direction (backwards) 
  } 
  if (directionn == 6)             // If directionn (direction) = 6 (right turn)     
  { 
    back (1); 
    turnR (6);                     // Right 
    Serial.print ("Right");      // Display direction (turn left) 
  } 
  if (directionn == 4)            // If directionn (direction) = 4 (turn left)     
  {   
    back (1);       
    turnL (6);                    // Left 
    Serial.print ("Left");       // Display direction (turn right)    
  }   
  if (directionn == 8)            // If directionn (direction) = 8 (forward)       
  { 
   advance (1);                   // Normal Forward   
   Serial.print ("Advance");     // Display direction (forward) 
   Serial.print ("     ");     
  } 
 }
 
