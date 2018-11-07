
#include <Servo.h>

//////////////////////////////////////////////////////////////////////
// Hardware Setup
//////////////////////////////////////////////////////////////////////
#define AZIMUTH_PIN 11
#define ALTITUDE_PIN 10
#define PIR_PIN 2
#define LASER_PIN 3

//////////////////////////////////////////////////////////////////////
// Field of Play
//////////////////////////////////////////////////////////////////////
// Set up some hard limits for servo motion. Absolute limits are more
// likes 540 and 2500, but these narrower 
#define MIN_COORD 800
#define MAX_COORD 2000

float clip(float x) {
  if( x<MIN_COORD) {
    return MIN_COORD;
  }
  if( x>MAX_COORD) {
    return MAX_COORD;
  }
  return x;
}

const unsigned int NUMBER_OF_MODES = 5;

__attribute__((section(".noinit"))) unsigned int mode;
__attribute__((section(".noinit"))) unsigned int maxX;
__attribute__((section(".noinit"))) unsigned int maxY;
__attribute__((section(".noinit"))) unsigned int minX;
__attribute__((section(".noinit"))) unsigned int minY;


//////////////////////////////////////////////////////////////////////
// Hermite polynomials for cubic splines
//////////////////////////////////////////////////////////////////////
float h00[] = {1,0,-3, 2}; 
float h10[] = {0,1,-2,1};
float h01[] = {0,0,3,-2};
float h11[] = {0,0,-1,1};

class Hermite {
  float p[4];
  float x;
  float m;

  public:
  void setup(float x0, float m0, float x1, float m1);
  void nextPoint(float x1, float m1);
  float value(float t);
};

void Hermite::setup(float x0, float m0, float x1, float m1) 
{
  for(int i=0; i<4; ++i) {
    p[i] = h00[i] * x0 + h10[i] * m0 
          + h01[i] * x1 + h11[i] * m1;
  }
  x = x1;
  m = m1;
}

void Hermite::nextPoint(float x1, float m1) {
  setup(x,m,x1,m1);
}

float Hermite::value(float t) {
  return ((p[3]*t + p[2]) * t + p[1]) * t + p[0];
}


//////////////////////////////////////////////////////////////////////
// Motion engine
//////////////////////////////////////////////////////////////////////
class DotMover {
  private:
  Servo azmServo;
  Servo altServo;
  int laserPin;
  
  bool moving;
  
  Hermite xh;
  Hermite yh;

  float x[16];
  float y[16];
  int flags[16];
  bool laserOn;
  
  float lastx;
  float lasty;
  
  int i;
  int top;
  
  float t;
  float s;
  float sf;
  float sp;

  int points = 0;
    
  public:
  void setup(int azm, int alt, int laser);
  void startMoving();
  void stopMoving();
  
  void push(float x, float y, bool laser, bool corner);
  void pop();
  
  void tick();

  void agitate();
  void settleDown();
  int getPoints() { return points; }
  void setSpeed(float speed);

  void (* more)();
  getMorePoints( void (*m) ()) { more = m; }
};

void 
DotMover::setup(int azm, int alt, int laser) {
  // initialize servos
  azmServo.attach(azm);
  altServo.attach(alt);
  
  // set up laser
  laserPin = laser;
  pinMode(laserPin, OUTPUT);

  i=0; top=0;
  t = 0;
  s = 0.01;
  sf = 1.0;
  sp = 1.0;
  
  // Initialize the splines to be stationary
  xh.setup((maxX + minX)/2, 0, (maxX + minX)/2, 0);
  yh.setup((maxY + minY)/2, 0, (maxY + minY)/2, 0);

  lastx = (maxX + minX) / 2;
  lasty = (maxY + minY) / 2;
  laserOn = false;
  
  push(lastx, lasty, true, false);
  push(lastx, lasty, true, false);
}

void DotMover::startMoving() {
  moving = true;
}

void DotMover::stopMoving() {
  moving = false;
}

void DotMover::push(float xin, float yin, bool laserOn=true, bool corner =false) {
  x[top] = clip(xin);
  y[top] = clip(yin);

  flags[top] = (laserOn ? 0x01 : 0x00) | (corner ? 0x02 : 0x00);
  
  top = (top +1) % 16;
  points += 1;
}

void DotMover::pop() {
  if(points < 1) {
    stopMoving();
    return;
  }
  
  float xn = x[i];
  float yn = y[i];

  i = (i+1) %16;
  points -= 1;


  laserOn = flags[i] & 0x01;
  sf = (flags[i] & 0x02) ? 0.0 : 1.0;

  int j = i;
  if(points > 0) {
    (i+1)%16;
    xh.nextPoint(xn, sf * (x[j] - lastx)/2);
    yh.nextPoint(yn, sf * (y[j] - lasty)/2);
  } else {
    xh.nextPoint(xn,0);
    yh.nextPoint(yn,0);
  }
  

  lastx = xn;
  lasty = yn;

  if(points < 1 && more != NULL) {
    (*more)();
  }
 }

void DotMover::setSpeed(float speed) {
  sp = speed;
  if(sp > 5) {
    sp = 5;
  }
}

void DotMover::tick() {
  if(!moving) {
    digitalWrite(laserPin, 0);
    return;
  }
  
  digitalWrite(laserPin, laserOn);
  azmServo.writeMicroseconds(clip(xh.value(t)));
  altServo.writeMicroseconds(clip(yh.value(t)));
  t += s * sp; 
  
  if(t >= 1) {
    t -= 1;
    pop();
  }
}

DotMover dm;


//////////////////////////////////////////////////////////////////////
// Movment Detector
//////////////////////////////////////////////////////////////////////
class MotionDetector {
  private:
  int counter;
  bool agitated;
    
  public:
  
  float excitement;
  void setup();
  void tick();
  void agitate();
};

void
MotionDetector::setup() {
  excitement = 3.0;
  counter = 0;
  agitated = false;
}

void MotionDetector::tick() {
  if(agitated) {
    excitement += 1.0;
    Serial.print("excitement "); Serial.print(excitement); Serial.print("\n");
  }
  
  agitated = false;

  counter++;
  if(counter >= 100) {
    excitement *= 0.99;
    Serial.print("excitement "); Serial.print(excitement); Serial.print("\n");
    counter = 0;    
  }
}

void MotionDetector::agitate() {
  agitated = true;
}

MotionDetector detector;



bool firstPass = true;
void resetLimits() {
  maxX = maxY = minX = minY = 1500;
}

void chooseMaxX() {
  if(firstPass) {
    maxX += 100;
    if(maxX >= MAX_COORD) {
      maxX = MAX_COORD;
      firstPass = false;
    }
  } else {
    maxX -= 100;
    if(maxX <= MIN_COORD) {
      maxX = MIN_COORD;
      firstPass = true;
    }
  }
  minX = maxX;
  dm.push(maxX, maxY, true, true);
  dm.push(maxX, maxY, true, true);
  dm.push(maxX, maxY, true, true);
  Serial.print("maxX "); Serial.print(maxX); Serial.print("\n");
}

void chooseMaxY() {
  if(!firstPass) {
    maxY += 100;
    if(maxY >= MAX_COORD) {
      maxY = MAX_COORD;
      firstPass = !firstPass;
    }
  } else {
    maxY -= 100;
    if(maxY <= MIN_COORD) {
      maxY = MIN_COORD;
      firstPass = !firstPass;
    }
  }
  minY = maxY;
  dm.push(maxX, maxY, true, true);
  dm.push(maxX, maxY, true, true);
  dm.push(maxX, maxY, true, true);
  Serial.print("maxY "); Serial.print(maxY); Serial.print("\n");
}

void chooseMinX() {
  if(firstPass) {
    minX -= 100;
    
    if(minX <= MIN_COORD) {
      minX = MIN_COORD;
      firstPass = false;
    }
  } else {
    minX += 100;
    if(minX >= maxX) {
      minX = maxX;
      firstPass = true;
    }
  }
  dm.push(minX, minY, true, true);
  dm.push(minX, minY, true, true);
  dm.push(minX, minY, true, true);
  Serial.print("minX "); Serial.print(minX); Serial.print("\n");
}

void chooseMinY() {
  if(firstPass) {
    minY -= 100;
    if(minY <= MIN_COORD) {
      minY = MIN_COORD;
      firstPass = false;
    }
  } else {
    minY += 100;
    if(minY >= maxY) {
      minY = maxY;
      firstPass = true;
    }
  }
  dm.push(minX, minY, true, true);
  dm.push(minX, minY, true, true);
  dm.push(minX, minY, true, true);
  Serial.print("minY "); Serial.print(minY); Serial.print("\n");
}

int lastx=0,lasty=0;

void playModeGetMorePoints() {
  if(!lastx) {
      lastx = random(minX,maxX);
      lasty = random(minY,maxY);    
  }

  for(int i=0; i<10; ++i) {
    switch(random(0,10)) {
      case 0:
        break;
        
      case 1:
      case 2:
      case 3:
        lastx = lastx - 20 + random(0,40);
        lasty = lasty - 20 + random(0,40);
        break;
        
      default:
        lastx = random(minX,maxX);
        lasty = random(minY,maxY);
        break;          
    }
    dm.push(lastx, lasty, true);
  }
}

bool sleepMode = false;

void setup() {
  // At startup, switch to the next mode.
  //if (++mode >= 6) mode = 0;
  mode = 5;
  maxX = 1700;
  maxY = 1900;
  minX = 1100;
  minY = 1500;

  detector.setup();
  
  Serial.begin(9600);
  Serial.print("Current mode: ");
  Serial.println(mode);
  pinMode(PIR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), motion_detected, RISING);

  randomSeed(analogRead(0));
  
  dm.setup(AZIMUTH_PIN, ALTITUDE_PIN, LASER_PIN);
  dm.startMoving();

  switch(mode) {
    case 0:
      resetLimits();
      dm.getMorePoints(chooseMaxX);
      break;
      
    case 1:
      dm.getMorePoints(chooseMaxY);
      break;
      
    case 2:
      dm.getMorePoints(chooseMinX);
      break;
      
    case 3:
      dm.getMorePoints(chooseMinY);
      break;
      
    case 4:
      Serial.print("maxX "); Serial.print(maxX); Serial.print("\n");
      Serial.print("maxY "); Serial.print(maxY); Serial.print("\n");
      Serial.print("minX "); Serial.print(minX); Serial.print("\n");
      Serial.print("minY "); Serial.print(minY); Serial.print("\n");

      dm.push(minX, minY, true, true);
      dm.push(minX, maxY, true, true);
      dm.push(maxX, maxY, true, true);
      dm.push(maxX, minY, true, true);
      dm.push(minX, minY, true, true);
      dm.push(minX, maxY, true, true);
      dm.push(maxX, maxY, true, true);
      dm.push(maxX, minY, true, true);
      break;
      
     case 5:
      dm.getMorePoints(playModeGetMorePoints);
      sleepMode = true;
      break;
  }
}

void motion_detected() {
  detector.agitate();
}

void loop() {
  digitalWrite(LED_BUILTIN, digitalRead(PIR_PIN));
  
  detector.tick();

  
  if(sleepMode) {
    dm.setSpeed(0.5 + detector.excitement / 5);

    if(detector.excitement < 0.1) {
      dm.stopMoving(); 
    } 
    if(detector.excitement > 0.1) {
      dm.startMoving();
    }
  }
  
  dm.tick();
    
  delay(10);  
}
