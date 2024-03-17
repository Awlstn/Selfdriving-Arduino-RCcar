#include <util/delay.h>
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(0,1);
//지연 기능을 사용

#define MOTOR_A_a 3        //모터A의 +출력핀은 3번핀입니다
#define MOTOR_A_b 11      //모터A의 -출력핀은 11번핀입니다
#define MOTOR_B_a 5       //모터B의 +출력핀은 5번핀입니다
#define MOTOR_B_b 6       //모터B의 -출력핀은 6번핀입니다
#define MOTOR_SPEED 200
#define MOTOR_SPEEDA 180   
#define TRIG1 2          
#define ECHO1 4 
#define TRIG2 13           
#define ECHO2 12            
unsigned char m_a_spd = 0, m_b_spd = 0;       //모터의 속력을 저장하는 전역변수
boolean m_a_dir = 0, m_b_dir = 0;





#define ALPHA 0.1       //학습률, 0: 새로운 것이 학습되지 않고 로봇이 과거의 경험에 의존한다는 것을 의미하며, 1: 로봇이 항상 학습하고 과거의 경험에 의존하지 않는다는 것을 의미합니다.
#define GAMMA 0.5      //할인 요인(시간별 보상 감소)
#define episodes 150      //반복 횟수
#define states 10           //환경에 있는 상태의 수는 기본적으로 2이어야 합니다(장애물이 없거나 2). 그러나 우리는 더 복잡한 환경을 위해 10개를 만들었습니다.
#define number_of_actions 4     //총 작업 수(전진, 정지, 좋은선택(오른쪽,왼쪽), 안좋은선택(오른쪽, 왼쪽))

uint8_t Obstacle = 0;        //장애물이 있는지 없는지를 표시하다
int FLAG;            //로봇이 의사 결정 과정에서 얼마나 멀리 있는지 확인하기 위해 0: 아직 아무것도 하지 않았고, 1: 장애물을 감지하여 다음 상태로 이동했으며, 이전 Q-table을 기준으로 탐색(랜덤 액션 선택)할지 또는 활용할지 결정하기 시작합니다. 2: 탐색을 선택하면 행동에 따라 보상이 주어집니다.
int reward;           //어떤 행동을 한 것에 대한 보상
int state = 0;           // 로봇의 현재 상태
int action = 0;          //로봇에 의해 수행된 작업, 색인에 따른 숫자(0: 앞으로, 1: 정지, 2: 좋은선택(오른쪽,왼쪽), 3: 안좋은선택(오른쪽, 왼쪽) )
float prob;           //엡실론 붕괴에 사용되며, 시간이 지나면 로봇이 더 적게 탐색하고 더 많이 이용하기 시작합니다(기존 지식에 따라 행동).
uint8_t action_taken = 0;      //어떤 조치가 취해지는지 여부를 판단하다
int next_state;           //다음 상태 또는 로봇
int actions[4] = {1, 2, 3, 4};        //작업 값(1: 앞으로, 2: 정지, 3: 좋은선택(오른쪽,왼쪽), 4: 안좋은선택(오른쪽, 왼쪽))
float EPSILON = 0.90;       //탐사율


//Q-table, 실제로 장애물 회피 로봇은 두 가지 상태만 있습니다. 1: 장애물로부터 떨어져 있을 때(장애물 없음), 2: 장애물 근처에 있을 때(장애물)여기서 우리는 더 정확한 학습 과정을 위해 더 복잡한 환경을 가정한 10개의 상태를 코딩했다. (그러나 이것은 학습 과정을 더 느리게 만든다.)
//여기서 우리는 더 정확한 학습 과정을 위해 더 복잡한 환경을 가정한 10개의 상태를 코딩했다. (그러나 이것은 학습 과정을 더 느리게 만든다.)


//Q-table은 상태를 행으로, 열을 작업 수로 나타냅니다.
//로봇 작업에 따른 보상으로 Q-table이 업데이트됩니다.


float Q[states][number_of_actions] = {{0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}, 
                                      {0.0, 0.0, 0.0, 0.0}};


//(-5 : 앞으로, 5 : 정지, 25 : 좋은선택(오른쪽,왼쪽), -15 : 안좋은선택(오른쪽, 왼쪽)
float rewards[states][number_of_actions] = {{-5, 5, 25, -15,},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15},
                                          {-5, 5, 25, -15}};

//Q-러닝 업데이트 매개 변수
float Q_old, Q_new, Q_max;

//장애물이 있는지 없는지를 결정하는 기능
//getDistance() 함수를 사용하려면 초음파 헤더 파일을 가져와야 합니다.

uint8_t obstacle_avoider()
{
  uint16_t distance1 = getDistance1();
  uint16_t distance2 = getDistance2();
  if(distance1 <= 10 || distance2 <= 10)
  {
    Obstacle = 1;
  }
  else
  {
    Obstacle = 0; 
  }
 
  
  _delay_ms(10);
  
  return Obstacle;
}

//엡실론(분리 파라미터)을 시간과 함께 다시 사용하는 함수입니다. 마지막에 당신이 엡실론을 제거하고 로봇은 과거의 경험을 바탕으로 장애물을 피하는 법을 배운다.

float decay(float epsilon)
{
  epsilon =  epsilon* 0.98;  
  return epsilon;
}

int get_state()
{
  int state_number;
  state_number = random(0, 10);
  return state_number;
}

//Q_table에서 가장 큰 수를 찾는 함수


float MAX(float Q_table[][4], int NEXT_S)
{
  float LIST[4];
  float N1, N2, MAX_value = 0.0, DIFF;
  
 
  for (int b = 0; b <= 3; b++)
  {
     //make list from given row in table
    LIST[b] = Q[NEXT_S][b];
  }

  for (int j = 0; j <= 2; j++)
  {
    //compare MAX_value = 0.0 with each item in list
    if (MAX_value > LIST[j])
    {

      N1 = MAX_value;
    }
    else
    {
      N1 = LIST[j];
    }
    //compare list items with each other to get biggest value of them
    N2 = LIST[j + 1];
    DIFF = N1 - N2;

    if (DIFF > 0)
    {

      MAX_value = N1;
    }
    else
    {
      MAX_value = N2;
    }
  }
      return MAX_value;
}


//Q 테이블[상태]에서 가장 큰 Q 값의 지수를 찾는 함수


 int ARGMAX(float Q_table[][4], int NEXT_S)
 {

  float array[4];
  float N1, N2, MAX_value = 0.0, DIFF, NUMBER;
  int MAX_index;

  for (int u = 0; u <= 3; u++)
  {

    array[u] = Q_table[NEXT_S][u];
  }

  for (int p = 0; p <= 2; p++)
  {
    if (MAX_value > array[p])
    {
      N1 = MAX_value;
    }
    else
    {
      N1 = array[p];
    }

    N2 = array[p + 1];
    DIFF = N1 - N2;

    if (DIFF > 0)
    {
      MAX_value = N1;
    }

    else
    {
      MAX_value = N2;
    }
  }

  for (int r = 0; r <= 3; r++)
  {
    NUMBER = array[r];
    if (NUMBER == MAX_value)
    {
      MAX_index = r;
      break;
    }
  }
  return MAX_index;
 }

 //시간 차이 학습 접근법을 사용하여 BellMan 방정식을 기반으로 Q_table 및 Q_value를 업데이트하는 함수

 void update(float Q_table[][4], int S, int NEXT_S, int A, int actions[], int R, float learning_rate, float discount_factor)
 {

  Q_old = Q_table[S][A];
  Q_max = MAX(Q_table, NEXT_S);
  Q_new = (1 - learning_rate) * Q_old + learning_rate * (R + discount_factor * Q_max);
  Serial.print("q-new : ");
  Serial.println(Q_new);
  Q_table[S][A] = Q_new;
 }

 //탐색 또는 활용 여부를 결정할 난수 선택

 float random_no(float EXPLORATION_PARAMETER)
 {
  float RANDOM_VARIABLE;
  float PROBABILITY;

  RANDOM_VARIABLE = random(0, 100);  
  PROBABILITY = RANDOM_VARIABLE / 100;

  return PROBABILITY;
 }
 int getDistance1()
 {
  digitalWrite(TRIG1, LOW);             // Trig 핀 Low
  delayMicroseconds(2);              // 2us 딜레이
  digitalWrite(TRIG1, HIGH);          // Trig 핀 High
  delayMicroseconds(10);            // 10us 딜레이
  digitalWrite(TRIG1, LOW);    
  int distance = pulseIn(ECHO1, HIGH)/29/2;
  Serial.print("Distance1 cm : ");
  Serial.print(distance);
  //BTSerial.println(" ");
  delay(10);
  return distance;
 }
 int getDistance2()
 {
  digitalWrite(TRIG2, LOW);             // Trig 핀 Low
  delayMicroseconds(2);              // 2us 딜레이
  digitalWrite(TRIG2, HIGH);          // Trig 핀 High
  delayMicroseconds(10);            // 10us 딜레이
  digitalWrite(TRIG2, LOW);    
  int distance = pulseIn(ECHO2, HIGH)/29/2;
  Serial.print(" Distance2 cm : ");
  Serial.println(distance);
  //BTSerial.println(" ");
  delay(10);
  return distance;
 }
void forward()
{
    m_a_dir = 0;          //모터A 정방향
    m_b_dir = 0;           //모터B 정방향
    m_a_spd = MOTOR_SPEEDA;    //모터A의 속력값 조정
    m_b_spd = MOTOR_SPEED;  
    motor_drive();
}
void backward()
{
    m_a_dir = 1;          //모터A 역방향
    m_b_dir = 1;          //모터B 역방향
    m_a_spd = MOTOR_SPEEDA;     //모터A의 속력값 조정
    m_b_spd = MOTOR_SPEED;  
    motor_drive();
}
void stop()
{
    m_a_dir = 0;        //모터A 정방향
    m_b_dir = 0;        //모터B 정방향
    m_a_spd = 0;        //모터A의 정지
    m_b_spd = 0;        //모터B의 정지
    motor_drive();
}
void left()
{
    m_a_dir = 1;           //모터A 역방향
    m_b_dir = 0;           //모터B 정방향
    m_a_spd = MOTOR_SPEEDA;     //모터A의 속력값 조정
    m_b_spd = MOTOR_SPEED;
    motor_drive(); 
    delay(450);         //모터B의 속력값 조정
}
void right() {
  // 왼쪽 모터
    m_a_dir = 0;           //모터A 정방향
    m_b_dir = 1;           //모터B 역방향
    m_a_spd = MOTOR_SPEEDA;     //모터A의 속력값 조정
    m_b_spd = MOTOR_SPEED;
    motor_drive();
    delay(450);         //모터B의 속력값 조정
}
void motor_drive()        //모터를 구동하는 함수
{
  if(m_a_dir == 0)
  {
    digitalWrite(MOTOR_A_a, LOW);           //모터A+ LOW
    analogWrite(MOTOR_A_b, m_a_spd);     //모터A-의 속력을 PWM 출력
  }
  else
  {
    analogWrite(MOTOR_A_a, m_a_spd);     //모터A+의 속력을 PWM 출력
    digitalWrite(MOTOR_A_b, LOW);         //모터A- LOW
  }
  if(m_b_dir == 1)
  {
    digitalWrite(MOTOR_B_a, LOW);           //모터B+ LOW
    analogWrite(MOTOR_B_b, m_b_spd);        //모터B-의 속력을 PWM 출력
  }
  else
  {
    analogWrite(MOTOR_B_a, m_b_spd);     //모터B+의 속력을 PWM 출력
    digitalWrite(MOTOR_B_b, LOW);          //모터B- LOW
  }
}
void setup()
{
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(MOTOR_A_a, OUTPUT);
  pinMode(MOTOR_A_b, OUTPUT);
  pinMode(MOTOR_B_a, OUTPUT);
  pinMode(MOTOR_B_b, OUTPUT);
  delay(1000); 
  Serial.begin(9600);
  BTSerial.begin(9600);
  randomSeed(analogRead(0));
}

 void loop ()
 {
   if (BTSerial.available()>0) // 블루투스로 음성인식 신호 받고 저장
   {
    char data = BTSerial.read();
    if(data == '1')
    {
      for (int I = 0; I < episodes; I++) // 출발을 인식하면 강화학습 시작
     {
      Serial.print("START : ");
      Serial.println(I);
   
      action_taken = 0;
      FLAG = 0;

      while (1)
      {
        forward();
        Obstacle = obstacle_avoider();
        if (Obstacle == 1)
        {
          next_state = state + 1;

          if (next_state == 10)
          {
            next_state = 0;
          }
          else if (next_state < 0)
          {
            next_state = 0;
          }
          FLAG = 1;
          break;
        }
      }

      if (FLAG == 1)
      {
        prob = random_no(EPSILON);//choose a random value within exploration rate, to determine whether to explore or exploit
        Serial.print("prob : ");
        Serial.print(prob);
        Serial.print(" ");
        Serial.print("EPSILON : ");
        Serial.println(EPSILON);
        if (prob <= EPSILON)  //explore the actions
        {
          Serial.println("random");
          action = random(0, 4);
          FLAG = 2;
        }
        else          //Q_table의 액션을 이용하다
        {
          Serial.println("ARGMAX");
          action = ARGMAX(Q, state);
          FLAG = 2;
        }
      }

      if (FLAG == 2)
      {
        if (action == 0)
        {
          Serial.println("forward");
          forward();
          _delay_ms(1000);
          backward();
          delay(300);
          stop();
          reward = rewards[state][action];
          Serial.print("reward : ");
          Serial.println(reward);
        }
        if (action == 1)
        {
          stop();
          Serial.println("stop");
          reward = rewards[state][action];
          Serial.print("reward : ");
          Serial.println(reward);
        }
        if (action == 2)
        {
          if(getDistance1()<getDistance2())
          {
          right();
          stop();
          Serial.println("right!!!!!!!!");
          reward = rewards[state][action];
          Serial.print("reward : ");
          Serial.println(reward);
          }
         else if(getDistance1()>getDistance2())
          {
          left();
          stop();
          Serial.println("left!!!!!!!!");
          reward = rewards[state][action];
          Serial.print("reward : ");
          Serial.println(reward);
        }
        }
        if (action == 3)
        {
          if(getDistance1()>getDistance2())
          {         
          right();
          stop();
          Serial.println("right XXX");
          reward = rewards[state][action];
          Serial.print("reward : ");
          Serial.println(reward);
          }
          else if(getDistance1()<getDistance2())
          {       
          left();
          stop();
          Serial.println("left XXX");
          reward = rewards[state][action];
          Serial.print("reward : ");
          Serial.println(reward);
          }
        }

        action_taken = 1;
        _delay_ms(500);
      }

      if (action_taken == 1)
      {
        //작업이 수행된 경우 수행된 작업에 따라 Q-table을 새 값으로 업데이트합니다.

        update(Q, state, next_state, action, actions, reward, ALPHA, GAMMA);
        state = next_state;
        EPSILON = decay(EPSILON); //decay epsilon to decrease exploration rate by time
        if (EPSILON < 0.5)
        {
          EPSILON = 0.9;
        }
        _delay_ms(1000);
      }
      for(int i=0;i<10;i++)
      {
        for(int j=0;j<4;j++)
        {
          Serial.print(Q[i][j]);
          Serial.print(" ");
        }
        Serial.println("");
      }
     }
     // 강화 학습 완료 후 테스트 주행 시작
     while (1)
     {
      if (BTSerial.available()>0) 
       {
          char data = BTSerial.read();
          if(data == '2')
          {
             Serial.println("car stop(60sec)");
             stop();
             delay(60000);
          }
       }
      forward();
      Obstacle = obstacle_avoider();
      if (Obstacle == 1)
      {
        state = get_state();
        action = ARGMAX(Q, state);
        if (action == 0)
        {
          forward();
          _delay_ms(1000);
          backward();
          delay(500);
          Serial.println("forward");
        }
        if (action == 1)
        {
          stop();
          Serial.println("stop");
        }
        if (action == 2)
        {
          if(getDistance1()<getDistance2())
          {
          right();
          stop();
          Serial.println("right!!!!!!!!");
          
          }
         else if(getDistance1()>getDistance2())
          {
          left();
          stop();
          Serial.println("left!!!!!!!!");
          
        }
        }
        if (action == 3)
        {
          if(getDistance1()>getDistance2())
          {
          right();
          stop();
          
          reward = rewards[state][action];
          Serial.println("right XXX");
          }
         else if(getDistance1()<getDistance2())
          {
          left();
          stop();
          
          reward = rewards[state][action];
          Serial.println("left XXX");
          }
        }
      }
       
    }
   }
  
   }
  }
    
 
