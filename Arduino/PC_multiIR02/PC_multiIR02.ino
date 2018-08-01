#include <IRremote.h>

// To support more than 5 receivers, remember to change the define
// IR_PARAMS_MAX in IRremoteInt.h as well.
#define RECEIVERS 3

IRrecv *irrecvs[RECEIVERS];

char* IR_dict[] = {"c952508a", "ef1f150a", "5b2f91b5", "989d243a", "cc36ff5f"};

int IR1_signal[6] = {0,0,0,0,0,0};
int IR2_signal[6] = {0,0,0,0,0,0};
int IR3_signal[6] = {0,0,0,0,0,0};


int IR_front[6] = {1,1,1,0,0,0};
int IR_back[6] = {0,0,0,1,1,0};
int IR_back2[6] = {1,0,0,1,1,0};

bool paired = false;
bool isPrintSignal = false;

int timeInst = 1;
long prevTime = 0;
decode_results results;

int positionInterval = 1000; // unit: ms

void setup()
{
  Serial.begin(9600);
  irrecvs[0] = new IRrecv(11); // Receiver #1
  irrecvs[1] = new IRrecv(12); // Receiver #2
  irrecvs[2] = new IRrecv(13); // Receiver #3

  for (int i = 0; i < RECEIVERS; i++)
    irrecvs[i]->enableIRIn();

  prevTime = millis();
}

void loop() {
  
  if (millis()- prevTime < positionInterval){
    for (int i = 0; i < 3; i++)
    {
      if (irrecvs[i]->decode(&results))
      {
        //Serial.print("Receiver #");
        //Serial.print(i);
        //Serial.print(":");
        //Serial.println(results.value, HEX);
        
        String valueStr = String(results.value, HEX);
        //Serial.println(valueStr);
        paired = false;
        for (int dictidx = 0; dictidx < 5; dictidx ++){
          if (valueStr == IR_dict[dictidx]){
            paired = true;
            if (i == 0){
              IR1_signal[dictidx] = 1;
            }else if (i == 1){
              IR2_signal[dictidx] = 1;
            }else if (i == 2){
              IR3_signal[dictidx] = 1;
            }
          }
        }
        if (paired == false){
          if (i == 0){
              IR1_signal[5] = 1;
            }else if (i == 1){
              IR2_signal[5] = 1;
            }else if (i == 2){
              IR3_signal[5] = 1;
          }
        }
        irrecvs[i]->resume();
      }
    }
    
  }else{


    if (isPrintSignal){
      
      Serial.print("Time: ");
      Serial.println(timeInst);
      for (int iridx = 0; iridx < 3; iridx++){
        Serial.print("Receiver ");
        Serial.print(iridx+1);
        Serial.print(" :[");
        for (int idx = 0; idx < 6; idx++){
          if (iridx == 0){
            Serial.print(IR1_signal[idx]);
          }else if (iridx == 1){
            Serial.print(IR2_signal[idx]);
          }else if (iridx == 2){
            Serial.print(IR3_signal[idx]);
          }
          Serial.print(" ,");
        }
        Serial.print("]  ");
      }
      Serial.println();
    
    }
    
    timeInst = timeInst+1;
    prevTime = millis();


    if (array_cmp(IR1_signal, IR_front,6) && array_cmp(IR3_signal, IR_back,6)){
      if (!array_cmp(IR2_signal, IR_front,6) && !array_cmp(IR2_signal, IR_back,6)){
        Serial.print("Docking Ready - Robot move in");
      }else if (array_cmp(IR2_signal, IR_front,6)){
        Serial.print("Docking minor adjustment required - Move backwards.");
      }else if (array_cmp(IR2_signal, IR_back,6)){
        Serial.print("Docking minor adjustment required - Move forward.");
      }
    }else if (array_cmp(IR1_signal, IR_front,6) && array_cmp(IR3_signal, IR_back2,6)){
       Serial.print("Docking variance");
    }else if (array_cmp(IR1_signal, IR_back,6)){
       Serial.print("Docking station nearby - Move forward.");
    }else if (array_cmp(IR3_signal, IR_front,6)){
       Serial.print("Docking station nearby -  Move backwards.");
    }
    Serial.println();
    Serial.println("=======================");

    for (int dictidx = 0; dictidx < 6; dictidx ++){
        IR1_signal[dictidx] = 0;
        IR2_signal[dictidx] = 0;
        IR3_signal[dictidx] = 0;
    }
  }
}


boolean array_cmp(int *a, int *b, int len){
     int n;

     // test each element to be the same. if not, return false
     for (n=0;n<len;n++) if (a[n]!=b[n]) return false;
}
