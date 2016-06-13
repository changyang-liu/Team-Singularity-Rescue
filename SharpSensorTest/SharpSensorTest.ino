//#include <SharpIR.h>
#include <math.h>

#define ir A8

const double e =  2.71828;

int model = 1080;
int delayTime = 0.005;
int samples =500;
int maxDiff = 17;
int i;

  double total = 0;
  double prev;
  double ave;
    double dist;
    double raw;

void setup(){
  
  Serial.begin(9600);
//  long overall = 0;
//  for (int j=1;j<=200;j++) {
//    float total = 0;
//    float prev;
//    float ave;
//    for (int i=1;i<=samples;i++) {
//      float raw = analogRead(ir);
//      if (i != 1) {
//       if (abs(raw - prev) > maxDiff) {
//         if (raw > prev) {
//           total += ave;
//           ave = total/i;
//         } else {
//            total = total - prev + 2*raw;
//           ave = total/i;
//         }
//       } else {
//          total += raw;
//          ave = total/i;
//       }
//      } else {
//        total += raw;
//       ave = total/i;
//      }
//     prev = raw;
//     delay(delayTime);
//    }
//    overall += ave;  
//  }
//
//  Serial.println(overall/200);
// 
// 

}





void loop(){

  Serial.println(distance());
}


double distance() {
    total = 0;
    prev = 0;
    ave = 0;
    delay(delayTime);
  for (i=1;i<=samples;i++) {
    raw = analogRead(ir);
    if (i != 1) {
      if (abs(raw - prev) > maxDiff) {
        if (raw > prev) {
          total += ave;
          ave = total/i;
        } else {
          total = total - prev + 2*raw;
          ave = total/i;
        }
      } else {
        total += raw;
        ave = total/i;
      }
    } else {
      total += raw;
      ave = total/i;
    }
    prev = raw;
    delay(delayTime);
  }
//  Serial.println(ave);
//

  if (model == 1080) {
    dist = pow(e,(log(ave)-log(3237))/-0.837);
  } else if (model == 20150) {
    if (ave <= 280) {
      dist = pow(e,(log(ave)-log(6755.6))/-0.823);
    } else {
      dist = (log(ave)-log(883.91))/-0.024;
    }
  }
  return dist;
}
