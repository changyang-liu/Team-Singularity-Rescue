
#include <Arduino.h>
#include <math.h>
#include <IR.h>

SharpIR::SharpIR(int ir_pin)
{
  ir = ir_pin;
}



double SharpIR::distance() {
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

