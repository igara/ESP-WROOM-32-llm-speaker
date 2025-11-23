#include <Arduino.h>
#include "AquesTalkTTS.h"

void setup()
{
  int iret;
  Serial.begin(115200);

  if (TTS.create())
  {
    Serial.println("ERR:TTS.create()");
  }
  TTS.setVolume(128); // 音量設定 0-255(max,default)

  TTS.play("akue_suto'-_ku/kido-shima'_shita.akue_suto'-_ku/kido-shima'_shita.akue_suto'-_ku/kido-shima'_shita.akue_suto'-_ku/kido-shima'_shita.");
  Serial.println("Try sending any Onsei-Kigou(UTF8/CR) from serial port");
}

void loop()
{
  if (Serial.available())
  {
    int iret;
    char kstr[256];
    size_t len = Serial.readBytesUntil('\r', kstr, 256);
    kstr[len] = 0;

    iret = TTS.play(kstr, 100);
    if (iret)
    {
      Serial.print("ERR:TTS.play()=");
      Serial.println(iret);
    }
  }
  delay(1000);
}
