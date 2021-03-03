#! /bin/bash

echo 'build a shared lib for serial cpu + stirrer communication...'

    gcc -shared -o libSerial.so Serial.c 
    gcc -shared -o libStirrer.so Stirrer.c 

echo 'Done'
