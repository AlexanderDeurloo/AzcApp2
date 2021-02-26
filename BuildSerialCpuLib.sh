1# /bin/bash

echo 'build a shared lib for serial cpu communication...'

    gcc -shared -o libSerial.so Serial.c 

echo 'Done'
