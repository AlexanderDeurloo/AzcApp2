!# /bin/bash

echo 'Build RevPi lib...'
	gcc -shared -o libpiControlIf.so piControlIf.c
echo 'Done...'
