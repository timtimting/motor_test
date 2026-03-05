open mks canable for candleLight
`sudo ip link set can0 type can bitrate 1000000`
`sudo ifconfig can0 up`
`sudo ifconfig can0 txqueuelen 100000`

build the project
`cd ./motor_test/`
`mkdir build`
`cd build`
`camke ..`
`make`

run the programe
`./bin/motor_test`

