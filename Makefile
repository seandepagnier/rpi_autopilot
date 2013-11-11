CFLAGS = -g -Wall -I imu -I servo -I misc
LDFLAGS = -lm -lpthread

OBJS = \
	examples/c/autopilot.o \
	servo/servo.o \
	imu/config.o \
	imu/quaternion.o \
	imu/imu.o \
	imu/MadgwickAHRS/MadgwickAHRS.o \
	imu/rotate.o \
	imu/vector.o \
	imu/matrix.o \
	imu/MahonyAHRS/MahonyAHRS.o \
	imu/calibration.o

.PHONY: all clean copy test run runnc

%.o : %.c *.h
	$(CC) $(CFLAGS) -c $< -o $@

all: autopilot

autopilot: $(OBJS)
	gcc -o autopilot $(OBJS) $(LDFLAGS)

clean:
	rm -f autopilot

copy:
	scp -pr 10.42.0.1:initial/*.c 10.42.0.1:initial/*.h 10.42.0.1:initial/Makefile .

test: copy autopilot

run: copy autopilot
	echo "running"; ./autopilot

runnc: copy autopilot
	echo "running"; ./autopilot | nc -l 12345
