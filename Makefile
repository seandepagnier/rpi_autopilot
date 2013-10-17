CFLAGS = -g -Wall
OBJS = autopilot.o MadgwickAHRS/MadgwickAHRS.o MahonyAHRS/MahonyAHRS.o quaternion.o vector.o rotate.o matrix.o calibration.o

.PHONY: all clean copy test run runnc

%.o : %.c *.h
	$(CC) $(CFLAGS) -c $< -o $@

all: autopilot

autopilot: $(OBJS)
	gcc -o autopilot $(OBJS) -lm

clean:
	rm -f autopilot

copy:
	scp -pr 10.42.0.1:initial/*.c 10.42.0.1:initial/*.h 10.42.0.1:initial/Makefile .

test: copy autopilot

run: copy autopilot
	echo "running"; ./autopilot

runnc: copy autopilot
	echo "running"; ./autopilot | nc -l 12345
