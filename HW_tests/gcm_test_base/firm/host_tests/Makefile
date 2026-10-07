CC ?= gcc
CFLAGS = -Wall -Wextra -std=gnu11 -O1 -g

run: test_host test_motors_sim
	./test_host
	./test_motors_sim

test_host: test_host.c ../main/mini_json.c ../main/proto_format.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

# motors.c e incluido direto no teste, com headers falsos em stubs/
test_motors_sim: test_motors_sim.c ../main/motors.c ../main/app_config.h
	$(CC) -w -std=gnu11 -O1 -g -Istubs -I../main -o $@ test_motors_sim.c -lm

clean:
	rm -f test_host test_motors_sim tel_samples.txt
