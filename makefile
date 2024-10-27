LDLIBS += -lm
CFLAGS = -std=c11 -Wall -Wextra -pedantic -fsanitize=address,undefined -g -O0

all: d2s s2d

clean:
	rm d2s s2d
