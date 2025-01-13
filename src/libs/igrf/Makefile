all: clean
	gcc main.c igrf.c -g -Wall -Wextra -Werror -pedantic -fsanitize=undefined,address -lm
	./a.out

clean:
	rm a.out || true
