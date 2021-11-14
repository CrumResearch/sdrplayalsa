all: sdrplayalsa

clean:
	rm -f sdrplayalsa

sdrplayalsa: sdrplayalsa.c
	$(CC) -Wall -O2 -o $@ $< -lsdrplay_api -lasound
