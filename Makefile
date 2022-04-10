CC=gcc
LIBS=-lncursesw

all: mecanumrover_commlib.o mecanumrover_monitor mecanumrover_commander mecanumrover_memmap_dump_to_file

mecanumrover_commlib.o: mecanumrover_commlib.h
	$(CC) -c mecanumrover_commlib.c

mecanumrover_monitor:
	$(CC) mecanumrover_monitor.c -o mecanumrover_monitor mecanumrover_commlib.o

mecanumrover_commander:
	$(CC) mecanumrover_commander.c -o mecanumrover_commander mecanumrover_commlib.o $(LIBS)

mecanumrover_memmap_dump_to_file:
	$(CC) mecanumrover_memmap_dump_to_file.c -o mecanumrover_memmap_dump_to_file mecanumrover_commlib.o

clean:
	rm -f *.o mecanumrover_monitor mecanumrover_commander mecanumrover_memmap_dump_to_file
