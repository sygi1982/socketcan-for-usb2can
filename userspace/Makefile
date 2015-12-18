TARGET = ux2cand

INCLUDES = WinTypes.h usb2can.h

CFLAGS = -02 -g $(INCLUDES)

LDFLAGS = -lusb2can

default: all

$(TARGET).o: $(TARGET).c $(INCLUDES)
	gcc -c $(CFLAGS) $(TARGET).c -o $(TARGET).o

all: $(TARGET).o
	gcc $(TARGET).o -o $(TARGET) $(LDFLAGS)

clean:
	rm -f *.o
	rm -f $(TARGET)
