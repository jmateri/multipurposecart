include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_DGC) $(CFLAGS_GLIB) $(CFLAGS_LCM) $(CFLAGS_LCMTYPES) -D_GNU_SOURCE -O2

LDFLAGS = $(LDFLAGS_DGC) $(LDFLAGS_COMMON) $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_LCM) $(LDFLAGS_LCMTYPES)

HOKUYO_OBJS = scip2.o timesync.o main.o

HOKUYO=$(BIN_PATH)/hokuyo

all: $(HOKUYO)


$(HOKUYO): $(HOKUYO_OBJS)
	@echo "    [$@]"
	$(CC) -o $@ $(HOKUYO_OBJS) $(LDFLAGS)

clean:
	rm -f *.o *.a *~ $(HOKUYO)
