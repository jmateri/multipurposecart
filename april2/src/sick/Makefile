include ../common.mk

CFLAGS = $(CFLAGS_STD) $(CFLAGS_COMMON) $(CFLAGS_DGC) $(CFLAGS_GLIB) $(CFLAGS_LCM) $(CFLAGS_LCMTYPES) -D_GNU_SOURCE

LDFLAGS = $(LDFLAGS_DGC) $(LDFLAGS_COMMON) $(LDFLAGS_STD) $(LDFLAGS_GLIB) $(LDFLAGS_LCM) $(LDFLAGS_LCMTYPES)

SICK_OBJS = sick.o main.o

SICK=$(BIN_PATH)/sick

all: $(SICK)


$(SICK): $(SICK_OBJS)
	@echo "    [$@]"
	$(CC) -o $@ $(SICK_OBJS) $(LDFLAGS)

clean:
	rm -f *.o *.a *~ $(SICK)
