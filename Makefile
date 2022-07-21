obj-m += tmate_baseboard.o

all:
	make -C ${LINUX_DIR} M=$(PWD) modules

clean:
	make -C ${LINUX_DIR} M=$(PWD) clean

