obj-m += panel-dxwy-d350t1013v1.o

all:
	make -C ${LINUX_DIR} M=$(PWD) modules

clean:
	make -C ${LINUX_DIR} M=$(PWD) clean

