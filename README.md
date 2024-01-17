# D350T1013V1 LCD panel

This provides a Linux driver for the D350T1013V1 LCD panel manufactured by 深圳市大显伟业科技有限公司 (DXWY).

The panel is connected via 2 DSI lanes and has been tested using a Raspberry Pi 4 Compute Module with the open source KMS stack.

When compiling for a Raspberry Pi downstream kernel specify `EXTRA_CFLAGS=-DRPI_KERNEL` during module build.

## License

GPL 2.0
