# DXWY LCD panels

This provides Linux drivers for LCD panels manufactured by 深圳市大显伟业科技有限公司 (DXWY):

- **D350T1013V1** — 3.5" panel connected via 2 DSI lanes
- **D500C2523V0** — 5.0" 720x1280 MIPI-DSI panel based on the Sitronix ST7703 controller, connected via 2 or 4 DSI lanes

The panels have been tested using a Raspberry Pi 4 Compute Module with the open source KMS stack.

When compiling for a Raspberry Pi downstream kernel specify `EXTRA_CFLAGS=-DRPI_KERNEL` during module build.

## License

GPL 2.0
