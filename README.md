# altera-arria10-uboot

## How to build:

* Install Intel SoC EDS
* Append directories containing following binaries to `$PATH`:
  * `arm-altera-eabi-gcc` (`$SOCEDS_INSTALL_DIR`/embedded/host_tools/mentor/gnu/arm/baremetal/bin)
  * `mkpimage` (`$SOCEDS_INSTALL_DIR`/embedded/host_tools/altera/mkpimage/)
* `export SOCEDS_DEST_ROOT=$SOCEDS_INSTALL_DIR/embedded`
* `make help` for available targets
