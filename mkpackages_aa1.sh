#!/bin/bash

set -e -u -o pipefail
shopt -s nullglob
shopt -s extglob

SELF_DIR="$(dirname $(readlink -f ${BASH_SOURCE[0]}))"

OUT_DIR="$SELF_DIR/packages_out"
DL_DIR="$OUT_DIR/tmp"

mkdir -p "$DL_DIR"

EBE_RELEASE=master
URL_BASE="http://www.enclustra.com/binaries/enclustra-bsp/${EBE_RELEASE}/refdes/"

# Files which should be reused from old packages
REUSE_FILES=(
	boot_full.conf
	boot_full_ramdisk.conf
	refdes.rbf
	refdes.rbf.img
	refdes_emmc.rbf
)
declare -A COPY_FILES=(
	['bscripts/uboot_ramdisk.scr']='%STORAGE%_uboot_ramdisk.scr'
	['bscripts/uboot.scr']='%STORAGE%_uboot.scr'
	['uboot_w_dtb-mkpimage.bin']='uboot_w_dtb-mkpimage-%STORAGE%.bin'
)

STORAGES=(mmc emmc qspi)

log() {
	echo -e "\x1B[34;1m$*\x1B[0m"
}

package_for() {
	local name="$1"
	local dl_url="${URL_BASE}/${name}.zip"

	local out_dir="$OUT_DIR/$name"
	local out_file="$OUT_DIR/${name}.zip"
	local dl_file="${DL_DIR}/${name}.zip"

	log "\nCREATING PACKAGE FOR: $name\n"

	cd $SELF_DIR

	mkdir -p "$out_dir"
	rm -rf $out_dir/*

	log "Downloading $dl_url  =>  ${dl_file#$SELF_DIR/}"
	wget -O "$dl_file" "$dl_url"

	log "Unpacking the archive"
	unzip -o "$dl_file" "${REUSE_FILES[@]}" -d "$out_dir"

	for storage in ${STORAGES[@]}; do

		log "${storage^^}: build"
		make clean && MERCURY_AA1_BOOTD=${storage^^} make -j$(nproc)

		log "${storage^^}: copy files"
		for f in "${!COPY_FILES[@]}"; do
			cp "$f" "$out_dir/${COPY_FILES[$f]//%STORAGE%/${storage}}"
		done

	done

	log "Create package"
	zip -j "$out_file" $out_dir/*

	log "Done: $out_file"
}

#	To build preloader for Mercury AA1 call:
#        - MMC boot:
#                make clean && make -j8
#        - QSPI boot:
#                make clean && make -j8
#        - EMMC boot:
#                make clean && MERCURY_AA1_BOOTD=EMMC make -j8
#
#To build for AA1 Rev1 it's necessary to set env variable:

# Rev 2
package_for mercury_aa1_pe1_sx480_i2
package_for mercury_aa1_pe1_sx270_i2

# Rev 1
export MERCURY_AA1_REV1=1
package_for mercury_aa1_pe1_sx480_i2_r1
