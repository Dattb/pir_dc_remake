################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../vendor/common/mi_api/libs/third_party/mbedtls/asn1parse.c \
../vendor/common/mi_api/libs/third_party/mbedtls/base64.c \
../vendor/common/mi_api/libs/third_party/mbedtls/ccm.c \
../vendor/common/mi_api/libs/third_party/mbedtls/md.c \
../vendor/common/mi_api/libs/third_party/mbedtls/md_wrap.c \
../vendor/common/mi_api/libs/third_party/mbedtls/sha256.c \
../vendor/common/mi_api/libs/third_party/mbedtls/sha256_hkdf.c 

OBJS += \
./vendor/common/mi_api/libs/third_party/mbedtls/asn1parse.o \
./vendor/common/mi_api/libs/third_party/mbedtls/base64.o \
./vendor/common/mi_api/libs/third_party/mbedtls/ccm.o \
./vendor/common/mi_api/libs/third_party/mbedtls/md.o \
./vendor/common/mi_api/libs/third_party/mbedtls/md_wrap.o \
./vendor/common/mi_api/libs/third_party/mbedtls/sha256.o \
./vendor/common/mi_api/libs/third_party/mbedtls/sha256_hkdf.o 


# Each subdirectory must supply rules for building sources it contributes
vendor/common/mi_api/libs/third_party/mbedtls/%.o: ../vendor/common/mi_api/libs/third_party/mbedtls/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TC32 Compiler'
	tc32-elf-gcc -ffunction-sections -fdata-sections -I"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE\pir_DC_new\firmware" -I"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE\pir_DC_new\firmware\vendor\common\mi_api\libs" -I"E:\D\Rang_Dong_tren_cty\CODE_Telink_BLE\pir_DC_new\firmware\vendor\common\mi_api\mijia_ble_api" -D__PROJECT_MESH__=1 -DCHIP_TYPE=CHIP_TYPE_8258 -Wall -O2 -fpack-struct -fshort-enums -finline-small-functions -std=gnu99 -fshort-wchar -fms-extensions -c -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


