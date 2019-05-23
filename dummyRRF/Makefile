

CPPFLAGS=-std=gnu++17 -fno-rtti -fno-exceptions

all: test svg



test: test.cpp FiveBarScaraKinematics.cpp
svg:   svg.cpp FiveBarScaraKinematics.cpp

# arm-none-eabi-g++ -std=gnu++17 -D__SAM4S8C__ -DRTOS -DDUET_M -D_XOPEN_SOURCE -I"C:\eclipse\Firmware\CoreNG\cores\arduino" -I"C:\eclipse\Firmware\CoreNG\libraries\Flash" -I"C:\eclipse\Firmware\CoreNG\libraries\SharedSpi" -I"C:\eclipse\Firmware\CoreNG\libraries\Storage" -I"C:\eclipse\Firmware\CoreNG\libraries\Wire" -I"C:\eclipse\Firmware\CoreNG\asf" -I"C:\eclipse\Firmware\CoreNG\asf\common\utils" -I"C:\eclipse\Firmware\CoreNG\asf\common\services\clock" -I"C:\eclipse\Firmware\CoreNG\asf\common\services\ioport" -I"C:\eclipse\Firmware\CoreNG\asf\sam\drivers" -I"C:\eclipse\Firmware\CoreNG\asf\sam\services\flash_efc" -I"C:\eclipse\Firmware\CoreNG\asf\sam\utils" -I"C:\eclipse\Firmware\CoreNG\asf\sam\utils\cmsis\sam4s\include" -I"C:\eclipse\Firmware\CoreNG\asf\sam\utils\header_files" -I"C:\eclipse\Firmware\CoreNG\asf\sam\utils\preprocessor" -I"C:\eclipse\Firmware\CoreNG\asf\thirdparty\CMSIS\Include" -I"C:\eclipse\Firmware\CoreNG\variants\sam4s" -I"C:\eclipse\Firmware\RepRapFirmware\src" -I"C:\eclipse\Firmware\RepRapFirmware\src\DuetM" -I"C:\eclipse\Firmware\RepRapFirmware\src\Networking" -I"C:\eclipse\Firmware\DuetWiFiSocketServer\src\include" -I"C:\eclipse\Firmware\FreeRTOS\src\include" -I"C:\eclipse\Firmware\FreeRTOS\src\portable\GCC\ARM_CM3" -I"C:\eclipse\Firmware\RRFLibraries\src" -O2 -Wall -c -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections -fno-threadsafe-statics -fno-rtti -fno-exceptions -nostdlib -Wdouble-promotion -fsingle-precision-constant "-Wa,-ahl=FiveBarScaraKinematics.s" -MMD -MP -MF"src/Movement/Kinematics/FiveBarScaraKinematics.d" -MT"src/Movement/Kinematics/FiveBarScaraKinematics.o" -o "src/Movement/Kinematics/FiveBarScaraKinematics.o" "../src/Movement/Kinematics/FiveBarScaraKinematics.cpp"
