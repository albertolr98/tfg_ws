add_library(tmc_lib
	src/TMC2130.c
	src/CRC.c
	src/trinamic_porting.c
	src/Functions.c)

target_include_directories(tmc_lib PUBLIC include)
ament_export_targets(tmc_lib HAS_LIBRARY_TARGET)
