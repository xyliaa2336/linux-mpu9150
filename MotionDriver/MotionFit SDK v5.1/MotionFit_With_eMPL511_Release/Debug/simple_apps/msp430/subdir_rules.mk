################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
simple_apps/msp430/mllite_test.obj: ../simple_apps/msp430/mllite_test.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv5/tools/compiler/msp430_4.1.2/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=restricted --near_data=globals -O0 --opt_for_speed=5 -g --include_path="C:/ti/ccsv5/ccs_base/msp430/include" --include_path="C:/ti/ccsv5/tools/compiler/msp430_4.1.2/include" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/driver/eMPL" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/driver/include" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/driver/msp430" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/driver/msp430/F5xx_F6xx_Core_Lib" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/driver/msp430/USB_API" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/driver/msp430/USB_eMPL" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/eMPL-hal" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/mllite" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/core/mpl" --include_path="C:/Invensense_Karthik_Local/UMPL/eMPLAppsRelease511/simple_apps/msp430" --gcc --define=__MSP430F5528__ --define=USE_DMP --define=MPL_LOG_NDEBUG=1 --define=CONFIG_INTERFACE_USB --define=MPU9150 --define=I2C_B0 --define=EMPL --define=EMPL_TARGET_MSP430 --diag_warning=225 --display_error_number --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=full --preproc_with_compile --preproc_dependency="simple_apps/msp430/mllite_test.pp" --obj_directory="simple_apps/msp430" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


