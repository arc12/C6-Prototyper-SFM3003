idf_component_register(SRCS "sfm3003.c"
                    INCLUDE_DIRS "include"
                    REQUIRES ulp esp_driver_i2c c6_prototyper_core)

# ULP support #
# 1. The LP Core app name must be unique (if multiple components use LP Core).
set(ulp_app_name lp_core_sfm)
#
# 2. Specify all C files.
#    Files should be placed into a separate directory (in this case, lp_core/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_lp_core_sources "lp_core/main.c")

#
# 3. List all the component source files which include automatically
#    generated LP Core export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "lp_i2c_main.c")

#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")