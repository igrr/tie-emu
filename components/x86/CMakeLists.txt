idf_component_register(SRCS x86.c x86_fpu.c INCLUDE_DIRS include)
# IDF builds with -fno-jump-tables (tables in flash are unsafe for IRAM code); none of this runs
# from IRAM, and the opcode switch is several times slower without them
target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables)
target_link_libraries(${COMPONENT_LIB} PUBLIC m) # x86_fpu.c
