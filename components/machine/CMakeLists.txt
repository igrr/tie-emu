idf_component_register(SRCS machine.c le_load.c dos.c dpmi.c bios.c devices.c sound.c opl3.c
                       INCLUDE_DIRS include
                       REQUIRES x86 aot
                       PRIV_REQUIRES heap)
target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables) # see components/x86
