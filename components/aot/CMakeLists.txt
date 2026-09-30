# Mixed-mode runtime (aot.c) and the translation of the game, generated at build time from
# Z_TIE__.EXE and the iMUSE modules in IMUSE.EXE by aot/aotgen.py. The output is derived from the
# game, so it is not in the repository.
set(AOT_DIR ${CMAKE_SOURCE_DIR}/aot)
set(TIE_GAME_DIR ${CMAKE_SOURCE_DIR}/sdcard/tie/c CACHE PATH "the game directory (the card's C:)")
set(AOT_EXE ${TIE_GAME_DIR}/Z_TIE__.EXE)
set(AOT_IMUSE ${TIE_GAME_DIR}/IMUSE.EXE)
# The iMUSE modules the game loads for the sound setup in its IMUSE.INI (FM4.MDR, SB16.WDR), and
# the addresses it loads them at
set(AOT_DLLS "imuse@0x219038;fm4@0x214044;sb16@0x228038")
set(AOT_FILES 32)

foreach(f ${AOT_EXE} ${AOT_IMUSE})
    if(NOT EXISTS ${f})
        message(FATAL_ERROR "${f} not found: the build translates the game's code. Prepare the "
                            "game files first (README.md), or set TIE_GAME_DIR to the directory with them.")
    endif()
endforeach()

if(EXISTS ${CMAKE_SOURCE_DIR}/.venv/bin/python)
    set(aot_python_default ${CMAKE_SOURCE_DIR}/.venv/bin/python)
else()
    set(aot_python_default python3)
endif()
set(AOT_PYTHON ${aot_python_default} CACHE FILEPATH "python with capstone (requirements.txt)")
execute_process(COMMAND ${AOT_PYTHON} -c "import capstone" RESULT_VARIABLE rc OUTPUT_QUIET ERROR_QUIET)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "${AOT_PYTHON} cannot import capstone: install requirements.txt (README.md), "
                        "or set AOT_PYTHON to a python that has it.")
endif()

set(AOT_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
file(MAKE_DIRECTORY ${AOT_GEN_DIR})
math(EXPR last "${AOT_FILES} - 1")
set(AOT_GEN_SOURCES ${AOT_GEN_DIR}/aot_table.c)
foreach(i RANGE ${last})
    if(i LESS 10)
        set(i "0${i}")
    endif()
    list(APPEND AOT_GEN_SOURCES ${AOT_GEN_DIR}/aot_gen_${i}.c)
endforeach()
set(AOT_GEN_ARGS --files ${AOT_FILES} --imuse ${AOT_IMUSE})
foreach(dll ${AOT_DLLS})
    list(APPEND AOT_GEN_ARGS --dll ${dll})
endforeach()
file(GLOB AOT_PY ${AOT_DIR}/*.py)
add_custom_command(OUTPUT ${AOT_GEN_SOURCES} ${AOT_GEN_DIR}/aot_gen.h
    COMMAND ${AOT_PYTHON} ${AOT_DIR}/aotgen.py ${AOT_EXE} ${AOT_GEN_DIR} ${AOT_GEN_ARGS}
    DEPENDS ${AOT_PY} ${AOT_EXE} ${AOT_IMUSE}
    COMMENT "Translating Z_TIE__.EXE and iMUSE to C"
    VERBATIM)

idf_component_register(SRCS aot.c aot_glue.c ${AOT_GEN_SOURCES}
                       INCLUDE_DIRS include
                       PRIV_INCLUDE_DIRS ${AOT_GEN_DIR}
                       REQUIRES x86)
target_compile_options(${COMPONENT_LIB} PRIVATE -fjump-tables) # the blocks of a function are a switch
# GCC 16 fails (an internal error in rtl-ssa) on the translation, with the guest registers in
# global register variables (aot_rt.h), unless its late-combine pass is off
set_source_files_properties(${AOT_GEN_SOURCES} PROPERTIES COMPILE_OPTIONS "-w;-fno-late-combine-instructions")
