macro(ompl_backup filename)
    if(NOT EXISTS "${filename}.orig")
        if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 2.6)
            file(RENAME "${filename}" "${filename}.orig")
        else()
            file(READ "${filename}" FILECONTENTS)
            file(WRITE "${filename}.orig" "${FILECONTENTS}")
        endif()
    endif(NOT EXISTS "${filename}.orig")
endmacro(ompl_backup)

message(STATUS "Patching PQP")
ompl_backup("${PQP_INCLUDE}/PQP_compile.h")
file(READ "${PQP_INCLUDE}/PQP_compile.h.orig" PQPCOMPILE_H_ORIG)
string(REPLACE "inline float sqrt(float x) { return (float)sqrt((double)x); }\ninline float cos(float x) { return (float)cos((double)x); }\ninline float sin(float x) { return (float)sin((double)x); }\ninline float fabs(float x) { return (float)fabs((double)x); }\n" "" PQPCOMPILE_H "${PQPCOMPILE_H_ORIG}")
file(WRITE "${PQP_INCLUDE}/PQP_compile.h" "${PQPCOMPILE_H}")
message(STATUS "Patched ${PQP_INCLUDE}/PQP_compile.h")

ompl_backup("${PQP_INCLUDE}/PQP.h")
file(READ "${PQP_INCLUDE}/PQP.h.orig" PQP_H_ORIG)
string(REPLACE "\nPQP_" "\n__declspec(dllexport) PQP_" PQP_H "${PQP_H_ORIG}")
file(WRITE "${PQP_INCLUDE}/PQP.h" "${PQP_H}")
message(STATUS "Patched ${PQP_INCLUDE}/PQP.h")

ompl_backup("${PQP_INCLUDE}/PQP_internal.h")
file(READ "${PQP_INCLUDE}/PQP_Internal.h.orig" PQPINTERNAL_H_ORIG)
string(REPLACE "class PQP_Model" "class __declspec(dllexport) PQP_Model" PQPINTERNAL_H_1 "${PQPINTERNAL_H_ORIG}")
string(REPLACE "struct CollisionPair" "struct __declspec(dllexport) CollisionPair" PQPINTERNAL_H_2 "${PQPINTERNAL_H_1}")
string(REPLACE "struct PQP_CollideResult" "struct __declspec(dllexport) PQP_CollideResult" PQPINTERNAL_H "${PQPINTERNAL_H_2}")
file(WRITE "${PQP_INCLUDE}/PQP_Internal.h" "${PQPINTERNAL_H}")
message(STATUS "Patched ${PQP_INCLUDE}/PQP_Internal.h")