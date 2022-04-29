# if(MSVC)
# 	FILE(GLOB LIBRARY_FILES_C "*.cpp" "*.rc")
# else()
# 	FILE(GLOB LIBRARY_FILES_C "*.cpp")
# endif()

FILE(GLOB LIBRARY_FILES_H "*.h" "*.inl")

cxx_executable_with_flags(ReconstructMesh "Apps" "${cxx_default}" "MVS;${OpenMVS_EXTRA_LIBS}" ReconstructMesh.cpp ${LIBRARY_FILES_H})
cxx_executable_with_flags(ReconstructMeshFromRGBD "Apps" "${cxx_default}" "MVS;${OpenMVS_EXTRA_LIBS}" ReconstructMeshFromRGBD.cpp ${LIBRARY_FILES_H})

# Install
INSTALL(TARGETS ReconstructMesh
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

# Install
INSTALL(TARGETS ReconstructMeshFromRGBD
	EXPORT OpenMVSTargets
	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
