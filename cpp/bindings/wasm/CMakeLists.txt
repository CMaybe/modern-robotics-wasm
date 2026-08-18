# Named apart from the static library target; OUTPUT_NAME still emits kinematics.js.
add_executable(kinematics_wasm bindings.cpp)
target_link_libraries(kinematics_wasm PRIVATE robotics::robotics)
target_compile_options(kinematics_wasm PRIVATE -O3)

# MODULARIZE + EXPORT_NAME gives a factory function instead of a global side
# effect, so the frontend can await instantiation. "node" is included so
# scripts/smoke_wasm.cjs can exercise the bindings headlessly.
target_link_options(kinematics_wasm PRIVATE
    -O3
    --bind
    -sMODULARIZE=1
    -sEXPORT_NAME=createKinematicsModule
    -sENVIRONMENT=web,node
    -sALLOW_MEMORY_GROWTH=1
    -sINITIAL_MEMORY=16MB
    -sASSERTIONS=0
    -sDISABLE_EXCEPTION_CATCHING=1
)

set_target_properties(kinematics_wasm PROPERTIES OUTPUT_NAME kinematics SUFFIX ".js")

# Drop kinematics.js / kinematics.wasm straight into the frontend's static assets.
set(WEB_WASM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../web/public/wasm)
add_custom_command(TARGET kinematics_wasm POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${WEB_WASM_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:kinematics_wasm> ${WEB_WASM_DIR}/kinematics.js
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE_DIR:kinematics_wasm>/kinematics.wasm ${WEB_WASM_DIR}/kinematics.wasm
    COMMENT "Copying kinematics.js / kinematics.wasm to web/public/wasm"
)
