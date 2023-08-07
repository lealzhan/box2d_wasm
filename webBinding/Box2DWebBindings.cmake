
MESSAGE("[Box2D]webBinding/Box2DWebBindings.cmake")

SET(WEB_BINDING_SOURCE_DIR ${PROJECT_SOURCE_DIR}/webBinding)

SET(CMAKE_CXX_FLAGS "-std=c++11 -fno-exceptions -ffunction-sections\
 -fdata-sections -Werror -ferror-limit=0 -Wall -Wextra -fstrict-aliasing -Wstrict-aliasing=2\
  -Weverything -Wno-documentation-deprecated-sync -Wno-documentation-unknown-command -Wno-float-equal\
   -Wno-padded -Wno-weak-vtables -Wno-cast-align -Wno-conversion -Wno-missing-noreturn\
    -Wno-missing-variable-declarations -Wno-shift-sign-overflow -Wno-covered-switch-default\
     -Wno-exit-time-destructors -Wno-global-constructors -Wno-missing-prototypes -Wno-unreachable-code\
      -Wno-unused-macros -Wno-unused-member-function -Wno-used-but-marked-unused -Wno-weak-template-vtables\
       -Wno-deprecated -Wno-non-virtual-dtor -Wno-invalid-noreturn -Wno-return-type-c-linkage\
        -Wno-reserved-id-macro -Wno-c++98-compat-pedantic -Wno-unused-local-typedef -Wno-old-style-cast\
         -Wno-newline-eof -Wno-unused-private-field -Wno-undefined-reinterpret-cast -Wno-invalid-offsetof\
          -Wno-unused-value -Wno-format-nonliteral  -Wno-undef\
          -Wno-unsafe-buffer-usage -fno-rtti -DEMSCRIPTEN_HAS_UNBOUND_TYPE_NAMES=0\
          -Wno-inconsistent-missing-destructor-override -Wno-suggest-destructor-override\
          -Wno-limited-postlink-optimizations\
          -Wbad-function-cast -Wcast-function-type\
          ")

# Build debug info for all configurations
SET(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 -O0 -g3")
SET(CMAKE_CXX_FLAGS_CHECKED "-std=c++11 -g3 -gdwarf-2 -O3")
SET(CMAKE_CXX_FLAGS_PROFILE "-std=c++11 -O3 -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3")

#https://emscripten.org/docs/optimizing/Optimizing-Code.html#optimizing-code-size
#https://github.com/emscripten-core/emscripten/blob/main/src/settings.js
if(${BUILD_WASM})
        SET(EMSCRIPTEN_BASE_OPTIONS "--bind -s MODULARIZE=1 -s EXPORT_NAME=BOX2D -s ALLOW_MEMORY_GROWTH=1\
         -s WASM=1 -s ENVIRONMENT=web -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s FILESYSTEM=0\
          -s MIN_SAFARI_VERSION=110000 -s DYNAMIC_EXECUTION=0 ")
else()
        SET(EMSCRIPTEN_BASE_OPTIONS "--bind -O3 -s MODULARIZE=1 -s EXPORT_NAME=BOX2D -s ALLOW_MEMORY_GROWTH=1\
         -s WASM=0 -s ENVIRONMENT=web -s SINGLE_FILE=1 -s ERROR_ON_UNDEFINED_SYMBOLS=0 -s DYNAMIC_EXECUTION=0")
endif()

SET(BOX2D_WEB_BINDINGS_SOURCE
        ${WEB_BINDING_SOURCE_DIR}/Box2DWebBindings.cpp
        )

ADD_EXECUTABLE(Box2DWebBindings ${BOX2D_WEB_BINDINGS_SOURCE})

SET_TARGET_PROPERTIES(Box2DWebBindings PROPERTIES
        LINK_FLAGS "${EMSCRIPTEN_BASE_OPTIONS}"
        )

if(${BUILD_WASM})
	set_target_properties(Box2DWebBindings PROPERTIES OUTPUT_NAME "box2d.${CMAKE_BUILD_TYPE}.wasm")
else()
	set_target_properties(Box2DWebBindings PROPERTIES OUTPUT_NAME "box2d.${CMAKE_BUILD_TYPE}.asm")
endif()

SET_TARGET_PROPERTIES(Box2DWebBindings PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

TARGET_LINK_LIBRARIES(Box2DWebBindings
        PUBLIC box2d
        # PUBLIC libtess2
        )

TARGET_INCLUDE_DIRECTORIES(Box2DWebBindings
        PUBLIC ${PROJECT_SOURCE_DIR}/include/box2d
        )        