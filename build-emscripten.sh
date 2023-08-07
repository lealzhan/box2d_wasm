#!/usr/bin/env bash

echo -e "\033[01;32m --------------- START -------------------- \033[0m"
now=`date +'%Y-%m-%d %H:%M:%S'`
start_time=$(date --date="$now" +%s)

#rm -rf BUILD_WASM
mkdir BUILD_WASM
cd BUILD_WASM

# make for debug/release and wasm/asm respectively
#check if imput parameter is debug
if [ "$1" = "debug" ]; then
    echo -e "\033[01;32m --------------- Build Debug --------------- \033[0m"
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=debug -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    echo -e "\033[01;32m ---------- Build wasm debug DONE ----------  \033[0m"
    
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=OFF -DCMAKE_BUILD_TYPE=debug -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    echo -e "\033[01;32m ---------- Build asm debug DONE ----------  \033[0m"

    cp -r ./bin/box2d.debug.asm.js ../../cocos-engine/native/external/emscripten/box2d
    cp -r ./bin/box2d.debug.wasm.js ../../cocos-engine/native/external/emscripten/box2d
    cp -r ./bin/box2d.debug.wasm.wasm ../../cocos-engine/native/external/emscripten/box2d

    echo -e "\033[01;32m ------------ Copy Done --------------  \033[0m"
else
    echo -e "\033[01;32m --------------- Build Release --------------- \033[0m"
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=release -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    
    echo -e "\033[01;32m ---------- Build wasm release DONE ----------  \033[0m"
    cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=OFF -DCMAKE_BUILD_TYPE=release -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
    make -j 12
    echo -e "\033[01;32m ---------- Build  asm release DONE ----------  \033[0m"

    cp -r ./bin/box2d.release.asm.js ../../cocos-engine/native/external/emscripten/box2d
    cp -r ./bin/box2d.release.wasm.js ../../cocos-engine/native/external/emscripten/box2d
    cp -r ./bin/box2d.release.wasm.wasm ../../cocos-engine/native/external/emscripten/box2d
    
    echo -e "\033[01;32m ------------ Copy Done --------------  \033[0m"
fi


now=`date +'%Y-%m-%d %H:%M:%S'`
end_time=$(date --date="$now" +%s);
echo -e "\033[01;32m Time Used: "$((end_time-start_time))"s  \033[1m"
echo -e "\033[01;32m ------------- END -----------------  \033[0m"