#!/usr/bin/env bash

#rm -rf BUILD_WASM
mkdir BUILD_WASM

cd BUILD_WASM
cmake .. -G "Unix Makefiles" -B ./ -DBUILD_WASM=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=${EMSCRIPTEN}/cmake/Modules/Platform/Emscripten.cmake
make -j 12

# copy built files to the web directory
cp -r ./bin/box2d.Debug.js ../emscriptenTest
cp -r ./bin/box2d.Debug.wasm ../emscriptenTest
cp -r ./bin/box2d.Release.js ../emscriptenTest
cp -r ./bin/box2d.Release.wasm ../emscriptenTest