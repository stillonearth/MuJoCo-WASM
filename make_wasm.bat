call python src/parse_mjxmacro.py
rmdir /s /q build
call ../emsdk/emsdk activate latest
mkdir build
cd build
call emcmake cmake ..
call emmake make
cd ../public
rem call npx tsembind mujoco_wasm.js > mujoco_wasm.d.ts
pause