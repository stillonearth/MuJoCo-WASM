# MuJoCo-WASM

MuJoCo built with emscripten for use in JavaScript and WebAssembly. This includes MuJoCo 2.3.1 built as static library and a simple example application.

## Usage

**1. Install emscripten**

**2. Build MuJoCo-WASM application**

```bash
mkdir build
cd build
emcmake cmake ..
make
```

**3. Run in broweser**

``bash
cd public
emrun mujoco_wasm.html
``

## Possible Usecases

One can then make bindings to `mj_Model` and `mj_Body` and visualize the simulation with [Three.js](https://threejs.org/)
