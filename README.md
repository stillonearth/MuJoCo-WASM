# mujoco_wasm

MuJoCo built with emscripten for use in JavaScript and WebAssembly. This includes MuJoCo 2.3.1 built as static library and a simple example application.

This repo is a fork of @stillonearth 's starter repository, adding tons of functionality and a comprehensive example scene.

## Usage

**1. Install emscripten**

**2. Build the mujoco_wasm Binary**

```bash
mkdir build
cd build
emcmake cmake ..
make
```

*3. (Optional) Update MuJoCo libs*

Build MuJoCo libs with wasm target and place to lib. Currently v0.3.1 included.

## JavaScript API

```javascript
import load_mujoco from "./mujoco_wasm.js";

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/humanoid.xml", await (await fetch("./public/scenes/humanoid.xml")).text());

// Load in the state from XML
let model       = new mujoco.Model("/working/humanoid.xml");
let state       = new mujoco.State(model);
let simulation  = new mujoco.Simulation(model, state);
```

Typescript definitions are available.

## Work In Progress Disclaimer

So far, mostly generic state variables are exposed.  Only three functions are exposed (loadXML, step, and applyForce).
