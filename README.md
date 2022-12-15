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

## JavaScript API

```bash
npm install mujoco-wasm --save
```

```javascript
import { Model, Simulation, State, downloadFile } from "mujoco-wasm";
```

**Utility Functions**

| method       | description                                                               |
| ------------ | ------------------------------------------------------------------------- |
| downloadFile | Download file from url and store in MEMFS so that wasm module can read it |

**Model**

| method        | description                 |
| ------------- | --------------------------- |
| load_from_xml | Load model from xml string  |
| ptr           | Get pointer to MuJoCo model |
| val           | Get MuJoCo model value      |
| names         | Get names of model          |
| mesh_vertadr  | Get mesh vertex address     |
| mesh_vertnum  | Get mesh vertex number      |
| mesh_faceadr  | Get mesh face address       |
| mesh_facenum  | Get mesh face number        |
| body_parentid | Get body parent id          |
| body_geomnum  | Get body geometry number    |
| body_geomadr  | Get body geometry address   |
| geom_type     | Get geometry type           |
| geom_bodyid   | Get geometry body id        |
| geom_group    | Get geometry group          |
| geom_contype  | Get geometry contact type   |
| mesh_normal   | Get mesh normal             |
| mesh_face     | Get mesh face               |
| mesh_vert     | Get mesh vertex             |
| name_meshadr  | Get name mesh address       |
| geom_pos      | Get geometry position       |
| geom_quat     | Get geometry quaternion     |
| geom_size     | Get geometry size           |
| geom_rgba     | Get geometry rgba           |
| body_pos      | Get body position           |
| body_quat     | Get body quaternion         |

**State**

| method | description                 |
| ------ | --------------------------- |
| ptr    | Get pointer to MuJoCo state |
| val    | Get MuJoCo state value      |

**Simulation**

| method | description          |
| ------ | -------------------- |
| step   | Step simulation      |
| state  | Get simulation state |
| model  | Get simulation model |
| xquat  | Get quaternion       |
| xpos   | Get position         |
