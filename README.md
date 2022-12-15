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

## Example

API followins `mujoco-rust` conventions for wrapping C code to JS objects. 

*All pointer fields from `mjModel` and `mjState` need to be wrapped in `Vec<T>` in `State` and `Model` in order to be exported to JS.*

```html
<!doctype html>
<html>
<script src="buffer.js"></script>

<script>
    let Buffer = buffer.Buffer;

    function downloadFile(url, outputPath) {
        return fetch(url)
            .then(x => x.arrayBuffer())
            .then(x => {
                FS.writeFile(outputPath, Buffer.from(x));
            });

    }

    var Module = {
        onRuntimeInitialized: function () {
            FS.mkdir('/working');
            FS.mount(MEMFS, { root: '.' }, '/working');
            downloadFile('simple.xml', '/working/simple.xml').then(() => {
                let model = Module.Model.load_from_xml("/working/simple.xml");
                let state = new Module.State(model);

                let simulation = new Module.Simulation(model, state);

                for (var i = 0; i < 100; i++) {
                    simulation.step();
                }
            });
        }
    };
</script>
<script src="mujoco_wasm.js"></script>

</html>
```

