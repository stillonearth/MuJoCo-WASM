import load_mujoco from "./mujoco_wasm.js"
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/simple.xml", await (await fetch("./simple.xml")).text());

// Load in the state from XML
let model       = mujoco.Model.load_from_xml("/working/simple.xml");
let state       = new mujoco.State     (model);
let simulation  = new mujoco.Simulation(model, state);

// Decode the null-terminated string
let textDecoder = new TextDecoder("utf-8");
let fullString  = textDecoder.decode(model.names());
let names       = fullString.split(fullString[12]);
console.log(names);

// Run the Simulation for 100 Steps
for (var i = 0; i < 100; i++) {
    simulation.step();
    console.log("bodies position:", simulation.xpos());
}
