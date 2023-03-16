import { Buffer } from "buffer";
import { mujoco_wasm } from "./public/mujoco-wasm";

export async function downloadFile(url, outputPath) {
    const x = await fetch(url);
    const x_1 = await x.arrayBuffer();
    FS.writeFile(outputPath, Buffer.from(x_1));

}

export * from mujoco_wasm;

