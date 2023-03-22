
import  *  as  THREE     from 'three';
import { GUI           } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { Reflector     } from './utils/Reflector.js';
import { DragStateManager       } from './utils/DragStateManager.js';
import { downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos } from './mujocoSceneLoader.js';
import   load_mujoco     from '../dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();
// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/humanoid.xml", await (await fetch("./examples/scenes/humanoid.xml")).text());

// Load in the state from XML
let model       = new mujoco.Model("/working/humanoid.xml");
let state       = new mujoco.State(model);
let simulation  = new mujoco.Simulation(model, state);

// Whether the simulation is running or paused.
let paused = false;

// Ctrlnoise.
let ctrlnoisestd = 0.0;
let ctrlnoiserate = 0.0;
let ctrlnoise = new Float64Array(model.nu());

let container, controls;
let camera, scene, renderer;
const params = { scene: "humanoid.xml" };
/** @type {DragStateManager} */
let dragStateManager;
let bodies, lights;

async function init() {
  container = document.createElement( 'div' );
  document.body.appendChild( container );

  scene = new THREE.Scene();
  scene.name = 'scene';

  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
  camera.position.set( 2.0, 1.7, 1.7 );

  camera.name = 'PerspectiveCamera';
  scene.add(camera);
  scene.background = new THREE.Color(0.15, 0.25, 0.35);
  scene.fog = new THREE.Fog(scene.background, 15, 25.5 );

  const ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
  ambientLight.name = 'AmbientLight';
  scene.add( ambientLight );

  renderer = new THREE.WebGLRenderer( { antialias: true } );
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap ; // default THREE.PCFShadowMap

  container.appendChild( renderer.domElement );

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.7, 0);
  controls.panSpeed = 2;
  controls.zoomSpeed = 1;
  controls.enableDamping = true;
  controls.dampingFactor = 0.10;
  controls.screenSpacePanning = true;
  controls.update();

  window.addEventListener('resize', onWindowResize);

  // Initialize the Drag State Manager.
  dragStateManager = new DragStateManager(scene, renderer, camera, container.parentElement, controls);

  const gui = new GUI();
  gui.add(params, 'scene', { "Humanoid": "humanoid.xml", "Cassie": "agility_cassie/scene.xml", "Hammock": "hammock.xml", "Balloons": "balloons.xml", "Hand": "shadow_hand/scene_right.xml", "Flag": "flag.xml", "Mug": "mug.xml", /*"Arm": "arm26.xml", "Adhesion": "adhesion.xml", "Boxes": "simple.xml" */})
    .name('Example Scene').onChange(value => {
      scene.remove(scene.getObjectByName("MuJoCo Root"));
      loadSceneFromURL(mujoco, value, scene).then((returnArray) => {
        [model, state, simulation, bodies, lights] = returnArray;
      }); // Initialize the three.js Scene using this .xml Model
    });

  // Add pause simulation button (can be triggered with spacebar).
  const pauseButton = gui.add({ pause: false }, 'pause').name('Pause Simulation');
  pauseButton.onChange((value) => {
    if (value) {
      paused = true;
      controls.enabled = false;
    } else {
      paused = false;
      controls.enabled = true;
    }
  });
  document.addEventListener('keydown', (event) => {
    if (event.code === 'Space') {
      pauseButton.setValue(!pauseButton.getValue());
    }
  });

  // Add a slider button for ctrlnoiserate and ctrlnoisestd.
  // min = 0, max = 2, step = 0.01
  gui.add({ ctrlnoiserate: 0.0 }, 'ctrlnoiserate', 0.0, 2.0, 0.01).name('Noise rate').onChange((value) => {
    ctrlnoiserate = value;
  });
  gui.add({ ctrlnoisestd: 0.0 }, 'ctrlnoisestd', 0.0, 2.0, 0.01).name('Noise scale').onChange((value) => {
    ctrlnoisestd = value;
  });

  gui.open();

  await downloadExampleScenesFolder(mujoco);    // Download the the examples to MuJoCo's virtual file system
  [model, state, simulation, bodies, lights] =  // Initialize the three.js Scene using this .xml Model
    await loadSceneFromURL(mujoco, "humanoid.xml", scene);
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize( window.innerWidth, window.innerHeight );
}

function animate(time) {
  requestAnimationFrame( animate );
  render(time);
}

// Standard normal random number generator using Box-Muller transform.
function standardNormal() {
  let u = 1 - Math.random();
  let v = Math.random();
  let z = Math.sqrt( -2.0 * Math.log( u ) ) * Math.cos( 2.0 * Math.PI * v );
  return z;
}

let mujoco_time = 0.0;
function render(timeMS) {
  controls.update();

  if (!paused) {
    let timestep = model.getOptions().timestep;
    if (timeMS - mujoco_time > 35.0) { mujoco_time = timeMS; }
    while (mujoco_time < timeMS) {
      // Inject noise.
      if (ctrlnoisestd > 0.0) {
        let rate = Math.exp(-timestep / Math.max(1e-10, ctrlnoiserate));
        let scale = ctrlnoisestd * Math.sqrt(1 - rate * rate);
        console.log("rate: " + rate + ", scale: " + scale);
        for (let i = 0; i < model.nu(); i++) {
          ctrlnoise[i] = rate * ctrlnoise[i] + scale * standardNormal();
          simulation.ctrl()[i] = ctrlnoise[i];
        }
      }

      // Clear old perturbations, apply new ones.
      for (let i = 0; i < simulation.qfrc_applied().length; i++) { simulation.qfrc_applied()[i] = 0.0; }
      let dragged = dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        for (let b = 0; b < model.nbody(); b++) {
          if (bodies[b]) {
            getPosition  (simulation.xpos (), b, bodies[b].position);
            getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
            bodies[b].updateWorldMatrix();
          }
        }
        let bodyID = dragged.bodyID;
        dragStateManager.update(); // Update the world-space force origin
        let force = toMujocoPos(dragStateManager.currentWorld.clone().sub(dragStateManager.worldHit).multiplyScalar(model.body_mass()[bodyID] * 250));
        let point = toMujocoPos(dragStateManager.worldHit.clone());
        simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);

        // TODO: Apply pose perturbations (mocap bodies only).
      }

      simulation.step();

      mujoco_time += timestep * 1000.0;
    }
  }  // End if (!paused)
  else {
    // TODO: Apply pose perturbations (mocap and dynamic bodies).
    let dragged = dragStateManager.physicsObject;
    if (dragged && dragged.bodyID) {

    }

    simulation.forward();
  }

  // Update body transforms.
  for (let b = 0; b < model.nbody(); b++) {
    if (bodies[b]) {
      getPosition  (simulation.xpos (), b, bodies[b].position);
      getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
      bodies[b].updateWorldMatrix();
    }
  }

  // Update light transforms.
  let tmpVec = new THREE.Vector3();
  for (let l = 0; l < model.nlight(); l++) {
    if (lights[l]) {
      getPosition(simulation.light_xpos(), l, lights[l].position);
      getPosition(simulation.light_xdir(), l, tmpVec);
      lights[l].lookAt(tmpVec.add(lights[l].position));
    }
  }

  // Render!
  renderer.render( scene, camera );
}

await init();
animate();
