
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

let container, controls;
let camera, scene, renderer;
const params = { scene: "humanoid.xml" };
/** @type {DragStateManager} */
let dragStateManager;

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

  const gui = new GUI();
  //gui.add(params, "acceleration", -0.1, 0.1, 0.001).name('Artificial Acceleration');
  gui.add(params, 'scene', { "Humanoid": "humanoid.xml", "Cassie": "agility_cassie/scene.xml", "Hand": "shadow_hand/scene_right.xml", "Balloons": "balloons.xml",  "Hammock": "hammock.xml", "Flag": "flag.xml", "Mug": "mug.xml",  /*"Arm": "arm26.xml", "Adhesion": "adhesion.xml", "Boxes": "simple.xml" */})
    .name('Example Scene').onChange(value => { scene.remove(bodies[0]); bodies = {}; meshes = {}; lights = [];  loadSceneFromURL(value); } );
  gui.open();

  // Initialize the Drag State Manager
  dragStateManager = new DragStateManager(scene, renderer, camera, container.parentElement, controls);

  await downloadExampleScenesFolder(mujoco);             // Download the rest of the examples to MuJoCo's virtual file system
  await loadSceneFromURL(mujoco, "humanoid.xml", scene); // Initialize the three.js Scene using this .xml Model
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

let mujoco_time = 0.0;
function render(timeMS) {
  controls.update();

  // Update MuJoCo Simulation
  let timestep = model.getOptions().timestep;
  if (timeMS - mujoco_time > 35.0) { mujoco_time = timeMS; }
  while (mujoco_time < timeMS) {
    simulation.step();

    // Update the transform of the dragged body
    let dragged = dragStateManager.physicsObject;
    if (dragged && dragged.bodyID) {
      let bodyID = dragged.bodyID;
      getPosition(simulation.xpos(), bodyID, dragged.position);
      getQuaternion(simulation.xquat(), bodyID, dragged.quaternion);
      dragged.updateWorldMatrix();
      dragStateManager.update(); // Update the world-space force origin
      let force = toMujocoPos(dragStateManager.currentWorld.clone().sub(dragStateManager.worldHit).multiplyScalar(model.body_mass()[bodyID] * 250));
      let point = toMujocoPos(dragStateManager.worldHit.clone());
      simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID); // Body ID
    } else {
      // Reset Applied Forces
      for (let i = 0; i < simulation.qfrc_applied().length; i++) { simulation.qfrc_applied()[i] = 0.0; }
    }

    mujoco_time += timestep * 1000.0;
  }

  for (let b = 0; b < model.nbody(); b++) {
    if (bodies[b]) {
      getPosition  (simulation.xpos (), b, bodies[b].position);
      getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
      bodies[b].updateWorldMatrix();
    }
  }

  // Set the transforms of lights
  let tmpVec = new THREE.Vector3();
  for (let l = 0; l < model.nlight(); l++) {
    if (lights[l]) {
      getPosition(simulation.light_xpos(), l, lights[l].position);
      getPosition(simulation.light_xdir(), l, tmpVec);
      lights[l].lookAt(tmpVec.add(lights[l].position));
    }
  }

  renderer.render( scene, camera );
}

await init();
animate();
