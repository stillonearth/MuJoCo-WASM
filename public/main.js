
import * as THREE from 'three';
import { GUI           } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import load_mujoco from "./mujoco_wasm.js"

let container, controls; // ModelLoader
let camera, scene, renderer, gridHelper, file = {}, material, sphere;
let model, state, simulation;
/** @type {THREE.Mesh} */
let mainModel, connections;
/** @type {THREE.Vector3} */
let tmp1 = new THREE.Vector3();
let raycaster, pointer = new THREE.Vector2();
const params = { alpha: 0.0 };

/** @type {Object.<number, THREE.Group>} */
let bodies = { };

async function init() {
  container = document.createElement( 'div' );
  document.body.appendChild( container );

  scene = new THREE.Scene();
  scene.name = 'scene';

  // ---------------------------------------------------------------------
  // Perspective Camera
  // ---------------------------------------------------------------------
  camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 10 );
  camera.position.set( 1.6, 1.4, 1.4 );

  camera.name = 'PerspectiveCamera';
  scene.add( camera );

  // ---------------------------------------------------------------------
  // Ambient light
  // ---------------------------------------------------------------------
  const ambientLight = new THREE.AmbientLight( 0xffffff, 0.2 );
  ambientLight.name = 'AmbientLight';
  scene.add( ambientLight );

  // ---------------------------------------------------------------------
  // DirectLight
  // ---------------------------------------------------------------------
  const dirLight = new THREE.DirectionalLight( 0xffffff, 1 );
  dirLight.target.position.set( 0, 0, - 1 );
  dirLight.add( dirLight.target );
  dirLight.lookAt( - 1, - 1, 0 );
  dirLight.name = 'DirectionalLight';
  scene.add( dirLight );

  // ---------------------------------------------------------------------
  // Grid
  // ---------------------------------------------------------------------
  gridHelper = new THREE.GridHelper( 5, 20, 0x222222, 0x444444 );
  //gridHelper.position.y = - 0.01;
  gridHelper.name = 'Grid';
  scene.add( gridHelper );

  //

  renderer = new THREE.WebGLRenderer( { antialias: true } );
  renderer.setPixelRatio( window.devicePixelRatio );
  renderer.setSize( window.innerWidth, window.innerHeight );

  container.appendChild( renderer.domElement );

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.1, 0);
  controls.panSpeed = 2;
  controls.zoomSpeed = 1;
  controls.enableDamping = true;
  controls.dampingFactor = 0.10;
  controls.screenSpacePanning = true;
  controls.update();

  //

  raycaster = new THREE.Raycaster();
  pointer   = new THREE.Vector2();

  window.addEventListener('resize', onWindowResize);
  document.addEventListener( 'pointermove', ( event ) => {
    pointer.x =   ( event.clientX / window.innerWidth  ) * 2 - 1;
    pointer.y = - ( event.clientY / window.innerHeight ) * 2 + 1;
  });

  const gui = new GUI();
  gui.add(params, "alpha", 0.0, 1.0, 0.001).name('Unfold Amount');
  gui.open();

  sphere = new THREE.Mesh(new THREE.SphereGeometry(0.01), new THREE.MeshPhysicalMaterial());
  sphere.position.y = 1.0;
  scene.add( sphere );

  material = new THREE.MeshPhysicalMaterial();

  // Load the MuJoCo WASM
  const mujoco = await load_mujoco();
  // Set up Emscripten's Virtual File System
  mujoco.FS.mkdir('/working');
  mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
  mujoco.FS.writeFile("/working/simple.xml", await (await fetch("./public/simple.xml")).text());

  // Load in the state from XML
  model       = mujoco.Model.load_from_xml("/working/simple.xml");
  state       = new mujoco.State     (model);
  simulation  = new mujoco.Simulation(model, state);

  // Decode the null-terminated string names
  let textDecoder = new TextDecoder("utf-8");
  let fullString  = textDecoder.decode(model.names());
  let names       = fullString.split(fullString[12]);
  console.log(names);

  for (let g = 0; g < model.ngeom(); g++) {
    let b = model.geom_bodyid()[g];
    let type = model.geom_type  ()[g];
    let size = [
      model.geom_size()[(g*3) + 0] * 2.0,
      model.geom_size()[(g*3) + 1] * 2.0,
      model.geom_size()[(g*3) + 2] * 2.0];
    console.log(size);
    console.log("Found geometry", g, " for body", b, ", Type:", type, ", named:", names[b + 1]);

    if (!(b in bodies)) { bodies[b] = new THREE.Group(); bodies[b].name = names[b + 1]; }

    let geometry;
    if (type == 0)        { // Plane is 0
      //geometry = new THREE.PlaneGeometry(size[0], size[1]);
      geometry = new THREE.BoxGeometry(size[0], size[2], size[1]);
    } else if (type == 1) { // Heightfield is 1
    } else if (type == 2) { // Sphere is 2
      geometry = new THREE.SphereGeometry(size[0]);
    } else if (type == 3) { // Capsule is 3
    } else if (type == 4) { // Ellipsoid is 4
    } else if (type == 5) { // Cylinder is 5
    } else if (type == 6) { // Box is 6
      geometry = new THREE.BoxGeometry(size[0], size[2], size[1]);
    } else if (type == 100) { // Arrow is 100
    
    }

    let mesh = new THREE.Mesh(geometry, material);
    bodies[b].add(mesh);
    getPosition  (model.geom_pos (), g, mesh.position  );
    getQuaternion(model.geom_quat(), g, mesh.quaternion);
  }

  for (let b = 0; b < model.nbody(); b++) {
    let parent_body = model.body_parentid()[b];
    if (parent_body == b) {
      scene.add(bodies[b]);
    } else {
      bodies[parent_body].add(bodies[b]);
    }
  }
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

function getPosition(buffer, index, target) {
  target.set(
    buffer[(index * 3) + 0],
    buffer[(index * 3) + 2],
    buffer[(index * 3) + 1]);
}

function getQuaternion(buffer, index, target) {
  return target.set(
    -buffer[(index * 4) + 1],
     buffer[(index * 4) + 3],
    -buffer[(index * 4) + 2],
     buffer[(index * 4) + 0]);
}

function render(time) {
  controls.update();

  // Update MuJoCo Simulation
  simulation.step();
  for (let b = 0; b < model.nbody(); b++) {
    getPosition  (simulation.xpos (), b, bodies[b].position  );
    getQuaternion(simulation.xquat(), b, bodies[b].quaternion);
  }

  renderer.render( scene, camera );
}

await init();
animate();
