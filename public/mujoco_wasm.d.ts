// Type definitions for Emscripten 1.39.16
// Project: https://emscripten.org
// Definitions by: Kensuke Matsuzaki <https://github.com/zakki>
//                 Periklis Tsirakidis <https://github.com/periklis>
//                 Bumsik Kim <https://github.com/kbumsik>
//                 Louis DeScioli <https://github.com/lourd>
// Definitions: https://github.com/DefinitelyTyped/DefinitelyTyped/blob/master/types/emscripten/index.d.ts
// TypeScript Version: 2.2

/** Other WebAssembly declarations, for compatibility with older versions of Typescript */
declare namespace WebAssembly {
    interface Module {}
}

declare namespace Emscripten {
  interface FileSystemType {}
  type EnvironmentType = 'WEB' | 'NODE' | 'SHELL' | 'WORKER';

  type JSType = 'number' | 'string' | 'array' | 'boolean';
  type TypeCompatibleWithC = number | string | any[] | boolean;

  type CIntType = 'i8' | 'i16' | 'i32' | 'i64';
  type CFloatType = 'float' | 'double';
  type CPointerType = 'i8*' | 'i16*' | 'i32*' | 'i64*' | 'float*' | 'double*' | '*';
  type CType = CIntType | CFloatType | CPointerType;

  type WebAssemblyImports = Array<{
      name: string;
      kind: string;
  }>;

  type WebAssemblyExports = Array<{
      module: string;
      name: string;
      kind: string;
  }>;

  interface CCallOpts {
      async?: boolean | undefined;
  }
}

interface EmscriptenModule {
  print(str: string): void;
  printErr(str: string): void;
  arguments: string[];
  environment: Emscripten.EnvironmentType;
  preInit: Array<{ (): void }>;
  preRun: Array<{ (): void }>;
  postRun: Array<{ (): void }>;
  onAbort: { (what: any): void };
  onRuntimeInitialized: { (): void };
  preinitializedWebGLContext: WebGLRenderingContext;
  noInitialRun: boolean;
  noExitRuntime: boolean;
  logReadFiles: boolean;
  filePackagePrefixURL: string;
  wasmBinary: ArrayBuffer;

  destroy(object: object): void;
  getPreloadedPackage(remotePackageName: string, remotePackageSize: number): ArrayBuffer;
  instantiateWasm(
      imports: Emscripten.WebAssemblyImports,
      successCallback: (module: WebAssembly.Module) => void,
  ): Emscripten.WebAssemblyExports;
  locateFile(url: string, scriptDirectory: string): string;
  onCustomMessage(event: MessageEvent): void;

  // USE_TYPED_ARRAYS == 1
  HEAP: Int32Array;
  IHEAP: Int32Array;
  FHEAP: Float64Array;

  // USE_TYPED_ARRAYS == 2
  HEAP8: Int8Array;
  HEAP16: Int16Array;
  HEAP32: Int32Array;
  HEAPU8: Uint8Array;
  HEAPU16: Uint16Array;
  HEAPU32: Uint32Array;
  HEAPF32: Float32Array;
  HEAPF64: Float64Array;

  TOTAL_STACK: number;
  TOTAL_MEMORY: number;
  FAST_MEMORY: number;

  addOnPreRun(cb: () => any): void;
  addOnInit(cb: () => any): void;
  addOnPreMain(cb: () => any): void;
  addOnExit(cb: () => any): void;
  addOnPostRun(cb: () => any): void;

  preloadedImages: any;
  preloadedAudios: any;

  _malloc(size: number): number;
  _free(ptr: number): void;
}

/**
* A factory function is generated when setting the `MODULARIZE` build option
* to `1` in your Emscripten build. It return a Promise that resolves to an
* initialized, ready-to-call `EmscriptenModule` instance.
*
* By default, the factory function will be named `Module`. It's recommended to
* use the `EXPORT_ES6` option, in which the factory function will be the
* default export. If used without `EXPORT_ES6`, the factory function will be a
* global variable. You can rename the variable using the `EXPORT_NAME` build
* option. It's left to you to declare any global variables as needed in your
* application's types.
* @param moduleOverrides Default properties for the initialized module.
*/
type EmscriptenModuleFactory<T extends EmscriptenModule = EmscriptenModule> = (
  moduleOverrides?: Partial<T>,
) => Promise<T>;

declare namespace FS {
  interface Lookup {
      path: string;
      node: FSNode;
  }

  interface FSStream {}
  interface FSNode {}
  interface ErrnoError {}

  let ignorePermissions: boolean;
  let trackingDelegate: any;
  let tracking: any;
  let genericErrors: any;

  //
  // paths
  //
  function lookupPath(path: string, opts: any): Lookup;
  function getPath(node: FSNode): string;

  //
  // nodes
  //
  function isFile(mode: number): boolean;
  function isDir(mode: number): boolean;
  function isLink(mode: number): boolean;
  function isChrdev(mode: number): boolean;
  function isBlkdev(mode: number): boolean;
  function isFIFO(mode: number): boolean;
  function isSocket(mode: number): boolean;

  //
  // devices
  //
  function major(dev: number): number;
  function minor(dev: number): number;
  function makedev(ma: number, mi: number): number;
  function registerDevice(dev: number, ops: any): void;

  //
  // core
  //
  function syncfs(populate: boolean, callback: (e: any) => any): void;
  function syncfs(callback: (e: any) => any, populate?: boolean): void;
  function mount(type: Emscripten.FileSystemType, opts: any, mountpoint: string): any;
  function unmount(mountpoint: string): void;

  function mkdir(path: string, mode?: number): any;
  function mkdev(path: string, mode?: number, dev?: number): any;
  function symlink(oldpath: string, newpath: string): any;
  function rename(old_path: string, new_path: string): void;
  function rmdir(path: string): void;
  function readdir(path: string): any;
  function unlink(path: string): void;
  function readlink(path: string): string;
  function stat(path: string, dontFollow?: boolean): any;
  function lstat(path: string): any;
  function chmod(path: string, mode: number, dontFollow?: boolean): void;
  function lchmod(path: string, mode: number): void;
  function fchmod(fd: number, mode: number): void;
  function chown(path: string, uid: number, gid: number, dontFollow?: boolean): void;
  function lchown(path: string, uid: number, gid: number): void;
  function fchown(fd: number, uid: number, gid: number): void;
  function truncate(path: string, len: number): void;
  function ftruncate(fd: number, len: number): void;
  function utime(path: string, atime: number, mtime: number): void;
  function open(path: string, flags: string, mode?: number, fd_start?: number, fd_end?: number): FSStream;
  function close(stream: FSStream): void;
  function llseek(stream: FSStream, offset: number, whence: number): any;
  function read(stream: FSStream, buffer: ArrayBufferView, offset: number, length: number, position?: number): number;
  function write(
      stream: FSStream,
      buffer: ArrayBufferView,
      offset: number,
      length: number,
      position?: number,
      canOwn?: boolean,
  ): number;
  function allocate(stream: FSStream, offset: number, length: number): void;
  function mmap(
      stream: FSStream,
      buffer: ArrayBufferView,
      offset: number,
      length: number,
      position: number,
      prot: number,
      flags: number,
  ): any;
  function ioctl(stream: FSStream, cmd: any, arg: any): any;
  function readFile(path: string, opts: { encoding: 'binary'; flags?: string | undefined }): Uint8Array;
  function readFile(path: string, opts: { encoding: 'utf8'; flags?: string | undefined }): string;
  function readFile(path: string, opts?: { flags?: string | undefined }): Uint8Array;
  function writeFile(path: string, data: string | ArrayBufferView, opts?: { flags?: string | undefined }): void;

  //
  // module-level FS code
  //
  function cwd(): string;
  function chdir(path: string): void;
  function init(
      input: null | (() => number | null),
      output: null | ((c: number) => any),
      error: null | ((c: number) => any),
  ): void;

  function createLazyFile(
      parent: string | FSNode,
      name: string,
      url: string,
      canRead: boolean,
      canWrite: boolean,
  ): FSNode;
  function createPreloadedFile(
      parent: string | FSNode,
      name: string,
      url: string,
      canRead: boolean,
      canWrite: boolean,
      onload?: () => void,
      onerror?: () => void,
      dontCreateFile?: boolean,
      canOwn?: boolean,
  ): void;
  function createDataFile(
      parent: string | FSNode,
      name: string,
      data: ArrayBufferView,
      canRead: boolean,
      canWrite: boolean,
      canOwn: boolean,
  ): FSNode;
}

declare var MEMFS: Emscripten.FileSystemType;
declare var NODEFS: Emscripten.FileSystemType;
declare var IDBFS: Emscripten.FileSystemType;

// https://emscripten.org/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.html
type StringToType<R extends any> = R extends Emscripten.JSType
? {
    number: number;
    string: string;
    array: number[] | string[] | boolean[] | Uint8Array | Int8Array;
    boolean: boolean;
    null: null;
  }[R]
: never;

type ArgsToType<T extends Array<Emscripten.JSType | null>> = Extract<
{
  [P in keyof T]: StringToType<T[P]>;
},
any[]
>;

type ReturnToType<R extends Emscripten.JSType | null> = R extends null
? null
: StringToType<Exclude<R, null>>;


interface Model {
  load_from_xml(str: string): Model;
  // MODEL_INTERFACE
  nq                    ():       number;
  nv                    ():       number;
  nu                    ():       number;
  na                    ():       number;
  nbody                 ():       number;
  njnt                  ():       number;
  ngeom                 ():       number;
  nsite                 ():       number;
  ncam                  ():       number;
  nlight                ():       number;
  nmesh                 ():       number;
  nmeshvert             ():       number;
  nmeshtexvert          ():       number;
  nmeshface             ():       number;
  nmeshgraph            ():       number;
  nskin                 ():       number;
  nskinvert             ():       number;
  nskintexvert          ():       number;
  nskinface             ():       number;
  nskinbone             ():       number;
  nskinbonevert         ():       number;
  nhfield               ():       number;
  nhfielddata           ():       number;
  ntex                  ():       number;
  ntexdata              ():       number;
  nmat                  ():       number;
  npair                 ():       number;
  nexclude              ():       number;
  neq                   ():       number;
  ntendon               ():       number;
  nwrap                 ():       number;
  nsensor               ():       number;
  nnumeric              ():       number;
  nnumericdata          ():       number;
  ntext                 ():       number;
  ntextdata             ():       number;
  ntuple                ():       number;
  ntupledata            ():       number;
  nkey                  ():       number;
  nmocap                ():       number;
  nplugin               ():       number;
  npluginattr           ():       number;
  nuser_body            ():       number;
  nuser_jnt             ():       number;
  nuser_geom            ():       number;
  nuser_site            ():       number;
  nuser_cam             ():       number;
  nuser_tendon          ():       number;
  nuser_actuator        ():       number;
  nuser_sensor          ():       number;
  nnames                ():       number;
  nM                    ():       number;
  nD                    ():       number;
  nemax                 ():       number;
  njmax                 ():       number;
  nconmax               ():       number;
  nstack                ():       number;
  nuserdata             ():       number;
  nsensordata           ():       number;
  npluginstate          ():       number;
  nbuffer               ():       number;
  qpos0                 (): Float64Array;
  qpos_spring           (): Float64Array;
  body_parentid         ():   Int32Array;
  body_rootid           ():   Int32Array;
  body_weldid           ():   Int32Array;
  body_mocapid          ():   Int32Array;
  body_jntnum           ():   Int32Array;
  body_jntadr           ():   Int32Array;
  body_dofnum           ():   Int32Array;
  body_dofadr           ():   Int32Array;
  body_geomnum          ():   Int32Array;
  body_geomadr          ():   Int32Array;
  body_simple           ():   Uint8Array;
  body_sameframe        ():   Uint8Array;
  body_pos              (): Float64Array;
  body_quat             (): Float64Array;
  body_ipos             (): Float64Array;
  body_iquat            (): Float64Array;
  body_mass             (): Float64Array;
  body_subtreemass      (): Float64Array;
  body_inertia          (): Float64Array;
  body_invweight0       (): Float64Array;
  body_gravcomp         (): Float64Array;
  body_plugin           ():   Int32Array;
  jnt_type              ():   Int32Array;
  jnt_qposadr           ():   Int32Array;
  jnt_dofadr            ():   Int32Array;
  jnt_bodyid            ():   Int32Array;
  jnt_group             ():   Int32Array;
  jnt_limited           ():   Uint8Array;
  jnt_solref            (): Float64Array;
  jnt_solimp            (): Float64Array;
  jnt_pos               (): Float64Array;
  jnt_axis              (): Float64Array;
  jnt_stiffness         (): Float64Array;
  jnt_range             (): Float64Array;
  jnt_margin            (): Float64Array;
  dof_bodyid            ():   Int32Array;
  dof_jntid             ():   Int32Array;
  dof_parentid          ():   Int32Array;
  dof_Madr              ():   Int32Array;
  dof_simplenum         ():   Int32Array;
  dof_solref            (): Float64Array;
  dof_solimp            (): Float64Array;
  dof_frictionloss      (): Float64Array;
  dof_armature          (): Float64Array;
  dof_damping           (): Float64Array;
  dof_invweight0        (): Float64Array;
  dof_M0                (): Float64Array;
  geom_type             ():   Int32Array;
  geom_contype          ():   Int32Array;
  geom_conaffinity      ():   Int32Array;
  geom_condim           ():   Int32Array;
  geom_bodyid           ():   Int32Array;
  geom_dataid           ():   Int32Array;
  geom_matid            ():   Int32Array;
  geom_group            ():   Int32Array;
  geom_priority         ():   Int32Array;
  geom_sameframe        ():   Uint8Array;
  geom_solmix           (): Float64Array;
  geom_solref           (): Float64Array;
  geom_solimp           (): Float64Array;
  geom_size             (): Float64Array;
  geom_rbound           (): Float64Array;
  geom_pos              (): Float64Array;
  geom_quat             (): Float64Array;
  geom_friction         (): Float64Array;
  geom_margin           (): Float64Array;
  geom_gap              (): Float64Array;
  geom_fluid            (): Float64Array;
  geom_rgba             (): Float32Array;
  site_type             ():   Int32Array;
  site_bodyid           ():   Int32Array;
  site_matid            ():   Int32Array;
  site_group            ():   Int32Array;
  site_sameframe        ():   Uint8Array;
  site_size             (): Float64Array;
  site_pos              (): Float64Array;
  site_quat             (): Float64Array;
  site_rgba             (): Float32Array;
  cam_mode              ():   Int32Array;
  cam_bodyid            ():   Int32Array;
  cam_targetbodyid      ():   Int32Array;
  cam_pos               (): Float64Array;
  cam_quat              (): Float64Array;
  cam_poscom0           (): Float64Array;
  cam_pos0              (): Float64Array;
  cam_mat0              (): Float64Array;
  cam_fovy              (): Float64Array;
  cam_ipd               (): Float64Array;
  light_mode            ():   Int32Array;
  light_bodyid          ():   Int32Array;
  light_targetbodyid    ():   Int32Array;
  light_directional     ():   Uint8Array;
  light_castshadow      ():   Uint8Array;
  light_active          ():   Uint8Array;
  light_pos             (): Float64Array;
  light_dir             (): Float64Array;
  light_poscom0         (): Float64Array;
  light_pos0            (): Float64Array;
  light_dir0            (): Float64Array;
  light_attenuation     (): Float32Array;
  light_cutoff          (): Float32Array;
  light_exponent        (): Float32Array;
  light_ambient         (): Float32Array;
  light_diffuse         (): Float32Array;
  light_specular        (): Float32Array;
  mesh_vertadr          ():   Int32Array;
  mesh_vertnum          ():   Int32Array;
  mesh_texcoordadr      ():   Int32Array;
  mesh_faceadr          ():   Int32Array;
  mesh_facenum          ():   Int32Array;
  mesh_graphadr         ():   Int32Array;
  mesh_vert             (): Float32Array;
  mesh_normal           (): Float32Array;
  mesh_texcoord         (): Float32Array;
  mesh_face             ():   Int32Array;
  mesh_graph            ():   Int32Array;
  skin_matid            ():   Int32Array;
  skin_group            ():   Int32Array;
  skin_rgba             (): Float32Array;
  skin_inflate          (): Float32Array;
  skin_vertadr          ():   Int32Array;
  skin_vertnum          ():   Int32Array;
  skin_texcoordadr      ():   Int32Array;
  skin_faceadr          ():   Int32Array;
  skin_facenum          ():   Int32Array;
  skin_boneadr          ():   Int32Array;
  skin_bonenum          ():   Int32Array;
  skin_vert             (): Float32Array;
  skin_texcoord         (): Float32Array;
  skin_face             ():   Int32Array;
  skin_bonevertadr      ():   Int32Array;
  skin_bonevertnum      ():   Int32Array;
  skin_bonebindpos      (): Float32Array;
  skin_bonebindquat     (): Float32Array;
  skin_bonebodyid       ():   Int32Array;
  skin_bonevertid       ():   Int32Array;
  skin_bonevertweight   (): Float32Array;
  hfield_size           (): Float64Array;
  hfield_nrow           ():   Int32Array;
  hfield_ncol           ():   Int32Array;
  hfield_adr            ():   Int32Array;
  hfield_data           (): Float32Array;
  tex_type              ():   Int32Array;
  tex_height            ():   Int32Array;
  tex_width             ():   Int32Array;
  tex_adr               ():   Int32Array;
  tex_rgb               ():   Uint8Array;
  mat_texid             ():   Int32Array;
  mat_texuniform        ():   Uint8Array;
  mat_texrepeat         (): Float32Array;
  mat_emission          (): Float32Array;
  mat_specular          (): Float32Array;
  mat_shininess         (): Float32Array;
  mat_reflectance       (): Float32Array;
  mat_rgba              (): Float32Array;
  pair_dim              ():   Int32Array;
  pair_geom1            ():   Int32Array;
  pair_geom2            ():   Int32Array;
  pair_signature        ():   Int32Array;
  pair_solref           (): Float64Array;
  pair_solimp           (): Float64Array;
  pair_margin           (): Float64Array;
  pair_gap              (): Float64Array;
  pair_friction         (): Float64Array;
  exclude_signature     ():   Int32Array;
  eq_type               ():   Int32Array;
  eq_obj1id             ():   Int32Array;
  eq_obj2id             ():   Int32Array;
  eq_active             ():   Uint8Array;
  eq_solref             (): Float64Array;
  eq_solimp             (): Float64Array;
  eq_data               (): Float64Array;
  tendon_adr            ():   Int32Array;
  tendon_num            ():   Int32Array;
  tendon_matid          ():   Int32Array;
  tendon_group          ():   Int32Array;
  tendon_limited        ():   Uint8Array;
  tendon_width          (): Float64Array;
  tendon_solref_lim     (): Float64Array;
  tendon_solimp_lim     (): Float64Array;
  tendon_solref_fri     (): Float64Array;
  tendon_solimp_fri     (): Float64Array;
  tendon_range          (): Float64Array;
  tendon_margin         (): Float64Array;
  tendon_stiffness      (): Float64Array;
  tendon_damping        (): Float64Array;
  tendon_frictionloss   (): Float64Array;
  tendon_lengthspring   (): Float64Array;
  tendon_length0        (): Float64Array;
  tendon_invweight0     (): Float64Array;
  tendon_rgba           (): Float32Array;
  wrap_type             ():   Int32Array;
  wrap_objid            ():   Int32Array;
  wrap_prm              (): Float64Array;
  actuator_trntype      ():   Int32Array;
  actuator_dyntype      ():   Int32Array;
  actuator_gaintype     ():   Int32Array;
  actuator_biastype     ():   Int32Array;
  actuator_trnid        ():   Int32Array;
  actuator_actadr       ():   Int32Array;
  actuator_actnum       ():   Int32Array;
  actuator_group        ():   Int32Array;
  actuator_ctrllimited  ():   Uint8Array;
  actuator_forcelimited ():   Uint8Array;
  actuator_actlimited   ():   Uint8Array;
  actuator_dynprm       (): Float64Array;
  actuator_gainprm      (): Float64Array;
  actuator_biasprm      (): Float64Array;
  actuator_ctrlrange    (): Float64Array;
  actuator_forcerange   (): Float64Array;
  actuator_actrange     (): Float64Array;
  actuator_gear         (): Float64Array;
  actuator_cranklength  (): Float64Array;
  actuator_acc0         (): Float64Array;
  actuator_length0      (): Float64Array;
  actuator_lengthrange  (): Float64Array;
  actuator_plugin       ():   Int32Array;
  sensor_type           ():   Int32Array;
  sensor_datatype       ():   Int32Array;
  sensor_needstage      ():   Int32Array;
  sensor_objtype        ():   Int32Array;
  sensor_objid          ():   Int32Array;
  sensor_reftype        ():   Int32Array;
  sensor_refid          ():   Int32Array;
  sensor_dim            ():   Int32Array;
  sensor_adr            ():   Int32Array;
  sensor_cutoff         (): Float64Array;
  sensor_noise          (): Float64Array;
  sensor_plugin         ():   Int32Array;
  plugin                ():   Int32Array;
  plugin_stateadr       ():   Int32Array;
  plugin_statenum       ():   Int32Array;
  plugin_attr           ():   Uint8Array;
  plugin_attradr        ():   Int32Array;
  numeric_adr           ():   Int32Array;
  numeric_size          ():   Int32Array;
  numeric_data          (): Float64Array;
  text_adr              ():   Int32Array;
  text_size             ():   Int32Array;
  text_data             ():   Uint8Array;
  tuple_adr             ():   Int32Array;
  tuple_size            ():   Int32Array;
  tuple_objtype         ():   Int32Array;
  tuple_objid           ():   Int32Array;
  tuple_objprm          (): Float64Array;
  key_time              (): Float64Array;
  name_bodyadr          ():   Int32Array;
  name_jntadr           ():   Int32Array;
  name_geomadr          ():   Int32Array;
  name_siteadr          ():   Int32Array;
  name_camadr           ():   Int32Array;
  name_lightadr         ():   Int32Array;
  name_meshadr          ():   Int32Array;
  name_skinadr          ():   Int32Array;
  name_hfieldadr        ():   Int32Array;
  name_texadr           ():   Int32Array;
  name_matadr           ():   Int32Array;
  name_pairadr          ():   Int32Array;
  name_excludeadr       ():   Int32Array;
  name_eqadr            ():   Int32Array;
  name_tendonadr        ():   Int32Array;
  name_actuatoradr      ():   Int32Array;
  name_sensoradr        ():   Int32Array;
  name_numericadr       ():   Int32Array;
  name_textadr          ():   Int32Array;
  name_tupleadr         ():   Int32Array;
  name_keyadr           ():   Int32Array;
  name_pluginadr        ():   Int32Array;
  names                 ():   Uint8Array;

}

interface mujoco extends EmscriptenModule {
  FS    : typeof FS;
  MEMFS : typeof MEMFS;
  Model: Model;
}
declare var load_mujoco: EmscriptenModuleFactory<mujoco>;
export default load_mujoco;