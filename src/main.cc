#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"

#include <emscripten/fetch.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <vector>

using namespace emscripten;

int finish(const char *msg = NULL, mjModel *m = NULL) {
  if (m  ) { mj_deleteModel(m); }
  if (msg) { std::printf("%s\n", msg); }
  return 0;
}

class Model {
public:
  Model() { m = NULL; }

  static Model load_from_xml(const std::string filename) {
    Model model;
    char error[1000] = "Could not load xml model";
    model.m = mj_loadXML(filename.c_str(), 0, error, 1000);
    if (!model.m) { finish(error, model.m); }
    return model;
  }

  mjModel *ptr     () { return m; }
  mjModel getVal   () { return *m; }

  val names        () { return val(typed_memory_view(m->nnames       , m->names        )); }
  val mesh_vertadr () { return val(typed_memory_view(m->nmesh        , m->mesh_vertadr )); }
  val mesh_vertnum () { return val(typed_memory_view(m->nmesh        , m->mesh_vertnum )); }
  val mesh_faceadr () { return val(typed_memory_view(m->nmesh        , m->mesh_faceadr )); }
  val mesh_facenum () { return val(typed_memory_view(m->nmesh        , m->mesh_facenum )); }
  val body_parentid() { return val(typed_memory_view(m->nbody        , m->body_parentid)); }
  val body_geomnum () { return val(typed_memory_view(m->nbody        , m->body_geomnum )); }
  val body_geomadr () { return val(typed_memory_view(m->nbody        , m->body_geomadr )); }
  val geom_type    () { return val(typed_memory_view(m->ngeom        , m->geom_type    )); }
  val geom_bodyid  () { return val(typed_memory_view(m->ngeom        , m->geom_bodyid  )); }
  val geom_group   () { return val(typed_memory_view(m->ngeom        , m->geom_group   )); }
  val geom_contype () { return val(typed_memory_view(m->ngeom        , m->geom_contype )); }
  val mesh_vert    () { return val(typed_memory_view(m->nmeshvert * 3, m->mesh_vert    )); }
  val mesh_normal  () { return val(typed_memory_view(m->nmeshvert * 3, m->mesh_normal  )); }
  val mesh_face    () { return val(typed_memory_view(m->nmeshface * 3, m->mesh_face    )); }
  val name_meshadr () { return val(typed_memory_view(m->nmesh        , m->name_meshadr )); }
  val geom_size    () { return val(typed_memory_view(m->ngeom     * 3, m->geom_size    )); }
  val geom_pos     () { return val(typed_memory_view(m->ngeom     * 3, m->geom_pos     )); }
  val geom_quat    () { return val(typed_memory_view(m->nbody     * 4, m->geom_quat    )); }
  val body_pos     () { return val(typed_memory_view(m->nbody     * 3, m->body_pos     )); }
  val body_quat    () { return val(typed_memory_view(m->nbody     * 4, m->body_quat    )); }
  val geom_rgba    () { return val(typed_memory_view(m->ngeom     * 4, m->geom_rgba    )); }

private:
  mjModel *m;
};

class State {
public:
  State(Model m)  { d = mj_makeData(m.ptr()); }
  mjData *ptr  () { return d; }
  mjData getVal() { return *d; }

private:
  mjData *d;
};

class Simulation {
public:
  Simulation(Model *m, State *s) {
    _model = m;
    _state = s;
  }

  State *state() { return _state; }
  Model *model() { return _model; }
  void    step() { mj_step(_model->ptr(), _state->ptr()); }

  val    xquat() { return val(typed_memory_view(_model->ptr()->nbody * 4, _state->ptr()->xquat)); }
  val    xpos () { return val(typed_memory_view(_model->ptr()->nbody * 3, _state->ptr()->xpos )); }

private:
  Model *_model;
  State *_state;
};

// main function
int main(int argc, char **argv) {
  std::printf("MuJoCo version: %d\n\n", mj_version());
  return 0;
}

EMSCRIPTEN_BINDINGS(mujoco_wasm) {
  class_<Model>("Model")
      .constructor<>()
      .class_function("load_from_xml", &Model::load_from_xml)
      .function("ptr", &Model::ptr, allow_raw_pointers())
      .function("getVal"       , &Model::getVal)
      .function("names"        , &Model::names)
      .function("mesh_vertadr" , &Model::mesh_vertadr)
      .function("mesh_vertnum" , &Model::mesh_vertnum)
      .function("mesh_faceadr" , &Model::mesh_faceadr)
      .function("mesh_facenum" , &Model::mesh_facenum)
      .function("body_parentid", &Model::body_parentid)
      .function("body_geomnum" , &Model::body_geomnum)
      .function("body_geomadr" , &Model::body_geomadr)
      .function("geom_type"    , &Model::geom_type)
      .function("geom_bodyid"  , &Model::geom_bodyid)
      .function("geom_group"   , &Model::geom_group)
      .function("geom_contype" , &Model::geom_contype)
      .function("mesh_normal"  , &Model::mesh_normal)
      .function("mesh_face"    , &Model::mesh_face)
      .function("mesh_vert"    , &Model::mesh_vert)
      .function("name_meshadr" , &Model::name_meshadr)
      .function("geom_pos"     , &Model::geom_pos)
      .function("geom_quat"    , &Model::geom_quat)
      .function("geom_size"    , &Model::geom_size)
      .function("geom_rgba"    , &Model::geom_rgba)
      .function("body_pos"     , &Model::body_pos)
      .function("body_quat"    , &Model::body_quat);

  class_<State>("State")
      .constructor<Model>()
      .function("ptr"   , &State::ptr, allow_raw_pointers())
      .function("getVal", &State::getVal);

  class_<Simulation>("Simulation")
      .constructor<Model *, State *>()
      .function("step" , &Simulation::step)
      .function("state", &Simulation::state, allow_raw_pointers())
      .function("model", &Simulation::model, allow_raw_pointers())
      .function("xquat", &Simulation::xquat)
      .function("xpos" , &Simulation::xpos);

  value_object<mjModel>("mjModel")
      .field("ngeom"      , &mjModel::ngeom)
      .field("nq"         , &mjModel::nq)
      .field("na"         , &mjModel::na)
      .field("nv"         , &mjModel::nv)
      .field("nu"         , &mjModel::nu)
      .field("nbody"      , &mjModel::nbody)
      .field("nsensordata", &mjModel::nsensordata)
      .field("nmesh"      , &mjModel::nmesh)
      .field("nmeshvert"  , &mjModel::nmeshvert)
      .field("nmeshface"  , &mjModel::nmeshface);

  value_object<mjData>("mjData");
}
