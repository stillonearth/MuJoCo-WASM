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

  // MJMODEL_DEFINITIONS

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
      .function("getVal"          , &Model::getVal      )
      // MJMODEL_BINDINGS
;

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

  //value_object<mjModel>("mjModel")
  //    .field("ngeom"      , &mjModel::ngeom)
  //    .field("nq"         , &mjModel::nq)
  //    .field("na"         , &mjModel::na)
  //    .field("nv"         , &mjModel::nv)
  //    .field("nu"         , &mjModel::nu)
  //    .field("nbody"      , &mjModel::nbody)
  //    .field("nsensordata", &mjModel::nsensordata)
  //    .field("nmesh"      , &mjModel::nmesh)
  //    .field("nmeshvert"  , &mjModel::nmeshvert)
  //    .field("nmeshface"  , &mjModel::nmeshface);
  //value_object<mjData>("mjData");
}
