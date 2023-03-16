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
  // deallocate model
  if (m) {
    mj_deleteModel(m);
  }

  // print message
  if (msg) {
    std::printf("%s\n", msg);
  }

  return 0;
}

class Model {
public:
  Model() { m = NULL; }

  static Model load_from_xml(const std::string filename) {
    Model model;
    char error[1000] = "Could not load xml model";
    model.m = mj_loadXML(filename.c_str(), 0, error, 1000);

    if (!model.m) {
      finish(error, model.m);
    }

    return model;
  }

  mjModel *ptr() { return m; }

  mjModel getVal() { return *m; }

  std::vector<std::string> names() {
    std::vector<std::string> names;
    int j = 0;

    for (int i = 0; i < m->nnames; i++) {
      std::string name;

      while (true) {
        char c = m->names[j];
        j += 1;
        if (c == 0) {
          names.push_back(name);
          break;
        } else {
          name.push_back(c);
        }
      }
    }

    return names;
  }

  std::vector<int> mesh_vertadr() {
    std::vector<int> ret;

    for (int i = 0; i < m->nmesh; i++) {
      int addr;
      ret.push_back(m->mesh_vertadr[i]);
    }

    return ret;
  }

  std::vector<int> mesh_vertnum() {
    std::vector<int> ret;

    for (int i = 0; i < m->nmesh; i++) {
      int addr;
      ret.push_back(m->mesh_vertnum[i]);
    }

    return ret;
  }

  std::vector<int> mesh_faceadr() {
    std::vector<int> ret;

    for (int i = 0; i < m->nmesh; i++) {
      int addr;
      ret.push_back(m->mesh_faceadr[i]);
    }

    return ret;
  }

  std::vector<int> mesh_facenum() {
    std::vector<int> ret;

    for (int i = 0; i < m->nmesh; i++) {
      int addr;
      ret.push_back(m->mesh_facenum[i]);
    }

    return ret;
  }

  std::vector<int> body_parentid() {
    std::vector<int> ret;

    for (int i = 0; i < m->nbody; i++) {
      int addr;
      ret.push_back(m->body_parentid[i]);
    }

    return ret;
  }

  std::vector<int> body_geomnum() {
    std::vector<int> ret;

    for (int i = 0; i < m->nbody; i++) {
      int addr;
      ret.push_back(m->body_geomnum[i]);
    }

    return ret;
  }

  std::vector<int> body_geomadr() {
    std::vector<int> ret;

    for (int i = 0; i < m->nbody; i++) {
      int addr;
      ret.push_back(m->body_geomadr[i]);
    }

    return ret;
  }

  std::vector<int> geom_type() {
    std::vector<int> ret;

    for (int i = 0; i < m->ngeom; i++) {
      int addr;
      ret.push_back(m->geom_type[i]);
    }

    return ret;
  }

  std::vector<int> geom_bodyid() {
    std::vector<int> ret;

    for (int i = 0; i < m->ngeom; i++) {
      int addr;
      ret.push_back(m->geom_bodyid[i]);
    }

    return ret;
  }

  std::vector<int> geom_group() {
    std::vector<int> ret;

    for (int i = 0; i < m->ngeom; i++) {
      int addr;
      ret.push_back(m->geom_group[i]);
    }

    return ret;
  }

  std::vector<int> geom_contype() {
    std::vector<int> ret;

    for (int i = 0; i < m->ngeom; i++) {
      int addr;
      ret.push_back(m->geom_contype[i]);
    }

    return ret;
  }

  std::vector<std::array<float, 3>> mesh_normal() {
    std::vector<std::array<float, 3>> ret;

    for (int i = 0; i < m->nmeshvert; i++) {
      ret.push_back({m->mesh_normal[3 * i], m->mesh_normal[3 * i + 1],
                     m->mesh_normal[3 * i + 2]});
    }

    return ret;
  }

  std::vector<std::array<int, 3>> mesh_face() {
    std::vector<std::array<int, 3>> ret;

    for (int i = 0; i < m->nmeshface; i++) {
      ret.push_back({m->mesh_face[3 * i], m->mesh_face[3 * i + 1],
                     m->mesh_face[3 * i + 2]});
    }

    return ret;
  }

  std::vector<std::array<float, 3>> mesh_vert() {
    std::vector<std::array<float, 3>> ret;

    for (int i = 0; i < m->nmeshvert; i++) {
      ret.push_back({m->mesh_vert[3 * i], m->mesh_vert[3 * i + 1],
                     m->mesh_vert[3 * i + 2]});
    }

    return ret;
  }

  std::vector<int> name_meshadr() {
    std::vector<int> ret;

    for (int i = 0; i < m->nmesh; i++) {
      int addr;
      ret.push_back(m->name_meshadr[i]);
    }

    return ret;
  }

  std::vector<std::array<double, 3>> geom_size() {
    std::vector<std::array<double, 3>> ret;

    for (int i = 0; i < m->ngeom; i++) {
      ret.push_back({m->geom_size[3 * i], m->geom_size[3 * i + 1],
                     m->geom_size[3 * i + 2]});
    }

    return ret;
  }

  std::vector<std::array<double, 3>> geom_pos() {
    std::vector<std::array<double, 3>> ret;

    for (int i = 0; i < m->ngeom; i++) {
      ret.push_back(
          {m->geom_pos[3 * i], m->geom_pos[3 * i + 1], m->geom_pos[3 * i + 2]});
    }

    return ret;
  }

  std::vector<std::array<double, 3>> body_pos() {
    std::vector<std::array<double, 3>> ret;

    for (int i = 0; i < m->nbody; i++) {
      ret.push_back(
          {m->body_pos[3 * i], m->body_pos[3 * i + 1], m->body_pos[3 * i + 2]});
    }

    return ret;
  }

  std::vector<std::array<double, 4>> geom_quat() {
    std::vector<std::array<double, 4>> ret;

    for (int i = 0; i < m->ngeom; i++) {
      ret.push_back({m->geom_quat[4 * i], m->geom_quat[4 * i + 1],
                     m->geom_quat[4 * i + 2], m->geom_quat[4 * i + 3]});
    }

    return ret;
  }

  std::vector<std::array<double, 4>> body_quat() {
    std::vector<std::array<double, 4>> ret;

    for (int i = 0; i < m->nbody; i++) {
      ret.push_back({m->body_quat[4 * i], m->body_quat[4 * i + 1],
                     m->body_quat[4 * i + 2], m->body_quat[4 * i + 3]});
    }

    return ret;
  }

  val geom_rgba() {
    return val(typed_memory_view(m->ngeom * 4, m->geom_rgba));
  }

private:
  mjModel *m;
};

class State {
public:
  State(Model m) { d = mj_makeData(m.ptr()); }

  mjData *ptr() { return d; }

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

  void step() { mj_step(_model->ptr(), _state->ptr()); }

  std::vector<std::array<double, 4>> xquat() {
    std::vector<std::array<double, 4>> ret;

    for (int i = 0; i < _model->ptr()->ngeom; i++) {
      ret.push_back(
          {_state->ptr()->xquat[4 * i    ], 
           _state->ptr()->xquat[4 * i + 1],
           _state->ptr()->xquat[4 * i + 2], 
           _state->ptr()->xquat[4 * i + 3]});
    }

    return ret;
  }

  std::vector<std::array<double, 3>> xpos() {
    std::vector<std::array<double, 3>> ret;

    for (int i = 0; i < _model->ptr()->ngeom; i++) {
      ret.push_back({_state->ptr()->xpos[3 * i    ], 
                     _state->ptr()->xpos[3 * i + 1],
                     _state->ptr()->xpos[3 * i + 2]});
    }

    return ret;
  }

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
      .function("getVal", &Model::getVal)
      .function("names", &Model::names)
      .function("mesh_vertadr", &Model::mesh_vertadr)
      .function("mesh_vertnum", &Model::mesh_vertnum)
      .function("mesh_faceadr", &Model::mesh_faceadr)
      .function("mesh_facenum", &Model::mesh_facenum)
      .function("body_parentid", &Model::body_parentid)
      .function("body_geomnum", &Model::body_geomnum)
      .function("body_geomadr", &Model::body_geomadr)
      .function("geom_type", &Model::geom_type)
      .function("geom_bodyid", &Model::geom_bodyid)
      .function("geom_group", &Model::geom_group)
      .function("geom_contype", &Model::geom_contype)
      .function("mesh_normal", &Model::mesh_normal)
      .function("mesh_face", &Model::mesh_face)
      .function("mesh_vert", &Model::mesh_vert)
      .function("name_meshadr", &Model::name_meshadr)
      .function("geom_pos", &Model::geom_pos)
      .function("geom_quat", &Model::geom_quat)
      .function("geom_size", &Model::geom_size)
      .function("geom_rgba", &Model::geom_rgba)
      .function("body_pos", &Model::body_pos)
      .function("body_quat", &Model::body_quat);

  class_<State>("State")
      .constructor<Model>()
      .function("ptr", &State::ptr, allow_raw_pointers())
      .function("getVal", &State::getVal);

  class_<Simulation>("Simulation")
      .constructor<Model *, State *>()
      .function("step", &Simulation::step)
      .function("state", &Simulation::state, allow_raw_pointers())
      .function("model", &Simulation::model, allow_raw_pointers())
      .function("xquat", &Simulation::xquat)
      .function("xpos", &Simulation::xpos);

  value_object<mjModel>("mjModel")
      .field("ngeom", &mjModel::ngeom)
      .field("nq", &mjModel::nq)
      .field("na", &mjModel::na)
      .field("nv", &mjModel::nv)
      .field("nu", &mjModel::nu)
      .field("nbody", &mjModel::nbody)
      .field("nsensordata", &mjModel::nsensordata)
      .field("nmesh", &mjModel::nmesh)
      .field("nmeshvert", &mjModel::nmeshvert)
      .field("nmeshface", &mjModel::nmeshface);

  register_vector<std::string>("vector<string>");
  register_vector<int>("vector<int>");
  //register_vector<val>("vector<val>");

  register_vector<std::array<float, 3>>("vector<std::array<float,3>>");
  register_vector<std::array<int, 3>>("vector<std::array<int,3>>");
  register_vector<std::array<float, 4>>("vector<std::array<float,4>>");
  register_vector<std::array<double, 3>>("vector<std::array<double,3>>");
  register_vector<std::array<double, 4>>("vector<std::array<double,4>>");

  value_object<mjData>("mjData");
}