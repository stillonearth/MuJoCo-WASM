// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <emscripten/fetch.h>
#include <emscripten/bind.h>

#include "mujoco/mujoco.h"

using namespace emscripten;

int finish(const char *msg = NULL, mjModel *m = NULL)
{
  // deallocate model
  if (m)
  {
    mj_deleteModel(m);
  }

  // print message
  if (msg)
  {
    std::printf("%s\n", msg);
  }

  return 0;
}

class Model
{
public:
  Model()
  {
    m = NULL;
  }

  static Model load_from_xml(const std::string filename)
  {
    Model model;
    char error[1000] = "Could not load xml model";
    model.m = mj_loadXML(filename.c_str(), 0, error, 1000);

    if (!model.m)
    {
      finish(error, model.m);
    }

    return model;
  }

  mjModel *ptr()
  {
    return m;
  }

  mjModel val()
  {
    return *m;
  }

private:
  mjModel *m;
};

class State
{
public:
  State(Model m)
  {
    d = mj_makeData(m.ptr());
  }

  mjData *ptr()
  {
    return d;
  }

  mjData val()
  {
    return *d;
  }

private:
  mjData *d;
};

class Simulation
{
public:
  Simulation(Model *m, State *s)
  {
    _model = m;
    _state = s;
  }

  State *state()
  {
    return _state;
  }

  Model *model()
  {
    return _model;
  }

  void step()
  {
    mj_step(_model->ptr(), _state->ptr());
  }

private:
  Model *_model;
  State *_state;
};

// main function
int main(int argc, char **argv)
{
  std::printf("MuJoCo version: %d\n\n", mj_version());
  return 0;
}

EMSCRIPTEN_BINDINGS(mujoco_wasm)
{
  class_<Model>("Model")
      .constructor<>()
      .class_function("load_from_xml", &Model::load_from_xml)
      .function("ptr", &Model::ptr, allow_raw_pointers())
      .function("val", &Model::val);

  class_<State>("State")
      .constructor<Model>()
      .function("ptr", &State::ptr, allow_raw_pointers())
      .function("val", &State::val);

  class_<Simulation>("Simulation")
      .constructor<Model *, State *>()
      .function("step", &Simulation::step)
      .function("state", &Simulation::state, allow_raw_pointers())
      .function("model", &Simulation::model, allow_raw_pointers());

  value_object<mjModel>("mjModel");
  value_object<mjData>("mjData");
}