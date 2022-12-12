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

#include "mujoco/mujoco.h"

// model and per-thread data
mjModel *m;
mjData *d;

// timer
std::chrono::system_clock::time_point tm_start;
mjtNum gettm(void)
{
  std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - tm_start;
  return elapsed.count();
}

// deallocate and print message
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

// main function
int main(int argc, char **argv)
{
  int nstep = 100;

  // load model
  char error[1000] = "Could not load xml model";
  m = mj_loadXML("simple.xml", 0, error, 1000);
  if (!m)
  {
    return finish(error, m);
  }

  d = mj_makeData(m);
  if (!d)
  {
    return finish("Could not allocate mjData", m);
  }

  std::printf("number of bodies: %d\n\n", m->nbody);

  // install timer callback for profiling if requested
  tm_start = std::chrono::system_clock::now();

  for (int i = 0; i < nstep; i++)
  {
    mj_step(m, d);
    for (int j = 0; j < 2 /*m->nbody*/; j++)
    {
      std::printf("qpos: %f, %f, %f\n", d->xpos[j * 3], d->xpos[j * 3 + 1], d->xpos[j * 3 + 2]);
    }
  }

  mj_deleteData(d);
  return finish();
}
