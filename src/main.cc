#include <cctype>
#include <cstddef>
#include <cstring>
#include <iostream>
#include "mujoco/mujoco.h"


// help
static constexpr char helpstring[] =
  "\n Usage:  compile infile outfile\n"
  "   infile can be in mjcf, urdf, mjb format\n"
  "   outfile can be in mjcf, mjb, txt format\n\n"
  " Example: compile model.xml model.mjb\n";


// deallocate and print message
int finish(const char* msg = 0, mjModel* m = 0) {
  // deallocated everything
  if (m) {
    mj_deleteModel(m);
  }

  // print message
  if (msg) {
    std::cout << msg << std::endl;
  }

  return 0;
}


// possible file types
enum {
  typeUNKNOWN = 0,
  typeXML,
  typeMJB,
  typeTXT
};


// determine file type
int filetype(const char* filename) {
  // convert to lower case for string comparison
  char lower[1000];
  std::size_t i=0;
  while (i<std::strlen(filename) && i<999) {
    lower[i] = (char)tolower(filename[i]);
    i++;
  }
  lower[i] = 0;

  // find last dot
  int dot = (int)std::strlen(lower);
  while (dot>=0 && lower[dot]!='.') {
    dot--;
  }

  // no dot found
  if (dot<0) {
    return typeUNKNOWN;
  }

  // check extension
  if (!std::strcmp(lower+dot, ".xml") || !std::strcmp(lower+dot, ".urdf")) {
    return typeXML;
  } else if (!std::strcmp(lower+dot, ".mjb")) {
    return typeMJB;
  } else if (!std::strcmp(lower+dot, ".txt")) {
    return typeTXT;
  } else {
    return typeUNKNOWN;
  }
}



// main function
int main(int argc, const char** argv) {
  // model and error
  mjModel* m = 0;
  char error[1000];

  // print help if arguments are missing
  if (argc!=3) {
    return finish(helpstring);
  }

  // determine file types
  int type1 = filetype(argv[1]);
  int type2 = filetype(argv[2]);

  // check types
  if (type1==typeUNKNOWN || type1==typeTXT ||
      type2==typeUNKNOWN || (type1==typeMJB && type2==typeXML)) {
    return finish("Illegal combination of file formats");
  }

  // check if output file exists
  std::FILE* fp = std::fopen(argv[2], "r");
  if (fp) {
    std::cout << "Output file already exists, overwrite? (Y/n) ";
    char c;
    std::cin >> c;
    if (c!='y' && c!='Y') {
      std::fclose(fp);
      return finish();
    }
  }

  // load model
  if (type1==typeXML) {
    m = mj_loadXML(argv[1], 0, error, 1000);
  } else {
    m = mj_loadModel(argv[1], 0);
  }

  // check error
  if (!m) {
    if (type1==typeXML) {
      return finish(error, 0);
    } else {
      return finish("Could not load model", 0);
    }
  }

  // save model
  if (type2==typeXML) {
    if (!mj_saveLastXML(argv[2], m, error, 1000)) {
      return finish(error, m);
    }
  } else if (type2==typeMJB) {
    mj_saveModel(m, argv[2], 0, 0);
  } else {
    mj_printModel(m, argv[2]);
  }

  // finalize
  return finish("Done", m);
}