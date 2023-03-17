mjModel_definitions    = ""
emscripten_bindings    = ""
typescript_definitions = ""

types_to_array_types = {"int":"Int32Array", "double":"Float64Array", "float": "Float32Array", "byte": "Uint8Array", "char": "Uint8Array"}

# Parse mjx Macro to get the emscripten bindings and typescript definitions
with open("include/mujoco/mjxmacro.h") as f:
    lines = f.readlines()

    parse_pointers = False; parse_ints = True
    for line in lines:
        if parse_pointers and line.startswith("    X("):
            elements = line.strip("    X(").split(""")""")[0].strip().split(",")
            elements = [e.strip().replace("mjtNum", "double").replace("mjtByte", "byte") for e in elements]
            is_int = elements[3].startswith("mj")
            try:
                multiplier = int(elements[3])
                is_int = True
            except:
                pass
            if is_int:
                mjModel_definitions    += '  val '+elements[1].ljust(22)+'() { return val(typed_memory_view(m->'+elements[2].ljust(15)+' * '+elements[3].ljust(9)+', m->'+elements[1].ljust(22)+' )); }\n'
                emscripten_bindings    += '      .function('+('"'+elements[1]+'"').ljust(24)+', &Model::'+elements[1].ljust(22)+')\n'
                typescript_definitions += '  '+elements[1].ljust(22)+'(): '+types_to_array_types[elements[0]].rjust(12)+';\n'

            else:
                mjModel_definitions += '//val '+elements[1].ljust(22)+'() { return val(typed_memory_view(m->'+elements[2].ljust(15)+' * '+elements[3].ljust(9)+', m->'+elements[1].ljust(22)+' )); }\n'
                emscripten_bindings += '    //.function('+('"'+elements[1]+'"').ljust(24)+', &Model::'+elements[1].ljust(22)+')\n'
        else:
            parse_pointers = False

        if parse_ints and line.startswith("    X("):
            name = line.strip("    X(").split(""")""")[0].strip()
            mjModel_definitions    += '  int '+name.ljust(14)+'() { return m->'+name.ljust(14)+'; }\n'
            emscripten_bindings    += '      .function('+('"'+name+'"').ljust(24)+', &Model::'+name.ljust(22)+')\n'
            typescript_definitions += '  '+name.ljust(22)+'(): '+('number').rjust(12)+';\n'
        else:
            parse_ints = False

        if "#define MJMODEL_INTS" in line:
            parse_ints = True
            continue
        if "#define MJMODEL_POINTERS" in line:
            parse_pointers = True
            continue

# Insert our auto-generated bindings into our template files
with open("src/main.template.cc") as f:
    content = f.read()
    content = content.replace("// MJMODEL_DEFINITIONS", "// MJMODEL_DEFINITIONS\n"+mjModel_definitions)
    content = content.replace("// MJMODEL_BINDINGS", "// MJMODEL_BINDINGS\n"+emscripten_bindings)
    with open("src/main.genned.cc", mode="w") as f:
        f.write(content)

with open("src/mujoco_wasm.template.d.ts") as f:
    content = f.read()
    content = content.replace("// MODEL_INTERFACE", "// MODEL_INTERFACE\n"+typescript_definitions)
    with open("public/mujoco_wasm.d.ts", mode="w") as f:
        f.write(content)
