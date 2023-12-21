import toml
import os
import params


# Utils to render the animation in Blender
# We use the script `blender_importer.py` from `https://gitlab.inria.fr/imagine_rennes/blender_toolbox`
# to set up the animation in Blender. This script (`blender_importer.py`) takes a toml file as input.
# See blender_importer.example.toml


# Adds the toml description of a given object to the static params.scene.blenderAnimationConfig
def addObjectConfig(node, name, indices, template, objectType, meshFilename, translation=[0, 0, 0], rotation=[0, 0, 0],
                    scale=[1, 1, 1]):
    objectConfig = {
        'mesh': os.path.realpath(meshFilename),
        'type': objectType,
        'name': name,
        'scale': [float(s) for s in scale]
    }
    if objectType == 'static':  # no animation
        objectConfig['translation'] = [float(t) for t in translation]
        objectConfig['rotation'] = [float(r) for r in rotation]

    objectConfig['monitor'] = params.scene.outputDir + name + '_x.txt'
    params.scene.blenderAnimationConfig['objects'].append(objectConfig)
    node.addObject('Monitor', name="monitor" + name, template=template, listening=True, ExportPositions=True,
                   ExportVelocities=False,
                   ExportForces=False, indices=indices, fileName=params.scene.outputDir + name)


# Export params.scene.blenderAnimationConfig
def exportAnimationConfig(filename):
    with open(filename, 'w+') as f:
        toml.dump(params.scene.blenderAnimationConfig, f)
