import toml

from scripts.utils.header import addSolvers, addHeader
from scripts.utils.physics import addDeformableObject, addRigidObject
from scripts.endoscopewagon.robot import EndoscopeWagon
from scripts.endoscope.robot import Endoscope

from config import getDefault, loadConfig


def getValue(item, itemName, fieldName, index):
    """
    Get the value of item[name][index], returns a default if necessary

    Args:
        item:
        itemName:
        fieldName:
        index:

    Returns: item[name][index] or default value if any

    """
    default = toml.loads(getDefault())
    if fieldName not in item:
        if itemName in default:
            defaultItem = toml.loads(getDefault())[itemName]
            if fieldName not in defaultItem:
                defaultItem = toml.loads(getDefault())['Item']
        else:
            defaultItem = toml.loads(getDefault())['Item']
        if fieldName not in defaultItem:
            return None
        return defaultItem[fieldName][0] if type(defaultItem[fieldName]) is list else defaultItem[fieldName]

    if type(item[fieldName]) is not list:
        return item[fieldName]
    if index < len(item[fieldName]):
        return item[fieldName][index]
    return item[fieldName][0]


def createScene(rootnode, withGUI=True):
    config = loadConfig()

    settings, modelling, simulation = addHeader(rootnode, multithreading=config['Simulation']['CPUMultithreading'])

    rootnode.RuleBasedContactManager.responseParams.value = 'mu=' + str(config['Simulation']['friction'])
    rootnode.VisualStyle.displayFlags = ['hideBehavior']
    addSolvers(simulation, iterative=False)

    # Endoscope Wagons
    endoscopewagons = []
    simulation.addData(name="nbEndoscopeWagons", type='int', help='number of endoscope wagon in the scene',
                       value=config['EndoscopeWagons']['number'])
    itemsName = 'EndoscopeWagons'
    items = config[itemsName]
    for i in range(items['number']):
        endoscopewagon = EndoscopeWagon(modelling, simulation, name="EndoscopeWagon" + str(i + 1),
                                        position=getValue(items, itemsName, 'position', i),
                                        translation=getValue(items, itemsName, 'translation', i),
                                        height=getValue(items, itemsName, 'height', i),
                                        inclination=getValue(items, itemsName, 'inclination', i),
                                        endoscopeTranslation=getValue(items, itemsName, 'endoscopeTranslation', i),
                                        endoscopeRotation=getValue(items, itemsName, 'endoscopeRotation', i),
                                        instrumentsHead=items['instrumentsHead'],
                                        hysteresis=items['hysteresis']
                                        )
        endoscopewagons.append(endoscopewagon)

    if simulation.nbEndoscopeWagons.value > 0:
        if withGUI:
            endoscopewagons[0].endoscope.addCamera(config['Scene']['sideViewSize'])
            endoscopewagons[0].endoscope.addLeds()
        framesOI = endoscopewagons[0].endoscope.addFramesOfInterest(showFrames=False)

        if config['Simulation']['GUI']:
            endoscopewagons[0].addGUIController()

        if config['Simulation']['ROS']:
            from controller import SofaROSInterface
            rootnode.addObject(SofaROSInterface(wagon=endoscopewagons[0], framesOI=framesOI, camera=rootnode.Camera))

    # Endoscope
    if "Endoscopes" in config:
        endoscopes = []
        simulation.addData(name="nbEndoscopes", type='int', help='number of endoscope wagon in the scene',
                           value=config['Endoscopes']['number'])
        itemsName = 'Endoscopes'
        items = config[itemsName]
        for i in range(items['number']):
            endoscope = Endoscope(modelling, simulation, name="Endoscope" + str(i + 1),
                                  instrumentsHead=config['Endoscopes']['instrumentsHead'],
                                  hysteresis=config['Endoscopes']['hysteresis'])
            endoscopes.append(endoscope)
        endoscopes[0].addCamera()
        if withGUI:
            endoscopes[0].addLeds()
        endoscopes[0].addController()
        endoscopes[0].addFramesOfInterest(showFrames=False)

    # Items of the environment
    if "Items" in config:
        for itemsName, items in config["Items"].items():
            node = modelling.addChild(itemsName)
            for i in range(getValue(items, itemsName, 'number', 0)):
                if getValue(items, itemsName, 'type', 0) == "Rigid":
                    addRigidObject(node, getValue(items, itemsName, 'filename', i),
                                   collisionFilename=getValue(items, itemsName, 'collisionFilename', i),
                                   translation=getValue(items, itemsName, 'position', i),
                                   rotation=getValue(items, itemsName, 'rotation', i),
                                   scale=getValue(items, itemsName, 'scale', i),
                                   density=getValue(items, itemsName, 'density', i),
                                   color=getValue(items, itemsName, 'color', i),
                                   name=itemsName + str(i),
                                   withSolver=getValue(items, itemsName, 'withSolver', i),
                                   withCollision=getValue(items, itemsName, 'withCollision', i),
                                   collisionGroup=getValue(items, itemsName, 'collisionGroup', i))
                elif getValue(items, itemsName, 'type', 0) == "Deformable":
                    addDeformableObject(node, getValue(items, itemsName, 'filename', i),
                                        collisionFilename=getValue(items, itemsName, 'collisionFilename', i),
                                        poissonRatio=getValue(items, itemsName, 'poissonRatio', i),
                                        youngModulus=getValue(items, itemsName, 'youngModulus', i),
                                        translation=getValue(items, itemsName, 'position', i),
                                        rotation=getValue(items, itemsName, 'rotation', i),
                                        scale=getValue(items, itemsName, 'scale', i),
                                        totalMass=getValue(items, itemsName, 'totalMass', i),
                                        color=getValue(items, itemsName, 'color', i),
                                        name=itemsName + str(i),
                                        withSolver=getValue(items, itemsName, 'withSolver', i),
                                        withCollision=getValue(items, itemsName, 'withCollision', i),
                                        collisionGroup=getValue(items, itemsName, 'collisionGroup', i))
                else:
                    print(
                        f"[ERROR] Object type {getValue(items, itemsName, 'type', 0)} not recognized. It should either be Rigid of Deformable.")
