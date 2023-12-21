import toml
import sys


def getRequired():

    required = """
    [Simulation]
    CPUMultithreading=true
    GUI=true
    ROS=false
    friction=0.0
        
    [Scene]
    sideViewSize=[750, 450]
    
    [EndoscopeWagons]
    number=1
    position=[[-600,-1000,0]] 
    translation=[[0,0,0]]
    height=[0]
    inclination=[0]
    endoscopeTranslation=[0] 
    endoscopeRotation=[0]
    instrumentsHead=["gripper","gripper"]  # options are "gripper" or "hook"
    hysteresis=[0, 0, 0, 0]  # in mm, cables' play 
    # hysteresis: [up/down flexion endoscope, 
    #              left/right flexion endoscope, 
    #              flexion left instrument, 
    #              flexion right instrument]
    """

    return required


def getDefault():

    default = """
    [Item]
    number=1
    position=[[0,0,0]]
    rotation=[[0,0,0]]
    scale=[[1,1,1]]
    density=[7.5e-6]
    color=[[1,1,1,1]]
    withSolver=false
    withCollision=false
    collisionGroup=1 
    """
    return default


def loadConfig():

    configFilename = 'configs/default.toml'
    if len(sys.argv[1:]) == 0:
        print('\033[93m' + '\033[1m' + "scene.py/config.py/loadConfig():" + '\033[0m')
        print('\033[93m' + "  You can provide a toml config file from the configs directory. " + '\033[0m')
        print('\033[93m' + "      ex: runSofa scene.py --argv configs/scenario1.toml" + '\033[0m')
        print('\033[93m' + "  If none is provided the default configuration is being used." + '\033[0m')
        print()
    else:
        configFilename = sys.argv[1:][0]

    requiredConfig = toml.loads(getRequired())
    config = toml.load(configFilename)

    for objectKey in requiredConfig:
        if objectKey in config:
            for key in requiredConfig[objectKey]:
                if key not in config[objectKey]:
                    config[objectKey][key] = requiredConfig[objectKey][key]
        else:
            config[objectKey] = requiredConfig[objectKey]

    print('\033[92m' + "Current simulation config:" + '\033[0m')
    print(toml.dumps(config["Simulation"]))
    print()

    return config
