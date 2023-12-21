from scripts.utils import addArticulationCenter
from scripts.endoscopewagon.controller import WagonController
from scripts.endoscopewagon.robotgui import EndoscopeWagonGUI
from scripts.endoscope.robot import Endoscope
from scripts.endoscope.params import Parameters as EndoscopeParameters
from scripts.instrument.params import Parameters as InstrumentParameters

import os

path = os.path.dirname(os.path.abspath(__file__))


def addVisual(node, name, filename, index, color=[1, 1, 1, 1], scale=[1, 1, 1], rotation=[0, 0, 0],
              translation=[0, 0, 0]):
    visual = node.addChild('VisualModel' + name)
    visual.addObject('MeshOBJLoader', name='loader', filename=filename, scale3d=scale, rotation=rotation,
                     translation=translation)
    visual.addObject('MeshTopology', src=visual.loader.getLinkPath())
    visual.addObject('OglModel', color=color)
    visual.addObject('RigidMapping', index=index)


class EndoscopeWagon:
    """
    Adjustable wagon (height and inclination) with an endoscope that can translate and rotate from the wagon's end effector.

    Arguments:
        height: distance
        inclination: angle in radian
        endoscopeTranslation: distance
        endoscopeRotation: angle in radian
    """

    endoscopeParams = EndoscopeParameters()
    instrumentParams = InstrumentParameters()

    def __init__(self, modelling, simulation, name='EndoscopeWagon', position=[0, 0, 0], translation=[0, 0, 0],
                 height=0., inclination=0.,
                 endoscopeTranslation=0., endoscopeRotation=0., instrumentsHead=['gripper', 'gripper'],
                 hysteresis=[0, 0, 0, 0]):
        self.name = name
        self.modelling = modelling
        self.simulation = simulation
        self.position = position
        self.translation = translation

        self.height = height
        self.inclination = inclination
        self.endoscopeTranslation = endoscopeTranslation
        self.endoscopeRotation = endoscopeRotation

        self.node = self.modelling.addChild(self.name)
        self.simulation.addChild(self.node)

        self.__addWagon()
        self.__addEndoscope(instrumentsHead, hysteresis)

    def __addWagon(self):
        self.node.addData(name='dofs', type='vector<float>',
                          help='height and angle of the wagon, translation and rotation of endoscope. ',
                          value=[self.height, self.inclination, self.endoscopeTranslation, self.endoscopeRotation])

        nbDofs = 4

        self.node.addObject('MechanicalObject', rest_position=self.node.dofs.getLinkPath(), template='Vec1')
        self.node.addObject('ArticulatedHierarchyContainer')
        self.node.addObject('UniformMass', totalMass=0.5)
        self.node.addObject('RestShapeSpringsForceField', stiffness=1e10, points=list(range(nbDofs)))

        rigid = self.node.addChild('Rigid')
        rigid.addObject('MechanicalObject', template='Rigid3', showObject=True, showObjectScale=15,
                        position=[self.position + [0, 0, 0, 1]] + [[0, 0, 0, 0, 0, 0, 1]] * nbDofs, translation=self.translation)
        rigid.addObject('ArticulatedSystemMapping', container=self.node.ArticulatedHierarchyContainer.getLinkPath(),
                        input1=self.node.MechanicalObject.getLinkPath(),
                        output=rigid.MechanicalObject.getLinkPath())

        addVisual(node=rigid, name='Base', filename=path + '/mesh/wagonBase.obj', index=0, scale=[1000, 1000, 1000],
                  rotation=[0, 90, 0], translation=[600, -250, 710])
        addVisual(node=rigid, name='Height', filename=path + '/mesh/wagonHeight.obj', index=1, scale=[1000, 1000, 1000],
                  rotation=[0, 90, 0], translation=[600, -700, 710])
        addVisual(node=rigid, name='RotationPart', filename=path + '/mesh/wagonHeight.obj', index=1,
                  scale=[1200, 700, 1400], rotation=[0, 90, 0], translation=[840, -170, 850])
        addVisual(node=rigid, name='Arm', filename=path + '/mesh/wagonArm.obj', index=2, scale=[1000, 1000, 1000],
                  rotation=[0, 90, 100], translation=[150, -950, 710])

        centers = self.node.addChild('ArticulationsCenters')
        addArticulationCenter(centers, 'CenterHeight', 0, 1, [0, 500, 0], [0, 0, 0], 0, 1, 0, [0, 1, 0], 0)
        addArticulationCenter(centers, 'CenterInclination', 1, 2, [0, 500, 0], [-600, 0, 0], 0, 0, 1, [0, 0, 1], 1)
        addArticulationCenter(centers, 'CenterEndoscopeTranslation', 2, 3, [0, 0, 0], [-10, 0, 0], 0, 1, 0, [1, 0, 0], 2)
        addArticulationCenter(centers, 'CenterEndoscopeRotation', 3, 4, [0, 0, 0], [-10, 0, 0], 0, 0, 1, [1, 0, 0], 3)

    def __addEndoscope(self, instrumentsHead, hysteresis):
        self.endoscope = Endoscope(self.modelling, self.simulation, translation=self.translation,
                                   baseRigid=self.node.Rigid,
                                   baseIndex=4, instrumentsHead=instrumentsHead, hysteresis=hysteresis)

    def addController(self):
        self.node.addObject(WagonController(self.node))

    def addGUIController(self):
        self.node.addObject(EndoscopeWagonGUI(wagon=self,
                                              height=self.height, inclination=self.inclination,
                                              endoscopeTranslation=self.endoscopeTranslation,
                                              endoscopeRotation=self.endoscopeRotation))


# Test scene
def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    rootnode.VisualStyle.displayFlags = ['hideBehavior']
    addSolvers(simulation)

    wagon = EndoscopeWagon(modelling, simulation, inclination=0, position=[-600, -1000, 0])
    wagon.endoscope.canals.angles.value = [0.5, -0.5, 0, 0]
    wagon.addGUIController()
