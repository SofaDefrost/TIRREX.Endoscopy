from scripts.instrument.params import Parameters
from scripts.instrument.controller import InstrumentController
from scripts.utils.basebeam import BaseBeam
from scripts.utils import addArticulationCenter

import os

from scripts.utils.physics import HysteresisController

path = os.path.dirname(os.path.abspath(__file__))


class Instrument(BaseBeam):
    """
    Motorized instrument. Parameters are set in the file `params.py` of the directory.
    """

    params: Parameters = Parameters()

    def __init__(self, modelling, simulation, head=None, name='Instrument', translation=[0, 0, 0], rotation=[0, 0, 0],
                 baseRigid=None, baseIndex=0, stacked=False, collisionGroup=0, hysteresis=0):
        super().__init__(modelling, simulation, self.params, name, translation, rotation, baseRigid, baseIndex,
                         stacked=stacked, collisionGroup=collisionGroup)

        self.head = head
        self.hysteresis = hysteresis

        self.__addCables()
        if self.head == 'hook':
            self.__addHook()
        elif self.head == 'gripper':
            self.__addGripper()

    def __addCables(self):
        # Added to the beam's rest position instead of the beam itself because
        # of contact problems 
        self.restBeam.addObject('RequiredPlugin', name='SoftRobots')

        cfp = self.params.passivePart
        cfo = self.params.orientablePart

        self.node.addData(name="cableL", type='float', value=0)
        self.node.addData(name="cableR", type='float', value=0)

        cables = []
        for i in range(2):
            cable = self.restBeam.addChild('Cable' + ['L', 'R'][i])
            cable.addObject('MechanicalObject', position=[[0, 0, self.params.radius - 0.1],
                                                          [0, 0, -self.params.radius + 0.1]][i]
                                                         * (cfo.nbVertebra + 1))
            cable.addObject('CableConstraint', name='Cable', hasPullPoint=False,
                            indices=list(range(cfo.nbVertebra + 1)),
                            value=self.node.findData('cable' + ['L', 'R'][i]).getLinkPath())
            cable.addObject('RigidMapping', globalToLocalCoords=False,
                            rigidIndexPerPoint=list(range(cfp.nbVertebra, cfp.nbVertebra + cfo.nbVertebra + 1)),
                            applyRestPosition=True)
            cables += [cable]
        self.node.addObject(HysteresisController(name='InstrumentHysteresisController',
                                                 cable1=self.node.cableL,
                                                 cable2=self.node.cableR,
                                                 hysteresis=self.hysteresis))

    def __addGripper(self):

        gripper = self.node.addChild('Gripper')
        gripper.addData(name='angles', type='vector<float>', value=[0., -0.])
        gripper.addObject('MechanicalObject', template='Vec1', position=[-0.0, 0.0],
                          rest_position=gripper.angles.getLinkPath())
        gripper.addObject('ArticulatedHierarchyContainer')
        gripper.addObject('RestShapeSpringsForceField', stiffness=1e12, points=[0, 1])

        rigid = gripper.addChild('RigidParts')
        self.deformable.addChild(rigid)
        rigid.addObject('MechanicalObject', template='Rigid3', showObject=False, showObjectScale=5,
                        position=[[0, 0, 0, 0, 0, 0, 1],
                                  [0, 0, 0, 0, 0, 0, 1],
                                  [0, 0, 0, 0, 0, 0, 1]])
        rigid.addObject('ArticulatedSystemMapping', container=gripper.ArticulatedHierarchyContainer.getLinkPath(),
                        indexInput2=self.node.indexExtremity.value - 1,
                        input1=gripper.MechanicalObject.getLinkPath(),
                        input2=self.deformable.MechanicalObject.getLinkPath(),
                        output=rigid.MechanicalObject.getLinkPath())

        centers = gripper.addChild('ArticulationsCenters')
        addArticulationCenter(centers, 'CenterL', 0, 1, [6.2, 0, -0.3], [0, 0, 0], 0, 0, 1, [0, 0, 1], 0)
        addArticulationCenter(centers, 'CenterR', 0, 2, [6.2, 0, 0.3], [0, 0, 0], 0, 0, 1, [0, 0, 1], 1)

        self.__addVisual(rigid, path + '/mesh/gripperBase.stl', rotation=[90, 90, 90], translation=[-3.3, 0., -0.1],
                         name='Base', index=0)
        self.__addVisual(rigid, path + '/mesh/leftPlier.stl', rotation=[90, 0, 90], translation=[0, 0., 0.28],
                         name='GripperL', index=1)
        self.__addVisual(rigid, path + '/mesh/rightPlier.stl', rotation=[90, 0, 90], translation=[0, 0., -0.28],
                         name='GripperR', index=2)
        self.__addCollision(rigid, path + '/mesh/leftPlierCollision.stl', rotation=[90, 0, 90],
                            translation=[0, 0., 0.28],
                            name='GripperL', index=1, collisionGroup=1)
        self.__addCollision(rigid, path + '/mesh/rightPlierCollision.stl', rotation=[90, 0, 90],
                            translation=[0, 0., -0.28],
                            name='GripperR', index=2, collisionGroup=1)

    def __addHook(self):

        hook = self.deformable.addChild('Hook')

        self.__addVisual(hook, path + '/mesh/hook.stl',
                         rotation=[90, -45, 90], translation=[-2., 0., 0.], index=self.node.indexExtremity.value - 1)
        self.__addCollision(hook, path + '/mesh/hookCollision.stl',
                            rotation=[90, -45, 90], translation=[-2., 0., 0.], index=self.node.indexExtremity.value - 1,
                            collisionGroup=1)

    def __addVisual(self, node, filename, rotation, translation, index, name=''):
        visu = node.addChild('Visual' + name)
        visu.addObject('MeshSTLLoader', filename=filename, rotation=rotation, translation=translation)
        visu.addObject('OglModel', src=visu.MeshSTLLoader.getLinkPath())
        visu.addObject('RigidMapping', index=index)

    def __addCollision(self, node, filename, rotation, translation, index, name='', collisionGroup=''):

        collision = node.addChild('Collision' + name)
        collision.addObject('MeshSTLLoader', filename=filename, rotation=rotation, translation=translation)
        collision.addObject('MeshTopology', src=collision.MeshSTLLoader.getLinkPath())
        collision.addObject('MechanicalObject')
        collision.addObject('TriangleCollisionModel', group=[self.collisionGroup, collisionGroup])
        collision.addObject('LineCollisionModel', group=[self.collisionGroup, collisionGroup])
        collision.addObject('PointCollisionModel', group=[self.collisionGroup, collisionGroup])
        collision.addObject('RigidMapping', index=index)

    def addController(self):
        controller = self.node.addObject(InstrumentController(self.node))
        return controller

    def getFlexionFromCableDisp(self, disp):
        # flexion is an angle in radian
        flexion = disp / self.params.hardware.pulleyDiameter * 2
        return flexion

    def getCableDispFromFlexion(self, flexion):
        # flexion is an angle in radian
        disp = flexion * self.params.hardware.pulleyDiameter / 2
        return disp


# Test scene
def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    from scripts.utils.physics import addSphereCollision

    settings, modelling, simulation = addHeader(rootnode, withConstraint=True)
    rootnode.VisualStyle.displayFlags = ['hideBehavior', 'showCollisionModels']
    rootnode.addObject('AttachBodyButtonSetting', stiffness=10)

    addSolvers(simulation, rayleighMass=0.1, iterative=False)

    instrument = Instrument(modelling, simulation, name="Instrument", head='gripper')
    instrument.addController()
    instrument.node.cableL.value = 1
    instrument.node.cableR.value = -1

    # Test attach to an external rigid
    rigid = simulation.addChild('ExternalRigidBase')
    rigid.addObject('MechanicalObject', template='Rigid3', position=[20, 20, 20, 0, 0, 0, 1])
    rigid.addObject('RestShapeSpringsForceField', name="RestShapeSpringsForceField1", points=0,
                    stiffness=1e10, angularStiffness=1e10)

    # Test contact
    addSphereCollision(rootnode, position=[150, 10, 0])
