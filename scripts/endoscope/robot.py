from scripts.instrument.params import Parameters as InstrumentParameters
from scripts.endoscope.params import Parameters
from scripts.endoscope.controller import EndoscopeController, LedController

from scripts.utils.basebeam import BaseBeam
from scripts.utils import addArticulationCenter
from scripts.instrument.robot import Instrument
from splib3.numerics import vadd
from scripts.utils.physics import HysteresisController

import Sofa
import os

path = os.path.dirname(os.path.abspath(__file__))


class Endoscope(BaseBeam):
    """
    Motorized endoscope. Parameters are set in the file `params.py` of the directory.
    """

    params: Parameters = Parameters()
    paramsInstrument: InstrumentParameters = InstrumentParameters()

    def __init__(self, modelling, simulation, name='Endoscope', translation=[0, 0, 0], rotation=[0, 0, 0],
                 baseRigid=None, baseIndex=0, collisionGroup=0, instrumentsHead=['gripper', 'gripper'],
                 hysteresis=[0, 0, 0, 0]):
        super().__init__(modelling, simulation, self.params, name, translation, rotation, baseRigid, baseIndex,
                         collisionGroup=collisionGroup)

        self.instruments = []
        self.instrumentsHead = instrumentsHead
        self.translation = translation
        self.rotation = rotation
        self.hysteresis = hysteresis

        self.__addCables()
        self.__addLateralCanals()
        self.__addInstruments()
        self.__addVisuals()

    def __addCables(self):

        self.restBeam.addObject('RequiredPlugin', name='SoftRobots')

        cfp = self.params.passivePart
        cfo = self.params.orientablePart

        self.node.addData(name="cableU", type='float', value=0)
        self.node.addData(name="cableD", type='float', value=0)
        self.node.addData(name="cableL", type='float', value=0)
        self.node.addData(name="cableR", type='float', value=0)

        for i in range(4):
            name = ['U', 'D', 'L', 'R'][i]
            cable = self.restBeam.addChild('Cable' + name)
            cable.addObject('MechanicalObject', position=[[0, self.params.radius - 1, 0],
                                                          [0, -self.params.radius + 1, 0],
                                                          [0, 0, self.params.radius - 1],
                                                          [0, 0, -self.params.radius + 1]][i]
                                                         * (cfo.nbVertebra + 1))
            cable.addObject('CableConstraint', name='Cable', hasPullPoint=False,
                            indices=list(range(cfo.nbVertebra + 1)),
                            value=self.node.getData('cable' + name).getLinkPath())
            cable.addObject('RigidMapping', globalToLocalCoords=False,
                            rigidIndexPerPoint=list(range(cfp.nbVertebra, cfp.nbVertebra + cfo.nbVertebra + 1)),
                            applyRestPosition=True)

        self.node.addObject((HysteresisController(name="EndoscopeHysteresisUDController",
                                                  cable1=self.node.cableU,
                                                  cable2=self.node.cableD,
                                                  hysteresis=self.hysteresis[0])))

        self.node.addObject((HysteresisController(name="EndoscopeHysteresisLRController",
                                                  cable1=self.node.cableL,
                                                  cable2=self.node.cableR,
                                                  hysteresis=self.hysteresis[1])))

    def __addLateralCanals(self):

        self.rigidBaseCanals = self.node.addChild('RigidBaseCanals')
        self.rigidBaseCanals.addObject('MechanicalObject', position=self.rod.MechanicalObject.position.value[-1],
                                       translation=self.translation, rotation=self.rotation, template='Rigid3')

        distance = self.deformable.addChild('Distance')
        self.rigidBaseCanals.addChild(distance)
        distance.addObject('MechanicalObject', template='Rigid3', rest_position=[0, 0, 0, 0, 0, 0, 1])
        distance.addObject('RestShapeSpringsForceField', points=0, stiffness=1e12, angularStiffness=1e12)
        distance.addObject('RigidDistanceMapping',
                           input1=self.deformable.MechanicalObject.getLinkPath(),
                           input2=self.rigidBaseCanals.MechanicalObject.getLinkPath(),
                           output=distance.MechanicalObject.getLinkPath(),
                           first_point=self.node.indexExtremity.value - 1,
                           second_point=0)

        self.canals = self.node.addChild('OrientableCanals')
        self.canals.addData(name='angles', type='vector<float>', value=[0, 0, 0, 0])
        self.canals.addObject('MechanicalObject', template='Vec1', position=[0, 0, 0, 0],
                              rest_position=self.canals.angles.getLinkPath())
        self.canals.addObject('ArticulatedHierarchyContainer')
        self.canals.addObject('UniformMass', totalMass=0.01)
        self.canals.addObject('RestShapeSpringsForceField', stiffness=1e10, points=[0, 1, 2, 3])

        rigid = self.canals.addChild('CanalsRigid')
        self.rigidBaseCanals.addChild(rigid)
        rigid.addObject('MechanicalObject', template='Rigid3', showObject=False, showObjectScale=5,
                        position=[[0, 0, 0, 0, 0, 0, 1]] * 5)
        rigid.addObject('ArticulatedSystemMapping', container=self.canals.ArticulatedHierarchyContainer.getLinkPath(),
                        input1=self.canals.MechanicalObject.getLinkPath(),
                        input2=self.rigidBaseCanals.MechanicalObject.getLinkPath(),
                        indexInput2=0,
                        output=rigid.MechanicalObject.getLinkPath(), applyRestPosition=True)

        centers = self.canals.addChild('ArticulationsCenters')
        t = self.params.canals.positionInstrumentL
        addArticulationCenter(centers, 'CenterL', 0, 1, [11 + t[0], t[1], t[2]], [0, 0, 0], 0, 0, 1, [0, 1, 0], 0)
        addArticulationCenter(centers, 'CenterLRotation', 1, 3, [0, 0, 0], [0, 0, 0], 0, 0, 1, [1, 0, 0], 2)
        t = self.params.canals.positionInstrumentR
        addArticulationCenter(centers, 'CenterR', 0, 2, [11 + t[0], t[1], t[2]], [0, 0, 0], 0, 0, 1, [0, 1, 0], 1)
        addArticulationCenter(centers, 'CenterRRotation', 2, 4, [0, 0, 0], [0, 0, 0], 0, 0, 1, [1, 0, 0], 3)

    def __addInstruments(self):
        for i in range(2):
            t = self.translation
            t = vadd(t, self.rod.MechanicalObject.position.value[-1][0:3])
            t = vadd(t, [self.params.canals.positionInstrumentL, self.params.canals.positionInstrumentR][i])
            instrument = Instrument(self.modelling, self.simulation, head=self.instrumentsHead[i],
                                    name='Instrument' + ['L', 'R'][i],
                                    baseRigid=self.canals.CanalsRigid, baseIndex=i + 3,
                                    translation=[t[0] + 11, t[1], t[2]], rotation=[0, 0, 0], stacked=True,
                                    hysteresis=self.hysteresis[i + 2])

            self.instruments += [instrument]

    def __addVisuals(self):

        def addVisual(node, name, filename, index,
                      rotation=[0., 0., 0.], translation=[0., 0., 0.]):
            visu = node.addChild('Visu' + name)
            visu.addObject('MeshSTLLoader', filename=filename, rotation=rotation, translation=translation)
            visu.addObject('OglModel', src=visu.MeshSTLLoader.getLinkPath())
            visu.addObject('RigidMapping', index=index)

        addVisual(self.rod, name='Head', filename=path + '/mesh/head.stl', index=self.node.indexExtremity.value,
                  rotation=[0, 90, 0])
        addVisual(self.canals.CanalsRigid, name='LeftJaw', filename=path + '/mesh/leftjaw.stl', index=1,
                  rotation=[0, 90, -90], translation=[0.32, 4.35, -0.85])
        addVisual(self.canals.CanalsRigid, name='RightJaw', filename=path + '/mesh/rightjaw.stl', index=2,
                  rotation=[0, -90, 90], translation=[0.32, 4.35, 0.85])

    def addCamera(self, showView=True, sideViewSize=[750, 450]):
        """
        Adds a camera at the tip of the endoscope.
        Args:
            showView: if false, the view won't show
            sideViewSize:

        Returns:
            the node of the camera
        """

        camera = self.rod.addChild('Camera')
        camera.addObject('MechanicalObject', template='Rigid3d', position=[27, 5, 0, 0, -0.707, 0, 0.707],
                         showObject=False, showObjectScale=10)
        camera.addObject('RigidMapping', name="mapping", index=self.node.indexExtremity.value)

        cameraVisu = camera.addChild('CameraVisu')
        cameraVisu.addObject('MeshOBJLoader', filename='mesh/cylinder.obj', name='loader', scale3d=[2, 0.3, 2],
                             rotation=[90, 0, 0])
        cameraVisu.addObject('OglModel', src='@loader', color=[0.1, 0.1, 0.1])
        cameraVisu.addObject('RigidMapping', name="mapping", index=0)

        self.node.getRoot().addObject('OglViewport', screenSize=sideViewSize, name='Camera',
                                      enabled=showView,
                                      swapMainView=True, zNear=5,
                                      zFar=-10, fovy=55,
                                      cameraRigid=camera.MechanicalObject.position.getLinkPath(), useFBO=False)

        return camera

    def addLeds(self):
        """
        Adds two leds at the tip of the endoscope.
        Returns:
            the node of the leds
        """

        leds = self.rod.addChild('Leds')
        leds.addObject('MechanicalObject', template='Rigid3d',
                      position=[[26, 5, 4.5, 0, 0, 0, 1], [26, 5, -4.5, 0, 0, 0, 1]], showObject=False,
                      showObjectScale=10)
        leds.addObject('RigidMapping', name="mapping", index=self.node.indexExtremity.value)

        for i in range(2):
            ledVisu = leds.addChild('LedVisu' + ['L', 'R'][i])
            ledVisu.addObject('MeshOBJLoader', filename='mesh/cylinder.obj', name='loader', scale3d=[1, 0.3, 1],
                              rotation=[0, 0, 90])
            ledVisu.addObject('OglModel', src='@loader',
                              material="Default Diffuse 1 1 1 1 1 Ambient 1 0.06 0.06 0.06 1 "
                                       "Specular 0 0.3 0.3 0.3 1 Emissive 1 1 1 1 1 Shininess 0 45")
            ledVisu.addObject('RigidMapping', name="mapping", index=i)

        spotL = self.node.getRoot().addObject('SpotLight', name='LedL', attenuation=0.01, cutoff=35,
                                              direction=[1, 0, 0])
        spotR = self.node.getRoot().addObject('SpotLight', name='LedR', attenuation=0.01, cutoff=35,
                                              direction=[1, 0, 0])
        spotDim = self.node.getRoot().addObject('SpotLight', name='LedDim', attenuation=0.008, cutoff=105,
                                                direction=[1, 0, 0])
        leds.addObject(LedController(leds, [spotL, spotR], spotDim))

        return leds

    def addController(self):
        controller = self.node.addObject(EndoscopeController(self.node))
        return controller

    def getFlexionFromCableDisp(self, disp):
        # flexion is an angle in radian
        flexion = disp / self.params.hardware.pulleyDiameter * 2
        return flexion

    def getCableDispFromFlexion(self, flexion):
        # flexion is an angle in radian
        disp = flexion * self.params.hardware.pulleyDiameter / 2
        return disp

    def addFramesOfInterest(self, showFrames=False):
        """
        Adds frames of interest:
            - f1, base of endoscope
            - f2, head of endoscope
            - f3, center of camera
            - f4, base of left instrument
            - f5, base of left instrument flexible part
            - f6, end of left instrument flexible part
            - f7, end of left instrument
            - f8, base of right instrument
            - f9, base of right instrument flexible part
            - f10, end of right instrument flexible part
            - f11, end of right instrument

        Params:
            showFrames: if true, display the frames
        Returns:
            mechanicalObject containing the frames
        """

        camera = self.rod.getChild("Camera")
        if camera is None:
            Sofa.msg_error(self.node, "[addFramesOfInterest] There is no camera in the node of the endoscope, yet "
                           "one frame of interest is on the camera. To fix this error, use endoscope.addCamera() "
                           "before calling endoscope.addFramesOfInterest().")
            return

        frames = self.base.addChild("FramesOfInterest")
        self.deformable.addChild(frames)
        camera.addChild(frames)
        self.canals.CanalsRigid.addChild(frames)
        self.instruments[0].deformable.addChild(frames)
        self.instruments[1].deformable.addChild(frames)
        frames.addObject('MechanicalObject', template='Rigid3')

        indexPairs = [[0, 0],
                      [1, len(self.deformable.getMechanicalState().position.value) - 1],
                      [2, 0],
                      [3, 1],
                      [4, self.paramsInstrument.passivePart.nbVertebra],
                      [4, self.paramsInstrument.passivePart.nbVertebra +
                          self.paramsInstrument.orientablePart.nbVertebra - 1],
                      [4, self.paramsInstrument.passivePart.nbVertebra +
                          self.paramsInstrument.orientablePart.nbVertebra - 1],
                      [3, 2],
                      [5, self.paramsInstrument.passivePart.nbVertebra],
                      [5, self.paramsInstrument.passivePart.nbVertebra +
                          self.paramsInstrument.orientablePart.nbVertebra - 1],
                      [5, self.paramsInstrument.passivePart.nbVertebra +
                          self.paramsInstrument.orientablePart.nbVertebra - 1]]
        frames.addObject('SubsetMultiMapping', template='Rigid3,Rigid3',
                         input=[self.base.getMechanicalState().linkpath,
                                self.deformable.getMechanicalState().linkpath,
                                camera.getMechanicalState().linkpath,
                                self.canals.CanalsRigid.getMechanicalState().linkpath,
                                self.instruments[0].deformable.getMechanicalState().linkpath,
                                self.instruments[1].deformable.getMechanicalState().linkpath],
                         output=frames.getMechanicalState().linkpath,
                         indexPairs=indexPairs)

        frames = frames.addChild('OrientedFrames')
        frames.addObject('MechanicalObject', template='Rigid3',
                         position=[[0, 0, 0, 0, 0.707, 0, 0.707],
                                   [0, 0, 0, 0, 0.707, 0, 0.707],
                                   [0, 0, 0, 0, 1, 0, 0],
                                   [16, 0, 0, 0, 0.707, 0, 0.707],
                                   [0, 0, 0, 0, 0.707, 0, 0.707],
                                   [0, 0, 0, 0, 0.707, 0, 0.707],
                                   [11, 0, 0, 0, 0.707, 0, 0.707],
                                   [16, 0, 0, 0, 0.707, 0, 0.707],
                                   [0, 0, 0, 0, 0.707, 0, 0.707],
                                   [0, 0, 0, 0, 0.707, 0, 0.707],
                                   [11, 0, 0, 0, 0.707, 0, 0.707],],
                         showObject=showFrames, showObjectScale=5)
        frames.addObject('RigidMapping', rigidIndexPerPoint=list(range(11)),
                         globalToLocalCoords=False)

        return frames.getMechanicalState()


# Test scene
def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers

    CG = False

    settings, modelling, simulation = addHeader(rootnode, withConstraint=not CG)
    rootnode.VisualStyle.displayFlags = ['']
    addSolvers(simulation, iterative=CG)

    endoscope = Endoscope(modelling, simulation, translation=[-550, 0, 0],
                          instrumentsHead=['gripper', 'hook'])
    endoscope.node.OrientableCanals.angles.value = [0.5, -0.5, 0, 0]
    endoscope.addController()
    # endoscope.addLeds()
    endoscope.addCamera(showView=False)
    endoscope.addFramesOfInterest(showFrames=True)
