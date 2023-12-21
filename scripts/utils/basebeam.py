import Sofa.Core
from splib3.numerics import Quat, vsub, Vec3
from math import pi
import numpy as np
from scripts.utils.header import addSolvers


class BeamController(Sofa.Core.Controller):
    """
    Control the beam deployment/retraction
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.name = 'BeamController'
        self.object = args[0]  # object node
        self.index = args[1]  # current index to deploy
        self.length = args[2]  # segment length
        self.indexInit = self.index
        self.displacement = 0

    def onAnimateBeginEvent(self, event):

        node = self.object.node
        dt = node.getRoot().dt.value

        # Used for commands received through ROS
        # The command for the beam's deployment / retraction is stored in node.displacement.value
        # and the current displacement is tracked in self.displacement
        if node.velocity.value > 0 and self.displacement + node.velocity.value * dt > node.displacement.value:
            node.velocity.value = 0
        if node.velocity.value < 0 and self.displacement + node.velocity.value * dt < node.displacement.value:
            node.velocity.value = 0

        if self.index < 0 or self.index > self.indexInit:  # the whole beam has been deployed or retracted
            node.velocity.value = 0  # stop the motion
            self.index = 0 if self.index < 0 else self.indexInit
            return

        velocity = node.velocity.value
        lengthList = self.object.restBeam.BeamInterpolation.lengthList

        # Deploy or retract
        if velocity > 0 or velocity < 0:
            lengths = list(np.copy(lengthList.value))
            lengths[self.index] += node.velocity.value * dt
            self.displacement += node.velocity.value * dt
            if self.displacement < 0:
                self.displacement = 0

            # Limit deployment
            if node.velocity.value > 0 and lengths[self.index] > self.length:
                lengths[self.index] = self.length
                self.index -= 1

            # Limit retraction
            if node.velocity.value < 0 and lengths[self.index] < 0.01:
                lengths[self.index] = 0.01
                self.index += 1

            lengthList.value = lengths
            self.object.rod.BeamInterpolation.lengthList.value = lengths


class BaseBeam:
    """
    Base rod with a passive and an orientable part, based on beam theory, and with a visual and a collision model.
    """

    node: Sofa.Core.Node
    base: Sofa.Core.Node
    deformable: Sofa.Core.Node
    rod: Sofa.Core.Node
    deformabletemplate = 'Rigid3'

    def __init__(self, modelling, simulation, params, name='BaseBeam', translation=[0, 0, 0], rotation=[0, 0, 0],
                 baseRigid=None, baseIndex=0, stacked=False, collisionGroup=0):

        self.modelling = modelling
        self.simulation = simulation
        self.params = params
        self.name = name
        self.translation = translation
        self.rotation = rotation
        self.stacked = stacked
        self.collisionGroup = collisionGroup

        self.base = baseRigid
        self.baseIndex = baseIndex

        self.__addRod()

    def __addRod(self):

        cfp = self.params.passivePart
        cfo = self.params.orientablePart

        self.node = self.modelling.addChild(self.name)
        self.simulation.addChild(self.node)
        self.node.addObject('RequiredPlugin', pluginName=['BeamAdapter'])

        nbTotalVertebra = cfp.nbVertebra + cfo.nbVertebra
        dxp = cfp.length / cfp.nbVertebra
        dxo = cfo.length / cfo.nbVertebra
        self.lengthList = [dxp] * cfp.nbVertebra + [dxo] * cfo.nbVertebra
        if self.stacked:
            position = [[0.01 * i, 0, 0, 0, 0, 0, 1] for i in range(cfp.nbVertebra + 1)]
            position += [[position[cfp.nbVertebra][0] + dxo * i, 0, 0, 0, 0, 0, 1] for i in
                         range(1, cfo.nbVertebra + 1)]
        else:
            position = [[dxp * i, 0, 0, 0, 0, 0, 1] for i in range(cfp.nbVertebra + 1)]
            position += [[cfp.length + dxo * i, 0, 0, 0, 0, 0, 1] for i in range(1, cfo.nbVertebra + 1)]

        indexPairs = [[0, self.baseIndex]]
        for i in range(nbTotalVertebra):
            indexPairs += [[1, i]]

        # Attach the beam on rigid base
        if self.base is not None:
            t = list(np.copy(self.base.MechanicalObject.position.value[self.baseIndex][0:3]))
            self.translation = [self.translation[i] + t[i] for i in range(3)]
            q = Quat(self.base.MechanicalObject.position.value[self.baseIndex][3:7])
            r = q.getEulerAngles()  # angles are in radian
            self.rotation = [self.rotation[i] + r[i] * 180 / pi for i in range(3)]
        else:
            self.base = self.node.addChild('RigidBase' + self.name)
            self.base.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 1],
                                translation=self.translation, rotation=self.rotation,
                                showObject=True, showObjectScale=5)
            self.base.addObject('RestShapeSpringsForceField', template='Rigid3', points=[0],
                                stiffness=1e9,
                                angularStiffness=1e9)

        # The beam rest position: we use this trick to bypass a problem with contacts
        self.restBeam = self.modelling.addChild('RestPosition' + self.name)
        addSolvers(self.restBeam)
        self.restBeam.addObject('VisualStyle', displayFlags=['hideBehavior'])
        self.restBeam.addObject('EdgeSetTopologyContainer', position=[pos[0:3] for pos in position],
                                edges=[[i, i + 1] for i in range(nbTotalVertebra)])
        self.restBeam.addObject('MechanicalObject', template='Rigid3', position=position)
        self.restBeam.addObject('FixedConstraint', indices=0)
        self.restBeam.addObject('BeamInterpolation', defaultYoungModulus=self.params.youngModulus,
                                dofsAndBeamsAligned=True,
                                straight=True,
                                radius=self.params.radius, crossSectionShape='circular',
                                defaultPoissonRatio=self.params.poissonRatio)
        self.restBeam.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True, massDensity=self.params.density)

        # The beam
        self.deformable = self.node.addChild('Deformable' + self.name)
        self.deformable.addObject('MechanicalObject', template='Rigid3', position=position[1:len(position)],
                                  translation=self.translation, rotation=self.rotation)

        self.rod = self.deformable.addChild('Rod' + self.name)
        self.base.addChild(self.rod)

        self.rod.addObject('EdgeSetTopologyContainer', position=[pos[0:3] for pos in position],
                           edges=[[i, i + 1] for i in range(nbTotalVertebra)])
        self.rod.addObject('MechanicalObject', template='Rigid3', position=position,
                           rest_position=self.restBeam.MechanicalObject.position.getLinkPath())

        self.node.addData(name="indexExtremity", type='int', value=len(self.rod.MechanicalObject.position.value) - 1)
        self.node.addData(name="displacement", type='double', value=0)

        self.rod.addObject('BeamInterpolation', defaultYoungModulus=self.params.youngModulus,
                           dofsAndBeamsAligned=True,
                           straight=False,
                           radius=self.params.radius, crossSectionShape='circular',
                           defaultPoissonRatio=self.params.poissonRatio
                           )
        self.rod.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True, massDensity=self.params.density)
        self.rod.addObject('SubsetMultiMapping', template="Rigid3,Rigid3",
                           input=[self.base.MechanicalObject.getLinkPath(),
                                  self.deformable.MechanicalObject.getLinkPath()],
                           output=self.rod.MechanicalObject.getLinkPath(),
                           indexPairs=indexPairs)

        # If stacked define velocity of deployment/retraction and adds controller
        if self.stacked:
            self.node.addData(name='velocity', type='float', help='deployment velocity', value=0)
            self.controller = self.node.addObject(BeamController(self, cfp.nbVertebra - 1, dxp))

        self.__addCylinderTopology()
        self.__addCollisionModel()
        self.__addVisualModel()

    def __addCylinderTopology(self):

        positions = self.rod.MechanicalObject.position.value

        topology = self.modelling.Topology.addChild(self.name + 'CylinderTopo')
        edgetopo = topology.addChild('Edge')
        edgetopo.addObject('EdgeSetTopologyContainer', edges=[[k, k + 1] for k in range(len(positions) - 1)])
        edgetopo.addObject('EdgeSetTopologyModifier')
        edgetopo.addObject('MechanicalObject', template='Rigid3',
                           position=self.rod.MechanicalObject.position.getLinkPath())

        quadtopo = edgetopo.addChild('Quad')
        quadtopo.addObject('QuadSetTopologyContainer')
        quadtopo.addObject('QuadSetTopologyModifier')
        quadtopo.addObject('MechanicalObject')
        quadtopo.addObject('Edge2QuadTopologicalMapping',
                           input=edgetopo.EdgeSetTopologyContainer.getLinkPath(),
                           output=quadtopo.QuadSetTopologyContainer.getLinkPath(),
                           flipNormals=True, nbPointsOnEachCircle=10, radius=self.params.radius)

    def __addVisualModel(self):

        quadtopo = self.modelling.Topology.getChild(self.name + 'CylinderTopo').Edge.Quad

        visual = self.rod.addChild('VisualModel')
        visual.addObject('MeshTopology', position=quadtopo.MechanicalObject.position.getLinkPath(),
                         quads=quadtopo.QuadSetTopologyContainer.quads.getLinkPath())
        visual.addObject('OglModel', color=[0.1, 0.1, 0.1, 1.0])
        visual.addObject('SkinningMapping')

    def __addCollisionModel(self):

        quadtopo = self.modelling.Topology.getChild(self.name + 'CylinderTopo').Edge.Quad

        collision = self.rod.addChild('CollisionModel')
        collision.addObject('MeshTopology', position=quadtopo.MechanicalObject.position.getLinkPath(),
                            quads=quadtopo.QuadSetTopologyContainer.quads.getLinkPath())
        collision.addObject('MechanicalObject')
        collision.addObject('TriangleCollisionModel', group=self.collisionGroup)
        collision.addObject('PointCollisionModel', group=self.collisionGroup)
        collision.addObject('LineCollisionModel', group=self.collisionGroup)
        collision.addObject('SkinningMapping')


# Test scene
def createScene(rootnode):
    from scripts.utils.header import addHeader, addSolvers
    from scripts.endoscope.params import Parameters

    settings, modelling, simulation = addHeader(rootnode)
    rootnode.VisualStyle.displayFlags = ['showBehavior']
    addSolvers(simulation)

    BaseBeam(modelling, simulation, Parameters, stacked=True)
