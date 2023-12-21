import os
import numpy as np
import math

import Sofa.Core

from splib3.numerics import Quat, to_radians, Vec3
import scripts.utils.animation as animation


def addDeformableObject(node, filename, collisionFilename=None,
                        youngModulus=8000, poissonRatio=0.45,
                        translation=[0, 0, 0], rotation=[0, 0, 0], scale=[1, 1, 1],
                        totalMass=0.1, color=[1, 1, 1, 1],
                        name='DeformableObject', withSolver=True, withCollision=True,
                        collisionGroup='', exportAnimation=False):
    if collisionFilename is None:
        collisionFilename = filename

    file, extension = os.path.splitext(filename)
    if extension == '.stl':
        loader = 'MeshSTLLoader'
    else:
        loader = 'MeshOBJLoader'

    deformable = node.addChild(name)
    deformable.addObject(loader, name='loader', filename=filename)
    deformable.addObject('SparseGridRamificationTopology', n=[4, 4, 4], src='@loader', onlyInsideCells=False)
    deformable.addObject('MechanicalObject', name='dofs', translation=translation, rotation=rotation, scale3d=scale)
    deformable.addObject('HexahedronFEMForceField', youngModulus=youngModulus, poissonRatio=poissonRatio)
    deformable.addObject('UniformMass', totalMass=totalMass)
    if withSolver:
        deformable.addObject('EulerImplicitSolver')
        deformable.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
        deformable.addObject('GenericConstraintCorrection')

    visu = deformable.addChild('Visu')
    visu.addObject("MeshTopology", name='topology', src='@../loader')
    visu.addObject('MechanicalObject', name='dofs')
    visu.addObject('BarycentricMapping')

    ogl = visu.addChild('Ogl')
    ogl.addObject('OglModel', src='@../topology', color=color)
    ogl.addObject('IdentityMapping')

    if withCollision:
        file, extension = os.path.splitext(collisionFilename)
        if extension == '.stl':
            loader = 'MeshSTLLoader'
        else:
            loader = 'MeshOBJLoader'

        collision = deformable.addChild('Collision')
        collision.addObject(loader, name='loader', filename=collisionFilename)
        collision.addObject('MeshTopology', src='@loader')
        collision.addObject('MechanicalObject')
        collision.addObject('TriangleCollisionModel', group=collisionGroup)
        collision.addObject('LineCollisionModel', group=collisionGroup)
        collision.addObject('PointCollisionModel', group=collisionGroup)
        collision.addObject('BarycentricMapping')

    # Set object config for animation
    if exportAnimation:
        animation.addObjectConfig(visu, name, objectType='deformable', template='Vec3',
                                  indices=list(range(len(visu.topology.position.value))),
                                  meshFilename=os.path.splitext(filename)[0])

    return deformable


def addRigidObject(node, filename, collisionFilename=None, translation=[0, 0, 0], rotation=[0, 0, 0],
                   scale=[1, 1, 1], color=[1, 1, 1], density=0.002,
                   name='Object', collisionGroup='', firstOrder=False,
                   withSolver=True, withCollision=True, static=False, exportAnimation=False):
    if collisionFilename is None:
        collisionFilename = filename

    file, extension = os.path.splitext(filename)
    if extension == '.stl':
        loader = 'MeshSTLLoader'
    else:
        loader = 'MeshOBJLoader'

    item = node.addChild(name)
    position = translation + list(Quat.createFromEuler(to_radians(rotation)))
    item.addObject('MechanicalObject', template='Rigid3', name='dofs', position=position, showObject=False,
                   showObjectScale=5)
    if withSolver:
        item.addObject('EulerImplicitSolver', firstOrder=firstOrder)
        item.addObject('CGLinearSolver', tolerance=1e-5, iterations=25, threshold=1e-5)
        item.addObject('UncoupledConstraintCorrection')

    visu = item.addChild('Visu')
    visu.addObject(loader, name='loader', filename=filename, scale3d=scale)
    visu.addObject('OglModel', src='@loader', color=color)
    visu.addObject('RigidMapping')

    item.addObject('GenerateRigidMass', name='mass', density=density, src=visu.loader.getLinkPath())
    item.mass.init()
    centerToOrigin = item.mass.centerToOrigin.value
    item.addObject('UniformMass', vertexMass="@mass.rigidMass")

    visu.loader.translation = centerToOrigin

    if withCollision:
        file, extension = os.path.splitext(collisionFilename)
        if extension == '.stl':
            loader = 'MeshSTLLoader'
        else:
            loader = 'MeshOBJLoader'

        collision = item.addChild('Collision')
        collision.addObject(loader, name='loader', filename=collisionFilename, scale3d=scale)
        collision.addObject('MeshTopology', src='@loader')
        collision.addObject('MechanicalObject', translation=centerToOrigin)
        collision.addObject('TriangleCollisionModel', group=collisionGroup)
        collision.addObject('LineCollisionModel', group=collisionGroup)
        collision.addObject('PointCollisionModel', group=collisionGroup)
        collision.addObject('RigidMapping')

    # Set object config for animation
    if exportAnimation:
        totalTranslation = list(np.copy(translation))
        centerToOrigin = Vec3(centerToOrigin).rotateFromEuler(to_radians(rotation))
        if static:
            totalTranslation = [translation[i] + centerToOrigin[i] for i in range(3)]

        animation.addObjectConfig(item, name, scale=scale, translation=totalTranslation, rotation=rotation,
                                  objectType='static' if static else 'rigid', template='Rigid3', indices=0,
                                  meshFilename=os.path.splitext(filename)[0])

    return item


def addSphereCollision(rootnode, position=[0, 0, 0]):
    sphere = rootnode.addChild('Sphere')
    sphere.addObject('EulerImplicitSolver', firstOrder=True)
    sphere.addObject('CGLinearSolver', tolerance=1e-5, iterations=25, threshold=1e-5)
    sphere.addObject('MechanicalObject', position=position)
    sphere.addObject('SphereCollisionModel', radius=5)
    sphere.addObject('UncoupledConstraintCorrection')


class HysteresisController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.name = kwargs['name']
        self.cable1 = kwargs['cable1']
        self.cable2 = kwargs['cable2']
        self.hysteresisInit = kwargs['hysteresis']

        self.hysteresis = self.hysteresisInit
        self.direction = 0

        self.apply = True  # apply hysteresis
        self.shift = False  # changed direction in hysteresis

        self.disp = self.cable1.value

    def onAnimateBeginEvent(self, e):

        disp = self.cable1.value  # Displacement from command

        if self.direction != 1 and disp > self.disp:
            self.direction = 1
            self.apply = True
            if self.hysteresis < self.hysteresisInit:
                self.shift = True
        elif self.direction != -1 and disp < self.disp:
            self.direction = -1
            self.apply = True
            if self.hysteresis < self.hysteresisInit:
                self.shift = True

        if self.apply is True and self.hysteresis > 0:
            if self.shift:
                self.hysteresis -= math.fabs(disp - self.disp) * -1
            else:
                self.hysteresis -= math.fabs(disp - self.disp)
            if self.hysteresis < 0:
                self.hysteresis = self.hysteresisInit
                self.apply = False
            elif self.hysteresis > self.hysteresisInit:
                self.hysteresis = self.hysteresisInit
                self.apply = False
            else:
                self.cable1.value = self.disp
                self.cable2.value = -self.disp

        if self.apply is False:
            self.shift = False
            self.cable1.value = disp
            self.cable2.value = - disp

        self.disp = self.cable1.value
