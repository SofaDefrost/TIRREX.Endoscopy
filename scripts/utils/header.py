
def addHeader(rootnode, multithreading=True, withConstraint=True):
    """
    Adds to rootnode the default headers for a simulation with contact. Also adds and returns three nodes:
        - Settings
        - Modelling
        - Simulation

    Args:
        rootnode:
        multithreading:
        withConstraint:

    Usage:
        addHeader(rootnode)

    Returns:
        the three SOFA nodes {settings, modelling, simulation}
    """
    settings = rootnode.addChild('Settings')
    settings.addObject('BackgroundSetting', color=[1, 1, 1, 1])
    settings.addObject('RequiredPlugin', name='Plugins', pluginName=[
        'BeamAdapter', 'SoftRobots',
        "ArticulatedSystemPlugin",
        'Cosserat',
        'Sofa.Component.SceneUtility', 'Sofa.Component.Visual',
        # Needed to use components ArticulatedHierarchyContainer,
        # ArticulatedSystemMapping, Articulation, ArticulationCenter
        "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
        "Sofa.Component.Collision.Detection.Algorithm",
        # Needed to use components BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline
        "Sofa.Component.Collision.Detection.Intersection",  # Needed to use components LocalMinDistance
        "Sofa.Component.Collision.Geometry",
        # Needed to use components LineCollisionModel, PointCollisionModel, TriangleCollisionModel
        "Sofa.Component.Collision.Response.Contact",  # Needed to use components RuleBasedContactManager
        "Sofa.Component.Constraint.Lagrangian.Correction",
        # Needed to use components GenericConstraintCorrection, UncoupledConstraintCorrection
        "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
        "Sofa.Component.Engine.Generate",  # Needed to use components GenerateRigidMass
        "Sofa.Component.IO.Mesh",  # Needed to use components MeshOBJLoader, MeshSTLLoader
        "Sofa.Component.LinearSolver.Iterative",  # Needed to use components CGLinearSolver
        "Sofa.Component.Mass",  # Needed to use components UniformMass
        "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
        "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
        "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
        "Sofa.Component.Topology.Container.Dynamic",
        # Needed to use components EdgeSetTopologyContainer, EdgeSetTopologyModifier,
        # QuadSetTopologyContainer, QuadSetTopologyModifier
        "Sofa.Component.Topology.Mapping",  # Needed to use components Edge2QuadTopologicalMapping
        "Sofa.GL.Component.Rendering2D",  # Needed to use components OglViewport
        "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
        "Sofa.GL.Component.Shader",  # Needed to use components LightManager, SpotLight
        "Sofa.GUI.Component",  # Needed to use components AttachBodyButtonSetting
        "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components HexahedronFEMForceField
        "Sofa.Component.Topology.Container.Grid",  # Needed to use components SparseGridRamificationTopology
        'Sofa.Component.Constraint.Projective',  # Needed to use components [FixedConstraint]
        'Sofa.Component.Mapping.Linear',  # Needed to use components [SkinningMapping,SubsetMultiMapping]
        'Sofa.Component.Mapping.NonLinear',  # Needed to use components [RigidMapping]
        'Sofa.Component.StateContainer'  # Needed to use components [MechanicalObject]
    ])

    settings.addObject('AttachBodyButtonSetting', stiffness=1)
    rootnode.addObject('VisualStyle')
    rootnode.addObject("DefaultVisualManagerLoop")
    rootnode.addObject('LightManager')
    if withConstraint:
        rootnode.addObject('CollisionPipeline')
        rootnode.addObject('RuleBasedContactManager', responseParams='mu=0.0', response='FrictionContactConstraint')
        rootnode.addObject('BruteForceBroadPhase')
        rootnode.addObject('BVHNarrowPhase')
        rootnode.addObject('LocalMinDistance', alarmDistance=2, contactDistance=0.1)
        rootnode.addObject('FreeMotionAnimationLoop', parallelCollisionDetectionAndFreeMotion=multithreading,
                           parallelODESolving=multithreading)
        rootnode.addObject('GenericConstraintSolver', name='ConstraintSolver', tolerance=1e-4, maxIterations=200,
                           multithreading=multithreading, printLog=False)
    rootnode.gravity = [0, -9810, 0]
    rootnode.dt = 0.01

    modelling = rootnode.addChild('Modelling')
    modelling.addChild('Topology')
    simulation = rootnode.addChild('Simulation')
    return settings, modelling, simulation


def addSolvers(node, template='CompressedRowSparseMatrixd', rayleighMass=0.01, rayleighStiffness=0.01, firstOrder=False,
               cuda=False, iterative=False):
    """
    Adds solvers (EulerImplicitSolver, LDLSolver, GenericConstraintCorrection) to the given node.

    Args:
        node:
        template: for the LDLSolver
        rayleighMass:
        rayleighStiffness:
        firstOrder: for the implicit scheme
        cuda: requires the plugin SofaCUDASolvers
        iterative:

    Usage:
        ddSolvers(node)
    """
    node.addObject('RequiredPlugin', name='SofaPlugins', pluginName=['Sofa.Component.LinearSolver.Direct',
                                                                     'Sofa.Component.ODESolver.Backward'])
    node.addObject('EulerImplicitSolver', firstOrder=firstOrder, rayleighStiffness=rayleighStiffness,
                   rayleighMass=rayleighMass)
    if iterative:
        node.addObject('CGLinearSolver', name='Solver', iterations=100, threshold=1e-5, tolerance=1e-8)
    else:
        if cuda:
            node.addObject('RequiredPlugin', name='SofaCUDASolvers')
            node.addObject('CudaSparseLDLSolver', name='Solver', template='AsyncCompressedRowSparseMatrixMat3x3f',
                           useMultiThread=True)
        else:
            node.addObject('SparseLDLSolver', name='Solver', template=template)
        node.addObject('GenericConstraintCorrection', linearSolver=node.Solver.getLinkPath())


# Test
def createScene(rootnode):

    addHeader(rootnode)

    node = rootnode.addChild('Node')
    addSolvers(node)
