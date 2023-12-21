def addArticulationCenter(node, name,
                          parentIndex, childIndex,
                          posOnParent, posOnChild,
                          articulationProcess,
                          isTranslation, isRotation, axis,
                          articulationIndex):
    """
    Adds articulation center, compact form.
    Args:
        node:
        name:
        parentIndex:
        childIndex:
        posOnParent:
        posOnChild:
        articulationProcess:
        isTranslation:
        isRotation:
        axis:
        articulationIndex:

    Returns:

    """

    center = node.addChild(name)
    center.addObject('ArticulationCenter', parentIndex=parentIndex, childIndex=childIndex, posOnParent=posOnParent, posOnChild=posOnChild, articulationProcess=articulationProcess)

    articulation = center.addChild('Articulation')
    articulation.addObject('Articulation', translation=isTranslation, rotation=isRotation, rotationAxis=axis, articulationIndex=articulationIndex)

    return center
