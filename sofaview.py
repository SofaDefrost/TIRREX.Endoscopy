from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtOpenGL import *

import Sofa.SofaGL
import Sofa
import SofaRuntime
import os

sofa_directory = os.environ['SOFA_ROOT']
from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np
from PIL import Image

from scene import createScene

# Install
# $ pip install pyopengl
# $ pip install pyqt5
# SOFA/build/external_directories/SofaPython3/$ cmake --install .
# export PYTHONPATH="/home/eulalie/Softs/SOFA/build/lib/python3/site-packages/":$PYTHONPATH
# export SOFA_ROOT="/home/eulalie/Softs/SOFA/build/"


class MainWindow(QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.sofaSimulation = SofaSimulation()
        self.sofaSimulation.initSimulation()

        self.sofaViewWidget = glSofaWidget(self.sofaSimulation.sceneCamera,
                                           self.sofaSimulation.root)

        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.sofaViewWidget)
        self.setLayout(mainLayout)
        self.simulationTimer = QTimer()
        self.simulationTimer.timeout.connect(self.stepSimulation)
        self.simulationTimer.setInterval(1)
        self.simulationTimer.start()

    def stepSimulation(self):
        self.sofaSimulation.stepSimulation()
        self.sofaViewWidget.update()
        # self.sofaSimulation.sceneCamera.position.value = self.sofaSimulation.camera.cameraRigid.value[0:3]
        # self.sofaSimulation.sceneCamera.orientation.value = self.sofaSimulation.camera.cameraRigid.value[3:7]

    def keyPressEvent(self, QKeyEvent):
        if QKeyEvent.key() == Qt.Key_Space:
            self.sofaViewWidget.getImage()


class glSofaWidget(QGLWidget):
    def __init__(self, camera, root):
        QGLWidget.__init__(self)
        self.camera = camera
        self.root = root

        self.setMinimumSize(1920, 1080)
        self.z_far = camera.zFar.value
        self.z_near = camera.zNear.value

    def initializeGL(self):
        glViewport(0, 0, self.width(), self.height())
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glEnable(GL_LIGHTING)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        Sofa.SofaGL.glewInit()
        Sofa.Simulation.initVisual(self.root)
        Sofa.Simulation.initTextures(self.root)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (self.width() / self.height()), self.z_near, self.z_far)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def paintGL(self):
        # Called in updateGL()
        glViewport(0, 0, self.width(), self.height())
        glClearColor(1, 1, 1, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (self.width() / self.height()), self.z_near, self.z_far)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        cameraMVM = self.camera.getOpenGLModelViewMatrix()
        glMultMatrixd(cameraMVM)

        Sofa.SofaGL.draw(self.root)

    def getImage(self):
        _, _, width, height = glGetIntegerv(GL_VIEWPORT)
        buff = glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT)
        image = np.frombuffer(buff, dtype=np.float32)
        image = image.reshape(height, width)
        image = np.flipud(image)  # image is now a numpy array you can use
        depthImage = - self.z_far * self.z_near / (self.z_far + image * (self.z_near - self.z_far))
        depthImage = (depthImage - depthImage.min()) / (depthImage.max() - depthImage.min()) * 255

        # Image.fromarray(depthImage.astype(np.uint8), 'L').show()
        return depthImage


class SofaSimulation:

    def __init__(self):
        # Register all the common component in the factory.
        SofaRuntime.PluginRepository.addFirstPath(os.path.join(sofa_directory, 'bin'))
        SofaRuntime.importPlugin('SofaOpenglVisual')
        SofaRuntime.importPlugin("Sofa.GL.Component.Rendering2D")  # Needed to use components OglViewport
        SofaRuntime.importPlugin("Sofa.GL.Component.Rendering3D")  # Needed to use components OglModel
        SofaRuntime.importPlugin("Sofa.GL.Component.Shader")  # Needed to use components LightManager, SpotLight
        SofaRuntime.importPlugin("Sofa.GUI.Component")  # Needed to use components AttachBodyButtonSetting
        SofaRuntime.importPlugin("Sofa.Component.IO.Mesh")

        self.root = Sofa.Core.Node("Root")
        createScene(self.root, False)

        self.root.addObject('LightManager')
        self.root.addObject("DirectionalLight", direction=[0, 0, 1])

        self.root.addObject("InteractiveCamera",
                            printLog=True,
                            position=[458, -50, 222],
                            orientation=[0.0412167, -0.358981, 0.00961764, 0.932385],
                            distance=0,
                            computeZClip=False,
                            zFar=40000,
                            zNear=0.1
                            )

        self.camera = None  # self.root.Camera
        self.sceneCamera = self.root.InteractiveCamera

    def initSimulation(self):
        Sofa.Simulation.init(self.root)

    def stepSimulation(self):
        Sofa.Simulation.animate(self.root, self.root.getDt())
        Sofa.Simulation.updateVisual(self.root)


if __name__ == '__main__':
    app = QApplication(['Sofa View'])
    window = MainWindow()
    window.show()
    app.exec_()
