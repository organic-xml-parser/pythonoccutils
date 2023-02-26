import typing

import vtkmodules
from PyQt5 import QtWidgets
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.vtkCommonColor import vtkNamedColors
import vtkmodules.vtkRenderingAnnotation
import vtkmodules.vtkInteractionWidgets
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkInteractionWidgets import vtkBoxWidget, vtk3DWidget
from vtkmodules.vtkRenderingCore import vtkActor, vtkCamera

from pythonoccutils.cad.gui.pyqt.widgets.widget_toolbar import WidgetToolbar
from pythonoccutils.cad.gui.render_spec import RenderSpec, RenderingColorSpec, EntityRenderingColorSpec
from pythonoccutils.cad.gui.vtk.interaction import MousePickingInteractorStyle
from pythonoccutils.cad.gui.vtk.vtk_occ_bridging import VtkActorsBuilder, VtkOccActorMap
from pythonoccutils.cad.model.session import Session
from pythonoccutils.part_manager import Part


class DisplayFrame(QtWidgets.QFrame):

    def __init__(self, session: Session, parent=None):
        super().__init__(parent)

        self._session = session
        self._scene_actors: typing.List[vtkActor] = []
        self._workspace = None
        self._actor_map: VtkOccActorMap = VtkOccActorMap()

        self._interactor = QVTKRenderWindowInteractor(self)

        self._renderer = vtkmodules.vtkRenderingCore.vtkRenderer()
        self._renderer.SetUseFXAA(True)

        self._interactor_style = MousePickingInteractorStyle(self._session, self._actor_map)
        self.selection_changed_signal = \
            self._interactor_style.selection_tracker.mousePickingEmitter.selectionChangedSignal
        self._interactor_style.SetDefaultRenderer(self._renderer)
        self._interactor.SetInteractorStyle(self._interactor_style)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._widget_toolbar = WidgetToolbar(self._interactor_style, session, self._actor_map, self)
        layout.addWidget(self._widget_toolbar)
        layout.addWidget(self._interactor)

        #ambient_occlusion = True
        #if ambient_occlusion:
        #    ssao = vtkSSAOPass()
        #    ssao.SetRadius(20)
        #    ssao.BlurOn()
        #    ssao.SetBias(0.01)
        #    ssao.SetKernelSize(100)
        #    ssao.SetDelegatePass(vtkRenderStepsPass())
        #    self._renderer.SetPass(ssao)
        #    self._renderer.SetUseSSAO(True)

        self._render_window = self._interactor.GetRenderWindow()
        self._render_window.AddRenderer(self._renderer)

        self._render_spec = RenderSpec(True, True, True)

        self._color_spec = RenderingColorSpec(
            edges_spec=EntityRenderingColorSpec("white", "black"),
            faces_spec=EntityRenderingColorSpec("lamp_black", "white"),
            edges_labelled_spec=EntityRenderingColorSpec("blue", "lilac"),
            faces_labelled_spec=EntityRenderingColorSpec("blue", "lilac"),
            face_annotations_spec=EntityRenderingColorSpec("red", "pink"),
            edge_annotations_spec=EntityRenderingColorSpec("green", "lime"))

        axes_actor = vtkmodules.vtkRenderingAnnotation.vtkAxesActor()
        self._marker_widget = vtkmodules.vtkInteractionWidgets.vtkOrientationMarkerWidget()
        self._marker_widget.SetOrientationMarker(axes_actor)
        self._marker_widget.SetInteractor(self._interactor)
        self._marker_widget.EnabledOn()
        self._marker_widget.InteractiveOn()

        self._renderer.SetBackground(vtkNamedColors().GetColor3d("ivory_black"))
        self._renderer.ResetCamera()
        camera: vtkCamera = self._renderer.GetActiveCamera()
        camera.SetParallelProjection(True)

        self._render_window.SetInteractor(self._interactor)
        self._render_window.SetMultiSamples(8)

        self._session.listener_manager.add_listener(self.session_changed)

        self.setLayout(layout)

    def start(self):
        self.show()
        self._interactor.Initialize()

        # needed to kickstart rendering when first opened
        self.session_changed(self._session)

    @property
    def interactor(self) -> QVTKRenderWindowInteractor:
        return self._interactor

    @property
    def interactor_style(self) -> MousePickingInteractorStyle:
        return self._interactor_style

    @property
    def actor_map(self) -> VtkOccActorMap:
        return self._actor_map

    def add_widget(self, widget: vtk3DWidget):
        if widget in self._widgets:
            raise ValueError("Widget already added")

        widget.SetInteractor(self._interactor)

        self._widgets.append(widget)

    def session_changed(self, session: Session):
        self._widget_toolbar.detach()

        for sa in self._scene_actors:
            self._renderer.RemoveActor(sa)

        self._actor_map.clear()
        self._interactor_style.clear()

        if self._session.workspace is not None:
            parts = self._session.workspace.parts

            self._scene_actors = VtkActorsBuilder({*parts.values()}, self._color_spec, self._render_spec) \
                .get_vtk_actors(self._actor_map)

            for actor in self._scene_actors:
                self._renderer.AddActor(actor)

        self.selection_changed_signal.emit({})
        self._renderer.ResetCamera()
        self._interactor.update()
