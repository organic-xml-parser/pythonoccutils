import typing

from vtkmodules.vtkCommonColor import vtkNamedColors


class EntityRenderingColorSpec:

    NAMED_COLORS = vtkNamedColors()

    ColorType = typing.Union[typing.Tuple[float, float, float], str]

    def __init__(self,
                 base_color: ColorType,
                 highlight_color: ColorType):
        self.base_color = EntityRenderingColorSpec.color_to_rgb(base_color)
        self.highlight_color = EntityRenderingColorSpec.color_to_rgb(highlight_color)

    @staticmethod
    def color_to_rgb(color_value: ColorType):
        if isinstance(color_value, str):
            named_color = EntityRenderingColorSpec.NAMED_COLORS.GetColor3ub(color_value)
            return named_color.GetRed(), named_color.GetGreen(), named_color.GetBlue()
        else:
            return color_value


class RenderingColorSpec:

    def __init__(self,
                 edges_spec: EntityRenderingColorSpec,
                 faces_spec: EntityRenderingColorSpec,
                 edges_labelled_spec: EntityRenderingColorSpec,
                 faces_labelled_spec: EntityRenderingColorSpec,
                 face_annotations_spec: EntityRenderingColorSpec,
                 edge_annotations_spec: EntityRenderingColorSpec):
        self.edges_spec = edges_spec
        self.faces_spec = faces_spec
        self.edges_labelled_spec = edges_labelled_spec
        self.faces_labelled_spec = faces_labelled_spec
        self.face_annotations_spec = face_annotations_spec
        self.edge_annotations_spec = edge_annotations_spec


class RenderSpec:

    def __init__(self,
                 visualize_face_normals: bool = True,
                 visualize_edge_directions: bool = True,
                 visualize_vertices: bool = True):
        self._visualize_face_normals = visualize_face_normals
        self._visualize_edge_directions = visualize_edge_directions
        self._visualize_vertices = visualize_vertices

    @property
    def visualize_face_normals(self):
        return self._visualize_face_normals

    @property
    def visualize_edge_directions(self):
        return self._visualize_edge_directions

    @property
    def visualize_vertices(self):
        return self._visualize_vertices
