from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _v_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _v_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _v_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(max(_dot(v, v), 1e-12))
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _panel_from_quad(
    quad: list[tuple[float, float, float]],
    *,
    thickness: float,
    outward_hint: tuple[float, float, float],
) -> MeshGeometry:
    points = list(quad)
    normal = _normalize(_cross(_v_sub(points[1], points[0]), _v_sub(points[2], points[0])))
    if _dot(normal, outward_hint) < 0.0:
        points = list(reversed(points))
        normal = _normalize(_cross(_v_sub(points[1], points[0]), _v_sub(points[2], points[0])))

    inner_points = [_v_add(point, _v_scale(normal, -thickness)) for point in points]

    geom = MeshGeometry()
    outer_ids = [geom.add_vertex(*point) for point in points]
    inner_ids = [geom.add_vertex(*point) for point in inner_points]

    _add_quad(geom, outer_ids[0], outer_ids[1], outer_ids[2], outer_ids[3])
    _add_quad(geom, inner_ids[3], inner_ids[2], inner_ids[1], inner_ids[0])
    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            geom,
            outer_ids[index],
            outer_ids[next_index],
            inner_ids[next_index],
            inner_ids[index],
        )
    return geom


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _solid_from_loops(
    bottom: list[tuple[float, float, float]],
    top: list[tuple[float, float, float]],
) -> MeshGeometry:
    geom = MeshGeometry()
    bottom_ids = [geom.add_vertex(*point) for point in bottom]
    top_ids = [geom.add_vertex(*point) for point in top]

    _add_quad(geom, bottom_ids[3], bottom_ids[2], bottom_ids[1], bottom_ids[0])
    _add_quad(geom, top_ids[0], top_ids[1], top_ids[2], top_ids[3])
    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            geom,
            bottom_ids[index],
            bottom_ids[next_index],
            top_ids[next_index],
            top_ids[index],
        )
    return geom


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pyramid_metronome")

    wood = model.material("wood", rgba=(0.53, 0.34, 0.20, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.36, 0.22, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.63, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    off_white = model.material("off_white", rgba=(0.90, 0.86, 0.74, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))

    base_width = 0.135
    base_depth = 0.105
    base_height = 0.022

    body_height = 0.182
    body_bottom_width = 0.104
    body_bottom_depth = 0.080
    body_top_width = 0.038
    body_top_depth = 0.042
    wall_thickness = 0.0045

    door_bottom_u = 0.006
    door_top_u = 0.085
    stile_width = 0.012
    door_side_gap = 0.0008
    door_top_gap = 0.0010

    def width_at(u: float) -> float:
        return body_bottom_width + (body_top_width - body_bottom_width) * (u / body_height)

    def depth_at(u: float) -> float:
        return body_bottom_depth + (body_top_depth - body_bottom_depth) * (u / body_height)

    def front_y_at(u: float) -> float:
        return depth_at(u) * 0.5

    def back_y_at(u: float) -> float:
        return -depth_at(u) * 0.5

    def z_at(u: float) -> float:
        return base_height + u

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, 0.235)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
    )
    housing.visual(
        Box((base_width, base_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=dark_wood,
        name="base_plinth",
    )
    housing.visual(
        Box((base_width * 0.88, base_depth * 0.82, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, base_height + 0.002)),
        material=wood,
        name="base_top_plate",
    )

    x0 = body_bottom_width * 0.5
    y0 = body_bottom_depth * 0.5
    x1 = body_top_width * 0.5
    y1 = body_top_depth * 0.5
    z0 = base_height
    z1 = base_height + body_height

    left_panel = _panel_from_quad(
        [
            (-x0, -y0, z0),
            (-x0, y0, z0),
            (-x1, y1, z1),
            (-x1, -y1, z1),
        ],
        thickness=wall_thickness,
        outward_hint=(-1.0, 0.0, 0.0),
    )
    right_panel = _panel_from_quad(
        [
            (x0, y0, z0),
            (x0, -y0, z0),
            (x1, -y1, z1),
            (x1, y1, z1),
        ],
        thickness=wall_thickness,
        outward_hint=(1.0, 0.0, 0.0),
    )
    back_panel = _panel_from_quad(
        [
            (x0, -y0, z0),
            (-x0, -y0, z0),
            (-x1, -y1, z1),
            (x1, -y1, z1),
        ],
        thickness=wall_thickness,
        outward_hint=(0.0, -1.0, 0.0),
    )

    front_lower_u = door_bottom_u
    front_upper_u = door_top_u
    full_half_bottom = width_at(front_lower_u) * 0.5
    full_half_top = width_at(front_upper_u) * 0.5
    open_half_bottom = full_half_bottom - stile_width
    open_half_top = full_half_top - stile_width

    left_stile = _panel_from_quad(
        [
            (-full_half_bottom, front_y_at(front_lower_u), z_at(front_lower_u)),
            (-open_half_bottom, front_y_at(front_lower_u), z_at(front_lower_u)),
            (-open_half_top, front_y_at(front_upper_u), z_at(front_upper_u)),
            (-full_half_top, front_y_at(front_upper_u), z_at(front_upper_u)),
        ],
        thickness=wall_thickness,
        outward_hint=(0.0, 1.0, 0.1),
    )
    right_stile = _panel_from_quad(
        [
            (open_half_bottom, front_y_at(front_lower_u), z_at(front_lower_u)),
            (full_half_bottom, front_y_at(front_lower_u), z_at(front_lower_u)),
            (full_half_top, front_y_at(front_upper_u), z_at(front_upper_u)),
            (open_half_top, front_y_at(front_upper_u), z_at(front_upper_u)),
        ],
        thickness=wall_thickness,
        outward_hint=(0.0, 1.0, 0.1),
    )
    upper_front_panel = _panel_from_quad(
        [
            (-full_half_top, front_y_at(front_upper_u), z_at(front_upper_u)),
            (full_half_top, front_y_at(front_upper_u), z_at(front_upper_u)),
            (x1, y1, z1),
            (-x1, y1, z1),
        ],
        thickness=wall_thickness,
        outward_hint=(0.0, 1.0, 0.1),
    )
    scale_strip = _panel_from_quad(
        [
            (-0.009, front_y_at(0.102) + 0.0006, z_at(0.102)),
            (0.009, front_y_at(0.102) + 0.0006, z_at(0.102)),
            (0.006, front_y_at(0.170) + 0.0006, z_at(0.170)),
            (-0.006, front_y_at(0.170) + 0.0006, z_at(0.170)),
        ],
        thickness=0.0012,
        outward_hint=(0.0, 1.0, 0.1),
    )

    housing.visual(_save_mesh("metronome_left_panel", left_panel), material=wood, name="left_panel")
    housing.visual(_save_mesh("metronome_right_panel", right_panel), material=wood, name="right_panel")
    housing.visual(_save_mesh("metronome_back_panel", back_panel), material=wood, name="back_panel")
    housing.visual(_save_mesh("metronome_left_stile", left_stile), material=wood, name="left_stile")
    housing.visual(_save_mesh("metronome_right_stile", right_stile), material=wood, name="right_stile")
    housing.visual(
        _save_mesh("metronome_upper_front", upper_front_panel),
        material=wood,
        name="upper_front_panel",
    )
    housing.visual(
        _save_mesh("metronome_scale_strip", scale_strip),
        material=off_white,
        name="tempo_scale",
    )
    housing.visual(
        Box((body_top_width + 0.006, body_top_depth + 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, z1 + 0.005)),
        material=dark_wood,
        name="top_cap",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(
            xyz=(0.0, back_y_at(0.060) - 0.005, z_at(0.060)),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="rear_key_boss",
    )
    housing.visual(
        Box((0.010, 0.022, 0.024)),
        origin=Origin(xyz=(-0.010, front_y_at(body_height) + 0.008, z1 + 0.012)),
        material=dark_metal,
        name="pivot_cheek_left",
    )
    housing.visual(
        Box((0.010, 0.022, 0.024)),
        origin=Origin(xyz=(0.010, front_y_at(body_height) + 0.008, z1 + 0.012)),
        material=dark_metal,
        name="pivot_cheek_right",
    )
    housing.visual(
        Cylinder(radius=0.0022, length=0.026),
        origin=Origin(
            xyz=(0.0, front_y_at(body_height) + 0.014, z1 + 0.012),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="pivot_shaft",
    )

    mechanism = model.part("mechanism")
    mechanism.inertial = Inertial.from_geometry(
        Box((0.055, 0.024, 0.145)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.004, 0.112)),
    )
    mechanism.visual(
        Box((0.024, 0.008, 0.122)),
        origin=Origin(xyz=(0.0, -0.006, 0.098)),
        material=brass,
        name="frame_plate",
    )
    mechanism.visual(
        Box((0.050, 0.016, 0.030)),
        origin=Origin(xyz=(0.0, -0.002, 0.037)),
        material=dark_metal,
        name="lower_mount_block",
    )
    mechanism.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.002, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="spring_barrel",
    )
    mechanism.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.015, 0.002, 0.084), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="gear_left",
    )
    mechanism.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.013, 0.002, 0.096), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="gear_right",
    )
    mechanism.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.002, 0.126), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="escapement_wheel",
    )
    mechanism.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.001, 0.140)),
        material=dark_metal,
        name="escapement_bridge",
    )
    mechanism.visual(
        Box((0.004, 0.010, 0.026)),
        origin=Origin(xyz=(-0.006, 0.002, 0.149)),
        material=dark_metal,
        name="anchor_left",
    )
    mechanism.visual(
        Box((0.004, 0.010, 0.026)),
        origin=Origin(xyz=(0.006, 0.002, 0.149)),
        material=dark_metal,
        name="anchor_right",
    )
    mechanism.visual(
        Box((0.016, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.002, 0.137)),
        material=dark_metal,
        name="anchor_bridge",
    )
    model.articulation(
        "housing_to_mechanism",
        ArticulationType.FIXED,
        parent=housing,
        child=mechanism,
        origin=Origin(),
    )

    door = model.part("front_door")
    door.inertial = Inertial.from_geometry(
        Box((0.080, 0.010, 0.080)),
        mass=0.055,
        origin=Origin(xyz=(0.0, -0.002, 0.040)),
    )

    door_half_bottom = open_half_bottom - door_side_gap
    door_half_top = open_half_top - door_side_gap
    door_height = (door_top_u - door_top_gap) - door_bottom_u
    front_base_y = front_y_at(door_bottom_u)
    front_top_y = front_y_at(door_top_u - door_top_gap) - front_base_y
    door_panel = _panel_from_quad(
        [
            (-door_half_bottom, 0.0, 0.0),
            (door_half_bottom, 0.0, 0.0),
            (door_half_top, front_top_y, door_height),
            (-door_half_top, front_top_y, door_height),
        ],
        thickness=0.0042,
        outward_hint=(0.0, 1.0, 0.1),
    )
    door.visual(_save_mesh("metronome_front_door", door_panel), material=wood, name="door_panel")
    door.visual(
        Cylinder(radius=0.0036, length=0.007),
        origin=Origin(
            xyz=(0.0, front_top_y * 0.62 + 0.0036, door_height * 0.62),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="door_knob",
    )
    model.articulation(
        "housing_to_front_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, front_y_at(door_bottom_u), z_at(door_bottom_u))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    pendulum = model.part("pendulum")
    pendulum.inertial = Inertial.from_geometry(
        Box((0.020, 0.020, 0.270)),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
    )
    pendulum.visual(
        Cylinder(radius=0.0015, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, -0.1275)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0034, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_metal,
        name="pendulum_pivot_sleeve",
    )
    pendulum.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.226)),
        material=brass,
        name="lower_counterweight",
    )
    pendulum.visual(
        Box((0.004, 0.002, 0.038)),
        origin=Origin(xyz=(0.0, 0.0015, -0.050)),
        material=off_white,
        name="tempo_pointer",
    )
    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, front_y_at(body_height) + 0.014, z1 + 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-0.42,
            upper=0.42,
        ),
    )

    weight = model.part("slider_weight")
    weight.inertial = Inertial.from_geometry(
        Box((0.018, 0.014, 0.022)),
        mass=0.035,
        origin=Origin(),
    )
    weight.visual(
        Box((0.0042, 0.010, 0.022)),
        origin=Origin(xyz=(-0.0055, 0.0, 0.0)),
        material=brass,
        name="weight_left_cheek",
    )
    weight.visual(
        Box((0.0042, 0.010, 0.022)),
        origin=Origin(xyz=(0.0055, 0.0, 0.0)),
        material=brass,
        name="weight_right_cheek",
    )
    weight.visual(
        Box((0.015, 0.0026, 0.005)),
        origin=Origin(xyz=(0.0, 0.0042, 0.006)),
        material=brass,
        name="weight_front_bridge",
    )
    weight.visual(
        Box((0.015, 0.0026, 0.005)),
        origin=Origin(xyz=(0.0, -0.0042, 0.006)),
        material=brass,
        name="weight_back_bridge",
    )
    weight.visual(
        _save_mesh(
            "metronome_weight_wedge",
            _solid_from_loops(
                [
                    (-0.0062, -0.0030, -0.0110),
                    (0.0062, -0.0030, -0.0110),
                    (0.0062, 0.0030, -0.0110),
                    (-0.0062, 0.0030, -0.0110),
                ],
                [
                    (-0.0054, -0.0030, -0.0086),
                    (0.0054, -0.0030, -0.0086),
                    (0.0062, 0.0030, -0.0048),
                    (-0.0062, 0.0030, -0.0048),
                ],
            ),
        ),
        material=brass,
        name="weight_wedge_body",
    )
    model.articulation(
        "pendulum_to_slider_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=0.105,
        ),
    )

    key = model.part("winding_key")
    key.inertial = Inertial.from_geometry(
        Box((0.030, 0.024, 0.016)),
        mass=0.012,
        origin=Origin(xyz=(0.0, -0.012, 0.003)),
    )
    key.visual(
        Cylinder(radius=0.0026, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="key_shaft",
    )
    key.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_hub",
    )
    key.visual(
        Box((0.006, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, 0.003)),
        material=brass,
        name="key_stem",
    )
    key.visual(
        Box((0.028, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, -0.0185, 0.006)),
        material=brass,
        name="key_crossbar",
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(0.0, back_y_at(0.060) - 0.010, z_at(0.060))),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    mechanism = object_model.get_part("mechanism")
    door = object_model.get_part("front_door")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("slider_weight")
    key = object_model.get_part("winding_key")

    door_joint = object_model.get_articulation("housing_to_front_door")
    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_slider_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    ctx.check("housing part exists", housing is not None)
    ctx.check("mechanism part exists", mechanism is not None)
    ctx.check("front door part exists", door is not None)
    ctx.check("pendulum part exists", pendulum is not None)
    ctx.check("slider weight part exists", weight is not None)
    ctx.check("winding key part exists", key is not None)

    ctx.check(
        "door hinge axis is along x",
        tuple(round(v, 3) for v in door_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "pendulum swings about front-back axis",
        tuple(round(v, 3) for v in pendulum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pendulum_joint.axis}",
    )
    ctx.check(
        "slider moves downward along rod",
        tuple(round(v, 3) for v in weight_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={weight_joint.axis}",
    )
    ctx.check(
        "winding key spins on rear-facing axis",
        abs(key_joint.axis[1]) > 0.99 and abs(key_joint.axis[0]) < 1e-6 and abs(key_joint.axis[2]) < 1e-6,
        details=f"axis={key_joint.axis}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.20}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return (
            0.5 * (min_corner[0] + max_corner[0]),
            0.5 * (min_corner[1] + max_corner[1]),
            0.5 * (min_corner[2] + max_corner[2]),
        )

    closed_door_pos = _aabb_center(closed_door_aabb)
    opened_door_pos = _aabb_center(opened_door_aabb)
    ctx.check(
        "front door opens outward and downward",
        closed_door_pos is not None
        and opened_door_pos is not None
        and abs(opened_door_pos[1] - closed_door_pos[1]) > 0.020
        and opened_door_pos[2] < closed_door_pos[2] - 0.010,
        details=f"closed={closed_door_pos}, open={opened_door_pos}",
    )

    with ctx.pose({weight_joint: 0.020}):
        upper_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: 0.100}):
        lower_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "slider weight travels down the pendulum rod",
        upper_weight_pos is not None
        and lower_weight_pos is not None
        and lower_weight_pos[2] < upper_weight_pos[2] - 0.060,
        details=f"upper={upper_weight_pos}, lower={lower_weight_pos}",
    )

    with ctx.pose({weight_joint: 0.060, pendulum_joint: 0.0}):
        centered_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: 0.060, pendulum_joint: 0.28}):
        swung_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "pendulum swing moves the slider laterally",
        centered_weight_pos is not None
        and swung_weight_pos is not None
        and abs(swung_weight_pos[0] - centered_weight_pos[0]) > 0.020,
        details=f"center={centered_weight_pos}, swung={swung_weight_pos}",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_origin_gap(
            mechanism,
            housing,
            axis="z",
            min_gap=-0.20,
            max_gap=0.20,
            name="mechanism stays nested within housing height band",
        )
        ctx.expect_contact(
            key,
            housing,
            elem_a="key_shaft",
            elem_b="rear_key_boss",
            contact_tol=0.0005,
            name="winding key shaft seats against rear boss",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
