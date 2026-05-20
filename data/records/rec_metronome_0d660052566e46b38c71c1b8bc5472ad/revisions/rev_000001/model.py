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


def _build_tapered_housing_shell(
    *,
    bottom_width: float,
    bottom_depth: float,
    top_width: float,
    top_depth: float,
    height: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    vertices = [
        (-bottom_width / 2.0, -bottom_depth / 2.0, 0.0),
        (bottom_width / 2.0, -bottom_depth / 2.0, 0.0),
        (bottom_width / 2.0, bottom_depth / 2.0, 0.0),
        (-bottom_width / 2.0, bottom_depth / 2.0, 0.0),
        (-top_width / 2.0, -top_depth / 2.0, height),
        (top_width / 2.0, -top_depth / 2.0, height),
        (top_width / 2.0, top_depth / 2.0, height),
        (-top_width / 2.0, top_depth / 2.0, height),
    ]
    ids = [geom.add_vertex(*vertex) for vertex in vertices]

    _add_quad(geom, ids[0], ids[1], ids[5], ids[4])  # rear wall
    _add_quad(geom, ids[1], ids[2], ids[6], ids[5])  # right wall
    _add_quad(geom, ids[2], ids[3], ids[7], ids[6])  # front wall
    _add_quad(geom, ids[3], ids[0], ids[4], ids[7])  # left wall
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_mechanical_metronome")

    wood_base = model.material("wood_base", rgba=(0.22, 0.14, 0.08, 1.0))
    wood_housing = model.material("wood_housing", rgba=(0.42, 0.25, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.66, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.08, 1.0))

    base_width = 0.124
    base_depth = 0.088
    base_height = 0.018

    body_bottom_width = 0.088
    body_bottom_depth = 0.070
    body_top_width = 0.034
    body_top_depth = 0.030
    body_height = 0.208

    front_face_tilt = math.atan((body_bottom_depth - body_top_depth) / (2.0 * body_height))

    base = model.part("base")
    base.visual(
        Box((base_width, base_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=wood_base,
        name="base_plinth",
    )
    base.visual(
        Box((0.112, 0.076, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, base_height - 0.001)),
        material=wood_housing,
        name="base_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_width, base_depth, base_height + 0.004)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(
            _build_tapered_housing_shell(
                bottom_width=body_bottom_width,
                bottom_depth=body_bottom_depth,
                top_width=body_top_width,
                top_depth=body_top_depth,
                height=body_height,
            ),
            "metronome_housing_shell",
        ),
        material=wood_housing,
        name="housing_shell",
    )
    housing.visual(
        Box((body_bottom_width, body_bottom_depth, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=wood_housing,
        name="inner_floor",
    )
    housing.visual(
        Box((0.010, 0.032, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, body_height - 0.005)),
        material=wood_housing,
        name="top_left_rail",
    )
    housing.visual(
        Box((0.010, 0.032, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, body_height - 0.005)),
        material=wood_housing,
        name="top_right_rail",
    )
    housing.visual(
        Box((0.024, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, body_height - 0.005)),
        material=wood_housing,
        name="top_front_bridge",
    )
    housing.visual(
        Box((0.024, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, body_height - 0.005)),
        material=wood_housing,
        name="top_rear_bridge",
    )
    housing.visual(
        Box((0.005, 0.012, 0.012)),
        origin=Origin(xyz=(-0.0065, 0.002, body_height - 0.012)),
        material=dark_steel,
        name="left_pivot_cheek",
    )
    housing.visual(
        Box((0.005, 0.012, 0.012)),
        origin=Origin(xyz=(0.0065, 0.002, body_height - 0.012)),
        material=dark_steel,
        name="right_pivot_cheek",
    )
    housing.visual(
        Box((0.031, 0.003, 0.192)),
        origin=Origin(xyz=(0.0, 0.024, 0.100), rpy=(front_face_tilt, 0.0, 0.0)),
        material=brass,
        name="scale_panel",
    )
    housing.visual(
        Box((0.004, 0.0015, 0.174)),
        origin=Origin(xyz=(0.0, 0.0255, 0.095), rpy=(front_face_tilt, 0.0, 0.0)),
        material=dark_steel,
        name="tempo_scale_strip",
    )
    housing.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, -0.031, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_bottom_width, body_bottom_depth, body_height)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.FIXED,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, base_height)),
    )

    pendulum = model.part("pendulum")
    rod_y = 0.040
    rod_length = 0.262
    rod_center_z = -0.041
    pendulum.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_arbor",
    )
    pendulum.visual(
        Box((0.004, 0.042, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, -0.004)),
        material=dark_steel,
        name="pendulum_head",
    )
    pendulum.visual(
        Cylinder(radius=0.0014, length=rod_length),
        origin=Origin(xyz=(0.0, rod_y, rod_center_z)),
        material=dark_steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.0, rod_y, -0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pendulum_bob",
    )
    pendulum.visual(
        Box((0.018, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, rod_y, -0.145)),
        material=dark_steel,
        name="bob_carrier",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.024, 0.050, 0.262)),
        mass=0.08,
        origin=Origin(xyz=(0.0, rod_y * 0.8, -0.050)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.002, body_height - 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    tempo_weight = model.part("tempo_weight")
    tempo_weight.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="weight_barrel_front",
    )
    tempo_weight.visual(
        Cylinder(radius=0.0038, length=0.008),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="weight_barrel_rear",
    )
    tempo_weight.visual(
        Box((0.004, 0.018, 0.010)),
        origin=Origin(xyz=(0.003, 0.001, 0.0)),
        material=dark_steel,
        name="weight_clamp",
    )
    tempo_weight.inertial = Inertial.from_geometry(
        Box((0.012, 0.018, 0.012)),
        mass=0.03,
        origin=Origin(xyz=(0.003, 0.001, 0.0)),
    )

    model.articulation(
        "pendulum_to_tempo_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=tempo_weight,
        origin=Origin(xyz=(0.0, rod_y, 0.064)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.25,
            lower=0.0,
            upper=0.102,
        ),
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0028, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.026, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=brass,
        name="key_wing",
    )
    winding_key.visual(
        Box((0.006, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.015, 0.005)),
        material=brass,
        name="key_grip",
    )
    winding_key.inertial = Inertial.from_geometry(
        Box((0.028, 0.026, 0.018)),
        mass=0.025,
        origin=Origin(xyz=(0.0, -0.013, 0.003)),
    )

    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, -0.034, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    tempo_weight = object_model.get_part("tempo_weight")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_tempo_weight")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def _aabb_dims(aabb):
        if aabb is None:
            return None
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        )

    ctx.expect_contact(
        housing,
        base,
        elem_a="inner_floor",
        elem_b="base_cap",
        contact_tol=1e-6,
        name="housing sits on the base cap",
    )
    ctx.expect_gap(
        pendulum,
        housing,
        axis="y",
        positive_elem="pendulum_rod",
        negative_elem="scale_panel",
        min_gap=0.0025,
        name="pendulum rod runs in front of the tempo scale",
    )
    ctx.expect_gap(
        tempo_weight,
        housing,
        axis="y",
        positive_elem="weight_barrel_front",
        negative_elem="scale_panel",
        min_gap=0.004,
        name="tempo weight clears the housing front",
    )

    ctx.check(
        "pendulum joint is a side-to-side revolute hinge",
        pendulum_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in pendulum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={pendulum_joint.articulation_type}, axis={pendulum_joint.axis}",
    )
    ctx.check(
        "tempo weight joint slides down the rod",
        weight_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in weight_joint.axis) == (0.0, 0.0, -1.0),
        details=f"type={weight_joint.articulation_type}, axis={weight_joint.axis}",
    )
    ctx.check(
        "winding key rotates on a continuous rear shaft",
        key_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in key_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={key_joint.articulation_type}, axis={key_joint.axis}",
    )

    rod_rest_aabb = ctx.part_element_world_aabb(pendulum, elem="pendulum_rod")
    with ctx.pose({pendulum_joint: 0.5}):
        rod_swung_aabb = ctx.part_element_world_aabb(pendulum, elem="pendulum_rod")
    rod_rest_x = _aabb_center_x(rod_rest_aabb)
    rod_swung_x = _aabb_center_x(rod_swung_aabb)
    ctx.check(
        "pendulum swings laterally when the hinge opens",
        rod_rest_x is not None and rod_swung_x is not None and abs(rod_swung_x - rod_rest_x) > 0.015,
        details=f"rest_x={rod_rest_x}, swung_x={rod_swung_x}",
    )

    weight_rest_pos = ctx.part_world_position(tempo_weight)
    with ctx.pose({weight_joint: 0.095}):
        weight_low_pos = ctx.part_world_position(tempo_weight)
    ctx.check(
        "tempo weight slides downward along the pendulum rod",
        weight_rest_pos is not None
        and weight_low_pos is not None
        and weight_low_pos[2] < weight_rest_pos[2] - 0.07,
        details=f"rest={weight_rest_pos}, lowered={weight_low_pos}",
    )

    wing_rest_aabb = ctx.part_element_world_aabb(winding_key, elem="key_wing")
    with ctx.pose({key_joint: math.pi / 2.0}):
        wing_turned_aabb = ctx.part_element_world_aabb(winding_key, elem="key_wing")
    rest_dims = _aabb_dims(wing_rest_aabb)
    turned_dims = _aabb_dims(wing_turned_aabb)
    ctx.check(
        "winding key actually rotates about its shaft",
        rest_dims is not None
        and turned_dims is not None
        and turned_dims[0] < rest_dims[0] * 0.5
        and turned_dims[2] > rest_dims[2] * 2.0,
        details=f"rest_dims={rest_dims}, turned_dims={turned_dims}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
