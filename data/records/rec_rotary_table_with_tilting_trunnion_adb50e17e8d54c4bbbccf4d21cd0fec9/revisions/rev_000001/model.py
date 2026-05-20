from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_round_member(part, a, b, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder_between(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_rotary_tilt_fixture")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.07, 0.08, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    face_material = model.material("work_face_light", rgba=(0.82, 0.84, 0.82, 1.0))
    rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    safety_red = model.material("safety_red", rgba=(0.72, 0.05, 0.04, 1.0))

    support = model.part("wall_support")
    support.visual(
        Box((0.56, 0.040, 0.62)),
        origin=Origin(xyz=(0.0, -0.030, 0.310)),
        material=painted_steel,
        name="wall_backplate",
    )
    support.visual(
        Box((0.38, 0.380, 0.040)),
        origin=Origin(xyz=(0.0, 0.140, 0.200)),
        material=painted_steel,
        name="cantilever_shelf",
    )
    support.visual(
        Cylinder(radius=0.125, length=0.028),
        origin=Origin(xyz=(0.0, 0.180, 0.234)),
        material=dark_bearing,
        name="stationary_bearing",
    )
    support.visual(
        Cylinder(radius=0.070, length=0.032),
        origin=Origin(xyz=(0.0, 0.180, 0.232)),
        material=brushed_steel,
        name="center_bearing_boss",
    )
    for index, x in enumerate((-0.135, 0.135)):
        _add_round_member(
            support,
            (x, -0.010, 0.085),
            (x, 0.286, 0.180),
            0.012,
            painted_steel,
            f"diagonal_strut_{index}",
        )
    for index, (x, z) in enumerate(
        ((-0.205, 0.145), (0.205, 0.145), (-0.205, 0.505), (0.205, 0.505))
    ):
        support.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(x, -0.006, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"wall_bolt_{index}",
        )
    support.inertial = Inertial.from_geometry(
        Box((0.58, 0.33, 0.64)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.035, 0.320)),
    )

    rotary_plate = model.part("rotary_plate")
    rotary_plate.visual(
        Cylinder(radius=0.165, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_bearing,
        name="lower_plate",
    )
    rotary_plate.visual(
        Cylinder(radius=0.095, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=brushed_steel,
        name="turntable_cap",
    )
    rotary_plate.visual(
        Box((0.035, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.132, 0.050)),
        material=safety_red,
        name="index_mark",
    )
    rotary_plate.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.420, 0.145, 0.300),
                span_width=0.310,
                trunnion_diameter=0.046,
                trunnion_center_z=0.190,
                base_thickness=0.055,
                corner_radius=0.010,
                center=False,
            ),
            "fixture_trunnion_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=painted_steel,
        name="trunnion_yoke",
    )
    for index, x in enumerate((-0.100, 0.100)):
        rotary_plate.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, -0.050, 0.049)),
            material=brushed_steel,
            name=f"yoke_base_bolt_{index}",
        )
    rotary_plate.inertial = Inertial.from_geometry(
        Box((0.44, 0.18, 0.35)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    work_face = model.part("work_face")
    work_face.visual(
        Box((0.250, 0.030, 0.215)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=face_material,
        name="face_panel",
    )
    work_face.visual(
        Box((0.270, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.021, 0.108)),
        material=painted_steel,
        name="top_frame",
    )
    work_face.visual(
        Box((0.270, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.021, -0.108)),
        material=painted_steel,
        name="bottom_frame",
    )
    for index, x in enumerate((-0.132, 0.132)):
        work_face.visual(
            Box((0.018, 0.016, 0.216)),
            origin=Origin(xyz=(x, 0.021, 0.0)),
            material=painted_steel,
            name=f"side_frame_{index}",
        )
    work_face.visual(
        Cylinder(radius=0.017, length=0.092),
        origin=Origin(xyz=(0.167, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion_pin_0",
    )
    work_face.visual(
        Cylinder(radius=0.017, length=0.092),
        origin=Origin(xyz=(-0.167, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion_pin_1",
    )
    work_face.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.216, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="thrust_collar_0",
    )
    work_face.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(-0.216, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="thrust_collar_1",
    )
    work_face.visual(
        Box((0.070, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, 0.065)),
        material=rubber,
        name="horizontal_pad",
    )
    work_face.visual(
        Box((0.012, 0.014, 0.070)),
        origin=Origin(xyz=(0.0, 0.020, 0.065)),
        material=rubber,
        name="vertical_pad",
    )
    work_face.visual(
        Box((0.030, 0.016, 0.014)),
        origin=Origin(xyz=(0.082, 0.022, 0.082)),
        material=safety_red,
        name="tilt_marker",
    )
    work_face.inertial = Inertial.from_geometry(
        Box((0.30, 0.05, 0.25)),
        mass=1.5,
        origin=Origin(),
    )

    model.articulation(
        "support_to_rotary_plate",
        ArticulationType.REVOLUTE,
        parent=support,
        child=rotary_plate,
        origin=Origin(xyz=(0.0, 0.180, 0.248)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "rotary_plate_to_work_face",
        ArticulationType.REVOLUTE,
        parent=rotary_plate,
        child=work_face,
        origin=Origin(xyz=(0.0, 0.0, 0.233)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.80, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("wall_support")
    rotary_plate = object_model.get_part("rotary_plate")
    work_face = object_model.get_part("work_face")
    rotary_joint = object_model.get_articulation("support_to_rotary_plate")
    tilt_joint = object_model.get_articulation("rotary_plate_to_work_face")

    ctx.check(
        "fixture has two revolute user axes",
        rotary_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"types={rotary_joint.articulation_type}, {tilt_joint.articulation_type}",
    )
    ctx.check(
        "rotary axis is vertical",
        tuple(rotary_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={rotary_joint.axis}",
    )
    ctx.check(
        "tilt axis runs through yoke cheeks",
        tuple(tilt_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.expect_gap(
        rotary_plate,
        support,
        axis="z",
        positive_elem="lower_plate",
        negative_elem="stationary_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower plate sits on stationary bearing",
    )
    ctx.expect_overlap(
        rotary_plate,
        support,
        axes="xy",
        elem_a="lower_plate",
        elem_b="stationary_bearing",
        min_overlap=0.18,
        name="rotary plate is centered over bearing",
    )
    ctx.expect_within(
        work_face,
        rotary_plate,
        axes="x",
        inner_elem="face_panel",
        outer_elem="trunnion_yoke",
        margin=0.002,
        name="work face is centered between yoke cheeks",
    )
    ctx.expect_overlap(
        work_face,
        rotary_plate,
        axes="x",
        elem_a="trunnion_pin_0",
        elem_b="trunnion_yoke",
        min_overlap=0.070,
        name="right trunnion pin passes through cheek bore",
    )
    ctx.expect_overlap(
        work_face,
        rotary_plate,
        axes="x",
        elem_a="trunnion_pin_1",
        elem_b="trunnion_yoke",
        min_overlap=0.070,
        name="left trunnion pin passes through cheek bore",
    )

    rest_index_aabb = ctx.part_element_world_aabb(rotary_plate, elem="index_mark")
    with ctx.pose({rotary_joint: math.pi / 2.0}):
        turned_index_aabb = ctx.part_element_world_aabb(rotary_plate, elem="index_mark")
    rest_index_x = None if rest_index_aabb is None else (rest_index_aabb[0][0] + rest_index_aabb[1][0]) * 0.5
    turned_index_x = None if turned_index_aabb is None else (turned_index_aabb[0][0] + turned_index_aabb[1][0]) * 0.5
    ctx.check(
        "lower plate visibly rotates about vertical axis",
        rest_index_x is not None and turned_index_x is not None and turned_index_x < rest_index_x - 0.08,
        details=f"rest_index_x={rest_index_x}, turned_index_x={turned_index_x}",
    )

    rest_marker_aabb = ctx.part_element_world_aabb(work_face, elem="tilt_marker")
    with ctx.pose({tilt_joint: 0.60}):
        tilted_marker_aabb = ctx.part_element_world_aabb(work_face, elem="tilt_marker")
    rest_marker_y = None if rest_marker_aabb is None else (rest_marker_aabb[0][1] + rest_marker_aabb[1][1]) * 0.5
    tilted_marker_y = None if tilted_marker_aabb is None else (tilted_marker_aabb[0][1] + tilted_marker_aabb[1][1]) * 0.5
    ctx.check(
        "work face tilts through the trunnion axis",
        rest_marker_y is not None and tilted_marker_y is not None and tilted_marker_y < rest_marker_y - 0.025,
        details=f"rest_marker_y={rest_marker_y}, tilted_marker_y={tilted_marker_y}",
    )

    return ctx.report()


object_model = build_object_model()
