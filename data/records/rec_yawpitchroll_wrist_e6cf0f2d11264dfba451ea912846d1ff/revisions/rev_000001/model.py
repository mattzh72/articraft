from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    width: float,
    name: str,
    axis: str = "z",
    segments: int = 72,
):
    """Build a compact annular bearing/race mesh centered on the local origin."""
    outer = CylinderGeometry(radius=outer_radius, height=width, radial_segments=segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=width + 0.004,
        radial_segments=segments,
    )
    ring = boolean_difference(outer, inner)
    if axis == "x":
        ring.rotate_y(math.pi / 2.0)
    elif axis == "y":
        ring.rotate_x(math.pi / 2.0)
    elif axis != "z":
        raise ValueError(f"unsupported ring axis: {axis}")
    return mesh_from_geometry(ring, name)


def _cylinder_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_y(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_wrist")

    dark_housing = model.material("dark_housing", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    anodized_blue = model.material("anodized_blue", rgba=(0.18, 0.28, 0.42, 1.0))
    pale_aluminum = model.material("pale_aluminum", rgba=(0.72, 0.74, 0.75, 1.0))
    black_bearing = model.material("black_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.02, 0.02, 0.022, 1.0))

    base = model.part("base")
    base.visual(
        _ring_mesh(
            outer_radius=0.145,
            inner_radius=0.074,
            width=0.026,
            name="base_flange_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_housing,
        name="base_flange",
    )
    base.visual(
        _ring_mesh(
            outer_radius=0.118,
            inner_radius=0.082,
            width=0.028,
            name="base_bearing_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black_bearing,
        name="base_bearing",
    )
    for index in range(6):
        angle = index * math.tau / 6.0
        base.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(
                xyz=(0.129 * math.cos(angle), 0.129 * math.sin(angle), 0.029)
            ),
            material=bolt_black,
            name=f"base_bolt_{index}",
        )

    pitch_axis_z = 0.130

    yaw_carrier = model.part("yaw_carrier")
    yaw_carrier.visual(
        _ring_mesh(
            outer_radius=0.110,
            inner_radius=0.075,
            width=0.026,
            name="yaw_race_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_steel,
        name="yaw_race",
    )
    yaw_carrier.visual(
        Box((0.070, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.095, 0.082)),
        material=anodized_blue,
        name="side_yoke_pos",
    )
    yaw_carrier.visual(
        Box((0.084, 0.075, 0.022)),
        origin=Origin(xyz=(0.0, 0.055, 0.024)),
        material=anodized_blue,
        name="yoke_foot_pos",
    )
    yaw_carrier.visual(
        _ring_mesh(
            outer_radius=0.028,
            inner_radius=0.015,
            width=0.018,
            axis="y",
            name="yaw_bearing_pos_mesh",
            segments=56,
        ),
        origin=Origin(xyz=(0.0, 0.083, pitch_axis_z)),
        material=black_bearing,
        name="yaw_bearing_pos",
    )
    yaw_carrier.visual(
        Cylinder(radius=0.007, length=0.038),
        origin=_origin_x((0.0, 0.095, pitch_axis_z + 0.030)),
        material=bolt_black,
        name="bearing_cap_pos",
    )
    yaw_carrier.visual(
        Box((0.070, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, -0.095, 0.082)),
        material=anodized_blue,
        name="side_yoke_neg",
    )
    yaw_carrier.visual(
        Box((0.084, 0.075, 0.022)),
        origin=Origin(xyz=(0.0, -0.055, 0.024)),
        material=anodized_blue,
        name="yoke_foot_neg",
    )
    yaw_carrier.visual(
        _ring_mesh(
            outer_radius=0.028,
            inner_radius=0.015,
            width=0.018,
            axis="y",
            name="yaw_bearing_neg_mesh",
            segments=56,
        ),
        origin=Origin(xyz=(0.0, -0.083, pitch_axis_z)),
        material=black_bearing,
        name="yaw_bearing_neg",
    )
    yaw_carrier.visual(
        Cylinder(radius=0.007, length=0.038),
        origin=_origin_x((0.0, -0.095, pitch_axis_z + 0.030)),
        material=bolt_black,
        name="bearing_cap_neg",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        _ring_mesh(
            outer_radius=0.062,
            inner_radius=0.034,
            width=0.044,
            axis="x",
            name="roll_housing_mesh",
        ),
        material=pale_aluminum,
        name="roll_housing",
    )
    pitch_yoke.visual(
        _cylinder_x(radius=0.010, length=0.044),
        origin=_origin_x((0.0, 0.046, 0.033)),
        material=pale_aluminum,
        name="upper_web_pos",
    )
    pitch_yoke.visual(
        _cylinder_x(radius=0.010, length=0.044),
        origin=_origin_x((0.0, 0.046, -0.033)),
        material=pale_aluminum,
        name="lower_web_pos",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=_origin_y((0.0, 0.061, 0.0)),
        material=satin_steel,
        name="pitch_trunnion_pos",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_origin_y((0.0, 0.047, 0.0)),
        material=black_bearing,
        name="trunnion_collar_pos",
    )
    pitch_yoke.visual(
        _cylinder_x(radius=0.010, length=0.044),
        origin=_origin_x((0.0, -0.046, 0.033)),
        material=pale_aluminum,
        name="upper_web_neg",
    )
    pitch_yoke.visual(
        _cylinder_x(radius=0.010, length=0.044),
        origin=_origin_x((0.0, -0.046, -0.033)),
        material=pale_aluminum,
        name="lower_web_neg",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=_origin_y((0.0, -0.061, 0.0)),
        material=satin_steel,
        name="pitch_trunnion_neg",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_origin_y((0.0, -0.047, 0.0)),
        material=black_bearing,
        name="trunnion_collar_neg",
    )

    roll_flange = model.part("roll_flange")
    roll_flange.visual(
        Cylinder(radius=0.034, length=0.105),
        origin=_origin_x((0.030, 0.0, 0.0)),
        material=satin_steel,
        name="roll_shaft",
    )
    roll_flange.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=_origin_x((0.0925, 0.0, 0.0)),
        material=pale_aluminum,
        name="tool_face",
    )
    roll_flange.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=_origin_x((0.1095, 0.0, 0.0)),
        material=satin_steel,
        name="pilot_boss",
    )
    for index in range(4):
        angle = index * math.tau / 4.0
        y = 0.037 * math.cos(angle)
        z = 0.037 * math.sin(angle)
        roll_flange.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=_origin_x((0.1055, y, z)),
            material=bolt_black,
            name=f"tool_bolt_{index}",
        )

    model.articulation(
        "yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yaw_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_carrier,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, 0.0, pitch_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "roll",
        ArticulationType.CONTINUOUS,
        parent=pitch_yoke,
        child=roll_flange,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw_carrier = object_model.get_part("yaw_carrier")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_flange = object_model.get_part("roll_flange")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.allow_overlap(
        pitch_yoke,
        roll_flange,
        elem_a="roll_housing",
        elem_b="roll_shaft",
        reason=(
            "The roll shaft is intentionally captured inside the compact bearing "
            "housing; the mesh ring is used as a proxy for a hollow bearing race."
        ),
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        yaw_carrier,
        base,
        axis="z",
        positive_elem="yaw_race",
        negative_elem="base_bearing",
        min_gap=-0.0005,
        max_gap=0.0015,
        name="yaw bearing is seated on base race",
    )
    ctx.expect_contact(
        yaw_carrier,
        pitch_yoke,
        elem_a="yaw_bearing_pos",
        elem_b="pitch_trunnion_pos",
        contact_tol=0.002,
        name="positive pitch trunnion is captured by side yoke",
    )
    ctx.expect_contact(
        yaw_carrier,
        pitch_yoke,
        elem_a="yaw_bearing_neg",
        elem_b="pitch_trunnion_neg",
        contact_tol=0.002,
        name="negative pitch trunnion is captured by side yoke",
    )
    ctx.expect_within(
        roll_flange,
        pitch_yoke,
        axes="yz",
        inner_elem="roll_shaft",
        outer_elem="roll_housing",
        margin=0.0,
        name="roll shaft is centered in the roll bearing housing",
    )
    ctx.expect_overlap(
        roll_flange,
        pitch_yoke,
        axes="x",
        elem_a="roll_shaft",
        elem_b="roll_housing",
        min_overlap=0.038,
        name="roll shaft remains inserted through housing",
    )

    rest_face = aabb_center(ctx.part_element_world_aabb(roll_flange, elem="tool_face"))
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_face = aabb_center(ctx.part_element_world_aabb(roll_flange, elem="tool_face"))
    ctx.check(
        "yaw joint swings the nested wrist about the vertical axis",
        rest_face is not None
        and yawed_face is not None
        and yawed_face[1] > rest_face[1] + 0.060,
        details=f"rest={rest_face}, yawed={yawed_face}",
    )

    with ctx.pose({pitch: 0.65}):
        pitched_face = aabb_center(ctx.part_element_world_aabb(roll_flange, elem="tool_face"))
    ctx.check(
        "positive pitch raises the tool flange",
        rest_face is not None
        and pitched_face is not None
        and pitched_face[2] > rest_face[2] + 0.035,
        details=f"rest={rest_face}, pitched={pitched_face}",
    )

    rest_bolt = aabb_center(ctx.part_element_world_aabb(roll_flange, elem="tool_bolt_0"))
    with ctx.pose({roll: 0.80}):
        rolled_bolt = aabb_center(ctx.part_element_world_aabb(roll_flange, elem="tool_bolt_0"))
    ctx.check(
        "roll joint spins the tool bolt pattern about the tool axis",
        rest_bolt is not None
        and rolled_bolt is not None
        and rolled_bolt[2] > rest_bolt[2] + 0.020,
        details=f"rest={rest_bolt}, rolled={rolled_bolt}",
    )

    return ctx.report()


object_model = build_object_model()
