from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_base_pitch_cradle")

    dark_cast = model.material("dark_cast_aluminum", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.017, 0.02, 1.0))
    bearing_steel = model.material("brushed_bearing_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    fastener = model.material("black_oxide_fasteners", rgba=(0.005, 0.005, 0.006, 1.0))
    glass = model.material("smoked_glass", rgba=(0.02, 0.035, 0.045, 0.82))

    ground_plate = model.part("ground_plate")
    ground_plate.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_cast,
        name="floor_flange",
    )
    ground_plate.visual(
        Cylinder(radius=0.092, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=bearing_steel,
        name="bearing_race",
    )
    for index, (x, y) in enumerate(
        ((0.098, 0.098), (-0.098, 0.098), (-0.098, -0.098), (0.098, -0.098))
    ):
        ground_plate.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, y, 0.037)),
            material=fastener,
            name=f"bolt_head_{index}",
        )

    pan_base = model.part("pan_base")
    pan_base.visual(
        Cylinder(radius=0.108, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_cast,
        name="turntable",
    )
    pan_base.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=satin_black,
        name="center_pedestal",
    )
    pan_base.visual(
        Box((0.026, 0.070, 0.008)),
        origin=Origin(xyz=(0.0, 0.083, 0.034)),
        material=fastener,
        name="front_index",
    )

    yoke_geometry = TrunnionYokeGeometry(
        (0.180, 0.098, 0.180),
        span_width=0.102,
        trunnion_diameter=0.029,
        trunnion_center_z=0.125,
        base_thickness=0.035,
        corner_radius=0.010,
        center=False,
    )
    pan_base.visual(
        mesh_from_geometry(yoke_geometry, "pitch_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_cast,
        name="pitch_yoke",
    )

    head = model.part("head")
    head_shell = superellipse_side_loft(
        [
            (-0.043, -0.030, 0.030, 0.068),
            (-0.018, -0.035, 0.035, 0.078),
            (0.018, -0.035, 0.035, 0.078),
            (0.043, -0.030, 0.030, 0.068),
        ],
        exponents=3.2,
        segments=48,
    )
    head.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        origin=Origin(),
        material=satin_black,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.0145, length=0.170),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.007),
        origin=Origin(xyz=(0.0435, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_cast,
        name="side_hub_0",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.007),
        origin=Origin(xyz=(-0.0435, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_cast,
        name="side_hub_1",
    )
    head.visual(
        Box((0.052, 0.008, 0.036)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=fastener,
        name="front_face",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=ground_plate,
        child=pan_base,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=pan_base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground_plate = object_model.get_part("ground_plate")
    pan_base = object_model.get_part("pan_base")
    head = object_model.get_part("head")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")

    ctx.allow_overlap(
        head,
        pan_base,
        elem_a="trunnion_shaft",
        elem_b="pitch_yoke",
        reason=(
            "The head shaft is intentionally captured through the yoke's simplified "
            "trunnion-bore envelope, matching a real supported pivot pin."
        ),
    )

    ctx.expect_contact(
        ground_plate,
        pan_base,
        elem_a="bearing_race",
        elem_b="turntable",
        contact_tol=0.001,
        name="turntable seats on the grounded bearing race",
    )
    ctx.expect_overlap(
        head,
        pan_base,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="pitch_yoke",
        min_overlap=0.150,
        name="trunnion shaft spans the yoke cheeks",
    )
    ctx.expect_within(
        head,
        pan_base,
        axes="x",
        inner_elem="head_shell",
        outer_elem="pitch_yoke",
        margin=0.0,
        name="compact head shell fits between the side cheeks",
    )

    yaw_limits = yaw.motion_limits
    pitch_limits = pitch.motion_limits
    ctx.check(
        "yaw is a limited vertical revolute pan joint",
        yaw.axis == (0.0, 0.0, 1.0)
        and yaw_limits is not None
        and yaw_limits.lower is not None
        and yaw_limits.upper is not None
        and yaw_limits.lower <= -math.pi
        and yaw_limits.upper >= math.pi,
        details=f"axis={yaw.axis}, limits={yaw_limits}",
    )
    ctx.check(
        "pitch is a trunnion-axis revolute joint",
        pitch.axis == (1.0, 0.0, 0.0)
        and pitch_limits is not None
        and pitch_limits.lower is not None
        and pitch_limits.upper is not None
        and pitch_limits.lower < 0.0
        and pitch_limits.upper > 0.0,
        details=f"axis={pitch.axis}, limits={pitch_limits}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    yaw_rest = _aabb_center(ctx.part_element_world_aabb(pan_base, elem="front_index"))
    with ctx.pose({yaw: math.pi / 2.0}):
        yaw_turned = _aabb_center(ctx.part_element_world_aabb(pan_base, elem="front_index"))
    ctx.check(
        "yaw joint rotates the forked support about the vertical axis",
        yaw_rest is not None
        and yaw_turned is not None
        and yaw_turned[0] < yaw_rest[0] - 0.060
        and yaw_turned[1] < yaw_rest[1] - 0.060,
        details=f"rest={yaw_rest}, turned={yaw_turned}",
    )

    lens_rest = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({pitch: 0.60}):
        lens_pitched = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "positive pitch raises the compact head's front lens",
        lens_rest is not None
        and lens_pitched is not None
        and lens_pitched[2] > lens_rest[2] + 0.020,
        details=f"rest={lens_rest}, pitched={lens_pitched}",
    )

    return ctx.report()


object_model = build_object_model()
