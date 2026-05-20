from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BASE_LENGTH = 0.38
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.055
BASE_CORNER_RADIUS = 0.028
COLUMN_RADIUS = 0.018
COLUMN_HEIGHT = 1.28
HINGE_Z = BASE_THICKNESS + COLUMN_HEIGHT + 0.025
BOOM_LENGTH = 0.64
BOOM_TUBE_RADIUS = 0.013
BOOM_Z = 0.028
SHADE_TOP_Z = -0.080
SHADE_BOTTOM_Z = -0.240


def _rounded_base_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                BASE_LENGTH,
                BASE_WIDTH,
                BASE_CORNER_RADIUS,
                corner_segments=10,
            ),
            BASE_THICKNESS,
            cap=True,
            center=True,
        ),
        "rounded_weighted_base",
    )


def _shade_shell_mesh():
    # A thin-walled lathed frustum: cream outer surface, open bottom and a
    # smaller top ring around the lampholder.
    shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.056, SHADE_TOP_Z),
            (0.084, -0.130),
            (0.155, SHADE_BOTTOM_Z),
        ],
        inner_profile=[
            (0.046, SHADE_TOP_Z - 0.006),
            (0.074, -0.130),
            (0.143, SHADE_BOTTOM_Z + 0.008),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    return mesh_from_geometry(shell, "hollow_cone_shade")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    model.material("weighted_black", rgba=(0.035, 0.037, 0.040, 1.0))
    model.material("satin_column", rgba=(0.48, 0.50, 0.52, 1.0))
    model.material("dark_hardware", rgba=(0.075, 0.075, 0.080, 1.0))
    model.material("warm_brass", rgba=(0.74, 0.57, 0.28, 1.0))
    model.material("shade_cream", rgba=(0.92, 0.88, 0.76, 1.0))
    model.material("warm_bulb", rgba=(1.0, 0.82, 0.42, 0.88))
    model.material("rubber", rgba=(0.010, 0.010, 0.011, 1.0))

    stand = model.part("stand")
    stand.visual(
        _rounded_base_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="weighted_black",
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0)),
        material="satin_column",
        name="fixed_column",
    )
    stand.visual(
        Cylinder(radius=0.031, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + 0.012)),
        material="dark_hardware",
        name="top_bearing",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z - 0.003)),
        material="warm_brass",
        name="bearing_washer",
    )
    for x in (-0.135, 0.135):
        for y in (-0.080, 0.080):
            stand.visual(
                Cylinder(radius=0.018, length=0.010),
                origin=Origin(xyz=(x, y, 0.004)),
                material="rubber",
                name=f"rubber_foot_{x:+.0e}_{y:+.0e}",
            )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.034, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="dark_hardware",
        name="swing_turntable",
    )
    boom.visual(
        Cylinder(radius=BOOM_TUBE_RADIUS, length=0.600),
        origin=Origin(xyz=(0.310, 0.0, BOOM_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_brass",
        name="boom_tube",
    )
    boom.visual(
        Box((0.035, 0.074, 0.024)),
        origin=Origin(xyz=(0.6025, 0.0, BOOM_Z)),
        material="dark_hardware",
        name="end_bridge",
    )
    for y in (-0.034, 0.034):
        boom.visual(
            Box((0.045, 0.012, 0.048)),
            origin=Origin(xyz=(BOOM_LENGTH, y, BOOM_Z)),
            material="dark_hardware",
            name=f"tilt_yoke_{0 if y < 0 else 1}",
        )
    boom.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(BOOM_LENGTH, 0.0, BOOM_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_brass",
        name="tilt_pin",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.014, length=0.044),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_hardware",
        name="tilt_barrel",
    )
    shade.visual(
        Cylinder(radius=0.007, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material="warm_brass",
        name="drop_stem",
    )
    shade.visual(
        Cylinder(radius=0.029, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material="dark_hardware",
        name="lamp_socket",
    )
    shade.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, SHADE_TOP_Z)),
        material="dark_hardware",
        name="shade_collar",
    )
    shade.visual(
        _shade_shell_mesh(),
        material="shade_cream",
        name="cone_shade",
    )
    shade.visual(
        Sphere(radius=0.033),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material="warm_bulb",
        name="glowing_bulb",
    )

    model.articulation(
        "swing_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.85, upper=1.85, effort=9.0, velocity=1.4),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(BOOM_LENGTH, 0.0, BOOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.70, effort=2.0, velocity=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")
    swing = object_model.get_articulation("swing_hinge")
    tilt = object_model.get_articulation("shade_tilt")

    ctx.allow_overlap(
        boom,
        shade,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The brass tilt pin is intentionally captured inside the shade hinge barrel.",
    )

    ctx.expect_gap(
        boom,
        stand,
        axis="z",
        positive_elem="swing_turntable",
        negative_elem="bearing_washer",
        max_gap=0.001,
        max_penetration=0.0,
        name="swing turntable seats on the column bearing washer",
    )
    ctx.expect_within(
        boom,
        shade,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_barrel",
        margin=0.001,
        name="tilt pin is centered inside the shade hinge barrel",
    )
    ctx.expect_overlap(
        boom,
        shade,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.040,
        name="tilt pin spans the captured barrel",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({swing: 1.20}):
        swung_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "boom swings laterally around the column",
        rest_shade_pos is not None
        and swung_shade_pos is not None
        and swung_shade_pos[1] > rest_shade_pos[1] + 0.35,
        details=f"rest={rest_shade_pos}, swung={swung_shade_pos}",
    )

    with ctx.pose({tilt: 0.0}):
        level_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({tilt: 0.60}):
        tilted_aabb = ctx.part_world_aabb(shade)
    ctx.check(
        "cone shade tilts on the boom-end revolute joint",
        level_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[0][0] < level_aabb[0][0] - 0.04,
        details=f"level_aabb={level_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
