from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


OUTER_LENGTH = 0.250
OUTER_RADIUS = 0.0155
OUTER_BORE_RADIUS = 0.0137
COLLAR_TOP = 0.264
INNER_RADIUS = 0.0125
DROP_TRAVEL = 0.060
INNER_INSERTION = 0.100
INNER_VISIBLE = 0.130
HINGE_Z = 0.164


def _tube_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 64,
):
    """Thin annular tube mesh with visible end rings and an open bore."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_travel_dropper_seatpost")

    black_anodized = model.material("black_anodized", rgba=(0.02, 0.022, 0.024, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.058, 0.062, 1.0))
    polished_stanchion = model.material("polished_stanchion", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.010, 0.011, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        mesh_from_geometry(
            _tube_shell(
                outer_radius=OUTER_RADIUS,
                inner_radius=OUTER_BORE_RADIUS,
                z_min=0.0,
                z_max=OUTER_LENGTH,
                segments=72,
            ),
            "outer_shell",
        ),
        material=black_anodized,
        name="outer_shell",
    )
    outer_tube.visual(
        mesh_from_geometry(
            _tube_shell(
                outer_radius=0.0190,
                inner_radius=OUTER_BORE_RADIUS + 0.0006,
                z_min=OUTER_LENGTH - 0.016,
                z_max=COLLAR_TOP,
                segments=72,
            ),
            "seal_collar",
        ),
        material=satin_black,
        name="seal_collar",
    )
    pad_depth = OUTER_BORE_RADIUS - INNER_RADIUS + 0.0004
    pad_center = INNER_RADIUS + pad_depth * 0.5
    for index, (x, y, sx, sy) in enumerate(
        (
            (pad_center, 0.0, pad_depth, 0.006),
            (-pad_center, 0.0, pad_depth, 0.006),
            (0.0, pad_center, 0.006, pad_depth),
            (0.0, -pad_center, 0.006, pad_depth),
        )
    ):
        outer_tube.visual(
            Box((sx, sy, 0.012)),
            origin=Origin(xyz=(x, y, OUTER_LENGTH - 0.007)),
            material=dark_hardware,
            name=f"guide_pad_{index}",
        )
    outer_tube.visual(
        Box((0.010, 0.014, 0.026)),
        origin=Origin(xyz=(OUTER_RADIUS + 0.0045, 0.0, 0.055)),
        material=satin_black,
        name="cable_port_boss",
    )
    outer_tube.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(
            xyz=(OUTER_RADIUS + 0.0105, 0.0, 0.055),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_hardware,
        name="cable_entry",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=INNER_RADIUS, length=INNER_VISIBLE + INNER_INSERTION),
        origin=Origin(xyz=(0.0, 0.0, (INNER_VISIBLE - INNER_INSERTION) * 0.5)),
        material=polished_stanchion,
        name="inner_tube",
    )
    inner_post.visual(
        Cylinder(radius=0.0155, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, INNER_VISIBLE + 0.003)),
        material=satin_black,
        name="head_collar",
    )
    inner_post.visual(
        Box((0.048, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        material=satin_black,
        name="lower_cradle",
    )
    for side, y in enumerate((-0.020, 0.020)):
        inner_post.visual(
            Box((0.032, 0.006, 0.030)),
            origin=Origin(xyz=(0.0, y, HINGE_Z)),
            material=satin_black,
            name=f"yoke_ear_{side}",
        )
        inner_post.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.18, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardware,
            name=f"pivot_boss_{side}",
        )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.0060, length=0.034),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="hinge_barrel",
    )
    saddle_clamp.visual(
        Box((0.020, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=satin_black,
        name="clamp_web",
    )
    saddle_clamp.visual(
        Box((0.064, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=satin_black,
        name="lower_rail_bed",
    )
    saddle_clamp.visual(
        Box((0.056, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=satin_black,
        name="upper_cap",
    )
    for side, y in enumerate((-0.012, 0.012)):
        saddle_clamp.visual(
            Cylinder(radius=0.0026, length=0.082),
            origin=Origin(xyz=(0.0, y, 0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"saddle_rail_{side}",
        )
    for index, x in enumerate((-0.023, 0.023)):
        saddle_clamp.visual(
            Cylinder(radius=0.0020, length=0.048),
            origin=Origin(xyz=(x, 0.0, 0.0255), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"clamp_bolt_{index}",
        )
        for side, y in enumerate((-0.026, 0.026)):
            saddle_clamp.visual(
                Cylinder(radius=0.0042, length=0.004),
                origin=Origin(xyz=(x, y, 0.0255), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=dark_hardware,
                name=f"bolt_head_{index}_{side}",
            )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, OUTER_LENGTH)),
        # Positive travel is the dropper motion: the stanchion retracts downward
        # into the outer tube while preserving hidden insertion.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.35, lower=0.0, upper=DROP_TRAVEL),
    )
    model.articulation(
        "inner_to_clamp",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.0,
            lower=-math.radians(15.0),
            upper=math.radians(15.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    saddle_clamp = object_model.get_part("saddle_clamp")
    drop_joint = object_model.get_articulation("outer_to_inner")
    clamp_joint = object_model.get_articulation("inner_to_clamp")

    with ctx.pose({drop_joint: 0.0}):
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_shell",
            min_overlap=0.090,
            name="extended inner post remains inserted",
        )

    rest_pos = ctx.part_world_position(inner_post)
    with ctx.pose({drop_joint: DROP_TRAVEL}):
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_shell",
            min_overlap=0.145,
            name="dropped inner post retains insertion",
        )
        dropped_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "dropper joint moves downward",
        rest_pos is not None and dropped_pos is not None and dropped_pos[2] < rest_pos[2] - 0.055,
        details=f"rest={rest_pos}, dropped={dropped_pos}",
    )

    level_aabb = ctx.part_element_world_aabb(saddle_clamp, elem="saddle_rail_0")
    with ctx.pose({clamp_joint: math.radians(12.0)}):
        tilted_aabb = ctx.part_element_world_aabb(saddle_clamp, elem="saddle_rail_0")
    ctx.check(
        "saddle clamp pitches on hinge",
        level_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[1][2] - tilted_aabb[0][2]) > (level_aabb[1][2] - level_aabb[0][2]) + 0.010,
        details=f"level={level_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
