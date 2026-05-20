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


def _hollow_ring_mesh(
    *,
    name: str,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 64,
):
    """Lathe a thin-walled open cylindrical sleeve or collar."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_mast_rotary_plate")

    anodized_black = model.material("anodized_black", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    survey_orange = model.material("survey_orange", rgba=(0.95, 0.45, 0.08, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.38, 0.40, 0.42, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.36, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_graphite,
        name="ground_plate",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=anodized_black,
        name="mast_foot",
    )
    base.visual(
        _hollow_ring_mesh(
            name="outer_sleeve",
            outer_radius=0.034,
            inner_radius=0.027,
            z0=0.040,
            z1=0.860,
        ),
        material=anodized_black,
        name="outer_sleeve",
    )
    base.visual(
        _hollow_ring_mesh(
            name="lower_collar",
            outer_radius=0.046,
            inner_radius=0.032,
            z0=0.040,
            z1=0.112,
        ),
        material=dark_graphite,
        name="lower_collar",
    )
    base.visual(
        _hollow_ring_mesh(
            name="outer_top_collar",
            outer_radius=0.047,
            inner_radius=0.032,
            z0=0.775,
            z1=0.865,
        ),
        material=dark_graphite,
        name="top_collar",
    )

    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Box((0.190, 0.020, 0.035)),
            origin=Origin(xyz=(0.105 * c, 0.105 * s, 0.052), rpy=(0.0, 0.0, angle)),
            material=anodized_black,
            name=f"base_rib_{index}",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(xyz=(0.135 * c, 0.135 * s, 0.047)),
            material=bolt_steel,
            name=f"anchor_bolt_{index}",
        )
        base.visual(
            Box((0.070, 0.040, 0.010)),
            origin=Origin(xyz=(0.185 * c, 0.185 * s, 0.005), rpy=(0.0, 0.0, angle)),
            material=rubber_black,
            name=f"rubber_foot_{index}",
        )

    middle_tube = model.part("middle_tube")
    middle_tube.visual(
        _hollow_ring_mesh(
            name="middle_member",
            outer_radius=0.0235,
            inner_radius=0.0180,
            z0=-0.575,
            z1=0.375,
        ),
        material=brushed_aluminum,
        name="middle_member",
    )
    middle_tube.visual(
        _hollow_ring_mesh(
            name="middle_top_collar",
            outer_radius=0.033,
            inner_radius=0.022,
            z0=0.285,
            z1=0.385,
        ),
        material=survey_orange,
        name="top_collar",
    )
    middle_tube.visual(
        _hollow_ring_mesh(
            name="middle_lower_wear_band",
            outer_radius=0.027,
            inner_radius=0.018,
            z0=-0.560,
            z1=-0.505,
        ),
        material=dark_graphite,
        name="lower_wear_band",
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        _hollow_ring_mesh(
            name="inner_member",
            outer_radius=0.0160,
            inner_radius=0.0120,
            z0=-0.420,
            z1=0.360,
        ),
        material=brushed_aluminum,
        name="inner_member",
    )
    inner_tube.visual(
        _hollow_ring_mesh(
            name="inner_top_adapter",
            outer_radius=0.027,
            inner_radius=0.015,
            z0=0.305,
            z1=0.365,
        ),
        material=dark_graphite,
        name="top_adapter",
    )
    inner_tube.visual(
        _hollow_ring_mesh(
            name="inner_lower_wear_band",
            outer_radius=0.0180,
            inner_radius=0.012,
            z0=-0.405,
            z1=-0.355,
        ),
        material=dark_graphite,
        name="lower_wear_band",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.045, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=anodized_black,
        name="bearing_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=bolt_steel,
        name="center_stem",
    )
    pan_head.visual(
        Box((0.128, 0.128, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=survey_orange,
        name="square_plate",
    )
    pan_head.visual(
        Box((0.104, 0.104, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=rubber_black,
        name="square_face",
    )
    pan_head.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=bolt_steel,
        name="threaded_boss",
    )
    for x in (-0.043, 0.043):
        for y in (-0.043, 0.043):
            pan_head.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(xyz=(x, y, 0.0615)),
                material=bolt_steel,
                name=f"face_screw_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )

    model.articulation(
        "base_to_middle_tube",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.350),
    )
    model.articulation(
        "middle_to_inner_tube",
        ArticulationType.PRISMATIC,
        parent=middle_tube,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.280),
    )
    model.articulation(
        "inner_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=inner_tube,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    middle = object_model.get_part("middle_tube")
    inner = object_model.get_part("inner_tube")
    pan = object_model.get_part("pan_head")
    lift_middle = object_model.get_articulation("base_to_middle_tube")
    lift_inner = object_model.get_articulation("middle_to_inner_tube")
    pan_joint = object_model.get_articulation("inner_to_pan_head")

    ctx.allow_overlap(
        base,
        middle,
        elem_a="outer_sleeve",
        elem_b="lower_wear_band",
        reason=(
            "The middle tube's lower wear band is intentionally represented as a "
            "snug sliding bushing captured inside the hollow outer sleeve."
        ),
    )
    ctx.allow_overlap(
        inner,
        middle,
        elem_a="lower_wear_band",
        elem_b="middle_member",
        reason=(
            "The inner tube's lower wear band is intentionally represented as a "
            "snug sliding bushing captured inside the hollow middle tube."
        ),
    )

    with ctx.pose({lift_middle: 0.0, lift_inner: 0.0, pan_joint: 0.0}):
        ctx.expect_within(
            middle,
            base,
            axes="xy",
            inner_elem="middle_member",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="middle lift tube is centered in the outer sleeve",
        )
        ctx.expect_overlap(
            middle,
            base,
            axes="z",
            elem_a="middle_member",
            elem_b="outer_sleeve",
            min_overlap=0.45,
            name="middle lift tube has retained insertion at rest",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="xy",
            inner_elem="inner_member",
            outer_elem="middle_member",
            margin=0.003,
            name="inner lift tube is centered in the middle tube",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="z",
            elem_a="inner_member",
            elem_b="middle_member",
            min_overlap=0.30,
            name="inner lift tube has retained insertion at rest",
        )
        ctx.expect_gap(
            pan,
            inner,
            axis="z",
            positive_elem="bearing_disk",
            negative_elem="top_adapter",
            max_gap=0.003,
            max_penetration=0.0,
            name="pan bearing sits on the top tube adapter",
        )

    rest_pan_position = ctx.part_world_position(pan)
    with ctx.pose({lift_middle: 0.350, lift_inner: 0.280, pan_joint: 0.0}):
        ctx.expect_within(
            middle,
            base,
            axes="xy",
            inner_elem="middle_member",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="extended middle tube remains centered",
        )
        ctx.expect_overlap(
            middle,
            base,
            axes="z",
            elem_a="middle_member",
            elem_b="outer_sleeve",
            min_overlap=0.20,
            name="extended middle tube remains captured",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="xy",
            inner_elem="inner_member",
            outer_elem="middle_member",
            margin=0.003,
            name="extended inner tube remains centered",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="z",
            elem_a="inner_member",
            elem_b="middle_member",
            min_overlap=0.12,
            name="extended inner tube remains captured",
        )
        extended_pan_position = ctx.part_world_position(pan)

    ctx.check(
        "serial lift raises the pan head",
        rest_pan_position is not None
        and extended_pan_position is not None
        and extended_pan_position[2] > rest_pan_position[2] + 0.60,
        details=f"rest={rest_pan_position}, extended={extended_pan_position}",
    )

    with ctx.pose({pan_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(pan)
    with ctx.pose({pan_joint: math.pi / 4.0}):
        rotated_aabb = ctx.part_world_aabb(pan)

    def _xy_size(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (hi[0] - lo[0], hi[1] - lo[1])

    rest_xy = _xy_size(rest_aabb)
    rotated_xy = _xy_size(rotated_aabb)
    ctx.check(
        "square rotary plate turns about the mast axis",
        rest_xy is not None
        and rotated_xy is not None
        and rotated_xy[0] > rest_xy[0] + 0.025
        and rotated_xy[1] > rest_xy[1] + 0.025,
        details=f"rest_xy={rest_xy}, rotated_xy={rotated_xy}",
    )

    return ctx.report()


object_model = build_object_model()
