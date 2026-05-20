from __future__ import annotations

from math import cos, pi, sin

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
    section_loft,
    tube_from_spline_points,
)


def _tube_shell(outer_radius: float, inner_radius: float, length: float, name: str):
    """Thin-walled revolved tube with real bore clearance."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, length)],
            [(inner_radius, 0.0), (inner_radius, length)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _saddle_shell_mesh():
    """Narrow racing saddle shell as a thin, slightly arched multi-section loft."""
    sections = []
    # x, overall width, section thickness, z center.  The profile is narrow at
    # the nose, broad at the rear wings, and gently arched like a race saddle.
    for x, width, thick, zc in (
        (-0.105, 0.085, 0.0060, 0.079),
        (-0.075, 0.128, 0.0062, 0.075),
        (-0.035, 0.116, 0.0058, 0.071),
        (0.030, 0.078, 0.0054, 0.072),
        (0.100, 0.046, 0.0052, 0.075),
        (0.158, 0.020, 0.0048, 0.080),
    ):
        loop = []
        for i in range(28):
            t = 2.0 * pi * i / 28
            # Flattened ellipse, with a barely crowned top and underside.
            loop.append((x, 0.5 * width * cos(t), zc + 0.5 * thick * sin(t)))
        sections.append(loop)
    return mesh_from_geometry(section_loft(sections, repair="mesh"), "saddle_shell")


def _rail_mesh(y: float, name: str):
    """One continuous 7 mm steel saddle rail, sagged through the clamp zone."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.088, y, 0.057),
                (-0.052, y, 0.047),
                (0.012, y, 0.0385),
                (0.070, y, 0.045),
                (0.118, y, 0.056),
            ],
            radius=0.0035,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_travel_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.010, 0.012, 0.014, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.025, 0.022, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    hardcoat = model.material("hardcoat_stanchion", rgba=(0.72, 0.72, 0.68, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    latch_red = model.material("latch_red", rgba=(0.72, 0.04, 0.02, 1.0))

    # Root: compact frame's short outer dropper tube.
    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _tube_shell(0.0185, 0.0148, 0.180, "outer_tube_shell"),
        material=anodized_black,
        name="outer_shell",
    )
    outer_tube.visual(
        Cylinder(radius=0.0172, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_black,
        name="base_plug",
    )
    outer_tube.visual(
        Box((0.018, 0.010, 0.030)),
        origin=Origin(xyz=(-0.0185, 0.0, 0.070)),
        material=satin_black,
        name="cable_boss",
    )
    outer_tube.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(-0.030, 0.0, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="cable_socket",
    )

    # Sliding stanchion/post.  The child frame sits at the outer tube lip; the
    # post extends downward into the short sleeve to retain engagement at full travel.
    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0132, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=hardcoat,
        name="stanchion",
    )
    for idx, z in enumerate((0.052, 0.086, 0.120)):
        inner_post.visual(
            Cylinder(radius=0.01345, length=0.0022),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=satin_black,
            name=f"travel_mark_{idx}",
        )
    inner_post.visual(
        Cylinder(radius=0.0160, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=satin_black,
        name="crown_cap",
    )
    inner_post.visual(
        Box((0.034, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, -0.019, 0.188)),
        material=satin_black,
        name="pivot_ear_0",
    )
    inner_post.visual(
        Box((0.034, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, 0.019, 0.188)),
        material=satin_black,
        name="pivot_ear_1",
    )
    inner_post.visual(
        Cylinder(radius=0.0050, length=0.004),
        origin=Origin(xyz=(0.0, -0.023, 0.188), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_head_0",
    )
    inner_post.visual(
        Cylinder(radius=0.0050, length=0.004),
        origin=Origin(xyz=(0.0, 0.023, 0.188), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_head_1",
    )

    # Lever-latch collar ring: a small rotary travel-lock collar around the top lip.
    collar_latch = model.part("collar_latch")
    collar_latch.visual(
        _tube_shell(0.0240, 0.0148, 0.026, "collar_latch_ring"),
        material=satin_black,
        name="collar_ring",
    )
    collar_latch.visual(
        Box((0.057, 0.010, 0.008)),
        origin=Origin(xyz=(0.050, 0.0, 0.015)),
        material=latch_red,
        name="lever",
    )
    collar_latch.visual(
        Cylinder(radius=0.0065, length=0.009),
        origin=Origin(xyz=(0.080, 0.0, 0.015)),
        material=latch_red,
        name="lever_paddle",
    )
    collar_latch.visual(
        Cylinder(radius=0.0052, length=0.016),
        origin=Origin(xyz=(0.025, 0.0, 0.014), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="latch_pivot",
    )

    # Pivoting saddle rail clamp head, carried on the post crown tilt axis.
    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Box((0.052, 0.028, 0.015)),
        origin=Origin(xyz=(0.010, 0.0, 0.006)),
        material=satin_black,
        name="body",
    )
    clamp_head.visual(
        Cylinder(radius=0.0046, length=0.032),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_barrel",
    )
    for idx, y in enumerate((-0.025, 0.025)):
        clamp_head.visual(
            Box((0.070, 0.010, 0.008)),
            origin=Origin(xyz=(0.012, y, 0.03106)),
            material=satin_black,
            name=f"rail_cradle_{idx}",
        )
        clamp_head.visual(
            Box((0.018, 0.022, 0.020)),
            origin=Origin(xyz=(0.012, y, 0.021)),
            material=satin_black,
            name=f"cradle_riser_{idx}",
        )

    # Saddle fixed to the clamp head, so the whole saddle/rail package follows
    # the clamp head's revolute tilt joint.
    saddle = model.part("saddle")
    saddle.visual(_saddle_shell_mesh(), material=rubber_black, name="shell")
    saddle.visual(
        Box((0.128, 0.016, 0.002)),
        origin=Origin(xyz=(-0.004, 0.0, 0.075)),
        material=satin_black,
        name="center_channel",
    )
    for idx, y in enumerate((-0.025, 0.025)):
        saddle.visual(_rail_mesh(y, f"saddle_rail_{idx}"), material=steel, name=f"rail_{idx}")
        saddle.visual(
            Box((0.012, 0.008, 0.024)),
            origin=Origin(xyz=(-0.086, y, 0.064)),
            material=rubber_black,
            name=f"rear_anchor_{idx}",
        )
        saddle.visual(
            Box((0.012, 0.008, 0.026)),
            origin=Origin(xyz=(0.116, y, 0.067)),
            material=rubber_black,
            name=f"front_anchor_{idx}",
        )

    model.articulation(
        "outer_to_post",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.075),
    )
    model.articulation(
        "outer_to_collar",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=collar_latch,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=0.70),
    )
    model.articulation(
        "post_to_head",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "head_to_saddle",
        ArticulationType.FIXED,
        parent=clamp_head,
        child=saddle,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    collar_latch = object_model.get_part("collar_latch")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")
    slide = object_model.get_articulation("outer_to_post")
    collar_joint = object_model.get_articulation("outer_to_collar")
    tilt = object_model.get_articulation("post_to_head")

    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_shell",
        margin=0.002,
        name="inner post is centered in the short outer sleeve",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_shell",
        min_overlap=0.12,
        name="collapsed post retains deep insertion",
    )
    rest_pos = ctx.part_world_position(inner_post)
    with ctx.pose({slide: 0.075}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_shell",
            margin=0.002,
            name="extended post stays centered in the outer sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_shell",
            min_overlap=0.045,
            name="extended post still retains insertion",
        )
        extended_pos = ctx.part_world_position(inner_post)
    ctx.check(
        "short-travel post extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.070,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        collar_latch,
        outer_tube,
        axis="z",
        positive_elem="collar_ring",
        negative_elem="outer_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="collar ring sits on the outer tube lip",
    )
    lever_rest = ctx.part_element_world_aabb(collar_latch, elem="lever")
    with ctx.pose({collar_joint: 0.60}):
        lever_rotated = ctx.part_element_world_aabb(collar_latch, elem="lever")
    if lever_rest is not None and lever_rotated is not None:
        rest_center_y = 0.5 * (lever_rest[0][1] + lever_rest[1][1])
        rotated_center_y = 0.5 * (lever_rotated[0][1] + lever_rotated[1][1])
    else:
        rest_center_y = rotated_center_y = 0.0
    ctx.check(
        "latch lever swings around the collar",
        abs(rotated_center_y - rest_center_y) > 0.020,
        details=f"rest_y={rest_center_y}, rotated_y={rotated_center_y}",
    )

    for idx in (0, 1):
        ctx.expect_overlap(
            saddle,
            clamp_head,
            axes="xy",
            elem_a=f"rail_{idx}",
            elem_b=f"rail_cradle_{idx}",
            min_overlap=0.006,
            name=f"saddle rail {idx} lies over its clamp cradle",
        )
        ctx.expect_gap(
            saddle,
            clamp_head,
            axis="z",
            positive_elem=f"rail_{idx}",
            negative_elem=f"rail_cradle_{idx}",
            max_gap=0.003,
            max_penetration=0.0002,
            name=f"saddle rail {idx} is seated just above the clamp cradle",
        )

    with ctx.pose({tilt: 0.15}):
        ctx.expect_gap(
            saddle,
            clamp_head,
            axis="z",
            positive_elem="shell",
            negative_elem="body",
            min_gap=0.030,
            name="tilted saddle shell remains above clamp head",
        )

    return ctx.report()


object_model = build_object_model()
