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
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 72,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_chamber_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.015, 0.017, 0.020, 1.0))
    satin_black = model.material("satin_black", rgba=(0.050, 0.055, 0.060, 1.0))
    hardcoat = model.material("hardcoat_stanchion", rgba=(0.16, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.25, 0.27, 0.29, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.035, 0.036, 0.038, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _mesh(
            "outer_tube_shell",
            _tube_shell(outer_radius=0.0190, inner_radius=0.0152, z_min=0.000, z_max=0.440),
        ),
        material=anodized_black,
        name="outer_shell",
    )
    outer_tube.visual(
        _mesh(
            "lower_chamber_sleeve",
            _tube_shell(outer_radius=0.0212, inner_radius=0.0186, z_min=0.055, z_max=0.178),
        ),
        material=satin_black,
        name="lower_chamber_sleeve",
    )
    outer_tube.visual(
        _mesh(
            "upper_chamber_sleeve",
            _tube_shell(outer_radius=0.0210, inner_radius=0.0186, z_min=0.274, z_max=0.405),
        ),
        material=satin_black,
        name="upper_chamber_sleeve",
    )
    for z_center, label in ((0.408, "upper"), (0.288, "mid")):
        outer_tube.visual(
            Box((0.0020, 0.0070, 0.032)),
            origin=Origin(xyz=(0.0142, 0.0, z_center)),
            material=gunmetal,
            name=f"{label}_guide_pad_pos_x",
        )
        outer_tube.visual(
            Box((0.0020, 0.0070, 0.032)),
            origin=Origin(xyz=(-0.0142, 0.0, z_center)),
            material=gunmetal,
            name=f"{label}_guide_pad_neg_x",
        )
        outer_tube.visual(
            Box((0.0070, 0.0020, 0.032)),
            origin=Origin(xyz=(0.0, 0.0142, z_center)),
            material=gunmetal,
            name=f"{label}_guide_pad_pos_y",
        )
        outer_tube.visual(
            Box((0.0070, 0.0020, 0.032)),
            origin=Origin(xyz=(0.0, -0.0142, z_center)),
            material=gunmetal,
            name=f"{label}_guide_pad_neg_y",
        )
    outer_tube.visual(
        _mesh(
            "top_wiper_collar",
            _tube_shell(outer_radius=0.0225, inner_radius=0.0144, z_min=0.425, z_max=0.456),
        ),
        material=rubber,
        name="top_wiper_collar",
    )
    outer_tube.visual(
        _mesh(
            "bottom_bushing_ring",
            _tube_shell(outer_radius=0.0212, inner_radius=0.0152, z_min=-0.008, z_max=0.022),
        ),
        material=gunmetal,
        name="bottom_bushing_ring",
    )
    outer_tube.visual(
        Cylinder(radius=0.0062, length=0.041),
        origin=Origin(xyz=(0.0375, 0.000, 0.305), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="cable_port_boss",
    )
    outer_tube.visual(
        Cylinder(radius=0.0044, length=0.021),
        origin=Origin(xyz=(0.065, 0.000, 0.305), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="cable_ferrule",
    )
    outer_tube.visual(
        _mesh(
            "remote_cable_stub",
            tube_from_spline_points(
                [
                    (0.056, 0.000, 0.305),
                    (0.076, -0.012, 0.314),
                    (0.095, -0.040, 0.337),
                    (0.115, -0.070, 0.362),
                ],
                radius=0.0024,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            ),
        ),
        material=rubber,
        name="remote_cable_stub",
    )

    lockout_collar = model.part("lockout_collar")
    lockout_collar.visual(
        _mesh(
            "lockout_collar_ring",
            _tube_shell(outer_radius=0.0260, inner_radius=0.0198, z_min=-0.014, z_max=0.014),
        ),
        material=gunmetal,
        name="collar_ring",
    )
    lockout_collar.visual(
        Cylinder(radius=0.0072, length=0.026),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="lever_hub_pin",
    )
    lockout_collar.visual(
        Box((0.016, 0.074, 0.010)),
        origin=Origin(xyz=(0.0, 0.059, 0.0)),
        material=gunmetal,
        name="lever_paddle",
    )
    lockout_collar.visual(
        Box((0.020, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.100, 0.0)),
        material=rubber,
        name="lever_tip_pad",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0132, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=hardcoat,
        name="stanchion",
    )
    inner_post.visual(
        _mesh(
            "post_top_band",
            _tube_shell(outer_radius=0.0142, inner_radius=0.0125, z_min=0.358, z_max=0.385),
        ),
        material=gunmetal,
        name="top_band",
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Cylinder(radius=0.0165, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=gunmetal,
        name="post_socket",
    )
    clamp_head.visual(
        Box((0.086, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=gunmetal,
        name="lower_clamp_body",
    )
    clamp_head.visual(
        Box((0.020, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=gunmetal,
        name="pivot_boss",
    )
    clamp_head.visual(
        Box((0.027, 0.084, 0.014)),
        origin=Origin(xyz=(-0.034, 0.0, 0.052)),
        material=gunmetal,
        name="bolt_cradle_0",
    )
    clamp_head.visual(
        Box((0.027, 0.084, 0.014)),
        origin=Origin(xyz=(0.034, 0.0, 0.052)),
        material=gunmetal,
        name="bolt_cradle_1",
    )
    clamp_head.visual(
        Cylinder(radius=0.0052, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="tilt_pivot_barrel",
    )
    for index, x_pos in enumerate((-0.034, 0.034)):
        clamp_head.visual(
            Cylinder(radius=0.0036, length=0.055),
            origin=Origin(xyz=(x_pos, 0.0, 0.064)),
            material=bolt_steel,
            name=f"clamp_bolt_{index}",
        )
        clamp_head.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(xyz=(x_pos, 0.0, 0.093)),
            material=steel,
            name=f"bolt_head_{index}",
        )

    saddle = model.part("saddle")
    saddle_shell = superellipse_side_loft(
        [
            (-0.125, 0.040, 0.074, 0.166),
            (-0.078, 0.042, 0.085, 0.190),
            (-0.020, 0.042, 0.086, 0.178),
            (0.050, 0.040, 0.078, 0.120),
            (0.128, 0.038, 0.064, 0.054),
            (0.154, 0.038, 0.055, 0.034),
        ],
        exponents=2.25,
        segments=60,
    ).rotate_z(-math.pi / 2.0)
    saddle.visual(
        _mesh("wide_comfort_saddle_shell", saddle_shell),
        material=saddle_vinyl,
        name="comfort_shell",
    )
    rail_points = [
        (-0.108, 0.0, -0.006),
        (-0.070, 0.0, -0.001),
        (-0.020, 0.0, 0.001),
        (0.045, 0.0, 0.001),
        (0.102, 0.0, -0.004),
    ]
    for index, (rail_name, y_pos) in enumerate((("rail_0", -0.034), ("rail_1", 0.034))):
        saddle.visual(
            _mesh(
                f"saddle_rail_{index}",
                tube_from_spline_points(
                    [(x, y_pos, z) for x, _, z in rail_points],
                    radius=0.0032,
                    samples_per_segment=14,
                    radial_segments=14,
                    cap_ends=True,
                ),
            ),
            material=steel,
            name=rail_name,
        )
        for mount_index, x_pos in enumerate((-0.080, 0.082)):
            saddle.visual(
                Cylinder(radius=0.0036, length=0.058),
                origin=Origin(xyz=(x_pos, y_pos, 0.022)),
                material=steel,
                name=f"rail_strut_{index}_{mount_index}",
            )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.55, lower=0.0, upper=0.125),
    )
    model.articulation(
        "collar_pivot",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=lockout_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "head_mount",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
    )
    model.articulation(
        "saddle_tilt",
        ArticulationType.REVOLUTE,
        parent=clamp_head,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.22, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    lockout_collar = object_model.get_part("lockout_collar")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")
    post_slide = object_model.get_articulation("post_slide")
    collar_pivot = object_model.get_articulation("collar_pivot")
    saddle_tilt = object_model.get_articulation("saddle_tilt")

    ctx.allow_overlap(
        clamp_head,
        saddle,
        elem_a="tilt_pivot_barrel",
        elem_b="rail_0",
        reason="The tilt pin is intentionally captured through the saddle rail cradle.",
    )
    ctx.allow_overlap(
        clamp_head,
        saddle,
        elem_a="tilt_pivot_barrel",
        elem_b="rail_1",
        reason="The tilt pin is intentionally captured through the saddle rail cradle.",
    )

    with ctx.pose({post_slide: 0.0}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_shell",
            margin=0.001,
            name="inner post is centered inside the outer tube bore",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_shell",
            min_overlap=0.14,
            name="collapsed post keeps deep insertion in the outer tube",
        )

    rest_position = ctx.part_world_position(inner_post)
    with ctx.pose({post_slide: 0.125}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_shell",
            margin=0.001,
            name="extended post remains centered in the bore",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_shell",
            min_overlap=0.055,
            name="extended post retains insertion in the outer tube",
        )
        extended_position = ctx.part_world_position(inner_post)
    ctx.check(
        "post slide raises the saddle assembly",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.110,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    ctx.expect_within(
        outer_tube,
        lockout_collar,
        axes="xy",
        inner_elem="outer_shell",
        outer_elem="collar_ring",
        margin=0.001,
        name="lockout collar surrounds the large outer tube",
    )
    ctx.expect_overlap(
        lockout_collar,
        outer_tube,
        axes="z",
        elem_a="collar_ring",
        elem_b="outer_shell",
        min_overlap=0.020,
        name="lockout collar sits at the tube midpoint",
    )

    lever_rest = ctx.part_element_world_aabb(lockout_collar, elem="lever_paddle")
    with ctx.pose({collar_pivot: 0.55}):
        lever_rotated = ctx.part_element_world_aabb(lockout_collar, elem="lever_paddle")
    ctx.check(
        "lockout lever pivots around the tube midpoint",
        lever_rest is not None
        and lever_rotated is not None
        and lever_rotated[0][0] < lever_rest[0][0] - 0.020,
        details=f"rest_aabb={lever_rest}, rotated_aabb={lever_rotated}",
    )

    ctx.expect_overlap(
        clamp_head,
        saddle,
        axes="y",
        elem_a="tilt_pivot_barrel",
        elem_b="rail_0",
        min_overlap=0.006,
        name="tilt pivot captures the first cradle rail",
    )
    ctx.expect_overlap(
        clamp_head,
        saddle,
        axes="y",
        elem_a="tilt_pivot_barrel",
        elem_b="rail_1",
        min_overlap=0.006,
        name="tilt pivot captures the second cradle rail",
    )

    saddle_rest = ctx.part_element_world_aabb(saddle, elem="comfort_shell")
    with ctx.pose({saddle_tilt: 0.20}):
        saddle_tilted = ctx.part_element_world_aabb(saddle, elem="comfort_shell")
    ctx.check(
        "saddle tilt pivot changes the comfort shell attitude",
        saddle_rest is not None
        and saddle_tilted is not None
        and saddle_tilted[0][2] < saddle_rest[0][2] - 0.010,
        details=f"rest_aabb={saddle_rest}, tilted_aabb={saddle_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
