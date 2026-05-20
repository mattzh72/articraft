from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_side_loft,
    tube_from_spline_points,
)


OUTER_TUBE_LENGTH = 0.360
OUTER_TUBE_RADIUS = 0.0180
OUTER_BORE_RADIUS = 0.0154
INNER_POST_RADIUS = 0.0139
POST_TRAVEL = 0.160
SADDLE_PIVOT_Z = 0.345


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _hollow_round_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 64,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _ellipse_section(
    x: float,
    *,
    half_width: float,
    center_z: float,
    half_height: float,
    segments: int = 32,
) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            math.cos(2.0 * math.pi * index / segments) * half_width,
            center_z + math.sin(2.0 * math.pi * index / segments) * half_height,
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.015, 0.017, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.058, 0.060, 1.0))
    polished_stanchion = model.material("polished_stanchion", rgba=(0.70, 0.72, 0.73, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.010, 0.010, 0.010, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    saddle_vinyl = model.material("saddle_vinyl", rgba=(0.020, 0.022, 0.025, 1.0))
    saddle_shell_mat = model.material("saddle_shell", rgba=(0.075, 0.077, 0.080, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _mesh(
            _hollow_round_shell(
                outer_radius=OUTER_TUBE_RADIUS,
                inner_radius=OUTER_BORE_RADIUS,
                z0=0.0,
                z1=OUTER_TUBE_LENGTH,
                segments=80,
            ),
            "wide_outer_tube",
        ),
        material=anodized_black,
        name="outer_shell",
    )
    outer_tube.visual(
        Cylinder(radius=OUTER_TUBE_RADIUS * 0.98, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_black,
        name="bottom_plug",
    )
    outer_tube.visual(
        _mesh(
            _hollow_round_shell(
                outer_radius=0.0205,
                inner_radius=0.0148,
                z0=OUTER_TUBE_LENGTH - 0.010,
                z1=OUTER_TUBE_LENGTH + 0.010,
                segments=72,
            ),
            "wiper_seal_collar",
        ),
        material=dark_rubber,
        name="wiper_seal",
    )
    for pad_name, xyz, size in (
        ("bushing_pad_0", (INNER_POST_RADIUS + 0.0010, 0.0, OUTER_TUBE_LENGTH - 0.004), (0.0020, 0.0080, 0.0240)),
        ("bushing_pad_1", (-(INNER_POST_RADIUS + 0.0010), 0.0, OUTER_TUBE_LENGTH - 0.004), (0.0020, 0.0080, 0.0240)),
        ("bushing_pad_2", (0.0, INNER_POST_RADIUS + 0.0010, OUTER_TUBE_LENGTH - 0.004), (0.0080, 0.0020, 0.0240)),
        ("bushing_pad_3", (0.0, -(INNER_POST_RADIUS + 0.0010), OUTER_TUBE_LENGTH - 0.004), (0.0080, 0.0020, 0.0240)),
    ):
        outer_tube.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=dark_rubber,
            name=pad_name,
        )

    # Side hydraulic port fitting on the lower outer tube.  The boss slightly
    # penetrates the tube wall within the same manufactured body so it reads as
    # a seated threaded hydraulic fitting rather than a floating cylinder.
    outer_tube.visual(
        Cylinder(radius=0.0060, length=0.030),
        origin=Origin(xyz=(OUTER_TUBE_RADIUS + 0.0140, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="port_boss",
    )
    outer_tube.visual(
        _mesh(
            CylinderGeometry(radius=0.0090, height=0.012, radial_segments=6)
            .rotate_y(math.pi / 2.0)
            .translate(OUTER_TUBE_RADIUS + 0.0305, 0.0, 0.105),
            "hex_hydraulic_fitting",
        ),
        material=steel,
        name="port_hex",
    )
    outer_tube.visual(
        Cylinder(radius=0.0042, length=0.025),
        origin=Origin(xyz=(OUTER_TUBE_RADIUS + 0.045, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="hose_stub",
    )

    binder_ring = model.part("binder_ring")
    binder_ring.visual(
        _mesh(
            _hollow_round_shell(
                outer_radius=0.0260,
                inner_radius=OUTER_TUBE_RADIUS + 0.0008,
                z0=OUTER_TUBE_LENGTH - 0.047,
                z1=OUTER_TUBE_LENGTH - 0.012,
                segments=80,
            ),
            "seat_tube_binder_ring",
        ),
        material=satin_black,
        name="binder_collar",
    )
    for index, y in enumerate((-0.012, 0.012)):
        binder_ring.visual(
            Box((0.020, 0.009, 0.030)),
            origin=Origin(xyz=(0.031, y, OUTER_TUBE_LENGTH - 0.030)),
            material=satin_black,
            name=f"pinch_ear_{index}",
        )
    binder_ring.visual(
        Cylinder(radius=0.0041, length=0.054),
        origin=Origin(xyz=(0.035, 0.0, OUTER_TUBE_LENGTH - 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="binder_bolt",
    )
    binder_ring.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.035, -0.029, OUTER_TUBE_LENGTH - 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bolt_head",
    )
    for pad_name, xyz, size in (
        ("clamp_pad_0", (OUTER_TUBE_RADIUS + 0.0008, 0.0, OUTER_TUBE_LENGTH - 0.030), (0.0016, 0.010, 0.026)),
        ("clamp_pad_1", (-(OUTER_TUBE_RADIUS + 0.0008), 0.0, OUTER_TUBE_LENGTH - 0.030), (0.0016, 0.010, 0.026)),
        ("clamp_pad_2", (0.0, OUTER_TUBE_RADIUS + 0.0008, OUTER_TUBE_LENGTH - 0.030), (0.010, 0.0016, 0.026)),
        ("clamp_pad_3", (0.0, -(OUTER_TUBE_RADIUS + 0.0008), OUTER_TUBE_LENGTH - 0.030), (0.010, 0.0016, 0.026)),
    ):
        binder_ring.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=satin_black,
            name=pad_name,
        )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=INNER_POST_RADIUS, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=polished_stanchion,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0160, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=satin_black,
        name="crown_collar",
    )
    inner_post.visual(
        Cylinder(radius=0.0125, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.319)),
        material=satin_black,
        name="head_stem",
    )
    inner_post.visual(
        Box((0.052, 0.086, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.324)),
        material=satin_black,
        name="clamp_neck",
    )
    for rail_name, y in (("rail_cradle_0", -0.024), ("rail_cradle_1", 0.024)):
        inner_post.visual(
            Box((0.070, 0.011, 0.008)),
            origin=Origin(xyz=(0.0, y, SADDLE_PIVOT_Z - 0.0072)),
            material=satin_black,
            name=rail_name,
        )
    for index, y in enumerate((-0.042, 0.042)):
        inner_post.visual(
            Box((0.052, 0.010, 0.032)),
            origin=Origin(xyz=(0.0, y, SADDLE_PIVOT_Z)),
            material=satin_black,
            name=f"tilt_cheek_{index}",
        )

    saddle = model.part("saddle")
    saddle_pad = section_loft(
        [
            _ellipse_section(-0.140, half_width=0.072, center_z=0.060, half_height=0.020),
            _ellipse_section(-0.060, half_width=0.068, center_z=0.071, half_height=0.023),
            _ellipse_section(0.055, half_width=0.044, center_z=0.073, half_height=0.020),
            _ellipse_section(0.140, half_width=0.023, center_z=0.060, half_height=0.015),
        ]
    )
    saddle.visual(_mesh(saddle_pad, "padded_saddle_shell"), material=saddle_vinyl, name="padded_shell")
    saddle_base = section_loft(
        [
            _ellipse_section(-0.132, half_width=0.060, center_z=0.040, half_height=0.012),
            _ellipse_section(-0.058, half_width=0.055, center_z=0.046, half_height=0.013),
            _ellipse_section(0.052, half_width=0.034, center_z=0.047, half_height=0.012),
            _ellipse_section(0.128, half_width=0.019, center_z=0.039, half_height=0.010),
        ]
    )
    saddle.visual(_mesh(saddle_base, "underside_saddle_shell"), material=saddle_shell_mat, name="underside_shell")
    for index, rail_name, y in ((0, "cradle_rail_0", -0.024), (1, "cradle_rail_1", 0.024)):
        rail_points = [
            (-0.123, y, 0.054),
            (-0.104, y, 0.030),
            (-0.064, y, 0.000),
            (0.064, y, 0.000),
            (0.108, y, 0.030),
            (0.132, y, 0.057),
        ]
        saddle.visual(
            _mesh(
                tube_from_spline_points(
                    rail_points,
                    radius=0.0032,
                    samples_per_segment=12,
                    radial_segments=18,
                ),
                f"saddle_cradle_rail_{index}",
            ),
            material=steel,
            name=rail_name,
        )
    saddle.visual(
        Cylinder(radius=0.0065, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_barrel",
    )
    for bridge_name, x in (("rail_bridge_0", -0.088), ("rail_bridge_1", 0.088)):
        saddle.visual(
            Box((0.026, 0.057, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.033)),
            material=saddle_shell_mat,
            name=bridge_name,
        )
    saddle.visual(
        Box((0.076, 0.062, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0068)),
        material=satin_black,
        name="top_clamp_cap",
    )
    saddle.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0123)),
        material=steel,
        name="bolt_washer",
    )
    saddle.visual(
        Cylinder(radius=0.0054, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
        material=steel,
        name="single_bolt",
    )

    model.articulation(
        "binder_mount",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=binder_ring,
        origin=Origin(),
    )
    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TUBE_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.45, lower=0.0, upper=POST_TRAVEL),
    )
    model.articulation(
        "saddle_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, SADDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.25, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    saddle = object_model.get_part("saddle")
    post_slide = object_model.get_articulation("post_slide")
    saddle_tilt = object_model.get_articulation("saddle_tilt")

    ctx.allow_overlap(
        inner_post,
        saddle,
        elem_a="rail_cradle_0",
        elem_b="cradle_rail_0",
        reason="The steel saddle rail is intentionally seated into the shallow clamp cradle channel.",
    )
    ctx.allow_overlap(
        inner_post,
        saddle,
        elem_a="rail_cradle_1",
        elem_b="cradle_rail_1",
        reason="The opposite steel saddle rail is intentionally seated into its shallow clamp cradle channel.",
    )

    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_shell",
        margin=0.001,
        name="inner post is centered in the wide outer tube",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_shell",
        min_overlap=0.12,
        name="retracted post remains deeply inserted",
    )
    rest_pos = ctx.part_world_position(inner_post)
    with ctx.pose({post_slide: POST_TRAVEL}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_shell",
            margin=0.001,
            name="extended post stays concentric in the tube",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_shell",
            min_overlap=0.035,
            name="extended post keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "dropper post extends upward on prismatic axis",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.145,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="cradle_rail_0",
        elem_b="rail_cradle_0",
        contact_tol=0.0025,
        name="one saddle rail sits in its cradle",
    )
    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="cradle_rail_1",
        elem_b="rail_cradle_1",
        contact_tol=0.0025,
        name="opposite saddle rail sits in its cradle",
    )

    rest_pad = ctx.part_element_world_aabb(saddle, elem="padded_shell")
    with ctx.pose({saddle_tilt: 0.25}):
        tilted_forward = ctx.part_element_world_aabb(saddle, elem="padded_shell")
    with ctx.pose({saddle_tilt: -0.25}):
        tilted_rearward = ctx.part_element_world_aabb(saddle, elem="padded_shell")
    ctx.check(
        "saddle tilt pivot changes the padded shell pitch",
        rest_pad is not None
        and tilted_forward is not None
        and tilted_rearward is not None
        and tilted_forward[0][2] < rest_pad[0][2] - 0.008
        and tilted_rearward[1][2] > rest_pad[1][2] + 0.008,
        details=f"rest={rest_pad}, forward={tilted_forward}, rearward={tilted_rearward}",
    )

    return ctx.report()


object_model = build_object_model()
