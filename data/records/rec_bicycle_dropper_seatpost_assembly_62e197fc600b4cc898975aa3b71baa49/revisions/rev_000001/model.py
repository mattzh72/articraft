from __future__ import annotations

from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


BLACK_ANODIZED = Material("black_anodized", rgba=(0.005, 0.006, 0.007, 1.0))
MATTE_BLACK = Material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
RUBBER = Material("black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
STEEL = Material("brushed_steel", rgba=(0.73, 0.73, 0.70, 1.0))
DARK_STEEL = Material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
SATIN_ALLOY = Material("satin_alloy", rgba=(0.46, 0.47, 0.46, 1.0))


def _shell_section(x: float, width: float, thickness: float, z: float, *, samples: int = 24):
    """Superellipse section in the YZ plane for the saddle body."""

    exponent = 2.8
    pts = []
    for i in range(samples):
        t = tau * i / samples
        c = cos(t)
        s = sin(t)
        y = (width * 0.5) * (1.0 if c >= 0.0 else -1.0) * (abs(c) ** (2.0 / exponent))
        zz = z + (thickness * 0.5) * (1.0 if s >= 0.0 else -1.0) * (abs(s) ** (2.0 / exponent))
        pts.append((x, y, zz))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="external_routing_dropper_seatpost")

    outer_tube = model.part("outer_tube")

    # A real dropper post lower is a hollow round sleeve; use an open bore so
    # the moving stanchion visibly nests in it instead of occupying a solid rod.
    outer_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0175, 0.035), (0.0175, 0.365)],
        inner_profile=[(0.0148, 0.035), (0.0148, 0.365)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    outer_tube.visual(
        mesh_from_geometry(outer_shell, "outer_tube_shell"),
        material=BLACK_ANODIZED,
        name="outer_shell",
    )
    top_wiper = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0185, 0.356), (0.0185, 0.374)],
        inner_profile=[(0.0135, 0.356), (0.0135, 0.374)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    outer_tube.visual(
        mesh_from_geometry(top_wiper, "top_wiper_seal"),
        material=RUBBER,
        name="top_wiper",
    )

    binder_ring = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0260, 0.000), (0.0260, 0.075)],
        inner_profile=[(0.0173, 0.000), (0.0173, 0.075)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    outer_tube.visual(
        mesh_from_geometry(binder_ring, "seat_tube_binder_ring"),
        material=SATIN_ALLOY,
        name="binder_ring",
    )
    outer_tube.visual(
        Box((0.010, 0.018, 0.036)),
        origin=Origin(xyz=(-0.010, 0.030, 0.039)),
        material=SATIN_ALLOY,
        name="binder_ear_0",
    )
    outer_tube.visual(
        Box((0.010, 0.018, 0.036)),
        origin=Origin(xyz=(0.010, 0.030, 0.039)),
        material=SATIN_ALLOY,
        name="binder_ear_1",
    )
    outer_tube.visual(
        Cylinder(radius=0.0032, length=0.038),
        origin=Origin(xyz=(0.0, 0.039, 0.041), rpy=(0.0, pi / 2.0, 0.0)),
        material=STEEL,
        name="binder_bolt",
    )
    outer_tube.visual(
        Cylinder(radius=0.0062, length=0.004),
        origin=Origin(xyz=(-0.021, 0.039, 0.041), rpy=(0.0, pi / 2.0, 0.0)),
        material=STEEL,
        name="binder_bolt_head",
    )

    # External routing details: an oval-like side port and a cable anchor block
    # riding proud of the same outside face of the lower post.
    outer_tube.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.0, 0.021, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
        name="cable_port",
    )
    outer_tube.visual(
        Cylinder(radius=0.0040, length=0.003),
        origin=Origin(xyz=(0.0, 0.027, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=MATTE_BLACK,
        name="port_opening",
    )
    outer_tube.visual(
        Box((0.029, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, 0.0235, 0.285)),
        material=DARK_STEEL,
        name="cable_anchor",
    )
    outer_tube.visual(
        Cylinder(radius=0.0032, length=0.035),
        origin=Origin(xyz=(0.0, 0.031, 0.292), rpy=(0.0, pi / 2.0, 0.0)),
        material=STEEL,
        name="anchor_pinch_bolt",
    )
    outer_tube.visual(
        Cylinder(radius=0.0048, length=0.004),
        origin=Origin(xyz=(0.0185, 0.031, 0.292), rpy=(0.0, pi / 2.0, 0.0)),
        material=STEEL,
        name="anchor_bolt_head",
    )
    cable = tube_from_spline_points(
        [
            (0.000, 0.027, 0.205),
            (0.004, 0.035, 0.225),
            (0.004, 0.037, 0.260),
            (0.001, 0.034, 0.292),
        ],
        radius=0.0022,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    outer_tube.visual(
        mesh_from_geometry(cable, "external_release_cable"),
        material=RUBBER,
        name="release_cable",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0135, length=0.405),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=DARK_STEEL,
        name="sliding_tube",
    )
    inner_post.visual(
        Cylinder(radius=0.0180, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.234)),
        material=BLACK_ANODIZED,
        name="crown_collar",
    )
    inner_post.visual(
        Cylinder(radius=0.0120, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.255), rpy=(pi / 2.0, 0.0, 0.0)),
        material=SATIN_ALLOY,
        name="tilt_pivot_axle",
    )
    inner_post.visual(
        Box((0.048, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.266)),
        material=BLACK_ANODIZED,
        name="crown_yoke",
    )
    inner_post.visual(
        Box((0.092, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.2764)),
        material=SATIN_ALLOY,
        name="clamp_plate",
    )
    inner_post.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(-0.030, 0.0, 0.2840)),
        material=STEEL,
        name="clamp_bolt_0",
    )
    inner_post.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.030, 0.0, 0.2840)),
        material=STEEL,
        name="clamp_bolt_1",
    )

    saddle = model.part("saddle")
    saddle_shell = section_loft(
        [
            _shell_section(-0.120, 0.145, 0.018, 0.074),
            _shell_section(-0.065, 0.142, 0.018, 0.073),
            _shell_section(0.015, 0.112, 0.016, 0.074),
            _shell_section(0.085, 0.070, 0.014, 0.077),
            _shell_section(0.150, 0.055, 0.014, 0.078),
        ],
        repair="mesh",
    )
    saddle.visual(
        mesh_from_geometry(saddle_shell, "flat_nose_saddle_shell"),
        material=MATTE_BLACK,
        name="saddle_shell",
    )

    for idx, y in enumerate((-0.018, 0.018)):
        rail = tube_from_spline_points(
            [
                (-0.095, y, 0.064),
                (-0.070, y, 0.041),
                (-0.030, y, 0.031),
                (0.038, y, 0.031),
                (0.095, y, 0.043),
                (0.125, y, 0.066),
            ],
            radius=0.0030,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
        saddle.visual(
            mesh_from_geometry(rail, f"saddle_rail_{idx}"),
            material=STEEL,
            name=f"rail_{idx}",
        )
        saddle.visual(
            Box((0.020, 0.010, 0.010)),
            origin=Origin(xyz=(-0.094, y, 0.063)),
            material=MATTE_BLACK,
            name=f"rear_boss_{idx}",
        )
        saddle.visual(
            Box((0.018, 0.010, 0.010)),
            origin=Origin(xyz=(0.124, y, 0.064)),
            material=MATTE_BLACK,
            name=f"front_boss_{idx}",
        )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.6, lower=0.0, upper=0.120),
    )
    model.articulation(
        "inner_to_saddle",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.20, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    inner = object_model.get_part("inner_post")
    saddle = object_model.get_part("saddle")
    lift = object_model.get_articulation("outer_to_inner")
    tilt = object_model.get_articulation("inner_to_saddle")

    ctx.allow_overlap(
        outer,
        inner,
        elem_a="top_wiper",
        elem_b="sliding_tube",
        reason="The rubber wiper seal is intentionally compressed around the sliding stanchion at the sleeve mouth.",
    )
    for rail_name in ("rail_0", "rail_1"):
        ctx.allow_overlap(
            saddle,
            inner,
            elem_a=rail_name,
            elem_b="clamp_plate",
            reason="The saddle rail is intentionally seated into the shallow two-bolt clamp plate groove.",
        )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="sliding_tube",
        outer_elem="outer_shell",
        margin=0.002,
        name="inner post remains concentric in the round lower tube",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="sliding_tube",
        elem_b="outer_shell",
        min_overlap=0.150,
        name="collapsed stanchion retains deep insertion in outer tube",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="sliding_tube",
        elem_b="top_wiper",
        min_overlap=0.015,
        name="wiper seal surrounds the sliding stanchion",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({lift: 0.120}):
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="sliding_tube",
            elem_b="outer_shell",
            min_overlap=0.055,
            name="extended dropper post still remains inserted",
        )
        raised_pos = ctx.part_world_position(inner)

    ctx.check(
        "prismatic joint extends the inner post upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    for rail_name in ("rail_0", "rail_1"):
        ctx.expect_overlap(
            saddle,
            inner,
            axes="xy",
            elem_a=rail_name,
            elem_b="clamp_plate",
            min_overlap=0.004,
            name=f"{rail_name} is captured over the two-bolt clamp plate",
        )
        ctx.expect_gap(
            saddle,
            inner,
            axis="z",
            positive_elem=rail_name,
            negative_elem="clamp_plate",
            max_penetration=0.001,
            max_gap=0.002,
            name=f"{rail_name} sits on the saddle clamp plate",
        )

    with ctx.pose({tilt: -0.18}):
        low_aabb = ctx.part_world_aabb(saddle)
    with ctx.pose({tilt: 0.18}):
        high_aabb = ctx.part_world_aabb(saddle)
    ctx.check(
        "tilt pivot changes the saddle shell attitude",
        low_aabb is not None
        and high_aabb is not None
        and abs(high_aabb[1][2] - low_aabb[1][2]) > 0.004,
        details=f"negative_tilt_aabb={low_aabb}, positive_tilt_aabb={high_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
