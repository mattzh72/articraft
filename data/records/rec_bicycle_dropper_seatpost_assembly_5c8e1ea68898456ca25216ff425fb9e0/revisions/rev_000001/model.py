from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    *,
    radius: float,
) -> list[tuple[float, float, float]]:
    corner = min(radius, width * 0.45, height * 0.45)
    profile = rounded_rect_profile(width, height, corner, corner_segments=8)
    return [(x, y, z_center + z) for y, z in profile]


def _arc_points(
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    z: float = 0.0,
    samples: int = 9,
) -> list[tuple[float, float, float]]:
    return [
        (
            center[0] + radius * cos(radians(start_deg + (end_deg - start_deg) * i / (samples - 1))),
            center[1] + radius * sin(radians(start_deg + (end_deg - start_deg) * i / (samples - 1))),
            z,
        )
        for i in range(samples)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hardcoat_gray = model.material("hardcoat_gray", rgba=(0.34, 0.35, 0.37, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.69, 1.0))
    saddle_shell = model.material("saddle_shell", rgba=(0.10, 0.10, 0.11, 1.0))

    outer_tube = model.part("outer_tube")
    outer_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0158, 0.000),
            (0.0158, 0.162),
            (0.0166, 0.170),
            (0.0166, 0.176),
        ],
        [
            (0.0142, 0.003),
            (0.0142, 0.171),
        ],
        segments=56,
    )
    outer_tube.visual(_mesh("outer_sleeve", outer_shell), material=anodized_black, name="outer_sleeve")
    outer_tube.visual(
        Cylinder(radius=0.0166, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=matte_black,
        name="lower_band",
    )
    top_seal_geom = LatheGeometry.from_shell_profiles(
        [
            (0.0160, -0.009),
            (0.0180, -0.009),
            (0.0180, 0.009),
            (0.0168, 0.009),
        ],
        [
            (0.0142, -0.008),
            (0.0142, 0.008),
        ],
        segments=48,
    )
    outer_tube.visual(
        _mesh("top_seal_housing", top_seal_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=hardcoat_gray,
        name="top_seal_housing",
    )
    outer_tube.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.022, 0.173)),
        material=hardcoat_gray,
        name="hinge_bridge",
    )
    outer_tube.visual(
        Cylinder(radius=0.0050, length=0.004),
        origin=Origin(xyz=(-0.006, 0.0225, 0.188), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_tab_left",
    )
    outer_tube.visual(
        Cylinder(radius=0.0050, length=0.004),
        origin=Origin(xyz=(0.006, 0.0225, 0.188), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_tab_right",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.205)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
    )

    collar_latch = model.part("collar_latch")
    collar_band = tube_from_spline_points(
        _arc_points(0.025, 125.0, 395.0, center=(0.0, -0.0225), samples=11),
        radius=0.0035,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    collar_latch.visual(_mesh("collar_band", collar_band), material=hardcoat_gray, name="collar_band")
    collar_latch.visual(
        Cylinder(radius=0.0050, length=0.008),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    collar_latch.visual(
        Box((0.014, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0085, 0.0)),
        material=hardcoat_gray,
        name="hinge_cheek",
    )
    collar_latch.visual(
        Box((0.018, 0.012, 0.008)),
        origin=Origin(xyz=(-0.016, -0.002, 0.0)),
        material=hardcoat_gray,
        name="left_band_strap",
    )
    collar_latch.visual(
        Box((0.018, 0.012, 0.008)),
        origin=Origin(xyz=(0.016, -0.002, 0.0)),
        material=hardcoat_gray,
        name="right_band_strap",
    )
    collar_latch.visual(
        Box((0.012, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.001)),
        material=hardcoat_gray,
        name="latch_body",
    )
    collar_latch.visual(
        Box((0.012, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, 0.038, 0.009)),
        material=matte_black,
        name="lever_arm",
    )
    collar_latch.visual(
        Cylinder(radius=0.0040, length=0.018),
        origin=Origin(xyz=(0.0, 0.057, 0.013), rpy=(0.0, pi / 2.0, 0.0)),
        material=matte_black,
        name="lever_tip",
    )
    collar_latch.inertial = Inertial.from_geometry(
        Box((0.060, 0.110, 0.040)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.008, 0.010)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0130, length=0.265),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=anodized_black,
        name="inner_shaft",
    )
    inner_post.visual(
        Cylinder(radius=0.0162, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=hardcoat_gray,
        name="travel_stop_collar",
    )
    inner_post.visual(
        Box((0.026, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=hardcoat_gray,
        name="crown_block",
    )
    inner_post.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(xyz=(0.0, -0.010, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_tab_left",
    )
    inner_post.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(xyz=(0.0, 0.010, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_tab_right",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.275)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    clamp_head.visual(
        Box((0.024, 0.014, 0.010)),
        origin=Origin(xyz=(0.024, 0.0, 0.007)),
        material=hardcoat_gray,
        name="head_core",
    )
    clamp_head.visual(
        Box((0.024, 0.016, 0.008)),
        origin=Origin(xyz=(0.016, -0.014, 0.003)),
        material=hardcoat_gray,
        name="left_web",
    )
    clamp_head.visual(
        Box((0.024, 0.016, 0.008)),
        origin=Origin(xyz=(0.016, 0.014, 0.003)),
        material=hardcoat_gray,
        name="right_web",
    )
    clamp_head.visual(
        Cylinder(radius=0.0040, length=0.040),
        origin=Origin(xyz=(0.020, -0.022, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_cradle",
    )
    clamp_head.visual(
        Cylinder(radius=0.0040, length=0.040),
        origin=Origin(xyz=(0.020, 0.022, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_cradle",
    )
    clamp_head.visual(
        Box((0.022, 0.012, 0.008)),
        origin=Origin(xyz=(0.034, 0.0, 0.003)),
        material=hardcoat_gray,
        name="front_bridge",
    )
    clamp_head.inertial = Inertial.from_geometry(
        Box((0.070, 0.050, 0.030)),
        mass=0.16,
        origin=Origin(xyz=(0.020, 0.0, 0.004)),
    )

    saddle = model.part("saddle")
    shell_geom = section_loft(
        [
            _yz_section(-0.090, 0.138, 0.022, 0.040, radius=0.008),
            _yz_section(-0.040, 0.120, 0.026, 0.031, radius=0.008),
            _yz_section(0.020, 0.084, 0.024, 0.026, radius=0.007),
            _yz_section(0.085, 0.052, 0.019, 0.022, radius=0.006),
            _yz_section(0.145, 0.024, 0.014, 0.018, radius=0.004),
        ]
    )
    saddle.visual(_mesh("saddle_shell", shell_geom), material=saddle_shell, name="shell")

    rail_points = [
        (-0.085, 0.0, 0.011),
        (-0.060, 0.0, 0.006),
        (-0.022, 0.0, 0.0015),
        (0.000, 0.0, 0.000),
        (0.050, 0.0, 0.0005),
        (0.095, 0.0, 0.0045),
        (0.120, 0.0, 0.010),
    ]
    left_rail = tube_from_spline_points(
        [(x, -0.022, z) for x, _, z in rail_points],
        radius=0.0035,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    right_rail = tube_from_spline_points(
        [(x, 0.022, z) for x, _, z in rail_points],
        radius=0.0035,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    saddle.visual(_mesh("left_rail", left_rail), material=steel, name="left_rail")
    saddle.visual(_mesh("right_rail", right_rail), material=steel, name="right_rail")
    for name, x, y, z, sx, sy, sz in [
        ("left_rear_stanchion", -0.046, -0.022, 0.009, 0.016, 0.010, 0.026),
        ("right_rear_stanchion", -0.046, 0.022, 0.009, 0.016, 0.010, 0.026),
        ("left_front_stanchion", 0.040, -0.022, 0.009, 0.016, 0.010, 0.024),
        ("right_front_stanchion", 0.040, 0.022, 0.009, 0.016, 0.010, 0.024),
    ]:
        saddle.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(x, y, z)),
            material=matte_black,
            name=name,
        )
    saddle.inertial = Inertial.from_geometry(
        Box((0.280, 0.150, 0.070)),
        mass=0.28,
        origin=Origin(xyz=(0.028, 0.0, 0.028)),
    )

    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.070),
    )
    model.articulation(
        "collar_unlock",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=collar_latch,
        origin=Origin(xyz=(0.0, 0.0225, 0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=radians(75.0)),
    )
    model.articulation(
        "saddle_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=radians(-8.0),
            upper=radians(12.0),
        ),
    )
    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent=clamp_head,
        child=saddle,
        origin=Origin(xyz=(0.020, 0.0, 0.0175)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    collar_latch = object_model.get_part("collar_latch")
    inner_post = object_model.get_part("inner_post")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")

    dropper_travel = object_model.get_articulation("dropper_travel")
    collar_unlock = object_model.get_articulation("collar_unlock")
    saddle_tilt = object_model.get_articulation("saddle_tilt")

    outer_sleeve = outer_tube.get_visual("outer_sleeve")
    top_seal_housing = outer_tube.get_visual("top_seal_housing")
    hinge_tab_left = outer_tube.get_visual("hinge_tab_left")
    hinge_tab_right = outer_tube.get_visual("hinge_tab_right")

    hinge_barrel = collar_latch.get_visual("hinge_barrel")
    lever_tip = collar_latch.get_visual("lever_tip")

    inner_shaft = inner_post.get_visual("inner_shaft")
    pivot_tab_left = inner_post.get_visual("pivot_tab_left")
    pivot_tab_right = inner_post.get_visual("pivot_tab_right")

    pivot_barrel = clamp_head.get_visual("pivot_barrel")
    left_cradle = clamp_head.get_visual("left_cradle")
    right_cradle = clamp_head.get_visual("right_cradle")

    left_rail = saddle.get_visual("left_rail")
    right_rail = saddle.get_visual("right_rail")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(
        contact_tol=0.001,
        name="fail_if_isolated_parts_with_telescoping_fit_clearance",
    )
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem=inner_shaft,
        outer_elem=outer_sleeve,
        margin=0.003,
        name="inner shaft stays centered in the short outer tube",
    )
    ctx.expect_contact(
        inner_post,
        outer_tube,
        elem_a="travel_stop_collar",
        elem_b=top_seal_housing,
        contact_tol=0.0005,
        name="travel stop collar seats on the top bushing head",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="z",
        elem_a=inner_shaft,
        elem_b=outer_sleeve,
        min_overlap=0.120,
        name="collapsed dropper retains deep insertion",
    )
    ctx.expect_contact(
        collar_latch,
        outer_tube,
        elem_a=hinge_barrel,
        elem_b=hinge_tab_left,
        name="collar hinge bears on the left tab",
    )
    ctx.expect_contact(
        collar_latch,
        outer_tube,
        elem_a=hinge_barrel,
        elem_b=hinge_tab_right,
        name="collar hinge bears on the right tab",
    )
    ctx.expect_contact(
        clamp_head,
        inner_post,
        elem_a=pivot_barrel,
        elem_b=pivot_tab_left,
        name="tilt barrel seats on the left pivot tab",
    )
    ctx.expect_contact(
        clamp_head,
        inner_post,
        elem_a=pivot_barrel,
        elem_b=pivot_tab_right,
        name="tilt barrel seats on the right pivot tab",
    )
    ctx.expect_contact(
        saddle,
        clamp_head,
        elem_a=left_rail,
        elem_b=left_cradle,
        contact_tol=0.001,
        name="left saddle rail sits in the left cradle",
    )
    ctx.expect_contact(
        saddle,
        clamp_head,
        elem_a=right_rail,
        elem_b=right_cradle,
        contact_tol=0.001,
        name="right saddle rail sits in the right cradle",
    )

    rest_inner = ctx.part_world_position(inner_post)
    with ctx.pose({dropper_travel: 0.070}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem=inner_shaft,
            outer_elem=outer_sleeve,
            margin=0.003,
            name="extended shaft stays centered in the sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a=inner_shaft,
            elem_b=outer_sleeve,
            min_overlap=0.050,
            name="extended dropper keeps retained insertion",
        )
        extended_inner = ctx.part_world_position(inner_post)
    ctx.check(
        "dropper extension moves the post upward",
        rest_inner is not None and extended_inner is not None and extended_inner[2] > rest_inner[2] + 0.060,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    with ctx.pose({collar_unlock: radians(75.0)}):
        ctx.expect_gap(
            collar_latch,
            outer_tube,
            axis="z",
            min_gap=0.030,
            positive_elem=lever_tip,
            negative_elem=top_seal_housing,
            name="unlock lever flips up above the collar seat",
        )

    with ctx.pose({saddle_tilt: radians(-8.0)}):
        nose_down = ctx.part_world_position(saddle)
    with ctx.pose({saddle_tilt: radians(12.0)}):
        nose_up = ctx.part_world_position(saddle)
    ctx.check(
        "positive saddle tilt lifts the saddle nose",
        nose_down is not None and nose_up is not None and nose_up[2] > nose_down[2] + 0.005,
        details=f"nose_down={nose_down}, nose_up={nose_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
