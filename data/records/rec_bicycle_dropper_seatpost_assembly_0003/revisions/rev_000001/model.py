from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _ring_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float, segments: int = 56):
    outer = superellipse_profile(outer_radius * 2.0, outer_radius * 2.0, exponent=2.0, segments=segments)
    inner = superellipse_profile(inner_radius * 2.0, inner_radius * 2.0, exponent=2.0, segments=segments)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height,
            center=False,
        ),
        name,
    )


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    z_center: float,
    radius_scale: float = 0.22,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    radius = min(width * radius_scale, height * 0.45)
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="internal_dropper_seatpost")

    black_anodized = model.material("black_anodized", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    hardcoat_black = model.material("hardcoat_black", rgba=(0.11, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    saddle_shell = model.material("saddle_shell", rgba=(0.18, 0.18, 0.19, 1.0))
    saddle_pad = model.material("saddle_pad", rgba=(0.09, 0.09, 0.10, 1.0))

    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.520)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )
    lower_sleeve.visual(
        _ring_mesh(
            "lower_clamp_sleeve",
            outer_radius=0.0190,
            inner_radius=0.0156,
            height=0.140,
        ),
        material=black_anodized,
        name="lower_clamp_sleeve",
    )
    lower_sleeve.visual(
        _ring_mesh(
            "outer_tube",
            outer_radius=0.01745,
            inner_radius=0.0156,
            height=0.380,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=hardcoat_black,
        name="outer_tube",
    )
    lower_sleeve.visual(
        Cylinder(radius=0.0190, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=black_anodized,
        name="sleeve_shoulder",
    )
    lower_sleeve.visual(
        Box((0.014, 0.038, 0.018)),
        origin=Origin(xyz=(-0.026, 0.0, 0.036)),
        material=dark_steel,
        name="pivot_mount_boss",
    )

    guide_collar = model.part("guide_collar")
    guide_collar.inertial = Inertial.from_geometry(
        Box((0.044, 0.044, 0.020)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    guide_collar.visual(
        _ring_mesh(
            "guide_collar_body",
            outer_radius=0.0205,
            inner_radius=0.0152,
            height=0.016,
        ),
        material=satin_black,
        name="guide_collar_body",
    )
    guide_collar.visual(
        _ring_mesh(
            "guide_wiper_lip",
            outer_radius=0.0185,
            inner_radius=0.0152,
            height=0.004,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=black_anodized,
        name="guide_wiper_lip",
    )

    inner_post = model.part("inner_post")
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0148, length=0.365),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.1825)),
    )
    inner_post.visual(
        Cylinder(radius=0.0148, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.1825)),
        material=dark_steel,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0138, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.341)),
        material=black_anodized,
        name="seal_head",
    )
    inner_post.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=black_anodized,
        name="upper_receiver",
    )

    base_pivot = model.part("base_pivot")
    base_pivot.inertial = Inertial.from_geometry(
        Box((0.056, 0.040, 0.034)),
        mass=0.08,
        origin=Origin(xyz=(-0.018, 0.0, 0.010)),
    )
    base_pivot.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    base_pivot.visual(
        Box((0.030, 0.010, 0.008)),
        origin=Origin(xyz=(-0.022, 0.0, 0.004)),
        material=dark_steel,
        name="actuator_arm",
    )
    base_pivot.visual(
        Cylinder(radius=0.0032, length=0.018),
        origin=Origin(xyz=(-0.026, 0.0, 0.009)),
        material=dark_steel,
        name="cable_pedestal",
    )
    base_pivot.visual(
        Cylinder(radius=0.0042, length=0.010),
        origin=Origin(xyz=(-0.026, 0.0, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="cable_eye",
    )

    clamp_head = model.part("clamp_head")
    clamp_head.inertial = Inertial.from_geometry(
        Box((0.066, 0.060, 0.046)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )
    clamp_head.visual(
        Box((0.022, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_anodized,
        name="yoke_stem",
    )
    clamp_head.visual(
        Cylinder(radius=0.0065, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_cartridge",
    )
    clamp_head.visual(
        Cylinder(radius=0.0028, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="side_bolt_shaft",
    )
    clamp_head.visual(
        Cylinder(radius=0.0048, length=0.004),
        origin=Origin(xyz=(0.0, 0.030, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="side_bolt_right",
    )
    clamp_head.visual(
        Cylinder(radius=0.0048, length=0.004),
        origin=Origin(xyz=(0.0, -0.030, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="side_bolt_left",
    )
    clamp_head.visual(
        Box((0.018, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=black_anodized,
        name="cradle_bridge",
    )
    clamp_head.visual(
        Box((0.020, 0.056, 0.005)),
        origin=Origin(xyz=(0.020, 0.0, 0.013)),
        material=black_anodized,
        name="front_lower_cradle",
    )
    clamp_head.visual(
        Box((0.020, 0.056, 0.005)),
        origin=Origin(xyz=(-0.020, 0.0, 0.013)),
        material=black_anodized,
        name="rear_lower_cradle",
    )
    clamp_head.visual(
        Box((0.020, 0.052, 0.005)),
        origin=Origin(xyz=(0.020, 0.0, 0.026)),
        material=satin_black,
        name="front_upper_cradle",
    )
    clamp_head.visual(
        Box((0.020, 0.052, 0.005)),
        origin=Origin(xyz=(-0.020, 0.0, 0.026)),
        material=satin_black,
        name="rear_upper_cradle",
    )
    clamp_head.visual(
        Cylinder(radius=0.0030, length=0.020),
        origin=Origin(xyz=(0.020, 0.0, 0.019)),
        material=brushed_steel,
        name="front_clamp_bolt",
    )
    clamp_head.visual(
        Cylinder(radius=0.0030, length=0.020),
        origin=Origin(xyz=(-0.020, 0.0, 0.019)),
        material=brushed_steel,
        name="rear_clamp_bolt",
    )

    saddle = model.part("saddle")
    saddle.inertial = Inertial.from_geometry(
        Box((0.255, 0.165, 0.092)),
        mass=0.34,
        origin=Origin(xyz=(-0.015, 0.0, 0.061)),
    )
    shell_sections = [
        _yz_section(-0.125, width=0.130, height=0.016, z_center=0.053),
        _yz_section(-0.085, width=0.150, height=0.018, z_center=0.054),
        _yz_section(-0.035, width=0.158, height=0.020, z_center=0.055),
        _yz_section(0.015, width=0.138, height=0.018, z_center=0.054),
        _yz_section(0.055, width=0.086, height=0.014, z_center=0.052),
        _yz_section(0.095, width=0.040, height=0.010, z_center=0.048),
    ]
    pad_sections = [
        _yz_section(-0.125, width=0.136, height=0.018, z_center=0.062),
        _yz_section(-0.085, width=0.156, height=0.022, z_center=0.064),
        _yz_section(-0.035, width=0.164, height=0.024, z_center=0.065),
        _yz_section(0.015, width=0.144, height=0.022, z_center=0.064),
        _yz_section(0.055, width=0.094, height=0.018, z_center=0.061),
        _yz_section(0.095, width=0.046, height=0.012, z_center=0.056),
    ]
    saddle.visual(
        mesh_from_geometry(section_loft(shell_sections), "saddle_shell_mesh_v2"),
        material=saddle_shell,
        name="saddle_shell",
    )
    saddle.visual(
        mesh_from_geometry(section_loft(pad_sections), "saddle_pad_mesh_v2"),
        material=saddle_pad,
        name="saddle_pad",
    )
    left_rail_points = [
        (-0.112, 0.024, 0.0185),
        (-0.080, 0.024, 0.0195),
        (-0.035, 0.024, 0.0205),
        (0.010, 0.0235, 0.0195),
        (0.056, 0.0215, 0.0175),
    ]
    saddle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                left_rail_points,
                radius=0.0035,
                samples_per_segment=18,
                radial_segments=16,
            ),
            "left_saddle_rail_v2",
        ),
        material=brushed_steel,
        name="left_rail",
    )
    saddle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                _mirror_y(left_rail_points),
                radius=0.0035,
                samples_per_segment=18,
                radial_segments=16,
            ),
            "right_saddle_rail_v2",
        ),
        material=brushed_steel,
        name="right_rail",
    )
    for support_name, support_origin in [
        ("left_rear_rail_support", (-0.082, 0.024, 0.034)),
        ("right_rear_rail_support", (-0.082, -0.024, 0.034)),
        ("left_front_rail_support", (0.012, 0.0235, 0.032)),
        ("right_front_rail_support", (0.012, -0.0235, 0.032)),
    ]:
        saddle.visual(
            Box((0.016, 0.010, 0.028)),
            origin=Origin(xyz=support_origin),
            material=saddle_shell,
            name=support_name,
        )
    saddle.visual(
        Box((0.072, 0.066, 0.008)),
        origin=Origin(xyz=(-0.058, 0.0, 0.050)),
        material=saddle_shell,
        name="rear_shell_bridge",
    )
    saddle.visual(
        Box((0.040, 0.054, 0.007)),
        origin=Origin(xyz=(0.014, 0.0, 0.045)),
        material=saddle_shell,
        name="front_shell_bridge",
    )

    model.articulation(
        "sleeve_to_collar",
        ArticulationType.FIXED,
        parent=lower_sleeve,
        child=guide_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
    )
    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.35, lower=0.0, upper=0.170),
    )
    model.articulation(
        "base_pivot_rotate",
        ArticulationType.REVOLUTE,
        parent=lower_sleeve,
        child=base_pivot,
        origin=Origin(xyz=(-0.040, 0.0, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-0.20, upper=0.45),
    )
    model.articulation(
        "post_to_clamp_head",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
    )
    model.articulation(
        "clamp_head_to_saddle",
        ArticulationType.FIXED,
        parent=clamp_head,
        child=saddle,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_sleeve = object_model.get_part("lower_sleeve")
    guide_collar = object_model.get_part("guide_collar")
    inner_post = object_model.get_part("inner_post")
    base_pivot = object_model.get_part("base_pivot")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")

    dropper_travel = object_model.get_articulation("dropper_travel")
    base_pivot_rotate = object_model.get_articulation("base_pivot_rotate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_isolated_part(
        inner_post,
        reason="The telescoping post rides on hidden internal bushings inside the lower sleeve with running clearance.",
    )
    ctx.allow_isolated_part(
        clamp_head,
        reason="The clamp head is rigidly mounted to the telescoping inner post that is supported by hidden internal bushings.",
    )
    ctx.allow_isolated_part(
        saddle,
        reason="The saddle is carried by the telescoping inner-post assembly, whose internal glide bushings are intentionally not modeled.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(guide_collar, lower_sleeve, name="guide_collar_seated_on_sleeve")
    ctx.expect_contact(base_pivot, lower_sleeve, name="base_pivot_attached_to_mount_boss")
    ctx.expect_contact(clamp_head, inner_post, name="clamp_head_seated_on_inner_post")
    ctx.expect_contact(saddle, clamp_head, name="saddle_rails_supported_by_cradle")
    ctx.expect_gap(
        clamp_head,
        guide_collar,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="dropped_head_sits_low_on_guide_collar",
    )
    ctx.expect_overlap(
        saddle,
        clamp_head,
        axes="xy",
        min_overlap=0.035,
        name="saddle_centered_over_clamp_head",
    )

    clamp_rest = ctx.part_world_position(clamp_head)
    assert clamp_rest is not None
    with ctx.pose({dropper_travel: 0.170}):
        clamp_extended = ctx.part_world_position(clamp_head)
        assert clamp_extended is not None
        ctx.check(
            "dropper_travel_moves_clamp_upward",
            clamp_extended[2] > clamp_rest[2] + 0.165,
            details=f"rest_z={clamp_rest[2]:.4f}, extended_z={clamp_extended[2]:.4f}",
        )
        ctx.expect_gap(
            clamp_head,
            guide_collar,
            axis="z",
            min_gap=0.168,
            max_gap=0.172,
            name="full_extension_matches_rated_travel",
        )

    eye_rest = ctx.part_element_world_aabb(base_pivot, elem="cable_eye")
    assert eye_rest is not None
    with ctx.pose({base_pivot_rotate: 0.35}):
        eye_actuated = ctx.part_element_world_aabb(base_pivot, elem="cable_eye")
        assert eye_actuated is not None
        ctx.check(
            "base_pivot_routes_cable_through_rotation",
            eye_actuated[1][2] > eye_rest[1][2] + 0.006
            and eye_actuated[0][0] > eye_rest[0][0] + 0.007,
            details=f"rest={eye_rest}, actuated={eye_actuated}",
        )
        ctx.expect_contact(base_pivot, lower_sleeve, name="base_pivot_stays_supported_when_actuated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
