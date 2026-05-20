from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _yz_rect_loop(
    x: float,
    center_y: float,
    center_z: float,
    size_y: float,
    size_z: float,
) -> list[tuple[float, float, float]]:
    half_y = size_y * 0.5
    half_z = size_z * 0.5
    return [
        (x, center_y - half_y, center_z - half_z),
        (x, center_y + half_y, center_z - half_z),
        (x, center_y + half_y, center_z + half_z),
        (x, center_y - half_y, center_z + half_z),
    ]


def _section_prism(
    name: str,
    sections: list[tuple[float, float, float, float, float]],
):
    geom = section_loft(
        [_yz_rect_loop(x, cy, cz, sy, sz) for x, cy, cz, sy, sz in sections]
    )
    return mesh_from_geometry(geom, name)


def _utility_blade_mesh():
    blade_profile = [
        (0.079, 0.003),
        (0.099, 0.003),
        (0.111, 0.0085),
        (0.101, 0.015),
        (0.085, 0.015),
        (0.079, 0.011),
    ]
    blade_geom = ExtrudeGeometry(blade_profile, 0.0008)
    blade_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(blade_geom, "utility_knife_blade")


def _blade_clamp_mesh():
    clamp_profile = [
        (0.064, 0.0030),
        (0.082, 0.0030),
        (0.086, 0.0048),
        (0.084, 0.0078),
        (0.070, 0.0078),
        (0.065, 0.0057),
    ]
    clamp_geom = ExtrudeGeometry(clamp_profile, 0.012)
    clamp_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(clamp_geom, "utility_knife_blade_clamp")


def _thumb_slider_mesh():
    slider_profile = [
        (0.0245, 0.0183),
        (0.0270, 0.0170),
        (0.0385, 0.0170),
        (0.0415, 0.0183),
        (0.0400, 0.0222),
        (0.0360, 0.0234),
        (0.0300, 0.0234),
        (0.0260, 0.0222),
    ]
    slider_geom = ExtrudeGeometry(slider_profile, 0.012)
    slider_geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(slider_geom, "utility_knife_thumb_slider_v2")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    shell_yellow = model.material("shell_yellow", rgba=(0.94, 0.77, 0.10, 1.0))
    grip_black = model.material("grip_black", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.79, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.45, 0.48, 1.0))

    floor_t = 0.003
    wall_t = 0.003

    handle_shell = model.part("handle_shell")
    handle_shell.visual(
        _section_prism(
            "knife_floor_track",
            [
                (0.000, 0.000, floor_t * 0.5, 0.036, floor_t),
                (0.100, 0.000, floor_t * 0.5, 0.031, floor_t),
                (0.173, 0.000, floor_t * 0.5, 0.023, floor_t),
            ],
        ),
        material=shell_yellow,
        name="floor_track",
    )
    handle_shell.visual(
        _section_prism(
            "knife_left_wall",
            [
                (0.000, 0.0165, 0.0120, wall_t, 0.018),
                (0.100, 0.0140, 0.0115, wall_t, 0.017),
                (0.173, 0.0100, 0.0095, wall_t, 0.013),
            ],
        ),
        material=shell_yellow,
        name="left_wall",
    )
    handle_shell.visual(
        _section_prism(
            "knife_right_wall",
            [
                (0.000, -0.0165, 0.0120, wall_t, 0.018),
                (0.100, -0.0140, 0.0115, wall_t, 0.017),
                (0.173, -0.0100, 0.0095, wall_t, 0.013),
            ],
        ),
        material=shell_yellow,
        name="right_wall",
    )
    handle_shell.visual(
        _section_prism(
            "knife_left_top_rail",
            [
                (0.048, 0.0094, 0.01425, 0.0092, 0.0045),
                (0.110, 0.0082, 0.01380, 0.0076, 0.0045),
                (0.150, 0.00605, 0.01210, 0.0069, 0.0045),
            ],
        ),
        material=shell_yellow,
        name="left_top_rail",
    )
    handle_shell.visual(
        _section_prism(
            "knife_right_top_rail",
            [
                (0.048, -0.0094, 0.01425, 0.0092, 0.0045),
                (0.110, -0.0082, 0.01380, 0.0076, 0.0045),
                (0.150, -0.00605, 0.01210, 0.0069, 0.0045),
            ],
        ),
        material=shell_yellow,
        name="right_top_rail",
    )
    handle_shell.visual(
        Box((0.048, 0.030, 0.0045)),
        origin=Origin(xyz=(0.024, 0.000, 0.01425)),
        material=shell_yellow,
        name="rear_bridge",
    )
    handle_shell.visual(
        _section_prism(
            "knife_left_grip",
            [
                (0.020, 0.0178, 0.0100, 0.0020, 0.0080),
                (0.090, 0.0152, 0.0100, 0.0020, 0.0080),
                (0.145, 0.0121, 0.0092, 0.0020, 0.0070),
            ],
        ),
        material=grip_black,
        name="left_grip",
    )
    handle_shell.visual(
        _section_prism(
            "knife_right_grip",
            [
                (0.020, -0.0178, 0.0100, 0.0020, 0.0080),
                (0.090, -0.0152, 0.0100, 0.0020, 0.0080),
                (0.145, -0.0121, 0.0092, 0.0020, 0.0070),
            ],
        ),
        material=grip_black,
        name="right_grip",
    )
    handle_shell.inertial = Inertial.from_geometry(
        Box((0.176, 0.038, 0.022)),
        mass=0.30,
        origin=Origin(xyz=(0.088, 0.000, 0.011)),
    )

    carrier_assembly = model.part("carrier_assembly")
    carrier_assembly.visual(
        Box((0.082, 0.012, 0.004)),
        origin=Origin(xyz=(0.041, 0.000, 0.005)),
        material=dark_steel,
        name="carrier_plate",
    )
    carrier_assembly.visual(
        _blade_clamp_mesh(),
        material=dark_steel,
        name="blade_clamp",
    )
    carrier_assembly.visual(
        Box((0.008, 0.004, 0.0105)),
        origin=Origin(xyz=(0.033, 0.000, 0.01225)),
        material=dark_steel,
        name="slider_stem",
    )
    carrier_assembly.visual(
        _thumb_slider_mesh(),
        material=grip_black,
        name="thumb_slider",
    )
    carrier_assembly.visual(
        _utility_blade_mesh(),
        material=steel,
        name="blade",
    )
    carrier_assembly.inertial = Inertial.from_geometry(
        Box((0.111, 0.012, 0.020)),
        mass=0.06,
        origin=Origin(xyz=(0.0555, 0.000, 0.010)),
    )

    model.articulation(
        "handle_to_carrier",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=carrier_assembly,
        origin=Origin(xyz=(0.048, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=-0.018,
            upper=0.022,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_shell = object_model.get_part("handle_shell")
    carrier_assembly = object_model.get_part("carrier_assembly")
    carrier_slide = object_model.get_articulation("handle_to_carrier")

    floor_track = handle_shell.get_visual("floor_track")
    left_top_rail = handle_shell.get_visual("left_top_rail")
    right_top_rail = handle_shell.get_visual("right_top_rail")
    blade = carrier_assembly.get_visual("blade")
    carrier_plate = carrier_assembly.get_visual("carrier_plate")
    thumb_slider = carrier_assembly.get_visual("thumb_slider")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
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

    handle_aabb = ctx.part_world_aabb(handle_shell)
    assert handle_aabb is not None
    handle_length = handle_aabb[1][0] - handle_aabb[0][0]
    handle_width = handle_aabb[1][1] - handle_aabb[0][1]
    handle_height = handle_aabb[1][2] - handle_aabb[0][2]
    ctx.check(
        "handle_realistic_size",
        0.16 <= handle_length <= 0.19
        and 0.03 <= handle_width <= 0.045
        and 0.015 <= handle_height <= 0.03,
        details=(
            f"Expected a utility-knife sized handle, got "
            f"{handle_length:.4f} x {handle_width:.4f} x {handle_height:.4f} m."
        ),
    )

    ctx.expect_contact(
        carrier_assembly,
        handle_shell,
        elem_a=carrier_plate,
        elem_b=floor_track,
        name="carrier_plate_contacts_floor_track",
    )
    ctx.expect_overlap(
        carrier_assembly,
        handle_shell,
        axes="xy",
        elem_a=carrier_plate,
        elem_b=floor_track,
        min_overlap=0.010,
        name="carrier_plate_over_floor_track",
    )
    ctx.expect_within(
        carrier_assembly,
        handle_shell,
        axes="y",
        inner_elem=carrier_plate,
        outer_elem=floor_track,
        margin=0.0,
        name="carrier_plate_centered_in_handle",
    )
    ctx.expect_gap(
        carrier_assembly,
        handle_shell,
        axis="z",
        positive_elem=thumb_slider,
        negative_elem=left_top_rail,
        min_gap=0.0002,
        max_gap=0.0020,
        name="thumb_slider_above_left_rail",
    )
    ctx.expect_gap(
        carrier_assembly,
        handle_shell,
        axis="z",
        positive_elem=thumb_slider,
        negative_elem=right_top_rail,
        min_gap=0.0002,
        max_gap=0.0020,
        name="thumb_slider_above_right_rail",
    )

    limits = carrier_slide.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    with ctx.pose({carrier_slide: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="carrier_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="carrier_lower_no_floating")
        ctx.expect_contact(
            carrier_assembly,
            handle_shell,
            elem_a=carrier_plate,
            elem_b=floor_track,
            name="carrier_lower_track_contact",
        )
        ctx.expect_gap(
            carrier_assembly,
            handle_shell,
            axis="z",
            positive_elem=thumb_slider,
            negative_elem=left_top_rail,
            min_gap=0.0002,
            max_gap=0.0020,
            name="slider_lower_left_rail_clearance",
        )
        blade_low_aabb = ctx.part_element_world_aabb(carrier_assembly, elem=blade)
        slider_low_aabb = ctx.part_element_world_aabb(carrier_assembly, elem=thumb_slider)
        assert blade_low_aabb is not None
        assert slider_low_aabb is not None

    with ctx.pose({carrier_slide: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="carrier_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="carrier_upper_no_floating")
        ctx.expect_contact(
            carrier_assembly,
            handle_shell,
            elem_a=carrier_plate,
            elem_b=floor_track,
            name="carrier_upper_track_contact",
        )
        ctx.expect_gap(
            carrier_assembly,
            handle_shell,
            axis="z",
            positive_elem=thumb_slider,
            negative_elem=left_top_rail,
            min_gap=0.0002,
            max_gap=0.0020,
            name="slider_upper_left_rail_clearance",
        )
        blade_high_aabb = ctx.part_element_world_aabb(carrier_assembly, elem=blade)
        slider_high_aabb = ctx.part_element_world_aabb(carrier_assembly, elem=thumb_slider)
        assert blade_high_aabb is not None
        assert slider_high_aabb is not None

    handle_nose_x = handle_aabb[1][0]
    blade_low_tip_x = blade_low_aabb[1][0]
    blade_high_tip_x = blade_high_aabb[1][0]
    slider_low_center_x = (slider_low_aabb[0][0] + slider_low_aabb[1][0]) * 0.5
    slider_high_center_x = (slider_high_aabb[0][0] + slider_high_aabb[1][0]) * 0.5

    ctx.check(
        "blade_retracts_inside_nose",
        blade_low_tip_x < handle_nose_x - 0.010,
        details=(
            f"Retracted blade tip should sit inside the handle nose; "
            f"blade_tip={blade_low_tip_x:.4f}, handle_nose={handle_nose_x:.4f}."
        ),
    )
    ctx.check(
        "blade_extends_from_nose",
        blade_high_tip_x > handle_nose_x + 0.006,
        details=(
            f"Extended blade tip should protrude beyond the nose; "
            f"blade_tip={blade_high_tip_x:.4f}, handle_nose={handle_nose_x:.4f}."
        ),
    )
    ctx.check(
        "thumb_slider_moves_with_carrier",
        slider_high_center_x > slider_low_center_x + 0.035,
        details=(
            f"Thumb slider should travel with the carrier; "
            f"low={slider_low_center_x:.4f}, high={slider_high_center_x:.4f}."
        ),
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
