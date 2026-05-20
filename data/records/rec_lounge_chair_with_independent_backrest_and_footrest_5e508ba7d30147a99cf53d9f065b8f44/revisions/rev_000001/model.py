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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _panel_section_xy(
    *,
    z: float,
    width: float,
    thickness: float,
    x_center: float = 0.0,
    radius: float = 0.02,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        thickness,
        width,
        min(radius, thickness * 0.45, width * 0.18),
        corner_segments=corner_segments,
    )
    return [(x + x_center, y, z) for x, y in profile]


def _panel_section_yz(
    *,
    x: float,
    width: float,
    thickness: float,
    z_center: float = 0.0,
    radius: float = 0.02,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        thickness,
        min(radius, thickness * 0.45, width * 0.18),
        corner_segments=corner_segments,
    )
    return [(x, y, z + z_center) for y, z in profile]


def _sling_section(
    *,
    x: float,
    half_width: float,
    edge_z: float,
    center_z: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    top = [
        (-half_width, edge_z),
        (-half_width * 0.62, edge_z - 0.012),
        (-half_width * 0.28, edge_z - 0.024),
        (0.0, center_z),
        (half_width * 0.28, edge_z - 0.024),
        (half_width * 0.62, edge_z - 0.012),
        (half_width, edge_z),
    ]
    bottom = [
        (half_width, edge_z - thickness),
        (half_width * 0.62, edge_z - 0.012 - thickness * 0.85),
        (half_width * 0.28, edge_z - 0.024 - thickness * 0.65),
        (0.0, center_z - thickness * 0.55),
        (-half_width * 0.28, edge_z - 0.024 - thickness * 0.65),
        (-half_width * 0.62, edge_z - 0.012 - thickness * 0.85),
        (-half_width, edge_z - thickness),
    ]
    return [(x, y, z) for y, z in top + bottom]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zero_gravity_lounge_chair")

    frame_metal = model.material("frame_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    sling_fabric = model.material("sling_fabric", rgba=(0.70, 0.61, 0.50, 1.0))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.56, 0.58, 0.60, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("side_frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.28, 0.72, 0.98)),
        mass=16.0,
        origin=Origin(xyz=(0.18, 0.0, 0.44)),
    )

    left_frame_path = [
        (-0.42, 0.33, 0.05),
        (-0.34, 0.33, 0.33),
        (-0.08, 0.33, 0.62),
        (0.00, 0.33, 0.50),
        (0.34, 0.33, 0.43),
        (0.50, 0.33, 0.25),
        (0.72, 0.33, 0.05),
    ]
    right_frame_path = _mirror_y(left_frame_path)

    frame.visual(
        _save_mesh(
            "zg_left_side_frame",
            wire_from_points(
                left_frame_path,
                radius=0.022,
                radial_segments=20,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.075,
                corner_segments=12,
            ),
        ),
        material=frame_metal,
        name="left_side_tube",
    )
    frame.visual(
        _save_mesh(
            "zg_right_side_frame",
            wire_from_points(
                right_frame_path,
                radius=0.022,
                radial_segments=20,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.075,
                corner_segments=12,
            ),
        ),
        material=frame_metal,
        name="right_side_tube",
    )

    for index, (x_pos, z_pos, radius, length) in enumerate(
        [
            (-0.18, 0.56, 0.016, 0.66),
            (-0.34, 0.33, 0.017, 0.68),
            (0.34, 0.43, 0.020, 0.68),
        ]
    ):
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x_pos, 0.0, z_pos), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=frame_metal,
            name=f"crossbar_{index}",
        )

    for side_sign in (-1.0, 1.0):
        y_box = side_sign * 0.312
        frame.visual(
            Box((0.045, 0.018, 0.090)),
            origin=Origin(xyz=(-0.08, y_box, 0.62)),
            material=frame_metal,
            name=f"back_pivot_tab_{'left' if side_sign > 0 else 'right'}",
        )
        frame.visual(
            Box((0.050, 0.020, 0.080)),
            origin=Origin(xyz=(0.50, side_sign * 0.337, 0.25)),
            material=frame_metal,
            name=f"leg_link_tab_{'left' if side_sign > 0 else 'right'}",
        )
        for joint_name, joint_x, joint_z, joint_sx in (
            ("crossbar_1_joint", -0.34, 0.33, 0.060),
            ("crossbar_2_joint", 0.34, 0.43, 0.065),
            ("leg_tab_joint", 0.455, 0.25, 0.100),
        ):
            frame.visual(
                Box(
                    (
                        0.160 if "crossbar" in joint_name else joint_sx,
                        0.040 if "crossbar" in joint_name else 0.014,
                        0.170 if "crossbar" in joint_name else 0.080,
                    )
                ),
                origin=Origin(
                    xyz=(
                        joint_x,
                        side_sign * (0.343 if "leg_tab" in joint_name else 0.330),
                        joint_z,
                    )
                ),
                material=frame_metal,
                name=f"{joint_name}_{'left' if side_sign > 0 else 'right'}",
            )

    seat_sling = model.part("seat_sling")
    seat_sling.inertial = Inertial.from_geometry(
        Box((0.40, 0.60, 0.08)),
        mass=4.0,
        origin=Origin(xyz=(0.18, 0.0, 0.46)),
    )

    sling_sections = [
        _sling_section(x=0.00, half_width=0.292, edge_z=0.495, center_z=0.456, thickness=0.010),
        _sling_section(x=0.12, half_width=0.292, edge_z=0.476, center_z=0.434, thickness=0.010),
        _sling_section(x=0.24, half_width=0.292, edge_z=0.452, center_z=0.420, thickness=0.010),
        _sling_section(x=0.30, half_width=0.292, edge_z=0.440, center_z=0.418, thickness=0.010),
    ]
    seat_sling.visual(
        _save_mesh("zg_seat_sling_shell", section_loft(sling_sections)),
        material=sling_fabric,
        name="seat_shell",
    )
    left_hem_path = [(0.00, 0.298, 0.495), (0.18, 0.298, 0.466), (0.30, 0.298, 0.440)]
    right_hem_path = _mirror_y(left_hem_path)
    seat_sling.visual(
        _save_mesh(
            "zg_left_seat_hem",
            wire_from_points(
                left_hem_path,
                radius=0.010,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.035,
                corner_segments=8,
            ),
        ),
        material=trim_dark,
        name="left_seat_hem",
    )
    seat_sling.visual(
        _save_mesh(
            "zg_right_seat_hem",
            wire_from_points(
                right_hem_path,
                radius=0.010,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.035,
                corner_segments=8,
            ),
        ),
        material=trim_dark,
        name="right_seat_hem",
    )

    back_panel = model.part("back_panel")
    back_panel.inertial = Inertial.from_geometry(
        Box((0.18, 0.58, 0.46)),
        mass=6.0,
        origin=Origin(xyz=(0.02, 0.0, 0.22)),
    )
    back_sections = [
        _panel_section_xy(z=0.03, width=0.54, thickness=0.055, x_center=0.040),
        _panel_section_xy(z=0.18, width=0.56, thickness=0.068, x_center=0.048),
        _panel_section_xy(z=0.31, width=0.52, thickness=0.074, x_center=0.053),
        _panel_section_xy(z=0.41, width=0.42, thickness=0.055, x_center=0.048),
    ]
    back_panel.visual(
        _save_mesh("zg_back_panel", section_loft(back_sections)),
        origin=Origin(rpy=(0.0, -0.58, 0.0)),
        material=cushion_fabric,
        name="back_cushion",
    )
    for side_sign in (-1.0, 1.0):
        side_name = "left" if side_sign > 0 else "right"
        back_panel.visual(
            Box((0.060, 0.050, 0.100)),
            origin=Origin(xyz=(0.012, side_sign * 0.255, 0.055)),
            material=frame_metal,
            name=f"back_bracket_{side_name}",
        )
        back_panel.visual(
            Cylinder(radius=0.028, length=0.048),
            origin=Origin(
                xyz=(0.0, side_sign * 0.279, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=frame_metal,
            name=f"back_knuckle_{side_name}",
        )

    leg_links = model.part("leg_support_links")
    leg_links.inertial = Inertial.from_geometry(
        Box((0.34, 0.62, 0.24)),
        mass=3.5,
        origin=Origin(xyz=(0.16, 0.0, 0.10)),
    )
    for side_sign in (-1.0, 1.0):
        side_name = "left" if side_sign > 0 else "right"
        leg_links.visual(
            Box((0.208, 0.016, 0.024)),
            origin=Origin(
                xyz=(0.118, side_sign * 0.305, 0.090),
                rpy=(0.0, -0.34, 0.0),
            ),
            material=frame_metal,
            name=f"support_link_{side_name}",
        )
        leg_links.visual(
            Cylinder(radius=0.022, length=0.046),
            origin=Origin(
                xyz=(0.0, side_sign * 0.304, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=frame_metal,
            name=f"rear_link_knuckle_{side_name}",
        )
        leg_links.visual(
            Box((0.090, 0.020, 0.100)),
            origin=Origin(xyz=(0.045, side_sign * 0.305, 0.050)),
            material=frame_metal,
            name=f"rear_link_stem_{side_name}",
        )
        leg_links.visual(
            Box((0.048, 0.018, 0.070)),
            origin=Origin(xyz=(0.240, side_sign * 0.305, 0.155)),
            material=frame_metal,
            name=f"front_link_tab_{side_name}",
        )
    leg_links.visual(
        Box((0.160, 0.620, 0.016)),
        origin=Origin(xyz=(0.135, 0.0, 0.082)),
        material=frame_metal,
        name="lower_link_brace",
    )
    leg_links.visual(
        Cylinder(radius=0.012, length=0.610),
        origin=Origin(xyz=(0.214, 0.0, 0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="upper_link_brace",
    )

    leg_rest = model.part("leg_rest")
    leg_rest.inertial = Inertial.from_geometry(
        Box((0.42, 0.56, 0.12)),
        mass=4.2,
        origin=Origin(xyz=(0.20, 0.0, 0.02)),
    )
    leg_sections = [
        _panel_section_yz(x=0.05, width=0.50, thickness=0.050, z_center=0.016),
        _panel_section_yz(x=0.15, width=0.54, thickness=0.066, z_center=0.026),
        _panel_section_yz(x=0.28, width=0.52, thickness=0.060, z_center=0.022),
        _panel_section_yz(x=0.38, width=0.44, thickness=0.045, z_center=0.016),
    ]
    leg_rest.visual(
        _save_mesh("zg_leg_rest_panel", section_loft(leg_sections)),
        origin=Origin(rpy=(0.0, 0.18, 0.0)),
        material=cushion_fabric,
        name="leg_pad",
    )
    for side_sign in (-1.0, 1.0):
        side_name = "left" if side_sign > 0 else "right"
        leg_rest.visual(
            Cylinder(radius=0.020, length=0.046),
            origin=Origin(
                xyz=(0.0, side_sign * 0.273, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=frame_metal,
            name=f"leg_knuckle_{side_name}",
        )
        leg_rest.visual(
            Box((0.080, 0.016, 0.070)),
            origin=Origin(
                xyz=(0.050, side_sign * 0.325, 0.000),
                rpy=(0.0, 0.12, 0.0),
            ),
            material=frame_metal,
            name=f"leg_clip_outer_{side_name}",
        )
        leg_rest.visual(
            Box((0.070, 0.100, 0.016)),
            origin=Origin(
                xyz=(0.055, side_sign * 0.285, 0.052),
                rpy=(0.0, 0.12, 0.0),
            ),
            material=frame_metal,
            name=f"leg_clip_lower_{side_name}",
        )
        leg_rest.visual(
            Box((0.120, 0.064, 0.060)),
            origin=Origin(xyz=(0.060, side_sign * 0.294, 0.015)),
            material=frame_metal,
            name=f"leg_side_rail_{side_name}",
        )

    model.articulation(
        "frame_to_seat_sling",
        ArticulationType.FIXED,
        parent=frame,
        child=seat_sling,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_back_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_panel,
        origin=Origin(xyz=(-0.08, 0.0, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.30,
            upper=0.50,
        ),
    )
    model.articulation(
        "frame_to_leg_support_links",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leg_links,
        origin=Origin(xyz=(0.50, 0.0, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.10,
            upper=0.70,
        ),
    )
    model.articulation(
        "links_to_leg_rest",
        ArticulationType.REVOLUTE,
        parent=leg_links,
        child=leg_rest,
        origin=Origin(xyz=(0.26, 0.0, 0.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")
    seat_sling = object_model.get_part("seat_sling")
    back_panel = object_model.get_part("back_panel")
    leg_links = object_model.get_part("leg_support_links")
    leg_rest = object_model.get_part("leg_rest")

    back_hinge = object_model.get_articulation("frame_to_back_panel")
    leg_link_hinge = object_model.get_articulation("frame_to_leg_support_links")
    footrest_hinge = object_model.get_articulation("links_to_leg_rest")

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

    ctx.expect_contact(seat_sling, frame, contact_tol=1e-4, name="seat sling is suspended from frame")
    ctx.expect_contact(back_panel, frame, contact_tol=1e-4, name="back panel pivot hardware touches frame")
    ctx.expect_contact(leg_links, frame, contact_tol=1e-4, name="leg support links hinge on frame")
    ctx.expect_contact(leg_rest, leg_links, contact_tol=1e-4, name="leg rest stays clipped to support links")
    ctx.expect_gap(leg_rest, seat_sling, axis="x", min_gap=0.03, name="leg rest stays ahead of seat")
    ctx.expect_within(seat_sling, frame, axes="y", margin=0.02, name="seat sling stays within frame width")

    ctx.check(
        "back hinge uses side-to-side axis",
        tuple(back_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={back_hinge.axis}",
    )
    ctx.check(
        "leg-link hinge uses side-to-side axis",
        tuple(leg_link_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={leg_link_hinge.axis}",
    )
    ctx.check(
        "footrest hinge uses side-to-side axis",
        tuple(footrest_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={footrest_hinge.axis}",
    )

    back_rest_aabb = ctx.part_element_world_aabb(back_panel, elem="back_cushion")
    leg_rest_aabb = ctx.part_element_world_aabb(leg_rest, elem="leg_pad")
    assert back_rest_aabb is not None
    assert leg_rest_aabb is not None

    with ctx.pose({back_hinge: 0.40}):
        back_upright_aabb = ctx.part_element_world_aabb(back_panel, elem="back_cushion")
        assert back_upright_aabb is not None
        ctx.expect_contact(back_panel, frame, contact_tol=1e-4)
        ctx.check(
            "back panel reclines on upper pivots",
            back_upright_aabb[1][0] > back_rest_aabb[1][0] + 0.08,
            details=f"rest_max_x={back_rest_aabb[1][0]:.4f}, posed_max_x={back_upright_aabb[1][0]:.4f}",
        )

    lifted_leg_max_z = leg_rest_aabb[1][2]
    with ctx.pose({leg_link_hinge: -0.45}):
        lifted_leg_aabb = ctx.part_element_world_aabb(leg_rest, elem="leg_pad")
        assert lifted_leg_aabb is not None
        ctx.expect_contact(leg_links, frame, contact_tol=1e-4)
        ctx.expect_contact(leg_rest, leg_links, contact_tol=1e-4)
        ctx.expect_gap(leg_rest, seat_sling, axis="x", min_gap=0.03)
        lifted_leg_max_z = lifted_leg_aabb[1][2]
        ctx.check(
            "leg support links elevate the footrest",
            lifted_leg_aabb[1][2] > leg_rest_aabb[1][2] + 0.20,
            details=f"rest_max_z={leg_rest_aabb[1][2]:.4f}, lifted_max_z={lifted_leg_aabb[1][2]:.4f}",
        )

    with ctx.pose({leg_link_hinge: -0.45, footrest_hinge: -0.35}):
        curled_leg_aabb = ctx.part_element_world_aabb(leg_rest, elem="leg_pad")
        assert curled_leg_aabb is not None
        ctx.expect_contact(leg_rest, leg_links, contact_tol=1e-4)
        ctx.check(
            "footrest rotates independently on the side links",
            curled_leg_aabb[1][2] > lifted_leg_max_z + 0.08,
            details=f"lifted_max_z={lifted_leg_max_z:.4f}, curled_max_z={curled_leg_aabb[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
