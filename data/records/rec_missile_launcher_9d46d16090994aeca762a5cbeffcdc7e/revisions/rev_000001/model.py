from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    tube_from_spline_points,
)


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_missile_launcher")

    base_gray = model.material("base_gray", rgba=(0.48, 0.50, 0.52, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    launcher_olive = model.material("launcher_olive", rgba=(0.32, 0.39, 0.24, 1.0))
    worn_olive = model.material("worn_olive", rgba=(0.39, 0.44, 0.28, 1.0))
    light_metal = model.material("light_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    optic_black = model.material("optic_black", rgba=(0.06, 0.07, 0.08, 1.0))
    lens_blue = model.material("lens_blue", rgba=(0.24, 0.37, 0.46, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((1.20, 1.20, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=base_gray,
        name="foundation_plinth",
    )
    pedestal_base.visual(
        Box((0.82, 0.82, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=dark_steel,
        name="pedestal_skirt",
    )
    pedestal_base.visual(
        Cylinder(radius=0.28, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=launcher_olive,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.40, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=dark_steel,
        name="bearing_ring",
    )
    pedestal_base.visual(
        Box((0.20, 0.52, 0.44)),
        origin=Origin(xyz=(0.26, 0.0, 0.42)),
        material=launcher_olive,
        name="service_box",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 1.28)),
        mass=780.0,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        Cylinder(radius=0.38, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="turntable_drum",
    )
    yaw_head.visual(
        Box((0.72, 0.58, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=launcher_olive,
        name="rotating_table",
    )
    yaw_head.visual(
        Box((0.30, 0.74, 0.28)),
        origin=Origin(xyz=(-0.12, 0.0, 0.36)),
        material=launcher_olive,
        name="lower_yoke_block",
    )
    yaw_head.visual(
        Box((0.24, 0.48, 0.26)),
        origin=Origin(xyz=(-0.30, 0.0, 0.37)),
        material=worn_olive,
        name="rear_power_box",
    )
    yaw_head.visual(
        Box((0.20, 0.06, 0.56)),
        origin=Origin(xyz=(-0.10, 0.40, 0.64)),
        material=launcher_olive,
        name="left_support_spine",
    )
    yaw_head.visual(
        Box((0.20, 0.06, 0.56)),
        origin=Origin(xyz=(-0.10, -0.40, 0.64)),
        material=launcher_olive,
        name="right_support_spine",
    )
    yaw_head.visual(
        Box((0.025, 0.12, 0.24)),
        origin=Origin(xyz=(-0.155, 0.36, 0.72)),
        material=worn_olive,
        name="left_cheek_a",
    )
    yaw_head.visual(
        Box((0.025, 0.12, 0.24)),
        origin=Origin(xyz=(-0.045, 0.36, 0.72)),
        material=worn_olive,
        name="left_cheek_b",
    )
    yaw_head.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(-0.10, 0.33, 0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_metal,
        name="left_bearing_pad",
    )
    yaw_head.visual(
        Box((0.025, 0.12, 0.24)),
        origin=Origin(xyz=(-0.155, -0.36, 0.72)),
        material=worn_olive,
        name="right_cheek_a",
    )
    yaw_head.visual(
        Box((0.025, 0.12, 0.24)),
        origin=Origin(xyz=(-0.045, -0.36, 0.72)),
        material=worn_olive,
        name="right_cheek_b",
    )
    yaw_head.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(-0.10, -0.33, 0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_metal,
        name="right_bearing_pad",
    )
    yaw_head.visual(
        Box((0.24, 0.82, 0.08)),
        origin=Origin(xyz=(-0.14, 0.0, 0.92)),
        material=launcher_olive,
        name="top_bridge",
    )
    yaw_head.inertial = Inertial.from_geometry(
        Box((0.72, 0.74, 0.96)),
        mass=240.0,
        origin=Origin(xyz=(-0.06, 0.0, 0.48)),
    )

    cradle_left_brace = _tube_mesh(
        "cradle_left_brace",
        [
            (-0.12, 0.12, -0.03),
            (0.18, 0.13, 0.00),
            (0.72, 0.14, 0.06),
            (1.42, 0.14, 0.10),
        ],
        radius=0.014,
    )
    cradle_right_brace = _tube_mesh(
        "cradle_right_brace",
        [
            (-0.12, -0.12, -0.03),
            (0.18, -0.13, 0.00),
            (0.72, -0.14, 0.06),
            (1.42, -0.14, 0.10),
        ],
        radius=0.014,
    )
    sight_cable = _tube_mesh(
        "sight_cable",
        [
            (-0.08, -0.10, 0.01),
            (0.10, -0.18, 0.04),
            (0.28, -0.21, 0.07),
            (0.42, -0.23, 0.10),
        ],
        radius=0.008,
    )

    launcher_cradle = model.part("launcher_cradle")
    launcher_cradle.visual(
        Box((0.22, 0.52, 0.16)),
        origin=Origin(xyz=(-0.06, 0.0, 0.0)),
        material=dark_steel,
        name="hub_block",
    )
    launcher_cradle.visual(
        Cylinder(radius=0.045, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=light_metal,
        name="trunnion_tube",
    )
    launcher_cradle.visual(
        Box((0.06, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, 0.285, 0.0)),
        material=dark_steel,
        name="left_trunnion_ear",
    )
    launcher_cradle.visual(
        Box((0.06, 0.03, 0.05)),
        origin=Origin(xyz=(0.0, -0.285, 0.0)),
        material=dark_steel,
        name="right_trunnion_ear",
    )
    launcher_cradle.visual(
        Box((0.74, 0.20, 0.12)),
        origin=Origin(xyz=(0.28, 0.0, 0.06)),
        material=launcher_olive,
        name="main_beam",
    )
    launcher_cradle.visual(
        Box((0.10, 0.36, 0.05)),
        origin=Origin(xyz=(0.24, 0.0, 0.14)),
        material=dark_steel,
        name="crossmember_rear",
    )
    launcher_cradle.visual(
        Box((0.10, 0.36, 0.05)),
        origin=Origin(xyz=(0.96, 0.0, 0.14)),
        material=dark_steel,
        name="crossmember_mid",
    )
    launcher_cradle.visual(
        Box((0.10, 0.36, 0.05)),
        origin=Origin(xyz=(1.72, 0.0, 0.14)),
        material=dark_steel,
        name="crossmember_front",
    )
    launcher_cradle.visual(
        Box((1.90, 0.05, 0.05)),
        origin=Origin(xyz=(1.07, 0.14, 0.18)),
        material=worn_olive,
        name="left_rail",
    )
    launcher_cradle.visual(
        Box((1.90, 0.05, 0.05)),
        origin=Origin(xyz=(1.07, -0.14, 0.18)),
        material=worn_olive,
        name="right_rail",
    )
    launcher_cradle.visual(
        Box((1.90, 0.016, 0.026)),
        origin=Origin(xyz=(1.07, 0.173, 0.193)),
        material=light_metal,
        name="left_rail_guide",
    )
    launcher_cradle.visual(
        Box((1.90, 0.016, 0.026)),
        origin=Origin(xyz=(1.07, -0.173, 0.193)),
        material=light_metal,
        name="right_rail_guide",
    )
    launcher_cradle.visual(
        Box((0.08, 0.05, 0.09)),
        origin=Origin(xyz=(2.03, 0.14, 0.19)),
        material=dark_steel,
        name="left_muzzle_stop",
    )
    launcher_cradle.visual(
        Box((0.08, 0.05, 0.09)),
        origin=Origin(xyz=(2.03, -0.14, 0.19)),
        material=dark_steel,
        name="right_muzzle_stop",
    )
    launcher_cradle.visual(cradle_left_brace, material=dark_steel, name="left_underbrace")
    launcher_cradle.visual(cradle_right_brace, material=dark_steel, name="right_underbrace")
    launcher_cradle.visual(
        Box((0.12, 0.16, 0.06)),
        origin=Origin(xyz=(0.34, -0.17, 0.10)),
        material=dark_steel,
        name="sight_bracket_inner",
    )
    launcher_cradle.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(0.42, -0.24, 0.12)),
        material=dark_steel,
        name="sight_bracket_outer",
    )
    launcher_cradle.visual(
        Box((0.26, 0.12, 0.18)),
        origin=Origin(xyz=(0.47, -0.31, 0.16)),
        material=optic_black,
        name="sight_box",
    )
    launcher_cradle.visual(
        Cylinder(radius=0.032, length=0.05),
        origin=Origin(xyz=(0.625, -0.31, 0.16), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_blue,
        name="sight_lens",
    )
    launcher_cradle.visual(sight_cable, material=dark_steel, name="sight_cable")
    launcher_cradle.inertial = Inertial.from_geometry(
        Box((2.14, 0.62, 0.32)),
        mass=115.0,
        origin=Origin(xyz=(0.86, 0.0, 0.10)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.8),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=yaw_head,
        child=launcher_cradle,
        origin=Origin(xyz=(-0.10, 0.0, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5500.0,
            velocity=0.9,
            lower=-0.20,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    yaw_head = object_model.get_part("yaw_head")
    launcher_cradle = object_model.get_part("launcher_cradle")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_rotation = object_model.get_articulation("elevation_rotation")

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

    ctx.check(
        "azimuth_axis_vertical",
        tuple(azimuth_rotation.axis) == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {azimuth_rotation.axis}",
    )
    ctx.check(
        "elevation_axis_horizontal",
        tuple(elevation_rotation.axis) == (0.0, 1.0, 0.0),
        f"expected left-right elevation axis, got {elevation_rotation.axis}",
    )

    ctx.expect_contact(
        yaw_head,
        pedestal_base,
        elem_a="turntable_drum",
        elem_b="bearing_ring",
        name="turntable_seats_on_pedestal_ring",
    )
    ctx.expect_overlap(
        yaw_head,
        pedestal_base,
        axes="xy",
        elem_a="turntable_drum",
        elem_b="bearing_ring",
        min_overlap=0.70,
        name="turntable_overlaps_bearing_ring",
    )
    ctx.expect_contact(
        launcher_cradle,
        yaw_head,
        elem_a="trunnion_tube",
        elem_b="left_bearing_pad",
        name="left_trunnion_contacts_bearing_pad",
    )
    ctx.expect_contact(
        launcher_cradle,
        yaw_head,
        elem_a="trunnion_tube",
        elem_b="right_bearing_pad",
        name="right_trunnion_contacts_bearing_pad",
    )

    with ctx.pose({elevation_rotation: 0.0}):
        ctx.expect_gap(
            launcher_cradle,
            yaw_head,
            axis="x",
            positive_elem="left_trunnion_ear",
            negative_elem="left_cheek_a",
            min_gap=0.008,
            max_gap=0.020,
            name="left_ear_front_slot_clearance_rest",
        )
        ctx.expect_gap(
            yaw_head,
            launcher_cradle,
            axis="x",
            positive_elem="left_cheek_b",
            negative_elem="left_trunnion_ear",
            min_gap=0.008,
            max_gap=0.020,
            name="left_ear_rear_slot_clearance_rest",
        )
        ctx.expect_gap(
            launcher_cradle,
            yaw_head,
            axis="x",
            positive_elem="right_trunnion_ear",
            negative_elem="right_cheek_a",
            min_gap=0.008,
            max_gap=0.020,
            name="right_ear_front_slot_clearance_rest",
        )
        ctx.expect_gap(
            yaw_head,
            launcher_cradle,
            axis="x",
            positive_elem="right_cheek_b",
            negative_elem="right_trunnion_ear",
            min_gap=0.008,
            max_gap=0.020,
            name="right_ear_rear_slot_clearance_rest",
        )

    with ctx.pose({elevation_rotation: 0.95}):
        ctx.expect_gap(
            launcher_cradle,
            yaw_head,
            axis="x",
            positive_elem="left_trunnion_ear",
            negative_elem="left_cheek_a",
            min_gap=0.004,
            max_gap=0.035,
            name="left_ear_front_slot_clearance_elevated",
        )
        ctx.expect_gap(
            yaw_head,
            launcher_cradle,
            axis="x",
            positive_elem="left_cheek_b",
            negative_elem="left_trunnion_ear",
            min_gap=0.004,
            max_gap=0.035,
            name="left_ear_rear_slot_clearance_elevated",
        )

    left_rail_aabb = ctx.part_element_world_aabb(launcher_cradle, elem="left_rail")
    right_rail_aabb = ctx.part_element_world_aabb(launcher_cradle, elem="right_rail")
    sight_box_aabb = ctx.part_element_world_aabb(launcher_cradle, elem="sight_box")
    if left_rail_aabb is not None and right_rail_aabb is not None:
        left_center_y = 0.5 * (left_rail_aabb[0][1] + left_rail_aabb[1][1])
        right_center_y = 0.5 * (right_rail_aabb[0][1] + right_rail_aabb[1][1])
        ctx.check(
            "twin_rails_are_parallel_and_separated",
            0.24 <= (left_center_y - right_center_y) <= 0.32,
            (
                "expected twin rails to stay side-by-side with about 0.28 m separation; "
                f"got {left_center_y - right_center_y:.3f} m"
            ),
        )
    if right_rail_aabb is not None and sight_box_aabb is not None:
        right_center_y = 0.5 * (right_rail_aabb[0][1] + right_rail_aabb[1][1])
        sight_center_y = 0.5 * (sight_box_aabb[0][1] + sight_box_aabb[1][1])
        ctx.check(
            "sight_box_is_offset_to_one_side",
            sight_center_y < (right_center_y - 0.10),
            (
                "expected sight box to sit outboard of one rail; "
                f"right rail y={right_center_y:.3f}, sight box y={sight_center_y:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
