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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_bracket_comm_dish")

    mount_gray = model.material("mount_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    zinc_gray = model.material("zinc_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dish_white = model.material("dish_white", rgba=(0.91, 0.93, 0.95, 1.0))
    lnb_black = model.material("lnb_black", rgba=(0.10, 0.11, 0.12, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def tube_shell(outer_radius: float, inner_radius: float, length: float, *, segments: int = 56):
        half = 0.5 * length
        return LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        )

    reflector_shell = save_mesh(
        "reflector_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.0, -0.115),
                (0.055, -0.112),
                (0.135, -0.098),
                (0.220, -0.070),
                (0.282, -0.032),
                (0.300, 0.000),
            ],
            [
                (0.0, -0.109),
                (0.052, -0.106),
                (0.130, -0.092),
                (0.214, -0.065),
                (0.276, -0.029),
                (0.294, -0.004),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    reflector_rim = save_mesh(
        "reflector_rim",
        TorusGeometry(radius=0.298, tube=0.008, radial_segments=18, tubular_segments=84),
    )
    mast_collar_mesh = save_mesh("mast_collar", tube_shell(0.045, 0.028, 0.120))
    elevation_sleeve_mesh = save_mesh("elevation_sleeve", tube_shell(0.028, 0.021, 0.024, segments=48))
    feed_arm_mesh = save_mesh(
        "feed_arm",
        tube_from_spline_points(
            [
                (0.250, -0.046, -0.048),
                (0.300, -0.046, -0.056),
                (0.360, -0.046, -0.030),
                (0.410, -0.046, 0.000),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.012, 0.180, 0.300)),
        origin=Origin(xyz=(0.006, 0.000, 0.220)),
        material=mount_gray,
        name="wall_plate",
    )
    wall_mount.visual(
        Box((0.216, 0.040, 0.040)),
        origin=Origin(xyz=(0.114, 0.000, 0.300)),
        material=mount_gray,
        name="upper_arm",
    )
    wall_mount.visual(
        Box((0.216, 0.040, 0.040)),
        origin=Origin(xyz=(0.114, 0.000, 0.140)),
        material=mount_gray,
        name="lower_arm",
    )
    wall_mount.visual(
        Box((0.220, 0.030, 0.030)),
        origin=Origin(xyz=(0.112, 0.000, 0.218), rpy=(0.0, -0.93, 0.0)),
        material=mount_gray,
        name="brace_bar",
    )
    wall_mount.visual(
        Box((0.080, 0.080, 0.060)),
        origin=Origin(xyz=(0.220, 0.000, 0.100)),
        material=dark_steel,
        name="mast_socket",
    )
    wall_mount.visual(
        Cylinder(radius=0.024, length=0.460),
        origin=Origin(xyz=(0.240, 0.000, 0.270)),
        material=zinc_gray,
        name="mast_pipe",
    )
    wall_mount.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.240, 0.000, 0.500)),
        material=dark_steel,
        name="mast_cap",
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        mast_collar_mesh,
        material=dark_steel,
        name="mast_collar",
    )
    clamp_head.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(xyz=(0.028, 0.000, 0.000)),
        material=zinc_gray,
        name="mast_pad_outer",
    )
    clamp_head.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(xyz=(-0.028, 0.000, 0.000)),
        material=zinc_gray,
        name="mast_pad_inner",
    )
    clamp_head.visual(
        Box((0.076, 0.050, 0.024)),
        origin=Origin(xyz=(0.071, 0.000, 0.042)),
        material=dark_steel,
        name="upper_web",
    )
    clamp_head.visual(
        Box((0.076, 0.050, 0.024)),
        origin=Origin(xyz=(0.071, 0.000, -0.042)),
        material=dark_steel,
        name="lower_web",
    )
    clamp_head.visual(
        Box((0.044, 0.008, 0.120)),
        origin=Origin(xyz=(0.120, -0.022, 0.000)),
        material=dark_steel,
        name="inner_cheek",
    )
    clamp_head.visual(
        Box((0.044, 0.008, 0.120)),
        origin=Origin(xyz=(0.120, 0.022, 0.000)),
        material=mount_gray,
        name="outer_capture_plate",
    )

    dish_frame = model.part("dish_frame")
    dish_frame.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_journal",
    )
    dish_frame.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.000, -0.016, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="inner_trunnion_flange",
    )
    dish_frame.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.000, 0.016, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="outer_trunnion_flange",
    )
    dish_frame.visual(
        Box((0.090, 0.024, 0.120)),
        origin=Origin(xyz=(0.040, 0.000, 0.000)),
        material=dark_steel,
        name="support_bracket",
    )
    dish_frame.visual(
        Box((0.220, 0.056, 0.070)),
        origin=Origin(xyz=(0.150, -0.024, 0.000)),
        material=dark_steel,
        name="support_boom",
    )
    dish_frame.visual(
        Cylinder(radius=0.054, length=0.100),
        origin=Origin(xyz=(0.250, -0.046, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_hub",
    )
    dish_frame.visual(
        reflector_shell,
        origin=Origin(xyz=(0.330, -0.046, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="reflector_shell",
    )
    dish_frame.visual(
        reflector_rim,
        origin=Origin(xyz=(0.330, -0.046, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc_gray,
        name="reflector_rim",
    )
    dish_frame.visual(
        feed_arm_mesh,
        material=zinc_gray,
        name="feed_arm",
    )
    dish_frame.visual(
        Box((0.030, 0.026, 0.030)),
        origin=Origin(xyz=(0.415, -0.046, 0.000)),
        material=lnb_black,
        name="feed_block",
    )
    dish_frame.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.445, -0.046, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lnb_black,
        name="feed_horn",
    )

    model.articulation(
        "mast_azimuth",
        ArticulationType.CONTINUOUS,
        parent=wall_mount,
        child=clamp_head,
        origin=Origin(xyz=(0.240, 0.000, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5),
    )
    model.articulation(
        "dish_elevation",
        ArticulationType.REVOLUTE,
        parent=clamp_head,
        child=dish_frame,
        origin=Origin(xyz=(0.120, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_mount = object_model.get_part("wall_mount")
    clamp_head = object_model.get_part("clamp_head")
    dish_frame = object_model.get_part("dish_frame")
    azimuth = object_model.get_articulation("mast_azimuth")
    elevation = object_model.get_articulation("dish_elevation")

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
        "azimuth_axis_is_vertical",
        azimuth.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical mast yaw axis, got {azimuth.axis!r}.",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        elevation.axis == (0.0, -1.0, 0.0),
        details=f"Expected side-bracket elevation axis, got {elevation.axis!r}.",
    )

    ctx.expect_contact(
        clamp_head,
        wall_mount,
        elem_a="mast_pad_outer",
        elem_b="mast_pipe",
        name="mast_pad_bears_on_pipe",
    )
    ctx.expect_contact(
        clamp_head,
        wall_mount,
        elem_a="mast_pad_inner",
        elem_b="mast_pipe",
        name="inner_mast_pad_bears_on_pipe",
    )
    ctx.expect_gap(
        dish_frame,
        clamp_head,
        axis="y",
        positive_elem="inner_trunnion_flange",
        negative_elem="inner_cheek",
        max_penetration=0.00001,
        max_gap=0.002,
        name="inner_cheek_keeps_trunnion_captured",
    )
    ctx.expect_gap(
        clamp_head,
        dish_frame,
        axis="y",
        positive_elem="outer_capture_plate",
        negative_elem="outer_trunnion_flange",
        max_penetration=0.00001,
        max_gap=0.002,
        name="outer_plate_keeps_trunnion_captured",
    )
    ctx.expect_gap(
        dish_frame,
        wall_mount,
        axis="x",
        positive_elem="reflector_shell",
        negative_elem="wall_plate",
        min_gap=0.350,
        name="reflector_stands_off_from_wall_bracket",
    )

    feed_rest = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
    assert feed_rest is not None
    feed_rest_center_z = 0.5 * (feed_rest[0][2] + feed_rest[1][2])
    dish_rest_pos = ctx.part_world_position(dish_frame)
    assert dish_rest_pos is not None

    with ctx.pose({azimuth: math.radians(35.0)}):
        dish_yawed_pos = ctx.part_world_position(dish_frame)
        assert dish_yawed_pos is not None
        ctx.check(
            "azimuth_rotates_dish_around_mast",
            abs(dish_yawed_pos[0] - dish_rest_pos[0]) > 0.02
            and abs(dish_yawed_pos[1] - dish_rest_pos[1]) > 0.02,
            details=(
                f"Expected dish origin to move in plan under yaw; "
                f"rest={dish_rest_pos!r}, yawed={dish_yawed_pos!r}."
            ),
        )

    with ctx.pose({elevation: math.radians(38.0)}):
        ctx.expect_gap(
            dish_frame,
            clamp_head,
            axis="y",
            positive_elem="inner_trunnion_flange",
            negative_elem="inner_cheek",
            max_penetration=0.00001,
            max_gap=0.002,
            name="posed_inner_cheek_still_captures_flange",
        )
        ctx.expect_gap(
            clamp_head,
            dish_frame,
            axis="y",
            positive_elem="outer_capture_plate",
            negative_elem="outer_trunnion_flange",
            max_penetration=0.00001,
            max_gap=0.002,
            name="posed_outer_plate_still_captures_flange",
        )
        ctx.expect_gap(
            dish_frame,
            wall_mount,
            axis="x",
            positive_elem="reflector_shell",
            negative_elem="wall_plate",
            min_gap=0.350,
            name="tilted_reflector_still_clears_wall_plate",
        )
        feed_tilted = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        assert feed_tilted is not None
        feed_tilted_center_z = 0.5 * (feed_tilted[0][2] + feed_tilted[1][2])
        ctx.check(
            "positive_elevation_raises_feed_horn",
            feed_tilted_center_z > feed_rest_center_z + 0.10,
            details=(
                f"Expected feed horn to rise under positive elevation; "
                f"rest z={feed_rest_center_z:.3f}, posed z={feed_tilted_center_z:.3f}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
