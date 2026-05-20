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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_sleeve_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 48,
):
    half = length * 0.5
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_flood_mast")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.21, 0.24, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.82, 0.90, 0.55))

    frame_length = 1.30
    frame_width = 0.90
    rail_size = 0.08
    frame_height = 0.06
    hinge_x = -0.55
    hinge_z = 0.30
    head_pivot_x = 0.22

    mast_hinge_sleeve = Cylinder(radius=0.028, length=0.18)
    trunnion_sleeve = Cylinder(radius=0.028, length=0.04)

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((rail_size, frame_width, frame_height)),
        origin=Origin(xyz=(-0.61, 0.0, frame_height * 0.5)),
        material=galvanized_steel,
        name="front_sill",
    )
    base_frame.visual(
        Box((rail_size, frame_width, frame_height)),
        origin=Origin(xyz=(0.61, 0.0, frame_height * 0.5)),
        material=galvanized_steel,
        name="rear_sill",
    )
    base_frame.visual(
        Box((frame_length - (2.0 * rail_size), rail_size, frame_height)),
        origin=Origin(xyz=(0.0, 0.41, frame_height * 0.5)),
        material=galvanized_steel,
        name="left_rail",
    )
    base_frame.visual(
        Box((frame_length - (2.0 * rail_size), rail_size, frame_height)),
        origin=Origin(xyz=(0.0, -0.41, frame_height * 0.5)),
        material=galvanized_steel,
        name="right_rail",
    )
    base_frame.visual(
        Box((rail_size, frame_width - (2.0 * rail_size), frame_height)),
        origin=Origin(xyz=(0.02, 0.0, frame_height * 0.5)),
        material=galvanized_steel,
        name="center_crossmember",
    )
    base_frame.visual(
        Box((rail_size, frame_width - (2.0 * rail_size), frame_height)),
        origin=Origin(xyz=(0.38, 0.0, frame_height * 0.5)),
        material=galvanized_steel,
        name="rear_crossmember",
    )
    base_frame.visual(
        Box((0.86, 0.50, 0.012)),
        origin=Origin(xyz=(0.16, 0.0, 0.066)),
        material=dark_steel,
        name="equipment_deck",
    )
    base_frame.visual(
        Box((0.16, 0.42, 0.08)),
        origin=Origin(xyz=(hinge_x, 0.0, 0.10)),
        material=dark_steel,
        name="hinge_web",
    )
    base_frame.visual(
        Box((0.08, 0.08, 0.22)),
        origin=Origin(xyz=(hinge_x, 0.14, 0.25)),
        material=dark_steel,
        name="left_hinge_stand",
    )
    base_frame.visual(
        Box((0.08, 0.08, 0.22)),
        origin=Origin(xyz=(hinge_x, -0.14, 0.25)),
        material=dark_steel,
        name="right_hinge_stand",
    )
    base_frame.visual(
        Cylinder(radius=0.020, length=0.20),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    base_frame.visual(
        Box((0.20, 0.46, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.10)),
        material=dark_steel,
        name="transport_support",
    )
    base_frame.visual(
        Box((0.20, 0.46, 0.02)),
        origin=Origin(xyz=(0.30, 0.0, 0.15)),
        material=dark_steel,
        name="transport_saddle",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((1.30, 0.90, 0.36)),
        mass=115.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    mast = model.part("mast")
    mast.visual(
        mast_hinge_sleeve,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_sleeve",
    )
    mast.visual(
        Box((0.12, 0.18, 0.072)),
        origin=Origin(xyz=(0.06, 0.0, 0.064)),
        material=dark_steel,
        name="mast_foot",
    )
    mast.visual(
        Box((0.12, 0.10, 0.08)),
        origin=Origin(xyz=(0.06, 0.0, 0.14)),
        material=dark_steel,
        name="mast_lower_shank",
    )
    mast.visual(
        Box((0.12, 0.08, 1.44)),
        origin=Origin(xyz=(0.08, 0.0, 0.90)),
        material=galvanized_steel,
        name="mast_post",
    )
    mast.visual(
        Box((0.164, 0.24, 0.14)),
        origin=Origin(xyz=(0.12, 0.0, 1.69)),
        material=dark_steel,
        name="head_mount_block",
    )
    mast.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(0.216, 0.12, 1.77), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_yoke_pin",
    )
    mast.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(0.216, -0.12, 1.77), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_yoke_pin",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.26, 0.40, 1.78)),
        mass=62.0,
        origin=Origin(xyz=(0.09, 0.0, 0.89)),
    )

    flood_head = model.part("flood_head")
    flood_head.visual(
        Box((0.164, 0.32, 0.18)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=dark_steel,
        name="rear_mount_box",
    )
    flood_head.visual(
        Box((0.052, 0.04, 0.18)),
        origin=Origin(xyz=(0.054, 0.18, 0.0)),
        material=dark_steel,
        name="left_trunnion_cheek",
    )
    flood_head.visual(
        Box((0.052, 0.04, 0.18)),
        origin=Origin(xyz=(0.054, -0.18, 0.0)),
        material=dark_steel,
        name="right_trunnion_cheek",
    )
    flood_head.visual(
        Box((0.28, 0.48, 0.24)),
        origin=Origin(xyz=(0.332, 0.0, 0.0)),
        material=lamp_black,
        name="head_housing",
    )
    flood_head.visual(
        Box((0.03, 0.52, 0.28)),
        origin=Origin(xyz=(0.487, 0.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    flood_head.visual(
        Box((0.004, 0.46, 0.22)),
        origin=Origin(xyz=(0.504, 0.0, 0.0)),
        material=glass,
        name="lens_panel",
    )
    flood_head.visual(
        Box((0.10, 0.54, 0.02)),
        origin=Origin(xyz=(0.42, 0.0, 0.13)),
        material=dark_steel,
        name="rain_visor",
    )
    flood_head.visual(
        trunnion_sleeve,
        origin=Origin(xyz=(0.0, 0.19, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_sleeve",
    )
    flood_head.visual(
        trunnion_sleeve,
        origin=Origin(xyz=(0.0, -0.19, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_sleeve",
    )
    flood_head.inertial = Inertial.from_geometry(
        Box((0.52, 0.54, 0.28)),
        mass=26.0,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_mast",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=mast,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.55,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=flood_head,
        origin=Origin(xyz=(head_pivot_x, 0.0, 1.77)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.1,
            lower=-math.radians(105.0),
            upper=math.radians(20.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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
    base_frame = object_model.get_part("base_frame")
    mast = object_model.get_part("mast")
    flood_head = object_model.get_part("flood_head")
    frame_to_mast = object_model.get_articulation("frame_to_mast")
    mast_to_head = object_model.get_articulation("mast_to_head")

    ctx.allow_overlap(
        base_frame,
        mast,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The transport mast rotates on a real hinge pin captured inside a sleeve.",
    )
    ctx.allow_overlap(
        flood_head,
        mast,
        elem_a="left_trunnion_sleeve",
        elem_b="left_yoke_pin",
        reason="The flood head pivots on a left trunnion pin running inside its sleeve.",
    )
    ctx.allow_overlap(
        flood_head,
        mast,
        elem_a="right_trunnion_sleeve",
        elem_b="right_yoke_pin",
        reason="The flood head pivots on a right trunnion pin running inside its sleeve.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    def center_from_aabb(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.expect_contact(
        mast,
        base_frame,
        elem_a="hinge_sleeve",
        elem_b="hinge_pin",
        name="mast_hinge_is_physically_supported",
    )
    ctx.expect_contact(
        flood_head,
        mast,
        elem_a="left_trunnion_sleeve",
        elem_b="left_yoke_pin",
        name="left_trunnion_is_physically_supported",
    )
    ctx.expect_contact(
        flood_head,
        mast,
        elem_a="right_trunnion_sleeve",
        elem_b="right_yoke_pin",
        name="right_trunnion_is_physically_supported",
    )
    ctx.expect_gap(
        flood_head,
        mast,
        axis="x",
        positive_elem="head_housing",
        negative_elem="mast_post",
        min_gap=0.02,
        name="flood_head_clears_mast_front_face",
    )
    ctx.expect_origin_gap(
        flood_head,
        base_frame,
        axis="z",
        min_gap=1.70,
        name="head_axis_is_raised_above_roof_frame",
    )

    rest_head_housing = ctx.part_element_world_aabb(flood_head, elem="head_housing")
    assert rest_head_housing is not None
    rest_head_center = center_from_aabb(rest_head_housing)

    mast_limits = frame_to_mast.motion_limits
    head_limits = mast_to_head.motion_limits
    assert mast_limits is not None
    assert head_limits is not None
    assert mast_limits.lower is not None and mast_limits.upper is not None
    assert head_limits.lower is not None and head_limits.upper is not None

    with ctx.pose({frame_to_mast: mast_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="frame_to_mast_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="frame_to_mast_lower_no_floating")

    with ctx.pose({frame_to_mast: mast_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="frame_to_mast_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="frame_to_mast_upper_no_floating")
        ctx.expect_contact(
            mast,
            base_frame,
            elem_a="mast_post",
            elem_b="transport_saddle",
            name="folded_mast_rests_on_transport_saddle",
        )
        folded_mast_aabb = ctx.part_world_aabb(mast)
        assert folded_mast_aabb is not None
        folded_height = folded_mast_aabb[1][2] - folded_mast_aabb[0][2]
        ctx.check(
            "mast_folds_to_transport_height",
            folded_height < 0.30,
            details=f"Folded mast height {folded_height:.3f} m is not near horizontal transport height.",
        )

    with ctx.pose({mast_to_head: head_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_to_head_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="mast_to_head_lower_no_floating")
        up_housing = ctx.part_element_world_aabb(flood_head, elem="head_housing")
        assert up_housing is not None
        up_center = center_from_aabb(up_housing)
        ctx.check(
            "head_can_tilt_upward",
            up_center[2] > (rest_head_center[2] + 0.05),
            details=(
                f"Up-tilted housing center z={up_center[2]:.3f} m did not move above "
                f"rest z={rest_head_center[2]:.3f} m."
            ),
        )

    with ctx.pose({mast_to_head: head_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="mast_to_head_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="mast_to_head_upper_no_floating")
        down_housing = ctx.part_element_world_aabb(flood_head, elem="head_housing")
        assert down_housing is not None
        down_center = center_from_aabb(down_housing)
        ctx.check(
            "head_can_tilt_downward",
            down_center[2] < (rest_head_center[2] - 0.05),
            details=(
                f"Down-tilted housing center z={down_center[2]:.3f} m did not move below "
                f"rest z={rest_head_center[2]:.3f} m."
            ),
        )

    with ctx.pose({frame_to_mast: mast_limits.upper, mast_to_head: head_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="transport_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="transport_pose_no_floating")
        transport_head = ctx.part_element_world_aabb(flood_head, elem="head_housing")
        assert transport_head is not None
        ctx.check(
            "transport_pose_keeps_lamp_above_roof_plane",
            transport_head[0][2] > 0.0,
            details=f"Transport pose dips lamp below roof plane: min z={transport_head[0][2]:.3f} m.",
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
