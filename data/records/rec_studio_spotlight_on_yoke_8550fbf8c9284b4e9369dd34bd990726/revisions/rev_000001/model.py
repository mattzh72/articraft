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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_scale_ticks(part, *, side: str, y: float, pivot_z: float, material: str) -> None:
    tick_angles = (-0.70, -0.42, -0.16, 0.10, 0.36, 0.62)
    radius = 0.060
    for index, angle in enumerate(tick_angles):
        x = 0.010 + radius * math.cos(angle)
        z = pivot_z + radius * math.sin(angle)
        tick_length = 0.022 if index == 2 else 0.014
        part.visual(
            Box((tick_length, 0.0020, 0.0030)),
            origin=Origin(xyz=(x, y, z)),
            material=material,
            name=f"{side}_scale_tick_{index}",
        )

    part.visual(
        Box((0.028, 0.0020, 0.0040)),
        origin=Origin(xyz=(0.066, y, pivot_z - 0.009)),
        material=material,
        name=f"{side}_zero_tick",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_on_yoke")

    model.material("stand_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("trim_black", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("index_mark", rgba=(0.83, 0.84, 0.86, 1.0))
    model.material("accent_gray", rgba=(0.33, 0.34, 0.37, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.52, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material="stand_gray",
        name="base_plate",
    )
    stand.visual(
        Box((0.24, 0.20, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material="accent_gray",
        name="base_plinth",
    )
    stand.visual(
        Box((0.09, 0.09, 0.96)),
        origin=Origin(xyz=(0.0, 0.0, 0.614)),
        material="stand_gray",
        name="mast",
    )
    stand.visual(
        Box((0.16, 0.12, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 1.139)),
        material="accent_gray",
        name="head_block",
    )
    stand.visual(
        Box((0.13, 0.13, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material="trim_black",
        name="height_collar",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.04),
        origin=Origin(xyz=(0.075, 0.0, 0.835), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="knob_black",
        name="collar_lock_knob",
    )
    stand.visual(
        Box((0.05, 0.012, 0.012)),
        origin=Origin(xyz=(0.105, 0.0, 0.835)),
        material="knob_black",
        name="collar_lock_handle",
    )
    for index, (x, y) in enumerate(
        ((0.21, 0.13), (0.21, -0.13), (-0.21, 0.13), (-0.21, -0.13))
    ):
        stand.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.007)),
            material="trim_black",
            name=f"foot_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.05, 0.10, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="stand_gray",
        name="stem",
    )
    yoke.visual(
        Box((0.09, 0.23, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, 0.040)),
        material="stand_gray",
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.040, 0.10, 0.012)),
        origin=Origin(xyz=(-0.030, 0.0, 0.050)),
        material="accent_gray",
        name="saddle_pad",
    )
    yoke.visual(
        Box((0.18, 0.014, 0.30)),
        origin=Origin(xyz=(0.0, 0.105, 0.160)),
        material="body_black",
        name="left_arm",
    )
    yoke.visual(
        Box((0.18, 0.014, 0.30)),
        origin=Origin(xyz=(0.0, -0.105, 0.160)),
        material="body_black",
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, 0.124, 0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="knob_black",
        name="left_lock_knob",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, -0.124, 0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="knob_black",
        name="right_lock_knob",
    )
    yoke.visual(
        Box((0.05, 0.008, 0.012)),
        origin=Origin(xyz=(0.046, 0.124, 0.220)),
        material="knob_black",
        name="left_lock_handle",
    )
    yoke.visual(
        Box((0.05, 0.008, 0.012)),
        origin=Origin(xyz=(0.046, -0.124, 0.220)),
        material="knob_black",
        name="right_lock_handle",
    )
    _add_scale_ticks(yoke, side="left", y=0.111, pivot_z=0.220, material="index_mark")
    _add_scale_ticks(yoke, side="right", y=-0.111, pivot_z=0.220, material="index_mark")

    can = model.part("spotlight_can")
    can.visual(
        Cylinder(radius=0.086, length=0.24),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="body_black",
        name="body_shell",
    )
    can.visual(
        Cylinder(radius=0.098, length=0.035),
        origin=Origin(xyz=(0.1775, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="trim_black",
        name="front_bezel",
    )
    can.visual(
        Cylinder(radius=0.073, length=0.06),
        origin=Origin(xyz=(-0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="trim_black",
        name="rear_cap",
    )
    can.visual(
        Box((0.11, 0.028, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.092)),
        material="accent_gray",
        name="datum_rail",
    )
    can.visual(
        Box((0.050, 0.014, 0.008)),
        origin=Origin(xyz=(-0.072, 0.0, 0.080)),
        material="accent_gray",
        name="rear_service_pad",
    )
    can.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.089, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim_black",
        name="trunnion_left",
    )
    can.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, -0.089, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="trim_black",
        name="trunnion_right",
    )
    can.visual(
        Box((0.030, 0.004, 0.006)),
        origin=Origin(xyz=(0.054, 0.0, 0.098)),
        material="index_mark",
        name="top_index_bar",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.FIXED,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.184)),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=-0.25,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("spotlight_can")
    tilt = object_model.get_articulation("yoke_tilt")

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

    ctx.expect_contact(yoke, stand, elem_a="stem", elem_b="head_block")
    ctx.expect_contact(can, yoke, elem_a="trunnion_left", elem_b="left_arm")
    ctx.expect_contact(can, yoke, elem_a="trunnion_right", elem_b="right_arm")
    ctx.expect_gap(
        can,
        yoke,
        axis="z",
        positive_elem="body_shell",
        negative_elem="saddle_pad",
        min_gap=0.065,
        max_gap=0.090,
        name="can_clears_saddle_with_controlled_gap",
    )
    ctx.expect_gap(
        yoke,
        can,
        axis="y",
        positive_elem="left_arm",
        negative_elem="body_shell",
        min_gap=0.010,
        max_gap=0.016,
        name="left_arm_body_gap_is_tight_and_repeatable",
    )
    ctx.expect_gap(
        can,
        yoke,
        axis="y",
        positive_elem="body_shell",
        negative_elem="right_arm",
        min_gap=0.010,
        max_gap=0.016,
        name="right_arm_body_gap_is_tight_and_repeatable",
    )

    with ctx.pose({tilt: -0.25}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_low_tilt_limit")

    with ctx.pose({tilt: 1.05}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_high_tilt_limit")

    with ctx.pose({tilt: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(can, elem="front_bezel")
    with ctx.pose({tilt: 0.75}):
        raised_aabb = ctx.part_element_world_aabb(can, elem="front_bezel")

    if closed_aabb is None or raised_aabb is None:
        ctx.fail("front_bezel_pose_measurement_available", "Could not measure bezel in posed checks.")
    else:
        closed_center_z = 0.5 * (closed_aabb[0][2] + closed_aabb[1][2])
        raised_center_z = 0.5 * (raised_aabb[0][2] + raised_aabb[1][2])
        ctx.check(
            "positive_tilt_raises_beam_axis",
            raised_center_z > closed_center_z + 0.09,
            (
                f"Expected positive tilt to lift the front bezel by > 0.09 m, "
                f"got {raised_center_z - closed_center_z:.4f} m."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
