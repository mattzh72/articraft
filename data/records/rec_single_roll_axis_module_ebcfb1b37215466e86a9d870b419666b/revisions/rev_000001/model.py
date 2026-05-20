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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_axis_spindle_module")

    frame_mat = model.material("frame_gray", color=(0.56, 0.57, 0.58, 1.0))
    cap_mat = model.material("cap_dark", color=(0.38, 0.39, 0.41, 1.0))
    spindle_mat = model.material("spindle_black", color=(0.18, 0.19, 0.20, 1.0))
    flange_mat = model.material("flange_steel", color=(0.44, 0.45, 0.47, 1.0))
    accent_mat = model.material("index_orange", color=(0.86, 0.46, 0.13, 1.0))

    foot_h = 0.012
    base_t = 0.016
    base_top = foot_h + base_t
    axis_z = 0.115

    base_len = 0.420
    base_w = 0.140
    rail_len = 0.340
    rail_w = 0.024
    rail_y = 0.048

    support_x = 0.112
    support_len = 0.028
    support_w = 0.082
    cheek_t = 0.014
    cheek_y = 0.032
    cap_t = 0.014

    shaft_r = 0.015
    lower_h = axis_z - shaft_r - base_top
    lower_z = base_top + lower_h / 2.0

    cap_z = axis_z + shaft_r + cap_t / 2.0
    column_h = axis_z + shaft_r - base_top
    column_z = base_top + column_h / 2.0

    frame = model.part("frame")
    frame.visual(
        Box((rail_len, rail_w, foot_h)),
        origin=Origin(xyz=(0.0, rail_y, foot_h / 2.0)),
        material=frame_mat,
        name="right_foot",
    )
    frame.visual(
        Box((rail_len, rail_w, foot_h)),
        origin=Origin(xyz=(0.0, -rail_y, foot_h / 2.0)),
        material=frame_mat,
        name="left_foot",
    )
    frame.visual(
        Box((base_len, base_w, base_t)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + base_t / 2.0)),
        material=frame_mat,
        name="base_plate",
    )

    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x = sign * support_x
        frame.visual(
            Box((support_len, 0.050, lower_h)),
            origin=Origin(xyz=(x, 0.0, lower_z)),
            material=frame_mat,
            name=f"{side_name}_lower_saddle",
        )
        frame.visual(
            Box((support_len, cheek_t, column_h)),
            origin=Origin(xyz=(x, cheek_y, column_z)),
            material=frame_mat,
            name=f"{side_name}_front_cheek",
        )
        frame.visual(
            Box((support_len, cheek_t, column_h)),
            origin=Origin(xyz=(x, -cheek_y, column_z)),
            material=frame_mat,
            name=f"{side_name}_rear_cheek",
        )
        frame.visual(
            Box((support_len + 0.010, support_w, cap_t)),
            origin=Origin(xyz=(x, 0.0, cap_z)),
            material=cap_mat,
            name=f"{side_name}_cap",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=shaft_r, length=0.320),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.142, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_mat,
        name="left_collar",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.142, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_mat,
        name="right_collar",
    )
    spindle.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_mat,
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.062, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_mat,
        name="tool_flange",
    )
    spindle.visual(
        Box((0.010, 0.018, 0.014)),
        origin=Origin(xyz=(0.013, 0.0, 0.055)),
        material=accent_mat,
        name="index_lug",
    )

    model.articulation(
        "frame_to_spindle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("frame_to_spindle")
    base_plate = frame.get_visual("base_plate")
    tool_flange = spindle.get_visual("tool_flange")
    lug = spindle.get_visual("index_lug")

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
        "roll joint uses shaft axis",
        roll.axis == (1.0, 0.0, 0.0),
        details=f"expected x-axis roll joint, got {roll.axis!r}",
    )
    ctx.expect_contact(
        spindle,
        frame,
        name="spindle shaft is clamped by the bearing blocks",
    )
    ctx.expect_origin_distance(
        frame,
        spindle,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="spindle axis stays centered between the bearing blocks",
    )
    ctx.expect_origin_gap(
        spindle,
        frame,
        axis="z",
        min_gap=0.114,
        max_gap=0.116,
        name="spindle roll axis sits at the intended bearing height",
    )
    ctx.expect_gap(
        spindle,
        frame,
        axis="z",
        positive_elem=tool_flange,
        negative_elem=base_plate,
        min_gap=0.020,
        name="tool flange clears the base plate",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple(
            (aabb[0][i] + aabb[1][i]) / 2.0
            for i in range(3)
        )

    with ctx.pose({roll: 0.0}):
        lug_aabb_0 = ctx.part_element_world_aabb(spindle, elem=lug)
    with ctx.pose({roll: math.pi / 2.0}):
        lug_aabb_90 = ctx.part_element_world_aabb(spindle, elem=lug)
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at quarter turn")

    if lug_aabb_0 is None or lug_aabb_90 is None:
        ctx.fail("index lug pose probe", "index lug bounds could not be evaluated")
    else:
        c0 = _aabb_center(lug_aabb_0)
        c1 = _aabb_center(lug_aabb_90)
        radial_motion = math.hypot(c1[1] - c0[1], c1[2] - c0[2])
        ctx.check(
            "index lug stays on one x station while rolling",
            abs(c1[0] - c0[0]) <= 0.001,
            details=f"lug moved {abs(c1[0] - c0[0]):.4f} m along x during roll",
        )
        ctx.check(
            "index lug visibly sweeps around the shaft",
            radial_motion >= 0.040,
            details=f"lug only moved {radial_motion:.4f} m in the y-z plane",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
