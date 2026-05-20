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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_W = 0.36
CASE_D = 0.24
LOWER_H = 0.075
LID_H = 0.055
WALL_T = 0.0035
FLOOR_T = 0.004
TOP_T = 0.004
CORNER_R = 0.028

HINGE_R = 0.007
LATCH_PIVOT_R = 0.005
LATCH_X = 0.105
LATCH_ARM_W = 0.046
LATCH_OPEN_ANGLE = 0.75

LOWER_FOAM_T = 0.022
LID_FOAM_T = 0.014

HINGE_ROD_LEN = CASE_W - 0.060
HINGE_AXIS_Y = CASE_D * 0.5 + HINGE_R
HINGE_AXIS_Z = LOWER_H + 0.003
LATCH_PIVOT_Y = -CASE_D * 0.5 - 0.028


def _rounded_rect_inset(width: float, depth: float, radius: float, inset: float) -> list[tuple[float, float]]:
    return rounded_rect_profile(
        width - 2.0 * inset,
        depth - 2.0 * inset,
        max(radius - inset, 0.002),
    )


def _rounded_panel_mesh(name: str, width: float, depth: float, radius: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius),
            thickness,
            center=True,
        ),
        name,
    )


def _rounded_ring_mesh(name: str, width: float, depth: float, radius: float, wall: float, height: float):
    outer = rounded_rect_profile(width, depth, radius)
    inner = _rounded_rect_inset(width, depth, radius, wall)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_instrument_case")

    shell_charcoal = model.material("shell_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    latch_black = model.material("latch_black", rgba=(0.07, 0.07, 0.08, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    foam_gray = model.material("foam_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.44, 0.45, 0.47, 1.0))

    inner_w = CASE_W - 2.0 * WALL_T
    inner_d = CASE_D - 2.0 * WALL_T
    inner_r = CORNER_R - WALL_T

    lower_ring_mesh = _rounded_ring_mesh("lower_wall_ring", CASE_W, CASE_D, CORNER_R, WALL_T, LOWER_H)
    lower_floor_mesh = _rounded_panel_mesh("lower_floor", inner_w, inner_d, inner_r, FLOOR_T)
    lower_foam_mesh = _rounded_panel_mesh(
        "lower_foam",
        inner_w - 0.020,
        inner_d - 0.020,
        max(inner_r - 0.010, 0.004),
        LOWER_FOAM_T,
    )

    lid_ring_mesh = _rounded_ring_mesh("lid_wall_ring", CASE_W, CASE_D, CORNER_R, WALL_T, LID_H)
    lid_top_mesh = _rounded_panel_mesh("lid_top_panel", inner_w, inner_d, inner_r, TOP_T)
    lid_foam_mesh = _rounded_panel_mesh(
        "lid_foam",
        inner_w - 0.020,
        inner_d - 0.020,
        max(inner_r - 0.010, 0.004),
        LID_FOAM_T,
    )

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        lower_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_H * 0.5)),
        material=shell_charcoal,
        name="lower_wall_ring",
    )
    lower_shell.visual(
        lower_floor_mesh,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T * 0.5)),
        material=shell_charcoal,
        name="lower_floor",
    )
    lower_shell.visual(
        lower_foam_mesh,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T + LOWER_FOAM_T * 0.5)),
        material=foam_gray,
        name="lower_foam",
    )
    lower_shell.visual(
        Box((0.085, 0.010, 0.006)),
        origin=Origin(xyz=(-0.108, -CASE_D * 0.5 + 0.005, LOWER_H - 0.004)),
        material=trim_gray,
        name="front_bumper_left",
    )
    lower_shell.visual(
        Box((0.085, 0.010, 0.006)),
        origin=Origin(xyz=(0.108, -CASE_D * 0.5 + 0.005, LOWER_H - 0.004)),
        material=trim_gray,
        name="front_bumper_right",
    )
    lower_shell.visual(
        Cylinder(radius=HINGE_R, length=HINGE_ROD_LEN),
        origin=Origin(
            xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_dark,
        name="rear_hinge_rod",
    )
    for side_x, side_name in ((-0.148, "left"), (0.148, "right")):
        lower_shell.visual(
            Box((0.012, 0.008, 0.010)),
            origin=Origin(xyz=(side_x, 0.123, 0.079)),
            material=hinge_dark,
            name=f"rear_hinge_support_{side_name}",
        )

    latch_pivot_z = LOWER_H - 0.010
    for side_name, latch_x in (("left", -LATCH_X), ("right", LATCH_X)):
        lower_shell.visual(
            Box((0.050, 0.028, 0.020)),
            origin=Origin(
                xyz=(
                    latch_x,
                    -CASE_D * 0.5 - 0.014,
                    latch_pivot_z,
                )
            ),
            material=hinge_dark,
            name=f"{side_name}_latch_mount",
        )
        lower_shell.visual(
            Cylinder(radius=LATCH_PIVOT_R, length=0.030),
            origin=Origin(
                xyz=(latch_x, LATCH_PIVOT_Y, latch_pivot_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_dark,
            name=f"{side_name}_latch_pin",
        )

    lower_shell.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D + 2.0 * HINGE_R, LOWER_H + 2.0 * HINGE_R)),
        mass=2.2,
        origin=Origin(
            xyz=(
                0.0,
                HINGE_R * 0.5,
                (LOWER_H + 2.0 * HINGE_R) * 0.5,
            )
        ),
    )

    lid_shell = model.part("lid_shell")
    lid_ring_y = -HINGE_AXIS_Y
    lid_ring_z = LOWER_H + LID_H * 0.5 - HINGE_AXIS_Z
    lid_shell.visual(
        lid_ring_mesh,
        origin=Origin(xyz=(0.0, lid_ring_y, lid_ring_z)),
        material=shell_charcoal,
        name="lid_wall_ring",
    )
    lid_shell.visual(
        lid_top_mesh,
        origin=Origin(
            xyz=(
                0.0,
                lid_ring_y,
                LOWER_H + LID_H - TOP_T * 0.5 - HINGE_AXIS_Z,
            )
        ),
        material=shell_charcoal,
        name="lid_top_panel",
    )
    lid_shell.visual(
        lid_foam_mesh,
        origin=Origin(
            xyz=(
                0.0,
                lid_ring_y,
                LOWER_H + LID_H - TOP_T - LID_FOAM_T * 0.5 - HINGE_AXIS_Z,
            )
        ),
        material=foam_gray,
        name="lid_foam",
    )
    lid_shell.visual(
        Cylinder(radius=HINGE_R + 0.002, length=HINGE_ROD_LEN - 0.020),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hinge_dark,
        name="lid_hinge_sleeve",
    )
    for side_name, catch_x in (("left", -LATCH_X), ("right", LATCH_X)):
        lid_shell.visual(
            Box((0.040, 0.014, 0.012)),
            origin=Origin(
                xyz=(
                    catch_x,
                    -0.239,
                    0.010,
                )
            ),
            material=trim_gray,
            name=f"{side_name}_latch_catch",
        )

    lid_shell.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D + HINGE_R, LID_H + HINGE_R)),
        mass=1.4,
        origin=Origin(
            xyz=(
                0.0,
                -(CASE_D * 0.5 + HINGE_R * 0.5),
                LID_H * 0.5 - HINGE_R * 0.5,
            )
        ),
    )

    left_latch = model.part("left_latch")
    right_latch = model.part("right_latch")
    for latch_part, side in ((left_latch, "left"), (right_latch, "right")):
        latch_part.visual(
            Cylinder(radius=LATCH_PIVOT_R, length=0.030),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=hinge_dark,
            name="latch_barrel",
        )
        latch_part.visual(
            Box((LATCH_ARM_W, 0.020, 0.060)),
            origin=Origin(xyz=(0.0, 0.010, -0.020)),
            material=latch_black,
            name="latch_body",
        )
        latch_part.visual(
            Box((0.034, 0.014, 0.016)),
            origin=Origin(xyz=(0.0, 0.018, 0.010)),
            material=latch_black,
            name="latch_hook",
        )
        latch_part.visual(
            Box((0.038, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, 0.026, -0.044)),
            material=latch_black,
            name=f"{side}_finger_tab",
        )
        latch_part.inertial = Inertial.from_geometry(
            Box((0.050, 0.050, 0.070)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.015, -0.020)),
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid_shell,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "left_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_latch,
        origin=Origin(xyz=(-LATCH_X, LATCH_PIVOT_Y, latch_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "right_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_latch,
        origin=Origin(xyz=(LATCH_X, LATCH_PIVOT_Y, latch_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_shell = object_model.get_part("lower_shell")
    lid_shell = object_model.get_part("lid_shell")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_latch_pivot = object_model.get_articulation("left_latch_pivot")
    right_latch_pivot = object_model.get_articulation("right_latch_pivot")

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
    ctx.allow_overlap(
        lid_shell,
        lower_shell,
        elem_a="lid_hinge_sleeve",
        elem_b="rear_hinge_rod",
        reason="Captured rear hinge rod passes through the lid sleeve.",
    )
    ctx.allow_overlap(
        lid_shell,
        lower_shell,
        elem_a="lid_hinge_sleeve",
        elem_b="lower_wall_ring",
        reason="The rear hinge sleeve nests into a shallow molded gutter at the lower shell rim.",
    )
    ctx.allow_overlap(
        left_latch,
        lower_shell,
        reason="Left latch sits in a molded front recess and rotates around a captured pin.",
    )
    ctx.allow_overlap(
        right_latch,
        lower_shell,
        reason="Right latch sits in a molded front recess and rotates around a captured pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    with ctx.pose({lid_hinge: 0.0, left_latch_pivot: 0.0, right_latch_pivot: 0.0}):
        ctx.expect_gap(
            lid_shell,
            lower_shell,
            axis="z",
            positive_elem="lid_wall_ring",
            negative_elem="lower_wall_ring",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid_closed_seam_gap",
        )
        ctx.expect_overlap(
            lid_shell,
            lower_shell,
            axes="xy",
            elem_a="lid_wall_ring",
            elem_b="lower_wall_ring",
            min_overlap=0.20,
            name="lid_closed_plan_overlap",
        )
        ctx.expect_overlap(
            lid_shell,
            lower_shell,
            axes="x",
            elem_a="lid_hinge_sleeve",
            elem_b="rear_hinge_rod",
            min_overlap=0.20,
            name="lid_hinge_axis_alignment",
        )
        ctx.check(
            "left_latch_barrel_present",
            ctx.part_element_world_aabb(left_latch, elem="latch_barrel") is not None,
            "left latch barrel AABB missing",
        )
        ctx.check(
            "right_latch_barrel_present",
            ctx.part_element_world_aabb(right_latch, elem="latch_barrel") is not None,
            "right latch barrel AABB missing",
        )

    lid_top_rest = ctx.part_element_world_aabb(lid_shell, elem="lid_top_panel")
    ctx.check("lid_top_rest_exists", lid_top_rest is not None, "lid top panel AABB missing")
    if lid_top_rest is not None:
        lid_limits = lid_hinge.motion_limits
        assert lid_limits is not None and lid_limits.upper is not None
        with ctx.pose({lid_hinge: lid_limits.upper}):
            lid_top_open = ctx.part_element_world_aabb(lid_shell, elem="lid_top_panel")
            ctx.check("lid_top_open_exists", lid_top_open is not None, "open lid top panel AABB missing")
            if lid_top_open is not None:
                ctx.check(
                    "lid_opens_upward",
                    lid_top_open[1][2] > lid_top_rest[1][2] + 0.10,
                    f"expected lid top max z to rise by >0.10 m, got rest={lid_top_rest[1][2]:.4f}, open={lid_top_open[1][2]:.4f}",
                )
                ctx.check(
                    "lid_swings_backward",
                    lid_top_open[1][1] > lid_top_rest[1][1] + 0.08,
                    f"expected lid to swing rearward, got rest max y={lid_top_rest[1][1]:.4f}, open max y={lid_top_open[1][1]:.4f}",
                )
            ctx.expect_overlap(
                lid_shell,
                lower_shell,
                axes="x",
                elem_a="lid_hinge_sleeve",
                elem_b="rear_hinge_rod",
                min_overlap=0.20,
                name="lid_hinge_alignment_at_open_limit",
            )

    left_latch_rest = ctx.part_world_aabb(left_latch)
    ctx.check("left_latch_rest_exists", left_latch_rest is not None, "left latch AABB missing")
    if left_latch_rest is not None:
        left_limits = left_latch_pivot.motion_limits
        assert left_limits is not None and left_limits.upper is not None
        with ctx.pose({left_latch_pivot: left_limits.upper}):
            left_latch_open = ctx.part_world_aabb(left_latch)
            ctx.check("left_latch_open_exists", left_latch_open is not None, "left latch open AABB missing")
            if left_latch_open is not None:
                ctx.check(
                    "left_latch_swings_forward",
                    left_latch_open[0][1] < left_latch_rest[0][1] - 0.020,
                    f"expected latch to move outward toward negative y, got rest min y={left_latch_rest[0][1]:.4f}, open min y={left_latch_open[0][1]:.4f}",
                )
                ctx.check(
                    "left_latch_swings_down",
                    left_latch_open[0][2] < left_latch_rest[0][2] - 0.001,
                    f"expected latch to swing downward, got rest min z={left_latch_rest[0][2]:.4f}, open min z={left_latch_open[0][2]:.4f}",
                )

    right_latch_rest = ctx.part_world_aabb(right_latch)
    ctx.check("right_latch_rest_exists", right_latch_rest is not None, "right latch AABB missing")
    if right_latch_rest is not None:
        right_limits = right_latch_pivot.motion_limits
        assert right_limits is not None and right_limits.upper is not None
        with ctx.pose({right_latch_pivot: right_limits.upper}):
            right_latch_open = ctx.part_world_aabb(right_latch)
            ctx.check("right_latch_open_exists", right_latch_open is not None, "right latch open AABB missing")
            if right_latch_open is not None:
                ctx.check(
                    "right_latch_swings_forward",
                    right_latch_open[0][1] < right_latch_rest[0][1] - 0.020,
                    f"expected latch to move outward toward negative y, got rest min y={right_latch_rest[0][1]:.4f}, open min y={right_latch_open[0][1]:.4f}",
                )
                ctx.check(
                    "right_latch_swings_down",
                    right_latch_open[0][2] < right_latch_rest[0][2] - 0.001,
                    f"expected latch to swing downward, got rest min z={right_latch_rest[0][2]:.4f}, open min z={right_latch_open[0][2]:.4f}",
                )

    for joint in (lid_hinge, left_latch_pivot, right_latch_pivot):
        limits = joint.motion_limits
        assert limits is not None and limits.lower is not None and limits.upper is not None
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    with ctx.pose(
        {
            lid_hinge: lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else 0.0,
            left_latch_pivot: left_latch_pivot.motion_limits.upper if left_latch_pivot.motion_limits is not None else 0.0,
            right_latch_pivot: right_latch_pivot.motion_limits.upper if right_latch_pivot.motion_limits is not None else 0.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_open_no_overlap")
        ctx.fail_if_isolated_parts(name="all_open_no_floating")

    ctx.fail_if_articulation_overlaps(max_pose_samples=48, name="articulation_sweep_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
