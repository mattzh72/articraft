from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BED_X = 0.38
BED_Y = 0.27
BED_T = 0.014
BED_TOP = BED_T

ARM_YAW = -0.60
ARM_COS = math.cos(ARM_YAW)
ARM_SIN = math.sin(ARM_YAW)
HINGE_AXIS = (ARM_SIN, -ARM_COS, 0.0)
HINGE_AXIS_YAW = math.atan2(HINGE_AXIS[1], HINGE_AXIS[0])
HINGE_ORIGIN = (-0.150, 0.094, BED_TOP + 0.013)


def _arm_offset(x_local: float, y_local: float, z_local: float) -> tuple[float, float, float]:
    return (
        x_local * ARM_COS - y_local * ARM_SIN,
        x_local * ARM_SIN + y_local * ARM_COS,
        z_local,
    )


def _arm_point(x_local: float, y_local: float, z_world: float) -> tuple[float, float, float]:
    dx, dy, _ = _arm_offset(x_local, y_local, 0.0)
    return (
        HINGE_ORIGIN[0] + dx,
        HINGE_ORIGIN[1] + dy,
        z_world,
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_guillotine_paper_cutter", assets=ASSETS)

    board = model.material("board", rgba=(0.88, 0.86, 0.78, 1.0))
    fence_gray = model.material("fence_gray", rgba=(0.52, 0.53, 0.55, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    arm_red = model.material("arm_red", rgba=(0.73, 0.16, 0.14, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    cutting_base = model.part("cutting_base")
    cutting_base.visual(
        Box((BED_X, BED_Y, BED_T)),
        origin=Origin(xyz=(0.0, 0.0, BED_T * 0.5)),
        material=board,
        name="bed_panel",
    )
    cutting_base.visual(
        Box((BED_X, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, BED_Y * 0.5 - 0.006, BED_TOP + 0.011)),
        material=fence_gray,
        name="rear_fence",
    )
    cutting_base.visual(
        Box((0.012, 0.170, 0.012)),
        origin=Origin(xyz=(-BED_X * 0.5 + 0.006, 0.015, BED_TOP + 0.006)),
        material=fence_gray,
        name="side_guide",
    )
    cutting_base.visual(
        Box((0.060, 0.050, 0.014)),
        origin=Origin(
            xyz=_arm_point(-0.050, 0.000, BED_TOP + 0.007),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=dark_gray,
        name="hinge_block",
    )
    cutting_base.visual(
        Box((0.024, 0.036, 0.016)),
        origin=Origin(
            xyz=_arm_point(-0.078, 0.000, BED_TOP + 0.015),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=dark_gray,
        name="hinge_cap",
    )
    cutting_base.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(
            xyz=_arm_point(-0.012, 0.016, HINGE_ORIGIN[2]),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=dark_gray,
        name="hinge_cheek_left",
    )
    cutting_base.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(
            xyz=_arm_point(-0.012, -0.016, HINGE_ORIGIN[2]),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=dark_gray,
        name="hinge_cheek_right",
    )
    cutting_base.visual(
        Cylinder(radius=0.005, length=0.048),
        origin=Origin(
            xyz=HINGE_ORIGIN,
            rpy=(0.0, math.pi * 0.5, HINGE_AXIS_YAW),
        ),
        material=blade_steel,
        name="hinge_pin",
    )
    cutting_base.visual(
        Box((0.315, 0.004, 0.001)),
        origin=Origin(
            xyz=_arm_point(0.160, 0.000, BED_TOP + 0.0005),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=dark_gray,
        name="cutting_strip",
    )
    cutting_base.inertial = Inertial.from_geometry(
        Box((BED_X, BED_Y, 0.050)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    cutter_arm = model.part("cutter_arm")
    cutter_arm.visual(
        Cylinder(radius=0.0075, length=0.016),
        origin=Origin(
            rpy=(0.0, math.pi * 0.5, HINGE_AXIS_YAW),
        ),
        material=dark_gray,
        name="hinge_barrel",
    )
    cutter_arm.visual(
        Box((0.090, 0.012, 0.008)),
        origin=Origin(
            xyz=_arm_offset(0.036, 0.000, 0.008),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=arm_red,
        name="heel_plate",
    )
    cutter_arm.visual(
        Box((0.205, 0.022, 0.022)),
        origin=Origin(
            xyz=_arm_offset(0.145, 0.004, -0.001),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=arm_red,
        name="arm_body",
    )
    cutter_arm.visual(
        Box((0.080, 0.026, 0.018)),
        origin=Origin(
            xyz=_arm_offset(0.280, 0.006, 0.000),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=arm_red,
        name="nose_body",
    )
    cutter_arm.visual(
        Box((0.305, 0.004, 0.002)),
        origin=Origin(
            xyz=_arm_offset(0.160, 0.000, -0.011),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=blade_steel,
        name="blade_strip",
    )
    cutter_arm.visual(
        Box((0.055, 0.018, 0.040)),
        origin=Origin(
            xyz=_arm_offset(0.230, 0.010, 0.020),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=arm_red,
        name="handle_riser",
    )
    cutter_arm.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(
            xyz=_arm_offset(0.260, 0.010, 0.032),
            rpy=(0.0, math.pi * 0.5, ARM_YAW),
        ),
        material=grip_black,
        name="handle_grip",
    )
    cutter_arm.inertial = Inertial.from_geometry(
        Box((0.390, 0.090, 0.060)),
        mass=0.55,
        origin=Origin(xyz=_arm_offset(0.175, 0.006, 0.004)),
    )

    model.articulation(
        "base_to_cutter_arm",
        ArticulationType.REVOLUTE,
        parent=cutting_base,
        child=cutter_arm,
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cutting_base = object_model.get_part("cutting_base")
    cutter_arm = object_model.get_part("cutter_arm")
    hinge = object_model.get_articulation("base_to_cutter_arm")

    bed_panel = cutting_base.get_visual("bed_panel")
    hinge_pin = cutting_base.get_visual("hinge_pin")
    cutting_strip = cutting_base.get_visual("cutting_strip")
    rear_fence = cutting_base.get_visual("rear_fence")

    hinge_barrel = cutter_arm.get_visual("hinge_barrel")
    arm_body = cutter_arm.get_visual("arm_body")
    blade_strip = cutter_arm.get_visual("blade_strip")
    handle_grip = cutter_arm.get_visual("handle_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        cutting_base,
        cutter_arm,
        elem_a=hinge_pin,
        elem_b=hinge_barrel,
        reason="The hinge pin passes through the arm's hinge barrel at the rear corner.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        cutting_base,
        cutter_arm,
        elem_a=hinge_pin,
        elem_b=hinge_barrel,
        name="hinge_barrel_is_seated_on_corner_pin",
    )
    ctx.expect_gap(
        cutter_arm,
        cutting_base,
        axis="z",
        positive_elem=blade_strip,
        negative_elem=cutting_strip,
        max_gap=0.001,
        max_penetration=1e-6,
        name="blade_strip_sits_flush_to_cutting_strip_when_closed",
    )
    ctx.expect_overlap(
        cutter_arm,
        cutting_base,
        axes="xy",
        elem_a=blade_strip,
        elem_b=cutting_strip,
        min_overlap=0.12,
        name="blade_and_bed_cut_line_share_same_diagonal_footprint",
    )
    ctx.expect_within(
        cutter_arm,
        cutting_base,
        axes="xy",
        inner_elem=blade_strip,
        outer_elem=bed_panel,
        margin=0.0,
        name="blade_stays_within_bed_footprint",
    )
    ctx.expect_overlap(
        cutter_arm,
        cutting_base,
        axes="xy",
        elem_a=arm_body,
        elem_b=bed_panel,
        min_overlap=0.10,
        name="arm_reads_as_spanning_over_the_bed",
    )

    bed_aabb = ctx.part_element_world_aabb(cutting_base, elem=bed_panel)
    blade_aabb = ctx.part_element_world_aabb(cutter_arm, elem=blade_strip)
    handle_rest_aabb = ctx.part_element_world_aabb(cutter_arm, elem=handle_grip)
    fence_aabb = ctx.part_element_world_aabb(cutting_base, elem=rear_fence)

    assert bed_aabb is not None
    assert blade_aabb is not None
    assert handle_rest_aabb is not None
    assert fence_aabb is not None

    blade_dx = blade_aabb[1][0] - blade_aabb[0][0]
    blade_dy = blade_aabb[1][1] - blade_aabb[0][1]
    ctx.check(
        "blade_runs_diagonally_across_the_bed",
        blade_dx > BED_X * 0.62 and blade_dy > BED_Y * 0.55,
        details=f"blade projected extents were dx={blade_dx:.3f}, dy={blade_dy:.3f}",
    )

    handle_rest_center = _aabb_center(handle_rest_aabb)
    handle_rest_radius = math.dist(handle_rest_center, HINGE_ORIGIN)
    ctx.check(
        "hinge_sits_below_rear_fence_line",
        HINGE_ORIGIN[1] < fence_aabb[0][1],
        details=f"hinge y={HINGE_ORIGIN[1]:.3f} should sit in front of fence min y={fence_aabb[0][1]:.3f}",
    )

    with ctx.pose({hinge: math.radians(40.0)}):
        ctx.expect_gap(
            cutter_arm,
            cutting_base,
            axis="z",
            positive_elem=blade_strip,
            negative_elem=cutting_strip,
            min_gap=0.006,
            name="blade_clears_the_bed_when_partially_open",
        )
        ctx.expect_contact(
            cutting_base,
            cutter_arm,
            elem_a=hinge_pin,
            elem_b=hinge_barrel,
            name="hinge_stays_connected_at_mid_travel",
        )

    with ctx.pose({hinge: math.radians(75.0)}):
        handle_open_aabb = ctx.part_element_world_aabb(cutter_arm, elem=handle_grip)
        assert handle_open_aabb is not None
        handle_open_center = _aabb_center(handle_open_aabb)
        handle_open_radius = math.dist(handle_open_center, HINGE_ORIGIN)
        ctx.expect_gap(
            cutter_arm,
            cutting_base,
            axis="z",
            positive_elem=blade_strip,
            negative_elem=cutting_strip,
            min_gap=0.015,
            name="blade_lifts_clear_when_opened_wide",
        )
        ctx.expect_contact(
            cutting_base,
            cutter_arm,
            elem_a=hinge_pin,
            elem_b=hinge_barrel,
            name="hinge_stays_connected_at_wide_open_pose",
        )
        ctx.check(
            "handle_rises_when_arm_opens",
            handle_open_center[2] > handle_rest_center[2] + 0.030,
            details=(
                f"handle z moved from {handle_rest_center[2]:.3f} "
                f"to {handle_open_center[2]:.3f}"
            ),
        )
        ctx.check(
            "arm_rotates_about_a_diagonal_hinge_line",
            abs(handle_open_radius - handle_rest_radius) < 0.01,
            details=(
                f"handle hinge radius changed from {handle_rest_radius:.3f} "
                f"to {handle_open_radius:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
