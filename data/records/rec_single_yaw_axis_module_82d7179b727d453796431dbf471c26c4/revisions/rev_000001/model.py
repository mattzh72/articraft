from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

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
    mesh_from_cadquery,
)


BOTTOM_FLANGE_D = 0.260
BOTTOM_FLANGE_T = 0.016
BASE_BODY_D = 0.220
BASE_BODY_H = 0.090
UPPER_SHOULDER_D = 0.168
UPPER_SHOULDER_H = 0.008
SUPPORT_NECK_D = 0.140
SUPPORT_NECK_H = 0.010

STAGE_JOINT_Z = BOTTOM_FLANGE_T + BASE_BODY_H + UPPER_SHOULDER_H + SUPPORT_NECK_H

SPINDLE_COLLAR_D = 0.100
SPINDLE_COLLAR_H = 0.006
SPINDLE_PILOT_D = 0.082
SPINDLE_PILOT_H = 0.016
SPINDLE_TOP_Z = STAGE_JOINT_Z + SPINDLE_COLLAR_H + SPINDLE_PILOT_H

PLATFORM_D = 0.190
PLATFORM_RING_D = 0.136
PLATFORM_RING_H = 0.006
PLATFORM_DISK_T = 0.022
PLATFORM_TOP_Z = PLATFORM_RING_H + PLATFORM_DISK_T
PLATFORM_CLEAR_BORE_D = 0.106
PLATFORM_INTERFACE_RELIEF_D = 0.124

BRACKET_BASE_LEN = 0.070
BRACKET_BASE_W = 0.040
BRACKET_BASE_T = 0.008
BRACKET_BASE_CX = 0.045
BRACKET_BASE_BOTTOM_Z = PLATFORM_TOP_Z
BRACKET_BASE_TOP_Z = BRACKET_BASE_BOTTOM_Z + BRACKET_BASE_T

BRACKET_UPRIGHT_T = 0.008
BRACKET_UPRIGHT_W = 0.064
BRACKET_UPRIGHT_H = 0.058
BRACKET_UPRIGHT_X = 0.074

BRACKET_LIP_LEN = 0.018
BRACKET_LIP_T = 0.006

BRACKET_RIB_LEN = 0.022
BRACKET_RIB_T = 0.006
BRACKET_RIB_H = 0.026
BRACKET_RIB_X = 0.061
BRACKET_RIB_Y = 0.014


def _radial_points(radius: float, count: int, start_angle: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(start_angle + (2.0 * pi * idx / count)),
            radius * sin(start_angle + (2.0 * pi * idx / count)),
        )
        for idx in range(count)
    ]


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _cylinder(diameter: float, height: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(diameter / 2.0).extrude(height).translate((0.0, 0.0, z0))


def _annulus(outer_d: float, inner_d: float, height: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_d / 2.0)
        .circle(inner_d / 2.0)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _cap_screw_pattern(
    points: list[tuple[float, float]],
    *,
    head_d: float,
    head_h: float,
    head_bottom_z: float,
    shank_d: float,
    shank_h: float,
    shank_bottom_z: float,
) -> cq.Workplane:
    heads = (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(head_d / 2.0)
        .extrude(head_h)
        .translate((0.0, 0.0, head_bottom_z))
    )
    shanks = (
        cq.Workplane("XY")
        .pushPoints(points)
        .circle(shank_d / 2.0)
        .extrude(shank_h)
        .translate((0.0, 0.0, shank_bottom_z))
    )
    return heads.union(shanks)


def _build_housing_shell() -> cq.Workplane:
    housing = _union_all(
        _cylinder(BOTTOM_FLANGE_D, BOTTOM_FLANGE_T, 0.0),
        _cylinder(BASE_BODY_D, BASE_BODY_H, BOTTOM_FLANGE_T),
        _cylinder(UPPER_SHOULDER_D, UPPER_SHOULDER_H, BOTTOM_FLANGE_T + BASE_BODY_H),
        _cylinder(SUPPORT_NECK_D, SUPPORT_NECK_H, BOTTOM_FLANGE_T + BASE_BODY_H + UPPER_SHOULDER_H),
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(_radial_points(0.100, 4, start_angle=pi / 4.0))
        .circle(0.005)
        .extrude(BOTTOM_FLANGE_T + 0.001)
    )
    housing = housing.cut(mount_holes)

    groove_width = 0.004
    groove_depth = 0.0012
    for radius in (0.050, 0.059):
        housing = housing.cut(
            _annulus(
                outer_d=(radius + groove_width / 2.0) * 2.0,
                inner_d=(radius - groove_width / 2.0) * 2.0,
                height=groove_depth,
                z0=STAGE_JOINT_Z - groove_depth,
            )
        )

    return housing


def _build_spindle_shape() -> cq.Workplane:
    return _union_all(
        _cylinder(SPINDLE_COLLAR_D, SPINDLE_COLLAR_H, STAGE_JOINT_Z),
        _cylinder(SPINDLE_PILOT_D, SPINDLE_PILOT_H, STAGE_JOINT_Z + SPINDLE_COLLAR_H),
    )


def _build_housing_screws() -> cq.Workplane:
    return _cap_screw_pattern(
        _radial_points(0.076, 6, start_angle=pi / 6.0),
        head_d=0.012,
        head_h=0.004,
        head_bottom_z=BOTTOM_FLANGE_T + BASE_BODY_H + UPPER_SHOULDER_H,
        shank_d=0.0055,
        shank_h=0.008,
        shank_bottom_z=BOTTOM_FLANGE_T + BASE_BODY_H,
    )


def _build_platform_shell() -> cq.Workplane:
    disk = _annulus(PLATFORM_D, PLATFORM_CLEAR_BORE_D, PLATFORM_DISK_T, PLATFORM_RING_H)
    support_ring = _annulus(PLATFORM_RING_D, PLATFORM_INTERFACE_RELIEF_D, PLATFORM_RING_H, 0.0)
    return disk.union(support_ring)


def _build_platform_screws() -> cq.Workplane:
    return _cap_screw_pattern(
        _radial_points(0.070, 4, start_angle=pi / 4.0),
        head_d=0.010,
        head_h=0.004,
        head_bottom_z=PLATFORM_TOP_Z,
        shank_d=0.0045,
        shank_h=0.008,
        shank_bottom_z=PLATFORM_TOP_Z - 0.008,
    )


def _build_bracket_body() -> cq.Workplane:
    base_plate = _box(
        BRACKET_BASE_LEN,
        BRACKET_BASE_W,
        BRACKET_BASE_T,
        (BRACKET_BASE_CX, 0.0, BRACKET_BASE_BOTTOM_Z + BRACKET_BASE_T / 2.0),
    )
    upright = _box(
        BRACKET_UPRIGHT_T,
        BRACKET_UPRIGHT_W,
        BRACKET_UPRIGHT_H,
        (
            BRACKET_UPRIGHT_X,
            0.0,
            BRACKET_BASE_TOP_Z + BRACKET_UPRIGHT_H / 2.0,
        ),
    )
    top_lip = _box(
        BRACKET_LIP_LEN,
        BRACKET_UPRIGHT_W,
        BRACKET_LIP_T,
        (
            BRACKET_UPRIGHT_X - (BRACKET_UPRIGHT_T + BRACKET_LIP_LEN) / 2.0,
            0.0,
            BRACKET_BASE_TOP_Z + BRACKET_UPRIGHT_H + BRACKET_LIP_T / 2.0,
        ),
    )
    rib_a = _box(
        BRACKET_RIB_LEN,
        BRACKET_RIB_T,
        BRACKET_RIB_H,
        (
            BRACKET_RIB_X,
            BRACKET_RIB_Y,
            BRACKET_BASE_TOP_Z + BRACKET_RIB_H / 2.0,
        ),
    )
    rib_b = _box(
        BRACKET_RIB_LEN,
        BRACKET_RIB_T,
        BRACKET_RIB_H,
        (
            BRACKET_RIB_X,
            -BRACKET_RIB_Y,
            BRACKET_BASE_TOP_Z + BRACKET_RIB_H / 2.0,
        ),
    )

    return _union_all(base_plate, upright, top_lip, rib_a, rib_b)


def _build_bracket_screws() -> cq.Workplane:
    base_points = [
        (BRACKET_BASE_CX - 0.020, -0.012),
        (BRACKET_BASE_CX - 0.020, 0.012),
        (BRACKET_BASE_CX + 0.020, -0.012),
        (BRACKET_BASE_CX + 0.020, 0.012),
    ]
    return _cap_screw_pattern(
        base_points,
        head_d=0.010,
        head_h=0.004,
        head_bottom_z=BRACKET_BASE_TOP_Z,
        shank_d=0.0045,
        shank_h=BRACKET_BASE_T,
        shank_bottom_z=BRACKET_BASE_BOTTOM_Z,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_stage")

    model.material("housing_aluminum", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("stage_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("anodized_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("black_oxide", rgba=(0.11, 0.12, 0.13, 1.0))

    lower_housing = model.part("lower_housing")
    lower_housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "lower_housing_shell"),
        material="housing_aluminum",
        name="housing_shell",
    )
    lower_housing.visual(
        mesh_from_cadquery(_build_spindle_shape(), "lower_housing_spindle"),
        material="stage_aluminum",
        name="spindle",
    )
    lower_housing.visual(
        mesh_from_cadquery(_build_housing_screws(), "lower_housing_screws"),
        material="black_oxide",
        name="housing_screws",
    )
    lower_housing.inertial = Inertial.from_geometry(
        Box((BOTTOM_FLANGE_D, BOTTOM_FLANGE_D, SPINDLE_TOP_Z)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_TOP_Z / 2.0)),
    )

    rotary_platform = model.part("rotary_platform")
    rotary_platform.visual(
        mesh_from_cadquery(_build_platform_shell(), "rotary_platform_shell"),
        material="stage_aluminum",
        name="platform_shell",
    )
    rotary_platform.visual(
        mesh_from_cadquery(_build_platform_screws(), "rotary_platform_screws"),
        material="black_oxide",
        name="platform_screws",
    )
    rotary_platform.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATFORM_D / 2.0, length=PLATFORM_TOP_Z),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_TOP_Z / 2.0)),
    )

    equipment_bracket = model.part("equipment_bracket")
    equipment_bracket.visual(
        mesh_from_cadquery(_build_bracket_body(), "equipment_bracket_body"),
        material="anodized_black",
        name="bracket_body",
    )
    equipment_bracket.visual(
        mesh_from_cadquery(_build_bracket_screws(), "equipment_bracket_screws"),
        material="black_oxide",
        name="bracket_screws",
    )
    equipment_bracket.inertial = Inertial.from_geometry(
        Box((0.082, 0.070, 0.072)),
        mass=0.85,
        origin=Origin(xyz=(0.050, 0.0, 0.064)),
    )

    model.articulation(
        "stage_yaw",
        ArticulationType.REVOLUTE,
        parent=lower_housing,
        child=rotary_platform,
        origin=Origin(xyz=(0.0, 0.0, STAGE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "platform_to_bracket",
        ArticulationType.FIXED,
        parent=rotary_platform,
        child=equipment_bracket,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_housing = object_model.get_part("lower_housing")
    rotary_platform = object_model.get_part("rotary_platform")
    equipment_bracket = object_model.get_part("equipment_bracket")
    stage_yaw = object_model.get_articulation("stage_yaw")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=16,
        ignore_fixed=True,
        name="stage_clearance_sweep",
    )

    ctx.check(
        "stage_yaw_axis_vertical",
        tuple(stage_yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={stage_yaw.axis}",
    )
    ctx.check(
        "stage_yaw_limits_span_realistic",
        (
            stage_yaw.motion_limits is not None
            and stage_yaw.motion_limits.lower is not None
            and stage_yaw.motion_limits.upper is not None
            and stage_yaw.motion_limits.lower <= -2.0
            and stage_yaw.motion_limits.upper >= 2.0
        ),
        details=f"limits={stage_yaw.motion_limits}",
    )

    ctx.expect_contact(
        rotary_platform,
        lower_housing,
        elem_a="platform_shell",
        elem_b="housing_shell",
        name="platform_supported_on_housing_neck",
    )
    ctx.expect_overlap(
        rotary_platform,
        lower_housing,
        axes="xy",
        elem_a="platform_shell",
        elem_b="housing_shell",
        min_overlap=0.130,
        name="platform_centered_over_lower_housing",
    )
    ctx.expect_contact(
        equipment_bracket,
        rotary_platform,
        elem_a="bracket_body",
        elem_b="platform_shell",
        name="bracket_fixed_to_platform",
    )
    ctx.expect_overlap(
        equipment_bracket,
        rotary_platform,
        axes="xy",
        elem_a="bracket_body",
        elem_b="platform_shell",
        min_overlap=0.030,
        name="bracket_base_footprint_supported_by_platform",
    )
    ctx.expect_origin_gap(
        rotary_platform,
        lower_housing,
        axis="z",
        min_gap=0.120,
        max_gap=0.128,
        name="platform_origin_sits_above_housing",
    )

    with ctx.pose(stage_yaw=1.8):
        ctx.expect_gap(
            equipment_bracket,
            lower_housing,
            axis="z",
            positive_elem="bracket_body",
            negative_elem="housing_shell",
            min_gap=0.020,
            name="bracket_clears_housing_through_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
