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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_FOOT_X = 0.145
BASE_FOOT_Y = 0.110
BASE_THICKNESS = 0.016

COLUMN_X = 0.052
COLUMN_Y = 0.040
COLUMN_HEIGHT = 0.255
SHOULDER_RISE = 0.050
SHOULDER_Z = BASE_THICKNESS + COLUMN_HEIGHT + SHOULDER_RISE

PIN_BORE_RADIUS = 0.0085

SHOULDER_EAR_THICK = 0.012
SHOULDER_GAP = 0.022
SHOULDER_TOTAL_WIDTH = 2.0 * SHOULDER_EAR_THICK + SHOULDER_GAP
SHOULDER_BOSS_RADIUS = 0.022
SHOULDER_CHEEK_LENGTH = 0.020
SHOULDER_CHEEK_HEIGHT = 0.052
SHOULDER_X = 0.066
UPPER_PROX_LUG_THICK = SHOULDER_GAP
UPPER_PROX_LUG_RADIUS = 0.020

UPPER_LINK_LENGTH = 0.220
UPPER_BODY_WIDTH = 0.018

ELBOW_EAR_THICK = 0.010
ELBOW_GAP = 0.020
ELBOW_TOTAL_WIDTH = 2.0 * ELBOW_EAR_THICK + ELBOW_GAP
ELBOW_BOSS_RADIUS = 0.018
FORELINK_PROX_LUG_THICK = ELBOW_GAP
FORELINK_PROX_LUG_RADIUS = 0.018

FORELINK_LENGTH = 0.180
FORELINK_BODY_WIDTH = 0.016

WRIST_EAR_THICK = 0.009
WRIST_GAP = 0.018
WRIST_TOTAL_WIDTH = 2.0 * WRIST_EAR_THICK + WRIST_GAP
WRIST_BOSS_RADIUS = 0.018
TOOL_LUG_THICK = WRIST_GAP
TOOL_LUG_RADIUS = 0.016

TOOL_BRACKET_REACH = 0.080


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _add_y_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_cantilever_arm")

    model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("arm_aluminum", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("tool_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    _add_box(
        base,
        name="base_column",
        size=(BASE_FOOT_X, BASE_FOOT_Y, BASE_THICKNESS),
        center=(0.0, 0.0, BASE_THICKNESS / 2.0),
        material="graphite",
    )
    _add_box(
        base,
        name="column_body",
        size=(COLUMN_X, COLUMN_Y, COLUMN_HEIGHT),
        center=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0),
        material="graphite",
    )
    for side in (-1.0, 1.0):
        y_pos = side * (SHOULDER_GAP / 2.0 + SHOULDER_EAR_THICK / 2.0)
        _add_box(
            base,
            name=f"shoulder_brace_{'left' if side > 0 else 'right'}",
            size=(0.060, SHOULDER_EAR_THICK, 0.052),
            center=(0.030, y_pos, SHOULDER_Z - 0.026),
            material="graphite",
        )
        _add_box(
            base,
            name=f"shoulder_ear_{'left' if side > 0 else 'right'}",
            size=(SHOULDER_CHEEK_LENGTH, SHOULDER_EAR_THICK, SHOULDER_CHEEK_HEIGHT),
            center=(SHOULDER_X - 0.010, y_pos, SHOULDER_Z),
            material="graphite",
        )
        _add_y_cylinder(
            base,
            name=f"shoulder_boss_{'left' if side > 0 else 'right'}",
            radius=SHOULDER_BOSS_RADIUS,
            length=SHOULDER_EAR_THICK,
            center=(SHOULDER_X, y_pos, SHOULDER_Z),
            material="graphite",
        )

    upper_link = model.part("upper_link")
    _add_y_cylinder(
        upper_link,
        name="upper_link_lug",
        radius=UPPER_PROX_LUG_RADIUS,
        length=UPPER_PROX_LUG_THICK,
        center=(0.0, 0.0, 0.0),
        material="arm_aluminum",
    )
    _add_box(
        upper_link,
        name="upper_link_body",
        size=(0.040, 0.012, 0.014),
        center=(0.038, 0.0, -0.004),
        material="arm_aluminum",
    )
    _add_box(
        upper_link,
        name="upper_link_beam",
        size=(0.116, UPPER_BODY_WIDTH, 0.022),
        center=(0.116, 0.0, -0.004),
        material="arm_aluminum",
    )
    _add_box(
        upper_link,
        name="elbow_root",
        size=(0.022, ELBOW_TOTAL_WIDTH, 0.016),
        center=(0.184, 0.0, -0.006),
        material="arm_aluminum",
    )
    for side in (-1.0, 1.0):
        y_pos = side * (ELBOW_GAP / 2.0 + ELBOW_EAR_THICK / 2.0)
        rail_y = side * (ELBOW_GAP / 2.0 + 0.005)
        _add_box(
            upper_link,
            name=f"elbow_rail_{'left' if side > 0 else 'right'}",
            size=(0.026, 0.010, 0.014),
            center=(0.200, rail_y, -0.004),
            material="arm_aluminum",
        )
        _add_box(
            upper_link,
            name=f"elbow_ear_{'left' if side > 0 else 'right'}",
            size=(0.012, ELBOW_EAR_THICK, 0.028),
            center=(UPPER_LINK_LENGTH - 0.006, y_pos, 0.0),
            material="arm_aluminum",
        )
        _add_y_cylinder(
            upper_link,
            name=f"elbow_boss_{'left' if side > 0 else 'right'}",
            radius=ELBOW_BOSS_RADIUS,
            length=ELBOW_EAR_THICK,
            center=(UPPER_LINK_LENGTH, y_pos, 0.0),
            material="arm_aluminum",
        )

    forelink = model.part("forelink")
    _add_y_cylinder(
        forelink,
        name="forelink_lug",
        radius=FORELINK_PROX_LUG_RADIUS,
        length=FORELINK_PROX_LUG_THICK,
        center=(0.0, 0.0, 0.0),
        material="arm_aluminum",
    )
    _add_box(
        forelink,
        name="forelink_body",
        size=(0.036, 0.010, 0.012),
        center=(0.035, 0.0, -0.004),
        material="arm_aluminum",
    )
    _add_box(
        forelink,
        name="forelink_beam",
        size=(0.090, FORELINK_BODY_WIDTH, 0.020),
        center=(0.098, 0.0, -0.004),
        material="arm_aluminum",
    )
    _add_box(
        forelink,
        name="wrist_root",
        size=(0.024, WRIST_TOTAL_WIDTH, 0.015),
        center=(0.146, 0.0, -0.006),
        material="arm_aluminum",
    )
    for side in (-1.0, 1.0):
        y_pos = side * (WRIST_GAP / 2.0 + WRIST_EAR_THICK / 2.0)
        rail_y = side * (WRIST_GAP / 2.0 + 0.0045)
        _add_box(
            forelink,
            name=f"wrist_rail_{'left' if side > 0 else 'right'}",
            size=(0.018, 0.009, 0.013),
            center=(0.165, rail_y, -0.004),
            material="arm_aluminum",
        )
        _add_box(
            forelink,
            name=f"wrist_ear_{'left' if side > 0 else 'right'}",
            size=(0.012, WRIST_EAR_THICK, 0.024),
            center=(FORELINK_LENGTH - 0.006, y_pos, 0.0),
            material="arm_aluminum",
        )
        _add_y_cylinder(
            forelink,
            name=f"wrist_boss_{'left' if side > 0 else 'right'}",
            radius=WRIST_BOSS_RADIUS,
            length=WRIST_EAR_THICK,
            center=(FORELINK_LENGTH, y_pos, 0.0),
            material="arm_aluminum",
        )

    tool_bracket = model.part("tool_bracket")
    _add_y_cylinder(
        tool_bracket,
        name="tool_lug",
        radius=TOOL_LUG_RADIUS,
        length=TOOL_LUG_THICK,
        center=(0.0, 0.0, 0.0),
        material="tool_black",
    )
    _add_box(
        tool_bracket,
        name="tool_stem",
        size=(0.028, 0.010, 0.012),
        center=(0.030, 0.0, -0.004),
        material="tool_black",
    )
    _add_box(
        tool_bracket,
        name="tool_arm",
        size=(0.026, 0.014, 0.012),
        center=(0.054, 0.0, -0.004),
        material="tool_black",
    )
    _add_box(
        tool_bracket,
        name="tool_web",
        size=(0.012, 0.016, 0.040),
        center=(0.068, 0.0, -0.020),
        material="tool_black",
    )
    _add_box(
        tool_bracket,
        name="tool_bracket_body",
        size=(0.010, 0.050, 0.060),
        center=(TOOL_BRACKET_REACH - 0.005, 0.0, -0.012),
        material="tool_black",
    )
    _add_box(
        tool_bracket,
        name="tool_flange",
        size=(0.024, 0.036, 0.008),
        center=(0.060, 0.0, -0.040),
        material="tool_black",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.15, effort=18.0, velocity=1.4),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.55, effort=12.0, velocity=1.8),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=tool_bracket,
        origin=Origin(xyz=(FORELINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.20, upper=1.20, effort=6.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_link = object_model.get_part("upper_link")
    forelink = object_model.get_part("forelink")
    tool_bracket = object_model.get_part("tool_bracket")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

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
        "all three joints pitch about local -Y",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist.axis == (0.0, -1.0, 0.0),
        f"axes={shoulder.axis}, {elbow.axis}, {wrist.axis}",
    )
    ctx.check(
        "motion limits span service positions",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and wrist.motion_limits is not None
        and shoulder.motion_limits.lower < 0.0 < shoulder.motion_limits.upper
        and elbow.motion_limits.lower < 0.0 < elbow.motion_limits.upper
        and wrist.motion_limits.lower < 0.0 < wrist.motion_limits.upper,
        "expected each revolute joint to swing both above and below neutral",
    )

    ctx.expect_origin_gap(
        upper_link,
        base,
        axis="z",
        min_gap=SHOULDER_Z - 0.002,
        max_gap=SHOULDER_Z + 0.002,
        name="shoulder pivot sits at the top of the column",
    )
    ctx.expect_origin_gap(
        forelink,
        upper_link,
        axis="x",
        min_gap=UPPER_LINK_LENGTH - 0.002,
        max_gap=UPPER_LINK_LENGTH + 0.002,
        name="elbow pivot is at the end of the upper link",
    )
    ctx.expect_origin_gap(
        tool_bracket,
        forelink,
        axis="x",
        min_gap=FORELINK_LENGTH - 0.002,
        max_gap=FORELINK_LENGTH + 0.002,
        name="wrist pivot is at the end of the forelink",
    )

    def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    neutral_bracket_z = _center_z(ctx.part_element_world_aabb(tool_bracket, elem="tool_bracket_body"))
    with ctx.pose({shoulder: 0.70}):
        shoulder_up_z = _center_z(ctx.part_element_world_aabb(tool_bracket, elem="tool_bracket_body"))
    with ctx.pose({elbow: 0.95}):
        elbow_up_z = _center_z(ctx.part_element_world_aabb(tool_bracket, elem="tool_bracket_body"))
    with ctx.pose({wrist: 0.70}):
        wrist_up_z = _center_z(ctx.part_element_world_aabb(tool_bracket, elem="tool_bracket_body"))

    ctx.check(
        "positive shoulder lifts the arm",
        neutral_bracket_z is not None and shoulder_up_z is not None and shoulder_up_z > neutral_bracket_z + 0.08,
        f"neutral_z={neutral_bracket_z}, shoulder_up_z={shoulder_up_z}",
    )
    ctx.check(
        "positive elbow lifts the forelink and bracket",
        neutral_bracket_z is not None and elbow_up_z is not None and elbow_up_z > neutral_bracket_z + 0.06,
        f"neutral_z={neutral_bracket_z}, elbow_up_z={elbow_up_z}",
    )
    ctx.check(
        "positive wrist pitches the tool bracket upward",
        neutral_bracket_z is not None and wrist_up_z is not None and wrist_up_z > neutral_bracket_z + 0.015,
        f"neutral_z={neutral_bracket_z}, wrist_up_z={wrist_up_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
