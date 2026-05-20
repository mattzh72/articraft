from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
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
    mesh_from_cadquery,
)


BASE_FOOTPRINT = 0.30
BASE_COLUMN = 0.22
BASE_HEIGHT = 0.445

UPPER_WIDTH = 0.12
ELBOW_GAP = 0.068
ELBOW_CENTER = (0.295, 0.0, 0.145)

FOREARM_WIDTH = 0.09
WRIST_GAP = 0.052
WRIST_CENTER = (0.198, 0.0, 0.004)


def _extruded_profile(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(width / 2.0, both=True)


def _make_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_FOOTPRINT, BASE_FOOTPRINT, 0.05, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )
    column = (
        cq.Workplane("XY")
        .box(BASE_COLUMN, BASE_COLUMN, 0.36, centered=(True, True, False))
        .translate((0.0, 0.0, 0.05))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.26, 0.26, BASE_HEIGHT - 0.41, centered=(True, True, False))
        .translate((0.0, 0.0, 0.41))
    )
    service_recess = cq.Workplane("XY").box(0.14, 0.02, 0.16).translate((0.0, 0.121, 0.18))
    return foot.union(column).union(cap).cut(service_recess)


def _make_upper_link_shape() -> cq.Workplane:
    shoulder_hub = cq.Workplane("XY").circle(0.078).extrude(0.038)
    shoulder_block = (
        cq.Workplane("XY")
        .box(0.14, 0.16, 0.10, centered=(True, True, False))
        .translate((0.02, 0.0, 0.018))
    )
    beam = cq.Workplane("XY").box(0.225, 0.12, 0.072).translate((0.1325, 0.0, 0.121))
    cover = cq.Workplane("XY").box(0.15, 0.10, 0.05).translate((0.145, 0.0, 0.165))

    elbow_cheek_y = ELBOW_GAP / 2.0 + 0.013
    elbow_cheek = cq.Workplane("XY").box(0.075, 0.026, 0.095).translate((0.2775, elbow_cheek_y, 0.1475))
    opposite_elbow_cheek = cq.Workplane("XY").box(0.075, 0.026, 0.095).translate((0.2775, -elbow_cheek_y, 0.1475))

    service_relief = cq.Workplane("XY").box(0.13, 0.07, 0.045).translate((0.10, 0.0, 0.078))
    return (
        shoulder_hub.union(shoulder_block)
        .union(beam)
        .union(cover)
        .union(elbow_cheek)
        .union(opposite_elbow_cheek)
        .cut(service_relief)
    )


def _make_forearm_shape() -> cq.Workplane:
    elbow_barrel = cq.Workplane("XZ").circle(0.033).extrude(ELBOW_GAP / 2.0, both=True)
    spine = cq.Workplane("XY").box(0.134, 0.07, 0.066).translate((0.097, 0.0, 0.005))
    cover = cq.Workplane("XY").box(0.09, 0.056, 0.038).translate((0.107, 0.0, 0.039))

    wrist_cheek_y = WRIST_GAP / 2.0 + 0.0095
    wrist_cheek = cq.Workplane("XY").box(0.056, 0.019, 0.082).translate((0.186, wrist_cheek_y, 0.004))
    opposite_wrist_cheek = cq.Workplane("XY").box(0.056, 0.019, 0.082).translate((0.186, -wrist_cheek_y, 0.004))

    underside_relief = cq.Workplane("XY").box(0.08, 0.04, 0.028).translate((0.115, 0.0, -0.005))
    return (
        elbow_barrel.union(spine)
        .union(cover)
        .union(wrist_cheek)
        .union(opposite_wrist_cheek)
        .cut(underside_relief)
    )


def _make_wrist_shape() -> cq.Workplane:
    wrist_barrel = cq.Workplane("XZ").circle(0.026).extrude(WRIST_GAP / 2.0, both=True)
    wrist_body = cq.Workplane("XY").box(0.088, 0.07, 0.064).translate((0.066, 0.0, 0.002))
    top_cover = cq.Workplane("XY").box(0.05, 0.05, 0.022).translate((0.072, 0.0, 0.045))
    tool_face = cq.Workplane("XY").box(0.032, 0.082, 0.09).translate((0.126, 0.0, 0.002))
    return wrist_barrel.union(wrist_body).union(top_cover).union(tool_face)


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_manipulator")

    dark_base = model.material("dark_base", color=(0.18, 0.19, 0.21, 1.0))
    arm_gray = model.material("arm_gray", color=(0.73, 0.75, 0.78, 1.0))
    wrist_gray = model.material("wrist_gray", color=(0.58, 0.61, 0.65, 1.0))

    base_column = model.part("base_column")
    base_column.visual(
        mesh_from_cadquery(_make_base_shape(), "base_column_shell"),
        origin=Origin(),
        material=dark_base,
        name="base_column_shell",
    )
    base_column.inertial = Inertial.from_geometry(
        Box((BASE_FOOTPRINT, BASE_FOOTPRINT, BASE_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.078, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=arm_gray,
        name="shoulder_hub",
    )
    upper_link.visual(
        Box((0.14, 0.16, 0.10)),
        origin=Origin(xyz=(0.025, 0.0, 0.05)),
        material=arm_gray,
        name="shoulder_block",
    )
    upper_link.visual(
        Box((0.20, 0.11, 0.07)),
        origin=Origin(xyz=(0.14, 0.0, 0.125)),
        material=arm_gray,
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.12, 0.09, 0.04)),
        origin=Origin(xyz=(0.17, 0.0, 0.175)),
        material=arm_gray,
        name="upper_cover",
    )
    upper_link.visual(
        Box((0.062, 0.018, 0.09)),
        origin=Origin(xyz=(0.286, 0.043, 0.145)),
        material=arm_gray,
        name="elbow_cheek_left",
    )
    upper_link.visual(
        Box((0.015, 0.018, 0.08)),
        origin=Origin(xyz=(0.2475, 0.043, 0.145)),
        material=arm_gray,
        name="elbow_strut_left",
    )
    upper_link.visual(
        Box((0.062, 0.018, 0.09)),
        origin=Origin(xyz=(0.286, -0.043, 0.145)),
        material=arm_gray,
        name="elbow_cheek_right",
    )
    upper_link.visual(
        Box((0.015, 0.018, 0.08)),
        origin=Origin(xyz=(0.2475, -0.043, 0.145)),
        material=arm_gray,
        name="elbow_strut_right",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.36, 0.16, 0.22)),
        mass=8.0,
        origin=Origin(xyz=(0.14, 0.0, 0.11)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.033, length=ELBOW_GAP),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_gray,
        name="elbow_barrel",
    )
    forearm.visual(
        Box((0.135, 0.07, 0.06)),
        origin=Origin(xyz=(0.1005, 0.0, 0.0)),
        material=arm_gray,
        name="forearm_spine",
    )
    forearm.visual(
        Box((0.09, 0.055, 0.034)),
        origin=Origin(xyz=(0.11, 0.0, 0.038)),
        material=arm_gray,
        name="forearm_cover",
    )
    forearm.visual(
        Box((0.05, 0.018, 0.082)),
        origin=Origin(xyz=(0.185, 0.035, 0.0)),
        material=arm_gray,
        name="wrist_cheek_left",
    )
    forearm.visual(
        Box((0.05, 0.018, 0.082)),
        origin=Origin(xyz=(0.185, -0.035, 0.0)),
        material=arm_gray,
        name="wrist_cheek_right",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.23, 0.11, 0.12)),
        mass=4.5,
        origin=Origin(xyz=(0.11, 0.0, 0.01)),
    )

    wrist_block = model.part("wrist_block")
    wrist_block.visual(
        Cylinder(radius=0.026, length=WRIST_GAP),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wrist_gray,
        name="wrist_barrel",
    )
    wrist_block.visual(
        Box((0.074, 0.07, 0.06)),
        origin=Origin(xyz=(0.063, 0.0, 0.0)),
        material=wrist_gray,
        name="wrist_body",
    )
    wrist_block.visual(
        Box((0.044, 0.05, 0.02)),
        origin=Origin(xyz=(0.074, 0.0, 0.04)),
        material=wrist_gray,
        name="wrist_top_cap",
    )
    wrist_block.visual(
        Box((0.032, 0.082, 0.09)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material=wrist_gray,
        name="tool_face",
    )
    wrist_block.inertial = Inertial.from_geometry(
        Box((0.14, 0.09, 0.10)),
        mass=2.0,
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
    )

    model.articulation(
        "base_shoulder",
        ArticulationType.REVOLUTE,
        parent=base_column,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.8, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=ELBOW_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=2.0, lower=-1.1, upper=1.25),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_block,
        origin=Origin(xyz=WRIST_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.5, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_column = object_model.get_part("base_column")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    wrist_block = object_model.get_part("wrist_block")

    base_shoulder = object_model.get_articulation("base_shoulder")
    upper_to_forearm = object_model.get_articulation("upper_to_forearm")
    forearm_to_wrist = object_model.get_articulation("forearm_to_wrist")

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

    ctx.check("base_column_present", base_column is not None, "base column missing")
    ctx.check("upper_link_present", upper_link is not None, "upper link missing")
    ctx.check("forearm_present", forearm is not None, "forearm missing")
    ctx.check("wrist_block_present", wrist_block is not None, "wrist block missing")

    ctx.expect_contact(upper_link, base_column, name="shoulder_turntable_seated")
    ctx.expect_contact(forearm, upper_link, name="elbow_hinge_connected")
    ctx.expect_contact(wrist_block, forearm, name="wrist_hinge_connected")

    ctx.expect_origin_gap(
        upper_link,
        base_column,
        axis="z",
        min_gap=BASE_HEIGHT - 0.001,
        max_gap=BASE_HEIGHT + 0.001,
        name="upper_link_mount_height",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_link,
        axis="x",
        min_gap=0.26,
        max_gap=0.33,
        name="compact_elbow_reach",
    )
    ctx.expect_origin_gap(
        wrist_block,
        forearm,
        axis="x",
        min_gap=0.17,
        max_gap=0.22,
        name="short_forearm_length",
    )

    ctx.check(
        "base_shoulder_axis_vertical",
        _axis_matches(base_shoulder.axis, (0.0, 0.0, 1.0)),
        f"expected vertical axis, got {base_shoulder.axis}",
    )
    ctx.check(
        "elbow_axis_horizontal",
        _axis_matches(upper_to_forearm.axis, (0.0, 1.0, 0.0)),
        f"expected horizontal elbow axis, got {upper_to_forearm.axis}",
    )
    ctx.check(
        "wrist_axis_horizontal",
        _axis_matches(forearm_to_wrist.axis, (0.0, 1.0, 0.0)),
        f"expected wrist hinge axis, got {forearm_to_wrist.axis}",
    )

    with ctx.pose(upper_to_forearm=0.9, forearm_to_wrist=-0.75):
        ctx.expect_origin_gap(
            wrist_block,
            base_column,
            axis="z",
            min_gap=0.10,
            name="lifted_arm_clears_base_when_flexed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
