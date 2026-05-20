from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _z_tube(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    *,
    center_xy: tuple[float, float] = (0.0, 0.0),
) -> cq.Workplane:
    x, y = center_xy
    length = z1 - z0
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    if inner_radius > 0.0:
        inner = cq.Workplane("XY").circle(inner_radius).extrude(length)
        outer = outer.cut(inner)
    return outer.translate((x, y, z0))


def _x_tube(
    outer_radius: float,
    inner_radius: float,
    x0: float,
    x1: float,
    *,
    center_yz: tuple[float, float] = (0.0, 0.0),
) -> cq.Workplane:
    y, z = center_yz
    length = x1 - x0
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    if inner_radius > 0.0:
        inner = cq.Workplane("YZ").circle(inner_radius).extrude(length)
        outer = outer.cut(inner)
    return outer.translate((x0, y, z))


def _y_tube(
    outer_radius: float,
    inner_radius: float,
    y0: float,
    y1: float,
    *,
    center_xz: tuple[float, float] = (0.0, 0.0),
) -> cq.Workplane:
    x, z = center_xz
    length = y1 - y0
    outer = cq.Workplane("XZ").circle(outer_radius).extrude(length)
    if inner_radius > 0.0:
        inner = cq.Workplane("XZ").circle(inner_radius).extrude(length)
        outer = outer.cut(inner)
    return outer.translate((x, y0, z))


def _make_base() -> cq.Workplane:
    foot = _box((0.128, 0.106, 0.022), (0.0, 0.0, -0.031))
    mounting_plinth = _box((0.090, 0.074, 0.012), (0.0, 0.0, -0.014))
    stator_ring = _z_tube(0.062, 0.048, -0.010, 0.004)
    thrust_pad = _z_tube(0.018, 0.0, 0.000, 0.006)
    lower_skirt = _z_tube(0.044, 0.028, -0.016, -0.010)
    return foot.union(mounting_plinth).union(stator_ring).union(thrust_pad).union(lower_skirt).clean()


def _make_yaw_collar() -> cq.Workplane:
    pitch_axis_x = 0.025
    pitch_axis_z = 0.090

    base_hub = _z_tube(0.018, 0.0, 0.006, 0.018)
    drum = _z_tube(0.040, 0.0, 0.018, 0.038)
    left_tower = _box((0.014, 0.016, 0.064), (0.025, 0.052, 0.086))
    right_tower = _box((0.014, 0.016, 0.064), (0.025, -0.052, 0.086))
    left_rib = _box((0.014, 0.010, 0.032), (0.014, 0.040, 0.052))
    right_rib = _box((0.014, 0.010, 0.032), (0.014, -0.040, 0.052))
    left_boss = _y_tube(0.022, 0.0138, 0.044, 0.060, center_xz=(pitch_axis_x, pitch_axis_z))
    right_boss = _y_tube(0.022, 0.0138, -0.060, -0.044, center_xz=(pitch_axis_x, pitch_axis_z))
    left_bore = _y_tube(0.0144, 0.0, 0.044, 0.060, center_xz=(pitch_axis_x, pitch_axis_z))
    right_bore = _y_tube(0.0144, 0.0, -0.060, -0.044, center_xz=(pitch_axis_x, pitch_axis_z))
    return (
        base_hub.union(drum)
        .union(left_tower)
        .union(right_tower)
        .union(left_rib)
        .union(right_rib)
        .union(left_boss)
        .union(right_boss)
        .cut(left_bore)
        .cut(right_bore)
        .clean()
    )


def _make_pitch_yoke() -> cq.Workplane:
    left_trunnion = _y_tube(0.0136, 0.0, 0.044, 0.060)
    right_trunnion = _y_tube(0.0136, 0.0, -0.060, -0.044)
    left_cap = _y_tube(0.018, 0.0, 0.060, 0.068)
    right_cap = _y_tube(0.018, 0.0, -0.068, -0.060)

    left_stem = _box((0.020, 0.008, 0.008), (0.040, 0.072, 0.020))
    left_stem_lower = _box((0.020, 0.008, 0.008), (0.040, 0.072, -0.020))
    right_stem = _box((0.020, 0.008, 0.008), (0.040, -0.072, 0.020))
    right_stem_lower = _box((0.020, 0.008, 0.008), (0.040, -0.072, -0.020))

    left_front_leg = _box((0.010, 0.008, 0.036), (0.055, 0.072, 0.0))
    right_front_leg = _box((0.010, 0.008, 0.036), (0.055, -0.072, 0.0))

    top_cross = _box((0.020, 0.108, 0.010), (0.075, 0.0, 0.022))
    bottom_cross = _box((0.020, 0.108, 0.010), (0.075, 0.0, -0.022))
    cartridge = _x_tube(0.024, 0.0185, 0.070, 0.082)

    return (
        left_trunnion.union(right_trunnion)
        .union(left_cap)
        .union(right_cap)
        .union(left_stem)
        .union(left_stem_lower)
        .union(right_stem)
        .union(right_stem_lower)
        .union(left_front_leg)
        .union(right_front_leg)
        .union(top_cross)
        .union(bottom_cross)
        .union(cartridge)
        .clean()
    )


def _make_roll_spindle() -> cq.Workplane:
    journal = _x_tube(0.0185, 0.0, 0.000, 0.012)
    shoulder = _x_tube(0.022, 0.0, 0.012, 0.018)
    nose_shaft = _x_tube(0.015, 0.0, 0.018, 0.078)
    flange = _x_tube(0.032, 0.0, 0.078, 0.090)

    spindle = journal.union(shoulder).union(nose_shaft).union(flange)

    center_bore = _x_tube(0.008, 0.0, 0.072, 0.096)
    bolt_holes = (
        _x_tube(0.0032, 0.0, 0.076, 0.094, center_yz=(0.020, 0.0))
        .union(_x_tube(0.0032, 0.0, 0.076, 0.094, center_yz=(-0.020, 0.0)))
        .union(_x_tube(0.0032, 0.0, 0.076, 0.094, center_yz=(0.0, 0.020)))
        .union(_x_tube(0.0032, 0.0, 0.076, 0.094, center_yz=(0.0, -0.020)))
    )

    return spindle.cut(center_bore).cut(bolt_holes).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tool_wrist")

    base_mtl = model.material("base_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    yaw_mtl = model.material("yaw_graphite", rgba=(0.30, 0.32, 0.35, 1.0))
    pitch_mtl = model.material("pitch_blackened", rgba=(0.16, 0.17, 0.19, 1.0))
    spindle_mtl = model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base(), "base_shell"), material=base_mtl, name="base_shell")

    yaw_collar = model.part("yaw_collar")
    yaw_collar.visual(
        mesh_from_cadquery(_make_yaw_collar(), "yaw_collar_shell"),
        material=yaw_mtl,
        name="yaw_collar_shell",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(_make_pitch_yoke(), "pitch_yoke_shell"),
        material=pitch_mtl,
        name="pitch_yoke_shell",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(_make_roll_spindle(), "roll_spindle_shell"),
        material=spindle_mtl,
        name="roll_spindle_shell",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.2, lower=-2.35, upper=2.35),
    )

    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_collar,
        child=pitch_yoke,
        origin=Origin(xyz=(0.025, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.0, lower=-0.75, upper=1.10),
    )

    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_spindle,
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw_collar = object_model.get_part("yaw_collar")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_spindle = object_model.get_part("roll_spindle")

    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

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
        yaw_collar,
        pitch_yoke,
        reason="pitch trunnion journals are intentionally nested inside yaw support bearing bores",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("yaw axis is vertical", tuple(yaw.axis) == (0.0, 0.0, 1.0), f"axis={yaw.axis}")
    ctx.check("pitch axis is cross-wise", tuple(pitch.axis) == (0.0, 1.0, 0.0), f"axis={pitch.axis}")
    ctx.check("roll axis is tool-forward", tuple(roll.axis) == (1.0, 0.0, 0.0), f"axis={roll.axis}")

    ctx.check(
        "yaw travel is collar-like",
        yaw.motion_limits is not None and yaw.motion_limits.lower <= -2.3 and yaw.motion_limits.upper >= 2.3,
        f"limits={yaw.motion_limits}",
    )
    ctx.check(
        "pitch travel spans below and above neutral",
        pitch.motion_limits is not None and pitch.motion_limits.lower < 0.0 < pitch.motion_limits.upper,
        f"limits={pitch.motion_limits}",
    )
    ctx.check(
        "roll spindle has near full turn travel",
        roll.motion_limits is not None and roll.motion_limits.lower <= -3.0 and roll.motion_limits.upper >= 3.0,
        f"limits={roll.motion_limits}",
    )

    ctx.expect_contact(yaw_collar, base, name="yaw collar thrust face seats on base pedestal")
    ctx.expect_contact(pitch_yoke, yaw_collar, name="pitch trunnions seat in yaw support towers")
    ctx.expect_contact(roll_spindle, pitch_yoke, name="roll spindle shoulder seats against pitch cartridge")

    with ctx.pose({pitch: pitch.motion_limits.lower, roll: 0.8, yaw: -1.1}):
        ctx.expect_contact(
            pitch_yoke,
            yaw_collar,
            name="pitch trunnion support stays seated at lower pitch limit",
        )
        ctx.expect_contact(
            roll_spindle,
            pitch_yoke,
            name="roll spindle cartridge stays seated at lower pitch limit",
        )

    with ctx.pose({pitch: pitch.motion_limits.upper, roll: -1.2, yaw: 1.1}):
        ctx.expect_contact(
            pitch_yoke,
            yaw_collar,
            name="pitch trunnion support stays seated at upper pitch limit",
        )
        ctx.expect_contact(
            roll_spindle,
            pitch_yoke,
            name="roll spindle cartridge stays seated at upper pitch limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
