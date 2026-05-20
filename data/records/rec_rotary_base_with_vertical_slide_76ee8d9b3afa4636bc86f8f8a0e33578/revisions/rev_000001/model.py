from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_FOOTPRINT = 0.38
BASE_PLINTH_H = 0.022
BASE_BODY_H = 0.110
BASE_CAP_H = 0.015
BASE_BEARING_H = 0.018
BASE_JOINT_Z = BASE_PLINTH_H + BASE_BODY_H + BASE_CAP_H + BASE_BEARING_H

TURRET_SIZE = 0.296
FRAME_RING_H = 0.014
TURRET_BODY_H = 0.074
TURRET_TOP_H = 0.016
FRAME_TOP_Z = FRAME_RING_H + TURRET_BODY_H + TURRET_TOP_H

RAM_JOINT_Y = 0.060
RAM_JOINT_Z = 0.182
RAM_STROKE = 0.180


def _box(
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    fillet: float | None = None,
) -> cq.Workplane:
    solid = (
        cq.Workplane("XY")
        .box(*size, centered=(True, True, False))
        .translate(xyz)
    )
    if fillet is not None and fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid


def _ring(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _build_base_shape() -> cq.Workplane:
    base = _box((BASE_FOOTPRINT, BASE_FOOTPRINT, BASE_PLINTH_H), (0.0, 0.0, 0.0), fillet=0.008)
    base = base.union(_box((0.34, 0.34, BASE_BODY_H), (0.0, 0.0, BASE_PLINTH_H), fillet=0.010))
    base = base.union(_box((0.31, 0.31, BASE_CAP_H), (0.0, 0.0, BASE_PLINTH_H + BASE_BODY_H), fillet=0.006))
    base = base.union(_ring(0.148, 0.102, BASE_BEARING_H, BASE_PLINTH_H + BASE_BODY_H + BASE_CAP_H))

    service_relief = _box((0.16, 0.055, 0.060), (0.0, 0.142, 0.050))
    cable_gland = _box((0.050, 0.020, 0.032), (0.0, -0.181, 0.070), fillet=0.002)
    base = base.cut(service_relief).union(cable_gland)
    return base


def _build_turret_frame_main_shape() -> cq.Workplane:
    frame = _ring(0.150, 0.100, FRAME_RING_H, 0.0)
    frame = frame.union(_box((TURRET_SIZE, TURRET_SIZE, TURRET_BODY_H), (0.0, 0.0, FRAME_RING_H), fillet=0.008))
    frame = frame.union(_box((0.220, 0.180, TURRET_TOP_H), (0.0, 0.0, FRAME_RING_H + TURRET_BODY_H), fillet=0.004))

    guide_slot = _box((0.088, 0.126, 0.430), (0.0, 0.064, FRAME_RING_H))
    frame = frame.cut(guide_slot)

    cheek_size = (0.028, 0.060, 0.340)
    frame = frame.union(_box(cheek_size, (-0.060, RAM_JOINT_Y, FRAME_TOP_Z), fillet=0.003))
    frame = frame.union(_box(cheek_size, (0.060, RAM_JOINT_Y, FRAME_TOP_Z), fillet=0.003))
    frame = frame.union(_box((0.116, 0.024, 0.340), (0.0, -0.018, FRAME_TOP_Z)))
    frame = frame.union(_box((0.116, 0.024, 0.028), (0.0, -0.018, 0.414)))

    shroud_outer = _box((0.084, 0.034, 0.220), (0.0, -0.028, 0.100))
    shroud_inner = _box((0.070, 0.020, 0.208), (0.0, -0.026, 0.106))
    frame = frame.union(shroud_outer.cut(shroud_inner))

    cable_stem = _box((0.012, 0.020, 0.128), (-0.088, 0.086, 0.208))
    cable_pad_low = _box((0.022, 0.026, 0.014), (-0.074, 0.078, 0.228))
    cable_pad_high = _box((0.022, 0.026, 0.014), (-0.074, 0.078, 0.302))
    return frame.union(cable_stem).union(cable_pad_low).union(cable_pad_high)


def _build_frame_guide_pad(x_center: float, z0: float) -> cq.Workplane:
    return _box((0.010, 0.024, 0.055), (x_center, RAM_JOINT_Y, z0), fillet=0.0015)


def _build_ram_body_shape() -> cq.Workplane:
    ram = _box((0.052, 0.030, 0.490), (0.0, 0.0, -0.160), fillet=0.003)
    ram = ram.union(_box((0.084, 0.058, 0.014), (0.0, 0.000, 0.332), fillet=0.003))
    ram = ram.union(_box((0.070, 0.008, 0.092), (0.0, 0.011, 0.208)))
    ram = ram.union(_box((0.058, 0.010, 0.260), (0.0, -0.018, -0.160)))
    ram = ram.union(_box((0.014, 0.040, 0.018), (-0.032, 0.000, 0.334)))
    ram = ram.union(_box((0.014, 0.040, 0.018), (0.032, 0.000, 0.334)))
    return ram


def _build_ram_rail(x_center: float) -> cq.Workplane:
    return _box((0.008, 0.018, 0.370), (x_center, 0.0, -0.140))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="welding_positioner")

    model.material("base_paint", color=(0.16, 0.17, 0.19))
    model.material("frame_paint", color=(0.34, 0.39, 0.45))
    model.material("ram_metal", color=(0.78, 0.80, 0.83))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_shell"),
        material="base_paint",
        name="base_shell",
    )

    turret_frame = model.part("turret_frame")
    turret_frame.visual(
        mesh_from_cadquery(_build_turret_frame_main_shape(), "turret_frame_shell"),
        material="frame_paint",
        name="turret_frame_shell",
    )
    turret_frame.visual(
        mesh_from_cadquery(_build_frame_guide_pad(-0.041, 0.194), "guide_left_lower"),
        material="frame_paint",
        name="guide_left_lower",
    )
    turret_frame.visual(
        mesh_from_cadquery(_build_frame_guide_pad(0.041, 0.194), "guide_right_lower"),
        material="frame_paint",
        name="guide_right_lower",
    )
    turret_frame.visual(
        mesh_from_cadquery(_build_frame_guide_pad(-0.041, 0.312), "guide_left_upper"),
        material="frame_paint",
        name="guide_left_upper",
    )
    turret_frame.visual(
        mesh_from_cadquery(_build_frame_guide_pad(0.041, 0.312), "guide_right_upper"),
        material="frame_paint",
        name="guide_right_upper",
    )

    ram = model.part("ram")
    ram.visual(
        mesh_from_cadquery(_build_ram_body_shape(), "ram_shell"),
        material="ram_metal",
        name="ram_shell",
    )
    ram.visual(
        mesh_from_cadquery(_build_ram_rail(-0.032), "rail_left"),
        material="ram_metal",
        name="rail_left",
    )
    ram.visual(
        mesh_from_cadquery(_build_ram_rail(0.032), "rail_right"),
        material="ram_metal",
        name="rail_right",
    )

    model.articulation(
        "base_to_turret",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret_frame,
        origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2, lower=-3.14159, upper=3.14159),
    )

    model.articulation(
        "turret_to_ram",
        ArticulationType.PRISMATIC,
        parent=turret_frame,
        child=ram,
        origin=Origin(xyz=(0.0, RAM_JOINT_Y, RAM_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.18, lower=0.0, upper=RAM_STROKE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turret_frame = object_model.get_part("turret_frame")
    ram = object_model.get_part("ram")
    turret_joint = object_model.get_articulation("base_to_turret")
    ram_joint = object_model.get_articulation("turret_to_ram")

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

    turret_limits = turret_joint.motion_limits
    ram_limits = ram_joint.motion_limits

    ctx.check(
        "turret_axis_is_vertical",
        tuple(round(v, 6) for v in turret_joint.axis) == (0.0, 0.0, 1.0),
        f"turret axis was {turret_joint.axis}",
    )
    ctx.check(
        "ram_axis_is_vertical",
        tuple(round(v, 6) for v in ram_joint.axis) == (0.0, 0.0, 1.0),
        f"ram axis was {ram_joint.axis}",
    )
    ctx.check(
        "turret_limits_span_real_rotation",
        turret_limits is not None
        and turret_limits.lower is not None
        and turret_limits.upper is not None
        and turret_limits.lower < -3.0
        and turret_limits.upper > 3.0,
        f"turret limits were {turret_limits}",
    )
    ctx.check(
        "ram_limits_match_compact_stroke",
        ram_limits is not None
        and ram_limits.lower == 0.0
        and ram_limits.upper is not None
        and abs(ram_limits.upper - RAM_STROKE) < 1e-6,
        f"ram limits were {ram_limits}",
    )

    ctx.expect_contact(base, turret_frame, name="turret_bearing_contact")
    ctx.expect_overlap(base, turret_frame, axes="xy", min_overlap=0.20, name="turret_sits_on_base")
    ctx.expect_gap(ram, base, axis="z", min_gap=0.015, name="ram_clears_base")

    with ctx.pose({ram_joint: 0.0}):
        ctx.expect_contact(
            ram,
            turret_frame,
            elem_a="rail_left",
            elem_b="guide_left_lower",
            name="left_lower_guide_contact_retracted",
        )
        ctx.expect_contact(
            ram,
            turret_frame,
            elem_a="rail_right",
            elem_b="guide_right_lower",
            name="right_lower_guide_contact_retracted",
        )

    rest_pos = ctx.part_world_position(ram)
    with ctx.pose({ram_joint: RAM_STROKE}):
        ctx.expect_contact(
            ram,
            turret_frame,
            elem_a="rail_left",
            elem_b="guide_left_upper",
            name="left_upper_guide_contact_extended",
        )
        ctx.expect_contact(
            ram,
            turret_frame,
            elem_a="rail_right",
            elem_b="guide_right_upper",
            name="right_upper_guide_contact_extended",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_lift")
        extended_pos = ctx.part_world_position(ram)

    ctx.check(
        "ram_extends_upward",
        rest_pos is not None
        and extended_pos is not None
        and (extended_pos[2] - rest_pos[2]) > RAM_STROKE * 0.99,
        f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
