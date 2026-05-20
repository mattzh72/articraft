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


BLOCK_RADIUS = 0.014
BOSS_RADIUS = 0.0095
PIN_RADIUS = 0.004
BAR_WIDTH = 0.016

BRACKET_THICKNESS = 0.008
LINK_THICKNESS = 0.005
BOSS_HEIGHT = 0.003

BRACKET_Z0 = -BRACKET_THICKNESS / 2.0
LINK1_Z0 = (BRACKET_THICKNESS / 2.0) + BOSS_HEIGHT
LINK2_Z0 = LINK1_Z0 + LINK_THICKNESS + BOSS_HEIGHT
LINK3_Z0 = LINK2_Z0 + LINK_THICKNESS + BOSS_HEIGHT

LINK1_END = (0.096, 0.018)
LINK2_END = (0.090, -0.017)
LINK3_TIP = (0.082, 0.012)


def _union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _disk(x: float, y: float, radius: float, thickness: float, z0: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z0))
    )


def _centered_rect(
    center: tuple[float, float],
    *,
    length: float,
    width: float,
    angle_deg: float,
    thickness: float,
    z0: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=(center[0], center[1], 0.0), rotate=(0.0, 0.0, angle_deg))
        .rect(length, width)
        .extrude(thickness)
        .translate((0.0, 0.0, z0))
    )


def _segment_rect(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    width: float,
    thickness: float,
    z0: float,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return _centered_rect(
        ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0),
        length=math.hypot(dx, dy),
        width=width,
        angle_deg=math.degrees(math.atan2(dy, dx)),
        thickness=thickness,
        z0=z0,
    )


def _hole_cutters(
    centers: list[tuple[float, float]],
    *,
    radius: float,
    zmin: float,
    zmax: float,
) -> cq.Workplane:
    return _union_all(
        [
            _disk(x, y, radius, zmax - zmin, zmin)
            for x, y in centers
        ]
    )


def _slot_cutter(
    center: tuple[float, float],
    *,
    length: float,
    diameter: float,
    angle_deg: float,
    zmin: float,
    zmax: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=(center[0], center[1], 0.0), rotate=(0.0, 0.0, angle_deg))
        .slot2D(length, diameter)
        .extrude(zmax - zmin)
        .translate((0.0, 0.0, zmin))
    )


def _make_bracket() -> cq.Workplane:
    plate = _centered_rect(
        (-0.022, 0.0),
        length=0.052,
        width=0.032,
        angle_deg=0.0,
        thickness=BRACKET_THICKNESS,
        z0=BRACKET_Z0,
    )
    foot = _centered_rect(
        (-0.031, -0.018),
        length=0.028,
        width=0.018,
        angle_deg=0.0,
        thickness=BRACKET_THICKNESS,
        z0=BRACKET_Z0,
    )
    pivot_block = _disk(0.0, 0.0, BLOCK_RADIUS + 0.002, BRACKET_THICKNESS, BRACKET_Z0)
    pivot_boss = _disk(0.0, 0.0, BOSS_RADIUS, BOSS_HEIGHT, BRACKET_Z0 + BRACKET_THICKNESS)

    bracket = _union_all([plate, foot, pivot_block, pivot_boss])
    bracket = bracket.cut(
        _hole_cutters(
            [(-0.034, -0.010), (-0.034, 0.010), (0.0, 0.0)],
            radius=PIN_RADIUS,
            zmin=BRACKET_Z0 - 0.001,
            zmax=BRACKET_Z0 + BRACKET_THICKNESS + BOSS_HEIGHT + 0.001,
        )
    )
    return bracket


def _make_link(distal: tuple[float, float], *, z0: float) -> cq.Workplane:
    link = _union_all(
        [
            _segment_rect((0.0, 0.0), distal, width=BAR_WIDTH, thickness=LINK_THICKNESS, z0=z0),
            _disk(0.0, 0.0, BLOCK_RADIUS, LINK_THICKNESS, z0),
            _disk(distal[0], distal[1], BLOCK_RADIUS, LINK_THICKNESS, z0),
            _disk(distal[0], distal[1], BOSS_RADIUS, BOSS_HEIGHT, z0 + LINK_THICKNESS),
        ]
    )
    link = link.cut(
        _hole_cutters(
            [(0.0, 0.0), distal],
            radius=PIN_RADIUS,
            zmin=z0 - 0.001,
            zmax=z0 + LINK_THICKNESS + BOSS_HEIGHT + 0.001,
        )
    )
    return link


def _make_terminal_link(tip: tuple[float, float], *, z0: float) -> cq.Workplane:
    tip_dx = tip[0]
    tip_dy = tip[1]
    tip_length = math.hypot(tip_dx, tip_dy)
    ux = tip_dx / tip_length
    uy = tip_dy / tip_length
    tip_angle = math.degrees(math.atan2(tip_dy, tip_dx))

    tab_center = (tip[0] - (0.007 * ux), tip[1] - (0.007 * uy))
    nose_center = (tip[0] + (0.010 * ux), tip[1] + (0.010 * uy))
    slot_center = (tip[0] - (0.004 * ux), tip[1] - (0.004 * uy))

    terminal = _union_all(
        [
            _segment_rect((0.0, 0.0), tip, width=BAR_WIDTH, thickness=LINK_THICKNESS, z0=z0),
            _disk(0.0, 0.0, BLOCK_RADIUS, LINK_THICKNESS, z0),
            _centered_rect(
                tab_center,
                length=0.034,
                width=0.020,
                angle_deg=tip_angle,
                thickness=LINK_THICKNESS,
                z0=z0,
            ),
            _disk(nose_center[0], nose_center[1], 0.010, LINK_THICKNESS, z0),
        ]
    )
    terminal = terminal.cut(
        _hole_cutters(
            [(0.0, 0.0)],
            radius=PIN_RADIUS,
            zmin=z0 - 0.001,
            zmax=z0 + LINK_THICKNESS + 0.001,
        )
    )
    terminal = terminal.cut(
        _slot_cutter(
            slot_center,
            length=0.019,
            diameter=0.0065,
            angle_deg=tip_angle,
            zmin=z0 - 0.001,
            zmax=z0 + LINK_THICKNESS + 0.001,
        )
    )
    return terminal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_over_center_lever_train")

    model.material("bracket_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("lever_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("lever_dark_steel", rgba=(0.48, 0.52, 0.58, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        mesh_from_cadquery(_make_bracket(), "bracket_body"),
        origin=Origin(),
        material="bracket_black",
        name="bracket_body",
    )

    lever1 = model.part("lever1")
    lever1.visual(
        mesh_from_cadquery(_make_link(LINK1_END, z0=LINK1_Z0), "lever1_body"),
        origin=Origin(),
        material="lever_steel",
        name="lever1_body",
    )

    lever2 = model.part("lever2")
    lever2.visual(
        mesh_from_cadquery(_make_link(LINK2_END, z0=LINK2_Z0), "lever2_body"),
        origin=Origin(),
        material="lever_dark_steel",
        name="lever2_body",
    )

    lever3 = model.part("lever3")
    lever3.visual(
        mesh_from_cadquery(_make_terminal_link(LINK3_TIP, z0=LINK3_Z0), "lever3_body"),
        origin=Origin(),
        material="lever_steel",
        name="lever3_body",
    )

    model.articulation(
        "bracket_to_lever1",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=lever1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.85, upper=1.05),
    )
    model.articulation(
        "lever1_to_lever2",
        ArticulationType.REVOLUTE,
        parent=lever1,
        child=lever2,
        origin=Origin(xyz=(LINK1_END[0], LINK1_END[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.15, upper=0.95),
    )
    model.articulation(
        "lever2_to_lever3",
        ArticulationType.REVOLUTE,
        parent=lever2,
        child=lever3,
        origin=Origin(xyz=(LINK2_END[0], LINK2_END[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.95, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    lever1 = object_model.get_part("lever1")
    lever2 = object_model.get_part("lever2")
    lever3 = object_model.get_part("lever3")

    joint1 = object_model.get_articulation("bracket_to_lever1")
    joint2 = object_model.get_articulation("lever1_to_lever2")
    joint3 = object_model.get_articulation("lever2_to_lever3")

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

    for part_name in ("bracket", "lever1", "lever2", "lever3"):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    for joint in (joint1, joint2, joint3):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_is_z",
            all(abs(a - b) < 1e-9 for a, b in zip(joint.axis, (0.0, 0.0, 1.0))),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{joint.name}_crosses_center",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"limits={limits}",
        )

    ctx.expect_contact(bracket, lever1, contact_tol=0.001, name="bracket_to_lever1_contact")
    ctx.expect_contact(lever1, lever2, contact_tol=0.001, name="lever1_to_lever2_contact")
    ctx.expect_contact(lever2, lever3, contact_tol=0.001, name="lever2_to_lever3_contact")

    ctx.expect_overlap(bracket, lever1, axes="xy", min_overlap=0.010, name="root_joint_overlap")
    ctx.expect_overlap(lever1, lever2, axes="xy", min_overlap=0.010, name="middle_joint_overlap")
    ctx.expect_overlap(lever2, lever3, axes="xy", min_overlap=0.010, name="end_joint_overlap")

    with ctx.pose(
        {
            joint1: 0.35,
            joint2: -0.55,
            joint3: 0.45,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_representative_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
