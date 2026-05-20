from __future__ import annotations

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


HALF_PI = math.pi / 2.0


def _y_cylinder(x: float, y: float, z: float, radius: float, length: float) -> tuple[Cylinder, Origin]:
    """Cylinder primitive and transform for a pin/boss running along the local Y axis."""
    return Cylinder(radius=radius, length=length), Origin(xyz=(x, y, z), rpy=(-HALF_PI, 0.0, 0.0))


def _add_revolute_link(
    part,
    *,
    length: float,
    prox_length: float,
    paint: str,
    steel: str,
    name_prefix: str,
) -> None:
    """Deep box-section link with a central proximal tongue and forked distal boss."""
    boss_radius = 0.075
    body_start = 0.065
    body_end = length - 0.130
    body_len = body_end - body_start

    geom, origin = _y_cylinder(0.0, 0.0, 0.0, boss_radius, prox_length)
    part.visual(geom, origin=origin, material=paint, name=f"{name_prefix}_prox_boss")

    cap_y = prox_length * 0.5 - 0.005
    for side, y in (("upper", cap_y), ("lower", -cap_y)):
        geom, origin = _y_cylinder(0.0, y, 0.0, 0.050, 0.010)
        part.visual(geom, origin=origin, material=steel, name=f"{name_prefix}_prox_cap_{side}")

    part.visual(
        Box((body_len, 0.070, 0.120)),
        origin=Origin(xyz=((body_start + body_end) * 0.5, 0.0, 0.0)),
        material=paint,
        name=f"{name_prefix}_box_web",
    )
    part.visual(
        Box((max(body_len - 0.060, 0.10), 0.094, 0.018)),
        origin=Origin(xyz=((body_start + body_end) * 0.5 + 0.020, 0.0, 0.068)),
        material=steel,
        name=f"{name_prefix}_top_flange",
    )
    part.visual(
        Box((max(body_len - 0.060, 0.10), 0.094, 0.018)),
        origin=Origin(xyz=((body_start + body_end) * 0.5 + 0.020, 0.0, -0.068)),
        material=steel,
        name=f"{name_prefix}_bottom_flange",
    )

    part.visual(
        Box((0.090, 0.165, 0.082)),
        origin=Origin(xyz=(length - 0.165, 0.0, 0.0)),
        material=paint,
        name=f"{name_prefix}_fork_bridge",
    )
    for side, y in (("upper", 0.064), ("lower", -0.064)):
        part.visual(
            Box((0.205, 0.037, 0.108)),
            origin=Origin(xyz=(length - 0.075, y, 0.0)),
            material=paint,
            name=f"{name_prefix}_fork_cheek_{side}",
        )
        geom, origin = _y_cylinder(length, y, 0.0, boss_radius, 0.037)
        part.visual(geom, origin=origin, material=paint, name=f"{name_prefix}_dist_boss_{side}")
        geom, origin = _y_cylinder(length, 0.088 if y > 0 else -0.088, 0.0, 0.048, 0.011)
        part.visual(geom, origin=origin, material=steel, name=f"{name_prefix}_dist_cap_{side}")


def _add_terminal_link(part, *, length: float, paint: str, steel: str, dark: str) -> None:
    """Final short link with the fourth joint tongue and a compact end tab."""
    geom, origin = _y_cylinder(0.0, 0.0, 0.0, 0.075, 0.091)
    part.visual(geom, origin=origin, material=paint, name="terminal_prox_boss")
    for side, y in (("upper", 0.0405), ("lower", -0.0405)):
        geom, origin = _y_cylinder(0.0, y, 0.0, 0.050, 0.010)
        part.visual(geom, origin=origin, material=steel, name=f"terminal_prox_cap_{side}")

    part.visual(
        Box((length - 0.095, 0.070, 0.105)),
        origin=Origin(xyz=((length - 0.095) * 0.5 + 0.060, 0.0, 0.0)),
        material=paint,
        name="terminal_short_web",
    )
    part.visual(
        Box((0.115, 0.128, 0.072)),
        origin=Origin(xyz=(length, 0.0, 0.0)),
        material=paint,
        name="terminal_end_tab",
    )
    geom, origin = _y_cylinder(length, 0.069, 0.0, 0.032, 0.010)
    part.visual(geom, origin=origin, material=dark, name="terminal_tab_hole_mark")
    geom, origin = _y_cylinder(length, -0.069, 0.0, 0.032, 0.010)
    part.visual(geom, origin=origin, material=dark, name="terminal_tab_hole_mark_1")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_four_joint_revolute_chain")
    base_mat = model.material("graphite_cast_base", rgba=(0.18, 0.20, 0.21, 1.0))
    link_orange = model.material("safety_orange_paint", rgba=(0.95, 0.34, 0.08, 1.0))
    link_blue = model.material("industrial_blue_paint", rgba=(0.05, 0.18, 0.38, 1.0))
    steel = model.material("brushed_pin_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    dark = model.material("dark_bore_shadow", rgba=(0.025, 0.028, 0.030, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.42, 0.075)),
        origin=Origin(xyz=(0.18, 0.0, -0.340)),
        material=base_mat,
        name="base_plate",
    )
    for side, y in (("upper", 0.105), ("lower", -0.105)):
        base.visual(
            Box((0.185, 0.045, 0.355)),
            origin=Origin(xyz=(0.0, y, -0.155)),
            material=base_mat,
            name=f"base_cheek_{side}",
        )
        geom, origin = _y_cylinder(0.0, y, 0.0, 0.098, 0.045)
        base.visual(geom, origin=origin, material=base_mat, name=f"base_pin_boss_{side}")
        geom, origin = _y_cylinder(0.0, 0.1345 if y > 0 else -0.1345, 0.0, 0.056, 0.014)
        base.visual(geom, origin=origin, material=steel, name=f"base_outer_pin_cap_{side}")
        base.visual(
            Box((0.300, 0.035, 0.100)),
            origin=Origin(xyz=(0.110, y, -0.300)),
            material=base_mat,
            name=f"base_foot_rib_{side}",
        )
        base.visual(
            Box((0.038, 0.080, 0.250)),
            origin=Origin(xyz=(-0.105, y, -0.205)),
            material=base_mat,
            name=f"base_vertical_rib_{side}",
        )

    base.visual(
        Box((0.155, 0.150, 0.080)),
        origin=Origin(xyz=(0.030, 0.0, -0.292)),
        material=base_mat,
        name="base_cross_tie",
    )
    for i, (x, y) in enumerate(((-0.130, -0.155), (-0.130, 0.155), (0.430, -0.155), (0.430, 0.155))):
        base.visual(
            Cylinder(radius=0.022, length=0.015),
            origin=Origin(xyz=(x, y, -0.295)),
            material=steel,
            name=f"base_anchor_bolt_{i}",
        )

    drive_link = model.part("drive_link")
    middle_link = model.part("middle_link")
    wrist_link = model.part("wrist_link")
    end_tab = model.part("end_tab")

    l0 = 0.580
    l1 = 0.500
    l2 = 0.420
    l3 = 0.300

    _add_revolute_link(drive_link, length=l0, prox_length=0.165, paint=link_orange, steel=steel, name_prefix="drive")
    _add_revolute_link(middle_link, length=l1, prox_length=0.091, paint=link_blue, steel=steel, name_prefix="middle")
    _add_revolute_link(wrist_link, length=l2, prox_length=0.091, paint=link_orange, steel=steel, name_prefix="wrist")
    _add_terminal_link(end_tab, length=l3, paint=link_blue, steel=steel, dark=dark)

    revolute_limits = MotionLimits(effort=250.0, velocity=1.2, lower=-1.15, upper=1.25)
    model.articulation(
        "base_to_drive",
        ArticulationType.REVOLUTE,
        parent=base,
        child=drive_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.9, lower=-0.75, upper=1.10),
    )
    model.articulation(
        "drive_to_middle",
        ArticulationType.REVOLUTE,
        parent=drive_link,
        child=middle_link,
        origin=Origin(xyz=(l0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "middle_to_wrist",
        ArticulationType.REVOLUTE,
        parent=middle_link,
        child=wrist_link,
        origin=Origin(xyz=(l1, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "wrist_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=wrist_link,
        child=end_tab,
        origin=Origin(xyz=(l2, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-1.30, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("base_to_drive"),
        object_model.get_articulation("drive_to_middle"),
        object_model.get_articulation("middle_to_wrist"),
        object_model.get_articulation("wrist_to_end_tab"),
    ]
    ctx.check(
        "serial chain has four revolute joints",
        len(joints) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "all hinge axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    base = object_model.get_part("base")
    drive_link = object_model.get_part("drive_link")
    middle_link = object_model.get_part("middle_link")
    wrist_link = object_model.get_part("wrist_link")
    end_tab = object_model.get_part("end_tab")

    ctx.expect_overlap(
        drive_link,
        base,
        axes="xz",
        min_overlap=0.050,
        name="first boss is supported by base cheeks",
    )

    for parent, child, check_name in (
        (drive_link, middle_link, "second joint has interleaved bosses"),
        (middle_link, wrist_link, "third joint has interleaved bosses"),
        (wrist_link, end_tab, "fourth joint has interleaved bosses"),
    ):
        ctx.expect_overlap(parent, child, axes="xz", min_overlap=0.050, name=check_name)

    rest_pos = ctx.part_world_position(end_tab)
    with ctx.pose({"base_to_drive": 0.35, "drive_to_middle": -0.30, "middle_to_wrist": 0.25, "wrist_to_end_tab": -0.20}):
        posed_pos = ctx.part_world_position(end_tab)
    ctx.check(
        "terminal tab follows the articulated chain",
        rest_pos is not None
        and posed_pos is not None
        and abs(posed_pos[0] - rest_pos[0]) > 0.020
        and abs(posed_pos[2] - rest_pos[2]) > 0.020,
        details=f"rest={rest_pos}, posed={posed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
