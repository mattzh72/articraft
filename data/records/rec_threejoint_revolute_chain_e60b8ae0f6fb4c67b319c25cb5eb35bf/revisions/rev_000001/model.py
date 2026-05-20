from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_chain")

    painted_steel = Material("painted_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    blue_link = Material("blue_link", rgba=(0.02, 0.20, 0.68, 1.0))
    orange_link = Material("orange_link", rgba=(0.93, 0.39, 0.05, 1.0))
    green_link = Material("green_link", rgba=(0.10, 0.46, 0.24, 1.0))

    model.material("painted_steel", rgba=painted_steel.rgba)
    model.material("dark_steel", rgba=dark_steel.rgba)
    model.material("blue_link", rgba=blue_link.rgba)
    model.material("orange_link", rgba=orange_link.rgba)
    model.material("green_link", rgba=green_link.rgba)

    y_cyl = Origin(rpy=(pi / 2.0, 0.0, 0.0))

    base = model.part("base")
    base.visual(
        Box((0.66, 0.30, 0.045)),
        origin=Origin(xyz=(0.05, 0.0, 0.0225)),
        material=painted_steel,
        name="base_foot",
    )
    base.visual(
        Box((0.17, 0.18, 0.075)),
        origin=Origin(xyz=(-0.20, 0.0, 0.0825)),
        material=painted_steel,
        name="stepped_plinth",
    )
    base.visual(
        Box((0.10, 0.13, 0.055)),
        origin=Origin(xyz=(-0.20, 0.0, 0.1475)),
        material=painted_steel,
        name="center_post",
    )
    for y, label, boss in (
        (-0.066, "fork_cheek_0", "base_boss_0"),
        (0.066, "fork_cheek_1", "base_boss_1"),
    ):
        base.visual(
            Box((0.095, 0.032, 0.225)),
            origin=Origin(xyz=(-0.20, y, 0.2325)),
            material=painted_steel,
            name=label,
        )
        base.visual(
            Cylinder(radius=0.074, length=0.032),
            origin=Origin(xyz=(-0.20, y, 0.245), rpy=y_cyl.rpy),
            material=dark_steel,
            name=boss,
        )
    base.visual(
        Box((0.135, 0.170, 0.032)),
        origin=Origin(xyz=(-0.20, 0.0, 0.136)),
        material=painted_steel,
        name="fork_bridge",
    )

    link_0 = model.part("root_link")
    link_0.visual(
        Cylinder(radius=0.061, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_cyl.rpy),
        material=blue_link,
        name="root_hub",
    )
    link_0.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_cyl.rpy),
        material=dark_steel,
        name="axle_pin",
    )
    link_0.visual(
        Box((0.175, 0.038, 0.070)),
        origin=Origin(xyz=(0.095, 0.0, 0.014)),
        material=blue_link,
        name="wide_step",
    )
    link_0.visual(
        Box((0.070, 0.038, 0.118)),
        origin=Origin(xyz=(0.190, 0.0, 0.047)),
        material=blue_link,
        name="raised_step_web",
    )
    link_0.visual(
        Box((0.105, 0.038, 0.055)),
        origin=Origin(xyz=(0.235, 0.0, 0.080)),
        material=blue_link,
        name="narrow_step",
    )
    link_0.visual(
        Box((0.036, 0.128, 0.072)),
        origin=Origin(xyz=(0.287, 0.0, 0.080)),
        material=blue_link,
        name="yoke_bridge",
    )
    for y, label, boss in (
        (-0.055, "distal_cheek_0", "distal_boss_0"),
        (0.055, "distal_cheek_1", "distal_boss_1"),
    ):
        link_0.visual(
            Box((0.095, 0.028, 0.055)),
            origin=Origin(xyz=(0.323, y, 0.080)),
            material=blue_link,
            name=label,
        )
        link_0.visual(
            Cylinder(radius=0.052, length=0.028),
            origin=Origin(xyz=(0.360, y, 0.080), rpy=y_cyl.rpy),
            material=blue_link,
            name=boss,
        )

    link_1 = model.part("middle_link")
    link_1.visual(
        Cylinder(radius=0.046, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_cyl.rpy),
        material=orange_link,
        name="root_hub",
    )
    link_1.visual(
        Cylinder(radius=0.012, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_cyl.rpy),
        material=dark_steel,
        name="axle_pin",
    )
    link_1.visual(
        Box((0.112, 0.036, 0.064)),
        origin=Origin(xyz=(0.066, 0.0, -0.012)),
        material=orange_link,
        name="root_step",
    )
    link_1.visual(
        Box((0.060, 0.036, 0.104)),
        origin=Origin(xyz=(0.140, 0.0, -0.039)),
        material=orange_link,
        name="drop_step_web",
    )
    link_1.visual(
        Box((0.120, 0.030, 0.046)),
        origin=Origin(xyz=(0.200, 0.0, -0.067)),
        material=orange_link,
        name="slim_offset_step",
    )
    link_1.visual(
        Box((0.034, 0.112, 0.060)),
        origin=Origin(xyz=(0.238, 0.0, -0.067)),
        material=orange_link,
        name="yoke_bridge",
    )
    for y, label, boss in (
        (-0.048, "tip_cheek_0", "tip_boss_0"),
        (0.048, "tip_cheek_1", "tip_boss_1"),
    ):
        link_1.visual(
            Box((0.082, 0.026, 0.046)),
            origin=Origin(xyz=(0.282, y, -0.067)),
            material=orange_link,
            name=label,
        )
        link_1.visual(
            Cylinder(radius=0.043, length=0.026),
            origin=Origin(xyz=(0.305, y, -0.067), rpy=y_cyl.rpy),
            material=orange_link,
            name=boss,
        )

    link_2 = model.part("tip_link")
    link_2.visual(
        Cylinder(radius=0.040, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_cyl.rpy),
        material=green_link,
        name="root_hub",
    )
    link_2.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_cyl.rpy),
        material=dark_steel,
        name="axle_pin",
    )
    link_2.visual(
        Box((0.084, 0.034, 0.050)),
        origin=Origin(xyz=(0.052, 0.0, 0.010)),
        material=green_link,
        name="short_step",
    )
    link_2.visual(
        Box((0.054, 0.034, 0.112)),
        origin=Origin(xyz=(0.112, 0.0, 0.048)),
        material=green_link,
        name="upturned_web",
    )
    link_2.visual(
        Box((0.134, 0.028, 0.042)),
        origin=Origin(xyz=(0.180, 0.0, 0.102)),
        material=green_link,
        name="narrow_tip_step",
    )
    link_2.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(0.246, 0.0, 0.102), rpy=y_cyl.rpy),
        material=green_link,
        name="round_tip",
    )

    model.articulation(
        "base_to_root",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(-0.20, 0.0, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-0.65, upper=0.75),
    )
    model.articulation(
        "root_to_middle",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.360, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.4, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "middle_to_tip",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.305, 0.0, -0.067)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.8, lower=-0.85, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("base_to_root"),
        object_model.get_articulation("root_to_middle"),
        object_model.get_articulation("middle_to_tip"),
    ]
    ctx.check(
        "three revolute serial joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and [j.parent for j in joints] == ["base", "root_link", "middle_link"]
        and [j.child for j in joints] == ["root_link", "middle_link", "tip_link"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in joints]}",
    )
    ctx.check(
        "all axes share one motion plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    base = object_model.get_part("base")
    root_link = object_model.get_part("root_link")
    middle_link = object_model.get_part("middle_link")
    tip_link = object_model.get_part("tip_link")

    ctx.expect_contact(
        root_link,
        base,
        elem_a="axle_pin",
        elem_b="base_boss_0",
        contact_tol=0.001,
        name="root pin seats in base fork",
    )
    ctx.expect_contact(
        middle_link,
        root_link,
        elem_a="axle_pin",
        elem_b="distal_boss_0",
        contact_tol=0.001,
        name="middle pin seats in root yoke",
    )
    ctx.expect_contact(
        tip_link,
        middle_link,
        elem_a="axle_pin",
        elem_b="tip_boss_0",
        contact_tol=0.001,
        name="tip pin seats in middle yoke",
    )

    rest_tip = ctx.part_world_position(tip_link)
    with ctx.pose({"base_to_root": 0.45, "root_to_middle": -0.55, "middle_to_tip": 0.70}):
        posed_tip = ctx.part_world_position(tip_link)

    ctx.check(
        "serial pose moves the tip link",
        rest_tip is not None
        and posed_tip is not None
        and ((posed_tip[0] - rest_tip[0]) ** 2 + (posed_tip[2] - rest_tip[2]) ** 2) ** 0.5 > 0.060
        and abs(posed_tip[1] - rest_tip[1]) < 0.001,
        details=f"rest={rest_tip}, posed={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
