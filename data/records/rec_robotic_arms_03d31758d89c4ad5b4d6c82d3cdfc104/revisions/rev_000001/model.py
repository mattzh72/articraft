from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


CYL_X = (0.0, pi / 2.0, 0.0)
CYL_Y = (pi / 2.0, 0.0, 0.0)


def _add_cap_bolts(
    part,
    *,
    prefix: str,
    x: float,
    y: float,
    z: float,
    radius: float,
    material: Material,
    count: int = 6,
) -> None:
    """Small side-facing fasteners, slightly seated into the cartridge cap."""

    sign = 1.0 if y >= 0.0 else -1.0
    for i in range(count):
        a = 2.0 * pi * i / count
        part.visual(
            Cylinder(radius=0.014, length=0.024),
            origin=Origin(
                xyz=(x + radius * cos(a), y + sign * 0.003, z + radius * sin(a)),
                rpy=CYL_Y,
            ),
            material=material,
            name=f"{prefix}_bolt_{i}",
        )


def _add_top_bolts(
    part,
    *,
    prefix: str,
    positions: tuple[tuple[float, float], ...],
    z: float,
    material: Material,
) -> None:
    for i, (x, y) in enumerate(positions):
        part.visual(
            Cylinder(radius=0.026, length=0.026),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, 0.0)),
            material=material,
            name=f"{prefix}_bolt_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_industrial_robot_arm")

    safety_yellow = Material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    guarded_yellow = Material("guarded_yellow", rgba=(0.95, 0.62, 0.02, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    red = Material("lockout_red", rgba=(0.86, 0.03, 0.02, 1.0))
    zinc = Material("zinc_fasteners", rgba=(0.68, 0.70, 0.69, 1.0))
    caution = Material("caution_orange", rgba=(1.0, 0.30, 0.02, 1.0))

    # Root: heavy fixed pedestal with visible anchor logic and rear cable feed.
    base = model.part("base")
    base.visual(
        Box((1.12, 0.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.36, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=dark_steel,
        name="pedestal_foot",
    )
    base.visual(
        Cylinder(radius=0.25, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=safety_yellow,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.38, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=dark_steel,
        name="fixed_bearing_ring",
    )
    base.visual(
        Box((0.20, 0.18, 0.48)),
        origin=Origin(xyz=(-0.38, -0.37, 0.32)),
        material=black,
        name="cable_riser",
    )
    base.visual(
        Box((0.42, 0.14, 0.14)),
        origin=Origin(xyz=(-0.20, -0.35, 0.54)),
        material=black,
        name="cable_feed_elbow",
    )
    _add_top_bolts(
        base,
        prefix="anchor",
        positions=(
            (-0.47, -0.34),
            (-0.47, 0.34),
            (0.47, -0.34),
            (0.47, 0.34),
            (-0.20, -0.34),
            (-0.20, 0.34),
            (0.20, -0.34),
            (0.20, 0.34),
        ),
        z=0.093,
        material=zinc,
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.36, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="rotating_table",
    )
    turret.visual(
        Box((0.56, 0.50, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=guarded_yellow,
        name="turret_box",
    )
    turret.visual(
        Box((0.30, 0.07, 0.65)),
        origin=Origin(xyz=(0.0, 0.26, 0.48)),
        material=safety_yellow,
        name="shoulder_yoke_plate_0",
    )
    turret.visual(
        Box((0.30, 0.07, 0.65)),
        origin=Origin(xyz=(0.0, -0.26, 0.48)),
        material=safety_yellow,
        name="shoulder_yoke_plate_1",
    )
    turret.visual(
        Box((0.26, 0.60, 0.08)),
        origin=Origin(xyz=(-0.05, 0.0, 0.845)),
        material=safety_yellow,
        name="shoulder_yoke_tie",
    )
    turret.visual(
        Box((0.10, 0.60, 0.16)),
        origin=Origin(xyz=(-0.19, 0.0, 0.38)),
        material=dark_steel,
        name="rear_web_plate",
    )
    for side, y in enumerate((0.315, -0.315)):
        turret.visual(
            Cylinder(radius=0.22, length=0.034),
            origin=Origin(xyz=(0.0, y, 0.62), rpy=CYL_Y),
            material=dark_steel,
            name=f"shoulder_bearing_cap_{side}",
        )
        _add_cap_bolts(
            turret,
            prefix=f"shoulder_cap_{side}",
            x=0.0,
            y=y + (0.018 if y > 0 else -0.018),
            z=0.62,
            radius=0.165,
            material=zinc,
        )
    turret.visual(
        Box((0.13, 0.09, 0.12)),
        origin=Origin(xyz=(0.19, 0.275, 0.77)),
        material=caution,
        name="shoulder_stop_block",
    )
    turret.visual(
        Box((0.09, 0.06, 0.34)),
        origin=Origin(xyz=(-0.19, 0.345, 0.56)),
        material=red,
        name="shoulder_lockout_bar",
    )
    turret.visual(
        Cylinder(radius=0.035, length=0.16),
        origin=Origin(xyz=(-0.19, 0.39, 0.74), rpy=CYL_Y),
        material=red,
        name="shoulder_lockout_handle",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.16, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=CYL_Y),
        material=dark_steel,
        name="shoulder_cartridge",
    )
    for side, y in enumerate((0.2015, -0.2015)):
        upper_arm.visual(
            Cylinder(radius=0.18, length=0.047),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=CYL_Y),
            material=dark_steel,
            name=f"shoulder_cartridge_cap_{side}",
        )
        _add_cap_bolts(
            upper_arm,
            prefix=f"shoulder_cartridge_cap_{side}",
            x=0.0,
            y=y + (0.0085 if y > 0 else -0.0085),
            z=0.0,
            radius=0.125,
            material=zinc,
        )
    for side, y in enumerate((0.195, -0.195)):
        upper_arm.visual(
            Box((0.89, 0.05, 0.18)),
            origin=Origin(xyz=(0.43, y, 0.02)),
            material=safety_yellow,
            name=f"upper_side_rail_{side}",
        )
    for side, y in enumerate((0.235, -0.235)):
        upper_arm.visual(
            Box((0.12, 0.07, 0.14)),
            origin=Origin(xyz=(0.91, y, 0.02)),
            material=safety_yellow,
            name=f"elbow_outer_connector_{side}",
        )
    upper_arm.visual(
        Box((0.70, 0.34, 0.05)),
        origin=Origin(xyz=(0.45, 0.0, 0.145)),
        material=guarded_yellow,
        name="upper_top_plate",
    )
    upper_arm.visual(
        Box((0.70, 0.34, 0.05)),
        origin=Origin(xyz=(0.45, 0.0, -0.105)),
        material=guarded_yellow,
        name="upper_bottom_plate",
    )
    upper_arm.visual(
        Box((0.58, 0.055, 0.075)),
        origin=Origin(xyz=(0.42, 0.0, 0.18), rpy=(0.0, -0.32, 0.0)),
        material=dark_steel,
        name="upper_diagonal_brace_0",
    )
    upper_arm.visual(
        Box((0.58, 0.055, 0.075)),
        origin=Origin(xyz=(0.42, 0.0, -0.14), rpy=(0.0, 0.32, 0.0)),
        material=dark_steel,
        name="upper_diagonal_brace_1",
    )
    for side, y in enumerate((0.23, -0.23)):
        upper_arm.visual(
            Box((0.24, 0.07, 0.42)),
            origin=Origin(xyz=(1.05, y, 0.05)),
            material=safety_yellow,
            name=f"elbow_yoke_plate_{side}",
        )
    upper_arm.visual(
        Box((0.22, 0.56, 0.07)),
        origin=Origin(xyz=(1.03, 0.0, 0.29)),
        material=safety_yellow,
        name="elbow_yoke_bridge",
    )
    upper_arm.visual(
        Box((0.11, 0.14, 0.12)),
        origin=Origin(xyz=(0.84, -0.08, 0.205)),
        material=caution,
        name="elbow_stop_block",
    )
    upper_arm.visual(
        Box((0.12, 0.05, 0.22)),
        origin=Origin(xyz=(0.91, 0.275, 0.04)),
        material=red,
        name="elbow_lockout_bar",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.13, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=CYL_Y),
        material=dark_steel,
        name="elbow_cartridge",
    )
    for side, y in enumerate((0.1775, -0.1775)):
        forearm.visual(
            Cylinder(radius=0.145, length=0.035),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=CYL_Y),
            material=dark_steel,
            name=f"elbow_cartridge_cap_{side}",
        )
    for side, y in enumerate((0.17, -0.17)):
        forearm.visual(
            Box((0.64, 0.04, 0.14)),
            origin=Origin(xyz=(0.42, y, 0.0)),
            material=safety_yellow,
            name=f"forearm_side_rail_{side}",
        )
    forearm.visual(
        Box((0.58, 0.36, 0.052)),
        origin=Origin(xyz=(0.43, 0.0, 0.085)),
        material=guarded_yellow,
        name="forearm_top_plate",
    )
    forearm.visual(
        Box((0.58, 0.36, 0.052)),
        origin=Origin(xyz=(0.43, 0.0, -0.085)),
        material=guarded_yellow,
        name="forearm_bottom_plate",
    )
    forearm.visual(
        Box((0.46, 0.30, 0.08)),
        origin=Origin(xyz=(0.40, 0.0, 0.125), rpy=(0.0, -0.28, 0.0)),
        material=dark_steel,
        name="forearm_diagonal_brace",
    )
    forearm.visual(
        Box((0.48, 0.06, 0.055)),
        origin=Origin(xyz=(0.43, -0.17, 0.135)),
        material=black,
        name="forearm_cable_tray",
    )
    for side, y in enumerate((0.18, -0.18)):
        forearm.visual(
            Box((0.20, 0.06, 0.30)),
            origin=Origin(xyz=(0.84, y, 0.0)),
            material=safety_yellow,
            name=f"wrist_yoke_plate_{side}",
        )
    forearm.visual(
        Box((0.16, 0.44, 0.06)),
        origin=Origin(xyz=(0.83, 0.0, 0.18)),
        material=safety_yellow,
        name="wrist_yoke_bridge",
    )
    forearm.visual(
        Box((0.095, 0.11, 0.10)),
        origin=Origin(xyz=(0.68, 0.08, 0.155)),
        material=caution,
        name="wrist_stop_block",
    )
    forearm.visual(
        Box((0.07, 0.05, 0.20)),
        origin=Origin(xyz=(0.72, -0.215, 0.02)),
        material=red,
        name="wrist_lockout_bar",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.105, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=CYL_Y),
        material=dark_steel,
        name="wrist_pitch_cartridge",
    )
    for side, y in enumerate((0.134, -0.134)):
        wrist.visual(
            Cylinder(radius=0.115, length=0.032),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=CYL_Y),
            material=dark_steel,
            name=f"wrist_pitch_cap_{side}",
        )
    wrist.visual(
        Box((0.25, 0.20, 0.22)),
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        material=safety_yellow,
        name="wrist_housing",
    )
    wrist.visual(
        Cylinder(radius=0.115, length=0.16),
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=CYL_X),
        material=dark_steel,
        name="roll_bearing_body",
    )
    wrist.visual(
        Box((0.11, 0.24, 0.07)),
        origin=Origin(xyz=(0.25, 0.0, 0.145)),
        material=black,
        name="roll_guard_bridge",
    )
    wrist.visual(
        Box((0.05, 0.12, 0.12)),
        origin=Origin(xyz=(0.26, 0.16, 0.0)),
        material=red,
        name="roll_lockout_tab",
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=CYL_X),
        material=dark_steel,
        name="flange_disk",
    )
    tool_flange.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=CYL_X),
        material=zinc,
        name="tool_boss",
    )
    for i in range(8):
        a = 2.0 * pi * i / 8.0
        tool_flange.visual(
            Cylinder(radius=0.012, length=0.024),
            origin=Origin(
                xyz=(0.087, 0.105 * cos(a), 0.105 * sin(a)),
                rpy=CYL_X,
            ),
            material=zinc,
            name=f"flange_bolt_{i}",
        )

    yaw = model.articulation(
        "base_to_turret",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.2, lower=-pi, upper=pi),
        motion_properties=MotionProperties(damping=8.0, friction=2.0),
    )
    shoulder = model.articulation(
        "turret_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1250.0, velocity=0.9, lower=-0.95, upper=1.15),
        motion_properties=MotionProperties(damping=10.0, friction=3.0),
    )
    elbow = model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(1.05, 0.0, 0.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=950.0, velocity=1.1, lower=-1.25, upper=1.35),
        motion_properties=MotionProperties(damping=8.0, friction=2.5),
    )
    wrist_pitch = model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.84, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=360.0, velocity=1.8, lower=-1.55, upper=1.55),
        motion_properties=MotionProperties(damping=5.0, friction=1.2),
    )
    roll = model.articulation(
        "wrist_to_tool_flange",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=tool_flange,
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.4, lower=-pi, upper=pi),
        motion_properties=MotionProperties(damping=3.0, friction=0.8),
    )

    # Keep symbolic names live so static analyzers do not hide the intended
    # base/shoulder/elbow/wrist order above.
    _ = (yaw, shoulder, elbow, wrist_pitch, roll)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    tool_flange = object_model.get_part("tool_flange")

    yaw = object_model.get_articulation("base_to_turret")
    shoulder = object_model.get_articulation("turret_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")
    wrist_pitch = object_model.get_articulation("forearm_to_wrist")
    roll = object_model.get_articulation("wrist_to_tool_flange")

    ctx.check(
        "industrial chain contains base shoulder elbow wrist roll",
        [joint.name for joint in object_model.articulations]
        == [
            "base_to_turret",
            "turret_to_upper_arm",
            "upper_arm_to_forearm",
            "forearm_to_wrist",
            "wrist_to_tool_flange",
        ],
    )
    ctx.check("yaw axis is vertical", tuple(yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("shoulder and elbow pitch upward on positive command", tuple(shoulder.axis) == (0.0, -1.0, 0.0) and tuple(elbow.axis) == (0.0, -1.0, 0.0))
    ctx.check("wrist roll axis follows the tool centerline", tuple(roll.axis) == (1.0, 0.0, 0.0))

    ctx.expect_gap(
        turret,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.00001,
        positive_elem="rotating_table",
        negative_elem="fixed_bearing_ring",
        name="turntable sits on fixed bearing ring",
    )
    ctx.expect_within(
        upper_arm,
        turret,
        axes="y",
        inner_elem="shoulder_cartridge",
        outer_elem="shoulder_yoke_tie",
        margin=0.02,
        name="shoulder cartridge is captured between guarded yoke plates",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="y",
        inner_elem="elbow_cartridge",
        outer_elem="elbow_yoke_bridge",
        margin=0.02,
        name="elbow cartridge remains centered inside upper-arm yoke",
    )
    ctx.expect_within(
        wrist,
        forearm,
        axes="y",
        inner_elem="wrist_pitch_cartridge",
        outer_elem="wrist_yoke_bridge",
        margin=0.02,
        name="wrist pitch cartridge remains centered inside forearm yoke",
    )

    rest_tool = ctx.part_world_position(tool_flange)
    with ctx.pose({shoulder: 0.55, elbow: 0.35, wrist_pitch: 0.25, roll: 1.0}):
        lifted_wrist = ctx.part_world_position(wrist)
        lifted_tool = ctx.part_world_position(tool_flange)
    ctx.check(
        "positive pitch commands lift the wrist and tool path",
        rest_tool is not None
        and lifted_wrist is not None
        and lifted_tool is not None
        and lifted_wrist[2] > rest_tool[2] + 0.25
        and lifted_tool[2] > rest_tool[2] + 0.25,
        details=f"rest_tool={rest_tool}, lifted_wrist={lifted_wrist}, lifted_tool={lifted_tool}",
    )

    return ctx.report()


object_model = build_object_model()
