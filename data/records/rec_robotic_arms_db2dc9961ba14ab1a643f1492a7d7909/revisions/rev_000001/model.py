from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_and_place_robotic_arm")

    base_paint = model.material("base_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.84, 0.85, 0.87, 1.0))
    accent = model.material("accent", rgba=(0.96, 0.43, 0.12, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.56, 0.59, 0.63, 1.0))
    tool_dark = model.material("tool_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((0.34, 0.34, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=base_paint,
        name="floor_base",
    )
    pedestal_base.visual(
        Box((0.22, 0.22, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=base_paint,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Box((0.26, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=machined_steel,
        name="turntable_cap",
    )
    pedestal_base.visual(
        Box((0.18, 0.26, 0.20)),
        origin=Origin(xyz=(-0.02, 0.0, 0.40)),
        material=base_paint,
        name="rear_service_cabinet",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.60)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )

    shoulder_assembly = model.part("shoulder_assembly")
    shoulder_assembly.visual(
        Box((0.26, 0.26, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=machined_steel,
        name="rotary_base_housing",
    )
    shoulder_assembly.visual(
        Box((0.30, 0.24, 0.20)),
        origin=Origin(xyz=(0.01, 0.0, 0.24)),
        material=arm_paint,
        name="shoulder_housing",
    )
    shoulder_assembly.visual(
        Box((0.44, 0.16, 0.18)),
        origin=Origin(xyz=(0.24, 0.0, 0.24)),
        material=arm_paint,
        name="upper_arm_beam",
    )
    shoulder_assembly.visual(
        Box((0.12, 0.24, 0.24)),
        origin=Origin(xyz=(0.49, 0.0, 0.24)),
        material=accent,
        name="elbow_housing",
    )
    shoulder_assembly.visual(
        Cylinder(radius=0.06, length=0.18),
        origin=Origin(xyz=(0.49, 0.0, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="elbow_bearing_cap",
    )
    shoulder_assembly.inertial = Inertial.from_geometry(
        Box((0.61, 0.30, 0.34)),
        mass=34.0,
        origin=Origin(xyz=(0.21, 0.0, 0.17)),
    )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        Box((0.10, 0.18, 0.20)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=accent,
        name="elbow_knuckle",
    )
    forearm_link.visual(
        Box((0.40, 0.12, 0.12)),
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        material=arm_paint,
        name="forearm_beam",
    )
    forearm_link.visual(
        Box((0.08, 0.14, 0.14)),
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
        material=machined_steel,
        name="wrist_drive_housing",
    )
    forearm_link.visual(
        Cylinder(radius=0.05, length=0.14),
        origin=Origin(xyz=(0.43, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="wrist_roll_bearing",
    )
    forearm_link.inertial = Inertial.from_geometry(
        Box((0.51, 0.20, 0.20)),
        mass=18.0,
        origin=Origin(xyz=(0.23, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Box((0.14, 0.12, 0.18)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=tool_dark,
        name="wrist_body",
    )
    wrist_head.visual(
        Box((0.05, 0.12, 0.06)),
        origin=Origin(xyz=(0.165, 0.0, -0.05)),
        material=machined_steel,
        name="tool_plate",
    )
    wrist_head.visual(
        Box((0.05, 0.05, 0.05)),
        origin=Origin(xyz=(0.10, 0.04, 0.07)),
        material=accent,
        name="wrist_service_block",
    )
    wrist_head.visual(
        Cylinder(radius=0.03, length=0.03),
        origin=Origin(xyz=(0.19, 0.0, -0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tool_dark,
        name="tool_flange",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.22, 0.14, 0.20)),
        mass=7.5,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=shoulder_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.6,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_assembly,
        child=forearm_link,
        origin=Origin(xyz=(0.55, 0.0, 0.24)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.8,
            lower=math.radians(-45.0),
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=wrist_head,
        origin=Origin(xyz=(0.47, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=3.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal_base = object_model.get_part("pedestal_base")
    shoulder_assembly = object_model.get_part("shoulder_assembly")
    forearm_link = object_model.get_part("forearm_link")
    wrist_head = object_model.get_part("wrist_head")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    with ctx.pose({shoulder_yaw: 0.0, elbow_pitch: 0.0, wrist_roll: 0.0}):
        ctx.expect_gap(
            shoulder_assembly,
            pedestal_base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="shoulder assembly seats on pedestal",
        )
        ctx.expect_overlap(
            shoulder_assembly,
            pedestal_base,
            axes="xy",
            min_overlap=0.20,
            name="shoulder base remains centered on pedestal",
        )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 1.0}):
        lifted_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch lifts the wrist head",
        rest_wrist_pos is not None
        and lifted_wrist_pos is not None
        and lifted_wrist_pos[2] > rest_wrist_pos[2] + 0.18,
        details=f"rest={rest_wrist_pos}, lifted={lifted_wrist_pos}",
    )

    rest_forearm_pos = ctx.part_world_position(forearm_link)
    with ctx.pose({shoulder_yaw: math.pi / 2.0}):
        swung_forearm_pos = ctx.part_world_position(forearm_link)
    ctx.check(
        "shoulder yaw swings the arm around the pedestal",
        rest_forearm_pos is not None
        and swung_forearm_pos is not None
        and swung_forearm_pos[1] > rest_forearm_pos[1] + 0.30,
        details=f"rest={rest_forearm_pos}, swung={swung_forearm_pos}",
    )

    rest_tool_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_plate"))
    with ctx.pose({wrist_roll: math.pi / 2.0}):
        rolled_tool_center = _aabb_center(ctx.part_element_world_aabb(wrist_head, elem="tool_plate"))
    ctx.check(
        "wrist roll reorients the offset tool plate",
        rest_tool_center is not None
        and rolled_tool_center is not None
        and abs(rolled_tool_center[1] - rest_tool_center[1]) > 0.035
        and rolled_tool_center[2] > rest_tool_center[2] + 0.03,
        details=f"rest={rest_tool_center}, rolled={rolled_tool_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
