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


def _tube_between(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, Origin]:
    """Return length and origin for a cylinder whose local +Z spans start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # The arm links lie in the XZ plane, so a pitch about local Y is enough to
    # rotate a cylinder from local +Z onto the desired bar direction.
    pitch = math.atan2(dx, dz)
    return length, Origin(xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5), rpy=(0.0, pitch, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_slide_arm")

    steel = model.material("dark_blued_steel", rgba=(0.06, 0.07, 0.08, 1.0))
    rail_metal = model.material("satin_galvanized_rail", rgba=(0.58, 0.62, 0.64, 1.0))
    carriage_paint = model.material("safety_yellow_carriage", rgba=(0.95, 0.66, 0.12, 1.0))
    arm_paint = model.material("matte_black_arm", rgba=(0.015, 0.017, 0.018, 1.0))
    pin_metal = model.material("brushed_pin_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    rubber = model.material("dark_rubber_tread", rgba=(0.01, 0.01, 0.012, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((1.30, 0.30, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=rail_metal,
        name="ceiling_plate",
    )
    top_support.visual(
        Box((1.20, 0.13, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=steel,
        name="rail_web",
    )
    for idx, y in enumerate((-0.0825, 0.0825)):
        top_support.visual(
            Box((1.20, 0.030, 0.110)),
            origin=Origin(xyz=(0.0, y, -0.035)),
            material=steel,
            name=f"side_flange_{idx}",
        )
    top_support.visual(
        Box((1.20, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -0.050, -0.099)),
        material=steel,
        name="lower_lip_0",
    )
    top_support.visual(
        Box((1.20, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.050, -0.099)),
        material=steel,
        name="lower_lip_1",
    )
    for idx, x in enumerate((-0.620, 0.620)):
        top_support.visual(
            Box((0.040, 0.170, 0.120)),
            origin=Origin(xyz=(x, 0.0, -0.035)),
            material=rail_metal,
            name=f"end_stop_{idx}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.190, 0.115, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=carriage_paint,
        name="trolley_body",
    )
    for idx, x in enumerate((-0.060, 0.060)):
        carriage.visual(
            Cylinder(radius=0.008, length=0.150),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_metal,
            name=f"roller_axle_{idx}",
        )
    carriage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.060, -0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_0_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.060, 0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_0_1",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.060, -0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_1_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.060, 0.064, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_1_1",
    )
    carriage.visual(
        Box((0.052, 0.050, 0.145)),
        origin=Origin(xyz=(0.0, 0.0, -0.1225)),
        material=carriage_paint,
        name="drop_stem",
    )
    carriage.visual(
        Box((0.095, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=carriage_paint,
        name="shoulder_bridge",
    )
    for idx, y in enumerate((-0.038, 0.038)):
        carriage.visual(
            Box((0.098, 0.014, 0.090)),
            origin=Origin(xyz=(0.0, y, -0.250)),
            material=carriage_paint,
            name=f"shoulder_cheek_{idx}",
        )
        carriage.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(0.0, y * 1.32, -0.255), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_metal,
            name=f"shoulder_pin_cap_{idx}",
        )

    upper_arm = model.part("upper_arm")
    elbow_local = (0.320, 0.0, -0.380)
    upper_len, upper_origin = _tube_between((0.0, 0.0, 0.0), elbow_local)
    upper_arm.visual(
        Cylinder(radius=0.019, length=upper_len),
        origin=upper_origin,
        material=arm_paint,
        name="upper_tube",
    )
    upper_arm.visual(
        Cylinder(radius=0.045, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_paint,
        name="shoulder_boss",
    )
    upper_arm.visual(
        Cylinder(radius=0.040, length=0.034),
        origin=Origin(xyz=elbow_local, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_paint,
        name="elbow_inner_boss",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.125),
        origin=Origin(xyz=elbow_local, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="elbow_pin",
    )

    forearm = model.part("forearm")
    wrist_local = (0.280, 0.0, -0.255)
    fore_len, fore_origin_base = _tube_between((0.026, 0.0, -0.024), wrist_local)
    forearm.visual(
        Cylinder(radius=0.013, length=fore_len),
        origin=Origin(xyz=(fore_origin_base.xyz[0], -0.060, fore_origin_base.xyz[2]), rpy=fore_origin_base.rpy),
        material=arm_paint,
        name="forearm_rail_0",
    )
    forearm.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_paint,
        name="elbow_outer_boss_0",
    )
    forearm.visual(
        Cylinder(radius=0.013, length=fore_len),
        origin=Origin(xyz=(fore_origin_base.xyz[0], 0.060, fore_origin_base.xyz[2]), rpy=fore_origin_base.rpy),
        material=arm_paint,
        name="forearm_rail_1",
    )
    forearm.visual(
        Cylinder(radius=0.038, length=0.018),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_paint,
        name="elbow_outer_boss_1",
    )
    forearm.visual(
        Cylinder(radius=0.033, length=0.150),
        origin=Origin(xyz=wrist_local, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_paint,
        name="wrist_crossbar",
    )
    forearm.visual(
        Box((0.070, 0.150, 0.030)),
        origin=Origin(xyz=(wrist_local[0] + 0.035, 0.0, wrist_local[2] - 0.015)),
        material=pin_metal,
        name="tool_mount",
    )

    model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(-0.300, 0.0, -0.126)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.40, lower=0.0, upper=0.600),
    )
    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.3, lower=-0.95, upper=1.15),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=elbow_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    slide = object_model.get_articulation("slide")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="elbow_outer_boss_0",
        elem_b="elbow_pin",
        reason="The elbow pin is intentionally captured through the forearm fork boss.",
    )
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="elbow_outer_boss_1",
        elem_b="elbow_pin",
        reason="The elbow pin is intentionally captured through the forearm fork boss.",
    )

    ctx.check("carriage rides on a prismatic slide", slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("shoulder joint is revolute", shoulder.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("elbow joint is revolute", elbow.articulation_type == ArticulationType.REVOLUTE)

    ctx.expect_within(
        carriage,
        top_support,
        axes="xy",
        margin=0.010,
        inner_elem="trolley_body",
        outer_elem="ceiling_plate",
        name="carriage remains under the support footprint",
    )
    ctx.expect_gap(
        top_support,
        carriage,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="lower_lip_0",
        negative_elem="roller_0_0",
        name="rollers touch the underside of the rail lips",
    )
    ctx.expect_overlap(
        upper_arm,
        carriage,
        axes="xy",
        min_overlap=0.025,
        elem_a="shoulder_boss",
        elem_b="shoulder_bridge",
        name="upper arm is centered at the shoulder bracket",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        min_gap=0.025,
        max_gap=0.045,
        positive_elem="elbow_inner_boss",
        negative_elem="elbow_outer_boss_0",
        name="near forearm cheek clears the elbow boss",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        min_gap=0.025,
        max_gap=0.045,
        positive_elem="elbow_outer_boss_1",
        negative_elem="elbow_inner_boss",
        name="far forearm cheek clears the elbow boss",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        min_overlap=0.020,
        elem_a="elbow_outer_boss_0",
        elem_b="elbow_pin",
        name="elbow pin passes through the near forearm boss",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="xz",
        min_overlap=0.020,
        elem_a="elbow_outer_boss_1",
        elem_b="elbow_pin",
        name="elbow pin passes through the far forearm boss",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.600}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            top_support,
            axes="xy",
            margin=0.010,
            inner_elem="trolley_body",
            outer_elem="ceiling_plate",
            name="extended carriage remains under the support footprint",
        )
    ctx.check(
        "slide translates the hanging carriage along the rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.55,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    elbow_pos = ctx.part_world_position(forearm)
    carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "two-link arm hangs below the carriage",
        elbow_pos is not None and carriage_pos is not None and elbow_pos[2] < carriage_pos[2] - 0.55,
        details=f"carriage={carriage_pos}, elbow={elbow_pos}",
    )

    with ctx.pose({shoulder: 0.55, elbow: -0.65}):
        posed_elbow = ctx.part_world_position(forearm)
    ctx.check(
        "revolute joints swing the arm in the vertical plane",
        elbow_pos is not None
        and posed_elbow is not None
        and abs(posed_elbow[0] - elbow_pos[0]) > 0.15
        and abs(posed_elbow[2] - elbow_pos[2]) > 0.05,
        details=f"rest_elbow={elbow_pos}, posed_elbow={posed_elbow}",
    )

    return ctx.report()


object_model = build_object_model()
