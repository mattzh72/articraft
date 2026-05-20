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


PIN_AXIS_RPY = (-pi / 2.0, 0.0, 0.0)  # rotate a cylinder's local +Z onto +Y


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_link_task_arm")

    cast_gray = _mat("cast_gray", (0.45, 0.47, 0.48, 1.0))
    dark_joint = _mat("dark_joint", (0.12, 0.13, 0.14, 1.0))
    link_blue = _mat("link_blue", (0.10, 0.28, 0.58, 1.0))
    rubber = _mat("black_rubber", (0.02, 0.02, 0.018, 1.0))
    warning = _mat("yellow_index", (0.95, 0.70, 0.10, 1.0))

    link_1 = 0.56
    link_2 = 0.44
    link_3 = 0.26

    base = model.part("base")
    base.visual(
        Box((0.36, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.64)),
        material=cast_gray,
        name="floor_plate",
    )
    base.visual(
        Cylinder(0.070, 0.535),
        origin=Origin(xyz=(0.0, 0.0, -0.3675)),
        material=cast_gray,
        name="pedestal",
    )
    base.visual(
        Cylinder(0.080, 0.18),
        origin=Origin(rpy=PIN_AXIS_RPY),
        material=dark_joint,
        name="shoulder_bearing",
    )
    base.visual(
        Box((0.080, 0.030, 0.24)),
        origin=Origin(xyz=(0.0, 0.105, -0.045)),
        material=cast_gray,
        name="fork_cheek_0",
    )
    base.visual(
        Box((0.080, 0.030, 0.24)),
        origin=Origin(xyz=(0.0, -0.105, -0.045)),
        material=cast_gray,
        name="fork_cheek_1",
    )
    base.visual(
        Box((0.050, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.080, -0.100)),
        material=cast_gray,
        name="neck_web_0",
    )
    base.visual(
        Box((0.050, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, -0.080, -0.100)),
        material=cast_gray,
        name="neck_web_1",
    )

    upper = model.part("upper_arm")
    upper.visual(
        Cylinder(0.088, 0.105),
        origin=Origin(rpy=PIN_AXIS_RPY),
        material=dark_joint,
        name="shoulder_hub",
    )
    upper.visual(
        Box(((link_1 - 0.058) - 0.080, 0.070, 0.055)),
        origin=Origin(xyz=((0.080 + link_1 - 0.058) / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="upper_beam",
    )
    upper.visual(
        Cylinder(0.075, 0.115),
        origin=Origin(xyz=(link_1, 0.0, 0.0), rpy=PIN_AXIS_RPY),
        material=dark_joint,
        name="elbow_bearing",
    )
    upper.visual(
        Box((0.090, 0.022, 0.018)),
        origin=Origin(xyz=(0.12, 0.0, 0.041)),
        material=warning,
        name="angle_index",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(0.070, 0.092),
        origin=Origin(rpy=PIN_AXIS_RPY),
        material=dark_joint,
        name="elbow_hub",
    )
    forearm.visual(
        Box(((link_2 - 0.050) - 0.064, 0.060, 0.050)),
        origin=Origin(xyz=((0.064 + link_2 - 0.050) / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="forearm_beam",
    )
    forearm.visual(
        Cylinder(0.062, 0.096),
        origin=Origin(xyz=(link_2, 0.0, 0.0), rpy=PIN_AXIS_RPY),
        material=dark_joint,
        name="wrist_bearing",
    )
    forearm.visual(
        Box((0.080, 0.018, 0.014)),
        origin=Origin(xyz=(0.16, 0.0, 0.032)),
        material=warning,
        name="elbow_index",
    )

    wrist = model.part("wrist_link")
    wrist.visual(
        Cylinder(0.058, 0.080),
        origin=Origin(rpy=PIN_AXIS_RPY),
        material=dark_joint,
        name="wrist_hub",
    )
    wrist.visual(
        Box(((link_3 - 0.030) - 0.052, 0.052, 0.045)),
        origin=Origin(xyz=((0.052 + link_3 - 0.030) / 2.0, 0.0, 0.0)),
        material=link_blue,
        name="short_link",
    )
    wrist.visual(
        Box((0.070, 0.046, 0.070)),
        origin=Origin(xyz=(link_3, 0.0, -0.035)),
        material=dark_joint,
        name="pad_mount",
    )
    wrist.visual(
        Box((0.20, 0.13, 0.030)),
        origin=Origin(xyz=(link_3 + 0.045, 0.0, -0.085)),
        material=rubber,
        name="rectangular_pad",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.85, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(link_1, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=-2.20, upper=2.20),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(link_2, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.85, upper=1.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist")

    for joint in (shoulder, elbow, wrist_joint):
        ctx.check(
            f"{joint.name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    ctx.allow_overlap(
        base,
        upper,
        elem_a="shoulder_bearing",
        elem_b="shoulder_hub",
        reason="The shoulder hub is intentionally captured coaxially inside the bearing housing.",
    )
    ctx.expect_overlap(
        base,
        upper,
        axes="xyz",
        elem_a="shoulder_bearing",
        elem_b="shoulder_hub",
        min_overlap=0.070,
        name="shoulder hub captured in bearing",
    )
    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_bearing",
        elem_b="elbow_hub",
        reason="The elbow hub is intentionally represented as a captured pin-and-bushing fit.",
    )
    ctx.expect_overlap(
        upper,
        forearm,
        axes="xyz",
        elem_a="elbow_bearing",
        elem_b="elbow_hub",
        min_overlap=0.060,
        name="elbow hub captured in bearing",
    )
    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="upper_beam",
        elem_b="elbow_hub",
        reason="The simplified elbow boss locally embeds the beam end to show the pinned link tongue.",
    )
    ctx.expect_overlap(
        upper,
        forearm,
        axes="xyz",
        elem_a="upper_beam",
        elem_b="elbow_hub",
        min_overlap=0.008,
        name="upper beam seated at elbow boss",
    )
    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_bearing",
        elem_b="forearm_beam",
        reason="The forearm beam root is locally seated in the simplified elbow bearing boss.",
    )
    ctx.expect_overlap(
        upper,
        forearm,
        axes="xyz",
        elem_a="elbow_bearing",
        elem_b="forearm_beam",
        min_overlap=0.008,
        name="forearm beam seated at elbow boss",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_bearing",
        elem_b="wrist_hub",
        reason="The wrist hub is intentionally represented as a coaxial captured bearing.",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="xyz",
        elem_a="wrist_bearing",
        elem_b="wrist_hub",
        min_overlap=0.052,
        name="wrist hub captured in bearing",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_bearing",
        elem_b="short_link",
        reason="The wrist link root is locally seated in the simplified bearing boss.",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="xyz",
        elem_a="wrist_bearing",
        elem_b="short_link",
        min_overlap=0.008,
        name="short link seated at wrist boss",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="forearm_beam",
        elem_b="wrist_hub",
        reason="The forearm tip is locally embedded in the simplified wrist hub to show the pinned tongue.",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="xyz",
        elem_a="forearm_beam",
        elem_b="wrist_hub",
        min_overlap=0.006,
        name="forearm beam seated at wrist hub",
    )

    rest_pad_aabb = ctx.part_element_world_aabb(wrist, elem="rectangular_pad")
    with ctx.pose({shoulder: 0.70, elbow: -1.05, wrist_joint: 0.80}):
        posed_pad_aabb = ctx.part_element_world_aabb(wrist, elem="rectangular_pad")
    ctx.check(
        "pad moves under shoulder elbow wrist pose",
        rest_pad_aabb is not None
        and posed_pad_aabb is not None
        and abs(posed_pad_aabb[0][2] - rest_pad_aabb[0][2]) > 0.08,
        details=f"rest={rest_pad_aabb}, posed={posed_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
