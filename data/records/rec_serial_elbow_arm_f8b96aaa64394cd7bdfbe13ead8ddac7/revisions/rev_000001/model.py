from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_elbow_arm_fixture")

    painted_steel = Material("warm_gray_painted_steel", rgba=(0.48, 0.50, 0.48, 1.0))
    dark_steel = Material("dark_blued_pin_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    zinc = Material("brushed_zinc_boss_faces", rgba=(0.72, 0.70, 0.64, 1.0))
    black = Material("black_recesses", rgba=(0.01, 0.01, 0.012, 1.0))

    # A compact floor / bench fixture: the grounded stand carries a shoulder
    # clevis, but the moving link has its own centered boss between the cheeks.
    stand = model.part("stand")
    stand.visual(
        Box((0.46, 0.32, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=painted_steel,
        name="base_plate",
    )
    stand.visual(
        Box((0.11, 0.11, 0.58)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=painted_steel,
        name="vertical_post",
    )
    stand.visual(
        Box((0.16, 0.20, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.6325)),
        material=painted_steel,
        name="shoulder_bridge",
    )
    for y, name in ((0.0775, "shoulder_cheek_0"), (-0.0775, "shoulder_cheek_1")):
        stand.visual(
            Box((0.13, 0.025, 0.18)),
            origin=Origin(xyz=(0.0, y, 0.72)),
            material=painted_steel,
            name=name,
        )
    stand.visual(
        Cylinder(radius=0.016, length=0.168),
        origin=Origin(xyz=(0.0, 0.0, 0.72), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_pin",
    )
    for y, name in ((0.092, "shoulder_pin_cap_0"), (-0.092, "shoulder_pin_cap_1")):
        stand.visual(
            Cylinder(radius=0.038, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.72), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=name,
        )
    for x, y, name in (
        (0.16, 0.10, "base_bolt_0"),
        (0.16, -0.10, "base_bolt_1"),
        (-0.16, 0.10, "base_bolt_2"),
        (-0.16, -0.10, "base_bolt_3"),
    ):
        stand.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(x, y, 0.047)),
            material=black,
            name=name,
        )

    upper = model.part("upper_link")
    upper_len = 0.46
    # Shoulder boss: a central hub that fits inside the stand clevis gap.
    upper.visual(
        Cylinder(radius=0.050, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="shoulder_boss",
    )
    # Distal elbow fork: two separated bosses leave a visible slot for the
    # second link's tongue.  The long side rails keep this link open-frame.
    upper_rail_start = 0.048
    upper_rail_len = upper_len - upper_rail_start
    for y, suffix in ((0.041, "0"), (-0.041, "1")):
        upper.visual(
            Box((upper_rail_len, 0.018, 0.036)),
            origin=Origin(xyz=(upper_rail_start + upper_rail_len / 2.0, y, 0.0)),
            material=painted_steel,
            name=f"upper_rail_{suffix}",
        )
    upper.visual(
        Cylinder(radius=0.016, length=0.094),
        origin=Origin(xyz=(upper_len * 0.50, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="upper_cross_tube",
    )
    upper.visual(
        Cylinder(radius=0.013, length=0.135),
        origin=Origin(xyz=(upper_len, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_pin",
    )
    for y, suffix in ((0.052, "0"), (-0.052, "1")):
        upper.visual(
            Cylinder(radius=0.048, length=0.026),
            origin=Origin(xyz=(upper_len, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"elbow_fork_boss_{suffix}",
        )
        upper.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(upper_len, y * 1.28, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"elbow_pin_head_{suffix}",
        )

    forearm = model.part("forearm_link")
    forearm_len = 0.38
    forearm.visual(
        Cylinder(radius=0.042, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="elbow_boss",
    )
    forearm_rail_start = 0.040
    forearm_rail_len = forearm_len - forearm_rail_start
    for y, suffix in ((0.018, "0"), (-0.018, "1")):
        forearm.visual(
            Box((forearm_rail_len, 0.012, 0.030)),
            origin=Origin(xyz=(forearm_rail_start + forearm_rail_len / 2.0, y, 0.0)),
            material=painted_steel,
            name=f"forearm_rail_{suffix}",
        )
    forearm.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(forearm_len * 0.58, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="forearm_cross_tube",
    )
    forearm.visual(
        Box((0.030, 0.18, 0.125)),
        origin=Origin(xyz=(forearm_len + 0.015, 0.0, 0.0)),
        material=painted_steel,
        name="end_plate",
    )
    for y, z, name in (
        (0.052, 0.036, "tool_bolt_0"),
        (0.052, -0.036, "tool_bolt_1"),
        (-0.052, 0.036, "tool_bolt_2"),
        (-0.052, -0.036, "tool_bolt_3"),
    ):
        forearm.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(forearm_len + 0.033, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=name,
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.45, upper=1.25),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-1.35, upper=1.35),
        motion_properties=MotionProperties(damping=0.06, friction=0.025),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm_link")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        stand,
        upper,
        elem_a="shoulder_pin",
        elem_b="shoulder_boss",
        reason="The exposed shoulder shaft is intentionally captured through the rotating upper-link boss.",
    )
    ctx.expect_overlap(
        stand,
        upper,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_boss",
        min_overlap=0.080,
        name="shoulder shaft crosses the upper boss",
    )
    ctx.expect_gap(
        stand,
        upper,
        axis="y",
        positive_elem="shoulder_cheek_0",
        negative_elem="shoulder_boss",
        min_gap=0.010,
        max_gap=0.030,
        name="upper boss clears the positive shoulder cheek",
    )

    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_pin",
        elem_b="elbow_boss",
        reason="The exposed elbow shaft is intentionally captured through the second-link boss.",
    )
    ctx.expect_overlap(
        upper,
        forearm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_boss",
        min_overlap=0.050,
        name="elbow shaft crosses the forearm boss",
    )
    ctx.expect_gap(
        upper,
        forearm,
        axis="y",
        positive_elem="elbow_fork_boss_0",
        negative_elem="elbow_boss",
        min_gap=0.006,
        max_gap=0.020,
        name="forearm boss sits inside the elbow fork slot",
    )

    shoulder_rest = ctx.part_element_world_aabb(upper, elem="upper_cross_tube")
    with ctx.pose({shoulder: 0.65}):
        shoulder_raised = ctx.part_element_world_aabb(upper, elem="upper_cross_tube")
    ctx.check(
        "shoulder revolute raises the upper link",
        shoulder_rest is not None
        and shoulder_raised is not None
        and shoulder_raised[0][2] > shoulder_rest[0][2] + 0.10,
        details=f"rest={shoulder_rest}, raised={shoulder_raised}",
    )

    end_rest = ctx.part_element_world_aabb(forearm, elem="end_plate")
    with ctx.pose({elbow: 0.75}):
        end_bent = ctx.part_element_world_aabb(forearm, elem="end_plate")
    ctx.check(
        "elbow revolute bends the end plate",
        end_rest is not None
        and end_bent is not None
        and end_bent[0][2] < end_rest[0][2] - 0.10,
        details=f"rest={end_rest}, bent={end_bent}",
    )

    return ctx.report()


object_model = build_object_model()
