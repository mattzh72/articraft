from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_carriage_elbow_arm")

    dark_steel = model.material("dark_steel", rgba=(0.11, 0.12, 0.13, 1.0))
    rail_gray = model.material("ground_rail_gray", rgba=(0.32, 0.34, 0.35, 1.0))
    wear_black = model.material("black_wear_strip", rgba=(0.025, 0.027, 0.03, 1.0))
    carriage_blue = model.material("anodized_carriage_blue", rgba=(0.10, 0.24, 0.42, 1.0))
    arm_orange = model.material("safety_orange_arm", rgba=(0.95, 0.43, 0.10, 1.0))
    arm_yellow = model.material("warm_yellow_forearm", rgba=(0.90, 0.68, 0.18, 1.0))
    cap_dark = model.material("dark_pivot_caps", rgba=(0.05, 0.055, 0.06, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((1.35, 0.40, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    rail.visual(
        Box((1.20, 0.24, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=rail_gray,
        name="rail_beam",
    )
    rail.visual(
        Box((1.12, 0.035, 0.046)),
        origin=Origin(xyz=(0.0, -0.125, 0.102)),
        material=rail_gray,
        name="guide_0",
    )
    rail.visual(
        Box((1.08, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.125, 0.126)),
        material=wear_black,
        name="wear_strip_0",
    )
    rail.visual(
        Box((1.12, 0.035, 0.046)),
        origin=Origin(xyz=(0.0, 0.125, 0.102)),
        material=rail_gray,
        name="guide_1",
    )
    rail.visual(
        Box((1.08, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.125, 0.126)),
        material=wear_black,
        name="wear_strip_1",
    )
    for idx, x in enumerate((-0.625, 0.625)):
        rail.visual(
            Box((0.050, 0.34, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=dark_steel,
            name=f"end_stop_{idx}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, -0.125, 0.009)),
        material=wear_black,
        name="shoe_0",
    )
    carriage.visual(
        Box((0.220, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.125, 0.009)),
        material=wear_black,
        name="shoe_1",
    )
    carriage.visual(
        Box((0.260, 0.320, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=carriage_blue,
        name="saddle_body",
    )
    carriage.visual(
        Box((0.180, 0.210, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=dark_steel,
        name="shoulder_mount",
    )
    carriage.visual(
        Cylinder(radius=0.073, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=cap_dark,
        name="shoulder_bearing",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.075, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=arm_orange,
        name="shoulder_hub",
    )
    upper_link.visual(
        Box((0.480, 0.070, 0.030)),
        origin=Origin(xyz=(0.240, 0.0, 0.020)),
        material=arm_orange,
        name="main_link",
    )
    upper_link.visual(
        Cylinder(radius=0.067, length=0.036),
        origin=Origin(xyz=(0.480, 0.0, 0.018)),
        material=arm_orange,
        name="elbow_bearing",
    )
    upper_link.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=cap_dark,
        name="shoulder_cap",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.058, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=arm_yellow,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.360, 0.058, 0.026)),
        origin=Origin(xyz=(0.180, 0.0, 0.018)),
        material=arm_yellow,
        name="fore_link",
    )
    forearm.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.360, 0.0, 0.017)),
        material=arm_yellow,
        name="tool_boss",
    )
    forearm.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=cap_dark,
        name="elbow_cap",
    )
    forearm.visual(
        Box((0.055, 0.090, 0.014)),
        origin=Origin(xyz=(0.405, 0.0, 0.017)),
        material=cap_dark,
        name="tool_pad",
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.42, 0.0, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.72),
    )
    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.6, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.480, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=0.0, upper=2.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    rail_slide = object_model.get_articulation("rail_slide")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.expect_contact(
        carriage,
        rail,
        elem_a="shoe_0",
        elem_b="wear_strip_0",
        name="one carriage shoe sits on its wear strip",
    )
    ctx.expect_contact(
        carriage,
        rail,
        elem_a="shoe_1",
        elem_b="wear_strip_1",
        name="second carriage shoe sits on its wear strip",
    )
    ctx.expect_contact(
        upper_link,
        carriage,
        elem_a="shoulder_hub",
        elem_b="shoulder_bearing",
        name="shoulder stack is seated on carriage",
    )
    ctx.expect_contact(
        forearm,
        upper_link,
        elem_a="elbow_hub",
        elem_b="elbow_bearing",
        name="elbow stack is seated on upper link",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({rail_slide: 0.72}):
        ctx.expect_overlap(
            carriage,
            rail,
            axes="xy",
            elem_a="shoe_0",
            elem_b="guide_0",
            min_overlap=0.03,
            name="extended carriage keeps one shoe engaged",
        )
        ctx.expect_overlap(
            carriage,
            rail,
            axes="xy",
            elem_a="shoe_1",
            elem_b="guide_1",
            min_overlap=0.03,
            name="extended carriage keeps second shoe engaged",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "rail slide moves carriage along the long rail",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.65,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 1.0}):
        swept_elbow_pos = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder revolute joint swings the carried elbow",
        rest_elbow_pos is not None
        and swept_elbow_pos is not None
        and swept_elbow_pos[1] > rest_elbow_pos[1] + 0.30,
        details=f"rest={rest_elbow_pos}, swept={swept_elbow_pos}",
    )

    rest_forearm_aabb = ctx.part_world_aabb(forearm)
    with ctx.pose({elbow: 1.0}):
        bent_forearm_aabb = ctx.part_world_aabb(forearm)

    def _coord(vec, idx: int) -> float:
        if hasattr(vec, "x"):
            return (vec.x, vec.y, vec.z)[idx]
        return vec[idx]

    ctx.check(
        "elbow revolute joint bends the second link",
        rest_forearm_aabb is not None
        and bent_forearm_aabb is not None
        and _coord(bent_forearm_aabb[1], 1) > _coord(rest_forearm_aabb[1], 1) + 0.20,
        details=f"rest={rest_forearm_aabb}, bent={bent_forearm_aabb}",
    )

    rail_aabb = ctx.part_world_aabb(rail)
    upper_aabb = ctx.part_world_aabb(upper_link)
    forearm_aabb = ctx.part_world_aabb(forearm)
    if rail_aabb is not None and upper_aabb is not None and forearm_aabb is not None:
        rail_len = _coord(rail_aabb[1], 0) - _coord(rail_aabb[0], 0)
        upper_len = _coord(upper_aabb[1], 0) - _coord(upper_aabb[0], 0)
        forearm_len = _coord(forearm_aabb[1], 0) - _coord(forearm_aabb[0], 0)
        ctx.check(
            "root rail stage reads larger than the carried links",
            rail_len > 1.8 * max(upper_len, forearm_len),
            details=f"rail_len={rail_len}, upper_len={upper_len}, forearm_len={forearm_len}",
        )
    else:
        ctx.fail("root rail stage reads larger than the carried links", "missing AABB")

    return ctx.report()


object_model = build_object_model()
