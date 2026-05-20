from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_laptop_stand")

    aluminum = Material("satin_anodized_aluminum", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_aluminum = Material("dark_vented_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base_len = 0.260
    base_width = 0.220
    base_thick = 0.010
    pivot_x = -0.075
    pivot_z = 0.034

    lower_len = 0.130
    upper_len = 0.130
    lower_pitch = -math.radians(63.0)
    upper_world_pitch = -math.radians(126.0)
    tray_world_pitch = math.radians(15.0)
    tray_len = 0.285
    tray_width = 0.235

    base = model.part("base")
    base.visual(
        Box((base_len, base_width, base_thick)),
        origin=Origin(xyz=(0.0, 0.0, base_thick / 2.0)),
        material=aluminum,
        name="base_plate",
    )
    for idx, y in enumerate((-0.101, 0.101)):
        base.visual(
            Box((0.020, 0.016, 0.040)),
            origin=Origin(xyz=(pivot_x, y, base_thick + 0.020)),
            material=aluminum,
            name=f"pivot_cheek_{idx}",
        )
    base.visual(
        Cylinder(radius=0.0032, length=0.220),
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="base_pivot_pin",
    )
    for idx, (x, y) in enumerate(
        (
            (-0.100, -0.080),
            (-0.100, 0.080),
            (0.095, -0.080),
            (0.095, 0.080),
        )
    ):
        base.visual(
            Box((0.030, 0.022, 0.004)),
            origin=Origin(xyz=(x, y, -0.0015)),
            material=black_rubber,
            name=f"foot_{idx}",
        )

    lower_arm = model.part("lower_arm")
    for idx, y in enumerate((-0.083, 0.083)):
        lower_arm.visual(
            Box((lower_len, 0.012, 0.010)),
            origin=Origin(xyz=(lower_len / 2.0, y, 0.0)),
            material=aluminum,
            name=f"rail_{idx}",
        )
    lower_arm.visual(
        Cylinder(radius=0.008, length=0.164),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="pivot_sleeve",
    )
    lower_arm.visual(
        Cylinder(radius=0.0032, length=0.182),
        origin=Origin(xyz=(lower_len, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="elbow_pin",
    )

    upper_arm = model.part("upper_arm")
    for idx, y in enumerate((-0.056, 0.056)):
        upper_arm.visual(
            Box((upper_len - 0.008, 0.010, 0.009)),
            origin=Origin(xyz=(0.008 + (upper_len - 0.008) / 2.0, y, 0.0)),
            material=aluminum,
            name=f"rail_{idx}",
        )
    upper_arm.visual(
        Cylinder(radius=0.008, length=0.138),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="elbow_sleeve",
    )
    upper_arm.visual(
        Cylinder(radius=0.0032, length=0.255),
        origin=Origin(xyz=(upper_len, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="top_hinge_pin",
    )

    tray = model.part("tray")
    tray_panel = SlotPatternPanelGeometry(
        (tray_len, tray_width),
        0.004,
        slot_size=(0.044, 0.006),
        pitch=(0.058, 0.020),
        frame=0.018,
        corner_radius=0.008,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(tray_panel, "vented_top_tray"),
        origin=Origin(xyz=(tray_len / 2.0, 0.0, 0.010)),
        material=dark_aluminum,
        name="vented_panel",
    )
    tray.visual(
        Box((tray_len, 0.010, 0.003)),
        origin=Origin(xyz=(tray_len / 2.0, -tray_width / 2.0 + 0.012, 0.013)),
        material=black_rubber,
        name="pad_0",
    )
    tray.visual(
        Box((tray_len, 0.010, 0.003)),
        origin=Origin(xyz=(tray_len / 2.0, tray_width / 2.0 - 0.012, 0.013)),
        material=black_rubber,
        name="pad_1",
    )
    tray.visual(
        Box((0.012, tray_width * 0.86, 0.026)),
        origin=Origin(xyz=(tray_len - 0.006, 0.0, 0.024)),
        material=aluminum,
        name="front_lip",
    )
    tray.visual(
        Box((0.032, 0.020, 0.034)),
        origin=Origin(xyz=(tray_len - 0.012, -tray_width / 2.0 + 0.028, 0.028)),
        material=aluminum,
        name="lip_tab_0",
    )
    tray.visual(
        Box((0.032, 0.020, 0.034)),
        origin=Origin(xyz=(tray_len - 0.012, tray_width / 2.0 - 0.028, 0.028)),
        material=aluminum,
        name="lip_tab_1",
    )
    for idx, y in enumerate((-0.100, 0.100)):
        tray.visual(
            Box((0.052, 0.040, 0.006)),
            origin=Origin(xyz=(0.024, y, 0.0103)),
            material=aluminum,
            name=f"rear_hinge_leaf_{idx}",
        )
    for idx, y in enumerate((-0.100, 0.100)):
        tray.visual(
            Cylinder(radius=0.0075, length=0.038),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"hinge_barrel_{idx}",
        )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z), rpy=(0.0, lower_pitch, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.45, upper=0.50),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(
            xyz=(lower_len, 0.0, 0.0),
            rpy=(0.0, upper_world_pitch - lower_pitch, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.45, upper=0.60),
    )
    model.articulation(
        "upper_arm_to_tray",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tray,
        origin=Origin(
            xyz=(upper_len, 0.0, 0.0),
            rpy=(0.0, tray_world_pitch - upper_world_pitch, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.25, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    tray = object_model.get_part("tray")
    base_joint = object_model.get_articulation("base_to_lower_arm")
    elbow_joint = object_model.get_articulation("lower_arm_to_upper_arm")
    tray_joint = object_model.get_articulation("upper_arm_to_tray")

    ctx.allow_overlap(
        base,
        lower_arm,
        elem_a="base_pivot_pin",
        elem_b="pivot_sleeve",
        reason="The base hinge pin is intentionally captured inside the lower-arm sleeve.",
    )
    ctx.expect_within(
        base,
        lower_arm,
        axes="xz",
        inner_elem="base_pivot_pin",
        outer_elem="pivot_sleeve",
        margin=0.001,
        name="base pin sits inside lower sleeve",
    )
    ctx.expect_overlap(
        base,
        lower_arm,
        axes="y",
        elem_a="base_pivot_pin",
        elem_b="pivot_sleeve",
        min_overlap=0.150,
        name="base hinge has retained pin length",
    )

    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        elem_a="elbow_pin",
        elem_b="elbow_sleeve",
        reason="The elbow pin is intentionally represented inside the upper-arm hinge sleeve.",
    )
    ctx.expect_within(
        lower_arm,
        upper_arm,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_sleeve",
        margin=0.001,
        name="elbow pin sits inside upper sleeve",
    )
    ctx.expect_overlap(
        lower_arm,
        upper_arm,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_sleeve",
        min_overlap=0.120,
        name="elbow hinge has retained pin length",
    )

    for barrel_name in ("hinge_barrel_0", "hinge_barrel_1"):
        ctx.allow_overlap(
            tray,
            upper_arm,
            elem_a=barrel_name,
            elem_b="top_hinge_pin",
            reason="The tray hinge barrel is intentionally captured around the top support-arm pin.",
        )
        ctx.expect_within(
            upper_arm,
            tray,
            axes="xz",
            inner_elem="top_hinge_pin",
            outer_elem=barrel_name,
            margin=0.0015,
            name=f"top pin sits inside {barrel_name}",
        )
        ctx.expect_overlap(
            tray,
            upper_arm,
            axes="y",
            elem_a=barrel_name,
            elem_b="top_hinge_pin",
            min_overlap=0.030,
            name=f"{barrel_name} retains top pin",
        )

    ctx.check(
        "stand has three revolute adjustments",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (base_joint, elbow_joint, tray_joint)),
        details="The lower link, upper link, and tray hinge must all rotate.",
    )

    upper_rest = ctx.part_world_position(upper_arm)
    with ctx.pose({base_joint: 0.32}):
        upper_moved = ctx.part_world_position(upper_arm)
    ctx.check(
        "lower arm joint moves elbow",
        upper_rest is not None
        and upper_moved is not None
        and abs(upper_moved[2] - upper_rest[2]) > 0.020,
        details=f"rest={upper_rest}, moved={upper_moved}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({elbow_joint: 0.34}):
        tray_moved = ctx.part_world_position(tray)
    ctx.check(
        "upper arm joint moves tray hinge",
        tray_rest is not None
        and tray_moved is not None
        and abs(tray_moved[0] - tray_rest[0]) + abs(tray_moved[2] - tray_rest[2]) > 0.025,
        details=f"rest={tray_rest}, moved={tray_moved}",
    )

    lip_rest = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_joint: 0.32}):
        lip_tilted = ctx.part_element_world_aabb(tray, elem="front_lip")
    ctx.check(
        "tray hinge tilts retaining lip",
        lip_rest is not None
        and lip_tilted is not None
        and lip_tilted[0][2] < lip_rest[0][2] - 0.015,
        details=f"rest={lip_rest}, tilted={lip_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
