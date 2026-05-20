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


BASE_WIDTH = 0.330
BASE_DEPTH = 0.238
BASE_HEIGHT = 0.055
BODY_WIDTH = 0.130
BODY_DEPTH = 0.105
BODY_HEIGHT = 0.840
BODY_WALL = 0.0045
BODY_BOTTOM = 0.010
BODY_TOP = 0.014
GRILLE_WIDTH = 0.088
GRILLE_HEIGHT = 0.770
GRILLE_BOTTOM = 0.040
BLOWER_RADIUS = 0.036
BLOWER_HEIGHT = 0.720
DIAL_DIAMETER = 0.034
DIAL_HEIGHT = 0.019
DIAL_BOSS_HEIGHT = 0.0035
CORNER_RADIUS = 0.012
DIAL_BOSS_NAMES = ("dial_boss_0", "dial_boss_1")


def _add_grille(part, material) -> None:
    frame_depth = 0.004
    side_frame = 0.006
    top_bottom_frame = 0.008
    clear_width = GRILLE_WIDTH - 2.0 * side_frame
    clear_height = GRILLE_HEIGHT - 2.0 * top_bottom_frame

    part.visual(
        Box((GRILLE_WIDTH, frame_depth, top_bottom_frame)),
        origin=Origin(xyz=(0.0, 0.0, top_bottom_frame * 0.5)),
        material=material,
        name="grille_bottom_bar",
    )
    part.visual(
        Box((GRILLE_WIDTH, frame_depth, top_bottom_frame)),
        origin=Origin(xyz=(0.0, 0.0, GRILLE_HEIGHT - top_bottom_frame * 0.5)),
        material=material,
        name="grille_top_bar",
    )
    for suffix, x_pos in (("left", -GRILLE_WIDTH * 0.5 + side_frame * 0.5), ("right", GRILLE_WIDTH * 0.5 - side_frame * 0.5)):
        part.visual(
            Box((side_frame, frame_depth, GRILLE_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, GRILLE_HEIGHT * 0.5)),
            material=material,
            name=f"grille_{suffix}_rail",
        )

    slat_count = 13
    slat_width = 0.0024
    if slat_count > 1:
        pitch = clear_width / (slat_count - 1)
        start_x = -clear_width * 0.5
        for index in range(slat_count):
            x_pos = start_x + pitch * index
            part.visual(
                Box((slat_width, 0.003, clear_height)),
                origin=Origin(xyz=(x_pos, 0.0, top_bottom_frame + clear_height * 0.5)),
                material=material,
                name=f"slat_{index}",
            )


def _add_blower_wheel(part, material) -> None:
    ring_thickness = 0.010
    blade_height = BLOWER_HEIGHT - 2.0 * ring_thickness

    part.visual(
        Cylinder(radius=0.013, length=BLOWER_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_HEIGHT * 0.5)),
        material=material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=BLOWER_RADIUS, length=ring_thickness),
        origin=Origin(xyz=(0.0, 0.0, ring_thickness * 0.5)),
        material=material,
        name="lower_ring",
    )
    part.visual(
        Cylinder(radius=BLOWER_RADIUS, length=ring_thickness),
        origin=Origin(xyz=(0.0, 0.0, BLOWER_HEIGHT - ring_thickness * 0.5)),
        material=material,
        name="upper_ring",
    )

    blade_count = 18
    blade_radius = 0.024
    blade_radial = 0.022
    blade_tangential = 0.0022
    for index in range(blade_count):
        angle = 2.0 * math.pi * index / blade_count
        part.visual(
            Box((blade_radial, blade_tangential, blade_height)),
            origin=Origin(
                xyz=(blade_radius * math.cos(angle), blade_radius * math.sin(angle), BLOWER_HEIGHT * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"blade_{index}",
        )


def _add_knurled_dial(part, material) -> None:
    dial_radius = DIAL_DIAMETER * 0.5
    part.visual(
        Cylinder(radius=dial_radius * 0.93, length=DIAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DIAL_HEIGHT * 0.5)),
        material=material,
        name="dial",
    )
    part.visual(
        Cylinder(radius=dial_radius, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=material,
        name="dial_skirt",
    )
    part.visual(
        Box((0.002, 0.010, 0.0015)),
        origin=Origin(xyz=(0.0, dial_radius * 0.55, DIAL_HEIGHT - 0.00075)),
        material=material,
        name="dial_marker",
    )

    ridge_count = 20
    ridge_radius = dial_radius * 0.88
    ridge_height = DIAL_HEIGHT * 0.78
    for index in range(ridge_count):
        angle = 2.0 * math.pi * index / ridge_count
        part.visual(
            Box((0.0036, 0.0011, ridge_height)),
            origin=Origin(
                xyz=(ridge_radius * math.cos(angle), ridge_radius * math.sin(angle), ridge_height * 0.5 + 0.001),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"ridge_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    base_plastic = model.material("base_plastic", rgba=(0.93, 0.94, 0.95, 1.0))
    body_plastic = model.material("body_plastic", rgba=(0.84, 0.86, 0.89, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.72, 0.75, 0.78, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.58, 0.60, 0.63, 1.0))
    blower_finish = model.material("blower_finish", rgba=(0.20, 0.22, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.108, 0.208, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=base_plastic,
        name="base_center",
    )
    for index, x_pos in enumerate((-0.054, 0.054)):
        base.visual(
            Cylinder(radius=0.104, length=0.012),
            origin=Origin(xyz=(x_pos, 0.0, 0.006)),
            material=base_plastic,
            name=f"base_end_{index}",
        )
    base.visual(
        Box((0.170, 0.140, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=base_plastic,
        name="base_upper",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=base_plastic,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=base_plastic,
        name="turntable",
    )
    base.visual(
        Box((0.070, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=base_plastic,
        name="mount_pad",
    )

    body = model.part("body")
    corner_x = BODY_WIDTH * 0.5 - CORNER_RADIUS
    corner_y = BODY_DEPTH * 0.5 - CORNER_RADIUS
    for index, (x_pos, y_pos) in enumerate(
        (
            (-corner_x, -corner_y),
            (-corner_x, corner_y),
            (corner_x, -corner_y),
            (corner_x, corner_y),
        )
    ):
        body.visual(
            Cylinder(radius=CORNER_RADIUS, length=BODY_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, BODY_HEIGHT * 0.5)),
            material=body_plastic,
            name=f"corner_post_{index}",
        )
    body.visual(
        Box((BODY_WIDTH - 2.0 * CORNER_RADIUS, BODY_DEPTH - 2.0 * CORNER_RADIUS, BODY_BOTTOM)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM * 0.5)),
        material=body_plastic,
        name="bottom_plate",
    )
    body.visual(
        Box((BODY_WALL, BODY_DEPTH - 2.0 * CORNER_RADIUS, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH * 0.5 + BODY_WALL * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_DEPTH - 2.0 * CORNER_RADIUS, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 - BODY_WALL * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=body_plastic,
        name="right_wall",
    )
    rear_wall_height = BODY_HEIGHT - 0.040
    body.visual(
        Box((BODY_WIDTH - 2.0 * CORNER_RADIUS, BODY_WALL, rear_wall_height)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + BODY_WALL * 0.5, rear_wall_height * 0.5)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * CORNER_RADIUS, BODY_WALL, 0.040)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - BODY_WALL * 0.5, 0.020)),
        material=body_plastic,
        name="front_bottom_bezel",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * CORNER_RADIUS, BODY_WALL, 0.030)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - BODY_WALL * 0.5, BODY_HEIGHT - 0.015)),
        material=body_plastic,
        name="front_top_bezel",
    )
    body.visual(
        Box((BODY_WIDTH, 0.040, BODY_TOP)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 - 0.020, BODY_HEIGHT - BODY_TOP * 0.5)),
        material=body_plastic,
        name="top_front_cap",
    )
    for index, x_pos in enumerate((-0.054, 0.054)):
        body.visual(
            Box((0.014, 0.080, BODY_TOP)),
            origin=Origin(xyz=(x_pos, -0.0125, BODY_HEIGHT - BODY_TOP * 0.5)),
            material=body_plastic,
            name=f"top_rail_{index}",
        )
    body.visual(
        Box((0.094, BODY_WALL, BODY_TOP)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + BODY_WALL * 0.5, BODY_HEIGHT - BODY_TOP * 0.5)),
        material=body_plastic,
        name="handle_bridge",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=body_plastic,
        name="lower_bearing",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=body_plastic,
        name="upper_bearing",
    )
    for index, x_pos in enumerate((-0.030, 0.030)):
        body.visual(
            Cylinder(radius=0.019, length=DIAL_BOSS_HEIGHT),
            origin=Origin(xyz=(x_pos, 0.012, BODY_HEIGHT + DIAL_BOSS_HEIGHT * 0.5)),
            material=body_plastic,
            name=DIAL_BOSS_NAMES[index],
        )

    grille = model.part("grille")
    _add_grille(grille, grille_finish)

    blower_wheel = model.part("blower_wheel")
    _add_blower_wheel(blower_wheel, blower_finish)

    for index, x_pos in enumerate((-0.030, 0.030)):
        dial = model.part(f"dial_{index}")
        _add_knurled_dial(dial, dial_finish)
        model.articulation(
            f"body_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=dial,
            origin=Origin(xyz=(x_pos, 0.012, BODY_HEIGHT + DIAL_BOSS_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.15, velocity=8.0),
        )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-0.78,
            upper=0.78,
        ),
    )

    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(
            xyz=(0.0, BODY_DEPTH * 0.5 - BODY_WALL - 0.001, GRILLE_BOTTOM)
        ),
    )

    model.articulation(
        "body_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    grille = object_model.get_part("grille")
    blower_wheel = object_model.get_part("blower_wheel")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")

    body_joint = object_model.get_articulation("base_to_body")
    blower_joint = object_model.get_articulation("body_to_blower_wheel")
    dial_joint_0 = object_model.get_articulation("body_to_dial_0")
    dial_joint_1 = object_model.get_articulation("body_to_dial_1")

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="bottom_plate",
        negative_elem="mount_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="body sits on the base turntable height",
    )
    ctx.expect_within(
        grille,
        body,
        axes="xz",
        margin=0.004,
        name="front grille stays within the tower silhouette",
    )
    ctx.expect_within(
        blower_wheel,
        body,
        axes="xy",
        margin=0.006,
        name="blower wheel stays centered inside the tower body",
    )
    ctx.expect_overlap(
        blower_wheel,
        body,
        axes="z",
        min_overlap=0.650,
        name="blower wheel spans most of the tower height",
    )
    ctx.expect_gap(
        dial_0,
        body,
        axis="z",
        positive_elem="dial_skirt",
        negative_elem="dial_boss_0",
        max_gap=0.002,
        max_penetration=1e-5,
        name="dial_0 mounts onto the top cap",
    )
    ctx.expect_gap(
        dial_1,
        body,
        axis="z",
        positive_elem="dial_skirt",
        negative_elem="dial_boss_1",
        max_gap=0.002,
        max_penetration=1e-5,
        name="dial_1 mounts onto the top cap",
    )

    base_aabb = ctx.part_world_aabb(base)
    body_aabb = ctx.part_world_aabb(body)
    total_height = None
    if base_aabb is not None and body_aabb is not None:
        total_height = float(max(base_aabb[1][2], body_aabb[1][2]) - min(base_aabb[0][2], body_aabb[0][2]))
    ctx.check(
        "tower fan is home appliance scale",
        total_height is not None and 0.85 <= total_height <= 1.05,
        details=f"total_height={total_height}",
    )

    ctx.check(
        "oscillation joint is bounded left-right rotation",
        body_joint.articulation_type == ArticulationType.REVOLUTE
        and body_joint.motion_limits is not None
        and body_joint.motion_limits.lower is not None
        and body_joint.motion_limits.upper is not None
        and body_joint.motion_limits.lower < 0.0 < body_joint.motion_limits.upper,
        details=f"joint_type={body_joint.articulation_type}, limits={body_joint.motion_limits}",
    )
    ctx.check(
        "blower wheel spins continuously",
        blower_joint.articulation_type == ArticulationType.CONTINUOUS
        and blower_joint.motion_limits is not None
        and blower_joint.motion_limits.lower is None
        and blower_joint.motion_limits.upper is None,
        details=f"joint_type={blower_joint.articulation_type}, limits={blower_joint.motion_limits}",
    )
    ctx.check(
        "both top dials rotate continuously",
        dial_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint_0.motion_limits is not None
        and dial_joint_1.motion_limits is not None
        and dial_joint_0.motion_limits.lower is None
        and dial_joint_1.motion_limits.lower is None
        and dial_joint_0.motion_limits.upper is None
        and dial_joint_1.motion_limits.upper is None,
        details=(
            f"dial_0={dial_joint_0.articulation_type}/{dial_joint_0.motion_limits}, "
            f"dial_1={dial_joint_1.articulation_type}/{dial_joint_1.motion_limits}"
        ),
    )

    lower = body_joint.motion_limits.lower if body_joint.motion_limits is not None else None
    upper = body_joint.motion_limits.upper if body_joint.motion_limits is not None else None
    lower_pos = None
    upper_pos = None
    if lower is not None and upper is not None:
        with ctx.pose({body_joint: lower}):
            lower_pos = ctx.part_world_position(dial_1)
        with ctx.pose({body_joint: upper}):
            upper_pos = ctx.part_world_position(dial_1)
    ctx.check(
        "oscillation swings the top cap laterally",
        lower_pos is not None
        and upper_pos is not None
        and upper_pos[1] > lower_pos[1] + 0.03,
        details=f"lower_pos={lower_pos}, upper_pos={upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
