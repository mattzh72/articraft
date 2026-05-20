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


LINK_LENGTH = 0.12
LINK_WIDTH = 0.034
LINK_THICKNESS = 0.010
LINK_RADIUS = 0.024
LINK_LAYER_STEP = 0.016
LINK_Z0 = 0.006
PIN_RADIUS = 0.006
PIN_CLEARANCE_GAP = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacking_support_foldout_arm")

    dark_steel = model.material("dark_burnished_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    rubbed_edges = model.material("rubbed_steel_edges", rgba=(0.46, 0.48, 0.48, 1.0))
    pin_steel = model.material("brushed_pin_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    platform_mat = model.material("plain_black_platform", rgba=(0.025, 0.026, 0.024, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.034, 0.180, 0.245)),
        origin=Origin(xyz=(-0.045, 0.0, 0.020)),
        material=dark_steel,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.043, 0.074, 0.028)),
        origin=Origin(xyz=(-0.006, 0.0, LINK_Z0 - 0.018)),
        material=dark_steel,
        name="hinge_standoff",
    )
    side_plate.visual(
        Cylinder(radius=LINK_RADIUS + 0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, LINK_Z0 - 0.010)),
        material=rubbed_edges,
        name="base_thrust_washer",
    )
    side_plate.visual(
        Cylinder(radius=PIN_RADIUS, length=LINK_Z0 + LINK_THICKNESS + 0.008),
        origin=Origin(xyz=(0.0, 0.0, (LINK_Z0 + LINK_THICKNESS + 0.008) / 2.0 - 0.004)),
        material=pin_steel,
        name="base_pin",
    )
    # Low-profile screw heads make the grounded bracket read as a bolted side plate.
    for idx, y in enumerate((-0.056, 0.056)):
        for jdx, z in enumerate((-0.065, 0.105)):
            side_plate.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(-0.026, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=pin_steel,
                name=f"screw_head_{idx}_{jdx}",
            )

    def add_short_link(part_name: str, layer_index: int, *, has_distal_pin: bool, has_platform: bool = False):
        part = model.part(part_name)
        z = LINK_Z0 + layer_index * LINK_LAYER_STEP
        part.visual(
            Box((LINK_LENGTH - 2.0 * LINK_RADIUS, LINK_WIDTH, LINK_THICKNESS)),
            origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, z)),
            material=dark_steel,
            name="link_web",
        )
        part.visual(
            Cylinder(radius=LINK_RADIUS, length=LINK_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name="proximal_boss",
        )
        part.visual(
            Cylinder(radius=LINK_RADIUS, length=LINK_THICKNESS),
            origin=Origin(xyz=(LINK_LENGTH, 0.0, z)),
            material=dark_steel,
            name="distal_boss",
        )
        part.visual(
            Cylinder(radius=LINK_RADIUS * 0.48, length=LINK_THICKNESS + 0.002),
            origin=Origin(xyz=(LINK_LENGTH, 0.0, z + 0.001)),
            material=rubbed_edges,
            name="distal_wear_ring",
        )
        if has_distal_pin:
            pin_length = LINK_LAYER_STEP + LINK_THICKNESS + PIN_CLEARANCE_GAP
            part.visual(
                Cylinder(radius=PIN_RADIUS, length=pin_length),
                origin=Origin(xyz=(LINK_LENGTH, 0.0, z + pin_length / 2.0 - PIN_CLEARANCE_GAP / 2.0)),
                material=pin_steel,
                name="distal_pin",
            )
        if has_platform:
            platform_center_x = LINK_LENGTH + 0.031
            part.visual(
                Box((0.014, 0.112, 0.070)),
                origin=Origin(xyz=(platform_center_x, 0.0, z + 0.003)),
                material=platform_mat,
                name="platform_face",
            )
            part.visual(
                Box((0.034, 0.044, LINK_THICKNESS)),
                origin=Origin(xyz=(LINK_LENGTH + 0.016, 0.0, z)),
                material=dark_steel,
                name="platform_neck",
            )
        return part

    link_0 = add_short_link("link_0", 0, has_distal_pin=True)
    link_1 = add_short_link("link_1", 1, has_distal_pin=True)
    link_2 = add_short_link("link_2", 2, has_distal_pin=True)
    link_3 = add_short_link("link_3", 3, has_distal_pin=False, has_platform=True)

    limits = MotionLimits(effort=18.0, velocity=2.4, lower=-2.60, upper=2.60)
    model.articulation(
        "side_to_link_0",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")

    side_to_link_0 = object_model.get_articulation("side_to_link_0")
    link_0_to_link_1 = object_model.get_articulation("link_0_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")

    ctx.allow_overlap(
        side_plate,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_boss",
        reason="The grounded hinge pin is intentionally captured inside the first link bushing.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The distal hinge pin is intentionally captured inside the next link bushing.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The distal hinge pin is intentionally captured inside the next link bushing.",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        reason="The distal hinge pin is intentionally captured inside the next link bushing.",
    )

    revolute_chain = (side_to_link_0, link_0_to_link_1, link_1_to_link_2, link_2_to_link_3)
    ctx.check(
        "four serial revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in revolute_chain)
        and [j.parent for j in revolute_chain] == ["side_plate", "link_0", "link_1", "link_2"]
        and [j.child for j in revolute_chain] == ["link_0", "link_1", "link_2", "link_3"],
        details="The arm should be a four-joint serial chain from the grounded plate to the platform link.",
    )
    ctx.check(
        "planar vertical hinge axes",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in revolute_chain),
        details="All four joints should rotate about the same vertical axis direction.",
    )

    ctx.expect_contact(
        side_plate,
        link_0,
        elem_a="base_pin",
        elem_b="proximal_boss",
        contact_tol=0.0,
        name="base pin is captured by first link",
    )
    ctx.expect_contact(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        contact_tol=0.0,
        name="first interlink hinge is pinned",
    )
    ctx.expect_contact(
        link_1,
        link_2,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        contact_tol=0.0,
        name="second interlink hinge is pinned",
    )
    ctx.expect_contact(
        link_2,
        link_3,
        elem_a="distal_pin",
        elem_b="proximal_boss",
        contact_tol=0.0,
        name="third interlink hinge is pinned",
    )
    ctx.expect_gap(
        link_3,
        side_plate,
        axis="x",
        min_gap=0.34,
        positive_elem="platform_face",
        negative_elem="wall_plate",
        name="deployed platform projects far from side plate",
    )

    with ctx.pose(
        {
            side_to_link_0: 0.0,
            link_0_to_link_1: 2.6,
            link_1_to_link_2: -2.6,
            link_2_to_link_3: 2.6,
        }
    ):
        ctx.expect_gap(
            link_3,
            side_plate,
            axis="x",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="platform_face",
            negative_elem="wall_plate",
            name="folded platform clears side plate",
        )
        platform_aabb = ctx.part_element_world_aabb(link_3, elem="platform_face")
        wall_aabb = ctx.part_element_world_aabb(side_plate, elem="wall_plate")
        folded_ok = False
        details = f"platform_aabb={platform_aabb}, wall_aabb={wall_aabb}"
        if platform_aabb is not None and wall_aabb is not None:
            platform_center_x = (platform_aabb[0][0] + platform_aabb[1][0]) / 2.0
            wall_front_x = wall_aabb[1][0]
            folded_ok = abs(platform_center_x - wall_front_x) < 0.060
            details = f"platform_center_x={platform_center_x:.3f}, wall_front_x={wall_front_x:.3f}"
        ctx.check(
            "folded pose nests platform close to side plate",
            folded_ok,
            details=details,
        )

    return ctx.report()


object_model = build_object_model()
