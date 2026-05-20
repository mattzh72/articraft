from __future__ import annotations

from math import atan2, pi

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


ALUMINUM = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
DARK_PLASTIC = Material("dark_plastic", rgba=(0.03, 0.035, 0.04, 1.0))
BLACK_RUBBER = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
SAFETY_YELLOW = Material("safety_yellow", rgba=(0.95, 0.72, 0.05, 1.0))


def _bar_between(part, name, p0, p1, thickness, material):
    """Add a square tube whose local +Z runs between two points in an X/Z plane."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = (dx * dx + dy * dy + dz * dz) ** 0.5
    # All ladder rails here lie in planes of constant Y, so a pitch-only
    # alignment gives a clean rectangular aluminum extrusion.
    pitch = atan2(dx, dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    part.visual(
        Box((thickness, thickness, length)),
        origin=Origin(xyz=center, rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")
    model.materials.extend([ALUMINUM, DARK_PLASTIC, BLACK_RUBBER, SAFETY_YELLOW])

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    hinge_xyz = (0.13, 0.0, 1.38)
    rail_y = 0.30

    # Front climbing frame: two leaning side rails, broad horizontal treads,
    # anti-slip nosings, a plastic top cap, and rubber foot pads.
    _bar_between(front, "front_rail_0", (-0.46, -rail_y, 0.05), (-0.06, -rail_y, 1.34), 0.046, ALUMINUM)
    _bar_between(front, "front_rail_1", (-0.46, rail_y, 0.05), (-0.06, rail_y, 1.34), 0.046, ALUMINUM)

    tread_specs = (
        ("tread_0", -0.35, 0.38),
        ("tread_1", -0.25, 0.70),
        ("tread_2", -0.16, 1.02),
    )
    for idx, (name, x, z) in enumerate(tread_specs):
        front.visual(
            Box((0.24, 0.72, 0.045)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=ALUMINUM,
            name=name,
        )
        front.visual(
            Box((0.026, 0.66, 0.008)),
            origin=Origin(xyz=(x - 0.075, 0.0, z + 0.025)),
            material=DARK_PLASTIC,
            name=f"grip_strip_{idx}_front",
        )
        front.visual(
            Box((0.026, 0.66, 0.008)),
            origin=Origin(xyz=(x + 0.045, 0.0, z + 0.025)),
            material=DARK_PLASTIC,
            name=f"grip_strip_{idx}_rear",
        )

    front.visual(
        Box((0.42, 0.74, 0.08)),
        origin=Origin(xyz=(-0.055, 0.0, 1.38)),
        material=DARK_PLASTIC,
        name="top_cap",
    )
    front.visual(
        Box((0.08, 0.25, 0.045)),
        origin=Origin(xyz=(-0.455, -rail_y, 0.025)),
        material=BLACK_RUBBER,
        name="front_foot_0",
    )
    front.visual(
        Box((0.08, 0.25, 0.045)),
        origin=Origin(xyz=(-0.455, rail_y, 0.025)),
        material=BLACK_RUBBER,
        name="front_foot_1",
    )
    front.visual(
        Cylinder(radius=0.018, length=0.80),
        origin=Origin(xyz=hinge_xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=ALUMINUM,
        name="hinge_pin",
    )

    # Rear support frame is authored in the hinge frame.  It is intentionally
    # long and leans well back, giving the open A-frame a pronounced stance.
    _bar_between(rear, "rear_leg_0", (0.05, -rail_y, -0.06), (0.78, -rail_y, -1.34), 0.046, ALUMINUM)
    _bar_between(rear, "rear_leg_1", (0.05, rail_y, -0.06), (0.78, rail_y, -1.34), 0.046, ALUMINUM)
    rear.visual(
        Box((0.055, 0.70, 0.045)),
        origin=Origin(xyz=(0.0525, 0.0, -0.045)),
        material=ALUMINUM,
        name="rear_hinge_tube",
    )
    rear.visual(
        Box((0.04, 0.70, 0.04)),
        origin=Origin(xyz=(0.34, 0.0, -0.58)),
        material=ALUMINUM,
        name="rear_crossbar_0",
    )
    rear.visual(
        Box((0.04, 0.70, 0.04)),
        origin=Origin(xyz=(0.59, 0.0, -1.03)),
        material=ALUMINUM,
        name="rear_crossbar_1",
    )
    rear.visual(
        Box((0.11, 0.25, 0.05)),
        origin=Origin(xyz=(0.79, -rail_y, -1.36)),
        material=BLACK_RUBBER,
        name="rear_foot_0",
    )
    rear.visual(
        Box((0.11, 0.25, 0.05)),
        origin=Origin(xyz=(0.79, rail_y, -1.36)),
        material=BLACK_RUBBER,
        name="rear_foot_1",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=hinge_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=0.78),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.check(
        "single revolute top hinge",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    tread_box = ctx.part_element_world_aabb(front, elem="tread_1")
    if tread_box is not None:
        tread_min, tread_max = tread_box
        tread_width = tread_max[1] - tread_min[1]
        tread_depth = tread_max[0] - tread_min[0]
    else:
        tread_width = tread_depth = 0.0
    ctx.check(
        "front frame has wide treads",
        tread_width > 0.65 and tread_depth > 0.20,
        details=f"width={tread_width:.3f}, depth={tread_depth:.3f}",
    )

    front_foot = ctx.part_element_world_aabb(front, elem="front_foot_0")
    rear_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    if front_foot is not None and rear_foot is not None:
        open_stance = rear_foot[0][0] - front_foot[1][0]
    else:
        open_stance = 0.0
    ctx.check(
        "rear frame creates pronounced open stance",
        open_stance > 1.05,
        details=f"open_stance={open_stance:.3f}",
    )

    rest_rear_foot = rear_foot
    with ctx.pose({hinge: 0.70}):
        folded_rear_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    if rest_rear_foot is not None and folded_rear_foot is not None:
        rest_center_x = (rest_rear_foot[0][0] + rest_rear_foot[1][0]) * 0.5
        folded_center_x = (folded_rear_foot[0][0] + folded_rear_foot[1][0]) * 0.5
    else:
        rest_center_x = folded_center_x = 0.0
    ctx.check(
        "rear frame folds toward front frame",
        folded_center_x < rest_center_x - 0.60,
        details=f"rest_x={rest_center_x:.3f}, folded_x={folded_center_x:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
