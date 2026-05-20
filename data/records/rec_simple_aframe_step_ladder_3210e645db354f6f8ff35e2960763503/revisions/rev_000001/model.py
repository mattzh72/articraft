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
    TestContext,
    TestReport,
)


def _member_origin_between(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[Origin, float]:
    """Origin for a box/cylinder whose local +Z length runs between two points."""
    ax, ay, az = a
    bx, by, bz = b
    dx, dy, dz = bx - ax, by - ay, bz - az
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # The ladder side rails live in XZ planes, so a Y rotation is sufficient and
    # keeps the rectangular tube faces visually flat and square.
    pitch = math.atan2(dx, dz)
    return Origin(xyz=((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5), rpy=(0.0, pitch, 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("satin_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    tread_mat = model.material("ribbed_tread", rgba=(0.18, 0.18, 0.17, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("hidden_steel", rgba=(0.55, 0.56, 0.56, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # The hinge line is the part origin.  It is tucked into the underside of the
    # top cap so the A-frame reads as a clean cap-to-cap joint rather than an
    # exposed barrel hinge.
    front.visual(
        Box((0.36, 0.72, 0.035)),
        origin=Origin(xyz=(0.025, 0.0, 0.035)),
        material=dark_plastic,
        name="top_cap_plate",
    )
    front.visual(
        Box((0.045, 0.72, 0.095)),
        origin=Origin(xyz=(0.215, 0.0, -0.012)),
        material=dark_plastic,
        name="top_cap_front_lip",
    )
    for side, y in enumerate((-0.365, 0.365)):
        front.visual(
            Box((0.34, 0.045, 0.080)),
            origin=Origin(xyz=(0.025, y, -0.010)),
            material=dark_plastic,
            name=f"top_cap_cheek_{side}",
        )

    front.visual(
        Cylinder(radius=0.010, length=0.70),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hidden_pin",
    )
    for side, y in enumerate((-0.31, 0.31)):
        front.visual(
            Box((0.160, 0.055, 0.040)),
            origin=Origin(xyz=(0.130, y, -0.053)),
            material=dark_plastic,
            name=f"top_cap_mount_{side}",
        )

    # Front climbing frame: two slightly leaning aluminum side rails carrying
    # three broad horizontal treads.
    front_top_z = -0.055
    front_foot_z = -1.165
    for side, y in enumerate((-0.31, 0.31)):
        rail_origin, rail_len = _member_origin_between(
            (0.060, y, front_top_z),
            (0.435, y, front_foot_z),
        )
        front.visual(
            Box((0.046, 0.046, rail_len)),
            origin=rail_origin,
            material=aluminum,
            name=f"front_rail_{side}",
        )
        front.visual(
            Box((0.155, 0.090, 0.035)),
            origin=Origin(xyz=(0.455, y, -1.188)),
            material=rubber,
            name=f"front_foot_{side}",
        )

    tread_specs = (
        (-0.335, 0.150, 0.245, 0.60),
        (-0.665, 0.260, 0.265, 0.64),
        (-0.995, 0.365, 0.285, 0.68),
    )
    for tread_i, (z, x, depth, width) in enumerate(tread_specs):
        front.visual(
            Box((depth, width, 0.038)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=tread_mat,
            name=f"tread_{tread_i}",
        )
        # Low anti-slip ribs are fused into each step surface.
        for rib_i, x_offset in enumerate((-0.085, -0.030, 0.030, 0.085)):
            front.visual(
                Box((0.014, width - 0.065, 0.007)),
                origin=Origin(xyz=(x + x_offset, 0.0, z + 0.020)),
                material=dark_plastic,
                name=f"tread_{tread_i}_rib_{rib_i}",
            )
        # Small side gussets make each tread visibly bolted into the front rails.
        for side, y in enumerate((-width * 0.5 + 0.025, width * 0.5 - 0.025)):
            front.visual(
                Box((0.040, 0.040, 0.075)),
                origin=Origin(xyz=(x - 0.075, y, z - 0.030)),
                material=aluminum,
                name=f"tread_{tread_i}_gusset_{side}",
            )

    # Rear support frame as the single moving link.  The visible hinge sleeve is
    # hidden in the top-cap cavity and rides on the front-frame pin.
    rear.visual(
        Cylinder(radius=0.016, length=0.64),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_sleeve",
    )
    for side, y in enumerate((-0.31, 0.31)):
        rear.visual(
            Box((0.090, 0.044, 0.036)),
            origin=Origin(xyz=(-0.045, y, -0.030)),
            material=aluminum,
            name=f"hinge_tab_{side}",
        )
        leg_origin, leg_len = _member_origin_between(
            (-0.070, y, -0.055),
            (-0.625, y, -1.150),
        )
        rear.visual(
            Box((0.046, 0.046, leg_len)),
            origin=leg_origin,
            material=aluminum,
            name=f"rear_leg_{side}",
        )
        rear.visual(
            Box((0.160, 0.090, 0.035)),
            origin=Origin(xyz=(-0.640, y, -1.176)),
            material=rubber,
            name=f"rear_foot_{side}",
        )

    for bar_i, z in enumerate((-0.545, -0.930)):
        # Match the rear leg slope so the crossbars land directly on the legs.
        t = (z - (-0.055)) / (-1.150 - (-0.055))
        x = -0.070 + t * (-0.625 - (-0.070))
        rear.visual(
            Box((0.050, 0.690, 0.040)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=f"rear_crossbar_{bar_i}",
        )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.check(
        "single top hinge",
        len(object_model.articulations) == 1,
        details=f"articulations={len(object_model.articulations)}",
    )
    ctx.check(
        "rear folds about width axis",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    ctx.allow_overlap(
        front,
        rear,
        elem_a="hidden_pin",
        elem_b="hinge_sleeve",
        reason="The concealed steel pin is intentionally captured inside the rear hinge sleeve within the top-cap cavity.",
    )
    ctx.expect_within(
        front,
        rear,
        axes="xz",
        inner_elem="hidden_pin",
        outer_elem="hinge_sleeve",
        margin=0.002,
        name="hidden pin centered in sleeve",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="y",
        elem_a="hidden_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.50,
        name="hinge sleeve spans the concealed pin",
    )

    rest_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    rest_x = None if rest_foot is None else (rest_foot[0][0] + rest_foot[1][0]) * 0.5
    with ctx.pose({hinge: 0.95}):
        folded_foot = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
        folded_x = None if folded_foot is None else (folded_foot[0][0] + folded_foot[1][0]) * 0.5
    ctx.check(
        "rear frame folds toward climbing frame",
        rest_x is not None and folded_x is not None and folded_x > rest_x + 0.85,
        details=f"rest_x={rest_x}, folded_x={folded_x}",
    )

    return ctx.report()


object_model = build_object_model()
