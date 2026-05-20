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


def _cylinder_between(part, name, p1, p2, radius, material):
    """Add a cylindrical rail between two points in the part frame."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"{name} has zero length")

    horiz = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(horiz, dz)
    yaw = math.atan2(dy, dx) if horiz > 1.0e-9 else 0.0
    midpoint = (
        (p1[0] + p2[0]) * 0.5,
        (p1[1] + p2[1]) * 0.5,
        (p1[2] + p2[2]) * 0.5,
    )
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=midpoint, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _front_rail_x_at_height(z: float) -> float:
    # Linear interpolation along the front side rail centerline.
    bottom_x, bottom_z = 0.30, 0.07
    top_x, top_z = 0.06, 1.52
    t = (z - bottom_z) / (top_z - bottom_z)
    return bottom_x + t * (top_x - bottom_x)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_a_frame_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_step = model.material("dark_grey_tread", rgba=(0.30, 0.32, 0.33, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    yellow_plastic = model.material("safety_yellow_plastic", rgba=(1.0, 0.70, 0.10, 1.0))
    steel = model.material("hinge_steel", rgba=(0.55, 0.57, 0.58, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # Front climbing A-frame: two inclined rails, wide treads, rubber feet, and
    # the molded top cap that carries the hinge pin.
    for y, suffix in ((-0.30, "0"), (0.30, "1")):
        _cylinder_between(
            front,
            f"front_side_rail_{suffix}",
            (0.30, y, 0.07),
            (0.06, 0.84 * y, 1.52),
            0.024,
            aluminum,
        )
        front.visual(
            Box((0.14, 0.09, 0.050)),
            origin=Origin(xyz=(0.31, y, 0.035)),
            material=black_rubber,
            name=f"front_foot_{suffix}",
        )
        front.visual(
            Box((0.070, 0.060, 0.095)),
            origin=Origin(xyz=(0.30, y, 0.080)),
            material=aluminum,
            name=f"front_foot_socket_{suffix}",
        )

    _cylinder_between(front, "front_bottom_spreader", (0.30, -0.31, 0.12), (0.30, 0.31, 0.12), 0.018, aluminum)

    tread_specs = [
        (0.34, 0.31, 0.68),
        (0.66, 0.28, 0.66),
        (0.98, 0.25, 0.64),
        (1.28, 0.22, 0.62),
    ]
    for i, (z, depth, width) in enumerate(tread_specs):
        x = _front_rail_x_at_height(z) + 0.015
        front.visual(
            Box((depth, width, 0.045)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=dark_step,
            name=f"tread_{i}",
        )
        for j, offset in enumerate((-0.075, 0.0, 0.075)):
            front.visual(
                Box((0.022, width - 0.10, 0.007)),
                origin=Origin(xyz=(x + offset, 0.0, z + 0.0245)),
                material=black_rubber,
                name=f"grip_{i}_{j}",
            )

    front.visual(
        Box((0.36, 0.68, 0.09)),
        origin=Origin(xyz=(0.06, 0.0, 1.565)),
        material=yellow_plastic,
        name="top_cap",
    )
    front.visual(
        Box((0.20, 0.42, 0.010)),
        origin=Origin(xyz=(0.075, 0.0, 1.612)),
        material=black_rubber,
        name="top_cap_grip",
    )
    for y, suffix in ((-0.34, "0"), (0.34, "1")):
        front.visual(
            Box((0.085, 0.040, 0.105)),
            origin=Origin(xyz=(0.0, y, 1.495)),
            material=yellow_plastic,
            name=f"hinge_cheek_{suffix}",
        )
    front.visual(
        Cylinder(radius=0.018, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 1.48), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_hinge_pin",
    )

    # Rear support frame in the child frame. Its origin is the top hinge axis,
    # so q=0 is the tall open stance; negative rotation folds the rear frame
    # forward toward the front climbing frame.
    rear.visual(
        Cylinder(radius=0.032, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_hinge_barrel",
    )
    for y, suffix in ((-0.27, "0"), (0.27, "1")):
        rear.visual(
            Box((0.070, 0.055, 0.060)),
            origin=Origin(xyz=(-0.018, y, -0.055)),
            material=aluminum,
            name=f"rear_hinge_lug_{suffix}",
        )
        _cylinder_between(
            rear,
            f"rear_side_rail_{suffix}",
            (-0.018, y, -0.045),
            (-0.27, 1.04 * y, -1.405),
            0.022,
            aluminum,
        )
        rear.visual(
            Box((0.13, 0.085, 0.050)),
            origin=Origin(xyz=(-0.28, 1.04 * y, -1.425)),
            material=black_rubber,
            name=f"rear_foot_{suffix}",
        )

    _cylinder_between(rear, "rear_bottom_spreader", (-0.27, -0.30, -1.31), (-0.27, 0.30, -1.31), 0.017, aluminum)
    _cylinder_between(rear, "rear_mid_spreader", (-0.16, -0.285, -0.82), (-0.16, 0.285, -0.82), 0.016, aluminum)
    _cylinder_between(rear, "rear_cross_brace_0", (-0.24, -0.29, -1.22), (-0.08, 0.27, -0.35), 0.012, aluminum)
    _cylinder_between(rear, "rear_cross_brace_1", (-0.24, 0.29, -1.22), (-0.08, -0.27, -0.35), 0.012, aluminum)

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.40, upper=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("top_hinge")

    ctx.allow_overlap(
        front,
        rear,
        elem_a="front_hinge_pin",
        elem_b="rear_hinge_barrel",
        reason="The steel hinge pin is intentionally captured inside the rear hinge barrel.",
    )
    ctx.expect_within(
        front,
        rear,
        axes="xz",
        inner_elem="front_hinge_pin",
        outer_elem="rear_hinge_barrel",
        margin=0.002,
        name="hinge pin is centered in the rear barrel",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="y",
        elem_a="front_hinge_pin",
        elem_b="rear_hinge_barrel",
        min_overlap=0.55,
        name="hinge pin spans the rear barrel",
    )

    ctx.check(
        "single revolute top hinge",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )

    open_front_aabb = ctx.part_world_aabb(front)
    open_rear_aabb = ctx.part_world_aabb(rear)
    ctx.check(
        "tall narrow open stance",
        open_front_aabb is not None
        and open_rear_aabb is not None
        and open_rear_aabb[0][0] < -0.24
        and open_front_aabb[1][0] > 0.33
        and (open_front_aabb[1][0] - open_rear_aabb[0][0]) < 0.85
        and open_front_aabb[1][2] > 1.58,
        details=f"front={open_front_aabb}, rear={open_rear_aabb}",
    )

    tread_aabb = ctx.part_element_world_aabb(front, elem="tread_1")
    ctx.check(
        "wide front treads",
        tread_aabb is not None
        and (tread_aabb[1][1] - tread_aabb[0][1]) > 0.60
        and (tread_aabb[1][0] - tread_aabb[0][0]) > 0.26,
        details=f"tread_1={tread_aabb}",
    )

    with ctx.pose({hinge: -0.38}):
        folded_rear_aabb = ctx.part_world_aabb(rear)
        ctx.check(
            "rear support folds toward front frame",
            open_rear_aabb is not None
            and folded_rear_aabb is not None
            and folded_rear_aabb[1][0] > open_rear_aabb[1][0] + 0.35,
            details=f"open={open_rear_aabb}, folded={folded_rear_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
