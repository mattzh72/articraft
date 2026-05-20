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
    Sphere,
    TestContext,
    TestReport,
)


Vec3 = tuple[float, float, float]


def _between(a: Vec3, b: Vec3) -> tuple[Vec3, float, Origin]:
    ax, ay, az = a
    bx, by, bz = b
    vx, vy, vz = bx - ax, by - ay, bz - az
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("zero length member")
    ux, uy, uz = vx / length, vy / length, vz / length
    radial = math.sqrt(ux * ux + uy * uy)
    yaw = math.atan2(uy, ux) if radial > 1e-9 else 0.0
    pitch = math.atan2(radial, uz)
    center = ((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5)
    return center, length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _cylinder_between(part, a: Vec3, b: Vec3, radius: float, *, material, name: str) -> None:
    _, length, origin = _between(a, b)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _box_between(
    part,
    a: Vec3,
    b: Vec3,
    size_xy: tuple[float, float],
    *,
    material,
    name: str,
) -> None:
    _, length, origin = _between(a, b)
    part.visual(Box((size_xy[0], size_xy[1], length)), origin=origin, material=material, name=name)


def _x_cylinder(part, center: Vec3, length: float, radius: float, *, material, name: str) -> None:
    x, y, z = center
    _cylinder_between(
        part,
        (x - length * 0.5, y, z),
        (x + length * 0.5, y, z),
        radius,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchard_tripod_picking_ladder")

    weathered_aluminum = model.material("weathered_aluminum", rgba=(0.78, 0.78, 0.72, 1.0))
    bright_aluminum = model.material("bright_rung_edges", rgba=(0.90, 0.88, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    safety_orange = model.material("safety_orange", rgba=(1.0, 0.38, 0.06, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    # Root: the rigid front section, with two splayed orchard-ladder stiles.
    front = model.part("front_frame")

    left_base = (-0.52, 0.0, 0.08)
    right_base = (0.52, 0.0, 0.08)
    left_top = (-0.17, 0.0, 2.35)
    right_top = (0.17, 0.0, 2.35)

    _cylinder_between(front, left_base, left_top, 0.032, material=weathered_aluminum, name="stile_0")
    _cylinder_between(front, right_base, right_top, 0.032, material=weathered_aluminum, name="stile_1")

    base_z = left_base[2]
    top_z = left_top[2]
    for idx, z in enumerate((0.38, 0.68, 0.98, 1.28, 1.58, 1.88, 2.16)):
        t = (z - base_z) / (top_z - base_z)
        x_left = left_base[0] + (left_top[0] - left_base[0]) * t
        x_right = right_base[0] + (right_top[0] - right_base[0]) * t
        _cylinder_between(
            front,
            (x_left - 0.018, 0.0, z),
            (x_right + 0.018, 0.0, z),
            0.024,
            material=bright_aluminum,
            name=f"rung_{idx}",
        )

    _cylinder_between(front, left_top, right_top, 0.030, material=weathered_aluminum, name="top_crossbar")
    front.visual(Sphere(radius=0.048), origin=Origin(xyz=left_base), material=black_rubber, name="foot_0")
    front.visual(Sphere(radius=0.048), origin=Origin(xyz=right_base), material=black_rubber, name="foot_1")

    hinge_world = (0.0, -0.08, 2.36)
    # Exposed top yoke and pin for the pivoting central rear leg.
    front.visual(
        Box((0.026, 0.125, 0.095)),
        origin=Origin(xyz=(-0.19, -0.045, 2.35)),
        material=dark_steel,
        name="hinge_yoke_0",
    )
    front.visual(
        Box((0.026, 0.125, 0.095)),
        origin=Origin(xyz=(0.19, -0.045, 2.35)),
        material=dark_steel,
        name="hinge_yoke_1",
    )
    _x_cylinder(front, (-0.22, hinge_world[1], hinge_world[2]), 0.16, 0.045, material=dark_steel, name="front_hinge_knuckle_0")
    _x_cylinder(front, (0.22, hinge_world[1], hinge_world[2]), 0.16, 0.045, material=dark_steel, name="front_hinge_knuckle_1")
    _x_cylinder(front, hinge_world, 0.68, 0.016, material=dark_steel, name="front_hinge_pin")

    # A clevis on the middle rung receives the folding lock strut.
    lock_world = (0.0, -0.03, 1.05)
    for side, x in enumerate((-0.065, 0.065)):
        front.visual(
            Box((0.026, 0.040, 0.145)),
            origin=Origin(xyz=(x, -0.003, 1.025)),
            material=dark_steel,
            name=f"lock_strap_{side}",
        )
        front.visual(
            Box((0.026, 0.055, 0.105)),
            origin=Origin(xyz=(x, -0.03, 1.05)),
            material=dark_steel,
            name=f"lock_ear_{side}",
        )
    _x_cylinder(front, lock_world, 0.30, 0.014, material=dark_steel, name="lock_pin")

    # Child: a single rear leg that swings from the top hinge.
    rear = model.part("rear_leg")
    rear_foot = (0.0, -1.15, -2.31)
    _x_cylinder(rear, (0.0, 0.0, 0.0), 0.23, 0.042, material=dark_steel, name="rear_hinge_barrel")
    _cylinder_between(
        rear,
        (0.0, -0.025, -0.045),
        rear_foot,
        0.034,
        material=weathered_aluminum,
        name="rear_stile",
    )
    _cylinder_between(
        rear,
        (0.0, -0.035, -0.035),
        (0.0, -0.030, -0.120),
        0.020,
        material=dark_steel,
        name="rear_head_socket",
    )
    rear.visual(Sphere(radius=0.050), origin=Origin(xyz=rear_foot), material=black_rubber, name="rear_foot")

    # The brace pivot is mounted on a side lug offset from the rear leg tube
    # rather than through the tube centerline.
    brace_attach = (0.0, -0.69, -1.11)
    for side, x in enumerate((-0.055, 0.055)):
        rear.visual(
            Box((0.045, 0.070, 0.090)),
            origin=Origin(xyz=(x, brace_attach[1], brace_attach[2])),
            material=dark_steel,
            name=f"brace_lug_{side}",
        )
    _cylinder_between(
        rear,
        (0.0, -0.555, -1.110),
        brace_attach,
        0.020,
        material=dark_steel,
        name="brace_mount_web",
    )
    _x_cylinder(rear, brace_attach, 0.30, 0.014, material=dark_steel, name="brace_pivot_pin")

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=hinge_world),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.3, lower=-1.05, upper=0.0),
    )

    # Child of the rear leg: a folding brace strut. At q=0 its front eye is
    # captured by the root lock pin; rotating it upward represents unlocking.
    brace = model.part("brace_strut")
    brace_end = (
        lock_world[0] - hinge_world[0] - brace_attach[0],
        lock_world[1] - hinge_world[1] - brace_attach[1],
        lock_world[2] - hinge_world[2] - brace_attach[2],
    )
    brace_x = 0.11
    _box_between(
        brace,
        (brace_x, 0.0, 0.0),
        (brace_x, brace_end[1], brace_end[2]),
        (0.040, 0.016),
        material=dark_steel,
        name="brace_bar",
    )
    _x_cylinder(brace, (brace_x, 0.0, 0.0), 0.035, 0.040, material=dark_steel, name="rear_brace_eye")
    _x_cylinder(
        brace,
        (brace_x, brace_end[1], brace_end[2]),
        0.035,
        0.040,
        material=dark_steel,
        name="front_brace_eye",
    )
    brace.visual(
        Box((0.060, 0.018, 0.050)),
        origin=Origin(xyz=(brace_x, brace_end[1] - 0.030, brace_end[2])),
        material=safety_orange,
        name="lock_tab",
    )

    model.articulation(
        "brace_hinge",
        ArticulationType.REVOLUTE,
        parent=rear,
        child=brace,
        origin=Origin(xyz=brace_attach),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_leg")
    brace = object_model.get_part("brace_strut")
    rear_hinge = object_model.get_articulation("rear_hinge")
    brace_hinge = object_model.get_articulation("brace_hinge")

    ctx.allow_overlap(
        front,
        rear,
        elem_a="front_hinge_pin",
        elem_b="rear_hinge_barrel",
        reason="The hinge pin is intentionally captured inside the rear leg's top barrel.",
    )
    ctx.allow_overlap(
        rear,
        brace,
        elem_a="brace_pivot_pin",
        elem_b="rear_brace_eye",
        reason="The brace pivot pin is intentionally captured through the rear eye of the folding strut.",
    )
    ctx.allow_overlap(
        rear,
        brace,
        elem_a="brace_pivot_pin",
        elem_b="brace_bar",
        reason="The flat brace strap is simplified without a drilled pivot hole, so the pin locally passes through the strap end.",
    )
    ctx.allow_overlap(
        front,
        brace,
        elem_a="lock_pin",
        elem_b="front_brace_eye",
        reason="The deployed brace eye is intentionally retained on the front lock pin.",
    )
    ctx.allow_overlap(
        front,
        brace,
        elem_a="lock_pin",
        elem_b="brace_bar",
        reason="The flat locking strap is simplified without a drilled front hole, so the lock pin locally passes through the strap end.",
    )

    ctx.expect_within(
        front,
        rear,
        axes="yz",
        inner_elem="front_hinge_pin",
        outer_elem="rear_hinge_barrel",
        margin=0.001,
        name="top hinge pin sits inside rear barrel",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="x",
        elem_a="front_hinge_pin",
        elem_b="rear_hinge_barrel",
        min_overlap=0.18,
        name="top hinge pin spans the rear barrel",
    )
    ctx.expect_within(
        rear,
        brace,
        axes="yz",
        inner_elem="brace_pivot_pin",
        outer_elem="rear_brace_eye",
        margin=0.001,
        name="brace pivot pin sits inside rear eye",
    )
    ctx.expect_overlap(
        rear,
        brace,
        axes="yz",
        elem_a="brace_pivot_pin",
        elem_b="brace_bar",
        min_overlap=0.010,
        name="brace bar end is centered on rear pivot",
    )
    ctx.expect_within(
        front,
        brace,
        axes="yz",
        inner_elem="lock_pin",
        outer_elem="front_brace_eye",
        margin=0.001,
        name="brace front eye is seated on lock pin",
    )
    ctx.expect_overlap(
        front,
        brace,
        axes="yz",
        elem_a="lock_pin",
        elem_b="brace_bar",
        min_overlap=0.010,
        name="brace bar end is centered on front lock pin",
    )

    ctx.check(
        "front section has two stiles and seven rungs",
        len([v for v in front.visuals if v.name.startswith("stile_")]) == 2
        and len([v for v in front.visuals if v.name.startswith("rung_")]) == 7,
        details="The orchard ladder front must read as two wide stiles joined by rungs.",
    )
    ctx.check(
        "rear leg and brace are revolute",
        rear_hinge.articulation_type == ArticulationType.REVOLUTE
        and brace_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"rear={rear_hinge.articulation_type}, brace={brace_hinge.articulation_type}",
    )

    rest_foot = ctx.part_element_world_aabb(rear, elem="rear_foot")
    with ctx.pose({rear_hinge: -0.90}):
        folded_foot = ctx.part_element_world_aabb(rear, elem="rear_foot")
    ctx.check(
        "rear leg folds upward from deployed pose",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[0][2] > rest_foot[0][2] + 0.55,
        details=f"rest rear foot aabb={rest_foot}, folded rear foot aabb={folded_foot}",
    )

    rest_eye = ctx.part_element_world_aabb(brace, elem="front_brace_eye")
    with ctx.pose({brace_hinge: 1.10}):
        raised_eye = ctx.part_element_world_aabb(brace, elem="front_brace_eye")
    rest_eye_z = (rest_eye[0][2] + rest_eye[1][2]) * 0.5 if rest_eye else None
    raised_eye_z = (raised_eye[0][2] + raised_eye[1][2]) * 0.5 if raised_eye else None
    ctx.check(
        "brace strut swings on its hinge",
        rest_eye_z is not None and raised_eye_z is not None and raised_eye_z > rest_eye_z + 0.25,
        details=f"rest_eye_z={rest_eye_z}, raised_eye_z={raised_eye_z}",
    )

    return ctx.report()


object_model = build_object_model()
