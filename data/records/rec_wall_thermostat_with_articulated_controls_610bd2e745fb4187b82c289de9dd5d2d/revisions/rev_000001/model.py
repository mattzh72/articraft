from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BODY_WIDTH = 0.128
BODY_DEPTH = 0.024
FACE_WIDTH = 0.118
FACE_DEPTH = 0.004
SEAT_RADIUS = 0.052
SEAT_DEPTH = 0.0025
DIAL_DIAMETER = 0.094
DIAL_HEIGHT = 0.0115
DIAL_MOUNT_Z = 0.027


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    housing_white = model.material("housing_white", rgba=(0.94, 0.94, 0.92, 1.0))
    face_white = model.material("face_white", rgba=(0.97, 0.97, 0.96, 1.0))
    dial_graphite = model.material("dial_graphite", rgba=(0.27, 0.29, 0.31, 1.0))
    pointer_white = model.material("pointer_white", rgba=(0.96, 0.97, 0.98, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_WIDTH, BODY_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, BODY_DEPTH / 2.0)),
        material=housing_white,
        name="housing_shell",
    )
    body.visual(
        Box((FACE_WIDTH, FACE_WIDTH, FACE_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, BODY_DEPTH - 0.001)),
        material=face_white,
        name="face_plate",
    )
    body.visual(
        Cylinder(radius=SEAT_RADIUS, length=SEAT_DEPTH),
        origin=Origin(xyz=(0.0, 0.0, BODY_DEPTH + 0.00175)),
        material=face_white,
        name="dial_seat",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_WIDTH, BODY_DEPTH + FACE_DEPTH)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_DIAMETER / 2.0, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=dial_graphite,
        name="dial_shell",
    )
    dial.visual(
        Cylinder(radius=0.0415, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=dial_graphite,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.022, 0.0012)),
        origin=Origin(xyz=(0.0, 0.018, 0.0108)),
        material=pointer_white,
        name="pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_DIAMETER / 2.0, length=DIAL_HEIGHT),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, DIAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, DIAL_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    body_aabb = ctx.part_world_aabb(body)
    dial_aabb = ctx.part_world_aabb(dial)

    ctx.check("body_aabb_present", body_aabb is not None, "Expected a body AABB.")
    ctx.check("dial_aabb_present", dial_aabb is not None, "Expected a dial AABB.")

    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "body_wall_control_scale",
            0.115 <= size[0] <= 0.135 and 0.115 <= size[1] <= 0.135 and 0.025 <= size[2] <= 0.029,
            details=f"body_size={size!r}",
        )

    if dial_aabb is not None:
        mins, maxs = dial_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "dial_large_central_scale",
            0.088 <= max(size[0], size[1]) <= 0.100 and 0.010 <= size[2] <= 0.014,
            details=f"dial_size={size!r}",
        )

    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=1e-6,
        name="dial axis centered in body",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_shell",
        elem_b="housing_shell",
        min_overlap=0.090,
        name="dial stays within square body footprint",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_shell",
        negative_elem="dial_seat",
        max_gap=0.0002,
        max_penetration=1e-5,
        name="dial seats on center ring",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.8}):
        turned_pos = ctx.part_world_position(dial)

    ctx.check(
        "dial_rotates_about_fixed_center",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(turned_pos[i] - rest_pos[i]) <= 1e-9 for i in range(3)),
        details=f"rest={rest_pos!r}, turned={turned_pos!r}",
    )

    return ctx.report()


object_model = build_object_model()
