from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


LEG_TUBE_RADIUS = 0.010
LEG_LENGTH = 0.355
LEG_REST_ANGLE = math.pi - 0.58
LEG_HUB_RADIUS = 0.076


def _add_tripod_leg(
    model: ArticulatedObject,
    crown,
    *,
    index: int,
    angle: float,
    metal,
    rubber,
):
    leg = model.part(f"leg_{index}")

    leg.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    leg.visual(
        Cylinder(radius=LEG_TUBE_RADIUS, length=LEG_LENGTH),
        origin=Origin(
            xyz=(
                0.5 * LEG_LENGTH * math.sin(LEG_REST_ANGLE),
                0.0,
                0.5 * LEG_LENGTH * math.cos(LEG_REST_ANGLE),
            ),
            rpy=(0.0, LEG_REST_ANGLE, 0.0),
        ),
        material=metal,
        name="tube",
    )
    leg.visual(
        Sphere(radius=0.014),
        origin=Origin(
            xyz=(
                LEG_LENGTH * math.sin(LEG_REST_ANGLE),
                0.0,
                LEG_LENGTH * math.cos(LEG_REST_ANGLE),
            )
        ),
        material=rubber,
        name="foot",
    )

    model.articulation(
        f"crown_to_leg_{index}",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=leg,
        origin=Origin(
            xyz=(
                LEG_HUB_RADIUS * math.cos(angle),
                LEG_HUB_RADIUS * math.sin(angle),
                -0.052,
            ),
            rpy=(0.0, 0.0, angle),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.60,
        ),
    )

    return leg


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sensor_pod_tripod")

    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    black = model.material("black", rgba=(0.10, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.27, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.34, 0.48, 0.54, 0.55))
    latch_finish = model.material("latch_finish", rgba=(0.78, 0.80, 0.82, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=graphite,
        name="top_collar",
    )
    crown.visual(
        Cylinder(radius=0.050, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=dark_metal,
        name="hub",
    )
    crown.visual(
        Cylinder(radius=0.017, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, -0.133)),
        material=dark_metal,
        name="column",
    )
    crown.visual(
        Box((0.030, 0.030, 0.016)),
        origin=Origin(xyz=(0.030, 0.0, -0.045)),
        material=graphite,
        name="pan_lock",
    )
    crown.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.047, 0.0, -0.045)),
        material=rubber,
        name="pan_knob",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        crown.visual(
            Box((0.022, 0.028, 0.018)),
            origin=Origin(
                xyz=(0.056 * math.cos(angle), 0.056 * math.sin(angle), -0.052),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"hinge_mount_{index}",
        )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        _add_tripod_leg(
            model,
            crown,
            index=index,
            angle=angle,
            metal=dark_metal,
            rubber=rubber,
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=graphite,
        name="pan_plate",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="pan_riser",
    )
    head.visual(
        Box((0.042, 0.136, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.044)),
        material=dark_metal,
        name="yoke_bridge",
    )
    head.visual(
        Box((0.030, 0.136, 0.040)),
        origin=Origin(xyz=(-0.018, 0.0, 0.065)),
        material=graphite,
        name="rear_brace",
    )
    for side in (-1.0, 1.0):
        head.visual(
            Box((0.058, 0.012, 0.104)),
            origin=Origin(xyz=(0.020, side * 0.068, 0.086)),
            material=dark_metal,
            name=f"cheek_{'neg' if side < 0.0 else 'pos'}",
        )

    model.articulation(
        "crown_to_head",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=-2.4,
            upper=2.4,
        ),
    )

    body = model.part("body")
    body.visual(
        Box((0.168, 0.102, 0.122)),
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        material=black,
        name="housing",
    )
    body.visual(
        Box((0.050, 0.094, 0.086)),
        origin=Origin(xyz=(-0.020, 0.0, -0.004)),
        material=graphite,
        name="rear_cap",
    )
    body.visual(
        Box((0.046, 0.074, 0.028)),
        origin=Origin(xyz=(0.060, 0.0, 0.073)),
        material=dark_metal,
        name="top_module",
    )
    for side in (-1.0, 1.0):
        body.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(
                xyz=(0.0, side * 0.056, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"trunnion_{'neg' if side < 0.0 else 'pos'}",
        )
    body.visual(
        Box((0.012, 0.096, 0.106)),
        origin=Origin(xyz=(0.174, 0.0, 0.0)),
        material=graphite,
        name="bezel",
    )
    body.visual(
        Box((0.004, 0.074, 0.084)),
        origin=Origin(xyz=(0.172, 0.0, 0.0)),
        material=glass,
        name="filter_window",
    )
    body.visual(
        Box((0.012, 0.012, 0.104)),
        origin=Origin(xyz=(0.174, 0.048, 0.0)),
        material=dark_metal,
        name="door_jamb",
    )

    model.articulation(
        "head_to_body",
        ArticulationType.REVOLUTE,
        parent=head,
        child=body,
        origin=Origin(xyz=(0.042, 0.0, 0.102)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.3,
            lower=-0.55,
            upper=0.85,
        ),
    )

    door = model.part("door")
    door.visual(
        Box((0.004, 0.088, 0.098)),
        origin=Origin(xyz=(0.002, -0.044, 0.0)),
        material=graphite,
        name="panel",
    )
    door.visual(
        Box((0.008, 0.016, 0.028)),
        origin=Origin(xyz=(0.006, -0.082, 0.0)),
        material=latch_finish,
        name="latch",
    )
    for index, z in enumerate((-0.032, 0.0, 0.032)):
        door.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(xyz=(0.004, 0.0, z)),
            material=dark_metal,
            name=f"knuckle_{index}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.180, 0.044, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    leg_0 = object_model.get_part("leg_0")

    pan = object_model.get_articulation("crown_to_head")
    tilt = object_model.get_articulation("head_to_body")
    leg_fold = object_model.get_articulation("crown_to_leg_0")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="panel",
        negative_elem="bezel",
        min_gap=0.0,
        max_gap=0.0015,
        name="filter door closes flush to the bezel",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        elem_a="panel",
        elem_b="filter_window",
        min_overlap=0.074,
        name="closed filter door covers the window",
    )

    crown_aabb = ctx.part_world_aabb(crown)
    for index in range(3):
        leg_aabb = ctx.part_world_aabb(object_model.get_part(f"leg_{index}"))
        ctx.check(
            f"leg_{index} reaches well below the crown",
            crown_aabb is not None
            and leg_aabb is not None
            and leg_aabb[0][2] < crown_aabb[1][2] - 0.30,
            details=f"crown={crown_aabb}, leg={leg_aabb}",
        )

    rest_body_pos = ctx.part_world_position(body)
    with ctx.pose({pan: 0.9}):
        panned_body_pos = ctx.part_world_position(body)
    ctx.check(
        "pan moves the sensor body around the mast",
        rest_body_pos is not None
        and panned_body_pos is not None
        and panned_body_pos[1] > rest_body_pos[1] + 0.02,
        details=f"rest={rest_body_pos}, panned={panned_body_pos}",
    )

    rest_body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({tilt: 0.55}):
        tilted_body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "tilt lifts the front of the body",
        rest_body_aabb is not None
        and tilted_body_aabb is not None
        and tilted_body_aabb[1][2] > rest_body_aabb[1][2] + 0.03,
        details=f"rest={rest_body_aabb}, tilted={tilted_body_aabb}",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.2}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "filter door swings outward from the housing front",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > rest_door_aabb[1][0] + 0.05,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_leg_aabb = ctx.part_world_aabb(leg_0)
    with ctx.pose({leg_fold: 0.60}):
        folded_leg_aabb = ctx.part_world_aabb(leg_0)
    ctx.check(
        "front leg folds inward toward the center column",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[1][0] < rest_leg_aabb[1][0] - 0.03,
        details=f"rest={rest_leg_aabb}, folded={folded_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
