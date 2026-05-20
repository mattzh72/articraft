from __future__ import annotations

from math import cos, pi, sin

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


WHEEL_CENTER_Z = 0.40
WHEEL_RADIUS = 0.245
BUCKET_COUNT = 12


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_overshot_demo_mill")

    weathered_wood = model.material("weathered_oak", rgba=(0.52, 0.34, 0.18, 1.0))
    pale_wood = model.material("pale_bucket_wood", rgba=(0.76, 0.58, 0.34, 1.0))
    dark_metal = model.material("dark_oiled_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    dull_galvanized = model.material("dull_galvanized", rgba=(0.58, 0.61, 0.61, 1.0))
    guard_glass = model.material("clear_polycarbonate", rgba=(0.60, 0.80, 1.0, 0.33))
    water_blue = model.material("shallow_water", rgba=(0.12, 0.36, 0.80, 0.70))

    frame = model.part("frame")
    frame.visual(Box((0.56, 0.72, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=weathered_wood, name="base_plank")

    for x, name in ((-0.18, "side_post_0"), (0.18, "side_post_1")):
        frame.visual(Box((0.035, 0.060, 0.700)), origin=Origin(xyz=(x, 0.0, 0.390)), material=weathered_wood, name=name)

    frame.visual(Box((0.43, 0.050, 0.036)), origin=Origin(xyz=(0.0, 0.0, 0.735)), material=weathered_wood, name="top_crossbar")
    frame.visual(Box((0.47, 0.040, 0.032)), origin=Origin(xyz=(0.0, 0.18, 0.056)), material=weathered_wood, name="rear_tie_beam")

    for x, name in ((-0.16, "bearing_block_0"), (0.16, "bearing_block_1")):
        frame.visual(Box((0.040, 0.085, 0.075)), origin=Origin(xyz=(x, 0.0, WHEEL_CENTER_Z)), material=dark_metal, name=name)

    frame.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(-0.146, 0.0, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dull_galvanized,
        name="bearing_face_0",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.146, 0.0, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dull_galvanized,
        name="bearing_face_1",
    )

    # A small open wooden feed trough perched over the overshot point.
    for x, name in ((-0.11, "feed_support_0"), (0.11, "feed_support_1")):
        frame.visual(Box((0.026, 0.31, 0.026)), origin=Origin(xyz=(x, 0.155, 0.735)), material=weathered_wood, name=f"{name}_strut")
        frame.visual(Box((0.026, 0.026, 0.095)), origin=Origin(xyz=(x, 0.300, 0.778)), material=weathered_wood, name=name)
    frame.visual(Box((0.21, 0.36, 0.018)), origin=Origin(xyz=(0.0, 0.300, 0.818)), material=weathered_wood, name="trough_bottom")
    frame.visual(Box((0.018, 0.36, 0.060)), origin=Origin(xyz=(-0.105, 0.300, 0.844)), material=weathered_wood, name="trough_side_0")
    frame.visual(Box((0.018, 0.36, 0.060)), origin=Origin(xyz=(0.105, 0.300, 0.844)), material=weathered_wood, name="trough_side_1")
    frame.visual(Box((0.17, 0.28, 0.006)), origin=Origin(xyz=(0.0, 0.275, 0.827)), material=water_blue, name="water_in_trough")
    frame.visual(Box((0.12, 0.050, 0.012)), origin=Origin(xyz=(0.0, 0.120, 0.813)), material=dull_galvanized, name="feed_lip")

    # A fixed hinge stile and exposed pin for the swing-away upper guard.
    frame.visual(Box((0.024, 0.030, 0.770)), origin=Origin(xyz=(-0.253, -0.330, 0.425)), material=weathered_wood, name="guard_hinge_stile")
    frame.visual(
        Cylinder(radius=0.006, length=0.330),
        origin=Origin(xyz=(-0.240, -0.330, 0.675)),
        material=dark_metal,
        name="guard_hinge_pin",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.048, length=0.110),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.280),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle",
    )

    rim_radius = WHEEL_RADIUS - 0.020
    for i in range(16):
        theta = 2.0 * pi * i / 16
        wheel.visual(
            Box((0.126, 0.096, 0.026)),
            origin=Origin(
                xyz=(0.0, rim_radius * sin(theta), rim_radius * cos(theta)),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=weathered_wood,
            name=f"rim_segment_{i}",
        )

    for i in range(8):
        theta = 2.0 * pi * i / 8
        wheel.visual(
            Box((0.095, 0.020, 0.205)),
            origin=Origin(
                xyz=(0.0, 0.127 * sin(theta), 0.127 * cos(theta)),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=weathered_wood,
            name=f"spoke_{i}",
        )

    bucket_radius = WHEEL_RADIUS + 0.013
    for i in range(BUCKET_COUNT):
        theta = 2.0 * pi * i / BUCKET_COUNT
        wheel.visual(
            Box((0.128, 0.092, 0.056)),
            origin=Origin(
                xyz=(0.0, bucket_radius * sin(theta), bucket_radius * cos(theta)),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=pale_wood,
            name=f"bucket_{i}",
        )

    guard = model.part("guard")
    guard.visual(Box((0.386, 0.010, 0.260)), origin=Origin(xyz=(0.230, 0.0, 0.0)), material=guard_glass, name="clear_panel")
    guard.visual(Box((0.030, 0.018, 0.300)), origin=Origin(xyz=(0.021, 0.0, 0.0)), material=dull_galvanized, name="hinge_leaf")
    guard.visual(Box((0.030, 0.018, 0.300)), origin=Origin(xyz=(0.423, 0.0, 0.0)), material=dull_galvanized, name="outer_frame")
    guard.visual(Box((0.430, 0.018, 0.024)), origin=Origin(xyz=(0.230, 0.0, 0.138)), material=dull_galvanized, name="top_frame")
    guard.visual(Box((0.430, 0.018, 0.024)), origin=Origin(xyz=(0.230, 0.0, -0.138)), material=dull_galvanized, name="bottom_frame")
    guard.visual(
        Box((0.480, 0.012, 0.012)),
        origin=Origin(xyz=(0.230, 0.0, 0.0), rpy=(0.0, -0.62, 0.0)),
        material=dull_galvanized,
        name="diagonal_brace",
    )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=guard,
        origin=Origin(xyz=(-0.240, -0.330, 0.675)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    guard = object_model.get_part("guard")
    axle_spin = object_model.get_articulation("axle_spin")
    guard_hinge = object_model.get_articulation("guard_hinge")

    ctx.expect_contact(wheel, frame, elem_a="axle", elem_b="bearing_face_0", contact_tol=0.001, name="axle seated in left bearing")
    ctx.expect_contact(wheel, frame, elem_a="axle", elem_b="bearing_face_1", contact_tol=0.001, name="axle seated in right bearing")

    with ctx.pose({axle_spin: pi / 2.0}):
        bucket_box = ctx.part_element_world_aabb(wheel, elem="bucket_0")
        if bucket_box is None:
            ctx.fail("bucket rotates with wheel", "bucket_0 AABB was unavailable")
        else:
            bucket_min, bucket_max = bucket_box
            bucket_center_y = 0.5 * (bucket_min[1] + bucket_max[1])
            bucket_center_z = 0.5 * (bucket_min[2] + bucket_max[2])
            ctx.check(
                "bucket rotates with wheel",
                bucket_center_y < -0.18 and 0.34 <= bucket_center_z <= 0.46,
                details=f"bucket_center_y={bucket_center_y:.3f}, bucket_center_z={bucket_center_z:.3f}",
            )

    with ctx.pose({guard_hinge: 0.0}):
        ctx.expect_gap(wheel, guard, axis="y", min_gap=0.020, name="closed guard clears wheel")

    closed_panel_box = ctx.part_element_world_aabb(guard, elem="clear_panel")
    with ctx.pose({guard_hinge: 1.35}):
        open_panel_box = ctx.part_element_world_aabb(guard, elem="clear_panel")
    if closed_panel_box is None or open_panel_box is None:
        ctx.fail("guard swings outward", f"closed={closed_panel_box}, open={open_panel_box}")
    else:
        closed_min, _ = closed_panel_box
        open_min, _ = open_panel_box
        ctx.check(
            "guard swings outward",
            open_min[1] < closed_min[1] - 0.18,
            details=f"closed_min_y={closed_min[1]:.3f}, open_min_y={open_min[1]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
