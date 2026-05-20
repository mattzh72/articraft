from __future__ import annotations

from math import cos, pi, sin

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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_drill_press")

    enamel = model.material("dark_blue_enamel", rgba=(0.05, 0.13, 0.22, 1.0))
    black = model.material("blackened_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    cast_iron = model.material("matte_cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    red = model.material("red_badge", rgba=(0.8, 0.06, 0.04, 1.0))

    frame = model.part("frame")

    # Flat electromagnetic base with a slightly proud steel shoe.
    frame.visual(
        Box((0.54, 0.32, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=black,
        name="magnetic_base",
    )
    frame.visual(
        Box((0.49, 0.27, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=steel,
        name="top_shoe",
    )
    frame.visual(
        Box((0.20, 0.012, 0.012)),
        origin=Origin(xyz=(0.05, -0.166, 0.050)),
        material=red,
        name="magnet_badge",
    )

    # Upright column and main quill/spindle head.
    frame.visual(
        Cylinder(radius=0.038, length=0.76),
        origin=Origin(xyz=(-0.18, 0.0, 0.45)),
        material=steel,
        name="round_column",
    )
    frame.visual(
        Box((0.27, 0.19, 0.19)),
        origin=Origin(xyz=(-0.025, 0.0, 0.64)),
        material=enamel,
        name="head_casting",
    )
    frame.visual(
        Box((0.16, 0.17, 0.13)),
        origin=Origin(xyz=(0.045, 0.0, 0.76)),
        material=enamel,
        name="motor_cap",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.18),
        origin=Origin(xyz=(0.115, 0.0, 0.52)),
        material=steel,
        name="quill_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.11),
        origin=Origin(xyz=(0.115, 0.0, 0.375)),
        material=steel,
        name="spindle_nose",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.022),
        origin=Origin(xyz=(0.115, 0.0, 0.312)),
        material=black,
        name="chuck_collet",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.032),
        origin=Origin(xyz=(0.045, -0.106, 0.64), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="feed_boss",
    )

    # Column clamp and yoke-style hinge bracket for the small tilt table.
    frame.visual(
        Box((0.105, 0.145, 0.085)),
        origin=Origin(xyz=(-0.18, 0.0, 0.265)),
        material=cast_iron,
        name="column_clamp",
    )
    frame.visual(
        Box((0.118, 0.050, 0.040)),
        origin=Origin(xyz=(-0.092, 0.0, 0.265)),
        material=cast_iron,
        name="table_arm",
    )
    frame.visual(
        Box((0.016, 0.225, 0.046)),
        origin=Origin(xyz=(-0.035, 0.0, 0.265)),
        material=cast_iron,
        name="hinge_crossbar",
    )
    for y in (-0.096, 0.096):
        frame.visual(
            Box((0.052, 0.022, 0.060)),
            origin=Origin(xyz=(-0.012, y, 0.265)),
            material=cast_iron,
            name=f"hinge_lug_{0 if y < 0 else 1}",
        )
    for y in (-0.060, 0.060):
        frame.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(-0.228, y, 0.285), rpy=(0.0, pi / 2, 0.0)),
            material=steel,
            name=f"clamp_bolt_{0 if y < 0 else 1}",
        )

    feed_wheel = model.part("feed_wheel")
    feed_wheel.visual(
        Cylinder(radius=0.017, length=0.072),
        origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="wheel_shaft",
    )
    feed_wheel.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(0.0, -0.077, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=cast_iron,
        name="hub",
    )
    feed_wheel.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.104, tube=0.006, radial_segments=14, tubular_segments=54),
            "feed_wheel_rim",
        ),
        origin=Origin(xyz=(0.0, -0.080, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=cast_iron,
        name="outer_rim",
    )
    spoke_radius = 0.105
    for i, angle in enumerate((pi / 2, pi / 2 + 2 * pi / 3, pi / 2 + 4 * pi / 3)):
        x = (spoke_radius / 2.0) * cos(angle)
        z = (spoke_radius / 2.0) * sin(angle)
        feed_wheel.visual(
            Box((spoke_radius, 0.014, 0.014)),
            origin=Origin(xyz=(x, -0.080, z), rpy=(0.0, -angle, 0.0)),
            material=cast_iron,
            name=f"spoke_{i}",
        )
        hx = spoke_radius * cos(angle)
        hz = spoke_radius * sin(angle)
        feed_wheel.visual(
            Cylinder(radius=0.014, length=0.065),
            origin=Origin(xyz=(hx, -0.096, hz), rpy=(-pi / 2, 0.0, 0.0)),
            material=rubber,
            name=f"handle_{i}",
        )
        feed_wheel.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(hx, -0.132, hz)),
            material=rubber,
            name=f"handle_knob_{i}",
        )

    table = model.part("tilt_table")
    table.visual(
        Cylinder(radius=0.018, length=0.152),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    table.visual(
        Box((0.235, 0.170, 0.026)),
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
        material=cast_iron,
        name="table_plate",
    )
    table.visual(
        Box((0.195, 0.012, 0.005)),
        origin=Origin(xyz=(0.132, -0.045, 0.0155)),
        material=black,
        name="tee_slot_0",
    )
    table.visual(
        Box((0.195, 0.012, 0.005)),
        origin=Origin(xyz=(0.132, 0.045, 0.0155)),
        material=black,
        name="tee_slot_1",
    )
    table.visual(
        Box((0.026, 0.165, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.002)),
        material=steel,
        name="rear_hinge_leaf",
    )

    model.articulation(
        "head_to_feed_wheel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=feed_wheel,
        origin=Origin(xyz=(0.045, -0.095, 0.64)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0, lower=-1.5708, upper=1.5708),
    )
    model.articulation(
        "bracket_to_table",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=table,
        origin=Origin(xyz=(-0.012, 0.0, 0.265)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.55, upper=0.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    feed_wheel = object_model.get_part("feed_wheel")
    table = object_model.get_part("tilt_table")
    feed_joint = object_model.get_articulation("head_to_feed_wheel")
    table_joint = object_model.get_articulation("bracket_to_table")

    ctx.allow_overlap(
        feed_wheel,
        frame,
        elem_a="wheel_shaft",
        elem_b="feed_boss",
        reason="The feed wheel shaft is intentionally captured inside the head-side feed boss.",
    )
    ctx.expect_gap(
        frame,
        feed_wheel,
        axis="y",
        positive_elem="feed_boss",
        negative_elem="wheel_shaft",
        max_penetration=0.040,
        name="feed shaft is only locally inserted into the boss",
    )
    ctx.expect_contact(
        feed_wheel,
        frame,
        elem_a="wheel_shaft",
        elem_b="feed_boss",
        contact_tol=0.002,
        name="feed wheel shaft is seated in the head boss",
    )
    ctx.expect_overlap(
        feed_wheel,
        frame,
        axes="xz",
        elem_a="hub",
        elem_b="head_casting",
        min_overlap=0.025,
        name="feed hub is aligned with the head side",
    )
    with ctx.pose({feed_joint: 1.0}):
        ctx.expect_overlap(
            feed_wheel,
            frame,
            axes="xz",
            elem_a="hub",
            elem_b="head_casting",
            min_overlap=0.025,
            name="feed hub stays on axis while rotating",
        )

    ctx.expect_overlap(
        table,
        frame,
        axes="y",
        elem_a="hinge_barrel",
        elem_b="hinge_crossbar",
        min_overlap=0.10,
        name="tilt table hinge spans the bracket yoke",
    )
    ctx.expect_gap(
        table,
        frame,
        axis="x",
        positive_elem="table_plate",
        negative_elem="hinge_crossbar",
        min_gap=0.010,
        max_gap=0.060,
        name="table clears the fixed hinge crossbar",
    )
    rest_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_joint: 0.60}):
        tilted_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "tilt table raises its free edge",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > rest_aabb[1][2] + 0.08,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
