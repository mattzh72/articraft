from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rectangular_lens_hood() -> MeshGeometry:
    """A hollow flared rectangular hood, oriented along the camera +X axis."""
    x_back = 0.300
    x_front = 0.410
    outer_back = (0.142, 0.104)   # width in Y, height in Z
    inner_back = (0.118, 0.082)
    outer_front = (0.205, 0.150)
    inner_front = (0.176, 0.124)

    def loop(x: float, width: float, height: float) -> list[tuple[float, float, float]]:
        hy = width / 2.0
        hz = height / 2.0
        return [
            (x, -hy, -hz),
            (x, hy, -hz),
            (x, hy, hz),
            (x, -hy, hz),
        ]

    loops = [
        loop(x_back, *outer_back),
        loop(x_front, *outer_front),
        loop(x_back, *inner_back),
        loop(x_front, *inner_front),
    ]

    geom = MeshGeometry()
    indices: list[list[int]] = []
    for corners in loops:
        indices.append([geom.add_vertex(*p) for p in corners])

    outer_back_i, outer_front_i, inner_back_i, inner_front_i = indices

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(4):
        j = (i + 1) % 4
        # Outside wall of the flared hood.
        quad(outer_back_i[i], outer_back_i[j], outer_front_i[j], outer_front_i[i])
        # Inside wall, wound opposite the outside surface.
        quad(inner_back_i[j], inner_back_i[i], inner_front_i[i], inner_front_i[j])
        # Rear and front rim surfaces close the wall thickness.
        quad(outer_back_i[j], outer_back_i[i], inner_back_i[i], inner_back_i[j])
        quad(outer_front_i[i], outer_front_i[j], inner_front_i[j], inner_front_i[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="documentary_camcorder")

    body_mat = model.material("charcoal_body", rgba=(0.035, 0.037, 0.040, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    dark_mat = model.material("dark_graphite", rgba=(0.11, 0.115, 0.12, 1.0))
    glass_mat = model.material("smoked_glass", rgba=(0.05, 0.13, 0.22, 0.92))
    lens_mat = model.material("deep_lens_glass", rgba=(0.0, 0.02, 0.045, 0.94))
    marking_mat = model.material("white_markings", rgba=(0.82, 0.84, 0.78, 1.0))

    body = model.part("body")

    body_shell = cq.Workplane("XY").box(0.360, 0.170, 0.200).edges().fillet(0.012)
    body.visual(
        mesh_from_cadquery(body_shell, "rounded_body", tolerance=0.0015),
        material=body_mat,
        name="rounded_body",
    )

    # Shoulder/hand strap side bulge and a rugged top carry handle.
    body.visual(
        Box((0.210, 0.026, 0.118)),
        origin=Origin(xyz=(-0.020, 0.098, -0.005)),
        material=rubber_mat,
        name="side_grip",
    )
    body.visual(
        Box((0.052, 0.062, 0.100)),
        origin=Origin(xyz=(-0.105, 0.000, 0.145)),
        material=dark_mat,
        name="rear_handle_post",
    )
    body.visual(
        Box((0.052, 0.062, 0.100)),
        origin=Origin(xyz=(0.095, 0.000, 0.145)),
        material=dark_mat,
        name="front_handle_post",
    )
    body.visual(
        Box((0.265, 0.060, 0.045)),
        origin=Origin(xyz=(-0.005, 0.000, 0.205)),
        material=dark_mat,
        name="top_handle",
    )
    body.visual(
        Box((0.180, 0.040, 0.012)),
        origin=Origin(xyz=(-0.006, 0.000, 0.176)),
        material=rubber_mat,
        name="handle_grip_pad",
    )

    # Lens train: fixed barrel, rotating control ring, glass, and broad hood.
    body.visual(
        Cylinder(radius=0.050, length=0.180),
        origin=Origin(xyz=(0.235, 0.000, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.416, 0.000, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_mat,
        name="front_glass",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.100),
        origin=Origin(xyz=(0.365, 0.000, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_mat,
        name="inner_lens_tube",
    )
    body.visual(
        mesh_from_geometry(_rectangular_lens_hood(), "wide_lens_hood"),
        material=rubber_mat,
        name="wide_lens_hood",
    )
    body.visual(
        Box((0.018, 0.032, 0.014)),
        origin=Origin(xyz=(-0.095, -0.087, 0.104)),
        material=marking_mat,
        name="record_button",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.045, -0.045, 0.105)),
        material=marking_mat,
        name="top_dial",
    )

    # Exposed side hinge yoke for the flip-out monitor arm.
    body.visual(
        Box((0.034, 0.034, 0.022)),
        origin=Origin(xyz=(0.080, -0.098, 0.043)),
        material=dark_mat,
        name="upper_side_yoke",
    )
    body.visual(
        Box((0.034, 0.034, 0.022)),
        origin=Origin(xyz=(0.080, -0.098, -0.043)),
        material=dark_mat,
        name="lower_side_yoke",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.102),
        origin=Origin(xyz=(0.080, -0.118, 0.000)),
        material=marking_mat,
        name="body_side_pin",
    )

    control_ring = model.part("control_ring")
    ring_geom = KnobGeometry(
        0.122,
        0.046,
        body_style="cylindrical",
        edge_radius=0.0015,
        grip=KnobGrip(style="knurled", count=56, depth=0.0020, helix_angle_deg=24.0),
        bore=KnobBore(style="round", diameter=0.101),
    )
    control_ring.visual(
        mesh_from_geometry(ring_geom, "knurled_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_mat,
        name="knurled_ring",
    )
    for pad_name, pad_origin, pad_size in (
        ("upper_bearing_pad", (0.000, 0.000, 0.0555), (0.032, 0.010, 0.011)),
        ("lower_bearing_pad", (0.000, 0.000, -0.0555), (0.032, 0.010, 0.011)),
        ("front_bearing_pad", (0.000, -0.0555, 0.000), (0.032, 0.011, 0.010)),
        ("rear_bearing_pad", (0.000, 0.0555, 0.000), (0.032, 0.011, 0.010)),
    ):
        control_ring.visual(
            Box(pad_size),
            origin=Origin(xyz=pad_origin),
            material=rubber_mat,
            name=pad_name,
        )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.0105, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=dark_mat,
        name="body_knuckle",
    )
    hinge_arm.visual(
        Box((0.108, 0.012, 0.018)),
        origin=Origin(xyz=(-0.054, 0.000, 0.066)),
        material=dark_mat,
        name="narrow_arm",
    )
    hinge_arm.visual(
        Box((0.012, 0.012, 0.072)),
        origin=Origin(xyz=(-0.014, 0.000, 0.036)),
        material=dark_mat,
        name="body_riser",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0050, length=0.112),
        origin=Origin(xyz=(-0.112, 0.000, 0.000)),
        material=marking_mat,
        name="screen_pin",
    )
    hinge_arm.visual(
        Box((0.016, 0.018, 0.024)),
        origin=Origin(xyz=(-0.106, 0.000, 0.066)),
        material=dark_mat,
        name="upper_screen_fork",
    )
    hinge_arm.visual(
        Box((0.016, 0.018, 0.024)),
        origin=Origin(xyz=(-0.106, 0.000, -0.066)),
        material=dark_mat,
        name="lower_screen_fork",
    )

    screen = model.part("viewing_screen")
    screen.visual(
        Cylinder(radius=0.0115, length=0.104),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=dark_mat,
        name="screen_knuckle",
    )
    screen.visual(
        Box((0.156, 0.014, 0.104)),
        origin=Origin(xyz=(-0.078, -0.001, 0.000)),
        material=dark_mat,
        name="screen_housing",
    )
    screen.visual(
        Box((0.124, 0.003, 0.074)),
        origin=Origin(xyz=(-0.088, -0.009, 0.000)),
        material=glass_mat,
        name="lcd_glass",
    )
    screen.visual(
        Box((0.022, 0.004, 0.010)),
        origin=Origin(xyz=(-0.145, -0.011, -0.038)),
        material=marking_mat,
        name="screen_button",
    )

    model.articulation(
        "body_to_control_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=control_ring,
        origin=Origin(xyz=(0.232, 0.000, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    model.articulation(
        "body_to_hinge_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hinge_arm,
        origin=Origin(xyz=(0.080, -0.118, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "hinge_arm_to_screen",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=screen,
        origin=Origin(xyz=(-0.112, 0.000, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.6, lower=0.0, upper=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("control_ring")
    arm = object_model.get_part("hinge_arm")
    screen = object_model.get_part("viewing_screen")
    ring_joint = object_model.get_articulation("body_to_control_ring")
    arm_joint = object_model.get_articulation("body_to_hinge_arm")
    screen_joint = object_model.get_articulation("hinge_arm_to_screen")

    ctx.allow_overlap(
        body,
        arm,
        elem_a="body_side_pin",
        elem_b="body_knuckle",
        reason="The fixed hinge pin is intentionally captured inside the arm knuckle.",
    )
    ctx.expect_overlap(
        body,
        arm,
        axes="z",
        elem_a="body_side_pin",
        elem_b="body_knuckle",
        min_overlap=0.035,
        name="body hinge pin spans arm knuckle",
    )
    ctx.allow_overlap(
        arm,
        screen,
        elem_a="screen_pin",
        elem_b="screen_knuckle",
        reason="The screen hinge pin is intentionally captured inside the display knuckle.",
    )
    ctx.expect_overlap(
        arm,
        screen,
        axes="z",
        elem_a="screen_pin",
        elem_b="screen_knuckle",
        min_overlap=0.080,
        name="screen hinge pin spans display knuckle",
    )

    ctx.expect_gap(
        body,
        screen,
        axis="y",
        positive_elem="rounded_body",
        negative_elem="screen_housing",
        min_gap=0.006,
        max_gap=0.050,
        name="closed screen rides just outside body side",
    )
    ctx.expect_overlap(
        ring,
        body,
        axes="x",
        elem_a="knurled_ring",
        elem_b="lens_barrel",
        min_overlap=0.035,
        name="control ring is seated around lens barrel length",
    )
    ctx.check(
        "three camera mechanisms are articulated",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS
        and arm_joint.articulation_type == ArticulationType.REVOLUTE
        and screen_joint.articulation_type == ArticulationType.REVOLUTE,
        details="Expected continuous lens ring plus two revolute screen hinges.",
    )

    rest_screen_pos = ctx.part_world_position(screen)
    with ctx.pose({arm_joint: 1.10, screen_joint: 1.10}):
        opened_screen_pos = ctx.part_world_position(screen)
        ctx.expect_gap(
            body,
            screen,
            axis="y",
            positive_elem="rounded_body",
            negative_elem="screen_housing",
            min_gap=0.075,
            name="opened screen clears body side",
        )
    ctx.check(
        "screen assembly swings outward from body",
        rest_screen_pos is not None
        and opened_screen_pos is not None
        and opened_screen_pos[1] < rest_screen_pos[1] - 0.075,
        details=f"rest={rest_screen_pos}, opened={opened_screen_pos}",
    )

    rest_lcd_aabb = ctx.part_element_world_aabb(screen, elem="lcd_glass")
    with ctx.pose({screen_joint: 1.00}):
        swivel_lcd_aabb = ctx.part_element_world_aabb(screen, elem="lcd_glass")
    ctx.check(
        "viewing screen hinge rotates display outward",
        rest_lcd_aabb is not None
        and swivel_lcd_aabb is not None
        and swivel_lcd_aabb[0][1] < rest_lcd_aabb[0][1] - 0.050,
        details=f"rest={rest_lcd_aabb}, swivel={swivel_lcd_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
