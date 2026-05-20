from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.165
BODY_WIDTH = 0.074
BODY_HALF_WIDTH = BODY_WIDTH / 2.0
WHEEL_RADIUS = 0.014
WHEEL_WIDTH = 0.011
WHEEL_Z = 0.016
FRONT_AXLE_X = 0.052
REAR_AXLE_X = -0.055


def _toy_hatchback_body() -> cq.Workplane:
    """One connected toy-car body with a short hood, tall cabin, rear deck and open arches."""
    # X is front-positive, Y is width, Z is up.  The side outline is intentionally
    # notch-backed so the separate rear liftgate and the small trunk lid can both
    # be read at toy scale.
    side_profile = [
        (-0.080, 0.014),
        (-0.079, 0.039),
        (-0.061, 0.047),
        (-0.043, 0.071),
        (-0.020, 0.080),
        (0.032, 0.077),
        (0.054, 0.052),
        (0.080, 0.042),
        (0.083, 0.019),
        (0.070, 0.012),
        (-0.074, 0.012),
    ]
    body = (
        cq.Workplane("XZ")
        .polyline(side_profile)
        .close()
        .extrude(BODY_WIDTH / 2.0, both=True)
    )

    # Round the toy shell lightly.  If a downstream CadQuery kernel cannot fillet
    # one of the complex arch edges, the plain shell is still valid and readable.
    try:
        body = body.edges("|Y").fillet(0.003)
    except Exception:
        pass

    arch_radius = 0.021
    for axle_x in (FRONT_AXLE_X, REAR_AXLE_X):
        arch_cutter = (
            cq.Workplane("XZ")
            .center(axle_x, WHEEL_Z)
            .circle(arch_radius)
            .extrude(BODY_WIDTH, both=True)
        )
        body = body.cut(arch_cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_hatchback_notchback")

    red = model.material("painted_toy_red", rgba=(0.88, 0.08, 0.045, 1.0))
    dark_red = model.material("separate_red_panels", rgba=(0.72, 0.035, 0.03, 1.0))
    black = model.material("soft_black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoky_blue_glass", rgba=(0.08, 0.16, 0.24, 0.88))
    silver = model.material("bright_silver_plastic", rgba=(0.78, 0.76, 0.70, 1.0))
    amber = model.material("amber_lenses", rgba=(1.0, 0.72, 0.22, 1.0))
    ruby = model.material("ruby_lenses", rgba=(0.95, 0.04, 0.04, 1.0))

    body = model.part("body_shell")
    body.visual(
        mesh_from_cadquery(_toy_hatchback_body(), "body_shell"),
        material=red,
        name="body_shell",
    )
    # Rear deck reference and black reveal groove beneath the separate trunk lid.
    body.visual(
        Box((0.030, 0.060, 0.002)),
        origin=Origin(xyz=(-0.068, 0.0, 0.048)),
        material=red,
        name="rear_deck",
    )
    body.visual(
        Box((0.032, 0.062, 0.0012)),
        origin=Origin(xyz=(-0.068, 0.0, 0.0495)),
        material=black,
        name="trunk_reveal",
    )
    # Dark windows and lighting are slightly embedded in the body so they remain
    # connected visual details rather than floating decals.
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        body.visual(
            Box((0.045, 0.0016, 0.016)),
            origin=Origin(xyz=(0.006, sign * (BODY_HALF_WIDTH + 0.0002), 0.061)),
            material=glass,
            name=f"side_window_{suffix}",
        )
    body.visual(
        Box((0.024, 0.055, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, 0.058), rpy=(0.0, 0.72, 0.0)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((0.0016, 0.052, 0.014)),
        origin=Origin(xyz=(-0.0795, 0.0, 0.044)),
        material=glass,
        name="rear_backlight_shadow",
    )
    for y in (-0.022, 0.022):
        body.visual(
            Box((0.004, 0.014, 0.006)),
            origin=Origin(xyz=(0.0818, y, 0.035)),
            material=amber,
            name=f"headlamp_{0 if y < 0 else 1}",
        )
        body.visual(
            Box((0.0015, 0.010, 0.008)),
            origin=Origin(xyz=(-0.0805, y, 0.033)),
            material=ruby,
            name=f"taillamp_{0 if y < 0 else 1}",
        )
    body.visual(
        Box((0.136, 0.014, 0.006)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0125)),
        material=black,
        name="underbody_rail",
    )
    # Simple axle shafts across the open wheel arches.  They stop just inside the
    # wheel inner faces, so the wheel joints read as axle-mounted without an
    # artificial overlap waiver.
    for axle_x, visual_name in ((FRONT_AXLE_X, "front_axle"), (REAR_AXLE_X, "rear_axle")):
        body.visual(
            Box((0.010, 0.014, 0.006)),
            origin=Origin(xyz=(axle_x, 0.0, 0.014)),
            material=black,
            name=f"{visual_name}_mount",
        )
        body.visual(
            Cylinder(radius=0.0017, length=0.084),
            origin=Origin(xyz=(axle_x, 0.0, WHEEL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=visual_name,
        )

    # Side doors: separate proud panels with their own glass, handles, and vertical hinges.
    door_length = 0.045
    door_height = 0.034
    door_thick = 0.0024
    hinge_x = 0.034
    hinge_z = 0.026
    for side_index, sign in enumerate((1.0, -1.0)):
        door = model.part(f"side_door_{side_index}")
        door.visual(
            Box((door_length, door_thick, door_height)),
            origin=Origin(xyz=(-door_length / 2.0, sign * door_thick / 2.0, door_height / 2.0)),
            material=dark_red,
            name="door_skin",
        )
        door.visual(
            Box((0.027, door_thick * 1.15, 0.012)),
            origin=Origin(xyz=(-0.023, sign * door_thick * 0.62, 0.024)),
            material=glass,
            name="door_window",
        )
        door.visual(
            Box((0.007, 0.0013, 0.0022)),
            origin=Origin(xyz=(-0.033, sign * (door_thick + 0.0005), 0.017)),
            material=silver,
            name="door_handle",
        )
        door.visual(
            Cylinder(radius=0.0012, length=door_height * 0.86),
            origin=Origin(xyz=(0.0, sign * 0.0008, door_height * 0.48)),
            material=black,
            name="hinge_pin",
        )
        model.articulation(
            f"body_to_side_door_{side_index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, sign * (BODY_HALF_WIDTH + 0.0010), hinge_z)),
            axis=(0.0, 0.0, -sign),
            motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=3.0, velocity=2.0),
        )

    # Rear liftgate hinged at the top of the tall passenger cabin.
    liftgate = model.part("rear_liftgate")
    liftgate.visual(
        Box((0.0024, 0.056, 0.030)),
        origin=Origin(xyz=(-0.0020, 0.0, -0.015)),
        material=dark_red,
        name="liftgate_skin",
    )
    liftgate.visual(
        Box((0.0014, 0.044, 0.014)),
        origin=Origin(xyz=(-0.0031, 0.0, -0.011)),
        material=glass,
        name="liftgate_window",
    )
    liftgate.visual(
        Box((0.0015, 0.016, 0.003)),
        origin=Origin(xyz=(-0.0033, 0.0, -0.026)),
        material=silver,
        name="liftgate_handle",
    )
    liftgate.visual(
        Cylinder(radius=0.0013, length=0.050),
        origin=Origin(xyz=(-0.001, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="top_hinge_barrel",
    )
    model.articulation(
        "body_to_rear_liftgate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=liftgate,
        origin=Origin(xyz=(-0.043, 0.0, 0.0715)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=4.0, velocity=1.8),
    )

    # Separate short trunk lid on the rear body deck, also upward-hinged.
    trunk = model.part("trunk_lid")
    trunk_length = 0.030
    trunk.visual(
        Box((trunk_length, 0.057, 0.0024)),
        origin=Origin(xyz=(-trunk_length / 2.0, 0.0, 0.0017)),
        material=dark_red,
        name="lid_skin",
    )
    trunk.visual(
        Cylinder(radius=0.0012, length=0.046),
        origin=Origin(xyz=(-0.001, 0.0, 0.0018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="deck_hinge_barrel",
    )
    trunk.visual(
        Box((0.006, 0.014, 0.0013)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0035)),
        material=silver,
        name="trunk_handle",
    )
    model.articulation(
        "body_to_trunk_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk,
        origin=Origin(xyz=(-0.053, 0.0, 0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=2.5, velocity=1.6),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.0092,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0012, count=20, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.0012, depth=0.0008),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.0015, radius=0.0012),
        ),
        "small_black_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.0095,
            WHEEL_WIDTH * 0.92,
            rim=WheelRim(inner_radius=0.0062, flange_height=0.0010, flange_thickness=0.0008),
            hub=WheelHub(radius=0.0032, width=0.006, cap_style="domed"),
            face=WheelFace(dish_depth=0.0012, front_inset=0.0006, rear_inset=0.0006),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0009, window_radius=0.0016),
            bore=WheelBore(style="round", diameter=0.0022),
        ),
        "small_silver_wheel",
    )
    wheel_positions = (
        ("front_wheel_0", FRONT_AXLE_X, BODY_HALF_WIDTH + WHEEL_WIDTH / 2.0 + 0.0006),
        ("front_wheel_1", FRONT_AXLE_X, -(BODY_HALF_WIDTH + WHEEL_WIDTH / 2.0 + 0.0006)),
        ("rear_wheel_0", REAR_AXLE_X, BODY_HALF_WIDTH + WHEEL_WIDTH / 2.0 + 0.0006),
        ("rear_wheel_1", REAR_AXLE_X, -(BODY_HALF_WIDTH + WHEEL_WIDTH / 2.0 + 0.0006)),
    )
    for wheel_name, axle_x, wheel_y in wheel_positions:
        side_sign = 1.0 if wheel_y > 0.0 else -1.0
        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=silver,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.003, length=0.007),
            origin=Origin(xyz=(0.0, -side_sign * 0.0042, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="inner_hub",
        )
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(axle_x, wheel_y, WHEEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body_shell")
    trunk = object_model.get_part("trunk_lid")

    ctx.expect_gap(
        trunk,
        body,
        axis="z",
        positive_elem="lid_skin",
        negative_elem="rear_deck",
        min_gap=0.001,
        max_gap=0.006,
        name="trunk lid sits visibly above rear deck",
    )
    ctx.expect_gap(
        "side_door_0",
        body,
        axis="y",
        positive_elem="door_skin",
        negative_elem="body_shell",
        min_gap=0.0002,
        max_gap=0.004,
        name="side door 0 is separate from body side",
    )
    ctx.expect_gap(
        body,
        "side_door_1",
        axis="y",
        positive_elem="body_shell",
        negative_elem="door_skin",
        min_gap=0.0002,
        max_gap=0.004,
        name="side door 1 is separate from body side",
    )

    wheel_joint_names = (
        "body_to_front_wheel_0",
        "body_to_front_wheel_1",
        "body_to_rear_wheel_0",
        "body_to_rear_wheel_1",
    )
    for joint_name in wheel_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a continuous wheel joint",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    def _elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        part = object_model.get_part(part_name)
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    door0 = object_model.get_articulation("body_to_side_door_0")
    door1 = object_model.get_articulation("body_to_side_door_1")
    liftgate_joint = object_model.get_articulation("body_to_rear_liftgate")
    trunk_joint = object_model.get_articulation("body_to_trunk_lid")

    closed_door0 = _elem_center("side_door_0", "door_skin")
    closed_door1 = _elem_center("side_door_1", "door_skin")
    with ctx.pose({door0: 0.8, door1: 0.8}):
        open_door0 = _elem_center("side_door_0", "door_skin")
        open_door1 = _elem_center("side_door_1", "door_skin")
    ctx.check(
        "side door 0 opens outward",
        closed_door0 is not None and open_door0 is not None and open_door0[1] > closed_door0[1] + 0.008,
        details=f"closed={closed_door0}, open={open_door0}",
    )
    ctx.check(
        "side door 1 opens outward",
        closed_door1 is not None and open_door1 is not None and open_door1[1] < closed_door1[1] - 0.008,
        details=f"closed={closed_door1}, open={open_door1}",
    )

    closed_liftgate = _elem_center("rear_liftgate", "liftgate_skin")
    with ctx.pose({liftgate_joint: 0.9}):
        raised_liftgate = _elem_center("rear_liftgate", "liftgate_skin")
    ctx.check(
        "rear liftgate raises on top hinge",
        closed_liftgate is not None
        and raised_liftgate is not None
        and raised_liftgate[2] > closed_liftgate[2] + 0.006,
        details=f"closed={closed_liftgate}, raised={raised_liftgate}",
    )

    closed_trunk = _elem_center("trunk_lid", "lid_skin")
    with ctx.pose({trunk_joint: 0.8}):
        raised_trunk = _elem_center("trunk_lid", "lid_skin")
    ctx.check(
        "trunk lid raises on deck hinge",
        closed_trunk is not None and raised_trunk is not None and raised_trunk[2] > closed_trunk[2] + 0.008,
        details=f"closed={closed_trunk}, raised={raised_trunk}",
    )

    return ctx.report()


object_model = build_object_model()
