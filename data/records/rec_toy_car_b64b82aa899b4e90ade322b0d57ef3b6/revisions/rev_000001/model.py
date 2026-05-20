from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


CAR_LENGTH = 0.180
CAR_WIDTH = 0.072
WHEEL_RADIUS = 0.0175
WHEEL_WIDTH = 0.012
WHEEL_CENTER_Z = WHEEL_RADIUS
FRONT_WHEEL_X = 0.060
REAR_WHEEL_X = -0.066
WHEEL_Y = 0.052

HOOD_HINGE_X = 0.025
HOOD_HINGE_Z = 0.049
HOOD_LENGTH = 0.064
HOOD_WIDTH = 0.052
HOOD_THICKNESS = 0.0032

DOOR_FRONT_X = 0.026
DOOR_LENGTH = 0.054
DOOR_BOTTOM_Z = 0.0275
DOOR_HEIGHT = 0.026
DOOR_THICKNESS = 0.0024
DOOR_Y = 0.0415


def _yz_section(
    x: float, width: float, height: float, z_center: float, radius: float
) -> list[tuple[float, float, float]]:
    """Rounded cross-section loop in a vertical YZ plane."""
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _add_y_cylinder(part, *, name: str, center, radius: float, length: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_door_visuals(part, *, side_sign: float, body_red, seam_black, glass_blue) -> None:
    outward = side_sign
    # A toy-like flat steel door whose local frame sits on the vertical A-pillar
    # hinge line. The red panel extends rearward along local -X.
    part.visual(
        Box((DOOR_LENGTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-DOOR_LENGTH / 2.0, 0.0, DOOR_HEIGHT / 2.0)),
        material=body_red,
        name="door_panel",
    )
    # Black cut lines are shallow proud strips on the outside face, giving an
    # explicit separate die-cast door outline rather than a painted rectangle.
    y_face = outward * (DOOR_THICKNESS / 2.0)
    strip_t = 0.0012
    part.visual(
        Box((DOOR_LENGTH * 0.95, strip_t, 0.0011)),
        origin=Origin(xyz=(-DOOR_LENGTH / 2.0, y_face, DOOR_HEIGHT - 0.0010)),
        material=seam_black,
        name="upper_cut",
    )
    part.visual(
        Box((DOOR_LENGTH * 0.92, strip_t, 0.0010)),
        origin=Origin(xyz=(-DOOR_LENGTH / 2.0, y_face, 0.0010)),
        material=seam_black,
        name="lower_cut",
    )
    part.visual(
        Box((0.0010, strip_t, DOOR_HEIGHT * 0.92)),
        origin=Origin(xyz=(-0.0010, y_face, DOOR_HEIGHT / 2.0)),
        material=seam_black,
        name="front_cut",
    )
    part.visual(
        Box((0.0010, strip_t, DOOR_HEIGHT * 0.82)),
        origin=Origin(xyz=(-DOOR_LENGTH + 0.0010, y_face, DOOR_HEIGHT / 2.0)),
        material=seam_black,
        name="rear_cut",
    )
    part.visual(
        Box((DOOR_LENGTH * 0.46, strip_t, 0.010)),
        origin=Origin(xyz=(-DOOR_LENGTH * 0.38, y_face, DOOR_HEIGHT * 0.70)),
        material=glass_blue,
        name="side_window",
    )
    part.visual(
        Box((0.006, strip_t, 0.0012)),
        origin=Origin(xyz=(-DOOR_LENGTH * 0.68, y_face, DOOR_HEIGHT * 0.48)),
        material=seam_black,
        name="door_handle",
    )
    part.visual(
        Cylinder(radius=0.0016, length=DOOR_HEIGHT * 0.80),
        origin=Origin(xyz=(0.0012, -outward * 0.00015, DOOR_HEIGHT * 0.50)),
        material=seam_black,
        name="hinge_barrel",
    )


def _add_wheel_visuals(part, *, prefix: str, side_sign: float, rubber, wheel_metal) -> None:
    # Wheel/Tire helpers build a spin axis along local +X. Rotate the visuals so
    # that local +X points outboard on each side and the joint axis is the car's Y axis.
    wheel_origin = Origin(rpy=(0.0, 0.0, side_sign * math.pi / 2.0))
    tire = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.0122,
        carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.04),
        tread=TireTread(style="ribbed", depth=0.0013, count=18, land_ratio=0.58),
        grooves=(
            TireGroove(center_offset=-0.0024, width=0.0009, depth=0.0007),
            TireGroove(center_offset=0.0024, width=0.0009, depth=0.0007),
        ),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.0012, radius=0.0008),
    )
    wheel = WheelGeometry(
        0.0122,
        WHEEL_WIDTH * 0.94,
        rim=WheelRim(
            inner_radius=0.0076,
            flange_height=0.0009,
            flange_thickness=0.0007,
            bead_seat_depth=0.0005,
        ),
        hub=WheelHub(
            radius=0.0042,
            width=WHEEL_WIDTH * 0.80,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.0056, hole_diameter=0.0007),
        ),
        face=WheelFace(dish_depth=0.0011, front_inset=0.00055, rear_inset=0.0004),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0008, window_radius=0.0014),
        bore=WheelBore(style="round", diameter=0.0022),
    )
    part.visual(
        _save_mesh(tire, f"{prefix}_tire"),
        origin=wheel_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        _save_mesh(wheel, f"{prefix}_rim"),
        origin=wheel_origin,
        material=wheel_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.0031, length=WHEEL_WIDTH * 0.92),
        origin=wheel_origin,
        material=wheel_metal,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_coupe_opening_hood")

    body_red = model.material("body_red", rgba=(0.82, 0.06, 0.035, 1.0))
    dark_red = model.material("dark_red", rgba=(0.56, 0.03, 0.025, 1.0))
    seam_black = model.material("seam_black", rgba=(0.015, 0.015, 0.017, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.022, 1.0))
    wheel_metal = model.material("bright_zinc", rgba=(0.78, 0.78, 0.74, 1.0))
    glass_blue = model.material("smoked_blue_glass", rgba=(0.16, 0.33, 0.47, 0.72))
    chrome = model.material("chrome", rgba=(0.88, 0.86, 0.78, 1.0))

    body = model.part("body")
    lower_sections = [
        _yz_section(-CAR_LENGTH / 2.0, 0.058, 0.019, 0.030, 0.006),
        _yz_section(-0.062, 0.071, 0.030, 0.035, 0.008),
        _yz_section(-0.014, 0.072, 0.031, 0.036, 0.009),
        _yz_section(0.046, 0.069, 0.026, 0.034, 0.008),
        _yz_section(CAR_LENGTH / 2.0, 0.056, 0.019, 0.031, 0.006),
    ]
    body.visual(
        _save_mesh(section_loft(lower_sections), "lower_body_shell"),
        material=body_red,
        name="lower_body_shell",
    )
    # Rear deck / trunk volume, deliberately blocky like a die-cast coupe.
    body.visual(
        Box((0.056, 0.058, 0.012)),
        origin=Origin(xyz=(-0.058, 0.0, 0.047)),
        material=body_red,
        name="trunk_deck",
    )
    body.visual(
        Box((0.050, 0.0010, 0.0010)),
        origin=Origin(xyz=(-0.057, 0.0, 0.0534)),
        material=seam_black,
        name="trunk_cross_seam",
    )
    body.visual(
        Box((0.0010, 0.022, 0.0010)),
        origin=Origin(xyz=(-0.082, 0.014, 0.0534)),
        material=seam_black,
        name="trunk_seam_left",
    )
    body.visual(
        Box((0.0010, 0.022, 0.0010)),
        origin=Origin(xyz=(-0.082, -0.014, 0.0534)),
        material=seam_black,
        name="trunk_seam_right",
    )
    # Fixed front and rear fender shoulders leave the center hood panel separate.
    for side_sign, front_name, rear_name, pillar_name in (
        (1.0, "front_fender_left", "rear_fender_left", "a_pillar_left"),
        (-1.0, "front_fender_right", "rear_fender_right", "a_pillar_right"),
    ):
        body.visual(
            Box((0.078, 0.010, 0.014)),
            origin=Origin(xyz=(0.050, side_sign * 0.033, 0.041)),
            material=body_red,
            name=front_name,
        )
        body.visual(
            Box((0.066, 0.011, 0.015)),
            origin=Origin(xyz=(-0.065, side_sign * 0.034, 0.041)),
            material=body_red,
            name=rear_name,
        )
        body.visual(
            Box((0.004, 0.005, 0.025)),
            origin=Origin(xyz=(
                DOOR_FRONT_X + 0.002,
                side_sign * 0.038,
                DOOR_BOTTOM_Z + DOOR_HEIGHT * 0.50,
            )),
            material=dark_red,
            name=pillar_name,
        )
    # Simple coupe greenhouse with separate glass patches.
    cabin_sections = [
        _yz_section(-0.047, 0.044, 0.010, 0.055, 0.004),
        _yz_section(-0.025, 0.050, 0.026, 0.059, 0.008),
        _yz_section(0.020, 0.042, 0.010, 0.055, 0.004),
    ]
    body.visual(
        _save_mesh(section_loft(cabin_sections), "roof_pillars"),
        material=body_red,
        name="roof_pillars",
    )
    body.visual(
        Box((0.032, 0.047, 0.0012)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0715)),
        material=body_red,
        name="roof_skin",
    )
    body.visual(
        Box((0.021, 0.038, 0.0010)),
        origin=Origin(xyz=(0.017, 0.0, 0.0585), rpy=(0.0, -0.55, 0.0)),
        material=glass_blue,
        name="windshield",
    )
    body.visual(
        Box((0.020, 0.038, 0.0010)),
        origin=Origin(xyz=(-0.049, 0.0, 0.0578), rpy=(0.0, 0.55, 0.0)),
        material=glass_blue,
        name="rear_window",
    )
    body.visual(
        Box((0.034, 0.0010, 0.011)),
        origin=Origin(xyz=(-0.016, 0.0258, 0.058)),
        material=glass_blue,
        name="left_quarter_window",
    )
    body.visual(
        Box((0.034, 0.0010, 0.011)),
        origin=Origin(xyz=(-0.016, -0.0258, 0.058)),
        material=glass_blue,
        name="right_quarter_window",
    )
    body.visual(
        Box((0.034, 0.074, 0.005)),
        origin=Origin(xyz=(0.092, 0.0, 0.034)),
        material=chrome,
        name="front_bumper",
    )
    body.visual(
        Box((0.032, 0.070, 0.005)),
        origin=Origin(xyz=(-0.093, 0.0, 0.035)),
        material=chrome,
        name="rear_bumper",
    )
    body.visual(
        Box((0.007, 0.056, 0.003)),
        origin=Origin(xyz=(HOOD_HINGE_X, 0.0, 0.0470)),
        material=dark_red,
        name="hood_cowl",
    )
    for side_sign, knuckle_name, leaf_name in (
        (1.0, "hood_hinge_knuckle_left", "hood_hinge_leaf_left"),
        (-1.0, "hood_hinge_knuckle_right", "hood_hinge_leaf_right"),
    ):
        body.visual(
            Box((0.006, 0.005, 0.003)),
            origin=Origin(xyz=(HOOD_HINGE_X + 0.0005, side_sign * 0.014, 0.0495)),
            material=seam_black,
            name=leaf_name,
        )
        body.visual(
            Cylinder(radius=0.00135, length=0.007),
            origin=Origin(
                xyz=(HOOD_HINGE_X + 0.0005, side_sign * 0.014, HOOD_HINGE_Z + 0.0018),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=seam_black,
            name=knuckle_name,
        )
    _add_y_cylinder(
        body,
        name="front_axle",
        center=(FRONT_WHEEL_X, 0.0, WHEEL_CENTER_Z),
        radius=0.0025,
        length=0.0944,
        material=chrome,
    )
    _add_y_cylinder(
        body,
        name="rear_axle",
        center=(REAR_WHEEL_X, 0.0, WHEEL_CENTER_Z),
        radius=0.0025,
        length=0.0944,
        material=chrome,
    )
    for x, side_sign, socket_name in (
        (FRONT_WHEEL_X, 1.0, "front_axle_socket_left"),
        (FRONT_WHEEL_X, -1.0, "front_axle_socket_right"),
        (REAR_WHEEL_X, 1.0, "rear_axle_socket_left"),
        (REAR_WHEEL_X, -1.0, "rear_axle_socket_right"),
    ):
        body.visual(
            Box((0.010, 0.007, 0.018)),
            origin=Origin(xyz=(x, side_sign * 0.0385, WHEEL_CENTER_Z + 0.0075)),
            material=seam_black,
            name=socket_name,
        )

    hood = model.part("hood")
    hood_profile = rounded_rect_profile(HOOD_LENGTH, HOOD_WIDTH, 0.006, corner_segments=8)
    hood.visual(
        _save_mesh(ExtrudeGeometry(hood_profile, HOOD_THICKNESS, center=True), "hood_skin"),
        origin=Origin(xyz=(HOOD_LENGTH / 2.0, 0.0, HOOD_THICKNESS / 2.0)),
        material=body_red,
        name="hood_skin",
    )
    hood.visual(
        Box((HOOD_LENGTH * 0.92, 0.0010, 0.0008)),
        origin=Origin(xyz=(HOOD_LENGTH / 2.0, HOOD_WIDTH / 2.0 + 0.0002, HOOD_THICKNESS + 0.0002)),
        material=seam_black,
        name="left_hood_gap",
    )
    hood.visual(
        Box((HOOD_LENGTH * 0.92, 0.0010, 0.0008)),
        origin=Origin(xyz=(HOOD_LENGTH / 2.0, -HOOD_WIDTH / 2.0 - 0.0002, HOOD_THICKNESS + 0.0002)),
        material=seam_black,
        name="right_hood_gap",
    )
    hood.visual(
        Cylinder(radius=0.00145, length=0.021),
        origin=Origin(xyz=(0.0005, 0.0, 0.0018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=seam_black,
        name="hood_hinge_barrel",
    )

    left_door = model.part("left_door")
    _add_door_visuals(left_door, side_sign=1.0, body_red=body_red, seam_black=seam_black, glass_blue=glass_blue)
    right_door = model.part("right_door")
    _add_door_visuals(right_door, side_sign=-1.0, body_red=body_red, seam_black=seam_black, glass_blue=glass_blue)

    wheels = {
        "front_left_wheel": (FRONT_WHEEL_X, WHEEL_Y, "front_left"),
        "front_right_wheel": (FRONT_WHEEL_X, -WHEEL_Y, "front_right"),
        "rear_left_wheel": (REAR_WHEEL_X, WHEEL_Y, "rear_left"),
        "rear_right_wheel": (REAR_WHEEL_X, -WHEEL_Y, "rear_right"),
    }
    for part_name, (_, y, prefix) in wheels.items():
        wheel = model.part(part_name)
        _add_wheel_visuals(
            wheel,
            prefix=prefix,
            side_sign=1.0 if y > 0.0 else -1.0,
            rubber=rubber,
            wheel_metal=wheel_metal,
        )

    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(HOOD_HINGE_X, 0.0, HOOD_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=1.12),
    )
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(DOOR_FRONT_X, DOOR_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(DOOR_FRONT_X, -DOOR_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.15),
    )

    for part_name, (x, y, _) in wheels.items():
        model.articulation(
            f"{part_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=part_name,
            origin=Origin(xyz=(x, y, WHEEL_CENTER_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    hood_hinge = object_model.get_articulation("hood_hinge")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    wheel_joints = [
        object_model.get_articulation("front_left_wheel_spin"),
        object_model.get_articulation("front_right_wheel_spin"),
        object_model.get_articulation("rear_left_wheel_spin"),
        object_model.get_articulation("rear_right_wheel_spin"),
    ]
    ctx.check(
        "four continuous wheel spins",
        all(getattr(j, "articulation_type", None) == ArticulationType.CONTINUOUS for j in wheel_joints),
        details="Each exposed wheel should be a continuous rotary child of the body.",
    )

    # Closed-fit checks show that the separate hood and doors are real panels,
    # not simply painted seams, while still reading as flush toy-car sheet metal.
    ctx.expect_gap(
        hood,
        body,
        axis="z",
        positive_elem="hood_skin",
        negative_elem="front_fender_left",
        min_gap=0.0002,
        max_gap=0.004,
        name="hood sits just above front fender",
    )
    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="lower_body_shell",
        min_gap=0.0015,
        max_gap=0.008,
        name="left door closes near body side",
    )
    ctx.expect_gap(
        body,
        right_door,
        axis="y",
        positive_elem="lower_body_shell",
        negative_elem="door_panel",
        min_gap=0.0015,
        max_gap=0.008,
        name="right door closes near body side",
    )

    left_rest = ctx.part_world_aabb(left_door)
    right_rest = ctx.part_world_aabb(right_door)
    hood_rest = ctx.part_world_aabb(hood)

    with ctx.pose({left_hinge: 0.85}):
        left_open = ctx.part_world_aabb(left_door)
    with ctx.pose({right_hinge: 0.85}):
        right_open = ctx.part_world_aabb(right_door)
    with ctx.pose({hood_hinge: 0.85}):
        hood_open = ctx.part_world_aabb(hood)

    ctx.check(
        "left door swings outward",
        left_rest is not None
        and left_open is not None
        and left_open[1][1] > left_rest[1][1] + 0.010,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right door swings outward",
        right_rest is not None
        and right_open is not None
        and right_open[0][1] < right_rest[0][1] - 0.010,
        details=f"rest={right_rest}, open={right_open}",
    )
    ctx.check(
        "hood lifts upward",
        hood_rest is not None
        and hood_open is not None
        and hood_open[1][2] > hood_rest[1][2] + 0.018,
        details=f"rest={hood_rest}, open={hood_open}",
    )

    return ctx.report()


object_model = build_object_model()
