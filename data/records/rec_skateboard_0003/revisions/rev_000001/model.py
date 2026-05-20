from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

DECK_LENGTH = 0.82
DECK_HALF_LENGTH = DECK_LENGTH * 0.5
DECK_CENTER_WIDTH = 0.205
DECK_THICKNESS = 0.011
DECK_BOTTOM_Z = 0.061
TRUCK_MOUNT_X = 0.195
WHEEL_RADIUS = 0.0265
WHEEL_WIDTH = 0.032
AXLE_HALF_SPAN = 0.086
AXLE_SHOULDER_LENGTH = 0.010
AXLE_TIP_LENGTH = 0.004
AXLE_RADIUS = 0.005
TRUCK_PIVOT_DROP = 0.020
HANGER_AXLE_DROP = 0.016
TRUCK_LEAN_LIMIT = math.radians(14.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _deck_section_parameters(x_pos: float) -> tuple[float, float, float, float]:
    x_norm = min(abs(x_pos) / DECK_HALF_LENGTH, 1.0)
    width = DECK_CENTER_WIDTH - 0.022 * (x_norm**1.6) - 0.010 * max(0.0, x_norm - 0.72) / 0.28
    width = max(width, 0.150)

    if x_norm <= 0.54:
        lift = 0.0
    elif x_norm <= 0.82:
        u = (x_norm - 0.54) / 0.28
        lift = 0.014 * (u**2)
    else:
        u = (x_norm - 0.82) / 0.18
        lift = 0.014 + 0.046 * (u**2)

    concave = 0.0032 * max(0.35, 1.0 - max(0.0, x_norm - 0.70) / 0.30)
    bottom_z = DECK_BOTTOM_Z + lift
    top_center_z = bottom_z + DECK_THICKNESS - concave
    top_edge_z = bottom_z + DECK_THICKNESS + 0.0007
    return width, bottom_z, top_center_z, top_edge_z


def _deck_loop(x_pos: float) -> list[tuple[float, float, float]]:
    width, bottom_z, top_center_z, top_edge_z = _deck_section_parameters(x_pos)
    half_width = width * 0.5
    top_shoulder_z = top_center_z + 0.0012
    bottom_shoulder_z = bottom_z + 0.0010
    bottom_edge_z = bottom_z + 0.0020
    return [
        (x_pos, -half_width, top_edge_z),
        (x_pos, -0.72 * half_width, top_shoulder_z),
        (x_pos, 0.0, top_center_z),
        (x_pos, 0.72 * half_width, top_shoulder_z),
        (x_pos, half_width, top_edge_z),
        (x_pos, 0.94 * half_width, bottom_edge_z),
        (x_pos, 0.44 * half_width, bottom_shoulder_z),
        (x_pos, 0.0, bottom_z),
        (x_pos, -0.44 * half_width, bottom_shoulder_z),
        (x_pos, -0.94 * half_width, bottom_edge_z),
    ]


def _grip_loop(x_pos: float) -> list[tuple[float, float, float]]:
    width, _, top_center_z, top_edge_z = _deck_section_parameters(x_pos)
    width -= 0.014
    half_width = width * 0.5
    grip_top_center = top_center_z + 0.00015
    grip_top_edge = top_edge_z + 0.00015
    grip_bottom_center = grip_top_center - 0.0018
    grip_bottom_edge = grip_top_edge - 0.0018
    grip_shoulder_top = grip_top_center + 0.0010
    grip_shoulder_bottom = grip_bottom_center + 0.0007
    return [
        (x_pos, -half_width, grip_top_edge),
        (x_pos, -0.72 * half_width, grip_shoulder_top),
        (x_pos, 0.0, grip_top_center),
        (x_pos, 0.72 * half_width, grip_shoulder_top),
        (x_pos, half_width, grip_top_edge),
        (x_pos, 0.94 * half_width, grip_bottom_edge),
        (x_pos, 0.46 * half_width, grip_shoulder_bottom),
        (x_pos, 0.0, grip_bottom_center),
        (x_pos, -0.46 * half_width, grip_shoulder_bottom),
        (x_pos, -0.94 * half_width, grip_bottom_edge),
    ]


def _deck_bottom_z_at(x_pos: float) -> float:
    _, bottom_z, _, _ = _deck_section_parameters(x_pos)
    return bottom_z


def _deck_top_z_at(x_pos: float) -> float:
    _, _, top_center_z, _ = _deck_section_parameters(x_pos)
    return top_center_z


def _build_deck_mesh():
    section_positions = (-0.41, -0.35, -0.28, -0.20, -0.10, 0.0, 0.10, 0.20, 0.28, 0.35, 0.41)
    return repair_loft(section_loft([_deck_loop(x_pos) for x_pos in section_positions]))


def _build_grip_mesh():
    section_positions = (-0.38, -0.32, -0.25, -0.17, -0.08, 0.0, 0.08, 0.17, 0.25, 0.32, 0.38)
    return repair_loft(section_loft([_grip_loop(x_pos) for x_pos in section_positions]))


def _build_wheel_mesh():
    half_width = WHEEL_WIDTH * 0.5
    return LatheGeometry(
        [
            (WHEEL_RADIUS * 0.60, -half_width),
            (WHEEL_RADIUS * 0.82, -0.96 * half_width),
            (WHEEL_RADIUS * 0.96, -0.62 * half_width),
            (WHEEL_RADIUS * 1.00, -0.16 * half_width),
            (WHEEL_RADIUS * 1.01, 0.0),
            (WHEEL_RADIUS * 1.00, 0.16 * half_width),
            (WHEEL_RADIUS * 0.96, 0.62 * half_width),
            (WHEEL_RADIUS * 0.82, 0.96 * half_width),
            (WHEEL_RADIUS * 0.60, half_width),
            (WHEEL_RADIUS * 0.54, 0.34 * half_width),
            (WHEEL_RADIUS * 0.52, 0.0),
            (WHEEL_RADIUS * 0.54, -0.34 * half_width),
            (WHEEL_RADIUS * 0.60, -half_width),
        ],
        segments=56,
    ).rotate_x(math.pi / 2.0)


def _build_baseplate_mesh():
    return ExtrudeGeometry(
        rounded_rect_profile(0.066, 0.054, 0.010, corner_segments=6),
        0.004,
        center=True,
    )


def _xy_loop_at_z(
    z_pos: float,
    width: float,
    depth: float,
    radius: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x_pos, center_y + y_pos, z_pos)
        for x_pos, y_pos in rounded_rect_profile(width, depth, radius, corner_segments=6)
    ]


def _xz_loop_at_y(
    y_pos: float,
    width: float,
    height: float,
    radius: float,
    *,
    center_x: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x_pos, y_pos, center_z + z_pos)
        for x_pos, z_pos in rounded_rect_profile(width, height, radius, corner_segments=6)
    ]


def _build_baseplate_pedestal_mesh(inboard_sign: float):
    return repair_loft(
        section_loft(
            [
                _xy_loop_at_z(0.0, 0.032, 0.036, 0.009, center_x=0.0),
                _xy_loop_at_z(-0.008, 0.026, 0.030, 0.007, center_x=0.0025 * inboard_sign),
                _xy_loop_at_z(-0.016, 0.020, 0.022, 0.005, center_x=0.0055 * inboard_sign),
            ]
        )
    )


def _build_hanger_web_mesh():
    return repair_loft(
        section_loft(
            [
                _xz_loop_at_y(-0.050, 0.020, 0.010, 0.004, center_z=0.000),
                _xz_loop_at_y(-0.022, 0.034, 0.012, 0.005, center_z=0.001),
                _xz_loop_at_y(0.000, 0.050, 0.015, 0.006, center_z=0.001),
                _xz_loop_at_y(0.022, 0.034, 0.012, 0.005, center_z=0.001),
                _xz_loop_at_y(0.050, 0.020, 0.010, 0.004, center_z=0.000),
            ]
        )
    )


def _truck_inboard_sign(mount_x: float) -> float:
    return -1.0 if mount_x > 0.0 else 1.0


def _add_baseplate_visuals(
    part,
    *,
    inboard_sign: float,
    top_plate_mesh,
    pedestal_mesh,
    baseplate_finish,
    hardware_dark,
    bushing_urethane,
) -> None:
    pivot_x = 0.014 * inboard_sign
    kingpin_x = pivot_x - (0.026 * inboard_sign)

    part.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=baseplate_finish,
        name="top_plate",
    )
    part.visual(
        Box((0.024, 0.030, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=baseplate_finish,
        name="mount_pad",
    )
    part.visual(
        pedestal_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=baseplate_finish,
        name="pedestal",
    )
    part.visual(
        Cylinder(radius=0.0068, length=0.010),
        origin=Origin(
            xyz=(pivot_x - (0.005 * inboard_sign), 0.0, -TRUCK_PIVOT_DROP),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_dark,
        name="pivot_boss",
    )
    part.visual(
        Cylinder(radius=0.0040, length=0.022),
        origin=Origin(xyz=(kingpin_x, 0.0, -0.010)),
        material=hardware_dark,
        name="kingpin_stem",
    )
    part.visual(
        Cylinder(radius=0.0098, length=0.008),
        origin=Origin(xyz=(kingpin_x, 0.0, -0.013)),
        material=bushing_urethane,
        name="upper_bushing",
    )
    part.visual(
        Cylinder(radius=0.0120, length=0.002),
        origin=Origin(xyz=(kingpin_x, 0.0, -0.008)),
        material=hardware_dark,
        name="upper_washer",
    )
    part.visual(
        Cylinder(radius=0.0060, length=0.004),
        origin=Origin(xyz=(kingpin_x, 0.0, -0.022)),
        material=hardware_dark,
        name="lower_anchor_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.070, 0.058, 0.030)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )


def _add_hanger_visuals(
    part,
    *,
    inboard_sign: float,
    hanger_web_mesh,
    truck_metal,
    hardware_dark,
    bushing_urethane,
) -> None:
    kingpin_dx = -0.026 * inboard_sign
    axle_x = 0.5 * kingpin_dx
    axle_z = -HANGER_AXLE_DROP

    part.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(
            xyz=(0.005 * inboard_sign, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=truck_metal,
        name="pivot_nose",
    )
    part.visual(
        hanger_web_mesh,
        origin=Origin(xyz=(0.52 * kingpin_dx, 0.0, -0.011)),
        material=truck_metal,
        name="hanger_web",
    )
    part.visual(
        Cylinder(radius=0.0100, length=0.016),
        origin=Origin(xyz=(kingpin_dx, 0.0, -0.008)),
        material=truck_metal,
        name="kingpin_bore_zone",
    )
    part.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(kingpin_dx, 0.0, -0.001)),
        material=bushing_urethane,
        name="lower_bushing",
    )
    part.visual(
        Cylinder(radius=0.0060, length=0.002),
        origin=Origin(xyz=(kingpin_dx, 0.0, -0.013)),
        material=hardware_dark,
        name="kingpin_nut",
    )
    part.visual(
        Cylinder(radius=AXLE_RADIUS, length=0.122),
        origin=Origin(xyz=(axle_x, 0.0, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=truck_metal,
        name="axle_bar",
    )
    part.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_SHOULDER_LENGTH),
        origin=Origin(
            xyz=(axle_x, AXLE_HALF_SPAN - 0.021, axle_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="left_axle_shoulder",
    )
    part.visual(
        Cylinder(radius=AXLE_RADIUS, length=AXLE_SHOULDER_LENGTH),
        origin=Origin(
            xyz=(axle_x, -AXLE_HALF_SPAN + 0.021, axle_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="right_axle_shoulder",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.095, 0.185, 0.040)),
        mass=0.30,
        origin=Origin(xyz=(axle_x, 0.0, -0.012)),
    )


def _add_wheel_visuals(
    part,
    *,
    side_sign: float,
    wheel_mesh,
    wheel_urethane,
    bearing_dark,
    hardware_dark,
) -> None:
    inner_sign = -side_sign
    part.visual(wheel_mesh, material=wheel_urethane, name="wheel_shell")
    part.visual(
        Cylinder(radius=0.0142, length=WHEEL_WIDTH - 0.004),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_urethane,
        name="core_barrel",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(
            xyz=(0.0, inner_sign * (0.5 * WHEEL_WIDTH - 0.002), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bearing_dark,
        name="inner_hub",
    )
    part.visual(
        Cylinder(radius=0.0080, length=0.004),
        origin=Origin(
            xyz=(0.0, -inner_sign * (0.5 * WHEEL_WIDTH - 0.002), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_dark,
        name="bearing_core",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.12,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_skateboard", assets=ASSETS)

    deck_finish = model.material("deck_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    grip_tape = model.material("grip_tape", rgba=(0.08, 0.08, 0.09, 1.0))
    truck_metal = model.material("truck_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    baseplate_finish = model.material("baseplate_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    bushing_urethane = model.material("bushing_urethane", rgba=(0.85, 0.62, 0.25, 1.0))
    wheel_urethane = model.material("wheel_urethane", rgba=(0.92, 0.91, 0.86, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.32, 0.33, 0.36, 1.0))

    deck_mesh = _save_mesh("premium_skateboard_deck.obj", _build_deck_mesh())
    grip_mesh = _save_mesh("premium_skateboard_grip.obj", _build_grip_mesh())
    wheel_mesh = _save_mesh("premium_skateboard_wheel.obj", _build_wheel_mesh())
    baseplate_mesh = _save_mesh("premium_skateboard_baseplate.obj", _build_baseplate_mesh())
    front_pedestal_mesh = _save_mesh(
        "premium_skateboard_front_baseplate_pedestal.obj",
        _build_baseplate_pedestal_mesh(_truck_inboard_sign(TRUCK_MOUNT_X)),
    )
    rear_pedestal_mesh = _save_mesh(
        "premium_skateboard_rear_baseplate_pedestal.obj",
        _build_baseplate_pedestal_mesh(_truck_inboard_sign(-TRUCK_MOUNT_X)),
    )
    hanger_web_mesh = _save_mesh("premium_skateboard_hanger_web.obj", _build_hanger_web_mesh())

    deck = model.part("deck")
    deck.visual(deck_mesh, material=deck_finish, name="deck_body")
    deck.visual(grip_mesh, material=grip_tape, name="grip_tape")
    for mount_x in (-TRUCK_MOUNT_X, TRUCK_MOUNT_X):
        bolt_z = _deck_top_z_at(mount_x) + 0.0001
        for dx in (-0.016, 0.016):
            for dy in (-0.015, 0.015):
                bolt_name = f"bolt_{'front' if mount_x > 0.0 else 'rear'}_{'l' if dx < 0.0 else 'r'}_{'in' if dy < 0.0 else 'out'}"
                deck.visual(
                    Cylinder(radius=0.0034, length=0.0014),
                    origin=Origin(xyz=(mount_x + dx, dy, bolt_z)),
                    material=hardware_dark,
                    name=bolt_name,
                )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_CENTER_WIDTH, 0.090)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    front_base = model.part("front_truck_base")
    _add_baseplate_visuals(
        front_base,
        inboard_sign=_truck_inboard_sign(TRUCK_MOUNT_X),
        top_plate_mesh=baseplate_mesh,
        pedestal_mesh=front_pedestal_mesh,
        baseplate_finish=baseplate_finish,
        hardware_dark=hardware_dark,
        bushing_urethane=bushing_urethane,
    )

    rear_base = model.part("rear_truck_base")
    _add_baseplate_visuals(
        rear_base,
        inboard_sign=_truck_inboard_sign(-TRUCK_MOUNT_X),
        top_plate_mesh=baseplate_mesh,
        pedestal_mesh=rear_pedestal_mesh,
        baseplate_finish=baseplate_finish,
        hardware_dark=hardware_dark,
        bushing_urethane=bushing_urethane,
    )

    front_hanger = model.part("front_truck_hanger")
    _add_hanger_visuals(
        front_hanger,
        inboard_sign=_truck_inboard_sign(TRUCK_MOUNT_X),
        hanger_web_mesh=hanger_web_mesh,
        truck_metal=truck_metal,
        hardware_dark=hardware_dark,
        bushing_urethane=bushing_urethane,
    )

    rear_hanger = model.part("rear_truck_hanger")
    _add_hanger_visuals(
        rear_hanger,
        inboard_sign=_truck_inboard_sign(-TRUCK_MOUNT_X),
        hanger_web_mesh=hanger_web_mesh,
        truck_metal=truck_metal,
        hardware_dark=hardware_dark,
        bushing_urethane=bushing_urethane,
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        side_sign=1.0,
        wheel_mesh=wheel_mesh,
        wheel_urethane=wheel_urethane,
        bearing_dark=bearing_dark,
        hardware_dark=hardware_dark,
    )
    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        side_sign=-1.0,
        wheel_mesh=wheel_mesh,
        wheel_urethane=wheel_urethane,
        bearing_dark=bearing_dark,
        hardware_dark=hardware_dark,
    )
    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        side_sign=1.0,
        wheel_mesh=wheel_mesh,
        wheel_urethane=wheel_urethane,
        bearing_dark=bearing_dark,
        hardware_dark=hardware_dark,
    )
    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        side_sign=-1.0,
        wheel_mesh=wheel_mesh,
        wheel_urethane=wheel_urethane,
        bearing_dark=bearing_dark,
        hardware_dark=hardware_dark,
    )

    front_mount_z = _deck_bottom_z_at(TRUCK_MOUNT_X)
    rear_mount_z = _deck_bottom_z_at(-TRUCK_MOUNT_X)
    front_inboard = _truck_inboard_sign(TRUCK_MOUNT_X)
    rear_inboard = _truck_inboard_sign(-TRUCK_MOUNT_X)

    model.articulation(
        "deck_to_front_truck_base",
        ArticulationType.FIXED,
        parent=deck,
        child=front_base,
        origin=Origin(xyz=(TRUCK_MOUNT_X, 0.0, front_mount_z)),
    )
    model.articulation(
        "deck_to_rear_truck_base",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_base,
        origin=Origin(xyz=(-TRUCK_MOUNT_X, 0.0, rear_mount_z)),
    )
    model.articulation(
        "front_truck_pivot",
        ArticulationType.REVOLUTE,
        parent=front_base,
        child=front_hanger,
        origin=Origin(xyz=(0.014 * front_inboard, 0.0, -TRUCK_PIVOT_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-TRUCK_LEAN_LIMIT,
            upper=TRUCK_LEAN_LIMIT,
        ),
    )
    model.articulation(
        "rear_truck_pivot",
        ArticulationType.REVOLUTE,
        parent=rear_base,
        child=rear_hanger,
        origin=Origin(xyz=(0.014 * rear_inboard, 0.0, -TRUCK_PIVOT_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-TRUCK_LEAN_LIMIT,
            upper=TRUCK_LEAN_LIMIT,
        ),
    )

    front_axle_x = -0.013 * front_inboard
    rear_axle_x = -0.013 * rear_inboard
    axle_z = -HANGER_AXLE_DROP

    for name, parent, axle_x, side_sign in (
        ("front_left_wheel_spin", front_hanger, front_axle_x, 1.0),
        ("front_right_wheel_spin", front_hanger, front_axle_x, -1.0),
        ("rear_left_wheel_spin", rear_hanger, rear_axle_x, 1.0),
        ("rear_right_wheel_spin", rear_hanger, rear_axle_x, -1.0),
    ):
        wheel_part = {
            "front_left_wheel_spin": front_left_wheel,
            "front_right_wheel_spin": front_right_wheel,
            "rear_left_wheel_spin": rear_left_wheel,
            "rear_right_wheel_spin": rear_right_wheel,
        }[name]
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel_part,
            origin=Origin(xyz=(axle_x, side_sign * AXLE_HALF_SPAN, axle_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=24.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    front_base = object_model.get_part("front_truck_base")
    rear_base = object_model.get_part("rear_truck_base")
    front_hanger = object_model.get_part("front_truck_hanger")
    rear_hanger = object_model.get_part("rear_truck_hanger")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    front_pivot = object_model.get_articulation("front_truck_pivot")
    rear_pivot = object_model.get_articulation("rear_truck_pivot")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_spin = object_model.get_articulation("front_right_wheel_spin")

    deck_body = deck.get_visual("deck_body")
    front_mount_pad = front_base.get_visual("mount_pad")
    rear_mount_pad = rear_base.get_visual("mount_pad")
    front_pivot_boss = front_base.get_visual("pivot_boss")
    rear_pivot_boss = rear_base.get_visual("pivot_boss")
    front_upper_bushing = front_base.get_visual("upper_bushing")
    rear_upper_bushing = rear_base.get_visual("upper_bushing")
    front_pivot_nose = front_hanger.get_visual("pivot_nose")
    rear_pivot_nose = rear_hanger.get_visual("pivot_nose")
    front_lower_bushing = front_hanger.get_visual("lower_bushing")
    rear_lower_bushing = rear_hanger.get_visual("lower_bushing")
    front_left_axle = front_hanger.get_visual("left_axle_shoulder")
    front_right_axle = front_hanger.get_visual("right_axle_shoulder")
    rear_left_axle = rear_hanger.get_visual("left_axle_shoulder")
    rear_right_axle = rear_hanger.get_visual("right_axle_shoulder")
    front_left_inner_hub = front_left_wheel.get_visual("inner_hub")
    front_right_inner_hub = front_right_wheel.get_visual("inner_hub")
    rear_left_inner_hub = rear_left_wheel.get_visual("inner_hub")
    rear_right_inner_hub = rear_right_wheel.get_visual("inner_hub")
    wheel_shell = front_left_wheel.get_visual("wheel_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        front_base,
        deck,
        elem_a=front_mount_pad,
        elem_b=deck_body,
        contact_tol=2e-05,
        name="front baseplate seats on deck",
    )
    ctx.expect_contact(
        rear_base,
        deck,
        elem_a=rear_mount_pad,
        elem_b=deck_body,
        contact_tol=2e-05,
        name="rear baseplate seats on deck",
    )
    ctx.expect_contact(front_base, front_hanger, elem_a=front_pivot_boss, elem_b=front_pivot_nose, name="front pivot nose seats in boss")
    ctx.expect_contact(rear_base, rear_hanger, elem_a=rear_pivot_boss, elem_b=rear_pivot_nose, name="rear pivot nose seats in boss")
    ctx.expect_gap(
        front_base,
        front_hanger,
        axis="z",
        positive_elem=front_upper_bushing,
        negative_elem=front_lower_bushing,
        min_gap=0.0005,
        max_gap=0.007,
        name="front bushing stack remains tight",
    )
    ctx.expect_gap(
        rear_base,
        rear_hanger,
        axis="z",
        positive_elem=rear_upper_bushing,
        negative_elem=rear_lower_bushing,
        min_gap=0.0005,
        max_gap=0.007,
        name="rear bushing stack remains tight",
    )

    ctx.expect_contact(
        front_left_wheel,
        front_hanger,
        elem_a=front_left_inner_hub,
        elem_b=front_left_axle,
        name="front left wheel seats on axle shoulder",
    )
    ctx.expect_contact(
        front_right_wheel,
        front_hanger,
        elem_a=front_right_inner_hub,
        elem_b=front_right_axle,
        name="front right wheel seats on axle shoulder",
    )
    ctx.expect_contact(
        rear_left_wheel,
        rear_hanger,
        elem_a=rear_left_inner_hub,
        elem_b=rear_left_axle,
        name="rear left wheel seats on axle shoulder",
    )
    ctx.expect_contact(
        rear_right_wheel,
        rear_hanger,
        elem_a=rear_right_inner_hub,
        elem_b=rear_right_axle,
        name="rear right wheel seats on axle shoulder",
    )

    ctx.expect_within(front_base, deck, axes="xy", margin=0.010, name="front base stays within deck footprint")
    ctx.expect_within(rear_base, deck, axes="xy", margin=0.010, name="rear base stays within deck footprint")
    ctx.expect_origin_distance(front_base, rear_base, axes="x", min_dist=0.36, max_dist=0.40, name="wheelbase spacing is believable")
    ctx.expect_gap(deck, front_left_wheel, axis="z", positive_elem=deck_body, negative_elem=wheel_shell, min_gap=0.006, name="front wheel clears deck")
    ctx.expect_gap(deck, rear_left_wheel, axis="z", positive_elem=deck_body, negative_elem=rear_left_wheel.get_visual("wheel_shell"), min_gap=0.006, name="rear wheel clears deck")

    front_left_rest = ctx.part_world_position(front_left_wheel)
    front_right_rest = ctx.part_world_position(front_right_wheel)
    rear_left_rest = ctx.part_world_position(rear_left_wheel)
    rear_right_rest = ctx.part_world_position(rear_right_wheel)
    ctx.check(
        "wheel centers resolve at rest",
        all(pos is not None for pos in (front_left_rest, front_right_rest, rear_left_rest, rear_right_rest)),
        details="Expected all wheel centers to be measurable in the rest pose.",
    )
    if front_left_rest and front_right_rest:
        with ctx.pose({front_pivot: TRUCK_LEAN_LIMIT * 0.85}):
            front_left_lean = ctx.part_world_position(front_left_wheel)
            front_right_lean = ctx.part_world_position(front_right_wheel)
            ctx.check(
                "front truck lean lifts one wheel and drops the other",
                front_left_lean is not None
                and front_right_lean is not None
                and front_left_lean[2] > front_left_rest[2] + 0.010
                and front_right_lean[2] < front_right_rest[2] - 0.010,
                details="Front hanger should visibly roll about the truck pivot axis.",
            )
            ctx.expect_contact(front_base, front_hanger, elem_a=front_pivot_boss, elem_b=front_pivot_nose)

    if rear_left_rest and rear_right_rest:
        with ctx.pose({rear_pivot: -TRUCK_LEAN_LIMIT * 0.85}):
            rear_left_lean = ctx.part_world_position(rear_left_wheel)
            rear_right_lean = ctx.part_world_position(rear_right_wheel)
            ctx.check(
                "rear truck lean mirrors the front articulation",
                rear_left_lean is not None
                and rear_right_lean is not None
                and rear_left_lean[2] < rear_left_rest[2] - 0.010
                and rear_right_lean[2] > rear_right_rest[2] + 0.010,
                details="Rear hanger should visibly roll in the opposite direction when posed negative.",
            )
            ctx.expect_contact(rear_base, rear_hanger, elem_a=rear_pivot_boss, elem_b=rear_pivot_nose)

    with ctx.pose({front_left_spin: math.pi * 0.75, front_right_spin: -math.pi * 0.5}):
        ctx.expect_contact(front_left_wheel, front_hanger, elem_a=front_left_inner_hub, elem_b=front_left_axle)
        ctx.expect_contact(front_right_wheel, front_hanger, elem_a=front_right_inner_hub, elem_b=front_right_axle)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
