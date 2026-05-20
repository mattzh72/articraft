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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_HALF_X = 0.40
TOP_Z = 0.86
FOOT_Z = 0.022
DECK_HALF_WIDTH = 0.070
WING_AXIS_OFFSET = 0.094
WING_DEPTH = 0.24
WING_OPEN_ANGLE = math.radians(58.0)
TUBE_RADIUS = 0.009
LEG_RADIUS = 0.0105
HINGE_RADIUS = 0.0084
WING_TUBE_RADIUS = 0.0075
TRANSPORT_WHEEL_RADIUS = 0.028
TRANSPORT_TIRE_WIDTH = 0.012
TRANSPORT_HUB_LENGTH = 0.020
TRANSPORT_WHEEL_X = FRAME_HALF_X + 0.038
TRANSPORT_BRACKET_X = TRANSPORT_WHEEL_X - 0.014
TRANSPORT_WHEEL_Z = 0.038

INNER_RAIL_Y = 0.018
INNER_RAIL_Z = 0.004
OUTER_RAIL_Y = INNER_RAIL_Y + WING_DEPTH * math.cos(WING_OPEN_ANGLE)
OUTER_RAIL_Z = INNER_RAIL_Z + WING_DEPTH * math.sin(WING_OPEN_ANGLE)
BRACE_Y = INNER_RAIL_Y + 0.14 * WING_DEPTH * math.cos(WING_OPEN_ANGLE)
BRACE_Z = INNER_RAIL_Z + 0.14 * WING_DEPTH * math.sin(WING_OPEN_ANGLE) - 0.002


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _point_at_z(
    top: tuple[float, float, float],
    foot: tuple[float, float, float],
    z_target: float,
) -> tuple[float, float, float]:
    dz = foot[2] - top[2]
    if abs(dz) < 1e-9:
        return top
    t = (z_target - top[2]) / dz
    return (
        top[0] + (foot[0] - top[0]) * t,
        top[1] + (foot[1] - top[1]) * t,
        z_target,
    )


def _wing_y(side_sign: float, offset: float) -> float:
    return side_sign * offset


def _wing_inboard_y(side_sign: float, offset: float) -> float:
    return -side_sign * offset


def _build_wing(
    part,
    *,
    side_sign: float,
    side_name: str,
    satin_aluminum,
    matte_graphite,
    soft_polymer,
) -> None:
    wing_span = 0.764
    inner_y = _wing_y(side_sign, INNER_RAIL_Y)
    outer_y = _wing_y(side_sign, OUTER_RAIL_Y)
    brace_y = _wing_y(side_sign, BRACE_Y)

    part.visual(
        Cylinder(radius=WING_TUBE_RADIUS, length=wing_span),
        origin=Origin(xyz=(0.0, inner_y, INNER_RAIL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name=f"{side_name}_inner_rail",
    )
    part.visual(
        Cylinder(radius=WING_TUBE_RADIUS, length=wing_span),
        origin=Origin(xyz=(0.0, outer_y, OUTER_RAIL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name=f"{side_name}_outer_rail",
    )

    for x, visual_name in [(-0.317, f"{side_name}_hinge_barrel_front"), (0.317, f"{side_name}_hinge_barrel_rear")]:
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.055),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_graphite,
            name=visual_name,
        )
        part.visual(
            Box((0.046, 0.020, 0.014)),
            origin=Origin(xyz=(x, side_sign * 0.009, 0.003)),
            material=matte_graphite,
            name=f"{side_name}_{'front' if x < 0.0 else 'rear'}_hinge_spine",
        )

    for x, cap_name in [(-0.382, f"{side_name}_front_cap"), (0.382, f"{side_name}_rear_cap")]:
        part.visual(
            Box((0.020, 0.018, 0.018)),
            origin=Origin(xyz=(x, outer_y, OUTER_RAIL_Z)),
            material=soft_polymer,
            name=cap_name,
        )
        _add_member(
            part,
            (x, inner_y, INNER_RAIL_Z),
            (x, outer_y, OUTER_RAIL_Z),
            WING_TUBE_RADIUS,
            satin_aluminum,
            name=f"{side_name}_{'front' if x < 0.0 else 'rear'}_end_stanchion",
        )

    for index, fraction in enumerate((0.26, 0.50, 0.74), start=1):
        y = _wing_y(side_sign, INNER_RAIL_Y + fraction * (OUTER_RAIL_Y - INNER_RAIL_Y))
        z = INNER_RAIL_Z + fraction * (OUTER_RAIL_Z - INNER_RAIL_Z)
        part.visual(
            Cylinder(radius=WING_TUBE_RADIUS * 0.92, length=wing_span),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_aluminum,
            name=f"{side_name}_cross_rail_{index}",
        )

    part.visual(
        Box((wing_span, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, brace_y, BRACE_Z)),
        material=matte_graphite,
        name=f"{side_name}_brace_lug",
    )


def _add_foot_cap(part, center: tuple[float, float, float], material, *, name: str) -> None:
    part.visual(
        Box((0.032, 0.020, 0.022)),
        origin=Origin(xyz=(center[0], center[1], 0.012)),
        material=material,
        name=name,
    )


def _add_transport_bracket(
    part,
    *,
    wheel_y: float,
    anchor: tuple[float, float, float],
    label: str,
    shell_material,
    hardware_material,
) -> None:
    crown_center = (TRANSPORT_BRACKET_X, wheel_y, TRANSPORT_WHEEL_Z + 0.062)
    _add_member(
        part,
        anchor,
        crown_center,
        TUBE_RADIUS * 0.46,
        shell_material,
        name=f"{label}_wheel_upright",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=crown_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_material,
        name=f"{label}_wheel_bridge",
    )
    for side_sign, side_name in [(-1.0, "inner"), (1.0, "outer")]:
        cheek_center = (TRANSPORT_WHEEL_X - 0.004, wheel_y + side_sign * 0.012, TRANSPORT_WHEEL_Z)
        part.visual(
            Box((0.010, 0.004, 0.022)),
            origin=Origin(xyz=cheek_center),
            material=shell_material,
            name=f"{label}_fork_{side_name}",
        )
        _add_member(
            part,
            (TRANSPORT_BRACKET_X, wheel_y + side_sign * 0.014, TRANSPORT_WHEEL_Z + 0.062),
            (TRANSPORT_WHEEL_X - 0.006, wheel_y + side_sign * 0.0105, TRANSPORT_WHEEL_Z + 0.010),
            TUBE_RADIUS * 0.24,
            shell_material,
            name=f"{label}_{side_name}_drop_link",
        )
    part.visual(
        Cylinder(radius=0.0035, length=0.028),
        origin=Origin(
            xyz=(TRANSPORT_WHEEL_X, wheel_y, TRANSPORT_WHEEL_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_material,
        name=f"{label}_wheel_pivot_pin",
    )


def _build_transport_wheel(
    part,
    *,
    label: str,
    tire_material,
    rim_material,
    hardware_material,
) -> None:
    tire_geom = TorusGeometry(
        radius=TRANSPORT_WHEEL_RADIUS - 0.006,
        tube=0.006,
        radial_segments=16,
        tubular_segments=32,
    )
    tire_geom.rotate_x(math.pi / 2.0)
    tire_mesh = mesh_from_geometry(tire_geom, ASSETS.mesh_path(f"{label}_wheel_tire.obj"))
    part.visual(
        tire_mesh,
        material=tire_material,
        name=f"{label}_tire",
    )
    part.visual(
        Cylinder(radius=0.008, length=TRANSPORT_HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name=f"{label}_hub",
    )
    for angle in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
        spoke_outer = (
            math.cos(angle) * (TRANSPORT_WHEEL_RADIUS - 0.007),
            0.0,
            math.sin(angle) * (TRANSPORT_WHEEL_RADIUS - 0.007),
        )
        _add_member(
            part,
            (0.0, 0.0, 0.0),
            spoke_outer,
            0.0026,
            rim_material,
            name=f"{label}_spoke_{int(round(angle / (math.pi * 0.5))) + 1}",
        )
    part.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(
            xyz=(0.0, TRANSPORT_HUB_LENGTH * 0.5 + 0.002, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rim_material,
        name=f"{label}_outer_cap",
    )
    part.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(
            xyz=(0.0, -(TRANSPORT_HUB_LENGTH * 0.5 + 0.002), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rim_material,
        name=f"{label}_inner_cap",
    )
    part.visual(
        Cylinder(radius=0.0026, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.0, TRANSPORT_WHEEL_RADIUS * 0.90),
        ),
        material=hardware_material,
        name=f"{label}_valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_drying_rack", assets=ASSETS)

    satin_aluminum = model.material("satin_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    soft_polymer = model.material("soft_polymer", rgba=(0.61, 0.63, 0.65, 1.0))
    warm_rubber = model.material("warm_rubber", rgba=(0.13, 0.14, 0.15, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.67, 0.69, 0.72, 1.0))

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.88, 0.48, 0.92)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    top_front_left = (-FRAME_HALF_X, -DECK_HALF_WIDTH, TOP_Z)
    top_rear_left = (-FRAME_HALF_X, DECK_HALF_WIDTH, TOP_Z)
    top_front_right = (FRAME_HALF_X, -DECK_HALF_WIDTH, TOP_Z)
    top_rear_right = (FRAME_HALF_X, DECK_HALF_WIDTH, TOP_Z)

    foot_front_left = (-FRAME_HALF_X, -0.23, FOOT_Z)
    foot_rear_left = (-FRAME_HALF_X, 0.23, FOOT_Z)
    foot_front_right = (FRAME_HALF_X, -0.23, FOOT_Z)
    foot_rear_right = (FRAME_HALF_X, 0.23, FOOT_Z)

    for start, end, name in [
        (top_front_left, foot_front_left, "front_left_leg"),
        (top_rear_left, foot_rear_left, "rear_left_leg"),
        (top_front_right, foot_front_right, "front_right_leg"),
        (top_rear_right, foot_rear_right, "rear_right_leg"),
    ]:
        _add_member(main_frame, start, end, LEG_RADIUS, satin_aluminum, name=name)

    _add_member(main_frame, top_front_left, top_rear_left, TUBE_RADIUS, matte_graphite, name="left_end_bridge")
    _add_member(main_frame, top_front_right, top_rear_right, TUBE_RADIUS, matte_graphite, name="right_end_bridge")

    main_frame.visual(
        Box((0.064, 0.184, 0.024)),
        origin=Origin(xyz=(-FRAME_HALF_X, 0.0, TOP_Z - 0.004)),
        material=matte_graphite,
        name="left_upper_housing",
    )
    main_frame.visual(
        Box((0.064, 0.184, 0.024)),
        origin=Origin(xyz=(FRAME_HALF_X, 0.0, TOP_Z - 0.004)),
        material=matte_graphite,
        name="right_upper_housing",
    )

    rail_positions = (
        (-DECK_HALF_WIDTH, "front_deck_rail"),
        (-0.042, "deck_rail_1"),
        (-0.018, "deck_rail_2"),
        (0.0, "center_top_rail"),
        (0.018, "deck_rail_3"),
        (0.042, "deck_rail_4"),
        (DECK_HALF_WIDTH, "rear_deck_rail"),
    )
    for y, name in rail_positions:
        main_frame.visual(
            Cylinder(radius=TUBE_RADIUS * (1.0 if abs(y) == DECK_HALF_WIDTH else 0.92), length=0.80),
            origin=Origin(xyz=(0.0, y, TOP_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_aluminum,
            name=name,
        )

    brace_level_z = 0.42
    left_front_brace_node = _point_at_z(top_front_left, foot_front_left, brace_level_z)
    left_rear_brace_node = _point_at_z(top_rear_left, foot_rear_left, brace_level_z)
    right_front_brace_node = _point_at_z(top_front_right, foot_front_right, brace_level_z)
    right_rear_brace_node = _point_at_z(top_rear_right, foot_rear_right, brace_level_z)

    _add_member(
        main_frame,
        left_front_brace_node,
        right_front_brace_node,
        TUBE_RADIUS * 0.9,
        matte_graphite,
        name="front_lower_rail",
    )
    _add_member(
        main_frame,
        left_rear_brace_node,
        right_rear_brace_node,
        TUBE_RADIUS * 0.9,
        matte_graphite,
        name="rear_lower_rail",
    )
    _add_member(
        main_frame,
        left_front_brace_node,
        left_rear_brace_node,
        TUBE_RADIUS * 0.86,
        satin_aluminum,
        name="left_mid_spreader",
    )
    _add_member(
        main_frame,
        right_front_brace_node,
        right_rear_brace_node,
        TUBE_RADIUS * 0.86,
        satin_aluminum,
        name="right_mid_spreader",
    )

    for foot_center, name in [
        (foot_front_left, "front_left_foot"),
        (foot_rear_left, "rear_left_foot"),
        (foot_front_right, "front_right_foot"),
        (foot_rear_right, "rear_right_foot"),
    ]:
        _add_foot_cap(main_frame, foot_center, warm_rubber, name=name)

    front_transport_anchor = _point_at_z(top_front_right, foot_front_right, 0.19)
    rear_transport_anchor = _point_at_z(top_rear_right, foot_rear_right, 0.19)
    _add_transport_bracket(
        main_frame,
        wheel_y=-0.165,
        anchor=front_transport_anchor,
        label="front_transport",
        shell_material=matte_graphite,
        hardware_material=hardware_steel,
    )
    _add_transport_bracket(
        main_frame,
        wheel_y=0.165,
        anchor=rear_transport_anchor,
        label="rear_transport",
        shell_material=matte_graphite,
        hardware_material=hardware_steel,
    )

    for side_sign, side_name in [(-1.0, "left"), (1.0, "right")]:
        for x, visual_name in [
            (-0.372, f"{side_name}_hinge_housing_front"),
            (0.372, f"{side_name}_hinge_housing_rear"),
        ]:
            main_frame.visual(
                Cylinder(radius=HINGE_RADIUS, length=0.055),
                origin=Origin(xyz=(x, side_sign * WING_AXIS_OFFSET, TOP_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=matte_graphite,
                name=visual_name,
            )
            main_frame.visual(
                Cylinder(radius=0.0048, length=0.012),
                origin=Origin(
                    xyz=(x + (-0.032 if x < 0.0 else 0.032), side_sign * WING_AXIS_OFFSET, TOP_Z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hardware_steel,
            )

        for index, x in enumerate((-0.33, 0.0, 0.33), start=1):
            main_frame.visual(
                Box((0.075 if x else 0.095, 0.022, 0.016)),
                origin=Origin(xyz=(x, side_sign * (WING_AXIS_OFFSET - 0.028), TOP_Z - 0.012)),
                material=matte_graphite,
                name=f"{side_name}_hinge_boss_{index}",
            )

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((0.80, 0.18, 0.22)),
        mass=1.15,
        origin=Origin(xyz=(0.0, -0.080, 0.105)),
    )
    _build_wing(
        left_wing,
        side_sign=-1.0,
        side_name="left",
        satin_aluminum=satin_aluminum,
        matte_graphite=matte_graphite,
        soft_polymer=soft_polymer,
    )

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((0.80, 0.18, 0.22)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.080, 0.105)),
    )
    _build_wing(
        right_wing,
        side_sign=1.0,
        side_name="right",
        satin_aluminum=satin_aluminum,
        matte_graphite=matte_graphite,
        soft_polymer=soft_polymer,
    )

    model.articulation(
        "left_wing_fold",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=left_wing,
        origin=Origin(xyz=(0.0, -WING_AXIS_OFFSET, TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.90, upper=0.0),
    )
    model.articulation(
        "right_wing_fold",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=right_wing,
        origin=Origin(xyz=(0.0, WING_AXIS_OFFSET, TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.90, upper=0.0),
    )

    front_transport_wheel = model.part("front_transport_wheel")
    front_transport_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=TRANSPORT_WHEEL_RADIUS, length=TRANSPORT_HUB_LENGTH),
        mass=0.16,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_transport_wheel(
        front_transport_wheel,
        label="front_transport",
        tire_material=warm_rubber,
        rim_material=satin_aluminum,
        hardware_material=hardware_steel,
    )

    rear_transport_wheel = model.part("rear_transport_wheel")
    rear_transport_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=TRANSPORT_WHEEL_RADIUS, length=TRANSPORT_HUB_LENGTH),
        mass=0.16,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _build_transport_wheel(
        rear_transport_wheel,
        label="rear_transport",
        tire_material=warm_rubber,
        rim_material=satin_aluminum,
        hardware_material=hardware_steel,
    )

    model.articulation(
        "front_transport_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=front_transport_wheel,
        origin=Origin(xyz=(TRANSPORT_WHEEL_X, -0.165, TRANSPORT_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "rear_transport_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=rear_transport_wheel,
        origin=Origin(xyz=(TRANSPORT_WHEEL_X, 0.165, TRANSPORT_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    main_frame = object_model.get_part("main_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    front_transport_wheel = object_model.get_part("front_transport_wheel")
    rear_transport_wheel = object_model.get_part("rear_transport_wheel")

    left_wing_fold = object_model.get_articulation("left_wing_fold")
    right_wing_fold = object_model.get_articulation("right_wing_fold")
    front_transport_wheel_roll = object_model.get_articulation("front_transport_wheel_roll")
    rear_transport_wheel_roll = object_model.get_articulation("rear_transport_wheel_roll")

    center_top_rail = main_frame.get_visual("center_top_rail")
    left_outer_rail = left_wing.get_visual("left_outer_rail")
    right_outer_rail = right_wing.get_visual("right_outer_rail")
    left_brace_lug = left_wing.get_visual("left_brace_lug")
    right_brace_lug = right_wing.get_visual("right_brace_lug")
    front_transport_valve_stem = front_transport_wheel.get_visual("front_transport_valve_stem")
    front_transport_hub = front_transport_wheel.get_visual("front_transport_hub")
    rear_transport_hub = rear_transport_wheel.get_visual("rear_transport_hub")
    front_transport_pin = main_frame.get_visual("front_transport_wheel_pivot_pin")
    rear_transport_pin = main_frame.get_visual("rear_transport_wheel_pivot_pin")

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
    ctx.allow_overlap(
        main_frame,
        front_transport_wheel,
        elem_a=front_transport_pin,
        elem_b=front_transport_hub,
        reason="The front transport wheel rotates on a captured steel axle pin inside its hub bore.",
    )
    ctx.allow_overlap(
        main_frame,
        rear_transport_wheel,
        elem_a=rear_transport_pin,
        elem_b=rear_transport_hub,
        reason="The rear transport wheel rotates on a captured steel axle pin inside its hub bore.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    main_aabb = ctx.part_world_aabb(main_frame)
    if ctx.check("main_frame_has_bounds", main_aabb is not None, "main frame has no measurable bounds") and main_aabb is not None:
        frame_length = main_aabb[1][0] - main_aabb[0][0]
        frame_depth = main_aabb[1][1] - main_aabb[0][1]
        frame_height = main_aabb[1][2] - main_aabb[0][2]
        ctx.check("main_frame_length_is_realistic", 0.78 <= frame_length <= 0.90, f"length={frame_length:.3f}")
        ctx.check("main_frame_depth_is_realistic", 0.42 <= frame_depth <= 0.50, f"depth={frame_depth:.3f}")
        ctx.check("main_frame_height_is_realistic", 0.82 <= frame_height <= 0.92, f"height={frame_height:.3f}")

    ctx.check(
        "left_wing_fold_axis_is_x",
        abs(left_wing_fold.axis[0]) == 1.0 and left_wing_fold.axis[1] == 0.0 and left_wing_fold.axis[2] == 0.0,
        f"axis={left_wing_fold.axis}",
    )
    ctx.check(
        "right_wing_fold_axis_is_x",
        abs(right_wing_fold.axis[0]) == 1.0 and right_wing_fold.axis[1] == 0.0 and right_wing_fold.axis[2] == 0.0,
        f"axis={right_wing_fold.axis}",
    )
    ctx.check(
        "front_transport_wheel_roll_axis_is_y",
        front_transport_wheel_roll.axis == (0.0, 1.0, 0.0),
        f"axis={front_transport_wheel_roll.axis}",
    )
    ctx.check(
        "rear_transport_wheel_roll_axis_is_y",
        rear_transport_wheel_roll.axis == (0.0, 1.0, 0.0),
        f"axis={rear_transport_wheel_roll.axis}",
    )

    ctx.expect_origin_gap(main_frame, left_wing, axis="y", min_gap=0.085, max_gap=0.105, name="left_wing_hinge_axis_offset")
    ctx.expect_origin_gap(right_wing, main_frame, axis="y", min_gap=0.085, max_gap=0.105, name="right_wing_hinge_axis_offset")
    ctx.expect_overlap(left_wing, main_frame, axes="x", min_overlap=0.72, name="left_wing_tracks_main_length")
    ctx.expect_overlap(right_wing, main_frame, axes="x", min_overlap=0.72, name="right_wing_tracks_main_length")
    ctx.expect_gap(
        left_wing,
        main_frame,
        axis="z",
        positive_elem=left_outer_rail,
        negative_elem=center_top_rail,
        min_gap=0.16,
        max_gap=0.24,
        name="left_wing_open_rail_height",
    )
    ctx.expect_gap(
        right_wing,
        main_frame,
        axis="z",
        positive_elem=right_outer_rail,
        negative_elem=center_top_rail,
        min_gap=0.16,
        max_gap=0.24,
        name="right_wing_open_rail_height",
    )
    ctx.expect_gap(
        left_wing,
        main_frame,
        axis="z",
        positive_elem=left_brace_lug,
        negative_elem=center_top_rail,
        min_gap=0.018,
        max_gap=0.040,
        name="left_brace_lug_clears_top_rails",
    )
    ctx.expect_gap(
        right_wing,
        main_frame,
        axis="z",
        positive_elem=right_brace_lug,
        negative_elem=center_top_rail,
        min_gap=0.018,
        max_gap=0.040,
        name="right_brace_lug_clears_top_rails",
    )
    ctx.expect_gap(right_wing, left_wing, axis="y", min_gap=0.12, name="wings_remain_separated_in_open_pose")
    ctx.expect_contact(main_frame, front_transport_wheel, contact_tol=0.0015, name="front_transport_wheel_is_axled_to_frame")
    ctx.expect_contact(main_frame, rear_transport_wheel, contact_tol=0.0015, name="rear_transport_wheel_is_axled_to_frame")
    ctx.expect_origin_gap(
        front_transport_wheel,
        main_frame,
        axis="x",
        min_gap=0.43,
        max_gap=0.45,
        name="front_transport_wheel_sits_outboard",
    )
    ctx.expect_origin_gap(
        rear_transport_wheel,
        front_transport_wheel,
        axis="y",
        min_gap=0.32,
        max_gap=0.34,
        name="transport_wheels_span_the_rack_depth",
    )

    left_outer_rest = ctx.part_element_world_aabb(left_wing, elem=left_outer_rail)
    right_outer_rest = ctx.part_element_world_aabb(right_wing, elem=right_outer_rail)
    front_valve_rest = ctx.part_element_world_aabb(front_transport_wheel, elem=front_transport_valve_stem)
    has_rest_outer_bounds = ctx.check(
        "wing_outer_rails_have_bounds",
        left_outer_rest is not None and right_outer_rest is not None and front_valve_rest is not None,
        "wing outer rails or front transport valve stem lack world bounds",
    )

    folded_left = left_wing_fold.motion_limits.lower if left_wing_fold.motion_limits is not None and left_wing_fold.motion_limits.lower is not None else -0.90
    folded_right = right_wing_fold.motion_limits.lower if right_wing_fold.motion_limits is not None and right_wing_fold.motion_limits.lower is not None else -0.90

    with ctx.pose({left_wing_fold: folded_left, right_wing_fold: folded_right}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wings_folded_no_overlap")
        ctx.fail_if_isolated_parts(name="wings_folded_no_floating")
        ctx.expect_overlap(
            left_wing,
            main_frame,
            axes="x",
            min_overlap=0.70,
            elem_a=left_outer_rail,
            elem_b=center_top_rail,
            name="left_wing_keeps_length_alignment_when_folded",
        )
        ctx.expect_overlap(
            right_wing,
            main_frame,
            axes="x",
            min_overlap=0.70,
            elem_a=right_outer_rail,
            elem_b=center_top_rail,
            name="right_wing_keeps_length_alignment_when_folded",
        )

        left_outer_fold = ctx.part_element_world_aabb(left_wing, elem=left_outer_rail)
        right_outer_fold = ctx.part_element_world_aabb(right_wing, elem=right_outer_rail)
        has_fold_outer_bounds = ctx.check(
            "folded_outer_rails_have_bounds",
            left_outer_fold is not None and right_outer_fold is not None,
            "one or both folded wing outer rails lack world bounds",
        )
        if has_rest_outer_bounds and has_fold_outer_bounds and left_outer_rest is not None and right_outer_rest is not None and left_outer_fold is not None and right_outer_fold is not None:
            left_drop = left_outer_rest[1][2] - left_outer_fold[1][2]
            right_drop = right_outer_rest[1][2] - right_outer_fold[1][2]
            ctx.check("left_wing_rotates_downward", left_drop > 0.14, f"left wing z-drop={left_drop:.3f}")
            ctx.check("right_wing_rotates_downward", right_drop > 0.14, f"right wing z-drop={right_drop:.3f}")
            ctx.check(
                "left_brace_lug_moves_below_deck_when_folded",
                left_outer_fold[1][2] < left_outer_rest[1][2] - 0.14,
                f"folded_z={left_outer_fold[1][2]:.3f}, rest_z={left_outer_rest[1][2]:.3f}",
            )
            ctx.check(
                "right_brace_lug_moves_below_deck_when_folded",
                right_outer_fold[1][2] < right_outer_rest[1][2] - 0.14,
                f"folded_z={right_outer_fold[1][2]:.3f}, rest_z={right_outer_rest[1][2]:.3f}",
            )

    with ctx.pose({left_wing_fold: 0.0, right_wing_fold: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wings_at_upper_limit_no_overlap")
        ctx.fail_if_isolated_parts(name="wings_at_upper_limit_no_floating")

    with ctx.pose({front_transport_wheel_roll: math.pi / 2.0, rear_transport_wheel_roll: math.pi / 4.0}):
        ctx.expect_contact(main_frame, front_transport_wheel, contact_tol=0.0015, name="front_transport_wheel_keeps_axle_contact_while_rolling")
        ctx.expect_contact(main_frame, rear_transport_wheel, contact_tol=0.0015, name="rear_transport_wheel_keeps_axle_contact_while_rolling")
        front_valve_turn = ctx.part_element_world_aabb(front_transport_wheel, elem=front_transport_valve_stem)
        has_valve_turn_bounds = ctx.check(
            "front_transport_valve_has_rotated_bounds",
            front_valve_turn is not None,
            "front transport valve stem lacks turned bounds",
        )
        if has_rest_outer_bounds and has_valve_turn_bounds and front_valve_rest is not None and front_valve_turn is not None:
            rest_center_x = (front_valve_rest[0][0] + front_valve_rest[1][0]) * 0.5
            rest_center_z = (front_valve_rest[0][2] + front_valve_rest[1][2]) * 0.5
            turn_center_x = (front_valve_turn[0][0] + front_valve_turn[1][0]) * 0.5
            turn_center_z = (front_valve_turn[0][2] + front_valve_turn[1][2]) * 0.5
            ctx.check("front_transport_wheel_rotation_moves_valve_in_x", turn_center_x > rest_center_x + 0.010, f"rest_x={rest_center_x:.3f}, turn_x={turn_center_x:.3f}")
            ctx.check("front_transport_wheel_rotation_moves_valve_in_z", turn_center_z < rest_center_z - 0.010, f"rest_z={rest_center_z:.3f}, turn_z={turn_center_z:.3f}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
