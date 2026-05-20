from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    width: float,
    height: float,
    radius: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_center + y_val, z_center + z_val)
        for y_val, z_val in rounded_rect_profile(width, height, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rescue_helicopter")

    body_white = model.material("body_white", rgba=(0.95, 0.96, 0.98, 1.0))
    rescue_red = model.material("rescue_red", rgba=(0.78, 0.10, 0.12, 1.0))
    rotor_black = model.material("rotor_black", rgba=(0.12, 0.13, 0.14, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.08, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.24, 0.30, 0.92))

    fuselage = model.part("fuselage")
    fuselage.visual(
        Box((2.40, 1.62, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=body_white,
        name="floor",
    )
    fuselage.visual(
        Box((2.26, 1.58, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, 1.18)),
        material=body_white,
        name="roof",
    )
    fuselage.visual(
        Box((2.24, 0.04, 1.14)),
        origin=Origin(xyz=(0.00, -0.79, 0.60)),
        material=body_white,
        name="port_wall",
    )
    fuselage.visual(
        Box((0.52, 0.04, 0.92)),
        origin=Origin(xyz=(0.88, 0.79, 0.49)),
        material=body_white,
        name="starboard_front_wall",
    )
    fuselage.visual(
        Box((0.74, 0.04, 0.92)),
        origin=Origin(xyz=(-0.85, 0.79, 0.49)),
        material=body_white,
        name="starboard_rear_wall",
    )
    fuselage.visual(
        Box((1.18, 0.04, 0.18)),
        origin=Origin(xyz=(0.00, 0.79, 0.09)),
        material=body_white,
        name="door_sill",
    )
    fuselage.visual(
        Box((1.24, 0.04, 0.21)),
        origin=Origin(xyz=(0.00, 0.79, 1.07)),
        material=body_white,
        name="door_header",
    )
    fuselage.visual(
        Box((0.10, 1.30, 1.02)),
        origin=Origin(xyz=(1.15, 0.0, 0.57)),
        material=body_white,
        name="front_frame",
    )
    fuselage.visual(
        Box((0.12, 1.46, 1.18)),
        origin=Origin(xyz=(-1.17, 0.0, 0.60)),
        material=body_white,
        name="rear_frame",
    )

    nose_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(1.10, 1.38, 1.02, 0.16, z_center=0.57),
                _yz_section(1.42, 1.10, 0.92, 0.14, z_center=0.56),
                _yz_section(1.70, 0.66, 0.74, 0.10, z_center=0.52),
                _yz_section(1.92, 0.18, 0.34, 0.05, z_center=0.47),
            ]
        ),
        "nose_fairing",
    )
    fuselage.visual(nose_mesh, material=body_white, name="nose_fairing")

    tail_root_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(-1.08, 1.04, 0.82, 0.12, z_center=0.70),
                _yz_section(-1.46, 0.58, 0.48, 0.08, z_center=0.78),
                _yz_section(-1.78, 0.24, 0.24, 0.04, z_center=0.84),
            ]
        ),
        "tail_root",
    )
    fuselage.visual(tail_root_mesh, material=body_white, name="tail_root")
    fuselage.visual(
        Box((2.46, 0.20, 0.20)),
        origin=Origin(xyz=(-2.98, 0.0, 0.84)),
        material=body_white,
        name="tail_boom",
    )
    fuselage.visual(
        Box((0.48, 0.90, 0.05)),
        origin=Origin(xyz=(-3.72, 0.0, 0.91)),
        material=body_white,
        name="tailplane",
    )
    fuselage.visual(
        Box((0.26, 0.05, 0.86)),
        origin=Origin(xyz=(-4.05, 0.0, 1.17)),
        material=body_white,
        name="vertical_fin",
    )
    fuselage.visual(
        Box((0.18, 0.17, 0.16)),
        origin=Origin(xyz=(-4.02, 0.085, 1.08)),
        material=body_white,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((2.66, 0.05, 0.07)),
        origin=Origin(xyz=(-0.20, 0.815, 0.99)),
        material=metal_gray,
        name="door_rail",
    )
    fuselage.visual(
        Box((2.05, 0.045, 0.05)),
        origin=Origin(xyz=(-0.38, 0.812, 0.20)),
        material=metal_gray,
        name="door_guide",
    )
    fuselage.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.10, 0.0, 1.295)),
        material=metal_gray,
        name="mast_pedestal",
    )
    fuselage.visual(
        Box((1.85, 0.015, 0.12)),
        origin=Origin(xyz=(0.10, -0.795, 0.54)),
        material=rescue_red,
        name="port_stripe",
    )
    fuselage.visual(
        Box((1.30, 0.015, 0.12)),
        origin=Origin(xyz=(-0.18, 0.795, 0.54)),
        material=rescue_red,
        name="starboard_stripe",
    )
    fuselage.visual(
        Box((0.012, 0.30, 0.48)),
        origin=Origin(xyz=(1.60, -0.22, 0.76), rpy=(0.0, -0.42, 0.0)),
        material=dark_glass,
        name="windshield_port",
    )
    fuselage.visual(
        Box((0.012, 0.30, 0.48)),
        origin=Origin(xyz=(1.60, 0.22, 0.76), rpy=(0.0, -0.42, 0.0)),
        material=dark_glass,
        name="windshield_starboard",
    )
    fuselage.visual(
        Box((0.52, 0.012, 0.34)),
        origin=Origin(xyz=(1.02, -0.805, 0.75)),
        material=dark_glass,
        name="port_cockpit_window",
    )
    fuselage.visual(
        Box((0.86, 0.012, 0.34)),
        origin=Origin(xyz=(0.16, -0.805, 0.75)),
        material=dark_glass,
        name="port_cabin_window",
    )
    fuselage.visual(
        Box((0.52, 0.012, 0.34)),
        origin=Origin(xyz=(1.02, 0.805, 0.75)),
        material=dark_glass,
        name="starboard_cockpit_window",
    )
    fuselage.visual(
        Box((0.34, 0.012, 0.30)),
        origin=Origin(xyz=(-0.88, 0.805, 0.74)),
        material=dark_glass,
        name="starboard_aft_window",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((6.20, 1.70, 1.60)),
        mass=1450.0,
        origin=Origin(xyz=(-1.10, 0.0, 0.76)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.05, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=metal_gray,
        name="rotor_shaft",
    )
    main_rotor.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=metal_gray,
        name="hub",
    )
    main_rotor.visual(
        Box((2.70, 0.14, 0.025)),
        origin=Origin(xyz=(1.50, 0.0, 0.24)),
        material=rotor_black,
        name="blade_0",
    )
    main_rotor.visual(
        Box((2.70, 0.14, 0.025)),
        origin=Origin(xyz=(-1.50, 0.0, 0.24)),
        material=rotor_black,
        name="blade_1",
    )
    main_rotor.visual(
        Box((0.14, 2.70, 0.025)),
        origin=Origin(xyz=(0.0, 1.50, 0.24)),
        material=rotor_black,
        name="blade_2",
    )
    main_rotor.visual(
        Box((0.14, 2.70, 0.025)),
        origin=Origin(xyz=(0.0, -1.50, 0.24)),
        material=rotor_black,
        name="blade_3",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Box((5.80, 5.80, 0.28)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.02, length=0.10),
        origin=Origin(xyz=(0.0, 0.05, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="tail_shaft",
    )
    tail_rotor.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.52, 0.025, 0.08)),
        origin=Origin(xyz=(0.0, 0.13, 0.0)),
        material=rotor_black,
        name="tail_blade_x",
    )
    tail_rotor.visual(
        Box((0.08, 0.025, 0.52)),
        origin=Origin(xyz=(0.0, 0.13, 0.0)),
        material=rotor_black,
        name="tail_blade_z",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.60, 0.22, 0.60)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
    )

    side_door = model.part("side_door")
    side_door.visual(
        Box((1.30, 0.05, 0.82)),
        material=body_white,
        name="panel",
    )
    side_door.visual(
        Box((1.14, 0.028, 0.045)),
        origin=Origin(xyz=(0.0, -0.022, 0.42)),
        material=metal_gray,
        name="carriage",
    )
    side_door.visual(
        Box((0.08, 0.028, 0.10)),
        origin=Origin(xyz=(0.42, -0.020, 0.37)),
        material=metal_gray,
        name="hanger_front",
    )
    side_door.visual(
        Box((0.08, 0.028, 0.10)),
        origin=Origin(xyz=(-0.42, -0.020, 0.37)),
        material=metal_gray,
        name="hanger_rear",
    )
    side_door.visual(
        Box((0.28, 0.032, 0.04)),
        origin=Origin(xyz=(-0.46, -0.020, -0.37)),
        material=metal_gray,
        name="guide_shoe",
    )
    side_door.visual(
        Box((0.56, 0.010, 0.24)),
        origin=Origin(xyz=(0.08, -0.024, 0.16)),
        material=dark_glass,
        name="window",
    )
    side_door.visual(
        Cylinder(radius=0.015, length=0.11),
        origin=Origin(xyz=(0.45, 0.030, 0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="handle",
    )
    side_door.inertial = Inertial.from_geometry(
        Box((1.32, 0.09, 0.90)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    port_gear = model.part("port_gear")
    port_gear.visual(
        Cylinder(radius=0.03, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=metal_gray,
        name="strut",
    )
    port_gear.visual(
        Box((0.10, 0.12, 0.07)),
        origin=Origin(xyz=(0.0, -0.16, -0.34)),
        material=metal_gray,
        name="fork",
    )
    port_gear.visual(
        Cylinder(radius=0.02, length=0.24),
        origin=Origin(xyz=(0.0, -0.12, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="axle",
    )
    port_gear.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, -0.22, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="wheel",
    )
    port_gear.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.58)),
        mass=28.0,
        origin=Origin(xyz=(0.0, -0.18, -0.24)),
    )

    starboard_gear = model.part("starboard_gear")
    starboard_gear.visual(
        Cylinder(radius=0.03, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
        material=metal_gray,
        name="strut",
    )
    starboard_gear.visual(
        Box((0.10, 0.12, 0.07)),
        origin=Origin(xyz=(0.0, 0.16, -0.34)),
        material=metal_gray,
        name="fork",
    )
    starboard_gear.visual(
        Cylinder(radius=0.02, length=0.24),
        origin=Origin(xyz=(0.0, 0.12, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="axle",
    )
    starboard_gear.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, 0.22, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="wheel",
    )
    starboard_gear.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.58)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.18, -0.24)),
    )

    nose_gear = model.part("nose_gear")
    nose_gear.visual(
        Cylinder(radius=0.025, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.21)),
        material=metal_gray,
        name="strut",
    )
    nose_gear.visual(
        Box((0.08, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        material=metal_gray,
        name="fork",
    )
    nose_gear.visual(
        Cylinder(radius=0.015, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.39), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="axle",
    )
    nose_gear.visual(
        Cylinder(radius=0.13, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.39), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="wheel",
    )
    nose_gear.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.64)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
    )

    model.articulation(
        "fuselage_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.10, 0.0, 1.385)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=45.0),
    )
    model.articulation(
        "fuselage_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-4.02, 0.17, 1.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=80.0),
    )
    model.articulation(
        "fuselage_to_side_door",
        ArticulationType.PRISMATIC,
        parent=fuselage,
        child=side_door,
        origin=Origin(xyz=(0.30, 0.876, 0.59)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.0, lower=0.0, upper=1.00),
    )
    model.articulation(
        "fuselage_to_port_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=port_gear,
        origin=Origin(xyz=(-0.62, -0.54, 0.0)),
    )
    model.articulation(
        "fuselage_to_starboard_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=starboard_gear,
        origin=Origin(xyz=(-0.62, 0.54, 0.0)),
    )
    model.articulation(
        "fuselage_to_nose_gear",
        ArticulationType.FIXED,
        parent=fuselage,
        child=nose_gear,
        origin=Origin(xyz=(1.02, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    side_door = object_model.get_part("side_door")

    main_joint = object_model.get_articulation("fuselage_to_main_rotor")
    tail_joint = object_model.get_articulation("fuselage_to_tail_rotor")
    door_joint = object_model.get_articulation("fuselage_to_side_door")

    ctx.check(
        "main rotor uses continuous vertical spin",
        main_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(main_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_joint.articulation_type}, axis={main_joint.axis}",
    )
    ctx.check(
        "tail rotor uses continuous transverse spin",
        tail_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(tail_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_joint.articulation_type}, axis={tail_joint.axis}",
    )
    ctx.check(
        "side door uses longitudinal slide",
        door_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(door_joint.axis) == (-1.0, 0.0, 0.0)
        and door_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper >= 0.95,
        details=(
            f"type={door_joint.articulation_type}, axis={door_joint.axis}, "
            f"limits={door_joint.motion_limits}"
        ),
    )

    ctx.expect_gap(
        main_rotor,
        fuselage,
        axis="z",
        positive_elem="rotor_shaft",
        negative_elem="mast_pedestal",
        min_gap=0.0,
        max_gap=0.002,
        name="main rotor shaft seats on mast pedestal",
    )
    ctx.expect_gap(
        tail_rotor,
        fuselage,
        axis="y",
        positive_elem="tail_shaft",
        negative_elem="tail_gearbox",
        min_gap=0.0,
        max_gap=0.002,
        name="tail rotor shaft seats on gearbox face",
    )
    ctx.expect_gap(
        side_door,
        fuselage,
        axis="y",
        positive_elem="carriage",
        negative_elem="door_rail",
        min_gap=0.0,
        max_gap=0.01,
        name="door carriage rides just outside the side rail",
    )

    closed_pos = ctx.part_world_position(side_door)
    upper = 1.0
    if door_joint.motion_limits is not None and door_joint.motion_limits.upper is not None:
        upper = door_joint.motion_limits.upper
    with ctx.pose({door_joint: upper}):
        open_pos = ctx.part_world_position(side_door)
        ctx.expect_gap(
            side_door,
            fuselage,
            axis="y",
            positive_elem="carriage",
            negative_elem="door_rail",
            min_gap=0.0,
            max_gap=0.01,
            name="door carriage stays on the rail when open",
        )
    ctx.check(
        "side door slides aft for loading access",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 0.80,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
