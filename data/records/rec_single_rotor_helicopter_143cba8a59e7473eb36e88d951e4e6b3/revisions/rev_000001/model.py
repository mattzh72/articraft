from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _blade_geometry(
    *,
    length: float,
    root_chord: float,
    tip_chord: float,
    thickness: float,
    root_embed: float,
    tip_length: float,
):
    half_root = root_chord * 0.5
    half_tip = tip_chord * 0.5
    profile = [
        (-half_root, -root_embed),
        (half_root, -root_embed),
        (half_root * 0.92, length * 0.16),
        (half_tip, length - tip_length),
        (0.0, length),
        (-half_tip, length - tip_length),
        (-half_root * 0.92, length * 0.16),
    ]
    return ExtrudeGeometry.from_z0(profile, thickness, cap=True)


def _fuselage_shell():
    return superellipse_side_loft(
        [
            (1.90, 0.95, 1.28, 0.10),
            (1.52, 0.78, 1.72, 0.84),
            (0.92, 0.58, 2.06, 1.82),
            (0.20, 0.48, 2.18, 2.24),
            (-0.72, 0.50, 2.12, 2.08),
            (-1.55, 0.86, 1.96, 1.12),
            (-2.45, 1.34, 1.96, 0.56),
            (-3.45, 1.60, 2.04, 0.34),
            (-4.42, 1.72, 2.18, 0.24),
        ],
        exponents=2.45,
        segments=64,
    )


def _sponson_geometry():
    return superellipse_side_loft(
        [
            (0.10, 0.86, 1.18, 0.22),
            (-0.35, 0.82, 1.18, 0.34),
            (-0.85, 0.80, 1.12, 0.36),
            (-1.25, 0.84, 1.05, 0.26),
        ],
        exponents=2.0,
        segments=40,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corporate_transport_helicopter")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.12, 0.19, 0.34, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.12, 0.18, 0.22, 0.82))
    metal_gray = model.material("metal_gray", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.34, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))

    fuselage = model.part("fuselage")
    fuselage.inertial = Inertial.from_geometry(
        Box((2.30, 6.40, 2.30)),
        mass=2600.0,
        origin=Origin(xyz=(0.0, -0.55, 1.34)),
    )

    fuselage.visual(_save_mesh("fuselage_shell", _fuselage_shell()), material=body_white, name="body_shell")
    fuselage.visual(
        Box((1.02, 0.44, 0.24)),
        origin=Origin(xyz=(0.0, 0.28, 2.10)),
        material=body_white,
        name="roof_doghouse",
    )
    fuselage.visual(
        Box((0.86, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, -0.09, 2.17)),
        material=metal_gray,
        name="panel_hinge_beam",
    )
    fuselage.visual(
        Box((1.36, 1.90, 0.26)),
        origin=Origin(xyz=(0.0, -0.18, 0.54)),
        material=accent_blue,
        name="belly_keel",
    )
    fuselage.visual(
        Box((0.74, 0.42, 0.18)),
        origin=Origin(xyz=(0.0, 1.60, 0.94)),
        material=accent_blue,
        name="nose_radome",
    )

    right_sponson = _sponson_geometry().translate(0.95, 0.0, 0.0)
    left_sponson = _sponson_geometry().translate(-0.95, 0.0, 0.0)
    fuselage.visual(_save_mesh("right_sponson", right_sponson), material=body_white, name="right_sponson")
    fuselage.visual(_save_mesh("left_sponson", left_sponson), material=body_white, name="left_sponson")

    fuselage.visual(
        Cylinder(radius=0.16, length=0.44),
        origin=Origin(xyz=(0.0, 0.18, 2.34)),
        material=dark_metal,
        name="mast_pylon",
    )
    fuselage.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.18, 2.46)),
        material=metal_gray,
        name="mast_cap",
    )
    fuselage.visual(
        Cylinder(radius=0.10, length=0.28),
        origin=Origin(xyz=(0.12, -4.40, 2.28), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((0.10, 0.76, 0.74)),
        origin=Origin(xyz=(0.0, -4.00, 2.34)),
        material=accent_blue,
        name="tail_fin",
    )
    fuselage.visual(
        Box((1.18, 0.28, 0.07)),
        origin=Origin(xyz=(0.0, -3.72, 1.92)),
        material=accent_blue,
        name="tailplane",
    )

    fuselage.visual(
        Box((0.92, 0.07, 0.68)),
        origin=Origin(xyz=(0.0, 1.34, 1.56), rpy=(0.68, 0.0, 0.0)),
        material=glass_tint,
        name="windscreen",
    )
    fuselage.visual(
        Box((0.04, 1.36, 0.52)),
        origin=Origin(xyz=(1.03, 0.20, 1.52)),
        material=glass_tint,
        name="right_window_band",
    )
    fuselage.visual(
        Box((0.04, 1.36, 0.52)),
        origin=Origin(xyz=(-1.03, 0.20, 1.52)),
        material=glass_tint,
        name="left_window_band",
    )
    fuselage.visual(
        Box((0.04, 0.52, 0.40)),
        origin=Origin(xyz=(0.74, 1.06, 1.56)),
        material=glass_tint,
        name="right_cockpit_window",
    )
    fuselage.visual(
        Box((0.04, 0.52, 0.40)),
        origin=Origin(xyz=(-0.74, 1.06, 1.56)),
        material=glass_tint,
        name="left_cockpit_window",
    )
    fuselage.visual(
        Box((0.04, 4.70, 0.16)),
        origin=Origin(xyz=(1.00, -0.85, 1.08)),
        material=accent_blue,
        name="right_stripe",
    )
    fuselage.visual(
        Box((0.04, 4.70, 0.16)),
        origin=Origin(xyz=(-1.00, -0.85, 1.08)),
        material=accent_blue,
        name="left_stripe",
    )

    for side_sign, side_name in ((1.0, "right"), (-1.0, "left")):
        wheel_x = 1.05 * side_sign
        fuselage.visual(
            Cylinder(radius=0.045, length=0.78),
            origin=Origin(xyz=(wheel_x, -0.72, 0.72)),
            material=metal_gray,
            name=f"{side_name}_main_leg",
        )
        fuselage.visual(
            Cylinder(radius=0.034, length=0.24),
            origin=Origin(xyz=(wheel_x, -0.72, 0.30), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal_gray,
            name=f"{side_name}_main_axle",
        )
        fuselage.visual(
            Cylinder(radius=0.20, length=0.12),
            origin=Origin(xyz=(1.09 * side_sign, -0.72, 0.24), rpy=(0.0, pi / 2.0, 0.0)),
            material=tire_black,
            name=f"{side_name}_main_wheel",
        )
        fuselage.visual(
            Cylinder(radius=0.10, length=0.10),
            origin=Origin(xyz=(1.09 * side_sign, -0.72, 0.24), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal_gray,
            name=f"{side_name}_main_hub",
        )

    fuselage.visual(
        Cylinder(radius=0.040, length=0.84),
        origin=Origin(xyz=(0.0, 1.20, 0.62)),
        material=metal_gray,
        name="nose_leg",
    )
    fuselage.visual(
        Box((0.18, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, 1.20, 0.34)),
        material=metal_gray,
        name="nose_fork",
    )
    fuselage.visual(
        Cylinder(radius=0.028, length=0.36),
        origin=Origin(xyz=(0.0, 1.24, 0.24), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_gray,
        name="nose_axle",
    )
    for x_offset, idx in ((0.11, 0), (-0.11, 1)):
        fuselage.visual(
            Cylinder(radius=0.14, length=0.09),
            origin=Origin(xyz=(x_offset, 1.24, 0.17), rpy=(0.0, pi / 2.0, 0.0)),
            material=tire_black,
            name=f"nose_wheel_{idx}",
        )
        fuselage.visual(
            Cylinder(radius=0.07, length=0.08),
            origin=Origin(xyz=(x_offset, 1.24, 0.17), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal_gray,
            name=f"nose_hub_{idx}",
        )

    passenger_door = model.part("passenger_door")
    passenger_door.inertial = Inertial.from_geometry(
        Box((0.06, 0.98, 1.20)),
        mass=42.0,
        origin=Origin(xyz=(0.03, -0.49, 0.0)),
    )
    passenger_door.visual(
        Box((0.045, 0.96, 1.18)),
        origin=Origin(xyz=(0.0225, -0.48, 0.0)),
        material=body_white,
        name="door_shell",
    )
    passenger_door.visual(
        Box((0.012, 0.42, 0.34)),
        origin=Origin(xyz=(0.045, -0.49, 0.18)),
        material=glass_tint,
        name="door_window",
    )
    passenger_door.visual(
        Box((0.010, 0.16, 0.05)),
        origin=Origin(xyz=(0.050, -0.72, -0.04)),
        material=metal_gray,
        name="door_handle",
    )
    for idx, z_pos in enumerate((-0.42, 0.0, 0.42)):
        passenger_door.visual(
            Cylinder(radius=0.014, length=0.16),
            origin=Origin(xyz=(0.0, -0.02, z_pos)),
            material=metal_gray,
            name=f"door_hinge_{idx}",
        )

    service_panel = model.part("service_panel")
    service_panel.inertial = Inertial.from_geometry(
        Box((0.82, 0.74, 0.06)),
        mass=22.0,
        origin=Origin(xyz=(0.0, -0.37, 0.03)),
    )
    service_panel.visual(
        Box((0.80, 0.72, 0.045)),
        origin=Origin(xyz=(0.0, -0.36, 0.0225)),
        material=body_white,
        name="panel_shell",
    )
    service_panel.visual(
        Box((0.18, 0.10, 0.025)),
        origin=Origin(xyz=(0.0, -0.60, 0.048)),
        material=accent_blue,
        name="panel_handle",
    )
    service_panel.visual(
        Cylinder(radius=0.018, length=0.40),
        origin=Origin(xyz=(0.0, -0.02, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_gray,
        name="panel_hinge_rod",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=3.70, length=0.16),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )
    main_rotor.visual(
        Cylinder(radius=0.24, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_metal,
        name="hub",
    )
    main_rotor.visual(
        Cylinder(radius=0.08, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=metal_gray,
        name="mast_stub",
    )
    main_rotor.visual(
        Box((0.78, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_metal,
        name="hub_bar_x",
    )
    main_rotor.visual(
        Box((0.12, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_metal,
        name="hub_bar_y",
    )
    for idx, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        blade = _blade_geometry(
            length=3.55,
            root_chord=0.24,
            tip_chord=0.10,
            thickness=0.028,
            root_embed=0.10,
            tip_length=0.24,
        ).rotate_z(angle)
        main_rotor.visual(
            _save_mesh(f"main_blade_{idx}", blade),
            origin=Origin(xyz=(0.0, 0.0, 0.06)),
            material=rotor_gray,
            name=f"blade_{idx}",
        )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.82, length=0.12),
        mass=16.0,
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    tail_rotor.visual(
        Cylinder(radius=0.10, length=0.10),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.08, 0.24, 0.24)),
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
        material=dark_metal,
        name="tail_spider",
    )
    for idx, angle in enumerate((0.0, pi / 2.0, pi, 3.0 * pi / 2.0)):
        tail_blade = _blade_geometry(
            length=0.62,
            root_chord=0.10,
            tip_chord=0.05,
            thickness=0.020,
            root_embed=0.06,
            tip_length=0.08,
        ).rotate_y(pi / 2.0).rotate_x(angle)
        tail_rotor.visual(
            _save_mesh(f"tail_blade_{idx}", tail_blade),
            origin=Origin(xyz=(0.05, 0.0, 0.0)),
            material=rotor_gray,
            name=f"tail_blade_{idx}",
        )

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, 0.18, 2.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=45.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(0.26, -4.40, 2.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=90.0),
    )
    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=service_panel,
        origin=Origin(xyz=(0.0, -0.04, 2.19)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "passenger_door_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=passenger_door,
        origin=Origin(xyz=(1.120, 0.72, 1.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    passenger_door = object_model.get_part("passenger_door")
    service_panel = object_model.get_part("service_panel")
    main_rotor = object_model.get_articulation("main_rotor_spin")
    tail_rotor = object_model.get_articulation("tail_rotor_spin")
    door_hinge = object_model.get_articulation("passenger_door_hinge")
    panel_hinge = object_model.get_articulation("service_panel_hinge")

    ctx.check(
        "main rotor uses continuous vertical spin",
        main_rotor.articulation_type == ArticulationType.CONTINUOUS
        and tuple(main_rotor.axis) == (0.0, 0.0, 1.0)
        and main_rotor.motion_limits is not None
        and main_rotor.motion_limits.lower is None
        and main_rotor.motion_limits.upper is None,
        details=str((main_rotor.articulation_type, main_rotor.axis, main_rotor.motion_limits)),
    )
    ctx.check(
        "tail rotor uses continuous transverse spin",
        tail_rotor.articulation_type == ArticulationType.CONTINUOUS
        and tuple(tail_rotor.axis) == (1.0, 0.0, 0.0)
        and tail_rotor.motion_limits is not None
        and tail_rotor.motion_limits.lower is None
        and tail_rotor.motion_limits.upper is None,
        details=str((tail_rotor.articulation_type, tail_rotor.axis, tail_rotor.motion_limits)),
    )

    ctx.expect_overlap(
        passenger_door,
        fuselage,
        axes="yz",
        elem_a="door_shell",
        min_overlap=0.80,
        name="closed passenger door covers the cabin opening footprint",
    )

    closed_door = ctx.part_element_world_aabb(passenger_door, elem="door_shell")
    open_door = None
    with ctx.pose({door_hinge: 1.15}):
        open_door = ctx.part_element_world_aabb(passenger_door, elem="door_shell")
    ctx.check(
        "passenger door opens outward",
        closed_door is not None
        and open_door is not None
        and open_door[1][0] > closed_door[1][0] + 0.30,
        details=f"closed={closed_door}, open={open_door}",
    )

    closed_panel = ctx.part_element_world_aabb(service_panel, elem="panel_shell")
    open_panel = None
    with ctx.pose({panel_hinge: 1.05}):
        open_panel = ctx.part_element_world_aabb(service_panel, elem="panel_shell")
    ctx.check(
        "service panel opens upward",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.28,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
