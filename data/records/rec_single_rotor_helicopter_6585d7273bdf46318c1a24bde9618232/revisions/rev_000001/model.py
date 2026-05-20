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
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(width: float, height: float, radius: float, x: float):
    radius = min(radius, width * 0.5 - 1e-4, height * 0.5 - 1e-4)
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="police_helicopter")

    police_white = model.material("police_white", rgba=(0.94, 0.95, 0.97, 1.0))
    police_blue = model.material("police_blue", rgba=(0.11, 0.18, 0.34, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.40, 0.57, 0.66, 0.45))
    skid_metal = model.material("skid_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.15, 1.0))

    fuselage = model.part("fuselage")

    cabin_geom = section_loft(
        [
            _yz_section(0.20, 0.28, 0.08, 1.45),
            _yz_section(1.10, 1.12, 0.24, 1.00),
            _yz_section(1.76, 1.74, 0.34, 0.28),
            _yz_section(1.72, 1.68, 0.34, -0.42),
            _yz_section(1.04, 0.96, 0.18, -1.10),
        ]
    )
    fuselage.visual(
        mesh_from_geometry(cabin_geom, "cabin_shell"),
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
        material=police_white,
        name="cabin_shell",
    )

    fuselage.visual(
        Box((2.10, 1.35, 0.20)),
        origin=Origin(xyz=(-0.05, 0.0, 0.68)),
        material=police_white,
        name="belly_fairing",
    )
    fuselage.visual(
        Box((2.05, 0.12, 0.32)),
        origin=Origin(xyz=(0.00, 0.76, 1.12)),
        material=police_blue,
        name="right_stripe",
    )
    fuselage.visual(
        Box((2.05, 0.12, 0.32)),
        origin=Origin(xyz=(0.00, -0.76, 1.12)),
        material=police_blue,
        name="left_stripe",
    )
    fuselage.visual(
        Box((0.52, 1.04, 0.58)),
        origin=Origin(xyz=(1.03, 0.0, 1.63)),
        material=glass_tint,
        name="windshield",
    )
    fuselage.visual(
        Box((0.84, 0.04, 0.52)),
        origin=Origin(xyz=(0.42, -0.84, 1.52)),
        material=glass_tint,
        name="left_window",
    )
    fuselage.visual(
        Box((1.18, 0.08, 1.14)),
        origin=Origin(xyz=(0.08, 0.86, 1.42)),
        material=dark_trim,
        name="door_opening",
    )
    fuselage.visual(
        Box((0.46, 0.08, 0.52)),
        origin=Origin(xyz=(-0.93, 0.86, 1.34)),
        material=dark_trim,
        name="hatch_opening",
    )
    _add_member(
        fuselage,
        (0.72, 0.94, 1.90),
        (-0.88, 0.94, 1.90),
        0.018,
        skid_metal,
        "door_rail",
    )
    fuselage.visual(
        Box((0.12, 0.08, 0.16)),
        origin=Origin(xyz=(0.46, 0.90, 1.82)),
        material=skid_metal,
        name="rail_support_0",
    )
    fuselage.visual(
        Box((0.12, 0.08, 0.16)),
        origin=Origin(xyz=(-0.58, 0.90, 1.82)),
        material=skid_metal,
        name="rail_support_1",
    )
    fuselage.visual(
        Box((0.05, 0.076, 0.54)),
        origin=Origin(xyz=(-1.16, 0.875, 1.34)),
        material=skid_metal,
        name="hatch_post",
    )

    _add_member(
        fuselage,
        (-1.05, 0.0, 1.46),
        (-4.05, 0.0, 1.52),
        0.13,
        police_blue,
        "tail_boom",
    )
    _add_member(
        fuselage,
        (-4.00, 0.0, 1.52),
        (-4.42, 0.0, 1.56),
        0.08,
        police_white,
        "tail_cone",
    )
    fuselage.visual(
        Box((0.62, 0.10, 1.18)),
        origin=Origin(xyz=(-3.95, 0.0, 2.03)),
        material=police_white,
        name="tail_fin",
    )
    fuselage.visual(
        Box((0.62, 0.76, 0.06)),
        origin=Origin(xyz=(-3.10, 0.0, 1.48)),
        material=police_white,
        name="tailplane",
    )
    _add_member(
        fuselage,
        (-4.34, 0.05, 1.58),
        (-4.31, 0.72, 1.62),
        0.035,
        police_blue,
        "tail_pylon",
    )
    fuselage.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(
            xyz=(-4.31, 0.72, 1.62),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=police_blue,
        name="tail_gearbox",
    )

    left_skid = tube_from_spline_points(
        [
            (-1.08, -0.92, 0.30),
            (-0.55, -0.92, 0.23),
            (0.55, -0.92, 0.23),
            (1.15, -0.92, 0.31),
        ],
        radius=0.055,
        samples_per_segment=18,
        radial_segments=18,
    )
    fuselage.visual(
        mesh_from_geometry(left_skid, "left_skid"),
        material=skid_metal,
        name="left_skid",
    )
    right_skid = tube_from_spline_points(
        [
            (-1.08, 0.92, 0.30),
            (-0.55, 0.92, 0.23),
            (0.55, 0.92, 0.23),
            (1.15, 0.92, 0.31),
        ],
        radius=0.055,
        samples_per_segment=18,
        radial_segments=18,
    )
    fuselage.visual(
        mesh_from_geometry(right_skid, "right_skid"),
        material=skid_metal,
        name="right_skid",
    )

    _add_member(
        fuselage,
        (0.48, -0.78, 0.70),
        (0.48, 0.78, 0.70),
        0.045,
        skid_metal,
        "front_crosstube",
    )
    _add_member(
        fuselage,
        (-0.42, -0.78, 0.67),
        (-0.42, 0.78, 0.67),
        0.045,
        skid_metal,
        "rear_crosstube",
    )
    for x, z_top, z_bottom in ((0.48, 0.70, 0.28), (-0.42, 0.67, 0.27)):
        _add_member(
            fuselage,
            (x, -0.78, z_top),
            (x, -0.92, z_bottom),
            0.032,
            skid_metal,
        )
        _add_member(
            fuselage,
            (x, 0.78, z_top),
            (x, 0.92, z_bottom),
            0.032,
            skid_metal,
        )

    fuselage.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(-0.08, 0.0, 2.07)),
        material=police_blue,
        name="mast_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.07, length=0.22),
        origin=Origin(xyz=(-0.08, 0.0, 2.19)),
        material=dark_trim,
        name="mast",
    )

    fuselage.inertial = Inertial.from_geometry(
        Box((6.0, 2.2, 2.6)),
        mass=1800.0,
        origin=Origin(xyz=(-1.40, 0.0, 1.35)),
    )

    access_door = model.part("access_door")
    access_door.visual(
        Box((1.12, 0.05, 1.12)),
        material=police_white,
        name="door_panel",
    )
    access_door.visual(
        Box((0.46, 0.03, 0.46)),
        origin=Origin(xyz=(0.08, -0.01, 0.18)),
        material=glass_tint,
        name="door_window",
    )
    access_door.visual(
        Box((0.04, 0.02, 0.18)),
        origin=Origin(xyz=(0.36, 0.03, -0.02)),
        material=dark_trim,
        name="door_handle",
    )
    access_door.inertial = Inertial.from_geometry(
        Box((1.12, 0.05, 1.12)),
        mass=45.0,
    )

    equipment_hatch = model.part("equipment_hatch")
    equipment_hatch.visual(
        Box((0.44, 0.035, 0.50)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=police_white,
        name="hatch_panel",
    )
    equipment_hatch.visual(
        Cylinder(radius=0.012, length=0.50),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=skid_metal,
        name="hatch_barrel",
    )
    equipment_hatch.visual(
        Box((0.08, 0.01, 0.12)),
        origin=Origin(xyz=(0.36, 0.012, 0.0)),
        material=dark_trim,
        name="hatch_latch",
    )
    equipment_hatch.inertial = Inertial.from_geometry(
        Box((0.44, 0.035, 0.50)),
        mass=12.0,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_trim,
        name="mast_sleeve",
    )
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_trim,
        name="hub",
    )
    main_rotor.visual(
        Box((2.55, 0.16, 0.028)),
        origin=Origin(xyz=(1.23, 0.0, 0.10), rpy=(0.0, math.radians(2.5), 0.0)),
        material=dark_trim,
        name="blade_0",
    )
    main_rotor.visual(
        Box((2.55, 0.16, 0.028)),
        origin=Origin(xyz=(-1.23, 0.0, 0.10), rpy=(0.0, math.radians(-2.5), 0.0)),
        material=dark_trim,
        name="blade_1",
    )
    main_rotor.visual(
        Box((0.16, 2.55, 0.028)),
        origin=Origin(xyz=(0.0, 1.23, 0.10), rpy=(math.radians(-2.5), 0.0, 0.0)),
        material=dark_trim,
        name="blade_2",
    )
    main_rotor.visual(
        Box((0.16, 2.55, 0.028)),
        origin=Origin(xyz=(0.0, -1.23, 0.10), rpy=(math.radians(2.5), 0.0, 0.0)),
        material=dark_trim,
        name="blade_3",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Box((5.20, 5.20, 0.22)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.028, 0.28, 0.05)),
        origin=Origin(xyz=(0.0, 0.195, 0.0)),
        material=dark_trim,
        name="tail_blade_0",
    )
    tail_rotor.visual(
        Box((0.028, 0.28, 0.05)),
        origin=Origin(xyz=(0.0, -0.195, 0.0)),
        material=dark_trim,
        name="tail_blade_1",
    )
    tail_rotor.visual(
        Box((0.028, 0.05, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=dark_trim,
        name="tail_blade_2",
    )
    tail_rotor.visual(
        Box((0.028, 0.05, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=dark_trim,
        name="tail_blade_3",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.12, 0.70, 0.70)),
        mass=16.0,
    )

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(-0.08, 0.0, 2.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=35.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-4.20, 0.72, 1.62)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=60.0),
    )
    model.articulation(
        "door_slide",
        ArticulationType.PRISMATIC,
        parent=fuselage,
        child=access_door,
        origin=Origin(xyz=(0.08, 0.983, 1.42)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.70, lower=0.0, upper=0.62),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=equipment_hatch,
        origin=Origin(xyz=(-1.16, 0.937, 1.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
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
    access_door = object_model.get_part("access_door")
    equipment_hatch = object_model.get_part("equipment_hatch")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    main_rotor_spin = object_model.get_articulation("main_rotor_spin")
    tail_rotor_spin = object_model.get_articulation("tail_rotor_spin")
    door_slide = object_model.get_articulation("door_slide")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.check(
        "main rotor uses vertical continuous spin",
        main_rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(main_rotor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_rotor_spin.articulation_type} axis={main_rotor_spin.axis}",
    )
    ctx.check(
        "tail rotor uses tail axis continuous spin",
        tail_rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(tail_rotor_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={tail_rotor_spin.articulation_type} axis={tail_rotor_spin.axis}",
    )

    ctx.expect_overlap(
        access_door,
        fuselage,
        axes="xz",
        min_overlap=0.85,
        name="door covers the right cabin opening at rest",
    )
    ctx.expect_overlap(
        equipment_hatch,
        fuselage,
        axes="xz",
        min_overlap=0.35,
        name="equipment hatch sits behind the access door",
    )
    ctx.expect_gap(
        main_rotor,
        fuselage,
        axis="z",
        min_gap=0.0,
        max_gap=0.12,
        positive_elem="mast_sleeve",
        negative_elem="mast",
        name="main rotor sits just above the mast fairing",
    )
    ctx.expect_origin_gap(
        tail_rotor,
        fuselage,
        axis="y",
        min_gap=0.30,
        name="tail rotor stays outboard of the tail boom",
    )

    door_closed = ctx.part_world_position(access_door)
    with ctx.pose({door_slide: door_slide.motion_limits.upper}):
        door_open = ctx.part_world_position(access_door)
        ctx.expect_overlap(
            access_door,
            fuselage,
            axes="z",
            min_overlap=1.0,
            name="open door stays level on the side rail",
        )
        ctx.expect_gap(
            access_door,
            equipment_hatch,
            axis="y",
            min_gap=0.001,
            negative_elem="hatch_panel",
            name="open door clears the aft equipment hatch",
        )
    ctx.check(
        "access door slides aft",
        door_closed is not None
        and door_open is not None
        and door_open[0] < door_closed[0] - 0.45
        and abs(door_open[1] - door_closed[1]) < 0.02
        and abs(door_open[2] - door_closed[2]) < 0.02,
        details=f"closed={door_closed}, open={door_open}",
    )

    hatch_closed = ctx.part_element_world_aabb(equipment_hatch, elem="hatch_panel")
    with ctx.pose({hatch_hinge: hatch_hinge.motion_limits.upper}):
        hatch_open = ctx.part_element_world_aabb(equipment_hatch, elem="hatch_panel")
    ctx.check(
        "equipment hatch swings outward on a vertical hinge",
        hatch_closed is not None
        and hatch_open is not None
        and ((hatch_open[0][1] + hatch_open[1][1]) * 0.5)
        > ((hatch_closed[0][1] + hatch_closed[1][1]) * 0.5) + 0.10
        and abs(((hatch_open[0][2] + hatch_open[1][2]) * 0.5) - ((hatch_closed[0][2] + hatch_closed[1][2]) * 0.5))
        < 0.03,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
