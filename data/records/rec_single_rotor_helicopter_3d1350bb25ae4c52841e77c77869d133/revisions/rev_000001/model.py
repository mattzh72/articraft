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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float,
    y_center: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_center, z + z_center)
        for z, y in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _blade_mesh(
    name: str,
    *,
    length: float,
    root_chord: float,
    tip_chord: float,
    root_thickness: float,
    tip_thickness: float,
    sweep: float,
    tip_drop: float,
) :
    sections = [
        _yz_section(
            x=0.22,
            width=root_chord,
            height=root_thickness,
            radius=min(root_thickness * 0.45, root_chord * 0.12),
            z_center=0.0,
            corner_segments=8,
        ),
        _yz_section(
            x=length * 0.38,
            width=root_chord * 0.88,
            height=root_thickness * 0.78,
            radius=min(root_thickness * 0.35, root_chord * 0.10),
            z_center=tip_drop * 0.20,
            y_center=sweep * 0.35,
            corner_segments=8,
        ),
        _yz_section(
            x=length,
            width=tip_chord,
            height=tip_thickness,
            radius=min(tip_thickness * 0.45, tip_chord * 0.25),
            z_center=tip_drop,
            y_center=sweep,
            corner_segments=8,
        ),
    ]
    return _mesh(name, section_loft(sections))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="search_rescue_helicopter")

    rescue_orange = model.material("rescue_orange", rgba=(0.92, 0.47, 0.12, 1.0))
    off_white = model.material("off_white", rgba=(0.94, 0.95, 0.93, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.26, 0.28, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.69, 0.72, 1.0))
    glass = model.material("glass", rgba=(0.40, 0.57, 0.67, 0.38))
    tire_black = model.material("tire_black", rgba=(0.07, 0.07, 0.07, 1.0))

    main_blade_mesh = _blade_mesh(
        "main_blade",
        length=5.55,
        root_chord=0.34,
        tip_chord=0.16,
        root_thickness=0.055,
        tip_thickness=0.015,
        sweep=0.18,
        tip_drop=-0.04,
    )
    fuselage_shell = _mesh(
        "fuselage_shell",
        section_loft(
            [
                _yz_section(x=2.75, width=1.25, height=1.40, radius=0.26, z_center=1.78),
                _yz_section(x=1.75, width=2.05, height=2.00, radius=0.32, z_center=1.95),
                _yz_section(x=0.25, width=2.42, height=2.34, radius=0.34, z_center=2.02),
                _yz_section(x=-1.55, width=2.40, height=2.30, radius=0.32, z_center=1.98),
                _yz_section(x=-2.65, width=1.40, height=1.34, radius=0.22, z_center=1.86),
            ]
        ),
    )

    fuselage = model.part("fuselage")
    fuselage.visual(fuselage_shell, material=off_white, name="body_shell")
    fuselage.visual(
        Box((1.55, 1.28, 0.44)),
        origin=Origin(xyz=(-0.15, 0.0, 3.16)),
        material=rescue_orange,
        name="engine_cowling",
    )
    fuselage.visual(
        Cylinder(radius=0.33, length=0.40),
        origin=Origin(xyz=(0.25, 0.0, 3.46)),
        material=graphite,
        name="mast_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.26, length=4.90),
        origin=Origin(xyz=(-5.05, 0.0, 2.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="tail_boom",
    )
    fuselage.visual(
        Box((1.32, 0.22, 1.85)),
        origin=Origin(xyz=(-6.72, 0.0, 3.05)),
        material=off_white,
        name="tail_fin",
    )
    fuselage.visual(
        Box((0.92, 0.22, 0.82)),
        origin=Origin(xyz=(-5.95, 0.0, 2.68)),
        material=off_white,
        name="tail_dorsal",
    )
    fuselage.visual(
        Box((1.18, 2.18, 0.08)),
        origin=Origin(xyz=(-5.92, 0.0, 2.12)),
        material=off_white,
        name="tail_stabilizer",
    )
    fuselage.visual(
        Box((0.42, 0.46, 0.46)),
        origin=Origin(xyz=(-7.02, 0.24, 3.18)),
        material=off_white,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((1.55, 0.44, 0.52)),
        origin=Origin(xyz=(-0.15, 1.40, 0.96)),
        material=rescue_orange,
        name="right_sponson",
    )
    fuselage.visual(
        Box((1.55, 0.44, 0.52)),
        origin=Origin(xyz=(-0.15, -1.40, 0.96)),
        material=rescue_orange,
        name="left_sponson",
    )
    fuselage.visual(
        Box((0.72, 0.52, 0.34)),
        origin=Origin(xyz=(1.52, 0.0, 0.78)),
        material=rescue_orange,
        name="nose_bay",
    )

    fuselage.visual(
        Box((0.48, 1.98, 0.96)),
        origin=Origin(xyz=(1.86, 0.0, 2.10), rpy=(0.0, -0.10, 0.0)),
        material=glass,
        name="windshield",
    )
    fuselage.visual(
        Box((0.30, 2.02, 0.18)),
        origin=Origin(xyz=(1.70, 0.0, 2.55), rpy=(0.0, -0.06, 0.0)),
        material=rescue_orange,
        name="windshield_brow",
    )
    fuselage.visual(
        Box((0.72, 0.10, 0.72)),
        origin=Origin(xyz=(1.60, 1.02, 2.14)),
        material=glass,
        name="right_cockpit_window",
    )
    fuselage.visual(
        Box((0.72, 0.10, 0.72)),
        origin=Origin(xyz=(1.60, -1.02, 2.14)),
        material=glass,
        name="left_cockpit_window",
    )
    fuselage.visual(
        Box((0.52, 0.08, 0.58)),
        origin=Origin(xyz=(-1.70, 1.10, 2.16)),
        material=glass,
        name="right_rear_window",
    )
    fuselage.visual(
        Box((0.52, 0.08, 0.58)),
        origin=Origin(xyz=(-1.70, -1.10, 2.16)),
        material=glass,
        name="left_rear_window",
    )

    fuselage.visual(
        Box((2.45, 0.04, 0.08)),
        origin=Origin(xyz=(0.00, 1.23, 2.60)),
        material=dark_metal,
        name="cargo_rail_upper",
    )
    fuselage.visual(
        Box((2.10, 0.03, 0.05)),
        origin=Origin(xyz=(0.00, 1.22, 1.16)),
        material=dark_metal,
        name="cargo_rail_lower",
    )
    fuselage.visual(
        Box((0.08, 0.03, 1.52)),
        origin=Origin(xyz=(1.08, 1.22, 1.88)),
        material=dark_metal,
        name="cargo_rail_front_post",
    )
    fuselage.visual(
        Box((0.08, 0.03, 1.52)),
        origin=Origin(xyz=(-1.08, 1.22, 1.88)),
        material=dark_metal,
        name="cargo_rail_rear_post",
    )

    _add_member(
        fuselage,
        (-0.34, 1.28, 1.01),
        (0.03, 1.78, 0.34),
        0.055,
        steel,
        name="right_main_strut",
    )
    _add_member(
        fuselage,
        (0.14, 1.25, 1.08),
        (0.03, 1.78, 0.34),
        0.036,
        steel,
        name="right_main_brace",
    )
    fuselage.visual(
        Cylinder(radius=0.34, length=0.18),
        origin=Origin(xyz=(0.03, 1.78, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="right_main_tire",
    )
    fuselage.visual(
        Cylinder(radius=0.20, length=0.22),
        origin=Origin(xyz=(0.03, 1.78, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_main_hub",
    )

    _add_member(
        fuselage,
        (-0.34, -1.28, 1.01),
        (0.03, -1.78, 0.34),
        0.055,
        steel,
        name="left_main_strut",
    )
    _add_member(
        fuselage,
        (0.14, -1.25, 1.08),
        (0.03, -1.78, 0.34),
        0.036,
        steel,
        name="left_main_brace",
    )
    fuselage.visual(
        Cylinder(radius=0.34, length=0.18),
        origin=Origin(xyz=(0.03, -1.78, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="left_main_tire",
    )
    fuselage.visual(
        Cylinder(radius=0.20, length=0.22),
        origin=Origin(xyz=(0.03, -1.78, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_main_hub",
    )

    _add_member(
        fuselage,
        (1.52, 0.0, 0.96),
        (1.60, 0.0, 0.28),
        0.045,
        steel,
        name="nose_strut",
    )
    _add_member(
        fuselage,
        (1.34, 0.0, 0.84),
        (1.60, 0.0, 0.28),
        0.030,
        steel,
        name="nose_brace",
    )
    fuselage.visual(
        Cylinder(radius=0.28, length=0.16),
        origin=Origin(xyz=(1.60, 0.0, 0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_black,
        name="nose_tire",
    )
    fuselage.visual(
        Cylinder(radius=0.15, length=0.20),
        origin=Origin(xyz=(1.60, 0.0, 0.28), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nose_hub",
    )

    fuselage.inertial = Inertial.from_geometry(
        Box((10.80, 3.90, 4.10)),
        mass=6200.0,
        origin=Origin(xyz=(-2.40, 0.0, 2.00)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.10, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=dark_metal,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.28, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=graphite,
        name="hub",
    )
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        main_rotor.visual(
            Box((0.48, 0.16, 0.08)),
            origin=Origin(xyz=(0.30, 0.0, 0.43), rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"root_{index}",
        )
        main_rotor.visual(
            main_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.43), rpy=(0.0, 0.0, angle)),
            material=graphite,
            name=f"blade_{index}",
        )
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=5.55, length=0.16),
        mass=380.0,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.12, length=0.20),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hub",
    )
    for index in range(4):
        angle = 0.5 * math.pi * index
        tail_rotor.visual(
            Box((0.18, 0.05, 0.05)),
            origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, angle, 0.0)),
            material=graphite,
            name=f"root_{index}",
        )
        tail_rotor.visual(
            Box((0.92, 0.045, 0.012)),
            origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=(0.0, angle, 0.0)),
            material=graphite,
            name=f"blade_{index}",
        )
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.02, length=0.20),
        mass=42.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    cargo_door = model.part("cargo_door")
    cargo_door.visual(
        Box((1.85, 0.055, 1.52)),
        material=rescue_orange,
        name="panel",
    )
    cargo_door.visual(
        Box((0.18, 0.055, 0.12)),
        origin=Origin(xyz=(0.64, -0.0325, 0.56)),
        material=dark_metal,
        name="front_hanger",
    )
    cargo_door.visual(
        Box((0.18, 0.055, 0.12)),
        origin=Origin(xyz=(-0.64, -0.0325, 0.56)),
        material=dark_metal,
        name="rear_hanger",
    )
    cargo_door.visual(
        Box((0.22, 0.045, 0.08)),
        origin=Origin(xyz=(-0.72, -0.015, -0.70)),
        material=dark_metal,
        name="lower_guide",
    )
    cargo_door.visual(
        Box((0.84, 0.018, 0.54)),
        origin=Origin(xyz=(0.12, 0.010, 0.16)),
        material=glass,
        name="window",
    )
    cargo_door.visual(
        Box((0.10, 0.035, 0.30)),
        origin=Origin(xyz=(0.82, 0.0, 0.0)),
        material=dark_metal,
        name="handle",
    )
    cargo_door.inertial = Inertial.from_geometry(
        Box((1.85, 0.10, 1.54)),
        mass=95.0,
        origin=Origin(),
    )

    maintenance_hatch = model.part("maintenance_hatch")
    maintenance_hatch.visual(
        Box((0.62, 0.045, 0.78)),
        origin=Origin(xyz=(-0.31, 0.0, 0.39)),
        material=rescue_orange,
        name="panel",
    )
    maintenance_hatch.visual(
        Box((0.06, 0.06, 0.78)),
        origin=Origin(xyz=(-0.03, 0.0, 0.39)),
        material=dark_metal,
        name="hinge_barrel",
    )
    maintenance_hatch.visual(
        Box((0.10, 0.048, 0.78)),
        origin=Origin(xyz=(-0.02, 0.001, 0.39)),
        material=dark_metal,
        name="hinge_leaf",
    )
    maintenance_hatch.visual(
        Box((0.08, 0.03, 0.18)),
        origin=Origin(xyz=(-0.55, 0.0, 0.39)),
        material=dark_metal,
        name="handle",
    )
    maintenance_hatch.inertial = Inertial.from_geometry(
        Box((0.64, 0.06, 0.80)),
        mass=18.0,
        origin=Origin(xyz=(-0.31, 0.0, 0.39)),
    )

    model.articulation(
        "mast_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.25, 0.0, 3.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=5.5),
    )
    model.articulation(
        "tail_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-7.05, 0.57, 3.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=16.0),
    )
    model.articulation(
        "cargo_slide",
        ArticulationType.PRISMATIC,
        parent=fuselage,
        child=cargo_door,
        origin=Origin(xyz=(0.60, 1.31, 2.03)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.70,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=maintenance_hatch,
        origin=Origin(xyz=(-0.64, -1.27, 1.86)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
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
    cargo_door = object_model.get_part("cargo_door")
    maintenance_hatch = object_model.get_part("maintenance_hatch")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    cargo_slide = object_model.get_articulation("cargo_slide")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    ctx.expect_gap(
        cargo_door,
        fuselage,
        axis="y",
        positive_elem="panel",
        negative_elem="body_shell",
        min_gap=0.03,
        max_gap=0.10,
        name="cargo door sits on an external rail line",
    )
    ctx.expect_overlap(
        cargo_door,
        fuselage,
        axes="z",
        elem_a="panel",
        elem_b="body_shell",
        min_overlap=1.35,
        name="cargo door spans the tall cabin opening height",
    )

    rest_door = _aabb_center(ctx.part_element_world_aabb(cargo_door, elem="panel"))
    with ctx.pose({cargo_slide: cargo_slide.motion_limits.upper}):
        open_door = _aabb_center(ctx.part_element_world_aabb(cargo_door, elem="panel"))
        ctx.expect_gap(
            cargo_door,
            fuselage,
            axis="y",
            positive_elem="panel",
            negative_elem="body_shell",
            min_gap=0.03,
            max_gap=0.12,
            name="open cargo door remains alongside the cabin wall",
        )
    ctx.check(
        "cargo door slides aft",
        rest_door is not None
        and open_door is not None
        and open_door[0] < rest_door[0] - 0.85,
        details=f"rest={rest_door}, open={open_door}",
    )

    rest_hatch = _aabb_center(ctx.part_element_world_aabb(maintenance_hatch, elem="panel"))
    with ctx.pose({hatch_hinge: hatch_hinge.motion_limits.upper}):
        open_hatch = _aabb_center(ctx.part_element_world_aabb(maintenance_hatch, elem="panel"))
    ctx.check(
        "maintenance hatch opens outward on the left side",
        rest_hatch is not None
        and open_hatch is not None
        and open_hatch[1] < rest_hatch[1] - 0.18,
        details=f"rest={rest_hatch}, open={open_hatch}",
    )

    main_pos = ctx.part_world_position(main_rotor)
    tail_pos = ctx.part_world_position(tail_rotor)
    ctx.check(
        "tail rotor is mounted aft on the fin",
        main_pos is not None
        and tail_pos is not None
        and tail_pos[0] < main_pos[0] - 6.5
        and tail_pos[2] > 2.8,
        details=f"main={main_pos}, tail={tail_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
