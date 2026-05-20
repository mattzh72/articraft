from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
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


CABIN_HALF_WIDTH = 0.975
ROOF_Z = 1.95
MAIN_MAST_X = 0.10
MAIN_MAST_Z = 2.13
DOOR_CENTER = (-0.05, 1.00, 0.84)
DOOR_TRAVEL = 1.25
HATCH_HINGE = (-0.18, 0.0, 1.96)
TAIL_ROTOR_ORIGIN = (-5.34, 0.24, 1.62)


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


def _make_fuselage_mesh():
    cabin = (
        cq.Workplane("XY")
        .box(3.05, 1.95, 2.20, centered=(True, True, False))
        .translate((-0.30, 0.0, -0.25))
        .edges("|Z")
        .fillet(0.16)
    )

    nose = (
        cq.Workplane("YZ", origin=(1.22, 0.0, 0.88))
        .rect(1.56, 1.56)
        .workplane(offset=0.90)
        .ellipse(0.44, 0.56)
        .loft(combine=True)
    )

    boom = (
        cq.Workplane("YZ", origin=(-1.74, 0.0, 1.02))
        .ellipse(0.34, 0.26)
        .workplane(offset=-3.70)
        .ellipse(0.12, 0.15)
        .loft(combine=True)
    )

    tailplane = (
        cq.Workplane("XY")
        .box(0.90, 1.45, 0.05)
        .translate((-4.55, 0.0, 1.10))
    )

    fin = (
        cq.Workplane("XZ", origin=(-4.90, 0.0, 0.74))
        .polyline([(0.0, 0.0), (0.36, 0.0), (0.26, 1.14), (-0.05, 0.52)])
        .close()
        .extrude(0.06, both=True)
    )

    mast_pylon = (
        cq.Workplane("XY")
        .circle(0.19)
        .extrude(0.22)
        .translate((MAIN_MAST_X, 0.0, ROOF_Z - 0.04))
    )

    body = cabin.union(nose).union(boom).union(tailplane).union(fin).union(mast_pylon)

    inner_cabin = (
        cq.Workplane("XY")
        .box(2.45, 1.70, 1.82, centered=(True, True, False))
        .translate((-0.32, 0.0, -0.10))
    )
    door_cut = cq.Workplane("XY").box(1.12, 0.46, 1.48).translate((-0.05, 0.98, 0.84))
    hatch_cut = cq.Workplane("XY").box(0.60, 0.76, 0.24).translate((-0.51, 0.0, 1.88))

    return body.cut(inner_cabin).cut(door_cut).cut(hatch_cut)


def _make_main_blade_mesh(length: float, root_overlap: float, thickness: float):
    root_chord = 0.30
    mid_chord = 0.24
    tip_chord = 0.13
    tip_round = 0.10
    x0 = -root_overlap
    x1 = length * 0.58
    x2 = length
    return (
        cq.Workplane("XY")
        .moveTo(x0, -root_chord / 2.0)
        .lineTo(x1, -mid_chord / 2.0)
        .lineTo(x2, -tip_chord / 2.0)
        .threePointArc((x2 + tip_round, 0.0), (x2, tip_chord / 2.0))
        .lineTo(x1, mid_chord / 2.0)
        .lineTo(x0, root_chord / 2.0)
        .close()
        .extrude(thickness, both=True)
    )


def _add_wheel_visuals(
    part,
    *,
    prefix: str,
    center_xyz: tuple[float, float, float],
    radius: float,
    width: float,
    side_sign: float,
    wheel_finish,
    tire_finish,
) -> None:
    yaw = side_sign * math.pi / 2.0
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            radius * 0.78,
            width * 0.56,
            rim=WheelRim(
                inner_radius=radius * 0.52,
                flange_height=0.014,
                flange_thickness=0.0045,
            ),
            hub=WheelHub(
                radius=radius * 0.18,
                width=width * 0.44,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=6,
                    circle_diameter=radius * 0.22,
                    hole_diameter=0.010,
                ),
            ),
            face=WheelFace(dish_depth=0.008, front_inset=0.003),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=radius * 0.07),
            bore=WheelBore(style="round", diameter=0.038),
        ),
        f"{prefix}_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            radius,
            width,
            inner_radius=radius * 0.78,
            tread=TireTread(style="circumferential", depth=0.004, count=3),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.004),
        ),
        f"{prefix}_tire",
    )

    part.visual(
        wheel_mesh,
        origin=Origin(xyz=center_xyz, rpy=(0.0, 0.0, yaw)),
        material=wheel_finish,
        name=f"{prefix}_wheel",
    )
    part.visual(
        tire_mesh,
        origin=Origin(xyz=center_xyz, rpy=(0.0, 0.0, yaw)),
        material=tire_finish,
        name=f"{prefix}_tire",
    )
    part.visual(
        Cylinder(radius=0.030, length=width + 0.04),
        origin=Origin(xyz=center_xyz, rpy=(side_sign * math.pi / 2.0, 0.0, 0.0)),
        material=wheel_finish,
        name=f"{prefix}_axle",
    )


def _build_main_gear(
    model: ArticulatedObject,
    *,
    name: str,
    prefix: str,
    side_sign: float,
    wheel_finish,
    tire_finish,
    strut_finish,
):
    gear = model.part(name)

    top_plate_size = (0.18, 0.08, 0.10)
    gear.visual(
        Box(top_plate_size),
        origin=Origin(xyz=(0.02, side_sign * 0.03, -0.04)),
        material=strut_finish,
        name="top_plate",
    )

    p0 = (0.02, side_sign * 0.04, -0.08)
    p1 = (0.10, side_sign * 0.18, -0.34)
    p2 = (0.12, side_sign * 0.27, -0.56)
    p3 = (-0.06, side_sign * 0.04, -0.10)
    p4 = (0.02, side_sign * 0.16, -0.42)
    wheel_center = (0.08, side_sign * 0.33, -0.76)

    _add_member(gear, p0, p1, 0.040, strut_finish, "forward_strut")
    _add_member(gear, p1, p2, 0.035, strut_finish, "oleo")
    _add_member(gear, p3, p4, 0.030, strut_finish, "rear_brace")
    _add_member(gear, p4, p2, 0.024, strut_finish, "lower_brace")
    _add_member(gear, p2, wheel_center, 0.028, strut_finish, "axle_post")

    _add_wheel_visuals(
        gear,
        prefix=prefix,
        center_xyz=wheel_center,
        radius=0.34,
        width=0.18,
        side_sign=side_sign,
        wheel_finish=wheel_finish,
        tire_finish=tire_finish,
    )
    return gear


def _build_nose_gear(model: ArticulatedObject, *, wheel_finish, tire_finish, strut_finish):
    gear = model.part("nose_gear")
    gear.visual(
        Box((0.18, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=strut_finish,
        name="top_plate",
    )
    _add_member(gear, (0.0, 0.0, -0.04), (0.0, 0.0, -0.48), 0.040, strut_finish, "oleo")
    gear.visual(
        Box((0.08, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, -0.56)),
        material=strut_finish,
        name="fork",
    )
    gear.visual(
        Cylinder(radius=0.025, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, -0.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_finish,
        name="axle",
    )

    _add_wheel_visuals(
        gear,
        prefix="nose_outer",
        center_xyz=(0.0, 0.11, -0.78),
        radius=0.26,
        width=0.12,
        side_sign=1.0,
        wheel_finish=wheel_finish,
        tire_finish=tire_finish,
    )
    _add_wheel_visuals(
        gear,
        prefix="nose_inner",
        center_xyz=(0.0, -0.11, -0.78),
        radius=0.26,
        width=0.12,
        side_sign=-1.0,
        wheel_finish=wheel_finish,
        tire_finish=tire_finish,
    )
    return gear


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_helicopter")

    body_olive = model.material("body_olive", rgba=(0.40, 0.49, 0.30, 1.0))
    darker_olive = model.material("darker_olive", rgba=(0.29, 0.35, 0.22, 1.0))
    glass = model.material("glass", rgba=(0.22, 0.29, 0.34, 0.42))
    rotor_gray = model.material("rotor_gray", rgba=(0.23, 0.24, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.76, 0.78, 0.80, 1.0))

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_cadquery(_make_fuselage_mesh(), "fuselage_body"),
        material=body_olive,
        name="body_shell",
    )
    fuselage.visual(
        Box((2.50, 0.04, 0.08)),
        origin=Origin(xyz=(-0.58, 1.00, 1.62)),
        material=dark_steel,
        name="door_rail",
    )
    for bracket_index, bracket_x in enumerate((-1.28, -0.58, 0.12)):
        fuselage.visual(
            Box((0.08, 0.06, 0.10)),
            origin=Origin(xyz=(bracket_x, 0.99, 1.64)),
            material=dark_steel,
            name=f"rail_bracket_{bracket_index}",
        )
    fuselage.visual(
        Box((0.06, 0.10, 0.04)),
        origin=Origin(xyz=(-0.15, 0.22, 1.97)),
        material=dark_steel,
        name="hatch_mount_0",
    )
    fuselage.visual(
        Box((0.06, 0.10, 0.04)),
        origin=Origin(xyz=(-0.15, -0.22, 1.97)),
        material=dark_steel,
        name="hatch_mount_1",
    )
    fuselage.visual(
        Box((0.40, 0.20, 0.20)),
        origin=Origin(xyz=(-5.16, 0.05, 1.62)),
        material=darker_olive,
        name="gearbox_fairing",
    )
    _add_member(
        fuselage,
        (-4.96, 0.00, 1.40),
        (-5.10, 0.05, 1.56),
        0.085,
        darker_olive,
        "gearbox_strut",
    )
    fuselage.visual(
        Box((0.34, 0.12, 0.56)),
        origin=Origin(xyz=(-5.00, 0.02, 1.40)),
        material=darker_olive,
        name="tail_pylon",
    )
    fuselage.visual(
        Cylinder(radius=0.11, length=0.24),
        origin=Origin(xyz=(-5.34, 0.12, 1.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_olive,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((0.02, 0.34, 0.78)),
        origin=Origin(xyz=(1.43, 0.18, 1.20), rpy=(0.0, 0.58, 0.0)),
        material=glass,
        name="windshield_0",
    )
    fuselage.visual(
        Box((0.02, 0.34, 0.78)),
        origin=Origin(xyz=(1.43, -0.18, 1.20), rpy=(0.0, 0.58, 0.0)),
        material=glass,
        name="windshield_1",
    )
    fuselage.visual(
        Box((0.56, 0.02, 0.54)),
        origin=Origin(xyz=(1.04, 0.85, 1.12), rpy=(0.0, -0.18, 0.0)),
        material=glass,
        name="cockpit_side_window_0",
    )
    fuselage.visual(
        Box((0.56, 0.02, 0.54)),
        origin=Origin(xyz=(1.04, -0.85, 1.12), rpy=(0.0, -0.18, 0.0)),
        material=glass,
        name="cockpit_side_window_1",
    )

    port_gear = _build_main_gear(
        model,
        name="port_gear",
        prefix="port",
        side_sign=-1.0,
        wheel_finish=wheel_gray,
        tire_finish=tire_black,
        strut_finish=steel,
    )
    starboard_gear = _build_main_gear(
        model,
        name="starboard_gear",
        prefix="starboard",
        side_sign=1.0,
        wheel_finish=wheel_gray,
        tire_finish=tire_black,
        strut_finish=steel,
    )
    nose_gear = _build_nose_gear(
        model,
        wheel_finish=wheel_gray,
        tire_finish=tire_black,
        strut_finish=steel,
    )

    cargo_door = model.part("cargo_door")
    cargo_door.visual(
        Box((1.12, 0.05, 1.46)),
        material=body_olive,
        name="door_panel",
    )
    cargo_door.visual(
        Box((0.48, 0.01, 0.42)),
        origin=Origin(xyz=(0.10, 0.01, 0.20)),
        material=glass,
        name="door_window",
    )
    cargo_door.visual(
        Box((0.16, 0.03, 0.14)),
        origin=Origin(xyz=(0.32, 0.03, 0.74)),
        material=dark_steel,
        name="front_carrier",
    )
    cargo_door.visual(
        Box((0.16, 0.03, 0.14)),
        origin=Origin(xyz=(-0.32, 0.03, 0.74)),
        material=dark_steel,
        name="rear_carrier",
    )
    cargo_door.visual(
        Box((0.20, 0.03, 0.10)),
        origin=Origin(xyz=(0.22, 0.02, -0.73)),
        material=dark_steel,
        name="front_guide",
    )
    cargo_door.visual(
        Box((0.20, 0.03, 0.10)),
        origin=Origin(xyz=(-0.22, 0.02, -0.73)),
        material=dark_steel,
        name="rear_guide",
    )

    roof_hatch = model.part("roof_hatch")
    roof_hatch.visual(
        Box((0.66, 0.74, 0.04)),
        origin=Origin(xyz=(-0.33, 0.0, 0.02)),
        material=darker_olive,
        name="panel",
    )
    roof_hatch.visual(
        Cylinder(radius=0.022, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.065), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    roof_hatch.visual(
        Box((0.08, 0.16, 0.04)),
        origin=Origin(xyz=(-0.03, 0.22, 0.05)),
        material=dark_steel,
        name="hinge_leaf_0",
    )
    roof_hatch.visual(
        Box((0.08, 0.16, 0.04)),
        origin=Origin(xyz=(-0.03, -0.22, 0.05)),
        material=dark_steel,
        name="hinge_leaf_1",
    )
    roof_hatch.visual(
        Box((0.10, 0.03, 0.04)),
        origin=Origin(xyz=(-0.58, 0.0, 0.04)),
        material=dark_steel,
        name="handle",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.06, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=steel,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_steel,
        name="hub",
    )
    main_blade_mesh = mesh_from_cadquery(
        _make_main_blade_mesh(length=3.16, root_overlap=0.18, thickness=0.028),
        "main_blade",
    )
    blade_specs = (
        ("blade_0", (0.0, 0.0, 0.19), (0.0, 0.0, 0.0)),
        ("blade_1", (0.0, 0.0, 0.19), (0.0, 0.0, math.pi)),
        ("blade_2", (0.0, 0.0, 0.19), (0.0, 0.0, math.pi / 2.0)),
        ("blade_3", (0.0, 0.0, 0.19), (0.0, 0.0, -math.pi / 2.0)),
    )
    for blade_name, blade_center, blade_rpy in blade_specs:
        main_rotor.visual(
            main_blade_mesh,
            origin=Origin(xyz=blade_center, rpy=blade_rpy),
            material=rotor_gray,
            name=blade_name,
        )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, 0.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    tail_rotor.visual(
        Box((0.64, 0.04, 0.10)),
        origin=Origin(xyz=(0.33, 0.02, 0.0)),
        material=rotor_gray,
        name="blade_0",
    )
    tail_rotor.visual(
        Box((0.64, 0.04, 0.10)),
        origin=Origin(xyz=(-0.33, 0.02, 0.0)),
        material=rotor_gray,
        name="blade_1",
    )
    tail_rotor.visual(
        Box((0.10, 0.04, 0.64)),
        origin=Origin(xyz=(0.0, 0.02, 0.33)),
        material=rotor_gray,
        name="blade_2",
    )
    tail_rotor.visual(
        Box((0.10, 0.04, 0.64)),
        origin=Origin(xyz=(0.0, 0.02, -0.33)),
        material=rotor_gray,
        name="blade_3",
    )

    model.articulation(
        "port_gear_mount",
        ArticulationType.FIXED,
        parent=fuselage,
        child=port_gear,
        origin=Origin(xyz=(-0.12, -0.985, 0.02)),
    )
    model.articulation(
        "starboard_gear_mount",
        ArticulationType.FIXED,
        parent=fuselage,
        child=starboard_gear,
        origin=Origin(xyz=(-0.12, 0.985, 0.02)),
    )
    model.articulation(
        "nose_gear_mount",
        ArticulationType.FIXED,
        parent=fuselage,
        child=nose_gear,
        origin=Origin(xyz=(1.02, 0.0, -0.25)),
    )
    model.articulation(
        "cargo_door_slide",
        ArticulationType.PRISMATIC,
        parent=fuselage,
        child=cargo_door,
        origin=Origin(xyz=DOOR_CENTER),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.70, lower=0.0, upper=DOOR_TRAVEL),
    )
    model.articulation(
        "roof_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=roof_hatch,
        origin=Origin(xyz=HATCH_HINGE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(MAIN_MAST_X, 0.0, MAIN_MAST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=25.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=TAIL_ROTOR_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    cargo_door = object_model.get_part("cargo_door")
    roof_hatch = object_model.get_part("roof_hatch")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    door_slide = object_model.get_articulation("cargo_door_slide")
    hatch_hinge = object_model.get_articulation("roof_hatch_hinge")
    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    ctx.check(
        "main rotor uses vertical continuous spin",
        main_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(main_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_spin.articulation_type!r}, axis={main_spin.axis!r}",
    )
    ctx.check(
        "tail rotor uses transverse continuous spin",
        tail_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(tail_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_spin.articulation_type!r}, axis={tail_spin.axis!r}",
    )
    ctx.expect_origin_gap(main_rotor, fuselage, axis="z", min_gap=2.0, max_gap=2.2, name="main rotor mast sits above roof")
    tail_pos = ctx.part_world_position(tail_rotor)
    ctx.check(
        "tail rotor sits at aft boom end",
        tail_pos is not None and tail_pos[0] < -5.2 and tail_pos[2] > 1.4,
        details=f"tail_pos={tail_pos}",
    )

    ctx.expect_within(
        cargo_door,
        fuselage,
        axes="x",
        inner_elem="front_carrier",
        outer_elem="door_rail",
        margin=0.0,
        name="front carrier stays under rail when closed",
    )
    ctx.expect_within(
        cargo_door,
        fuselage,
        axes="x",
        inner_elem="rear_carrier",
        outer_elem="door_rail",
        margin=0.0,
        name="rear carrier stays under rail when closed",
    )

    closed_door_aabb = ctx.part_element_world_aabb(cargo_door, elem="door_panel")
    with ctx.pose({door_slide: DOOR_TRAVEL}):
        ctx.expect_within(
            cargo_door,
            fuselage,
            axes="x",
            inner_elem="front_carrier",
            outer_elem="door_rail",
            margin=0.0,
            name="front carrier stays under rail when open",
        )
        ctx.expect_within(
            cargo_door,
            fuselage,
            axes="x",
            inner_elem="rear_carrier",
            outer_elem="door_rail",
            margin=0.0,
            name="rear carrier stays under rail when open",
        )
        open_door_aabb = ctx.part_element_world_aabb(cargo_door, elem="door_panel")
    closed_door_center = _aabb_center(closed_door_aabb)
    open_door_center = _aabb_center(open_door_aabb)
    ctx.check(
        "cargo door slides aft along cabin wall",
        closed_door_center is not None
        and open_door_center is not None
        and open_door_center[0] < closed_door_center[0] - 1.0
        and abs(open_door_center[1] - closed_door_center[1]) < 0.02
        and abs(open_door_center[2] - closed_door_center[2]) < 0.02,
        details=f"closed={closed_door_center}, open={open_door_center}",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(roof_hatch, elem="panel")
    with ctx.pose({hatch_hinge: 1.0}):
        open_hatch_aabb = ctx.part_element_world_aabb(roof_hatch, elem="panel")
    ctx.check(
        "roof hatch opens upward behind mast",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and float(open_hatch_aabb[1][2]) > float(closed_hatch_aabb[1][2]) + 0.20,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
