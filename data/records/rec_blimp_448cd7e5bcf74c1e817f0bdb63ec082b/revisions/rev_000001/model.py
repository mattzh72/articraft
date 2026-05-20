from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _cylinder_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    name: str | None = None,
) -> None:
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        return
    horizontal = math.sqrt(vx * vx + vy * vy)
    pitch = math.atan2(horizontal, vz)
    yaw = math.atan2(vy, vx) if horizontal > 1e-9 else 0.0
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _airship_envelope_mesh() -> MeshGeometry:
    """Connected cigar envelope: a long cylindrical mid-body with rounded tapering ends."""
    sections = [
        (-15.0, 0.0),
        (-14.2, 0.75),
        (-12.9, 1.85),
        (-10.6, 2.95),
        (-7.0, 3.50),
        (0.0, 3.62),
        (7.0, 3.50),
        (10.9, 2.80),
        (13.5, 1.35),
        (15.0, 0.0),
    ]
    radial_segments = 72
    z_scale = 0.95
    geom = MeshGeometry()
    rings: list[list[int] | int] = []
    for x, radius in sections:
        if radius <= 1e-6:
            rings.append(geom.add_vertex(x, 0.0, 0.0))
        else:
            ring = []
            for i in range(radial_segments):
                theta = 2.0 * math.pi * i / radial_segments
                ring.append(geom.add_vertex(x, radius * math.cos(theta), z_scale * radius * math.sin(theta)))
            rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        if isinstance(a, int) and isinstance(b, list):
            for i in range(radial_segments):
                geom.add_face(a, b[(i + 1) % radial_segments], b[i])
        elif isinstance(a, list) and isinstance(b, int):
            for i in range(radial_segments):
                geom.add_face(a[i], a[(i + 1) % radial_segments], b)
        elif isinstance(a, list) and isinstance(b, list):
            for i in range(radial_segments):
                a0 = a[i]
                a1 = a[(i + 1) % radial_segments]
                b0 = b[i]
                b1 = b[(i + 1) % radial_segments]
                geom.add_face(a0, b0, b1)
                geom.add_face(a0, b1, a1)
    return geom


def _plate_xz(points: list[tuple[float, float]], thickness_y: float) -> MeshGeometry:
    geom = MeshGeometry()
    half = thickness_y * 0.5
    front = [geom.add_vertex(x, half, z) for x, z in points]
    back = [geom.add_vertex(x, -half, z) for x, z in points]
    for i in range(1, len(points) - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _plate_xy(points: list[tuple[float, float]], thickness_z: float) -> MeshGeometry:
    geom = MeshGeometry()
    half = thickness_z * 0.5
    top = [geom.add_vertex(x, y, half) for x, y in points]
    bottom = [geom.add_vertex(x, y, -half) for x, y in points]
    for i in range(1, len(points) - 1):
        geom.add_face(top[0], top[i], top[i + 1])
        geom.add_face(bottom[0], bottom[i + 1], bottom[i])
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(top[i], bottom[i], bottom[j])
        geom.add_face(top[i], bottom[j], top[j])
    return geom


def _fan_mesh(name: str):
    rotor = FanRotorGeometry(
        0.66,
        0.13,
        3,
        thickness=0.085,
        blade_pitch_deg=32.0,
        blade_sweep_deg=27.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18),
        hub=FanRotorHub(style="spinner", bore_diameter=0.055),
    )
    return mesh_from_geometry(rotor, name)


def _wheel_visuals(part, prefix: str, tire_mat: Material, rim_mat: Material, hub_mat: Material) -> None:
    tire = TireGeometry(
        0.32,
        0.16,
        inner_radius=0.22,
        tread=TireTread(style="ribbed", depth=0.012, count=20, land_ratio=0.56),
        grooves=(TireGroove(center_offset=0.0, width=0.018, depth=0.006),),
        sidewall=TireSidewall(style="rounded", bulge=0.06),
        shoulder=TireShoulder(width=0.012, radius=0.004),
    )
    wheel = WheelGeometry(
        0.23,
        0.15,
        rim=WheelRim(inner_radius=0.15, flange_height=0.012, flange_thickness=0.006),
        hub=WheelHub(radius=0.070, width=0.11, cap_style="domed"),
        face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.008, window_radius=0.030),
        bore=WheelBore(style="round", diameter=0.055),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=tire_mat, name="tire")
    part.visual(mesh_from_geometry(wheel, f"{prefix}_rim"), material=rim_mat, name="rim")
    part.visual(Cylinder(radius=0.055, length=0.18), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=hub_mat, name="hub")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communications_blimp")

    envelope_fabric = _mat(model, "envelope_fabric", (0.82, 0.86, 0.86, 1.0))
    blue_marking = _mat(model, "blue_marking", (0.08, 0.22, 0.42, 1.0))
    gondola_paint = _mat(model, "gondola_paint", (0.75, 0.78, 0.76, 1.0))
    dark_glass = _mat(model, "dark_glass", (0.03, 0.09, 0.13, 1.0))
    dark_metal = _mat(model, "dark_metal", (0.13, 0.14, 0.15, 1.0))
    brushed_metal = _mat(model, "brushed_metal", (0.54, 0.56, 0.58, 1.0))
    rubber = _mat(model, "rubber", (0.025, 0.025, 0.023, 1.0))
    prop_black = _mat(model, "prop_black", (0.015, 0.016, 0.017, 1.0))
    fin_paint = _mat(model, "fin_paint", (0.76, 0.81, 0.84, 1.0))

    hull = model.part("hull")
    hull.visual(mesh_from_geometry(_airship_envelope_mesh(), "cigar_envelope"), material=envelope_fabric, name="envelope")
    hull.visual(Box((5.9, 0.95, 0.18)), origin=Origin(xyz=(0.0, 0.0, -3.42)), material=blue_marking, name="keel_fairing")
    hull.visual(
        mesh_from_geometry(
            _plate_xz([(-10.0, 2.35), (-13.75, 2.15), (-13.75, 4.35), (-11.1, 4.95)], 0.18),
            "top_fin",
        ),
        material=fin_paint,
        name="top_fin",
    )
    hull.visual(
        mesh_from_geometry(
            _plate_xz([(-10.0, -2.35), (-13.75, -2.15), (-13.75, -4.35), (-11.1, -4.95)], 0.18),
            "ventral_fin",
        ),
        material=fin_paint,
        name="ventral_fin",
    )
    hull.visual(
        mesh_from_geometry(
            _plate_xy([(-10.15, 2.20), (-13.70, 2.78), (-13.70, 5.05), (-11.05, 5.70)], 0.18),
            "port_tailplane",
        ),
        material=fin_paint,
        name="port_tailplane",
    )
    hull.visual(
        mesh_from_geometry(
            _plate_xy([(-10.15, -2.20), (-13.70, -2.78), (-13.70, -5.05), (-11.05, -5.70)], 0.18),
            "starboard_tailplane",
        ),
        material=fin_paint,
        name="starboard_tailplane",
    )

    gondola = model.part("gondola")
    gondola.visual(Box((5.6, 1.8, 1.30)), material=gondola_paint, name="gondola_body")
    gondola.visual(Box((1.05, 1.82, 0.38)), origin=Origin(xyz=(2.05, 0.0, 0.18)), material=dark_glass, name="forward_windows")
    gondola.visual(Box((0.50, 0.045, 0.32)), origin=Origin(xyz=(0.85, 0.92, 0.20)), material=dark_glass, name="port_window_0")
    gondola.visual(Box((0.50, 0.045, 0.32)), origin=Origin(xyz=(0.10, 0.92, 0.20)), material=dark_glass, name="port_window_1")
    gondola.visual(Box((0.50, 0.045, 0.32)), origin=Origin(xyz=(-0.65, 0.92, 0.20)), material=dark_glass, name="port_window_2")
    gondola.visual(Box((0.50, 0.045, 0.32)), origin=Origin(xyz=(0.85, -0.92, 0.20)), material=dark_glass, name="starboard_window_0")
    gondola.visual(Box((0.50, 0.045, 0.32)), origin=Origin(xyz=(0.10, -0.92, 0.20)), material=dark_glass, name="starboard_window_1")
    gondola.visual(Box((0.50, 0.045, 0.32)), origin=Origin(xyz=(-0.65, -0.92, 0.20)), material=dark_glass, name="starboard_window_2")
    gondola.visual(Box((5.8, 0.18, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.73)), material=dark_metal, name="roof_beam")
    for x in (-2.05, 2.05):
        for y in (-0.42, 0.42):
            _cylinder_between(gondola, (x, y, 0.62), (x, y, 1.04), 0.055, dark_metal, name=f"suspension_{x}_{y}")
    _cylinder_between(gondola, (-2.25, -0.45, 0.67), (-1.25, 0.45, 0.94), 0.035, dark_metal, name="diagonal_stay_0")
    _cylinder_between(gondola, (-2.25, 0.45, 0.67), (-1.25, -0.45, 0.94), 0.035, dark_metal, name="diagonal_stay_1")
    _cylinder_between(gondola, (1.25, -0.45, 0.94), (2.25, 0.45, 0.67), 0.035, dark_metal, name="diagonal_stay_2")
    _cylinder_between(gondola, (1.25, 0.45, 0.94), (2.25, -0.45, 0.67), 0.035, dark_metal, name="diagonal_stay_3")
    gondola.visual(Cylinder(radius=0.16, length=0.16), origin=Origin(xyz=(-1.7, 0.0, -0.73)), material=dark_glass, name="sensor_dome")
    gondola.visual(Cylinder(radius=0.025, length=0.24), origin=Origin(xyz=(-1.1, 0.0, 0.86)), material=brushed_metal, name="antenna_mast")
    gondola.visual(Box((0.85, 0.04, 0.03)), origin=Origin(xyz=(-1.1, 0.0, 0.99)), material=brushed_metal, name="antenna_crossbar")

    def add_engine(name: str, side: float):
        engine = model.part(name)
        nacelle = mesh_from_geometry(CapsuleGeometry(0.35, 0.92, radial_segments=36).rotate_y(math.pi / 2.0), f"{name}_nacelle")
        engine.visual(nacelle, material=brushed_metal, name="nacelle")
        engine.visual(Cylinder(radius=0.23, length=0.12), origin=Origin(xyz=(0.80, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_metal, name="nose_cowl")
        engine.visual(Cylinder(radius=0.055, length=0.12), origin=Origin(xyz=(0.84, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_metal, name="prop_shaft")
        engine.visual(Box((0.22, 0.76, 0.13)), origin=Origin(xyz=(0.42, -0.72 * side, 0.08)), material=dark_metal, name="front_pylon")
        engine.visual(Box((0.22, 0.76, 0.13)), origin=Origin(xyz=(-0.38, -0.72 * side, 0.08)), material=dark_metal, name="rear_pylon")
        return engine

    port_engine = add_engine("port_engine", 1.0)
    starboard_engine = add_engine("starboard_engine", -1.0)

    port_propeller = model.part("port_propeller")
    port_propeller.visual(_fan_mesh("port_propeller_rotor"), material=prop_black, name="rotor")
    starboard_propeller = model.part("starboard_propeller")
    starboard_propeller.visual(_fan_mesh("starboard_propeller_rotor"), material=prop_black, name="rotor")

    landing_gear = model.part("landing_gear")
    landing_gear.visual(Box((1.05, 0.55, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.46)), material=dark_metal, name="mount_plate")
    landing_gear.visual(Cylinder(radius=0.055, length=1.56), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="axle")
    _cylinder_between(landing_gear, (-0.38, -0.20, 0.43), (0.0, -0.66, 0.0), 0.035, dark_metal, name="port_front_strut")
    _cylinder_between(landing_gear, (0.38, -0.20, 0.43), (0.0, -0.66, 0.0), 0.035, dark_metal, name="port_rear_strut")
    _cylinder_between(landing_gear, (-0.38, 0.20, 0.43), (0.0, 0.66, 0.0), 0.035, dark_metal, name="starboard_front_strut")
    _cylinder_between(landing_gear, (0.38, 0.20, 0.43), (0.0, 0.66, 0.0), 0.035, dark_metal, name="starboard_rear_strut")
    _cylinder_between(landing_gear, (0.0, -0.66, 0.0), (0.0, 0.66, 0.0), 0.035, dark_metal, name="cross_tie")

    port_wheel = model.part("port_wheel")
    _wheel_visuals(port_wheel, "port_landing_wheel", rubber, brushed_metal, dark_metal)
    starboard_wheel = model.part("starboard_wheel")
    _wheel_visuals(starboard_wheel, "starboard_landing_wheel", rubber, brushed_metal, dark_metal)

    rudder = model.part("rudder")
    rudder.visual(Box((0.78, 0.12, 1.75)), origin=Origin(xyz=(-0.39, 0.0, 0.88)), material=fin_paint, name="rudder_panel")
    rudder.visual(Cylinder(radius=0.065, length=1.82), origin=Origin(xyz=(-0.10, 0.0, 0.91)), material=dark_metal, name="hinge_barrel")

    port_elevator = model.part("port_elevator")
    port_elevator.visual(Box((0.82, 2.02, 0.12)), origin=Origin(xyz=(-0.41, 0.0, 0.0)), material=fin_paint, name="elevator_panel")
    port_elevator.visual(Cylinder(radius=0.055, length=2.06), origin=Origin(xyz=(-0.08, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="hinge_barrel")
    starboard_elevator = model.part("starboard_elevator")
    starboard_elevator.visual(Box((0.82, 2.02, 0.12)), origin=Origin(xyz=(-0.41, 0.0, 0.0)), material=fin_paint, name="elevator_panel")
    starboard_elevator.visual(Cylinder(radius=0.055, length=2.06), origin=Origin(xyz=(-0.08, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="hinge_barrel")

    model.articulation("hull_to_gondola", ArticulationType.FIXED, parent=hull, child=gondola, origin=Origin(xyz=(0.0, 0.0, -4.55)))
    model.articulation("gondola_to_port_engine", ArticulationType.FIXED, parent=gondola, child=port_engine, origin=Origin(xyz=(0.35, 2.00, -0.40)))
    model.articulation("gondola_to_starboard_engine", ArticulationType.FIXED, parent=gondola, child=starboard_engine, origin=Origin(xyz=(0.35, -2.00, -0.40)))
    model.articulation(
        "port_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=port_engine,
        child=port_propeller,
        origin=Origin(xyz=(0.928, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=70.0),
    )
    model.articulation(
        "starboard_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=starboard_engine,
        child=starboard_propeller,
        origin=Origin(xyz=(0.928, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=70.0),
    )
    model.articulation("gondola_to_gear", ArticulationType.FIXED, parent=gondola, child=landing_gear, origin=Origin(xyz=(0.65, 0.0, -1.14)))
    model.articulation(
        "port_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=landing_gear,
        child=port_wheel,
        origin=Origin(xyz=(0.0, 0.82, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=35.0),
    )
    model.articulation(
        "starboard_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=landing_gear,
        child=starboard_wheel,
        origin=Origin(xyz=(0.0, -0.82, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=35.0),
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-13.75, 0.0, 2.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "port_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=port_elevator,
        origin=Origin(xyz=(-13.70, 3.92, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "starboard_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=starboard_elevator,
        origin=Origin(xyz=(-13.70, -3.92, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    landing_gear = object_model.get_part("landing_gear")
    port_engine = object_model.get_part("port_engine")
    starboard_engine = object_model.get_part("starboard_engine")
    port_propeller = object_model.get_part("port_propeller")
    starboard_propeller = object_model.get_part("starboard_propeller")
    port_wheel = object_model.get_part("port_wheel")
    starboard_wheel = object_model.get_part("starboard_wheel")

    ctx.allow_overlap(
        port_engine,
        port_propeller,
        elem_a="prop_shaft",
        elem_b="rotor",
        reason="The propeller hub is intentionally captured on the engine shaft.",
    )
    ctx.allow_overlap(
        starboard_engine,
        starboard_propeller,
        elem_a="prop_shaft",
        elem_b="rotor",
        reason="The propeller hub is intentionally captured on the engine shaft.",
    )
    ctx.allow_overlap(
        landing_gear,
        port_wheel,
        elem_a="axle",
        elem_b="hub",
        reason="The wheel hub is intentionally mounted around the fixed landing-gear axle.",
    )
    ctx.allow_overlap(
        landing_gear,
        port_wheel,
        elem_a="axle",
        elem_b="rim",
        reason="The landing-gear axle passes through the bored wheel rim at the hub.",
    )
    ctx.allow_overlap(
        landing_gear,
        starboard_wheel,
        elem_a="axle",
        elem_b="hub",
        reason="The wheel hub is intentionally mounted around the fixed landing-gear axle.",
    )
    ctx.allow_overlap(
        landing_gear,
        starboard_wheel,
        elem_a="axle",
        elem_b="rim",
        reason="The landing-gear axle passes through the bored wheel rim at the hub.",
    )

    ctx.expect_gap(
        hull,
        gondola,
        axis="z",
        min_gap=0.30,
        positive_elem="envelope",
        negative_elem="gondola_body",
        name="gondola body hangs below envelope",
    )
    ctx.expect_within(
        gondola,
        hull,
        axes="y",
        margin=0.02,
        inner_elem="gondola_body",
        outer_elem="envelope",
        name="gondola remains on hull centerline",
    )
    ctx.expect_contact(
        landing_gear,
        gondola,
        elem_a="mount_plate",
        elem_b="gondola_body",
        contact_tol=0.003,
        name="gear mount is bolted to gondola underside",
    )
    ctx.expect_overlap(
        port_engine,
        port_propeller,
        axes="x",
        elem_a="prop_shaft",
        elem_b="rotor",
        min_overlap=0.010,
        name="port propeller hub retains shaft insertion",
    )
    ctx.expect_overlap(
        starboard_engine,
        starboard_propeller,
        axes="x",
        elem_a="prop_shaft",
        elem_b="rotor",
        min_overlap=0.010,
        name="starboard propeller hub retains shaft insertion",
    )
    ctx.expect_overlap(
        landing_gear,
        port_wheel,
        axes="y",
        elem_a="axle",
        elem_b="hub",
        min_overlap=0.030,
        name="port wheel hub sits on axle",
    )
    ctx.expect_overlap(
        landing_gear,
        port_wheel,
        axes="y",
        elem_a="axle",
        elem_b="rim",
        min_overlap=0.030,
        name="port wheel rim is retained on axle",
    )
    ctx.expect_overlap(
        landing_gear,
        starboard_wheel,
        axes="y",
        elem_a="axle",
        elem_b="hub",
        min_overlap=0.030,
        name="starboard wheel hub sits on axle",
    )
    ctx.expect_overlap(
        landing_gear,
        starboard_wheel,
        axes="y",
        elem_a="axle",
        elem_b="rim",
        min_overlap=0.030,
        name="starboard wheel rim is retained on axle",
    )
    for joint_name in (
        "port_propeller_spin",
        "starboard_propeller_spin",
        "port_wheel_spin",
        "starboard_wheel_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} is continuous", joint.articulation_type == ArticulationType.CONTINUOUS)
    for joint_name in ("rudder_hinge", "port_elevator_hinge", "starboard_elevator_hinge"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} is limited revolute", joint.articulation_type == ArticulationType.REVOLUTE)

    return ctx.report()


object_model = build_object_model()
