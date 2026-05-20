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
)


def _envelope_geometry() -> MeshGeometry:
    """A passenger-airship scale streamlined envelope, nose forward (+X)."""
    length_sections = (
        (-22.5, 0.0),
        (-21.4, 0.16),
        (-18.0, 0.52),
        (-12.0, 0.88),
        (-3.0, 1.00),
        (7.0, 0.96),
        (14.0, 0.70),
        (19.5, 0.30),
        (22.5, 0.0),
    )
    ry = 5.2
    rz = 5.1
    zc = 6.6
    segments = 56

    geom = MeshGeometry()
    rings: list[list[int]] = []
    rear_center = geom.add_vertex(length_sections[0][0], 0.0, zc)
    front_center = None

    for x, scale in length_sections[1:-1]:
        ring: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            # Slightly superelliptic lower half keeps the hull airship-like
            # without becoming a sphere stretched into a placeholder.
            y = ry * scale * math.cos(a)
            z = zc + rz * scale * math.sin(a) * (0.96 if math.sin(a) < 0.0 else 1.0)
            ring.append(geom.add_vertex(x, y, z))
        rings.append(ring)

    front_center = geom.add_vertex(length_sections[-1][0], 0.0, zc)

    first = rings[0]
    for i in range(segments):
        geom.add_face(rear_center, first[(i + 1) % segments], first[i])

    for a_ring, b_ring in zip(rings, rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a_ring[i], a_ring[j], b_ring[j])
            geom.add_face(a_ring[i], b_ring[j], b_ring[i])

    last = rings[-1]
    for i in range(segments):
        geom.add_face(last[i], last[(i + 1) % segments], front_center)

    return geom


def _plate_xz(points: list[tuple[float, float]], thickness_y: float) -> MeshGeometry:
    """Extrude a fin/control-surface polygon drawn in XZ along local Y."""
    geom = MeshGeometry()
    half = thickness_y / 2.0
    front = [geom.add_vertex(x, -half, z) for x, z in points]
    back = [geom.add_vertex(x, half, z) for x, z in points]
    for i in range(1, len(points) - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    for i in range(len(points)):
        j = (i + 1) % len(points)
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _plate_xy(points: list[tuple[float, float]], thickness_z: float) -> MeshGeometry:
    """Extrude a tailplane polygon drawn in XY along local Z."""
    geom = MeshGeometry()
    half = thickness_z / 2.0
    bottom = [geom.add_vertex(x, y, -half) for x, y in points]
    top = [geom.add_vertex(x, y, half) for x, y in points]
    for i in range(1, len(points) - 1):
        geom.add_face(bottom[0], bottom[i + 1], bottom[i])
        geom.add_face(top[0], top[i], top[i + 1])
    for i in range(len(points)):
        j = (i + 1) % len(points)
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sightseeing_blimp")

    hull_mat = Material("warm_envelope_fabric", rgba=(0.92, 0.88, 0.74, 1.0))
    stripe_mat = Material("deep_blue_stripe", rgba=(0.04, 0.18, 0.48, 1.0))
    gondola_mat = Material("white_gondola", rgba=(0.90, 0.93, 0.95, 1.0))
    window_mat = Material("smoked_passenger_glass", rgba=(0.05, 0.12, 0.18, 0.72))
    metal_mat = Material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_mat = Material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    fin_mat = Material("tail_blue_fabric", rgba=(0.10, 0.24, 0.58, 1.0))
    prop_mat = Material("dark_composite_propeller", rgba=(0.02, 0.025, 0.03, 1.0))

    airframe = model.part("airframe")
    airframe.visual(
        mesh_from_geometry(_envelope_geometry(), "streamlined_envelope"),
        material=hull_mat,
        name="streamlined_envelope",
    )

    # Passenger gondola: long rounded cabin slung under the hull.
    gondola_geom = CapsuleGeometry(1.0, 13.6, radial_segments=32, height_segments=8)
    gondola_geom.rotate_y(math.pi / 2.0).scale(1.0, 1.22, 0.82)
    airframe.visual(
        mesh_from_geometry(gondola_geom, "long_gondola_shell"),
        origin=Origin(xyz=(2.5, 0.0, 0.45)),
        material=gondola_mat,
        name="long_gondola_shell",
    )

    # Mounted passenger windows and a blue hull stripe, embedded slightly in the
    # cabin/envelope fabric so they read as fitted panels instead of stickers.
    for side_index, y in enumerate((-1.065, 1.065)):
        airframe.visual(
            Box((12.0, 0.12, 0.34)),
            origin=Origin(xyz=(1.5, y, 0.82)),
            material=window_mat,
            name=f"window_band_{side_index}",
        )
        for i, x in enumerate((-3.2, -1.6, 0.0, 1.6, 3.2, 4.8, 6.4)):
            airframe.visual(
                Box((0.82, 0.14, 0.44)),
                origin=Origin(xyz=(x, y, 0.88)),
                material=window_mat,
                name=f"window_{side_index}_{i}",
            )
        airframe.visual(
            Box((13.5, 0.055, 0.34)),
            origin=Origin(xyz=(1.8, 5.04 if y > 0.0 else -5.04, 6.90)),
            material=stripe_mat,
            name=f"hull_stripe_{side_index}",
        )

    # Gondola suspension struts are actual load paths from cabin roof to hull.
    for i, x in enumerate((-4.2, -1.0, 2.2, 5.4, 8.6)):
        for side_index, y in enumerate((-0.76, 0.76)):
            airframe.visual(
                Box((0.13, 0.13, 1.40)),
                origin=Origin(xyz=(x, y, 1.72)),
                material=metal_mat,
                name=f"suspension_strut_{i}_{side_index}",
            )

    # Side propeller pods with real pylons and short shafts.
    for side_index, y in enumerate((-2.58, 2.58)):
        airframe.visual(
            Cylinder(0.38, 1.62),
            origin=Origin(xyz=(2.4, y, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"side_pod_{side_index}",
        )
        airframe.visual(
            Box((0.50, 1.90, 0.30)),
            origin=Origin(xyz=(2.28, -1.73 if y < 0.0 else 1.73, 1.02)),
            material=metal_mat,
            name=f"pod_pylon_{side_index}",
        )
    airframe.visual(
        Cylinder(0.075, 0.50),
        origin=Origin(xyz=(3.35, -2.58, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="propeller_shaft_0",
    )
    airframe.visual(
        Cylinder(0.075, 0.50),
        origin=Origin(xyz=(3.35, 2.58, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="propeller_shaft_1",
    )

    # Cruciform fixed tail: top/bottom vertical fins plus left/right tailplanes.
    top_fin = _plate_xz([(-20.7, 7.05), (-16.8, 9.35), (-18.0, 12.25), (-22.25, 9.25)], 0.20)
    bottom_fin = _plate_xz([(-20.7, 6.15), (-16.8, 4.05), (-18.0, 1.55), (-22.25, 4.10)], 0.20)
    airframe.visual(mesh_from_geometry(top_fin, "fixed_top_fin"), material=fin_mat, name="fixed_top_fin")
    airframe.visual(mesh_from_geometry(bottom_fin, "fixed_bottom_fin"), material=fin_mat, name="fixed_bottom_fin")
    airframe.visual(
        Cylinder(0.070, 4.85),
        origin=Origin(xyz=(-22.50, 0.0, 6.62)),
        material=metal_mat,
        name="rudder_hinge_post",
    )
    for side_index, sign in enumerate((-1.0, 1.0)):
        tailplane = _plate_xy(
            [(-17.3, sign * 2.35), (-17.9, sign * 5.15), (-20.55, sign * 5.75), (-20.55, sign * 2.95)],
            0.16,
        )
        airframe.visual(
            mesh_from_geometry(tailplane, f"fixed_tailplane_{side_index}"),
            origin=Origin(xyz=(0.0, 0.0, 6.60)),
            material=fin_mat,
            name=f"fixed_tailplane_{side_index}",
        )
    airframe.visual(
        Cylinder(0.055, 0.72),
        origin=Origin(xyz=(-20.55, -4.35, 6.60), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="elevator_hinge_0",
    )
    airframe.visual(
        Cylinder(0.055, 0.72),
        origin=Origin(xyz=(-20.55, 4.35, 6.60), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="elevator_hinge_1",
    )

    # Supported wheeled undercarriage attached to the gondola structure.
    airframe.visual(
        Box((2.4, 1.9, 0.16)),
        origin=Origin(xyz=(-1.5, 0.0, -0.42)),
        material=metal_mat,
        name="main_gear_mount",
    )
    airframe.visual(
        Cylinder(0.055, 3.25),
        origin=Origin(xyz=(-1.5, 0.0, -0.76), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="main_axle",
    )
    for side_index, y in enumerate((-1.20, 1.20)):
        airframe.visual(
            Box((0.18, 0.18, 0.84)),
            origin=Origin(xyz=(-1.5, y, -0.55)),
            material=metal_mat,
            name=f"main_gear_leg_{side_index}",
        )
        airframe.visual(
            Box((0.60, 0.62, 0.08)),
            origin=Origin(xyz=(-1.5, y * 0.68, -0.25)),
            material=metal_mat,
            name=f"main_gear_outrigger_{side_index}",
        )

    airframe.visual(
        Box((0.55, 0.36, 0.15)),
        origin=Origin(xyz=(7.2, 0.0, -0.42)),
        material=metal_mat,
        name="nose_gear_mount",
    )
    airframe.visual(
        Box((0.12, 0.12, 0.12)),
        origin=Origin(xyz=(7.2, 0.0, -0.43)),
        material=metal_mat,
        name="nose_gear_strut",
    )
    for side_index, y in enumerate((-0.25, 0.25)):
        airframe.visual(
            Box((0.10, 0.09, 0.46)),
            origin=Origin(xyz=(7.2, y, -0.64)),
            material=metal_mat,
            name=f"nose_fork_leg_{side_index}",
        )
    airframe.visual(
        Box((0.14, 0.62, 0.10)),
        origin=Origin(xyz=(7.2, 0.0, -0.44)),
        material=metal_mat,
        name="nose_fork_crown",
    )
    airframe.visual(
        Cylinder(0.042, 0.64),
        origin=Origin(xyz=(7.2, 0.0, -0.83), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="nose_axle",
    )

    # Shared wheel meshes.
    main_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.40,
            0.30,
            inner_radius=0.285,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
            tread=TireTread(style="block", depth=0.018, count=22, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.020, depth=0.006),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.022, radius=0.008),
        ),
        "main_tire",
    )
    main_wheel_rim = Cylinder(0.305, 0.255)
    main_wheel_hub = Cylinder(0.105, 0.345)
    nose_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.31,
            0.24,
            inner_radius=0.220,
            tread=TireTread(style="ribbed", depth=0.010, count=18, land_ratio=0.60),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.015, radius=0.006),
        ),
        "nose_tire",
    )
    nose_wheel_rim = Cylinder(0.235, 0.205)
    nose_wheel_hub = Cylinder(0.080, 0.285)

    for side_index, y in enumerate((-1.52, 1.52)):
        wheel = model.part(f"wheel_{side_index}")
        wheel.visual(main_tire_mesh, material=dark_mat, name="tire")
        wheel.visual(main_wheel_rim, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=metal_mat, name="rim")
        wheel.visual(main_wheel_hub, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=metal_mat, name="hub")
        model.articulation(
            f"axle_to_wheel_{side_index}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=wheel,
            origin=Origin(xyz=(-1.5, y, -0.76), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=60.0, velocity=18.0),
        )

    nose_wheel = model.part("wheel_2")
    nose_wheel.visual(nose_tire_mesh, material=dark_mat, name="tire")
    nose_wheel.visual(nose_wheel_rim, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=metal_mat, name="rim")
    nose_wheel.visual(nose_wheel_hub, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=metal_mat, name="hub")
    model.articulation(
        "axle_to_wheel_2",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=nose_wheel,
        origin=Origin(xyz=(7.2, 0.0, -0.83), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=18.0),
    )

    # Continuous side propellers.  FanRotorGeometry spins about local Z; the
    # joint frame pitches that local Z onto the blimp's longitudinal X axis.
    prop_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.92,
            0.16,
            5,
            thickness=0.16,
            blade_pitch_deg=32.0,
            blade_sweep_deg=25.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.18),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.055, rear_collar_radius=0.13, bore_diameter=0.045),
        ),
        "side_propeller",
    )
    for side_index, y in enumerate((-2.58, 2.58)):
        prop = model.part(f"propeller_{side_index}")
        prop.visual(prop_mesh, material=prop_mat, name="rotor")
        model.articulation(
            f"pod_to_propeller_{side_index}",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=prop,
            origin=Origin(xyz=(3.52, y, 1.02), rpy=(0.0, math.pi / 2.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=25.0, velocity=60.0),
        )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(_plate_xz([(0.0, -2.05), (-1.55, -1.45), (-1.72, 1.62), (0.0, 2.28)], 0.16), "rudder_surface"),
        material=fin_mat,
        name="rudder_surface",
    )
    model.articulation(
        "tail_to_rudder",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-22.52, 0.0, 6.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    for side_index, y in enumerate((-4.35, 4.35)):
        elevator = model.part(f"elevator_{side_index}")
        elevator.visual(
            mesh_from_geometry(
                _plate_xy([(0.02, -1.30), (0.02, 1.30), (-1.60, 1.45), (-1.72, -1.15)], 0.12),
                f"elevator_surface_{side_index}",
            ),
            material=fin_mat,
            name="elevator_surface",
        )
        model.articulation(
            f"tail_to_elevator_{side_index}",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=elevator,
            origin=Origin(xyz=(-20.58, y, 6.60)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.45, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")
    propeller_0 = object_model.get_part("propeller_0")
    propeller_1 = object_model.get_part("propeller_1")
    rudder = object_model.get_part("rudder")
    elevator_0 = object_model.get_part("elevator_0")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    wheel_2 = object_model.get_part("wheel_2")

    ctx.check(
        "two continuous side propellers",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in ("pod_to_propeller_0", "pod_to_propeller_1")
        ),
        details="Both side propellers must be continuous rotors.",
    )
    ctx.check(
        "all landing wheels rotate continuously",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in ("axle_to_wheel_0", "axle_to_wheel_1", "axle_to_wheel_2")
        ),
        details="The undercarriage wheels should spin on their axles.",
    )

    ctx.allow_overlap(
        airframe,
        propeller_0,
        elem_a="propeller_shaft_0",
        elem_b="rotor",
        reason="The propeller hub is intentionally captured on the side pod shaft.",
    )
    ctx.allow_overlap(
        airframe,
        propeller_1,
        elem_a="propeller_shaft_1",
        elem_b="rotor",
        reason="The propeller hub is intentionally captured on the side pod shaft.",
    )
    ctx.allow_overlap(
        airframe,
        wheel_0,
        elem_a="main_axle",
        elem_b="rim",
        reason="The landing wheel rim surrounds the fixed axle at the bearing.",
    )
    ctx.allow_overlap(
        airframe,
        wheel_1,
        elem_a="main_axle",
        elem_b="rim",
        reason="The landing wheel rim surrounds the fixed axle at the bearing.",
    )
    ctx.allow_overlap(
        airframe,
        wheel_2,
        elem_a="nose_axle",
        elem_b="rim",
        reason="The nose wheel rim surrounds the fixed axle at the bearing.",
    )
    ctx.allow_overlap(
        airframe,
        wheel_0,
        elem_a="main_axle",
        elem_b="hub",
        reason="The landing wheel hub is intentionally pierced by the fixed axle.",
    )
    ctx.allow_overlap(
        airframe,
        wheel_1,
        elem_a="main_axle",
        elem_b="hub",
        reason="The landing wheel hub is intentionally pierced by the fixed axle.",
    )
    ctx.allow_overlap(
        airframe,
        wheel_2,
        elem_a="nose_axle",
        elem_b="hub",
        reason="The nose wheel hub is intentionally pierced by the fixed axle.",
    )
    ctx.allow_overlap(
        airframe,
        rudder,
        elem_a="rudder_hinge_post",
        elem_b="rudder_surface",
        reason="The rudder leading edge wraps around the vertical hinge post.",
    )
    ctx.allow_overlap(
        airframe,
        elevator_0,
        elem_a="elevator_hinge_0",
        elem_b="elevator_surface",
        reason="The elevator leading edge is captured by the horizontal hinge pin.",
    )
    ctx.allow_overlap(
        airframe,
        object_model.get_part("elevator_1"),
        elem_a="elevator_hinge_1",
        elem_b="elevator_surface",
        reason="The elevator leading edge is captured by the horizontal hinge pin.",
    )

    ctx.expect_overlap(propeller_0, airframe, axes="x", min_overlap=0.04, elem_a="rotor", elem_b="propeller_shaft_0")
    ctx.expect_overlap(propeller_1, airframe, axes="x", min_overlap=0.04, elem_a="rotor", elem_b="propeller_shaft_1")
    ctx.expect_overlap(wheel_0, airframe, axes="y", min_overlap=0.10, elem_a="rim", elem_b="main_axle")
    ctx.expect_overlap(wheel_1, airframe, axes="y", min_overlap=0.10, elem_a="rim", elem_b="main_axle")
    ctx.expect_overlap(wheel_2, airframe, axes="y", min_overlap=0.08, elem_a="rim", elem_b="nose_axle")
    ctx.expect_overlap(wheel_0, airframe, axes="y", min_overlap=0.10, elem_a="hub", elem_b="main_axle")
    ctx.expect_overlap(wheel_1, airframe, axes="y", min_overlap=0.10, elem_a="hub", elem_b="main_axle")
    ctx.expect_overlap(wheel_2, airframe, axes="y", min_overlap=0.08, elem_a="hub", elem_b="nose_axle")
    ctx.expect_overlap(rudder, airframe, axes="z", min_overlap=2.8, elem_a="rudder_surface", elem_b="rudder_hinge_post")
    ctx.expect_overlap(elevator_0, airframe, axes="y", min_overlap=0.45, elem_a="elevator_surface", elem_b="elevator_hinge_0")

    rudder_joint = object_model.get_articulation("tail_to_rudder")
    elevator_joint = object_model.get_articulation("tail_to_elevator_0")
    with ctx.pose({rudder_joint: 0.40, elevator_joint: 0.32}):
        ctx.expect_origin_distance(rudder, airframe, axes="xy", max_dist=22.6, name="rudder stays at tail hinge")
        ctx.expect_origin_distance(elevator_0, airframe, axes="xy", max_dist=21.2, name="elevator remains tail mounted")

    ctx.expect_gap(
        wheel_0,
        airframe,
        axis="z",
        max_penetration=0.90,
        positive_elem="tire",
        negative_elem="main_gear_mount",
        name="main wheel sits below gondola mount",
    )
    ctx.expect_gap(
        airframe,
        wheel_2,
        axis="z",
        min_gap=0.12,
        positive_elem="long_gondola_shell",
        negative_elem="tire",
        name="nose wheel remains below gondola",
    )

    return ctx.report()


object_model = build_object_model()
