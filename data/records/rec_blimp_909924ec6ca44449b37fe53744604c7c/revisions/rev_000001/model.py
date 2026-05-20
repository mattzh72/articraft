from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


def _trapezoid_prism_xz(points_xz, thickness_y: float) -> MeshGeometry:
    """Extrude a polygon in the XZ plane into a thin Y-thickness airfoil plate."""
    geom = MeshGeometry()
    half = thickness_y / 2.0
    front = [geom.add_vertex(x, -half, z) for x, z in points_xz]
    back = [geom.add_vertex(x, half, z) for x, z in points_xz]
    n = len(points_xz)

    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], front[j], back[j])
        geom.add_face(front[i], back[j], back[i])
    return geom


def _trapezoid_prism_xy(points_xy, thickness_z: float) -> MeshGeometry:
    """Extrude a polygon in the XY plane into a thin Z-thickness stabilizer plate."""
    geom = MeshGeometry()
    half = thickness_z / 2.0
    lower = [geom.add_vertex(x, y, -half) for x, y in points_xy]
    upper = [geom.add_vertex(x, y, half) for x, y in points_xy]
    n = len(points_xy)

    for i in range(1, n - 1):
        geom.add_face(lower[0], lower[i], lower[i + 1])
        geom.add_face(upper[0], upper[i + 1], upper[i])

    for i in range(n):
        j = (i + 1) % n
        geom.add_face(lower[i], lower[j], upper[j])
        geom.add_face(lower[i], upper[j], upper[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_blimp")

    envelope_mat = model.material("warm_silver_fabric", rgba=(0.78, 0.82, 0.80, 1.0))
    seam_mat = model.material("stitched_gray_seams", rgba=(0.28, 0.30, 0.32, 1.0))
    gondola_mat = model.material("matte_white_gondola", rgba=(0.88, 0.90, 0.88, 1.0))
    glass_mat = model.material("blue_tinted_glass", rgba=(0.08, 0.20, 0.32, 0.75))
    dark_mat = model.material("dark_composite", rgba=(0.03, 0.035, 0.04, 1.0))
    fin_mat = model.material("tail_gray_composite", rgba=(0.58, 0.62, 0.61, 1.0))
    control_mat = model.material("control_surface_slate", rgba=(0.36, 0.42, 0.43, 1.0))
    prop_mat = model.material("black_propeller", rgba=(0.01, 0.012, 0.014, 1.0))
    hub_mat = model.material("brushed_metal", rgba=(0.58, 0.56, 0.52, 1.0))
    tire_mat = model.material("rubber_black", rgba=(0.006, 0.006, 0.005, 1.0))

    hull = model.part("hull")

    envelope = SphereGeometry(1.0, width_segments=64, height_segments=32)
    envelope.scale(5.6, 1.18, 1.18)
    hull.visual(
        mesh_from_geometry(envelope, "long_envelope"),
        material=envelope_mat,
        name="long_envelope",
    )

    # Reinforcing bands lie slightly proud of the fabric envelope and give the
    # large airship a readable scale and stitched-panel treatment.
    for idx, x in enumerate((-3.8, -2.2, -0.6, 1.0, 2.6, 4.0)):
        section_radius = 1.18 * math.sqrt(max(0.04, 1.0 - (x / 5.6) ** 2))
        band = TorusGeometry(section_radius * 0.998, 0.018, radial_segments=12, tubular_segments=64)
        hull.visual(
            mesh_from_geometry(band, f"envelope_band_{idx}"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=seam_mat,
            name=f"envelope_band_{idx}",
        )

    # Lower gondola/cabin assembly: small relative to the envelope, but with a
    # glass observation band and sensor turret.
    hull.visual(
        Box((1.55, 0.62, 0.42)),
        origin=Origin(xyz=(0.60, 0.0, -1.46)),
        material=gondola_mat,
        name="gondola_shell",
    )
    hull.visual(
        Box((0.40, 0.64, 0.36)),
        origin=Origin(xyz=(1.48, 0.0, -1.43)),
        material=gondola_mat,
        name="rounded_nose_block",
    )
    hull.visual(
        Sphere(0.32),
        origin=Origin(xyz=(1.72, 0.0, -1.43), rpy=(0.0, 0.0, 0.0)),
        material=gondola_mat,
        name="rounded_nose_cap",
    )
    hull.visual(
        Box((1.20, 0.024, 0.20)),
        origin=Origin(xyz=(0.74, -0.323, -1.34)),
        material=glass_mat,
        name="cabin_window_0",
    )
    hull.visual(
        Box((1.20, 0.024, 0.20)),
        origin=Origin(xyz=(0.74, 0.323, -1.34)),
        material=glass_mat,
        name="cabin_window_1",
    )
    hull.visual(
        Sphere(0.20),
        origin=Origin(xyz=(0.95, 0.0, -1.78)),
        material=dark_mat,
        name="sensor_dome",
    )
    hull.visual(
        Cylinder(radius=0.12, length=0.10),
        origin=Origin(xyz=(0.95, 0.0, -1.60)),
        material=dark_mat,
        name="sensor_neck",
    )

    # Streamlined suspension struts physically carry the gondola from the hull.
    for idx, x in enumerate((0.05, 0.95)):
        hull.visual(
            Box((0.10, 0.12, 0.72)),
            origin=Origin(xyz=(x, 0.0, -1.05)),
            material=seam_mat,
            name=f"gondola_pylon_{idx}",
        )
    for idx, y in enumerate((-0.28, 0.28)):
        hull.visual(
            Box((1.00, 0.055, 0.15)),
            origin=Origin(xyz=(0.50, y, -1.22), rpy=(0.0, 0.18 if y < 0 else -0.18, 0.0)),
            material=seam_mat,
            name=f"diagonal_cabin_strut_{idx}",
        )

    # Side-mounted engines aft of the cabin. Nacelles and pylons are static
    # structure; the propeller rotors are articulated children.
    engine_positions = [(-1.20, -1.54, -0.92), (-1.20, 1.54, -0.92)]
    hull.visual(
        Box((0.78, 1.06, 0.16)),
        origin=Origin(xyz=(-0.86, -1.00, -0.98)),
        material=seam_mat,
        name="engine_pylon_0",
    )
    hull.visual(
        Cylinder(radius=0.23, length=0.78),
        origin=Origin(xyz=(-1.20, -1.54, -0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="engine_nacelle_0",
    )
    hull.visual(
        Cylinder(radius=0.16, length=0.18),
        origin=Origin(xyz=(-0.77, -1.54, -0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_mat,
        name="engine_spinner_base_0",
    )
    hull.visual(
        Box((0.78, 1.06, 0.16)),
        origin=Origin(xyz=(-0.86, 1.00, -0.98)),
        material=seam_mat,
        name="engine_pylon_1",
    )
    hull.visual(
        Cylinder(radius=0.23, length=0.78),
        origin=Origin(xyz=(-1.20, 1.54, -0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="engine_nacelle_1",
    )
    hull.visual(
        Cylinder(radius=0.16, length=0.18),
        origin=Origin(xyz=(-0.77, 1.54, -0.92), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_mat,
        name="engine_spinner_base_1",
    )

    # Fork for the small rear mooring wheel under the gondola tail.
    hull.visual(
        Box((0.52, 0.46, 0.12)),
        origin=Origin(xyz=(-0.27, 0.0, -1.70)),
        material=seam_mat,
        name="wheel_fork_bridge",
    )
    hull.visual(
        Box((0.10, 0.055, 0.46)),
        origin=Origin(xyz=(-0.35, -0.19, -1.97)),
        material=seam_mat,
        name="wheel_fork_tine_0",
    )
    hull.visual(
        Box((0.10, 0.055, 0.46)),
        origin=Origin(xyz=(-0.35, 0.19, -1.97)),
        material=seam_mat,
        name="wheel_fork_tine_1",
    )
    hull.visual(
        Cylinder(radius=0.035, length=0.44),
        origin=Origin(xyz=(-0.35, 0.0, -2.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="wheel_axle_caps",
    )

    # Fixed tail assembly, visibly separate from the envelope with hard-edged
    # fins and stabilizers. The moving rudder/elevator pieces are children.
    vertical_fin = _trapezoid_prism_xz(
        [(-5.05, 0.20), (-5.45, 1.12), (-5.74, 0.86), (-5.77, 0.18)],
        0.075,
    )
    hull.visual(
        mesh_from_geometry(vertical_fin, "vertical_fin"),
        material=fin_mat,
        name="vertical_fin",
    )
    ventral_fin = _trapezoid_prism_xz(
        [(-5.04, -0.18), (-5.55, -0.86), (-5.78, -0.64), (-5.58, -0.12)],
        0.065,
    )
    hull.visual(
        mesh_from_geometry(ventral_fin, "ventral_tail_skeg"),
        material=fin_mat,
        name="ventral_tail_skeg",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        stabilizer = _trapezoid_prism_xy(
            [(-5.02, sign * 0.28), (-5.25, sign * 1.12), (-5.77, sign * 1.18), (-5.72, sign * 0.30)],
            0.075,
        )
        hull.visual(
            mesh_from_geometry(stabilizer, f"horizontal_stabilizer_{idx}"),
            material=fin_mat,
            name=f"horizontal_stabilizer_{idx}",
        )
    hull.visual(
        Cylinder(radius=0.055, length=2.48),
        origin=Origin(xyz=(-5.82, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam_mat,
        name="elevator_hinge_tube",
    )
    hull.visual(
        Cylinder(radius=0.050, length=0.78),
        origin=Origin(xyz=(-5.82, 0.0, 0.54)),
        material=seam_mat,
        name="rudder_hinge_post",
    )

    # Port and starboard propeller rotors.
    prop_geom = FanRotorGeometry(
        0.36,
        0.075,
        5,
        thickness=0.030,
        blade_pitch_deg=33.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=12.0, camber=0.12),
        hub=FanRotorHub(style="spinner", bore_diameter=0.018),
    )
    for idx, (x, y, z) in enumerate(engine_positions):
        prop = model.part(f"propeller_{idx}")
        prop.visual(
            mesh_from_geometry(prop_geom, f"propeller_rotor_{idx}"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_mat,
            name="propeller_rotor",
        )
        prop.visual(
            Cylinder(radius=0.030, length=0.16),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_mat,
            name="propeller_shaft",
        )
        model.articulation(
            f"engine_to_propeller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=hull,
            child=prop,
            origin=Origin(xyz=(x - 0.47, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=90.0),
        )

    # Rudder: child frame is on the vertical hinge line; the panel extends aft.
    rudder = model.part("rudder")
    rudder.visual(
        Box((0.46, 0.055, 0.64)),
        origin=Origin(xyz=(-0.23, 0.0, 0.16)),
        material=control_mat,
        name="rudder_panel",
    )
    rudder.visual(
        Cylinder(radius=0.035, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=hub_mat,
        name="rudder_barrel",
    )
    model.articulation(
        "tail_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-5.82, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.48, upper=0.48),
    )

    # Elevators: left/right surfaces are a single connected moving part joined by
    # a torque tube so the pair moves together on a horizontal hinge.
    elevators = model.part("elevators")
    elevators.visual(
        Cylinder(radius=0.035, length=2.20),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="elevator_torque_tube",
    )
    elevators.visual(
        Box((0.48, 0.78, 0.060)),
        origin=Origin(xyz=(-0.24, -0.72, 0.0)),
        material=control_mat,
        name="elevator_panel_0",
    )
    elevators.visual(
        Box((0.48, 0.78, 0.060)),
        origin=Origin(xyz=(-0.24, 0.72, 0.0)),
        material=control_mat,
        name="elevator_panel_1",
    )
    model.articulation(
        "tail_to_elevators",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=elevators,
        origin=Origin(xyz=(-5.82, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.38, upper=0.38),
    )

    # Mooring wheel. The mesh wheel spins around local X; rotate the visual so
    # the real axle is the blimp's lateral Y axis.
    wheel = model.part("mooring_wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.17,
                0.080,
                inner_radius=0.115,
                tread=TireTread(style="block", depth=0.010, count=16, land_ratio=0.55),
                grooves=(TireGroove(center_offset=0.0, width=0.012, depth=0.003),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
                shoulder=TireShoulder(width=0.010, radius=0.004),
            ),
            "mooring_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=tire_mat,
        name="mooring_tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.116,
                0.065,
                rim=WheelRim(inner_radius=0.070, flange_height=0.008, flange_thickness=0.004),
                hub=WheelHub(radius=0.038, width=0.052, cap_style="domed"),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.016),
                bore=WheelBore(style="round", diameter=0.032),
            ),
            "mooring_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=hub_mat,
        name="mooring_rim",
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=hull,
        child=wheel,
        origin=Origin(xyz=(-0.35, 0.0, -2.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hull = object_model.get_part("hull")
    wheel = object_model.get_part("mooring_wheel")
    rudder = object_model.get_part("rudder")
    elevators = object_model.get_part("elevators")

    prop_0 = object_model.get_part("propeller_0")
    prop_1 = object_model.get_part("propeller_1")
    rudder_joint = object_model.get_articulation("tail_to_rudder")
    elevator_joint = object_model.get_articulation("tail_to_elevators")
    wheel_joint = object_model.get_articulation("fork_to_wheel")

    ctx.allow_overlap(
        hull,
        rudder,
        elem_a="rudder_hinge_post",
        elem_b="rudder_barrel",
        reason="The simplified rudder hinge barrel is captured around the fixed vertical hinge post.",
    )
    ctx.allow_overlap(
        hull,
        elevators,
        elem_a="elevator_hinge_tube",
        elem_b="elevator_torque_tube",
        reason="The elevator torque tube is intentionally nested inside the fixed horizontal hinge tube.",
    )
    ctx.allow_overlap(
        hull,
        elevators,
        elem_a="elevator_hinge_tube",
        elem_b="elevator_panel_0",
        reason="The fixed hinge tube slightly penetrates the elevator leading-edge skin at the hinge line.",
    )
    ctx.allow_overlap(
        hull,
        elevators,
        elem_a="elevator_hinge_tube",
        elem_b="elevator_panel_1",
        reason="The fixed hinge tube slightly penetrates the elevator leading-edge skin at the hinge line.",
    )
    ctx.allow_overlap(
        hull,
        rudder,
        elem_a="rudder_hinge_post",
        elem_b="rudder_panel",
        reason="The vertical hinge post is seated through the rudder leading-edge skin at the hinge line.",
    )
    ctx.allow_overlap(
        hull,
        wheel,
        elem_a="wheel_axle_caps",
        elem_b="mooring_rim",
        reason="The small mooring wheel rim is intentionally captured by the axle running through its bore.",
    )

    ctx.check(
        "propellers use continuous spin joints",
        object_model.get_articulation("engine_to_propeller_0").articulation_type == ArticulationType.CONTINUOUS
        and object_model.get_articulation("engine_to_propeller_1").articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "mooring wheel uses continuous axle joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
    )
    ctx.check(
        "tail control surfaces have bounded hinge travel",
        rudder_joint.motion_limits is not None
        and elevator_joint.motion_limits is not None
        and rudder_joint.motion_limits.lower < 0.0
        and rudder_joint.motion_limits.upper > 0.0
        and elevator_joint.motion_limits.lower < 0.0
        and elevator_joint.motion_limits.upper > 0.0,
    )
    ctx.expect_within(
        wheel,
        hull,
        axes="y",
        margin=0.01,
        inner_elem="mooring_rim",
        outer_elem="wheel_fork_bridge",
        name="mooring wheel sits in the short support fork",
    )
    ctx.expect_overlap(
        wheel,
        hull,
        axes="xz",
        min_overlap=0.04,
        elem_a="mooring_rim",
        elem_b="wheel_axle_caps",
        name="mooring wheel axle passes through the rim",
    )
    ctx.expect_overlap(
        rudder,
        hull,
        axes="z",
        min_overlap=0.40,
        elem_a="rudder_barrel",
        elem_b="rudder_hinge_post",
        name="rudder barrel remains on the vertical hinge post",
    )
    ctx.expect_overlap(
        rudder,
        hull,
        axes="z",
        min_overlap=0.40,
        elem_a="rudder_panel",
        elem_b="rudder_hinge_post",
        name="rudder leading edge is carried by the hinge post",
    )
    ctx.expect_overlap(
        elevators,
        hull,
        axes="y",
        min_overlap=1.50,
        elem_a="elevator_torque_tube",
        elem_b="elevator_hinge_tube",
        name="elevator torque tube spans the horizontal hinge",
    )
    ctx.expect_overlap(
        elevators,
        hull,
        axes="z",
        min_overlap=0.04,
        elem_a="elevator_panel_0",
        elem_b="elevator_hinge_tube",
        name="port elevator leading edge wraps the hinge tube",
    )
    ctx.expect_overlap(
        elevators,
        hull,
        axes="z",
        min_overlap=0.04,
        elem_a="elevator_panel_1",
        elem_b="elevator_hinge_tube",
        name="starboard elevator leading edge wraps the hinge tube",
    )
    ctx.expect_contact(
        prop_0,
        hull,
        contact_tol=0.002,
        elem_a="propeller_shaft",
        elem_b="engine_nacelle_0",
        name="port propeller shaft seats on nacelle",
    )
    ctx.expect_contact(
        prop_1,
        hull,
        contact_tol=0.002,
        elem_a="propeller_shaft",
        elem_b="engine_nacelle_1",
        name="starboard propeller shaft seats on nacelle",
    )
    ctx.expect_overlap(
        prop_0,
        hull,
        axes="yz",
        min_overlap=0.10,
        elem_a="propeller_rotor",
        elem_b="engine_nacelle_0",
        name="port propeller centered on its nacelle axis",
    )
    ctx.expect_overlap(
        prop_1,
        hull,
        axes="yz",
        min_overlap=0.10,
        elem_a="propeller_rotor",
        elem_b="engine_nacelle_1",
        name="starboard propeller centered on its nacelle axis",
    )

    rest_rudder_pos = ctx.part_world_position(rudder)
    rest_elevator_pos = ctx.part_world_position(elevators)
    with ctx.pose({rudder_joint: 0.35, elevator_joint: 0.25, wheel_joint: 1.0}):
        moved_rudder_pos = ctx.part_world_position(rudder)
        moved_elevator_pos = ctx.part_world_position(elevators)
    ctx.check(
        "control hinges keep their origins fixed",
        rest_rudder_pos == moved_rudder_pos and rest_elevator_pos == moved_elevator_pos,
        details=f"rudder {rest_rudder_pos}->{moved_rudder_pos}, elevators {rest_elevator_pos}->{moved_elevator_pos}",
    )

    return ctx.report()


object_model = build_object_model()
