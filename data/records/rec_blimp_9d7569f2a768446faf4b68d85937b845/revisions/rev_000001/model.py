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
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _cigar_envelope(length: float, radius_y: float, radius_z: float) -> MeshGeometry:
    """Tapered ellipsoidal airship envelope, long axis in X."""
    rings = 30
    segments = 72
    geom = MeshGeometry()

    tail_tip = geom.add_vertex(-length / 2.0, 0.0, 0.0)
    ring_ids: list[list[int]] = []
    for i in range(1, rings + 1):
        t = i / (rings + 1)
        x = (t - 0.5) * length
        # Fuller middle with softly pointed ends, slightly fuller nose.
        s = math.sin(math.pi * t) ** 0.52
        nose_fullness = 0.96 + 0.07 * t
        ry = radius_y * s * nose_fullness
        rz = radius_z * s * (0.98 + 0.04 * t)
        ring: list[int] = []
        for j in range(segments):
            a = 2.0 * math.pi * j / segments
            ring.append(geom.add_vertex(x, ry * math.cos(a), rz * math.sin(a)))
        ring_ids.append(ring)
    nose_tip = geom.add_vertex(length / 2.0, 0.0, 0.0)

    first = ring_ids[0]
    for j in range(segments):
        geom.add_face(tail_tip, first[(j + 1) % segments], first[j])

    for a, b in zip(ring_ids, ring_ids[1:]):
        for j in range(segments):
            j2 = (j + 1) % segments
            geom.add_face(a[j], a[j2], b[j2])
            geom.add_face(a[j], b[j2], b[j])

    last = ring_ids[-1]
    for j in range(segments):
        geom.add_face(last[j], last[(j + 1) % segments], nose_tip)
    return geom


def _prism_from_xz(points: list[tuple[float, float]], thickness_y: float) -> MeshGeometry:
    """Closed prism from an XZ polygon, extruded in Y."""
    geom = MeshGeometry()
    half = thickness_y / 2.0
    front = [geom.add_vertex(x, half, z) for x, z in points]
    back = [geom.add_vertex(x, -half, z) for x, z in points]
    n = len(points)

    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _prism_from_xy(points: list[tuple[float, float]], thickness_z: float) -> MeshGeometry:
    """Closed prism from an XY polygon, extruded in Z."""
    geom = MeshGeometry()
    half = thickness_z / 2.0
    top = [geom.add_vertex(x, y, half) for x, y in points]
    bottom = [geom.add_vertex(x, y, -half) for x, y in points]
    n = len(points)

    for i in range(1, n - 1):
        geom.add_face(top[0], top[i], top[i + 1])
        geom.add_face(bottom[0], bottom[i + 1], bottom[i])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(top[i], bottom[i], bottom[j])
        geom.add_face(top[i], bottom[j], top[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="communications_blimp")

    envelope_mat = model.material("silver_fabric", rgba=(0.78, 0.83, 0.86, 1.0))
    fin_mat = model.material("tail_blue", rgba=(0.10, 0.20, 0.38, 1.0))
    gondola_mat = model.material("gondola_white", rgba=(0.88, 0.90, 0.86, 1.0))
    window_mat = model.material("smoked_windows", rgba=(0.03, 0.09, 0.14, 1.0))
    metal_mat = model.material("brushed_metal", rgba=(0.45, 0.47, 0.46, 1.0))
    dark_mat = model.material("rubber_black", rgba=(0.01, 0.01, 0.012, 1.0))
    prop_mat = model.material("propeller_dark", rgba=(0.025, 0.025, 0.030, 1.0))
    warning_mat = model.material("orange_markings", rgba=(1.0, 0.42, 0.06, 1.0))

    hull = model.part("hull")
    hull.visual(
        mesh_from_geometry(_cigar_envelope(14.0, 1.58, 1.48), "cigar_envelope"),
        material=envelope_mat,
        name="cigar_envelope",
    )

    # Four fixed tail fins are embedded into the aft envelope as real stabilizer roots.
    upper_fin = _prism_from_xz(
        [(-5.05, 0.86), (-5.55, 2.18), (-6.35, 2.08), (-6.35, 0.78), (-5.45, 0.82)],
        0.11,
    )
    lower_fin = _prism_from_xz(
        [(-5.05, -0.86), (-5.55, -2.05), (-6.55, -1.88), (-6.35, -0.72), (-5.45, -0.82)],
        0.11,
    )
    side_fin_pos = _prism_from_xy(
        [(-5.08, 0.72), (-5.55, 2.14), (-6.35, 2.06), (-6.35, 0.70), (-5.42, 0.70)],
        0.11,
    )
    side_fin_neg = _prism_from_xy(
        [(-5.08, -0.72), (-5.55, -2.14), (-6.35, -2.06), (-6.35, -0.70), (-5.42, -0.70)],
        0.11,
    )
    hull.visual(mesh_from_geometry(upper_fin, "upper_fin"), material=fin_mat, name="upper_fin")
    hull.visual(mesh_from_geometry(lower_fin, "lower_fin"), material=fin_mat, name="lower_fin")
    hull.visual(mesh_from_geometry(side_fin_pos, "side_fin_0"), material=fin_mat, name="side_fin_0")
    hull.visual(mesh_from_geometry(side_fin_neg, "side_fin_1"), material=fin_mat, name="side_fin_1")
    hull.visual(
        Box((0.035, 0.16, 1.35)),
        origin=Origin(xyz=(-6.360, 0.0, 1.43)),
        material=metal_mat,
        name="rudder_hinge",
    )
    hull.visual(
        Box((0.035, 1.42, 0.12)),
        origin=Origin(xyz=(-6.360, 1.39, 0.0)),
        material=metal_mat,
        name="elevator_hinge_0",
    )
    hull.visual(
        Box((0.035, 1.42, 0.12)),
        origin=Origin(xyz=(-6.360, -1.39, 0.0)),
        material=metal_mat,
        name="elevator_hinge_1",
    )

    # Communication utility markings and a dorsal blade antenna seated into the fabric.
    hull.visual(
        Box((2.2, 0.035, 0.16)),
        origin=Origin(xyz=(1.6, 1.57, 0.0)),
        material=warning_mat,
        name="side_marking_0",
    )
    hull.visual(
        Box((2.2, 0.035, 0.16)),
        origin=Origin(xyz=(1.6, -1.57, 0.0)),
        material=warning_mat,
        name="side_marking_1",
    )
    hull.visual(
        Box((0.12, 0.05, 0.42)),
        origin=Origin(xyz=(0.2, 0.0, 1.67)),
        material=metal_mat,
        name="comms_blade",
    )

    gondola = model.part("gondola")
    gondola.visual(Box((3.20, 0.88, 0.70)), material=gondola_mat, name="equipment_cabin")
    gondola.visual(
        Box((0.42, 0.035, 0.20)),
        origin=Origin(xyz=(0.95, 0.445, 0.10)),
        material=window_mat,
        name="window_0",
    )
    gondola.visual(
        Box((0.42, 0.035, 0.20)),
        origin=Origin(xyz=(0.25, 0.445, 0.10)),
        material=window_mat,
        name="window_1",
    )
    gondola.visual(
        Box((0.42, 0.035, 0.20)),
        origin=Origin(xyz=(-0.45, 0.445, 0.10)),
        material=window_mat,
        name="window_2",
    )
    gondola.visual(
        Box((0.42, 0.035, 0.20)),
        origin=Origin(xyz=(0.95, -0.445, 0.10)),
        material=window_mat,
        name="window_3",
    )
    gondola.visual(
        Box((0.42, 0.035, 0.20)),
        origin=Origin(xyz=(0.25, -0.445, 0.10)),
        material=window_mat,
        name="window_4",
    )
    gondola.visual(
        Box((0.42, 0.035, 0.20)),
        origin=Origin(xyz=(-0.45, -0.445, 0.10)),
        material=window_mat,
        name="window_5",
    )
    # Four roof struts keep the gondola visibly suspended below the hull centerline.
    for idx, (x, y) in enumerate(((1.10, 0.32), (1.10, -0.32), (-1.10, 0.32), (-1.10, -0.32))):
        gondola.visual(
            Cylinder(radius=0.035, length=0.50),
            origin=Origin(xyz=(x, y, 0.60)),
            material=metal_mat,
            name=f"suspension_strut_{idx}",
        )
    gondola.visual(
        Box((2.60, 0.72, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
        material=metal_mat,
        name="suspension_spreader",
    )
    gondola.visual(
        Box((1.20, 0.10, 0.10)),
        origin=Origin(xyz=(0.60, 0.0, -0.34)),
        material=metal_mat,
        name="rear_equipment_rail",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -2.25)),
    )

    # Side engine pods on short booms; each propeller is its own continuous rotor.
    for idx, sign in enumerate((1.0, -1.0)):
        engine = model.part(f"engine_pod_{idx}")
        engine.visual(
            mesh_from_geometry(CapsuleGeometry(0.22, 0.64, radial_segments=32), f"nacelle_{idx}"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="nacelle",
        )
        engine.visual(
            Box((0.18, 0.34, 0.16)),
            origin=Origin(xyz=(0.0, -sign * 0.35, 0.0)),
            material=metal_mat,
            name="side_boom",
        )
        engine.visual(
            Cylinder(radius=0.08, length=0.16),
            origin=Origin(xyz=(0.50, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name="shaft_collar",
        )
        model.articulation(
            f"gondola_to_engine_pod_{idx}",
            ArticulationType.FIXED,
            parent=gondola,
            child=engine,
            origin=Origin(xyz=(0.50, sign * 0.93, -0.08)),
        )

        propeller = model.part(f"propeller_{idx}")
        propeller.visual(
            mesh_from_geometry(
                FanRotorGeometry(
                    0.42,
                    0.075,
                    5,
                    thickness=0.050,
                    blade_pitch_deg=34.0,
                    blade_sweep_deg=25.0,
                    blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.12),
                    hub=FanRotorHub(style="spinner", bore_diameter=0.016),
                ),
                f"propeller_rotor_{idx}",
            ),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_mat,
            name="rotor",
        )
        propeller.visual(
            Cylinder(radius=0.024, length=0.17),
            origin=Origin(xyz=(-0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="propeller_shaft",
        )
        model.articulation(
            f"engine_pod_{idx}_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=engine,
            child=propeller,
            origin=Origin(xyz=(0.715, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=80.0),
        )

    # Hinged tail control surfaces.
    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(
            _prism_from_xz([(-0.02, -0.48), (-0.02, 0.56), (-0.68, 0.42), (-0.62, -0.36)], 0.09),
            "rudder_panel",
        ),
        material=fin_mat,
        name="rudder_panel",
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-6.35, 0.0, 1.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.45, upper=0.45),
    )

    for idx, sign in enumerate((1.0, -1.0)):
        elevator = model.part(f"elevator_{idx}")
        inner = sign * 0.72
        outer = sign * 2.06
        mid = sign * 1.39
        elevator.visual(
            mesh_from_geometry(
                _prism_from_xy(
                    [
                        (-0.02, inner - mid),
                        (-0.02, outer - mid),
                        (-0.70, sign * 1.86 - mid),
                        (-0.62, sign * 0.78 - mid),
                    ],
                    0.085,
                ),
                f"elevator_panel_{idx}",
            ),
            material=fin_mat,
            name="elevator_panel",
        )
        model.articulation(
            f"hull_to_elevator_{idx}",
            ArticulationType.REVOLUTE,
            parent=hull,
            child=elevator,
            origin=Origin(xyz=(-6.35, mid, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=14.0, velocity=1.1, lower=-0.35, upper=0.35),
        )

    # Rear mooring wheel and a short U fork mounted under the gondola tail.
    wheel_fork = model.part("wheel_fork")
    wheel_fork.visual(
        Box((0.40, 0.34, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=metal_mat,
        name="fork_mount",
    )
    wheel_fork.visual(
        Box((0.14, 0.050, 0.38)),
        origin=Origin(xyz=(0.0, 0.135, -0.220)),
        material=metal_mat,
        name="fork_cheek_0",
    )
    wheel_fork.visual(
        Box((0.14, 0.050, 0.38)),
        origin=Origin(xyz=(0.0, -0.135, -0.220)),
        material=metal_mat,
        name="fork_cheek_1",
    )
    model.articulation(
        "gondola_to_wheel_fork",
        ArticulationType.FIXED,
        parent=gondola,
        child=wheel_fork,
        origin=Origin(xyz=(-1.12, 0.0, -0.34)),
    )

    mooring_wheel = model.part("mooring_wheel")
    mooring_wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                0.165,
                0.095,
                inner_radius=0.115,
                tread=TireTread(style="ribbed", depth=0.006, count=18, land_ratio=0.60),
                sidewall=TireSidewall(style="rounded", bulge=0.04),
                shoulder=TireShoulder(width=0.006, radius=0.003),
            ),
            "mooring_tire",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=dark_mat,
        name="tire",
    )
    mooring_wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.112,
                0.070,
                rim=WheelRim(inner_radius=0.070, flange_height=0.006, flange_thickness=0.004),
                hub=WheelHub(radius=0.034, width=0.050, cap_style="domed"),
                face=WheelFace(dish_depth=0.004),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.012),
            ),
            "mooring_rim",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=metal_mat,
        name="rim",
    )
    mooring_wheel.visual(
        Cylinder(radius=0.020, length=0.25),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="axle_stub",
    )
    model.articulation(
        "wheel_fork_to_mooring_wheel",
        ArticulationType.CONTINUOUS,
        parent=wheel_fork,
        child=mooring_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=35.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    fork = object_model.get_part("wheel_fork")
    wheel = object_model.get_part("mooring_wheel")
    rudder = object_model.get_part("rudder")
    elevator_0 = object_model.get_part("elevator_0")
    elevator_1 = object_model.get_part("elevator_1")

    for idx in range(4):
        ctx.allow_overlap(
            hull,
            gondola,
            elem_a="cigar_envelope",
            elem_b=f"suspension_strut_{idx}",
            reason="The gondola suspension strut is intentionally seated into a hull hardpoint on the envelope.",
        )
        ctx.expect_overlap(
            hull,
            gondola,
            axes="z",
            elem_a="cigar_envelope",
            elem_b=f"suspension_strut_{idx}",
            min_overlap=0.005,
            name=f"suspension strut {idx} is inserted into the hull hardpoint",
        )

    for idx in range(2):
        engine = object_model.get_part(f"engine_pod_{idx}")
        propeller = object_model.get_part(f"propeller_{idx}")
        ctx.allow_overlap(
            engine,
            gondola,
            elem_a="side_boom",
            elem_b="equipment_cabin",
            reason="The side engine boom is bolted through a shallow socket in the gondola side wall.",
        )
        ctx.expect_overlap(
            engine,
            gondola,
            axes="y",
            elem_a="side_boom",
            elem_b="equipment_cabin",
            min_overlap=0.010,
            name=f"engine boom {idx} is seated in the cabin side socket",
        )
        ctx.allow_overlap(
            engine,
            propeller,
            elem_a="shaft_collar",
            elem_b="propeller_shaft",
            reason="The spinning propeller shaft is intentionally captured inside the engine collar bearing.",
        )
        ctx.expect_overlap(
            engine,
            propeller,
            axes="x",
            elem_a="shaft_collar",
            elem_b="propeller_shaft",
            min_overlap=0.020,
            name=f"propeller shaft {idx} remains inserted in its collar bearing",
        )

    ctx.allow_overlap(
        gondola,
        fork,
        elem_a="equipment_cabin",
        elem_b="fork_mount",
        reason="The mooring wheel fork top plate is seated into the underside of the equipment cabin.",
    )
    ctx.expect_overlap(
        gondola,
        fork,
        axes="z",
        elem_a="equipment_cabin",
        elem_b="fork_mount",
        min_overlap=0.005,
        name="wheel fork mount is seated into cabin underside",
    )
    for cheek_name in ("fork_cheek_0", "fork_cheek_1"):
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a=cheek_name,
            elem_b="axle_stub",
            reason="The mooring wheel axle stub is captured through the fork cheek bore.",
        )
        ctx.expect_overlap(
            fork,
            wheel,
            axes="y",
            elem_a=cheek_name,
            elem_b="axle_stub",
            min_overlap=0.005,
            name=f"mooring wheel axle is captured by {cheek_name}",
        )

    ctx.allow_overlap(
        hull,
        rudder,
        elem_a="rudder_hinge",
        elem_b="rudder_panel",
        reason="The rudder leading edge is captured in the vertical hinge barrel at the fin trailing edge.",
    )
    ctx.expect_overlap(
        hull,
        rudder,
        axes="x",
        elem_a="rudder_hinge",
        elem_b="rudder_panel",
        min_overlap=0.005,
        name="rudder panel is captured by its hinge barrel",
    )
    for idx, elevator in enumerate((elevator_0, elevator_1)):
        ctx.allow_overlap(
            hull,
            elevator,
            elem_a=f"elevator_hinge_{idx}",
            elem_b="elevator_panel",
            reason="The elevator leading edge is captured in the horizontal tail hinge barrel.",
        )
        ctx.expect_overlap(
            hull,
            elevator,
            axes="x",
            elem_a=f"elevator_hinge_{idx}",
            elem_b="elevator_panel",
            min_overlap=0.005,
            name=f"elevator {idx} panel is captured by its hinge barrel",
        )

    ctx.expect_origin_gap(
        hull,
        gondola,
        axis="z",
        min_gap=2.0,
        max_gap=2.5,
        name="gondola is suspended below the envelope",
    )
    ctx.expect_overlap(
        hull,
        gondola,
        axes="xy",
        min_overlap=0.70,
        name="gondola remains under the hull centerline footprint",
    )
    ctx.expect_origin_gap(
        gondola,
        wheel,
        axis="z",
        min_gap=0.45,
        max_gap=0.85,
        name="mooring wheel hangs under the gondola tail",
    )
    ctx.expect_within(
        wheel,
        fork,
        axes="y",
        margin=0.02,
        name="mooring wheel is captured between fork cheeks",
    )

    prop_joint_0 = object_model.get_articulation("engine_pod_0_to_propeller")
    prop_joint_1 = object_model.get_articulation("engine_pod_1_to_propeller")
    wheel_joint = object_model.get_articulation("wheel_fork_to_mooring_wheel")
    ctx.check(
        "propellers and mooring wheel use continuous spin joints",
        prop_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and prop_joint_1.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={prop_joint_0.articulation_type}, {prop_joint_1.articulation_type}, {wheel_joint.articulation_type}",
    )

    rudder_joint = object_model.get_articulation("hull_to_rudder")
    rest_aabb = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_joint: 0.40}):
        deflected_aabb = ctx.part_world_aabb(rudder)
    ctx.check(
        "rudder deflects sideways about a vertical trailing-edge hinge",
        rest_aabb is not None
        and deflected_aabb is not None
        and (deflected_aabb[1][1] - deflected_aabb[0][1]) > (rest_aabb[1][1] - rest_aabb[0][1]) + 0.15,
        details=f"rest={rest_aabb}, deflected={deflected_aabb}",
    )

    elevator_joint_0 = object_model.get_articulation("hull_to_elevator_0")
    elevator_joint_1 = object_model.get_articulation("hull_to_elevator_1")
    elev0_rest = ctx.part_world_aabb(elevator_0)
    elev1_rest = ctx.part_world_aabb(elevator_1)
    with ctx.pose({elevator_joint_0: 0.30, elevator_joint_1: 0.30}):
        elev0_raised = ctx.part_world_aabb(elevator_0)
        elev1_raised = ctx.part_world_aabb(elevator_1)
    ctx.check(
        "horizontal control surfaces rotate upward on horizontal hinges",
        elev0_rest is not None
        and elev1_rest is not None
        and elev0_raised is not None
        and elev1_raised is not None
        and elev0_raised[1][2] > elev0_rest[1][2] + 0.10
        and elev1_raised[1][2] > elev1_rest[1][2] + 0.10,
        details=f"rest={elev0_rest}, {elev1_rest}; raised={elev0_raised}, {elev1_raised}",
    )

    return ctx.report()


object_model = build_object_model()
