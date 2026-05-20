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
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _prism_xz(points: list[tuple[float, float]], thickness_y: float) -> MeshGeometry:
    """Thin triangular/trapezoid panel from an XZ profile, extruded in Y."""
    geom = MeshGeometry()
    half = thickness_y / 2.0
    front = [geom.add_vertex(x, -half, z) for x, z in points]
    back = [geom.add_vertex(x, half, z) for x, z in points]
    n = len(points)
    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _prism_xy(points: list[tuple[float, float]], thickness_z: float) -> MeshGeometry:
    """Thin triangular/trapezoid panel from an XY profile, extruded in Z."""
    geom = MeshGeometry()
    half = thickness_z / 2.0
    low = [geom.add_vertex(x, y, -half) for x, y in points]
    high = [geom.add_vertex(x, y, half) for x, y in points]
    n = len(points)
    for i in range(1, n - 1):
        geom.add_face(low[0], low[i + 1], low[i])
        geom.add_face(high[0], high[i], high[i + 1])
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(low[i], low[j], high[j])
        geom.add_face(low[i], high[j], high[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_blimp")

    envelope_mat = model.material("pearl_envelope", rgba=(0.86, 0.88, 0.84, 1.0))
    trim_mat = model.material("blue_trim", rgba=(0.08, 0.18, 0.38, 1.0))
    cabin_mat = model.material("warm_gray_cabin", rgba=(0.38, 0.39, 0.38, 1.0))
    window_mat = model.material("smoked_blue_glass", rgba=(0.03, 0.12, 0.20, 1.0))
    metal_mat = model.material("satin_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    prop_mat = model.material("black_propeller", rgba=(0.02, 0.02, 0.018, 1.0))

    airframe = model.part("airframe")

    # A sixty-meter airship envelope: a smooth lathed teardrop/cigar rotated so
    # the long axis is X, with the slightly sharper tail at -X.
    hull_profile = [
        (0.00, -30.0),
        (0.90, -28.4),
        (3.20, -24.0),
        (5.25, -16.0),
        (6.20, -2.0),
        (6.05, 12.0),
        (4.70, 22.0),
        (2.10, 27.5),
        (0.00, 30.0),
    ]
    envelope = LatheGeometry(hull_profile, segments=80, closed=True).rotate_y(math.pi / 2.0)
    airframe.visual(
        mesh_from_geometry(envelope, "streamlined_envelope"),
        material=envelope_mat,
        name="streamlined_envelope",
    )

    # Dark longitudinal trim band to make the blimp's orientation read cleanly.
    airframe.visual(
        Box((42.0, 0.045, 0.25)),
        origin=Origin(xyz=(-2.0, -6.08, -0.25)),
        material=trim_mat,
        name="side_trim_0",
    )
    airframe.visual(
        Box((42.0, 0.045, 0.25)),
        origin=Origin(xyz=(-2.0, 6.08, -0.25)),
        material=trim_mat,
        name="side_trim_1",
    )

    # Long under-slung passenger cabin/gondola, rounded at nose and tail.
    cabin_shape = CapsuleGeometry(radius=1.28, length=15.5, radial_segments=36, height_segments=10)
    cabin_shape.rotate_y(math.pi / 2.0).scale(1.0, 1.0, 0.78)
    airframe.visual(
        mesh_from_geometry(cabin_shape, "passenger_cabin_shell"),
        origin=Origin(xyz=(1.5, 0.0, -6.45)),
        material=cabin_mat,
        name="passenger_cabin_shell",
    )
    # Fared pylons from the envelope belly into the gondola roof.
    for x in (-5.2, 1.5, 8.2):
        airframe.visual(
            Box((0.85, 2.1, 1.2)),
            origin=Origin(xyz=(x, 0.0, -5.65)),
            material=cabin_mat,
            name=f"cabin_pylon_{x:g}",
        )

    # Rows of large passenger windows on both sides of the cabin.
    for side, y in enumerate((-1.31, 1.31)):
        for i, x in enumerate((-4.8, -2.9, -1.0, 0.9, 2.8, 4.7, 6.6)):
            airframe.visual(
                Box((1.05, 0.12, 0.44)),
                origin=Origin(xyz=(x, -1.23 if y < 0.0 else 1.23, -6.25)),
                material=window_mat,
                name=f"window_{side}_{i}",
            )
    airframe.visual(
        Box((0.08, 1.25, 0.48)),
        origin=Origin(xyz=(9.55, 0.0, -6.25)),
        material=window_mat,
        name="front_windscreen",
    )

    # Side engine pods near the gondola with broad pylons tied into the cabin.
    pod_y = 7.7
    pod_z = -4.40
    pod_x = 1.2
    for side, y_sign in enumerate((-1.0, 1.0)):
        y = y_sign * pod_y
        pylon_center_y = y_sign * 4.05
        airframe.visual(
            Box((1.15, 6.05, 0.70)),
            origin=Origin(xyz=(pod_x, pylon_center_y, -5.15)),
            material=metal_mat,
            name=f"pod_pylon_{side}",
        )
        airframe.visual(
            Cylinder(radius=0.78, length=3.05),
            origin=Origin(xyz=(pod_x, y, pod_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"engine_pod_{side}",
        )
        airframe.visual(
            Cylinder(radius=0.16, length=0.28),
            origin=Origin(xyz=(2.865, y, pod_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"prop_shaft_{side}",
        )

    # Cruciform fixed stabilizers. The movable control surfaces attach at the
    # aft trailing hinge lines modeled as separate articulated parts below.
    top_fin = _prism_xz([(-20.2, 3.6), (-26.1, 2.15), (-26.1, 8.05), (-22.0, 10.2)], 0.32)
    bottom_fin = _prism_xz([(-20.2, -3.6), (-26.1, -2.15), (-26.6, -7.2), (-22.3, -8.8)], 0.32)
    airframe.visual(mesh_from_geometry(top_fin, "fixed_top_fin"), material=envelope_mat, name="fixed_top_fin")
    airframe.visual(mesh_from_geometry(bottom_fin, "fixed_bottom_fin"), material=envelope_mat, name="fixed_bottom_fin")
    for side, y_sign in enumerate((-1.0, 1.0)):
        fin = _prism_xy(
            [(-20.1, y_sign * 3.2), (-26.1, y_sign * 3.5), (-26.1, y_sign * 9.25), (-22.2, y_sign * 10.7)],
            0.30,
        )
        airframe.visual(mesh_from_geometry(fin, f"fixed_horizontal_fin_{side}"), material=envelope_mat, name=f"fixed_horizontal_fin_{side}")

    # Spinning propellers. The fan helper spins about local Z; rotate the visual
    # so local Z is the blimp's longitudinal shaft axis X.
    prop_mesh = FanRotorGeometry(
        outer_radius=1.35,
        hub_radius=0.28,
        blade_count=5,
        thickness=0.24,
        blade_pitch_deg=32.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
        hub=FanRotorHub(style="spinner", bore_diameter=0.09),
    )
    for side, y_sign in enumerate((-1.0, 1.0)):
        propeller = model.part(f"propeller_{side}")
        propeller.visual(
            Cylinder(radius=0.25, length=0.34),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="hub_socket",
        )
        propeller.visual(
            mesh_from_geometry(prop_mesh, f"propeller_rotor_{side}"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=prop_mat,
            name="rotor",
        )
        model.articulation(
            f"propeller_{side}_spin",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=propeller,
            origin=Origin(xyz=(3.10, y_sign * pod_y, pod_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=260.0, velocity=80.0),
        )

    # Rudder: a vertical control surface on the trailing edge of the top tail.
    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_geometry(_prism_xz([(0.0, -2.45), (-3.25, -2.10), (-3.25, 2.05), (0.0, 2.45)], 0.26), "rudder_panel"),
        material=envelope_mat,
        name="rudder_panel",
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-26.1, 0.0, 5.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.2, lower=-0.45, upper=0.45),
    )

    # Port/starboard elevator panels on the horizontal stabilizer trailing edges.
    for side, y_sign in enumerate((-1.0, 1.0)):
        elevator = model.part(f"elevator_{side}")
        elevator.visual(
            mesh_from_geometry(_prism_xy([(0.0, -2.65), (-3.20, -2.32), (-3.20, 2.32), (0.0, 2.65)], 0.24), f"elevator_panel_{side}"),
            material=envelope_mat,
            name="elevator_panel",
        )
        model.articulation(
            f"elevator_{side}_hinge",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=elevator,
            origin=Origin(xyz=(-26.1, y_sign * 6.35, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=700.0, velocity=1.2, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    prop_joints = [object_model.get_articulation("propeller_0_spin"), object_model.get_articulation("propeller_1_spin")]
    for side in (0, 1):
        ctx.allow_overlap(
            "airframe",
            f"propeller_{side}",
            elem_a=f"prop_shaft_{side}",
            elem_b="hub_socket",
            reason="The visible propeller hub socket is intentionally captured on the short metal shaft.",
        )
        ctx.expect_gap(
            f"propeller_{side}",
            "airframe",
            axis="x",
            positive_elem="hub_socket",
            negative_elem=f"prop_shaft_{side}",
            max_gap=0.004,
            max_penetration=0.090,
            name=f"propeller_{side} hub seated on shaft",
        )
        ctx.expect_overlap(
            f"propeller_{side}",
            "airframe",
            axes="yz",
            elem_a="hub_socket",
            elem_b=f"prop_shaft_{side}",
            min_overlap=0.20,
            name=f"propeller_{side} hub centered on shaft",
        )

    ctx.check(
        "propellers use continuous spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in prop_joints),
        details=[j.articulation_type for j in prop_joints],
    )
    for joint in prop_joints:
        ctx.check(
            f"{joint.name} shaft axis is longitudinal",
            tuple(round(v, 3) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    for name in ("rudder_hinge", "elevator_0_hinge", "elevator_1_hinge"):
        joint = object_model.get_articulation(name)
        limits = joint.motion_limits
        ctx.check(
            f"{name} has realistic bidirectional limits",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and limits.upper <= 0.50,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    elevator = object_model.get_part("elevator_0")
    elevator_hinge = object_model.get_articulation("elevator_0_hinge")
    rest_aabb = ctx.part_world_aabb(elevator)
    with ctx.pose({elevator_hinge: 0.35}):
        deflected_aabb = ctx.part_world_aabb(elevator)
    ctx.check(
        "elevator trailing edge deflects upward",
        rest_aabb is not None
        and deflected_aabb is not None
        and deflected_aabb[1][2] > rest_aabb[1][2] + 0.75,
        details=f"rest={rest_aabb}, deflected={deflected_aabb}",
    )

    rudder = object_model.get_part("rudder")
    rudder_hinge = object_model.get_articulation("rudder_hinge")
    rest_pos = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_hinge: 0.45}):
        deflected_pos = ctx.part_world_aabb(rudder)
    ctx.check(
        "rudder yaws away from the centerline",
        rest_pos is not None
        and deflected_pos is not None
        and abs(deflected_pos[0][1] - rest_pos[0][1]) > 0.7,
        details=f"rest={rest_pos}, deflected={deflected_pos}",
    )

    return ctx.report()


object_model = build_object_model()
