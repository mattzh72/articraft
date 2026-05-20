from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveillance_blimp")

    fabric = model.material("warm_silver_fabric", rgba=(0.78, 0.80, 0.78, 1.0))
    seam = model.material("soft_panel_seams", rgba=(0.55, 0.58, 0.58, 1.0))
    cabin_mat = model.material("white_composite_gondola", rgba=(0.88, 0.91, 0.88, 1.0))
    glass = model.material("smoked_sensor_glass", rgba=(0.04, 0.08, 0.10, 0.85))
    metal = model.material("satin_dark_metal", rgba=(0.18, 0.20, 0.21, 1.0))
    control = model.material("light_tail_controls", rgba=(0.70, 0.73, 0.72, 1.0))
    prop_mat = model.material("black_composite_propeller", rgba=(0.03, 0.03, 0.035, 1.0))

    airframe = model.part("airframe")

    # Long observation-airship envelope, about 18 m long and 4.3 m in diameter.
    envelope = SphereGeometry(1.0, width_segments=72, height_segments=36).scale(9.0, 2.15, 2.15)
    airframe.visual(mesh_from_geometry(envelope, "long_envelope"), material=fabric, name="long_envelope")

    for i, x in enumerate((-6.0, -3.0, 0.0, 3.0, 6.0)):
        ring_radius = 2.15 * math.sqrt(max(0.0, 1.0 - (x / 9.0) ** 2)) + 0.018
        ring = TorusGeometry(ring_radius, 0.018, radial_segments=18, tubular_segments=80)
        ring.rotate_y(math.pi / 2.0).translate(x, 0.0, 0.0)
        airframe.visual(mesh_from_geometry(ring, f"envelope_seam_{i}"), material=seam, name=f"envelope_seam_{i}")

    # Small sensor gondola carried below the envelope by visible suspension struts.
    gondola = CapsuleGeometry(0.42, 1.70, radial_segments=32, height_segments=10)
    gondola.rotate_y(math.pi / 2.0).translate(-0.45, 0.0, -2.80)
    airframe.visual(mesh_from_geometry(gondola, "sensor_gondola"), material=cabin_mat, name="sensor_gondola")
    airframe.visual(Box((0.70, 0.025, 0.18)), origin=Origin(xyz=(-0.25, 0.425, -2.70)), material=glass, name="side_window_0")
    airframe.visual(Box((0.70, 0.025, 0.18)), origin=Origin(xyz=(-0.25, -0.425, -2.70)), material=glass, name="side_window_1")
    airframe.visual(Sphere(0.22), origin=Origin(xyz=(0.60, 0.0, -3.08)), material=glass, name="sensor_turret")

    for x in (-1.00, 0.20):
        for y in (-0.28, 0.28):
            airframe.visual(
                Cylinder(radius=0.035, length=0.42),
                origin=Origin(xyz=(x, y, -2.27)),
                material=metal,
                name=f"suspension_strut_{x}_{y}",
            )

    # Tail assembly: fixed fin surfaces are part of the airframe, while the
    # trailing rudder and elevators are separate hinged links.
    vertical_fin_profile = [
        (-0.55, 0.00),
        (0.65, 0.00),
        (0.42, 1.25),
        (-0.35, 1.45),
    ]
    vertical_fin = ExtrudeGeometry(vertical_fin_profile, 0.10, center=True).rotate_x(math.pi / 2.0).translate(-8.10, 0.0, 0.82)
    airframe.visual(mesh_from_geometry(vertical_fin, "vertical_stabilizer"), material=control, name="vertical_stabilizer")
    airframe.visual(
        Cylinder(radius=0.035, length=1.08),
        origin=Origin(xyz=(-8.615, 0.0, 1.60)),
        material=metal,
        name="rudder_hinge_post",
    )

    for side, name in ((1.0, "horizontal_stabilizer_0"), (-1.0, "horizontal_stabilizer_1")):
        horizontal_profile = [
            (-7.45, side * 0.68),
            (-8.65, side * 0.82),
            (-8.72, side * 2.08),
            (-7.75, side * 1.72),
        ]
        tailplane = ExtrudeGeometry(horizontal_profile, 0.08, center=True).translate(0.0, 0.0, 0.05)
        airframe.visual(mesh_from_geometry(tailplane, name), material=control, name=name)

    # Side-mounted vectoring engine pylons aft of the cabin.
    def add_pylon_strut(side: float, z: float, suffix: str) -> None:
        start = (-1.25, side * 0.34)
        end = (-2.35, side * 1.10)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        airframe.visual(
            Box((length, 0.075, 0.075)),
            origin=Origin(xyz=((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, z), rpy=(0.0, 0.0, yaw)),
            material=metal,
            name=f"pylon_strut_{suffix}",
        )

    for side, idx in ((1.0, 0), (-1.0, 1)):
        add_pylon_strut(side, -2.54, f"{idx}_upper")
        add_pylon_strut(side, -2.76, f"{idx}_lower")
        airframe.visual(
            Box((0.13, 0.09, 0.42)),
            origin=Origin(xyz=(-2.35, side * 1.10, -2.65)),
            material=metal,
            name=f"pylon_end_plate_{idx}",
        )
        airframe.visual(
            Cylinder(radius=0.12, length=0.18),
            origin=Origin(xyz=(-2.35, side * 1.11, -2.65), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_socket_0" if idx == 0 else "pivot_socket_1",
        )

    for side, idx in ((1.0, 0), (-1.0, 1)):
        pod = model.part(f"pod_{idx}")
        pod_body = CapsuleGeometry(0.28, 0.74, radial_segments=32, height_segments=8)
        pod_body.rotate_y(math.pi / 2.0).translate(0.02, side * 0.34, 0.0)
        pod.visual(mesh_from_geometry(pod_body, f"engine_nacelle_{idx}"), material=metal, name="engine_nacelle")
        pod.visual(
            Cylinder(radius=0.070, length=0.30),
            origin=Origin(xyz=(0.0, side * 0.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="pivot_pin",
        )

        model.articulation(
            f"pylon_to_pod_{idx}",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=pod,
            origin=Origin(xyz=(-2.35, side * 1.20, -2.65)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=150.0, velocity=0.8, lower=-0.55, upper=0.55),
        )

        propeller = model.part(f"propeller_{idx}")
        rotor = FanRotorGeometry(
            0.36,
            0.085,
            5,
            thickness=0.045,
            blade_pitch_deg=30.0,
            blade_sweep_deg=24.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.14),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.022, rear_collar_radius=0.070),
        )
        propeller.visual(mesh_from_geometry(rotor, f"propeller_rotor_{idx}"), material=prop_mat, name="rotor")
        propeller.visual(
            Cylinder(radius=0.035, length=0.11),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=metal,
            name="propeller_shaft",
        )
        model.articulation(
            f"pod_to_propeller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=pod,
            child=propeller,
            origin=Origin(xyz=(0.70, side * 0.34, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=25.0, velocity=80.0),
        )

    rudder = model.part("rudder")
    rudder.visual(Box((0.56, 0.070, 1.00)), origin=Origin(xyz=(-0.28, 0.0, 0.05)), material=control, name="rudder_panel")
    rudder.visual(Cylinder(radius=0.030, length=1.02), origin=Origin(xyz=(-0.035, 0.0, 0.05)), material=metal, name="rudder_hinge")
    model.articulation(
        "airframe_to_rudder",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rudder,
        origin=Origin(xyz=(-8.65, 0.0, 1.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.45, upper=0.45),
    )

    for side, idx in ((1.0, 0), (-1.0, 1)):
        elevator = model.part(f"elevator_{idx}")
        elevator.visual(Box((0.50, 1.08, 0.060)), origin=Origin(xyz=(-0.25, 0.0, 0.0)), material=control, name="elevator_panel")
        elevator.visual(
            Cylinder(radius=0.028, length=1.06),
            origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="elevator_hinge",
        )
        model.articulation(
            f"airframe_to_elevator_{idx}",
            ArticulationType.REVOLUTE,
            parent=airframe,
            child=elevator,
            origin=Origin(xyz=(-8.715, side * 1.45, 0.06)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-0.40, upper=0.40),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    airframe = object_model.get_part("airframe")

    for idx in (0, 1):
        pod = object_model.get_part(f"pod_{idx}")
        propeller = object_model.get_part(f"propeller_{idx}")
        prop_joint = object_model.get_articulation(f"pod_to_propeller_{idx}")
        pylon_joint = object_model.get_articulation(f"pylon_to_pod_{idx}")

        ctx.allow_overlap(
            propeller,
            pod,
            elem_a="propeller_shaft",
            elem_b="engine_nacelle",
            reason="The spinning propeller shaft is intentionally captured inside the front nacelle bearing.",
        )
        ctx.expect_overlap(
            propeller,
            pod,
            axes="x",
            elem_a="propeller_shaft",
            elem_b="engine_nacelle",
            min_overlap=0.030,
            name=f"propeller_{idx} shaft remains inserted in nacelle",
        )
        ctx.expect_within(
            propeller,
            pod,
            axes="yz",
            inner_elem="propeller_shaft",
            outer_elem="engine_nacelle",
            margin=0.010,
            name=f"propeller_{idx} shaft is centered in nacelle bearing",
        )
        ctx.check(
            f"propeller_{idx} spins continuously",
            prop_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={prop_joint.articulation_type}",
        )
        ctx.check(
            f"pod_{idx} has a bounded vectoring pivot",
            pylon_joint.motion_limits is not None
            and pylon_joint.motion_limits.lower is not None
            and pylon_joint.motion_limits.upper is not None
            and pylon_joint.motion_limits.lower < -0.4
            and pylon_joint.motion_limits.upper > 0.4,
            details=f"limits={pylon_joint.motion_limits}",
        )

    ctx.expect_gap(
        object_model.get_part("pod_0"),
        airframe,
        axis="y",
        positive_elem="pivot_pin",
        negative_elem="pivot_socket_0",
        max_gap=0.005,
        max_penetration=0.002,
        name="pod_0 pivot pin seats in pylon socket",
    )
    ctx.expect_gap(
        airframe,
        object_model.get_part("pod_1"),
        axis="y",
        positive_elem="pivot_socket_1",
        negative_elem="pivot_pin",
        max_gap=0.005,
        max_penetration=0.002,
        name="pod_1 pivot pin seats in pylon socket",
    )

    for elevator_name, stabilizer_elem in (
        ("elevator_0", "horizontal_stabilizer_0"),
        ("elevator_1", "horizontal_stabilizer_1"),
    ):
        elevator = object_model.get_part(elevator_name)
        ctx.allow_overlap(
            airframe,
            elevator,
            elem_a=stabilizer_elem,
            elem_b="elevator_panel",
            reason="The elevator leading edge is intentionally seated into a narrow tailplane hinge gap.",
        )
        ctx.expect_gap(
            airframe,
            elevator,
            axis="x",
            positive_elem=stabilizer_elem,
            negative_elem="elevator_panel",
            max_gap=0.002,
            max_penetration=0.006,
            name=f"{elevator_name} has only a narrow hinge-line inset",
        )

    rudder = object_model.get_part("rudder")
    rudder_joint = object_model.get_articulation("airframe_to_rudder")
    elevator_0 = object_model.get_part("elevator_0")
    elevator_joint = object_model.get_articulation("airframe_to_elevator_0")
    pod_0_joint = object_model.get_articulation("pylon_to_pod_0")
    propeller_0 = object_model.get_part("propeller_0")

    def coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    def element_center_axis(part, elem: str, index: int) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        return 0.5 * (coord(box[0], index) + coord(box[1], index))

    rudder_rest_y = element_center_axis(rudder, "rudder_panel", 1)
    with ctx.pose({rudder_joint: 0.35}):
        rudder_deflected_y = element_center_axis(rudder, "rudder_panel", 1)
    ctx.check(
        "rudder rotates about vertical hinge",
        rudder_rest_y is not None and rudder_deflected_y is not None and abs(rudder_deflected_y - rudder_rest_y) > 0.07,
        details=f"rest_y={rudder_rest_y}, deflected_y={rudder_deflected_y}",
    )

    elevator_rest_z = element_center_axis(elevator_0, "elevator_panel", 2)
    with ctx.pose({elevator_joint: 0.35}):
        elevator_deflected_z = element_center_axis(elevator_0, "elevator_panel", 2)
    ctx.check(
        "elevator rotates about horizontal tail hinge",
        elevator_rest_z is not None and elevator_deflected_z is not None and abs(elevator_deflected_z - elevator_rest_z) > 0.05,
        details=f"rest_z={elevator_rest_z}, deflected_z={elevator_deflected_z}",
    )

    propeller_rest_pos = ctx.part_world_position(propeller_0)
    with ctx.pose({pod_0_joint: 0.45}):
        propeller_vectored_pos = ctx.part_world_position(propeller_0)
    ctx.check(
        "engine pod vectors propeller axis",
        propeller_rest_pos is not None
        and propeller_vectored_pos is not None
        and abs(propeller_vectored_pos[2] - propeller_rest_pos[2]) > 0.20,
        details=f"rest={propeller_rest_pos}, vectored={propeller_vectored_pos}",
    )

    return ctx.report()


object_model = build_object_model()
