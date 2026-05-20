from __future__ import annotations

from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


HULL_CENTER_Z = 7.0
HULL_LENGTH = 30.0
GONDOLA_ORIGIN = (0.0, 0.45, 1.15)
LEFT_POD_ORIGIN = (4.75, -0.55, 5.70)
RIGHT_POD_ORIGIN = (-4.75, -0.55, 5.70)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _streamlined_body_mesh(
    control_points: list[tuple[float, float]],
    name: str,
    *,
    segments: int = 72,
    samples_per_segment: int = 10,
):
    outer = sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=samples_per_segment,
        closed=False,
    )
    start_z = control_points[0][1]
    end_z = control_points[-1][1]
    profile = [(0.0, start_z)]
    for radius, z in outer[1:-1]:
        profile.append((max(0.0, radius), z))
    profile.append((0.0, end_z))
    geometry = LatheGeometry(profile, segments=segments).rotate_x(-pi / 2.0)
    return mesh_from_geometry(geometry, name)


def _pod_mesh(name: str):
    return _streamlined_body_mesh(
        [
            (0.0, -1.22),
            (0.10, -1.15),
            (0.24, -0.86),
            (0.35, -0.18),
            (0.38, 0.40),
            (0.29, 0.95),
            (0.10, 1.16),
            (0.0, 1.25),
        ],
        name,
        segments=48,
        samples_per_segment=8,
    )


def _add_propeller_visuals(part, *, blade_radius: float, hub_radius: float, material, hub_material) -> None:
    part.visual(
        Cylinder(radius=hub_radius, length=0.30),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(Sphere(radius=hub_radius * 1.10), material=hub_material, name="spinner")
    for blade_index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        part.visual(
            Box((blade_radius * 1.80, 0.10, 0.032)),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.18, angle, 0.0)),
            material=material,
            name=f"blade_{blade_index}",
        )


def _add_wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    hub_radius: float,
    hub_width: float,
    tire_material,
    hub_material,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(Sphere(radius=hub_radius * 0.60), material=hub_material, name="cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    envelope = model.material("envelope", rgba=(0.78, 0.81, 0.84, 1.0))
    orange = model.material("orange", rgba=(0.86, 0.38, 0.12, 1.0))
    pod_gray = model.material("pod_gray", rgba=(0.39, 0.42, 0.46, 1.0))
    structure = model.material("structure", rgba=(0.29, 0.30, 0.32, 1.0))
    glass = model.material("glass", rgba=(0.63, 0.77, 0.86, 0.40))
    tire = model.material("tire", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))

    hull = model.part("hull")
    hull.visual(
        _streamlined_body_mesh(
            [
                (0.0, -15.0),
                (0.35, -14.2),
                (1.05, -12.8),
                (2.30, -9.8),
                (3.55, -4.6),
                (4.15, 0.0),
                (4.05, 4.9),
                (3.00, 9.8),
                (1.55, 13.2),
                (0.42, 14.5),
                (0.0, 15.0),
            ],
            "hull_envelope",
        ),
        origin=Origin(xyz=(0.0, 0.0, HULL_CENTER_Z)),
        material=envelope,
        name="envelope",
    )
    hull.visual(
        Box((0.92, 3.60, 0.30)),
        origin=Origin(xyz=(0.0, 0.45, 2.66)),
        material=structure,
        name="gondola_mount",
    )
    hull.visual(
        Box((0.48, 5.20, 0.24)),
        origin=Origin(xyz=(0.0, 0.18, 2.40)),
        material=pod_gray,
        name="keel_fairing",
    )
    hull.visual(
        Box((0.36, 1.25, 0.28)),
        origin=Origin(xyz=(4.17, -0.55, 6.15)),
        material=structure,
        name="left_pod_saddle",
    )
    hull.visual(
        Box((0.36, 1.25, 0.28)),
        origin=Origin(xyz=(-4.17, -0.55, 6.15)),
        material=structure,
        name="right_pod_saddle",
    )
    hull.visual(
        Box((1.30, 2.00, 1.45)),
        origin=Origin(xyz=(0.0, -13.55, 7.0)),
        material=orange,
        name="tail_boom",
    )
    hull.visual(
        Box((0.20, 1.30, 4.70)),
        origin=Origin(xyz=(0.0, -14.55, 7.0)),
        material=orange,
        name="rear_fin",
    )
    hull.visual(
        Box((3.10, 0.95, 0.18)),
        origin=Origin(xyz=(1.55, -13.73, 7.0)),
        material=orange,
        name="left_stabilizer",
    )
    hull.visual(
        Box((3.10, 0.95, 0.18)),
        origin=Origin(xyz=(-1.55, -13.73, 7.0)),
        material=orange,
        name="right_stabilizer",
    )
    hull.visual(
        Sphere(radius=0.18),
        origin=Origin(xyz=(0.0, 15.05, HULL_CENTER_Z)),
        material=wheel_metal,
        name="nose_cap",
    )

    gondola = model.part("gondola")
    gondola.visual(
        Box((2.04, 3.20, 1.15)),
        material=pod_gray,
        name="gondola_shell",
    )
    gondola.visual(
        Box((1.76, 2.92, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=orange,
        name="roof_cap",
    )
    gondola.visual(
        Box((1.22, 1.72, 0.36)),
        origin=Origin(xyz=(0.0, -0.18, -0.62)),
        material=structure,
        name="belly_box",
    )
    gondola.visual(
        Box((1.38, 0.72, 0.52)),
        origin=Origin(xyz=(0.0, 1.67, 0.10)),
        material=pod_gray,
        name="nose_cabin",
    )
    gondola.visual(
        Box((1.20, 0.06, 0.38)),
        origin=Origin(xyz=(0.0, 1.62, 0.12)),
        material=glass,
        name="front_window",
    )
    gondola.visual(
        Box((0.05, 1.32, 0.34)),
        origin=Origin(xyz=(1.03, 0.55, 0.12)),
        material=glass,
        name="left_window",
    )
    gondola.visual(
        Box((0.05, 1.32, 0.34)),
        origin=Origin(xyz=(-1.03, 0.55, 0.12)),
        material=glass,
        name="right_window",
    )
    gondola.visual(
        Box((0.72, 2.64, 0.10)),
        origin=Origin(xyz=(0.0, 0.12, 0.72)),
        material=structure,
        name="roof_saddle",
    )
    for index, (lower, upper) in enumerate(
        (
            ((0.76, 1.00, 0.60), (0.42, 1.16, 1.338)),
            ((-0.76, 1.00, 0.60), (-0.42, 1.16, 1.338)),
            ((0.74, -1.06, 0.60), (0.40, -1.12, 1.338)),
            ((-0.74, -1.06, 0.60), (-0.40, -1.12, 1.338)),
        )
    ):
        _add_member(
            gondola,
            lower,
            upper,
            radius=0.050,
            material=structure,
            name=f"support_strut_{index}",
        )
    gondola.visual(
        Cylinder(radius=0.050, length=1.68),
        origin=Origin(xyz=(0.0, -0.22, -0.87), rpy=(0.0, pi / 2.0, 0.0)),
        material=structure,
        name="main_axle",
    )
    for index, (upper, lower) in enumerate(
        (
            ((0.52, -0.08, -0.48), (0.86, -0.22, -0.87)),
            ((-0.52, -0.08, -0.48), (-0.86, -0.22, -0.87)),
            ((0.34, 0.42, -0.42), (0.86, -0.22, -0.87)),
            ((-0.34, 0.42, -0.42), (-0.86, -0.22, -0.87)),
        )
    ):
        _add_member(
            gondola,
            upper,
            lower,
            radius=0.040,
            material=structure,
            name=f"main_gear_leg_{index}",
        )
    gondola.visual(
        Cylinder(radius=0.032, length=0.08),
        origin=Origin(xyz=(0.86, -0.22, -0.87), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="left_stub_axle",
    )
    gondola.visual(
        Cylinder(radius=0.032, length=0.08),
        origin=Origin(xyz=(-0.86, -0.22, -0.87), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="right_stub_axle",
    )
    gondola.visual(
        Box((0.34, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 1.08, -0.58)),
        material=wheel_metal,
        name="nose_fork_crown",
    )
    _add_member(
        gondola,
        (0.0, 0.92, -0.28),
        (0.0, 1.08, -0.58),
        radius=0.040,
        material=structure,
        name="nose_gear_leg",
    )
    _add_member(
        gondola,
        (0.0, 0.42, -0.42),
        (0.0, 0.98, -0.60),
        radius=0.028,
        material=structure,
        name="nose_drag_brace",
    )
    _add_member(
        gondola,
        (0.152, 1.08, -0.58),
        (0.152, 1.18, -0.84),
        radius=0.022,
        material=wheel_metal,
        name="nose_fork_left",
    )
    _add_member(
        gondola,
        (-0.152, 1.08, -0.58),
        (-0.152, 1.18, -0.84),
        radius=0.022,
        material=wheel_metal,
        name="nose_fork_right",
    )

    left_pod = model.part("left_pod")
    left_pod.visual(_pod_mesh("left_pod_body"), material=pod_gray, name="pod_body")
    left_pod.visual(
        Box((0.48, 1.16, 0.16)),
        origin=Origin(xyz=(-0.10, 0.0, 0.46)),
        material=structure,
        name="pylon_fairing",
    )
    _add_member(
        left_pod,
        (-0.16, 0.28, 0.16),
        (-0.40, 0.38, 0.58),
        radius=0.045,
        material=structure,
        name="left_front_pylon",
    )
    _add_member(
        left_pod,
        (-0.16, -0.28, 0.16),
        (-0.40, -0.36, 0.58),
        radius=0.045,
        material=structure,
        name="left_rear_pylon",
    )
    left_pod.visual(
        Cylinder(radius=0.050, length=0.18),
        origin=Origin(xyz=(0.0, 1.12, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="shaft",
    )

    right_pod = model.part("right_pod")
    right_pod.visual(_pod_mesh("right_pod_body"), material=pod_gray, name="pod_body")
    right_pod.visual(
        Box((0.48, 1.16, 0.16)),
        origin=Origin(xyz=(0.10, 0.0, 0.46)),
        material=structure,
        name="pylon_fairing",
    )
    _add_member(
        right_pod,
        (0.16, 0.28, 0.16),
        (0.40, 0.38, 0.58),
        radius=0.045,
        material=structure,
        name="right_front_pylon",
    )
    _add_member(
        right_pod,
        (0.16, -0.28, 0.16),
        (0.40, -0.36, 0.58),
        radius=0.045,
        material=structure,
        name="right_rear_pylon",
    )
    right_pod.visual(
        Cylinder(radius=0.050, length=0.18),
        origin=Origin(xyz=(0.0, 1.12, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="shaft",
    )

    rudder = model.part("rudder")
    rudder.visual(
        Box((0.16, 0.96, 4.38)),
        origin=Origin(xyz=(0.0, -0.48, 0.0)),
        material=orange,
        name="rudder_surface",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Box((2.44, 0.90, 0.14)),
        origin=Origin(xyz=(0.0, -0.45, 0.0)),
        material=orange,
        name="elevator_surface",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Box((2.44, 0.90, 0.14)),
        origin=Origin(xyz=(0.0, -0.45, 0.0)),
        material=orange,
        name="elevator_surface",
    )

    left_propeller = model.part("left_propeller")
    _add_propeller_visuals(
        left_propeller,
        blade_radius=0.78,
        hub_radius=0.12,
        material=prop_black,
        hub_material=wheel_metal,
    )

    right_propeller = model.part("right_propeller")
    _add_propeller_visuals(
        right_propeller,
        blade_radius=0.78,
        hub_radius=0.12,
        material=prop_black,
        hub_material=wheel_metal,
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        tire_radius=0.28,
        tire_width=0.16,
        hub_radius=0.11,
        hub_width=0.20,
        tire_material=tire,
        hub_material=wheel_metal,
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        tire_radius=0.28,
        tire_width=0.16,
        hub_radius=0.11,
        hub_width=0.20,
        tire_material=tire,
        hub_material=wheel_metal,
    )

    nose_wheel = model.part("nose_wheel")
    _add_wheel_visuals(
        nose_wheel,
        tire_radius=0.24,
        tire_width=0.12,
        hub_radius=0.09,
        hub_width=0.26,
        tire_material=tire,
        hub_material=wheel_metal,
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=GONDOLA_ORIGIN),
    )
    model.articulation(
        "hull_to_left_pod",
        ArticulationType.FIXED,
        parent=hull,
        child=left_pod,
        origin=Origin(xyz=LEFT_POD_ORIGIN),
    )
    model.articulation(
        "hull_to_right_pod",
        ArticulationType.FIXED,
        parent=hull,
        child=right_pod,
        origin=Origin(xyz=RIGHT_POD_ORIGIN),
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(0.0, -15.20, 7.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.48, upper=0.48),
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(2.15, -14.205, 7.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-2.15, -14.205, 7.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "left_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=left_pod,
        child=left_propeller,
        origin=Origin(xyz=(0.0, 1.40, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    model.articulation(
        "right_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=right_pod,
        child=right_propeller,
        origin=Origin(xyz=(0.0, 1.40, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=left_wheel,
        origin=Origin(xyz=(1.00, -0.22, -0.87)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=16.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=right_wheel,
        origin=Origin(xyz=(-1.00, -0.22, -0.87)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=16.0),
    )
    model.articulation(
        "nose_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=gondola,
        child=nose_wheel,
        origin=Origin(xyz=(0.0, 1.18, -0.88)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "hull",
        "left_pod",
        reason="The left engine pod is intentionally blended into the hull through faired saddle hardpoints and support pylons.",
    )
    ctx.allow_overlap(
        "hull",
        "right_pod",
        reason="The right engine pod is intentionally blended into the hull through faired saddle hardpoints and support pylons.",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    left_prop_spin = object_model.get_articulation("left_prop_spin")
    right_prop_spin = object_model.get_articulation("right_prop_spin")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    nose_wheel_spin = object_model.get_articulation("nose_wheel_spin")
    rudder_hinge = object_model.get_articulation("rudder_hinge")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")
    right_elevator_hinge = object_model.get_articulation("right_elevator_hinge")

    rotary_ok = all(
        joint.articulation_type == ArticulationType.CONTINUOUS
        for joint in (
            left_prop_spin,
            right_prop_spin,
            left_wheel_spin,
            right_wheel_spin,
            nose_wheel_spin,
        )
    )
    ctx.check(
        "continuous rotary joints are preserved",
        rotary_ok,
        details=(
            f"left_prop={left_prop_spin.articulation_type}, right_prop={right_prop_spin.articulation_type}, "
            f"left_wheel={left_wheel_spin.articulation_type}, right_wheel={right_wheel_spin.articulation_type}, "
            f"nose_wheel={nose_wheel_spin.articulation_type}"
        ),
    )

    rudder_limits = rudder_hinge.motion_limits
    left_elevator_limits = left_elevator_hinge.motion_limits
    right_elevator_limits = right_elevator_hinge.motion_limits
    ctx.check(
        "tail control limits are workmanlike",
        rudder_hinge.articulation_type == ArticulationType.REVOLUTE
        and left_elevator_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_elevator_hinge.articulation_type == ArticulationType.REVOLUTE
        and rudder_limits is not None
        and rudder_limits.lower is not None
        and rudder_limits.upper is not None
        and rudder_limits.lower <= -0.40
        and rudder_limits.upper >= 0.40
        and left_elevator_limits is not None
        and left_elevator_limits.lower is not None
        and left_elevator_limits.upper is not None
        and left_elevator_limits.lower <= -0.35
        and left_elevator_limits.upper >= 0.35
        and right_elevator_limits is not None
        and right_elevator_limits.lower is not None
        and right_elevator_limits.upper is not None
        and right_elevator_limits.lower <= -0.35
        and right_elevator_limits.upper >= 0.35,
        details=(
            f"rudder={rudder_limits}, left={left_elevator_limits}, right={right_elevator_limits}"
        ),
    )

    ctx.expect_gap(
        "hull",
        "gondola",
        axis="z",
        positive_elem="gondola_mount",
        negative_elem="gondola_shell",
        min_gap=0.55,
        max_gap=1.20,
        name="gondola hangs below the hull on visible struts",
    )
    ctx.expect_origin_gap(
        "left_wheel",
        "right_wheel",
        axis="x",
        min_gap=1.80,
        max_gap=2.20,
        name="main wheels keep a stable track width",
    )
    ctx.expect_origin_gap(
        "nose_wheel",
        "left_wheel",
        axis="y",
        min_gap=1.10,
        max_gap=1.70,
        name="nose wheel sits forward of the main gear",
    )
    ctx.expect_origin_gap(
        "gondola",
        "left_wheel",
        axis="z",
        min_gap=0.70,
        max_gap=1.10,
        name="main wheels remain below the gondola body",
    )
    ctx.expect_origin_gap(
        "left_propeller",
        "left_pod",
        axis="y",
        min_gap=1.20,
        max_gap=1.60,
        name="left propeller stays out on the pod nose shaft",
    )

    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")

    rudder_rest = _aabb_center(ctx.part_world_aabb(rudder))
    left_elevator_rest = _aabb_center(ctx.part_world_aabb(left_elevator))

    rudder_posed = None
    if rudder_limits is not None and rudder_limits.upper is not None:
        with ctx.pose({rudder_hinge: rudder_limits.upper}):
            rudder_posed = _aabb_center(ctx.part_world_aabb(rudder))

    elevator_posed = None
    if left_elevator_limits is not None and left_elevator_limits.upper is not None:
        with ctx.pose({left_elevator_hinge: left_elevator_limits.upper}):
            elevator_posed = _aabb_center(ctx.part_world_aabb(left_elevator))

    ctx.check(
        "rudder swings laterally at full deflection",
        rudder_rest is not None
        and rudder_posed is not None
        and abs(rudder_posed[0] - rudder_rest[0]) > 0.15,
        details=f"rest={rudder_rest}, posed={rudder_posed}",
    )
    ctx.check(
        "elevator changes pitch at full deflection",
        left_elevator_rest is not None
        and elevator_posed is not None
        and abs(elevator_posed[2] - left_elevator_rest[2]) > 0.10,
        details=f"rest={left_elevator_rest}, posed={elevator_posed}",
    )

    return ctx.report()


object_model = build_object_model()
