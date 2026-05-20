from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _scaled_profile(
    profile: list[tuple[float, float]], scale: float
) -> list[tuple[float, float]]:
    return [(x * scale, y * scale) for x, y in profile]


def _arm_visual(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = hypot(hypot(dx, dy), dz)
    mx = (start[0] + end[0]) * 0.5
    my = (start[1] + end[1]) * 0.5
    mz = (start[2] + end[2]) * 0.5
    yaw = atan2(dy, dx)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(mx, my, mz), rpy=(0.0, pi * 0.5, yaw)),
        material=material,
        name=name,
    )


def _propeller_blade_mesh(
    *,
    diameter: float,
    hub_radius: float,
    thickness: float,
    root_chord: float,
    tip_chord: float,
) -> ExtrudeGeometry:
    half_span = diameter * 0.5
    blade_profile = [
        (hub_radius * 0.82, -root_chord * 0.5),
        (half_span * 0.72, -tip_chord * 0.60),
        (half_span * 0.93, -tip_chord * 0.18),
        (half_span, 0.0),
        (half_span * 0.93, tip_chord * 0.18),
        (half_span * 0.72, tip_chord * 0.60),
        (hub_radius * 0.82, root_chord * 0.5),
    ]
    blade = ExtrudeGeometry.centered(blade_profile, thickness, cap=True, closed=True)
    return blade.merge(blade.clone().rotate_z(pi))


def _add_propeller_part(
    model: ArticulatedObject,
    *,
    name: str,
    diameter: float,
    material,
    nut_material,
):
    prop = model.part(name)
    hub_radius = 0.012
    hub_thickness = 0.006
    blades = _propeller_blade_mesh(
        diameter=diameter,
        hub_radius=hub_radius,
        thickness=0.0024,
        root_chord=0.020,
        tip_chord=0.012,
    )
    prop.visual(
        mesh_from_geometry(blades, f"{name}_blades"),
        material=material,
        name="blades",
    )
    prop.visual(
        Cylinder(radius=hub_radius, length=hub_thickness),
        origin=Origin(),
        material=nut_material,
        name="hub",
    )
    prop.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=nut_material,
        name="nut",
    )
    prop.inertial = Inertial.from_geometry(
        Box((diameter, 0.022, 0.010)),
        mass=0.035,
        origin=Origin(),
    )
    return prop


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tricopter_drone")

    carbon = model.material("carbon", rgba=(0.08, 0.08, 0.09, 1.0))
    carbon_tube = model.material("carbon_tube", rgba=(0.10, 0.10, 0.11, 1.0))
    pod_metal = model.material("pod_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    machined = model.material("machined", rgba=(0.72, 0.73, 0.77, 1.0))
    prop_black = model.material("prop_black", rgba=(0.11, 0.11, 0.12, 1.0))

    airframe = model.part("airframe")

    plate_profile = [
        (0.0, 0.120),
        (0.095, -0.030),
        (0.070, -0.095),
        (-0.070, -0.095),
        (-0.095, -0.030),
    ]
    lower_plate = ExtrudeGeometry.centered(plate_profile, 0.0038, cap=True, closed=True)
    upper_plate = ExtrudeGeometry.centered(
        _scaled_profile(plate_profile, 0.84),
        0.0032,
        cap=True,
        closed=True,
    )
    lower_plate.translate(0.0, 0.0, -0.012)
    upper_plate.translate(0.0, 0.0, 0.012)
    airframe.visual(
        mesh_from_geometry(lower_plate, "lower_center_plate"),
        material=carbon,
        name="lower_center_plate",
    )
    airframe.visual(
        mesh_from_geometry(upper_plate, "upper_center_plate"),
        material=carbon,
        name="upper_center_plate",
    )

    for idx, (sx, sy) in enumerate(
        [(-0.040, 0.030), (0.040, 0.030), (-0.035, -0.040), (0.035, -0.040)],
        start=1,
    ):
        airframe.visual(
            Cylinder(radius=0.0046, length=0.024),
            origin=Origin(xyz=(sx, sy, 0.0)),
            material=machined,
            name=f"standoff_{idx}",
        )

    front_left_mount = (-0.230, 0.150, 0.0)
    front_right_mount = (0.230, 0.150, 0.0)
    rear_hinge = (0.0, -0.275, 0.0)

    _arm_visual(
        airframe,
        (0.0, 0.0, 0.0),
        front_left_mount,
        0.008,
        material=carbon_tube,
        name="front_left_arm",
    )
    _arm_visual(
        airframe,
        (0.0, 0.0, 0.0),
        front_right_mount,
        0.008,
        material=carbon_tube,
        name="front_right_arm",
    )
    _arm_visual(
        airframe,
        (0.0, 0.0, 0.0),
        rear_hinge,
        0.008,
        material=carbon_tube,
        name="rear_arm",
    )

    for name, pos in (
        ("front_left_motor_mount", front_left_mount),
        ("front_right_motor_mount", front_right_mount),
    ):
        airframe.visual(
            Box((0.034, 0.024, 0.006)),
            origin=Origin(xyz=(pos[0], pos[1], 0.003)),
            material=pod_metal,
            name=name,
        )

    airframe.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(front_left_mount[0], front_left_mount[1], 0.016)),
        material=pod_metal,
        name="front_left_motor_pod",
    )
    airframe.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(front_left_mount[0], front_left_mount[1], 0.031)),
        material=machined,
        name="front_left_motor_cap",
    )
    airframe.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(front_right_mount[0], front_right_mount[1], 0.016)),
        material=pod_metal,
        name="front_right_motor_pod",
    )
    airframe.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(front_right_mount[0], front_right_mount[1], 0.031)),
        material=machined,
        name="front_right_motor_cap",
    )

    airframe.visual(
        Box((0.022, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, rear_hinge[1] + 0.004, 0.002)),
        material=pod_metal,
        name="rear_servo_block",
    )
    airframe.visual(
        Box((0.004, 0.016, 0.016)),
        origin=Origin(xyz=(-0.013, rear_hinge[1] - 0.001, 0.004)),
        material=pod_metal,
        name="rear_left_hinge_cheek",
    )
    airframe.visual(
        Box((0.004, 0.016, 0.016)),
        origin=Origin(xyz=(0.013, rear_hinge[1] - 0.001, 0.004)),
        material=pod_metal,
        name="rear_right_hinge_cheek",
    )

    airframe.inertial = Inertial.from_geometry(
        Box((0.52, 0.44, 0.06)),
        mass=0.95,
        origin=Origin(),
    )

    front_left_prop = _add_propeller_part(
        model,
        name="front_left_prop",
        diameter=0.254,
        material=prop_black,
        nut_material=machined,
    )
    front_right_prop = _add_propeller_part(
        model,
        name="front_right_prop",
        diameter=0.254,
        material=prop_black,
        nut_material=machined,
    )

    rear_pod = model.part("rear_pod")
    _arm_visual(
        rear_pod,
        (0.0, 0.0, 0.0),
        (0.0, -0.028, 0.0),
        0.011,
        material=pod_metal,
        name="hinge_collar",
    )
    rear_pod.visual(
        Box((0.018, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, -0.028, 0.014)),
        material=pod_metal,
        name="motor_stem",
    )
    rear_pod.visual(
        Box((0.004, 0.020, 0.014)),
        origin=Origin(xyz=(-0.007, -0.018, 0.007)),
        material=pod_metal,
        name="left_tilt_web",
    )
    rear_pod.visual(
        Box((0.004, 0.020, 0.014)),
        origin=Origin(xyz=(0.007, -0.018, 0.007)),
        material=pod_metal,
        name="right_tilt_web",
    )
    rear_pod.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.0, -0.028, 0.018)),
        material=pod_metal,
        name="rear_motor_pod",
    )
    rear_pod.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.0, -0.028, 0.031)),
        material=machined,
        name="rear_motor_cap",
    )
    rear_pod.inertial = Inertial.from_geometry(
        Box((0.040, 0.060, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.022, 0.018)),
    )

    rear_prop = _add_propeller_part(
        model,
        name="rear_prop",
        diameter=0.254,
        material=prop_black,
        nut_material=machined,
    )

    model.articulation(
        "front_left_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=front_left_prop,
        origin=Origin(xyz=(front_left_mount[0], front_left_mount[1], 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=160.0),
    )
    model.articulation(
        "front_right_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=front_right_prop,
        origin=Origin(xyz=(front_right_mount[0], front_right_mount[1], 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=160.0),
    )
    model.articulation(
        "rear_tilt",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=rear_pod,
        origin=Origin(xyz=rear_hinge),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=6.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "rear_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_pod,
        child=rear_prop,
        origin=Origin(xyz=(0.0, -0.028, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=160.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    front_left_prop = object_model.get_part("front_left_prop")
    front_right_prop = object_model.get_part("front_right_prop")
    rear_pod = object_model.get_part("rear_pod")
    rear_prop = object_model.get_part("rear_prop")
    front_left_prop_spin = object_model.get_articulation("front_left_prop_spin")
    front_right_prop_spin = object_model.get_articulation("front_right_prop_spin")
    rear_tilt = object_model.get_articulation("rear_tilt")
    rear_prop_spin = object_model.get_articulation("rear_prop_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "tricopter parts exist",
        all(
            part is not None
            for part in (airframe, front_left_prop, front_right_prop, rear_pod, rear_prop)
        ),
        details="Expected airframe, rear pod, and three propeller parts.",
    )
    ctx.check(
        "propeller joints spin on vertical axles",
        front_left_prop_spin.axis == (0.0, 0.0, 1.0)
        and front_right_prop_spin.axis == (0.0, 0.0, 1.0)
        and rear_prop_spin.axis == (0.0, 0.0, 1.0),
        details=(
            f"left={front_left_prop_spin.axis}, right={front_right_prop_spin.axis}, "
            f"rear={rear_prop_spin.axis}"
        ),
    )
    ctx.check(
        "rear pod tilts about rear boom axis",
        rear_tilt.axis == (0.0, 1.0, 0.0)
        and rear_tilt.motion_limits is not None
        and rear_tilt.motion_limits.lower is not None
        and rear_tilt.motion_limits.upper is not None
        and rear_tilt.motion_limits.lower < 0.0 < rear_tilt.motion_limits.upper,
        details=f"axis={rear_tilt.axis}, limits={rear_tilt.motion_limits}",
    )

    ctx.expect_contact(
        front_left_prop,
        airframe,
        elem_a="hub",
        elem_b="front_left_motor_cap",
        name="left prop hub seats on left motor cap",
    )
    ctx.expect_contact(
        front_right_prop,
        airframe,
        elem_a="hub",
        elem_b="front_right_motor_cap",
        name="right prop hub seats on right motor cap",
    )
    ctx.expect_contact(
        rear_pod,
        airframe,
        elem_a="hinge_collar",
        elem_b="rear_arm",
        name="rear tilt pod is mounted to rear arm tip",
    )
    ctx.expect_contact(
        rear_prop,
        rear_pod,
        elem_a="hub",
        elem_b="rear_motor_cap",
        name="rear prop hub seats on rear motor cap",
    )
    ctx.expect_overlap(
        front_left_prop,
        airframe,
        axes="xy",
        min_overlap=0.020,
        elem_a="hub",
        elem_b="front_left_motor_pod",
        name="left prop stays centered over left pod",
    )
    ctx.expect_overlap(
        front_right_prop,
        airframe,
        axes="xy",
        min_overlap=0.020,
        elem_a="hub",
        elem_b="front_right_motor_pod",
        name="right prop stays centered over right pod",
    )
    ctx.expect_origin_distance(
        front_left_prop,
        front_right_prop,
        axes="x",
        min_dist=0.42,
        max_dist=0.50,
        name="front motor spacing is full tricopter width",
    )
    ctx.expect_origin_gap(
        front_left_prop,
        rear_prop,
        axis="y",
        min_gap=0.40,
        max_gap=0.48,
        name="rear rotor sits well behind the front pair",
    )

    rear_rest = ctx.part_world_position(rear_prop)
    with ctx.pose({rear_tilt: 0.45}):
        ctx.expect_contact(
            rear_prop,
            rear_pod,
            elem_a="hub",
            elem_b="rear_motor_cap",
            name="rear prop stays mounted while yaw pod tilts",
        )
        rear_tilted = ctx.part_world_position(rear_prop)

    ctx.check(
        "rear tilt moves rotor sideways for yaw vectoring",
        rear_rest is not None
        and rear_tilted is not None
        and abs(rear_tilted[0] - rear_rest[0]) > 0.012,
        details=f"rest={rear_rest}, tilted={rear_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
