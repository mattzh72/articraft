from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _octagon_profile(radius: float) -> list[tuple[float, float]]:
    """Flat-sided octagon profile in the XY plane."""
    return [
        (radius * math.cos(math.pi / 8.0 + i * math.pi / 4.0), radius * math.sin(math.pi / 8.0 + i * math.pi / 4.0))
        for i in range(8)
    ]


def _radial_origin(distance: float, z: float, angle: float, *, pitch: float = 0.0) -> Origin:
    return Origin(
        xyz=(distance * math.cos(angle), distance * math.sin(angle), z),
        rpy=(0.0, pitch, angle),
    )


def _local_to_world(angle: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c, z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="octorotor_heavy_lift_drone")

    matte_carbon = model.material("matte_carbon", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_plate = model.material("dark_anodized_plate", rgba=(0.10, 0.115, 0.12, 1.0))
    motor_black = model.material("motor_black", rgba=(0.015, 0.015, 0.017, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.55, 0.57, 0.58, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    battery_blue = model.material("battery_blue", rgba=(0.05, 0.16, 0.32, 1.0))
    marker_red = model.material("front_red_marker", rgba=(0.75, 0.04, 0.025, 1.0))

    center_plate = model.part("center_plate")

    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(_octagon_profile(0.38), 0.055, cap=True, center=True),
        "octagonal_center_plate",
    )
    center_plate.visual(
        plate_mesh,
        origin=Origin(),
        material=dark_plate,
        name="octagonal_plate",
    )

    # Raised battery and avionics details make the broad center plate read as a
    # heavy-lift airframe instead of a bare abstract hub.
    center_plate.visual(
        Box((0.42, 0.22, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=battery_blue,
        name="battery_pack",
    )
    center_plate.visual(
        Sphere(0.047),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=dark_plate,
        name="gps_dome",
    )
    center_plate.visual(
        Box((0.15, 0.055, 0.010)),
        origin=Origin(xyz=(0.18, 0.0, 0.034)),
        material=marker_red,
        name="front_marker",
    )

    motor_radius = 1.16
    arm_length = 1.04
    arm_center = 0.66
    propeller_height = 0.180
    propeller_radius = 0.335

    for i in range(8):
        angle = i * math.tau / 8.0
        # Round carbon booms pass through the octagonal center plate and into the motor clamps.
        center_plate.visual(
            Cylinder(radius=0.035, length=arm_length),
            origin=_radial_origin(arm_center, 0.0, angle, pitch=math.pi / 2.0),
            material=matte_carbon,
            name=f"arm_{i}",
        )
        center_plate.visual(
            Box((0.18, 0.080, 0.065)),
            origin=_radial_origin(motor_radius - 0.060, 0.020, angle),
            material=dark_plate,
            name=f"motor_clamp_{i}",
        )
        center_plate.visual(
            Cylinder(radius=0.105, length=0.110),
            origin=Origin(
                xyz=(motor_radius * math.cos(angle), motor_radius * math.sin(angle), 0.070),
            ),
            material=motor_black,
            name=f"motor_pod_{i}",
        )
        center_plate.visual(
            Cylinder(radius=0.065, length=0.018),
            origin=Origin(
                xyz=(motor_radius * math.cos(angle), motor_radius * math.sin(angle), 0.134),
            ),
            material=brushed_metal,
            name=f"motor_cap_{i}",
        )
        center_plate.visual(
            Cylinder(radius=0.012, length=0.070),
            origin=Origin(
                xyz=(motor_radius * math.cos(angle), motor_radius * math.sin(angle), 0.166),
            ),
            material=brushed_metal,
            name=f"propeller_shaft_{i}",
        )

    # One shared, mesh-backed two-blade rotor profile is reused by all eight
    # spinning propeller parts.  It has a broad scimitar planform and a through
    # bore so the stationary motor shaft can visibly pass through the hub.
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            propeller_radius,
            0.052,
            2,
            thickness=0.022,
            blade_pitch_deg=18.0,
            blade_sweep_deg=28.0,
            blade_root_chord=0.090,
            blade_tip_chord=0.055,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=9.0, camber=0.10),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.012, rear_collar_radius=0.040, bore_diameter=0.042),
        ),
        "broad_carbon_propeller",
    )
    thrust_bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.031, tube=0.006, radial_segments=16, tubular_segments=32),
        "propeller_thrust_bearing",
    )

    for i in range(8):
        angle = i * math.tau / 8.0
        propeller = model.part(f"propeller_{i}")
        propeller.visual(
            rotor_mesh,
            origin=Origin(rpy=(0.0, 0.0, i * math.pi / 8.0)),
            material=matte_carbon,
            name="rotor",
        )
        propeller.visual(
            thrust_bearing_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.031)),
            material=brushed_metal,
            name="thrust_bearing",
        )
        for post_index, post_angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            propeller.visual(
                Cylinder(radius=0.006, length=0.033),
                origin=Origin(
                    xyz=(
                        0.036 * math.cos(post_angle),
                        0.036 * math.sin(post_angle),
                        -0.0205,
                    )
                ),
                material=brushed_metal,
                name=f"drive_post_{post_index}",
            )
        model.articulation(
            f"propeller_axle_{i}",
            ArticulationType.CONTINUOUS,
            parent=center_plate,
            child=propeller,
            origin=Origin(
                xyz=(motor_radius * math.cos(angle), motor_radius * math.sin(angle), propeller_height),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=120.0),
        )

    # Four diagonal landing legs hinge from clevis brackets under the center
    # plate.  In the default pose they are deployed downward and outward.
    leg_angles = [math.pi / 4.0 + i * math.pi / 2.0 for i in range(4)]
    hinge_radius = 0.235
    hinge_z = -0.080
    leg_length = 0.465
    leg_pitch = 0.880

    for i, angle in enumerate(leg_angles):
        hx = hinge_radius * math.cos(angle)
        hy = hinge_radius * math.sin(angle)

        center_plate.visual(
            Box((0.095, 0.125, 0.026)),
            origin=Origin(xyz=(hx, hy, -0.040), rpy=(0.0, 0.0, angle)),
            material=dark_plate,
            name=f"gear_mount_{i}",
        )
        for side, side_name in ((-1.0, "neg"), (1.0, "pos")):
            ox, oy, _ = _local_to_world(angle, 0.0, side * 0.043, 0.0)
            center_plate.visual(
                Box((0.070, 0.022, 0.056)),
                origin=Origin(xyz=(hx + ox, hy + oy, hinge_z), rpy=(0.0, 0.0, angle)),
                material=brushed_metal,
                name=f"gear_lug_{side_name}_{i}",
            )

        gear_leg = model.part(f"gear_leg_{i}")
        gear_leg.visual(
            Cylinder(radius=0.018, length=0.064),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="hinge_barrel",
        )

        strut_dx = 0.5 * leg_length * math.cos(leg_pitch)
        strut_dz = -0.5 * leg_length * math.sin(leg_pitch)
        gear_leg.visual(
            Box((leg_length, 0.034, 0.034)),
            origin=Origin(xyz=(strut_dx, 0.0, strut_dz), rpy=(0.0, leg_pitch, 0.0)),
            material=matte_carbon,
            name="diagonal_strut",
        )

        foot_x = leg_length * math.cos(leg_pitch) + 0.025
        foot_z = -leg_length * math.sin(leg_pitch)
        gear_leg.visual(
            Cylinder(radius=0.018, length=0.300),
            origin=Origin(xyz=(foot_x, 0.0, foot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="foot_skid",
        )
        gear_leg.visual(
            Box((0.070, 0.055, 0.040)),
            origin=Origin(xyz=(foot_x - 0.020, 0.0, foot_z + 0.010)),
            material=rubber,
            name="foot_socket",
        )

        model.articulation(
            f"gear_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=center_plate,
            child=gear_leg,
            origin=Origin(xyz=(hx, hy, hinge_z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=0.95),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    propellers = [object_model.get_part(f"propeller_{i}") for i in range(8)]
    propeller_axles = [object_model.get_articulation(f"propeller_axle_{i}") for i in range(8)]
    gear_legs = [object_model.get_part(f"gear_leg_{i}") for i in range(4)]
    gear_hinges = [object_model.get_articulation(f"gear_hinge_{i}") for i in range(4)]

    ctx.check("eight separate spinning propellers", len(propellers) == 8)
    ctx.check("four folding landing legs", len(gear_legs) == 4)
    ctx.check(
        "propellers use continuous axles",
        all(getattr(joint, "articulation_type", None) == ArticulationType.CONTINUOUS for joint in propeller_axles),
    )
    ctx.check(
        "landing legs use revolute hinges",
        all(getattr(joint, "articulation_type", None) == ArticulationType.REVOLUTE for joint in gear_hinges),
    )

    center_plate = object_model.get_part("center_plate")
    for i, propeller in enumerate(propellers):
        ctx.expect_origin_distance(
            propeller,
            center_plate,
            axes="xy",
            min_dist=1.12,
            max_dist=1.20,
            name=f"propeller {i} sits at an arm tip",
        )
        ctx.expect_gap(
            propeller,
            center_plate,
            axis="z",
            positive_elem="rotor",
            negative_elem=f"motor_cap_{i}",
            min_gap=0.020,
            max_gap=0.070,
            name=f"propeller {i} clears its motor cap",
        )

    for i, gear_leg in enumerate(gear_legs):
        ctx.expect_origin_distance(
            gear_leg,
            center_plate,
            axes="xy",
            min_dist=0.20,
            max_dist=0.27,
            name=f"gear leg {i} hinges under the center plate",
        )
        ctx.expect_overlap(
            gear_leg,
            center_plate,
            axes="xz",
            elem_a="hinge_barrel",
            elem_b=f"gear_lug_pos_{i}",
            min_overlap=0.012,
            name=f"gear leg {i} barrel is captured by its clevis",
        )

        rest_aabb = ctx.part_element_world_aabb(gear_leg, elem="foot_skid")
        with ctx.pose({gear_hinges[i]: 0.95}):
            folded_aabb = ctx.part_element_world_aabb(gear_leg, elem="foot_skid")

        rest_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        folded_z = None if folded_aabb is None else (folded_aabb[0][2] + folded_aabb[1][2]) * 0.5
        ctx.check(
            f"gear leg {i} folds upward",
            rest_z is not None and folded_z is not None and folded_z > rest_z + 0.25,
            details=f"rest_z={rest_z}, folded_z={folded_z}",
        )

    return ctx.report()


object_model = build_object_model()
