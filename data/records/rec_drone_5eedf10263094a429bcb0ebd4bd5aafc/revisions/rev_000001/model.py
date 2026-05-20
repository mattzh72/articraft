from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hub_shell_geometry():
    outer_profile = [
        (0.044, 0.072),
        (0.078, 0.078),
        (0.104, 0.094),
        (0.111, 0.128),
        (0.104, 0.161),
        (0.078, 0.176),
        (0.040, 0.184),
    ]
    inner_profile = [
        (0.031, 0.078),
        (0.071, 0.083),
        (0.097, 0.096),
        (0.103, 0.128),
        (0.097, 0.157),
        (0.071, 0.170),
        (0.031, 0.176),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _rotor_ring_geometry(inner_radius: float, outer_radius: float, thickness: float):
    half_t = thickness * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_t), (outer_radius, half_t)],
        [(inner_radius, -half_t), (inner_radius, half_t)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def _blade_pair_geometry(
    *,
    root_start: float,
    tip_radius: float,
    thickness: float,
    root_width: float,
    mid_width: float,
    tip_width: float,
):
    mid_x = root_start + 0.58 * (tip_radius - root_start)
    blade_profile = [
        (root_start, -root_width * 0.5),
        (mid_x, -mid_width * 0.5),
        (tip_radius, -tip_width * 0.5),
        (tip_radius * 0.988, tip_width * 0.5),
        (mid_x, mid_width * 0.5),
        (root_start, root_width * 0.5),
    ]
    blade = ExtrudeGeometry.centered(blade_profile, thickness, cap=True, closed=True)
    return blade.merge(blade.copy().rotate_z(math.pi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_twin_rotor_drone")

    hub_shell_mat = model.material("hub_shell", rgba=(0.16, 0.17, 0.19, 1.0))
    rotor_blade_mat = model.material("rotor_blade", rgba=(0.05, 0.05, 0.06, 1.0))
    rotor_hub_mat = model.material("rotor_hub", rgba=(0.28, 0.30, 0.33, 1.0))
    axle_mat = model.material("axle_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    leg_mat = model.material("carbon_fiber", rgba=(0.09, 0.10, 0.12, 1.0))
    foot_mat = model.material("landing_foot", rgba=(0.18, 0.18, 0.19, 1.0))

    hub = model.part("hub")
    hub.visual(
        mesh_from_geometry(_hub_shell_geometry(), "hub_shell"),
        material=hub_shell_mat,
        name="body_shell",
    )
    hub.visual(
        Cylinder(radius=0.086, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=hub_shell_mat,
        name="base_plate",
    )
    hub.visual(
        Cylinder(radius=0.014, length=0.224),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=axle_mat,
        name="axle_shaft",
    )

    for z_pos, name in (
        (0.195, "lower_bearing_bottom"),
        (0.215, "lower_bearing_top"),
        (0.245, "upper_bearing_bottom"),
        (0.265, "upper_bearing_top"),
    ):
        hub.visual(
            Cylinder(radius=0.028, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=axle_mat,
            name=name,
        )

    leg_length = 0.136
    leg_radius = 0.0065
    foot_radius = 0.011
    mount_radius = 0.066
    mount_z = 0.082
    splay = 0.88
    embed = 0.006
    vertical_rpy = (0.0, math.pi - splay, 0.0)

    for idx in range(4):
        yaw = idx * math.pi * 0.5
        direction = (
            math.sin(splay) * math.cos(yaw),
            math.sin(splay) * math.sin(yaw),
            -math.cos(splay),
        )
        mount = (
            mount_radius * math.cos(yaw),
            mount_radius * math.sin(yaw),
            mount_z,
        )
        strut_center = (
            mount[0] + direction[0] * (0.5 * leg_length - embed),
            mount[1] + direction[1] * (0.5 * leg_length - embed),
            mount[2] + direction[2] * (0.5 * leg_length - embed),
        )
        foot_center = (
            mount[0] + direction[0] * (leg_length - embed),
            mount[1] + direction[1] * (leg_length - embed),
            mount[2] + direction[2] * (leg_length - embed),
        )
        hub.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(xyz=strut_center, rpy=(vertical_rpy[0], vertical_rpy[1], yaw)),
            material=leg_mat,
            name=f"leg_{idx}_strut",
        )
        hub.visual(
            Sphere(radius=foot_radius),
            origin=Origin(xyz=foot_center),
            material=foot_mat,
            name=f"leg_{idx}_foot",
        )

    rotor_ring = _rotor_ring_geometry(inner_radius=0.020, outer_radius=0.030, thickness=0.018)
    blade_pair = _blade_pair_geometry(
        root_start=0.027,
        tip_radius=0.305,
        thickness=0.005,
        root_width=0.040,
        mid_width=0.031,
        tip_width=0.018,
    )

    lower_rotor = model.part("lower_rotor")
    lower_rotor.visual(
        mesh_from_geometry(rotor_ring, "lower_rotor_ring"),
        material=rotor_hub_mat,
        name="hub_ring",
    )
    lower_rotor.visual(
        mesh_from_geometry(blade_pair.copy(), "lower_rotor_blades"),
        material=rotor_blade_mat,
        name="blade_pair",
    )

    upper_rotor = model.part("upper_rotor")
    upper_rotor.visual(
        mesh_from_geometry(rotor_ring.copy(), "upper_rotor_ring"),
        material=rotor_hub_mat,
        name="hub_ring",
    )
    upper_rotor.visual(
        mesh_from_geometry(blade_pair.copy(), "upper_rotor_blades"),
        origin=Origin(rpy=(0.0, 0.0, math.pi * 0.25)),
        material=rotor_blade_mat,
        name="blade_pair",
    )

    model.articulation(
        "hub_to_lower_rotor",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=lower_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=80.0),
    )
    model.articulation(
        "hub_to_upper_rotor",
        ArticulationType.CONTINUOUS,
        parent=hub,
        child=upper_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub")
    lower_rotor = object_model.get_part("lower_rotor")
    upper_rotor = object_model.get_part("upper_rotor")
    lower_joint = object_model.get_articulation("hub_to_lower_rotor")
    upper_joint = object_model.get_articulation("hub_to_upper_rotor")

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
        "rotor joints are vertical continuous spins",
        lower_joint.articulation_type == ArticulationType.CONTINUOUS
        and upper_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in lower_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 6) for v in upper_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"lower=({lower_joint.articulation_type}, axis={lower_joint.axis}), "
            f"upper=({upper_joint.articulation_type}, axis={upper_joint.axis})"
        ),
    )
    ctx.expect_contact(
        lower_rotor,
        hub,
        elem_a="hub_ring",
        elem_b="lower_bearing_bottom",
        name="lower rotor seats on lower bearing shoulder",
    )
    ctx.expect_contact(
        lower_rotor,
        hub,
        elem_a="hub_ring",
        elem_b="lower_bearing_top",
        name="lower rotor is captured under the inter-rotor spacer shoulder",
    )
    ctx.expect_contact(
        upper_rotor,
        hub,
        elem_a="hub_ring",
        elem_b="upper_bearing_bottom",
        name="upper rotor seats on upper lower bearing shoulder",
    )
    ctx.expect_contact(
        upper_rotor,
        hub,
        elem_a="hub_ring",
        elem_b="upper_bearing_top",
        name="upper rotor is captured below the mast cap shoulder",
    )
    ctx.expect_gap(
        lower_rotor,
        hub,
        axis="z",
        positive_elem="blade_pair",
        negative_elem="body_shell",
        min_gap=0.012,
        max_gap=0.030,
        name="lower rotor disc clears the top of the hub body",
    )
    ctx.expect_gap(
        upper_rotor,
        lower_rotor,
        axis="z",
        positive_elem="blade_pair",
        negative_elem="blade_pair",
        min_gap=0.040,
        max_gap=0.060,
        name="stacked rotor discs keep a realistic vertical separation",
    )
    ctx.expect_origin_distance(
        upper_rotor,
        lower_rotor,
        axes="xy",
        max_dist=0.001,
        name="upper and lower rotors remain coaxial in plan",
    )
    ctx.expect_origin_gap(
        upper_rotor,
        lower_rotor,
        axis="z",
        min_gap=0.045,
        max_gap=0.055,
        name="upper rotor sits above the lower rotor on the shared axle",
    )

    rest_pos = ctx.part_world_position(upper_rotor)
    with ctx.pose({upper_joint: math.pi * 0.5, lower_joint: -math.pi * 0.5}):
        spun_pos = ctx.part_world_position(upper_rotor)
        ctx.expect_gap(
            upper_rotor,
            lower_rotor,
            axis="z",
            positive_elem="blade_pair",
            negative_elem="blade_pair",
            min_gap=0.040,
            max_gap=0.060,
            name="rotor spin keeps the stacked discs vertically separated",
        )
    ctx.check(
        "upper rotor spins in place around the mast",
        rest_pos is not None
        and spun_pos is not None
        and math.dist(rest_pos, spun_pos) <= 1e-9,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
