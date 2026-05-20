from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BLADE_COUNT = 5
OUTER_RADIUS = 0.704
SPIN_JOINT = "mount_to_blade_assembly"


def _solid_of_revolution(profile: list[tuple[float, float]], name: str):
    return mesh_from_geometry(LatheGeometry(profile, segments=96), name)


def _blade_panel_geometry(angle: float) -> MeshGeometry:
    """A single pitched, tapered ceiling-fan paddle blade."""
    root_x = 0.300
    tip_x = 0.680
    root_half = 0.065
    tip_half = 0.095
    z_center = -0.066
    thickness = 0.009
    pitch = math.radians(9.0)

    profile = [
        (root_x, -0.035),
        (root_x + 0.035, -root_half),
        (tip_x - 0.070, -tip_half),
        (tip_x - 0.005, -0.080),
        (tip_x + 0.020, -0.035),
        (tip_x + 0.024, 0.000),
        (tip_x + 0.020, 0.035),
        (tip_x - 0.005, 0.080),
        (tip_x - 0.070, tip_half),
        (root_x + 0.035, root_half),
        (root_x, 0.035),
        (root_x - 0.014, 0.000),
    ]

    geom = MeshGeometry()
    bottom: list[int] = []
    top: list[int] = []
    for x, y in profile:
        mid_z = z_center + y * math.sin(pitch)
        bottom.append(geom.add_vertex(x, y, mid_z - thickness * 0.5))
    for x, y in profile:
        mid_z = z_center + y * math.sin(pitch)
        top.append(geom.add_vertex(x, y, mid_z + thickness * 0.5))

    # Triangulate caps and side wall.  The profile is ordered around the blade.
    for i in range(1, len(profile) - 1):
        geom.add_face(bottom[0], bottom[i + 1], bottom[i])
        geom.add_face(top[0], top[i], top[i + 1])
    for i in range(len(profile)):
        j = (i + 1) % len(profile)
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])

    return geom.rotate_z(angle)


def _radial_xy(radius: float, offset: float, angle: float) -> tuple[float, float]:
    """Point at radial distance plus tangential offset for a blade at angle."""
    return (
        radius * math.cos(angle) - offset * math.sin(angle),
        radius * math.sin(angle) + offset * math.cos(angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_hugger_ceiling_fan")

    satin_white = model.material("satin_white", rgba=(0.92, 0.91, 0.86, 1.0))
    warm_blade = model.material("warm_maple_blade", rgba=(0.78, 0.63, 0.42, 1.0))
    brushed_metal = model.material("brushed_nickel", rgba=(0.55, 0.56, 0.55, 1.0))
    shadow = model.material("dark_shadow", rgba=(0.10, 0.10, 0.10, 1.0))

    mount = model.part("ceiling_mount")
    mount.visual(
        _solid_of_revolution(
            [
                (0.000, 0.000),
                (0.315, 0.000),
                (0.315, -0.032),
                (0.245, -0.032),
                (0.258, -0.052),
                (0.270, -0.102),
                (0.255, -0.154),
                (0.215, -0.197),
                (0.142, -0.226),
                (0.058, -0.236),
                (0.000, -0.236),
            ],
            "ceiling_plate_and_housing",
        ),
        material=satin_white,
        name="plate_housing",
    )
    mount.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
        material=shadow,
        name="bearing_shadow_ring",
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        _solid_of_revolution(
            [
                (0.000, 0.000),
                (0.052, 0.000),
                (0.135, -0.005),
                (0.185, -0.021),
                (0.196, -0.050),
                (0.166, -0.081),
                (0.086, -0.104),
                (0.000, -0.108),
            ],
            "rotating_motor_hub",
        ),
        material=satin_white,
        name="motor_hub",
    )

    for index in range(BLADE_COUNT):
        theta = index * math.tau / BLADE_COUNT
        yaw = theta

        blade_assembly.visual(
            mesh_from_geometry(_blade_panel_geometry(theta), f"blade_{index}"),
            material=warm_blade,
            name=f"blade_{index}",
        )

        # A low-profile blade iron: central neck from hub, cross bridge, and two
        # fork straps bolted to the blade root.
        neck_xy = _radial_xy(0.215, 0.0, theta)
        blade_assembly.visual(
            Box((0.185, 0.034, 0.018)),
            origin=Origin(xyz=(neck_xy[0], neck_xy[1], -0.052), rpy=(0.0, 0.0, yaw)),
            material=brushed_metal,
            name=f"iron_neck_{index}",
        )
        bridge_xy = _radial_xy(0.285, 0.0, theta)
        blade_assembly.visual(
            Box((0.052, 0.138, 0.016)),
            origin=Origin(xyz=(bridge_xy[0], bridge_xy[1], -0.052), rpy=(0.0, 0.0, yaw)),
            material=brushed_metal,
            name=f"iron_bridge_{index}",
        )
        for side, offset in enumerate((-0.046, 0.046)):
            fork_xy = _radial_xy(0.333, offset, theta)
            blade_assembly.visual(
                Box((0.112, 0.026, 0.016)),
                origin=Origin(xyz=(fork_xy[0], fork_xy[1], -0.052), rpy=(0.0, 0.0, yaw)),
                material=brushed_metal,
                name=f"iron_fork_{index}_{side}",
            )
            screw_xy = _radial_xy(0.350, offset, theta)
            blade_assembly.visual(
                Cylinder(radius=0.012, length=0.005),
                origin=Origin(xyz=(screw_xy[0], screw_xy[1], -0.043)),
                material=shadow,
                name=f"screw_{index}_{side}",
            )

    model.articulation(
        SPIN_JOINT,
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.246)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("ceiling_mount")
    blades = object_model.get_part("blade_assembly")
    spin = object_model.get_articulation(SPIN_JOINT)

    ctx.check(
        "continuous vertical blade spin",
        spin is not None
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"spin={spin!r}",
    )
    ctx.check(
        "five separate ceiling fan blades",
        blades is not None and sum(1 for v in blades.visuals if v.name.startswith("blade_")) == BLADE_COUNT,
        details="Expected five named blade visuals on the rotating assembly.",
    )
    if mount is not None and blades is not None:
        ctx.expect_gap(
            mount,
            blades,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="bearing_shadow_ring",
            negative_elem="motor_hub",
            name="rotating hub has close bearing clearance",
        )
        ctx.expect_overlap(
            mount,
            blades,
            axes="xy",
            min_overlap=0.12,
            elem_a="plate_housing",
            elem_b="motor_hub",
            name="motor hub centered under ceiling housing",
        )

        aabb = ctx.part_world_aabb(blades)
        if aabb is not None:
            mins, maxs = aabb
            diameter = max(float(maxs[0] - mins[0]), float(maxs[1] - mins[1]))
            ctx.check(
                "believable hugger fan sweep",
                1.30 <= diameter <= 1.46,
                details=f"diameter={diameter:.3f}",
            )

        blade0_aabb = ctx.part_element_world_aabb(blades, elem="blade_0")
        with ctx.pose({spin: math.pi / 5.0}):
            moved_aabb = ctx.part_element_world_aabb(blades, elem="blade_0")
        if blade0_aabb is not None and moved_aabb is not None:
            rest_center = (
                0.5 * (blade0_aabb[0][0] + blade0_aabb[1][0]),
                0.5 * (blade0_aabb[0][1] + blade0_aabb[1][1]),
            )
            moved_center = (
                0.5 * (moved_aabb[0][0] + moved_aabb[1][0]),
                0.5 * (moved_aabb[0][1] + moved_aabb[1][1]),
            )
            ctx.check(
                "blade assembly rotates with joint pose",
                moved_center[1] > rest_center[1] + 0.20 and moved_center[0] < rest_center[0],
                details=f"rest={rest_center!r}, moved={moved_center!r}",
            )

    return ctx.report()


object_model = build_object_model()
