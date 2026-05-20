from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rotated_xy(radial: float, tangential: float, angle: float) -> tuple[float, float]:
    """Return a point in a local radial/tangential frame rotated about global Z."""
    c = math.cos(angle)
    s = math.sin(angle)
    return (radial * c - tangential * s, radial * s + tangential * c)


def _globe_shell_geometry() -> LatheGeometry:
    """Thin frosted glass globe with a small open neck for the metal collar."""
    radius = 0.080
    wall = 0.004
    center_z = -0.105
    start_theta = 0.43  # leaves a realistic fitter opening at the top of the globe
    steps = 24

    outer_profile: list[tuple[float, float]] = []
    inner_profile: list[tuple[float, float]] = []
    for i in range(steps + 1):
        theta = start_theta + (math.pi - start_theta) * i / steps
        outer_profile.append(
            (radius * math.sin(theta), center_z + radius * math.cos(theta))
        )
        inner_radius = radius - wall
        inner_profile.append(
            (
                inner_radius * math.sin(theta),
                center_z + inner_radius * math.cos(theta),
            )
        )

    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_branching_ceiling_light")

    warm_brass = model.material(
        "satin_warm_brass", rgba=(0.86, 0.63, 0.34, 1.0)
    )
    darker_brass = model.material(
        "shadowed_brass", rgba=(0.45, 0.34, 0.20, 1.0)
    )
    frosted_glass = model.material(
        "frosted_milk_glass", rgba=(0.88, 0.94, 0.96, 0.46)
    )
    warm_bulb = model.material("warm_bulb", rgba=(1.0, 0.82, 0.42, 0.80))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=0.185, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warm_brass,
        name="round_canopy",
    )
    canopy.visual(
        Cylinder(radius=0.205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=warm_brass,
        name="ceiling_plate",
    )
    canopy.visual(
        mesh_from_geometry(TorusGeometry(radius=0.184, tube=0.008), "rolled_canopy_rim"),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=darker_brass,
        name="rolled_rim",
    )
    canopy.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=darker_brass,
        name="central_finial_base",
    )
    canopy.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=warm_brass,
        name="central_finial",
    )

    hinge_radius = 0.205
    branch_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(branch_angles):
        # The canopy-side fork has two tangential cheeks so the rotating branch
        # barrel is visibly captured without interpenetrating the parent.
        for side, tangential in enumerate((-0.015, 0.015)):
            x, y = _rotated_xy(0.197, tangential, angle)
            canopy.visual(
                Box((0.052, 0.008, 0.046)),
                origin=Origin(xyz=(x, y, -0.014), rpy=(0.0, 0.0, angle)),
                material=darker_brass,
                name=f"rim_fork_{index}_{side}",
            )

        arm = model.part(f"arm_{index}")
        arm.visual(
            Cylinder(radius=0.012, length=0.036),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=warm_brass,
            name="hinge_barrel",
        )
        arm.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
            material=darker_brass,
            name="upper_pivot_washer",
        )
        arm.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.021)),
            material=darker_brass,
            name="lower_pivot_washer",
        )
        arm_curve = tube_from_spline_points(
            [
                (0.010, 0.0, 0.000),
                (0.090, 0.0, -0.006),
                (0.225, 0.0, -0.028),
                (0.350, 0.0, -0.060),
            ],
            radius=0.009,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=True,
        )
        arm.visual(
            mesh_from_geometry(arm_curve, f"curved_branch_arm_{index}"),
            material=warm_brass,
            name="curved_arm",
        )
        arm.visual(
            Cylinder(radius=0.008, length=0.090),
            origin=Origin(xyz=(0.348, 0.0, -0.060), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_brass,
            name="shade_fork_bridge",
        )
        for side, y in enumerate((-0.038, 0.038)):
            arm.visual(
                Box((0.076, 0.008, 0.044)),
                origin=Origin(xyz=(0.384, y, -0.064)),
                material=darker_brass,
                name=f"shade_fork_cheek_{side}",
            )

        model.articulation(
            f"canopy_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=arm,
            origin=Origin(
                xyz=(
                    hinge_radius * math.cos(angle),
                    hinge_radius * math.sin(angle),
                    -0.014,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=1.6, lower=-0.35, upper=0.35
            ),
        )

        shade = model.part(f"shade_{index}")
        shade.visual(
            Cylinder(radius=0.010, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_brass,
            name="tilt_pin",
        )
        shade.visual(
            Cylinder(radius=0.009, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, -0.026)),
            material=warm_brass,
            name="socket_stem",
        )
        shade.visual(
            Cylinder(radius=0.036, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=darker_brass,
            name="fitter_collar",
        )
        shade.visual(
            Cylinder(radius=0.018, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=warm_brass,
            name="lamp_socket",
        )
        shade.visual(
            Sphere(radius=0.027),
            origin=Origin(xyz=(0.0, 0.0, -0.104)),
            material=warm_bulb,
            name="warm_bulb",
        )
        shade.visual(
            mesh_from_geometry(_globe_shell_geometry(), f"open_globe_shell_{index}"),
            material=frosted_glass,
            name="globe_shell",
        )

        model.articulation(
            f"arm_{index}_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=shade,
            origin=Origin(xyz=(0.390, 0.0, -0.065)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=1.2, lower=-0.45, upper=0.45
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    swing_joints = [
        object_model.get_articulation(f"canopy_to_arm_{index}") for index in range(3)
    ]
    tilt_joints = [
        object_model.get_articulation(f"arm_{index}_to_shade_{index}")
        for index in range(3)
    ]

    ctx.check(
        "three canopy rim branch hinges",
        len(swing_joints) == 3
        and all(j.motion_limits.lower < 0.0 < j.motion_limits.upper for j in swing_joints),
        details="Expected three limited revolute hinges at the canopy rim.",
    )
    ctx.check(
        "three horizontal shade tilt joints",
        len(tilt_joints) == 3
        and all(j.axis == (0.0, 1.0, 0.0) for j in tilt_joints),
        details="Each globe shade should tilt on a horizontal arm-tip axis.",
    )

    for index in range(3):
        arm = object_model.get_part(f"arm_{index}")
        shade = object_model.get_part(f"shade_{index}")
        for side in range(2):
            ctx.allow_overlap(
                canopy,
                arm,
                elem_a=f"rim_fork_{index}_{side}",
                elem_b="hinge_barrel",
                reason=(
                    "The branch hinge barrel is intentionally seated in the "
                    "canopy-rim fork with a tiny captured-bushing overlap."
                ),
            )
            ctx.expect_overlap(
                canopy,
                arm,
                axes="z",
                min_overlap=0.024,
                elem_a=f"rim_fork_{index}_{side}",
                elem_b="hinge_barrel",
                name=f"arm_{index} hinge barrel is vertically captured in fork {side}",
            )
            ctx.allow_overlap(
                arm,
                shade,
                elem_a=f"shade_fork_cheek_{side}",
                elem_b="tilt_pin",
                reason=(
                    "The shade tilt pin is intentionally captured by the fork "
                    "cheek bushing at the arm tip."
                ),
            )
            ctx.expect_overlap(
                arm,
                shade,
                axes="xz",
                min_overlap=0.012,
                elem_a=f"shade_fork_cheek_{side}",
                elem_b="tilt_pin",
                name=f"shade_{index} tilt pin is captured by cheek {side}",
            )
        ctx.expect_gap(
            canopy,
            shade,
            axis="z",
            min_gap=0.030,
            positive_elem="round_canopy",
            negative_elem="tilt_pin",
            name=f"shade_{index} hangs below the semi-flush canopy",
        )

    shade_0 = object_model.get_part("shade_0")
    rest_shade_origin = ctx.part_world_position(shade_0)
    with ctx.pose({swing_joints[0]: 0.28}):
        swung_shade_origin = ctx.part_world_position(shade_0)
    ctx.check(
        "rim hinge swings branch around canopy",
        rest_shade_origin is not None
        and swung_shade_origin is not None
        and swung_shade_origin[1] > rest_shade_origin[1] + 0.070,
        details=f"rest={rest_shade_origin}, swung={swung_shade_origin}",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_globe_center = _center_from_aabb(
        ctx.part_element_world_aabb(shade_0, elem="globe_shell")
    )
    with ctx.pose({tilt_joints[0]: -0.35}):
        tilted_globe_center = _center_from_aabb(
            ctx.part_element_world_aabb(shade_0, elem="globe_shell")
        )
    ctx.check(
        "shade tilt visibly pitches globe",
        rest_globe_center is not None
        and tilted_globe_center is not None
        and tilted_globe_center[0] > rest_globe_center[0] + 0.025,
        details=f"rest={rest_globe_center}, tilted={tilted_globe_center}",
    )

    return ctx.report()


object_model = build_object_model()
