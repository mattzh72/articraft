from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 56,
):
    half = length * 0.5
    return _mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=segments,
        ),
    )


def _arm_tube_mesh(name: str, angle: float, *, r0: float, r1: float, radius: float, z: float = 0.0):
    c = math.cos(angle)
    s = math.sin(angle)
    return _mesh(
        name,
        tube_from_spline_points(
            [
                (r0 * c, r0 * s, z),
                (r1 * c, r1 * s, z),
            ],
            radius=radius,
            samples_per_segment=2,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def _brace_tube_mesh(
    name: str,
    angle: float,
    *,
    start_r: float,
    start_z: float,
    end_r: float,
    end_z: float,
    radius: float,
):
    c = math.cos(angle)
    s = math.sin(angle)
    return _mesh(
        name,
        tube_from_spline_points(
            [
                (start_r * c, start_r * s, start_z),
                (end_r * c, end_r * s, end_z),
            ],
            radius=radius,
            samples_per_segment=2,
            radial_segments=16,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_turnstile")

    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.73, 0.10, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    zinc = model.material("zinc", rgba=(0.60, 0.62, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    support = model.part("support_frame")
    support.visual(
        Box((0.56, 0.50, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=charcoal,
        name="base_plate",
    )
    for side_sign in (-1.0, 1.0):
        tag = "right" if side_sign > 0.0 else "left"
        support.visual(
            Box((0.20, 0.38, 0.04)),
            origin=Origin(xyz=(0.10, side_sign * 0.43, 0.02)),
            material=charcoal,
            name=f"{tag}_runner_bridge",
        )
        support.visual(
            Box((0.94, 0.08, 0.04)),
            origin=Origin(xyz=(0.47, side_sign * 0.62, 0.02)),
            material=charcoal,
            name=f"{tag}_runner",
        )
        for post_x, post_name in ((0.28, "front"), (0.90, "end")):
            support.visual(
                Cylinder(radius=0.018, length=0.92),
                origin=Origin(xyz=(post_x, side_sign * 0.62, 0.50)),
                material=safety_yellow,
                name=f"{tag}_{post_name}_guard_post",
            )
        support.visual(
            Cylinder(radius=0.016, length=0.62),
            origin=Origin(
                xyz=(0.59, side_sign * 0.62, 0.45),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=safety_yellow,
            name=f"{tag}_lower_guard_rail",
        )
        support.visual(
            Cylinder(radius=0.016, length=0.62),
            origin=Origin(
                xyz=(0.59, side_sign * 0.62, 0.92),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=safety_yellow,
            name=f"{tag}_upper_guard_rail",
        )
        support.visual(
            Cylinder(radius=0.012, length=0.47),
            origin=Origin(xyz=(0.59, side_sign * 0.62, 0.685)),
            material=safety_yellow,
            name=f"{tag}_mid_guard_stanchion",
        )

    support.visual(
        Box((0.18, 0.18, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
        material=safety_yellow,
        name="central_post",
    )
    support.visual(
        Cylinder(radius=0.050, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        material=charcoal,
        name="bearing_pedestal",
    )
    support.visual(
        Cylinder(radius=0.095, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.907)),
        material=dark_steel,
        name="lower_bearing_collar",
    )
    support.visual(
        Cylinder(radius=0.044, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.889)),
        material=brushed_steel,
        name="spindle_core",
    )
    support.visual(
        Cylinder(radius=0.070, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.907)),
        material=dark_steel,
        name="retention_collar",
    )
    support.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.893)),
        material=charcoal,
        name="top_weather_cap",
    )

    for y_sign, name in ((1.0, "front_base_web"), (-1.0, "rear_base_web")):
        support.visual(
            Box((0.14, 0.012, 0.18)),
            origin=Origin(xyz=(0.07, y_sign * 0.09, 0.09)),
            material=charcoal,
            name=name,
        )
    for x_sign, name in ((1.0, "right_base_web"), (-1.0, "left_base_web")):
        support.visual(
            Box((0.012, 0.14, 0.18)),
            origin=Origin(xyz=(x_sign * 0.09, 0.07, 0.09)),
            material=charcoal,
            name=name,
        )

    support.visual(
        Box((0.12, 0.10, 0.04)),
        origin=Origin(xyz=(0.13, 0.07, 0.80)),
        material=charcoal,
        name="lockout_mount",
    )
    support.visual(
        Box((0.026, 0.032, 0.10)),
        origin=Origin(xyz=(0.185, 0.085, 0.85)),
        material=charcoal,
        name="lockout_riser",
    )
    support.visual(
        Box((0.10, 0.05, 0.06)),
        origin=Origin(xyz=(0.23, 0.085, 0.87)),
        material=charcoal,
        name="lockout_housing",
    )
    support.visual(
        Box((0.026, 0.050, 0.016)),
        origin=Origin(xyz=(0.194, 0.035, 0.853)),
        material=dark_steel,
        name="lockout_pawl",
    )
    support.visual(
        Cylinder(radius=0.013, length=0.036),
        origin=Origin(
            xyz=(0.245, 0.118, 0.890),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="lockout_actuator",
    )
    support.visual(
        Box((0.028, 0.14, 0.022)),
        origin=Origin(xyz=(0.230, 0.0, 0.847)),
        material=charcoal,
        name="stop_yoke_plate",
    )
    support.visual(
        Box((0.040, 0.024, 0.030)),
        origin=Origin(xyz=(0.236, 0.058, 0.859)),
        material=dark_steel,
        name="stop_block_pos",
    )
    support.visual(
        Box((0.040, 0.024, 0.030)),
        origin=Origin(xyz=(0.236, -0.058, 0.859)),
        material=dark_steel,
        name="stop_block_neg",
    )

    for idx, (x_pos, y_pos) in enumerate(
        [
            (-0.20, -0.18),
            (-0.20, 0.18),
            (0.20, -0.18),
            (0.20, 0.18),
            (0.28, -0.62),
            (0.28, 0.62),
            (0.90, -0.62),
            (0.90, 0.62),
        ]
    ):
        support.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, 0.024)),
            material=brushed_steel,
            name=f"anchor_bolt_{idx}",
        )
    for idx, (x_pos, z_pos) in enumerate(
        [
            (0.205, 0.854),
            (0.205, 0.886),
            (0.255, 0.854),
            (0.255, 0.886),
        ]
    ):
        support.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(
                xyz=(x_pos, 0.111, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"lockout_bolt_{idx}",
        )

    support.inertial = Inertial.from_geometry(
        Box((1.02, 1.32, 1.12)),
        mass=145.0,
        origin=Origin(xyz=(0.39, 0.0, 0.56)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.095, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_steel,
        name="bearing_sleeve",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=charcoal,
        name="top_hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.125, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=charcoal,
        name="bottom_hub_flange",
    )

    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        c = math.cos(angle)
        s = math.sin(angle)

        rotor.visual(
            _arm_tube_mesh(
                f"turnstile_arm_main_{index}",
                angle,
                r0=0.125,
                r1=0.455,
                radius=0.022,
            ),
            material=zinc,
            name=f"arm_{index}",
        )
        rotor.visual(
            Box((0.055, 0.012, 0.040)),
            origin=Origin(
                xyz=(0.145 * c, 0.145 * s, 0.020),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"arm_brace_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.024),
            origin=Origin(
                xyz=(0.467 * c, 0.467 * s, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=black_rubber,
            name=f"arm_end_bumper_{index}",
        )
        rotor.visual(
            Box((0.050, 0.028, 0.028)),
            origin=Origin(
                xyz=(0.150 * c, 0.150 * s, -0.060),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"stop_lug_{index}",
        )
        rotor.visual(
            Box((0.030, 0.018, 0.060)),
            origin=Origin(
                xyz=(0.125 * c, 0.125 * s, -0.030),
                rpy=(0.0, 0.0, angle),
            ),
            material=charcoal,
            name=f"stop_web_{index}",
        )

    for index in range(6):
        angle = index * (2.0 * math.pi / 6.0) + (math.pi / 6.0)
        rotor.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(
                xyz=(0.094 * math.cos(angle), 0.094 * math.sin(angle), 0.108),
            ),
            material=brushed_steel,
            name=f"hub_bolt_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((0.98, 0.98, 0.26)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    model.articulation(
        "support_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("support_to_rotor")

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
        "rotor articulation is vertical continuous spindle rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (0.0, 0.0, 1.0),
        f"expected continuous vertical axis, got type={spin.articulation_type} axis={spin.axis}",
    )
    ctx.expect_origin_distance(
        rotor,
        support,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="rotor stays centered on support post",
    )
    ctx.expect_contact(
        rotor,
        support,
        elem_a="bottom_hub_flange",
        elem_b="retention_collar",
        name="rotor thrust flange seats on support collar",
    )
    ctx.expect_gap(
        support,
        rotor,
        axis="x",
        positive_elem="lockout_pawl",
        negative_elem="stop_lug_0",
        min_gap=0.004,
        max_gap=0.020,
        name="lockout pawl stands off from stop lug without overlap",
    )
    ctx.expect_gap(
        support,
        rotor,
        axis="y",
        positive_elem="lockout_housing",
        negative_elem="arm_0",
        min_gap=0.03,
        max_gap=0.08,
        name="lockout housing stays outboard of arm sweep",
    )
    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_gap(
            support,
            rotor,
            axis="y",
            positive_elem="right_upper_guard_rail",
            negative_elem="arm_0",
            min_gap=0.12,
            name="rotated arm clears right guard rail",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
