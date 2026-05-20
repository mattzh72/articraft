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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dirt_jump_bike_fork_front_end")

    raw_aluminum = model.material("raw_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    blasted_alloy = model.material("blasted_alloy", rgba=(0.66, 0.68, 0.71, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    head_tilt = math.radians(20.0)
    head_axis_rpy = (0.0, -head_tilt, 0.0)

    head_frame = model.part("head_frame")

    head_tube_shell = LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.064),
            (0.036, -0.054),
            (0.036, 0.054),
            (0.034, 0.064),
        ],
        [
            (0.024, -0.064),
            (0.024, 0.064),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    head_frame.visual(
        mesh_from_geometry(head_tube_shell, "dj_head_tube_shell"),
        origin=Origin(rpy=head_axis_rpy),
        material=dark_anodized,
        name="head_tube_shell",
    )

    head_frame.inertial = Inertial.from_geometry(
        Box((0.11, 0.08, 0.16)),
        mass=1.4,
        origin=Origin(),
    )

    steering = model.part("steering_assembly")

    steering.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=raw_aluminum,
        name="steerer_tube",
    )
    steering.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=blasted_alloy,
        name="upper_headset_cover",
    )
    steering.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.069)),
        material=blasted_alloy,
        name="lower_crown_race",
    )

    steering.visual(
        Box((0.102, 0.164, 0.030)),
        origin=Origin(xyz=(0.024, 0.0, -0.110)),
        material=blasted_alloy,
        name="crown_main",
    )
    steering.visual(
        Box((0.070, 0.118, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, -0.088)),
        material=raw_aluminum,
        name="crown_top_rib",
    )
    steering.visual(
        Box((0.032, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.106)),
        material=blasted_alloy,
        name="steerer_crown_boss",
    )

    leg_y = 0.064
    leg_x = 0.046
    leg_length = 0.390
    leg_center_z = -0.304
    dropout_z = -0.505
    for side_name, side_y in (("left", leg_y), ("right", -leg_y)):
        steering.visual(
            Cylinder(radius=0.018, length=leg_length),
            origin=Origin(xyz=(leg_x, side_y, leg_center_z)),
            material=raw_aluminum,
            name=f"{side_name}_leg",
        )
        steering.visual(
            Box((0.030, 0.018, 0.038)),
            origin=Origin(xyz=(leg_x + 0.006, side_y, dropout_z)),
            material=blasted_alloy,
            name=f"{side_name}_dropout",
        )

    steering.visual(
        Cylinder(radius=0.024, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=blasted_alloy,
        name="steerer_clamp",
    )
    steering.visual(
        Box((0.070, 0.050, 0.030)),
        origin=Origin(xyz=(0.032, 0.0, 0.128)),
        material=blasted_alloy,
        name="stem_body",
    )
    steering.visual(
        Box((0.046, 0.060, 0.034)),
        origin=Origin(xyz=(0.065, 0.0, 0.144)),
        material=blasted_alloy,
        name="bar_clamp_body",
    )
    steering.visual(
        Box((0.016, 0.058, 0.044)),
        origin=Origin(xyz=(0.085, 0.0, 0.144)),
        material=raw_aluminum,
        name="faceplate",
    )

    bolt_y = 0.021
    bolt_z = 0.014
    for index, (sy, sz) in enumerate(
        (
            (bolt_y, bolt_z),
            (-bolt_y, bolt_z),
            (bolt_y, -bolt_z),
            (-bolt_y, -bolt_z),
        )
    ):
        steering.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(
                xyz=(0.095, sy, 0.144 + sz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_anodized,
            name=f"faceplate_bolt_{index + 1}",
        )

    steering.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_anodized,
        name="spacer_stack",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=dark_anodized,
        name="top_cap",
    )

    bar_geom = tube_from_spline_points(
        [
            (0.014, -0.390, 0.152),
            (0.026, -0.300, 0.151),
            (0.043, -0.185, 0.148),
            (0.056, -0.090, 0.145),
            (0.056, 0.090, 0.145),
            (0.043, 0.185, 0.148),
            (0.026, 0.300, 0.151),
            (0.014, 0.390, 0.152),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    steering.visual(
        mesh_from_geometry(bar_geom, "dj_handlebar"),
        material=raw_aluminum,
        name="handlebar",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.014, -0.357, 0.152), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="left_grip",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.014, 0.357, 0.152), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="right_grip",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.86, 0.18, 0.74)),
        mass=4.3,
        origin=Origin(xyz=(0.02, 0.0, -0.17)),
    )

    model.articulation(
        "head_tube_steer",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=steering,
        origin=Origin(rpy=head_axis_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=4.0,
            lower=-0.80,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_frame = object_model.get_part("head_frame")
    steering = object_model.get_part("steering_assembly")
    steer = object_model.get_articulation("head_tube_steer")

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
    ctx.allow_overlap(
        head_frame,
        steering,
        elem_a="head_tube_shell",
        elem_b="upper_headset_cover",
        reason="The simplified headset cover is intentionally nested onto the top of the head-tube shell to represent the bearing cap interface.",
    )
    ctx.allow_overlap(
        head_frame,
        steering,
        elem_a="head_tube_shell",
        elem_b="lower_crown_race",
        reason="The lower crown race is intentionally represented as nested against the lower head-tube bearing seat.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    left_grip = steering.get_visual("left_grip")
    right_grip = steering.get_visual("right_grip")
    faceplate = steering.get_visual("faceplate")
    crown = steering.get_visual("crown_main")
    bar_clamp = steering.get_visual("bar_clamp_body")

    ctx.check(
        "key steering visuals exist",
        all(v is not None for v in (left_grip, right_grip, faceplate, crown, bar_clamp)),
        details="Missing one of left_grip, right_grip, faceplate, crown_main, or bar_clamp_body.",
    )

    crown_center = _aabb_center(ctx.part_element_world_aabb(steering, elem="crown_main"))
    head_center = _aabb_center(ctx.part_element_world_aabb(head_frame, elem="head_tube_shell"))
    faceplate_center = _aabb_center(ctx.part_element_world_aabb(steering, elem="faceplate"))
    bar_clamp_center = _aabb_center(ctx.part_element_world_aabb(steering, elem="bar_clamp_body"))
    left_rest = _aabb_center(ctx.part_element_world_aabb(steering, elem="left_grip"))
    right_rest = _aabb_center(ctx.part_element_world_aabb(steering, elem="right_grip"))

    ctx.check(
        "crown sits below the head tube and stem clamp sits above it",
        crown_center is not None
        and head_center is not None
        and bar_clamp_center is not None
        and crown_center[2] < head_center[2] - 0.05
        and bar_clamp_center[2] > head_center[2] + 0.07,
        details=(
            f"crown_center={crown_center}, head_center={head_center}, "
            f"bar_clamp_center={bar_clamp_center}"
        ),
    )
    ctx.check(
        "faceplate is forward of the stem clamp body",
        faceplate_center is not None
        and bar_clamp_center is not None
        and faceplate_center[0] > bar_clamp_center[0] + 0.015,
        details=f"faceplate_center={faceplate_center}, bar_clamp_center={bar_clamp_center}",
    )
    ctx.check(
        "wide dirt jump bar span is present",
        left_rest is not None
        and right_rest is not None
        and abs(left_rest[1] - right_rest[1]) > 0.68,
        details=f"left_rest={left_rest}, right_rest={right_rest}",
    )

    with ctx.pose({steer: 0.45}):
        left_turned = _aabb_center(ctx.part_element_world_aabb(steering, elem="left_grip"))
        right_turned = _aabb_center(ctx.part_element_world_aabb(steering, elem="right_grip"))

    ctx.check(
        "positive steering rotates the bars around the head axis",
        left_rest is not None
        and right_rest is not None
        and left_turned is not None
        and right_turned is not None
        and left_turned[0] > left_rest[0] + 0.08
        and right_turned[0] < right_rest[0] - 0.08,
        details=(
            f"left_rest={left_rest}, left_turned={left_turned}, "
            f"right_rest={right_rest}, right_turned={right_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
