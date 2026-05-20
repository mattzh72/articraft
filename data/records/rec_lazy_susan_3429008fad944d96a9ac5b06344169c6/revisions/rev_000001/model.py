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


POST_RADIUS = 0.018
LOWER_TIER_Z = 0.100
UPPER_TIER_Z = 0.300


def _build_tier_shell_mesh(
    *,
    outer_radius: float,
    collar_outer_radius: float,
    collar_inner_radius: float,
    name: str,
):
    outer_profile = [
        (collar_outer_radius, -0.020),
        (collar_outer_radius, 0.020),
        (0.060, 0.026),
        (outer_radius, 0.026),
        (outer_radius, 0.052),
    ]
    inner_profile = [
        (collar_inner_radius, -0.018),
        (collar_inner_radius, 0.018),
        (0.072, 0.032),
        (outer_radius - 0.012, 0.032),
        (outer_radius - 0.012, 0.044),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _arc_points(radius: float, z: float, start_angle: float, end_angle: float, samples: int):
    return [
        (
            radius * math.cos(angle),
            radius * math.sin(angle),
            z,
        )
        for angle in [
            start_angle + (end_angle - start_angle) * index / (samples - 1)
            for index in range(samples)
        ]
    ]


def _add_guard_rail(
    tier_part,
    *,
    radius: float,
    rail_z: float,
    post_bottom_z: float,
    start_angle: float,
    end_angle: float,
    prefix: str,
    material,
):
    rail_radius = 0.0045
    rail_points = _arc_points(radius, rail_z, start_angle, end_angle, 9)
    tier_part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                rail_points,
                radius=rail_radius,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
            f"{prefix}_guard_rail_mesh",
        ),
        material=material,
        name=f"{prefix}_guard_rail",
    )

    post_length = (rail_z + rail_radius) - post_bottom_z
    for index, angle in enumerate((start_angle, end_angle)):
        tier_part.visual(
            Cylinder(radius=rail_radius, length=post_length),
            origin=Origin(
                xyz=(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    post_bottom_z + post_length * 0.5,
                )
            ),
            material=material,
            name=f"{prefix}_rail_post_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_lazy_susan")

    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.160, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=graphite,
        name="base_foot",
    )
    stand.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=graphite,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=satin_steel,
        name="center_post",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.438)),
        material=graphite,
        name="top_stop",
    )
    for index, xy in enumerate(((0.100, 0.100), (-0.100, 0.100), (0.0, -0.120))):
        stand.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(xy[0], xy[1], 0.003)),
            material=dark_rubber,
            name=f"foot_pad_{index}",
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, 0.454)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.227)),
    )

    lower_shelf = model.part("lower_shelf")
    lower_shelf.visual(
        _build_tier_shell_mesh(
            outer_radius=0.165,
            collar_outer_radius=0.033,
            collar_inner_radius=0.0192,
            name="lower_tier_shell_mesh",
        ),
        material=satin_steel,
        name="lower_tier_shell",
    )
    _add_guard_rail(
        lower_shelf,
        radius=0.147,
        rail_z=0.064,
        post_bottom_z=0.032,
        start_angle=-0.85,
        end_angle=0.85,
        prefix="lower",
        material=graphite,
    )
    lower_shelf.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=0.093),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    upper_shelf = model.part("upper_shelf")
    upper_shelf.visual(
        _build_tier_shell_mesh(
            outer_radius=0.135,
            collar_outer_radius=0.031,
            collar_inner_radius=0.0192,
            name="upper_tier_shell_mesh",
        ),
        material=satin_steel,
        name="upper_tier_shell",
    )
    _add_guard_rail(
        upper_shelf,
        radius=0.117,
        rail_z=0.062,
        post_bottom_z=0.032,
        start_angle=-0.70,
        end_angle=0.95,
        prefix="upper",
        material=graphite,
    )
    upper_shelf.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.090),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "stand_to_lower_shelf",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=lower_shelf,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TIER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "stand_to_upper_shelf",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=upper_shelf,
        origin=Origin(xyz=(0.0, 0.0, UPPER_TIER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    stand = object_model.get_part("stand")
    lower_shelf = object_model.get_part("lower_shelf")
    upper_shelf = object_model.get_part("upper_shelf")
    lower_joint = object_model.get_articulation("stand_to_lower_shelf")
    upper_joint = object_model.get_articulation("stand_to_upper_shelf")
    base_foot = stand.get_visual("base_foot")
    lower_guard = lower_shelf.get_visual("lower_guard_rail")
    upper_guard = upper_shelf.get_visual("upper_guard_rail")

    ctx.allow_isolated_part(
        lower_shelf,
        reason="The lower tier is intentionally carried on a bearing collar with operating clearance around the fixed center post.",
    )
    ctx.allow_isolated_part(
        upper_shelf,
        reason="The upper tier is intentionally carried on its own separate bearing collar with operating clearance around the fixed center post.",
    )

    ctx.expect_origin_distance(
        lower_shelf,
        stand,
        axes="xy",
        max_dist=1e-6,
        name="lower shelf stays centered on the post",
    )
    ctx.expect_origin_distance(
        upper_shelf,
        stand,
        axes="xy",
        max_dist=1e-6,
        name="upper shelf stays centered on the post",
    )
    ctx.expect_gap(
        lower_shelf,
        stand,
        axis="z",
        min_gap=0.050,
        positive_elem="lower_tier_shell",
        negative_elem=base_foot,
        name="lower tier clears the weighted base",
    )
    ctx.expect_gap(
        upper_shelf,
        lower_shelf,
        axis="z",
        min_gap=0.090,
        name="upper tier sits clearly above the lower tier",
    )

    ctx.check(
        "lower shelf rotates about a vertical axis",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper shelf rotates about its own vertical axis",
        tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={upper_joint.axis}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    lower_rest = _aabb_center(ctx.part_element_world_aabb(lower_shelf, elem=lower_guard))
    upper_rest = _aabb_center(ctx.part_element_world_aabb(upper_shelf, elem=upper_guard))

    with ctx.pose({lower_joint: math.pi / 2.0, upper_joint: 0.0}):
        lower_turned = _aabb_center(ctx.part_element_world_aabb(lower_shelf, elem=lower_guard))
        upper_steady = _aabb_center(ctx.part_element_world_aabb(upper_shelf, elem=upper_guard))

    ctx.check(
        "lower tier guard rail moves when the lower shelf rotates",
        lower_rest is not None
        and lower_turned is not None
        and abs(lower_rest[0] - lower_turned[0]) > 0.05
        and abs(lower_rest[1] - lower_turned[1]) > 0.05,
        details=f"rest={lower_rest}, turned={lower_turned}",
    )
    ctx.check(
        "upper tier remains fixed while only the lower shelf rotates",
        upper_rest is not None
        and upper_steady is not None
        and abs(upper_rest[0] - upper_steady[0]) < 0.005
        and abs(upper_rest[1] - upper_steady[1]) < 0.005,
        details=f"rest={upper_rest}, steady={upper_steady}",
    )

    with ctx.pose({lower_joint: 0.0, upper_joint: -math.pi / 2.0}):
        lower_steady = _aabb_center(ctx.part_element_world_aabb(lower_shelf, elem=lower_guard))
        upper_turned = _aabb_center(ctx.part_element_world_aabb(upper_shelf, elem=upper_guard))

    ctx.check(
        "upper tier guard rail moves when the upper shelf rotates",
        upper_rest is not None
        and upper_turned is not None
        and abs(upper_rest[0] - upper_turned[0]) > 0.04
        and abs(upper_rest[1] - upper_turned[1]) > 0.04,
        details=f"rest={upper_rest}, turned={upper_turned}",
    )
    ctx.check(
        "lower tier remains fixed while only the upper shelf rotates",
        lower_rest is not None
        and lower_steady is not None
        and abs(lower_rest[0] - lower_steady[0]) < 0.005
        and abs(lower_rest[1] - lower_steady[1]) < 0.005,
        details=f"rest={lower_rest}, steady={lower_steady}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
