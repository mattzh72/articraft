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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _build_slider_block_mesh():
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.078, 0.060, 0.014, corner_segments=6),
        [_circle_profile(0.017, segments=36)],
        0.110,
        cap=True,
        center=True,
        closed=True,
    )


def _build_shade_shell_mesh():
    outer_profile = [
        (0.010, 0.000),
        (0.016, 0.018),
        (0.040, 0.080),
        (0.078, 0.175),
        (0.100, 0.220),
    ]
    inner_profile = [
        (0.0075, 0.002),
        (0.0130, 0.020),
        (0.0360, 0.080),
        (0.0730, 0.174),
        (0.0950, 0.218),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tension_pole_floor_lamp")

    brass = model.material("brass", rgba=(0.72, 0.60, 0.33, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.89, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))

    tension_pole = model.part("tension_pole")
    tension_pole.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=charcoal,
        name="floor_pad",
    )
    tension_pole.visual(
        Cylinder(radius=0.023, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=brass,
        name="lower_socket",
    )
    tension_pole.visual(
        Cylinder(radius=0.0135, length=2.180),
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        material=brass,
        name="pole_shaft",
    )
    tension_pole.visual(
        Cylinder(radius=0.019, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 2.305)),
        material=brass,
        name="tension_housing",
    )
    tension_pole.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 2.420)),
        material=satin_steel,
        name="tension_rod",
    )
    tension_pole.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 2.487)),
        material=charcoal,
        name="ceiling_pad",
    )
    tension_pole.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 2.494)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 1.247)),
    )

    arm_bracket = model.part("arm_bracket")
    arm_bracket.visual(
        mesh_from_geometry(_build_slider_block_mesh(), "lamp_slider_block"),
        material=brass,
        name="slider_block",
    )
    guide_pad_radius = 0.003
    guide_pad_offset = 0.0135 + guide_pad_radius
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        arm_bracket.visual(
            Cylinder(radius=guide_pad_radius, length=0.110),
            origin=Origin(
                xyz=(
                    guide_pad_offset * math.cos(angle),
                    guide_pad_offset * math.sin(angle),
                    0.0,
                )
            ),
            material=satin_steel,
            name=f"guide_pad_{index}",
        )
    arm_bracket.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.012, 0.038, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="clamp_stem",
    )
    arm_bracket.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.012, 0.052, 0.000)),
        material=charcoal,
        name="clamp_knob",
    )
    arm_bracket.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.037, 0.0, 0.008),
                    (0.130, 0.0, 0.022),
                    (0.225, 0.0, 0.050),
                    (0.279, 0.0, 0.074),
                ],
                radius=0.009,
                samples_per_segment=20,
                radial_segments=18,
                cap_ends=True,
            ),
            "lamp_support_arm",
        ),
        material=brass,
        name="support_arm",
    )
    arm_bracket.visual(
        Box((0.014, 0.030, 0.022)),
        origin=Origin(xyz=(0.287, 0.0, 0.074)),
        material=brass,
        name="pivot_block",
    )
    arm_bracket.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.305, -0.018, 0.074), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_yoke",
    )
    arm_bracket.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.305, 0.018, 0.074), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_yoke",
    )
    arm_bracket.inertial = Inertial.from_geometry(
        Box((0.340, 0.080, 0.150)),
        mass=1.6,
        origin=Origin(xyz=(0.170, 0.0, 0.035)),
    )

    model.articulation(
        "pole_to_arm_bracket",
        ArticulationType.PRISMATIC,
        parent=tension_pole,
        child=arm_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=0.780,
        ),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0105, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="pivot_barrel",
    )
    shade.visual(
        Cylinder(radius=0.0085, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="socket_neck",
    )
    shade.visual(
        mesh_from_geometry(_build_shade_shell_mesh(), "lamp_cone_shade"),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.250, 0.220, 0.220)),
        mass=0.55,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    model.articulation(
        "bracket_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm_bracket,
        child=shade,
        origin=Origin(xyz=(0.305, 0.0, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.70,
            upper=0.90,
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
    tension_pole = object_model.get_part("tension_pole")
    arm_bracket = object_model.get_part("arm_bracket")
    shade = object_model.get_part("shade")
    slide_joint = object_model.get_articulation("pole_to_arm_bracket")
    tilt_joint = object_model.get_articulation("bracket_to_shade")

    ctx.expect_origin_distance(
        arm_bracket,
        tension_pole,
        axes="xy",
        max_dist=0.001,
        name="arm bracket stays centered on the tension pole",
    )
    ctx.expect_gap(
        shade,
        tension_pole,
        axis="x",
        min_gap=0.235,
        name="shade projects forward from the pole",
    )

    rest_bracket_pos = ctx.part_world_position(arm_bracket)
    with ctx.pose({slide_joint: 0.780}):
        ctx.expect_origin_distance(
            arm_bracket,
            tension_pole,
            axes="xy",
            max_dist=0.001,
            name="raised bracket remains aligned to the pole axis",
        )
        raised_bracket_pos = ctx.part_world_position(arm_bracket)
    ctx.check(
        "arm bracket slides upward along the pole",
        rest_bracket_pos is not None
        and raised_bracket_pos is not None
        and raised_bracket_pos[2] > rest_bracket_pos[2] + 0.70,
        details=f"rest={rest_bracket_pos}, raised={raised_bracket_pos}",
    )

    shade_shell_rest = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({tilt_joint: 0.75}):
        shade_shell_tilted = ctx.part_element_world_aabb(shade, elem="shade_shell")
    if shade_shell_rest is not None and shade_shell_tilted is not None:
        rest_center_z = (shade_shell_rest[0][2] + shade_shell_rest[1][2]) * 0.5
        tilted_center_z = (shade_shell_tilted[0][2] + shade_shell_tilted[1][2]) * 0.5
        ctx.check(
            "positive shade tilt lifts the cone shade",
            tilted_center_z > rest_center_z + 0.045,
            details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
        )
    else:
        ctx.fail("positive shade tilt lifts the cone shade", "missing shade shell bounds")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
