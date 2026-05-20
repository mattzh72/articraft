from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
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


def _tube_shell(inner_radius: float, outer_radius: float, length: float, *, segments: int = 48):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )


def _lower_leg_mesh(name: str):
    outer_profile = [
        (0.026, -0.230),
        (0.025, -0.170),
        (0.024, -0.040),
        (0.0225, 0.130),
    ]
    inner_profile = [
        (0.0188, -0.224),
        (0.0188, 0.112),
        (0.0170, 0.118),
        (0.0170, 0.124),
    ]
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=112,
    )
    geom.merge(BoxGeometry((0.034, 0.030, 0.040)).translate(0.0, 0.0, -0.245))
    geom.merge(BoxGeometry((0.040, 0.022, 0.018)).translate(0.0, 0.0, -0.215))
    return _mesh(name, geom)


def _stem_mesh(name: str):
    geom = _tube_shell(inner_radius=0.0150, outer_radius=0.0220, length=0.042, segments=52)
    geom.merge(BoxGeometry((0.032, 0.060, 0.016)).translate(0.0, 0.044, -0.013))
    geom.merge(BoxGeometry((0.024, 0.026, 0.018)).translate(0.0, 0.022, -0.004))
    geom.merge(
        _tube_shell(inner_radius=0.0122, outer_radius=0.0205, length=0.044, segments=48)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.080, 0.015)
    )
    geom.merge(BoxGeometry((0.018, 0.030, 0.012)).translate(0.0, 0.065, -0.010))
    return _mesh(name, geom)


def _handlebar_mesh(name: str):
    return _mesh(
        name,
        tube_from_spline_points(
            [
                (-0.360, -0.040, 0.000),
                (-0.255, -0.028, 0.002),
                (-0.140, -0.014, 0.003),
                (0.000, 0.000, 0.000),
                (0.140, -0.014, 0.003),
                (0.255, -0.028, 0.002),
                (0.360, -0.040, 0.000),
            ],
            radius=0.011,
            samples_per_segment=20,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xc_suspension_fork_with_stem_and_bar")

    crown_black = model.material("crown_black", rgba=(0.14, 0.15, 0.16, 1.0))
    lower_black = model.material("lower_black", rgba=(0.10, 0.11, 0.12, 1.0))
    stanchion_silver = model.material("stanchion_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    stem_black = model.material("stem_black", rgba=(0.12, 0.12, 0.13, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))

    crown = model.part("crown")
    crown.visual(
        Box((0.188, 0.046, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=crown_black,
        name="crown_body",
    )
    crown.visual(
        Box((0.060, 0.012, 0.032)),
        origin=Origin(xyz=(-0.075, 0.028, -0.018)),
        material=crown_black,
        name="left_front_gusset",
    )
    crown.visual(
        Box((0.060, 0.012, 0.032)),
        origin=Origin(xyz=(-0.075, -0.028, -0.018)),
        material=crown_black,
        name="left_rear_gusset",
    )
    crown.visual(
        Box((0.060, 0.012, 0.032)),
        origin=Origin(xyz=(0.075, 0.028, -0.018)),
        material=crown_black,
        name="right_front_gusset",
    )
    crown.visual(
        Box((0.060, 0.012, 0.032)),
        origin=Origin(xyz=(0.075, -0.028, -0.018)),
        material=crown_black,
        name="right_rear_gusset",
    )
    crown.visual(
        Cylinder(radius=0.0143, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=stanchion_silver,
        name="steerer",
    )
    crown.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=crown_black,
        name="steerer_collar",
    )
    crown.visual(
        Cylinder(radius=0.0205, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.161)),
        material=crown_black,
        name="headset_spacer",
    )
    crown.visual(
        Cylinder(radius=0.0225, length=0.200),
        origin=Origin(xyz=(-0.075, 0.0, -0.125)),
        material=crown_black,
        name="left_upper_sleeve",
    )
    crown.visual(
        Cylinder(radius=0.0225, length=0.200),
        origin=Origin(xyz=(0.075, 0.0, -0.125)),
        material=crown_black,
        name="right_upper_sleeve",
    )
    crown.inertial = Inertial.from_geometry(
        Box((0.220, 0.060, 0.500)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    left_lower = model.part("left_lower_leg")
    left_lower.visual(
        Cylinder(radius=0.017, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=stanchion_silver,
        name="left_stanchion",
    )
    left_lower.visual(
        Cylinder(radius=0.024, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=lower_black,
        name="left_lower_slider",
    )
    left_lower.visual(
        Box((0.040, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.302)),
        material=lower_black,
        name="left_dropout",
    )
    left_lower.inertial = Inertial.from_geometry(
        Box((0.060, 0.045, 0.390)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    right_lower = model.part("right_lower_leg")
    right_lower.visual(
        Cylinder(radius=0.017, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=stanchion_silver,
        name="right_stanchion",
    )
    right_lower.visual(
        Cylinder(radius=0.024, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=lower_black,
        name="right_lower_slider",
    )
    right_lower.visual(
        Box((0.040, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.302)),
        material=lower_black,
        name="right_dropout",
    )
    right_lower.inertial = Inertial.from_geometry(
        Box((0.060, 0.045, 0.390)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    stem = model.part("stem")
    stem.visual(
        Box((0.008, 0.056, 0.030)),
        origin=Origin(xyz=(-0.0183, 0.010, 0.0)),
        material=stem_black,
        name="left_steerer_cheek",
    )
    stem.visual(
        Box((0.008, 0.056, 0.030)),
        origin=Origin(xyz=(0.0183, 0.010, 0.0)),
        material=stem_black,
        name="right_steerer_cheek",
    )
    stem.visual(
        Box((0.048, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=stem_black,
        name="steerer_back_bridge",
    )
    stem.visual(
        Box((0.034, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, 0.048, -0.010)),
        material=stem_black,
        name="stem_beam",
    )
    stem.visual(
        Box((0.042, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.080, -0.004)),
        material=stem_black,
        name="bar_saddle",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.050, 0.120, 0.060)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.045, 0.010)),
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        _handlebar_mesh("flat_handlebar"),
        material=stem_black,
        name="bar_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stem_black,
        name="bar_center_clamp",
    )
    handlebar.visual(
        Cylinder(radius=0.0165, length=0.110),
        origin=Origin(xyz=(-0.315, -0.034, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.0165, length=0.110),
        origin=Origin(xyz=(0.315, -0.034, 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.760, 0.120, 0.080)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.020, 0.004)),
    )

    model.articulation(
        "crown_to_left_lower",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=left_lower,
        origin=Origin(xyz=(-0.075, 0.0, -0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.60,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "crown_to_right_lower",
        ArticulationType.PRISMATIC,
        parent=crown,
        child=right_lower,
        origin=Origin(xyz=(0.075, 0.0, -0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.60,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "steerer_to_stem",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.5,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.080, 0.015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.40,
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

    crown = object_model.get_part("crown")
    left_lower = object_model.get_part("left_lower_leg")
    right_lower = object_model.get_part("right_lower_leg")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")

    left_slide = object_model.get_articulation("crown_to_left_lower")
    right_slide = object_model.get_articulation("crown_to_right_lower")
    steer_joint = object_model.get_articulation("steerer_to_stem")
    bar_roll = object_model.get_articulation("stem_to_handlebar")

    ctx.allow_overlap(
        crown,
        left_lower,
        elem_a="left_upper_sleeve",
        elem_b="left_stanchion",
        reason="The left stanchion is intentionally represented as a solid slider telescoping inside the upper fork sleeve proxy.",
    )
    ctx.allow_overlap(
        crown,
        right_lower,
        elem_a="right_upper_sleeve",
        elem_b="right_stanchion",
        reason="The right stanchion is intentionally represented as a solid slider telescoping inside the upper fork sleeve proxy.",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            crown,
            stem,
            axis="x",
            positive_elem="steerer",
            negative_elem="left_steerer_cheek",
            max_gap=0.0002,
            max_penetration=0.0002,
            name="left stem cheek sits against the steerer",
        )
        ctx.expect_gap(
            stem,
            crown,
            axis="x",
            positive_elem="right_steerer_cheek",
            negative_elem="steerer",
            max_gap=0.0002,
            max_penetration=0.0002,
            name="right stem cheek sits against the steerer",
        )
        ctx.expect_gap(
            handlebar,
            stem,
            axis="z",
            positive_elem="bar_center_clamp",
            negative_elem="bar_saddle",
            max_gap=0.0002,
            max_penetration=0.0002,
            name="handlebar clamp section seats on the stem saddle",
        )
        ctx.expect_within(
            left_lower,
            crown,
            axes="xy",
            inner_elem="left_stanchion",
            outer_elem="left_upper_sleeve",
            margin=0.001,
            name="left stanchion stays centered inside upper sleeve at rest",
        )
        ctx.expect_within(
            right_lower,
            crown,
            axes="xy",
            inner_elem="right_stanchion",
            outer_elem="right_upper_sleeve",
            margin=0.001,
            name="right stanchion stays centered inside upper sleeve at rest",
        )
        ctx.expect_overlap(
            crown,
            left_lower,
            axes="z",
            elem_a="left_upper_sleeve",
            elem_b="left_stanchion",
            min_overlap=0.190,
            name="left stanchion remains deeply inserted at rest",
        )
        ctx.expect_overlap(
            crown,
            right_lower,
            axes="z",
            elem_a="right_upper_sleeve",
            elem_b="right_stanchion",
            min_overlap=0.190,
            name="right stanchion remains deeply inserted at rest",
        )
        ctx.expect_gap(
            crown,
            left_lower,
            axis="z",
            positive_elem="left_upper_sleeve",
            negative_elem="left_lower_slider",
            min_gap=0.095,
            max_gap=0.105,
            name="left lower slider sits below the upper sleeve at rest",
        )
        ctx.expect_gap(
            crown,
            right_lower,
            axis="z",
            positive_elem="right_upper_sleeve",
            negative_elem="right_lower_slider",
            min_gap=0.095,
            max_gap=0.105,
            name="right lower slider sits below the upper sleeve at rest",
        )
        left_rest = ctx.part_world_position(left_lower)
        right_rest = ctx.part_world_position(right_lower)
        bar_rest = ctx.part_world_position(handlebar)

    with ctx.pose({left_slide: 0.085, right_slide: 0.085}):
        ctx.expect_within(
            left_lower,
            crown,
            axes="xy",
            inner_elem="left_stanchion",
            outer_elem="left_upper_sleeve",
            margin=0.001,
            name="left stanchion stays centered inside upper sleeve in compression",
        )
        ctx.expect_within(
            right_lower,
            crown,
            axes="xy",
            inner_elem="right_stanchion",
            outer_elem="right_upper_sleeve",
            margin=0.001,
            name="right stanchion stays centered inside upper sleeve in compression",
        )
        ctx.expect_overlap(
            crown,
            left_lower,
            axes="z",
            elem_a="left_upper_sleeve",
            elem_b="left_stanchion",
            min_overlap=0.190,
            name="left stanchion still remains inserted when compressed",
        )
        ctx.expect_overlap(
            crown,
            right_lower,
            axes="z",
            elem_a="right_upper_sleeve",
            elem_b="right_stanchion",
            min_overlap=0.190,
            name="right stanchion still remains inserted when compressed",
        )
        ctx.expect_gap(
            crown,
            left_lower,
            axis="z",
            positive_elem="left_upper_sleeve",
            negative_elem="left_lower_slider",
            min_gap=0.010,
            max_gap=0.020,
            name="left lower slider nearly reaches the upper sleeve under compression",
        )
        ctx.expect_gap(
            crown,
            right_lower,
            axis="z",
            positive_elem="right_upper_sleeve",
            negative_elem="right_lower_slider",
            min_gap=0.010,
            max_gap=0.020,
            name="right lower slider nearly reaches the upper sleeve under compression",
        )
        left_compressed = ctx.part_world_position(left_lower)
        right_compressed = ctx.part_world_position(right_lower)

    ctx.check(
        "left lower leg moves upward under compression",
        left_rest is not None
        and left_compressed is not None
        and left_compressed[2] > left_rest[2] + 0.05,
        details=f"rest={left_rest}, compressed={left_compressed}",
    )
    ctx.check(
        "right lower leg moves upward under compression",
        right_rest is not None
        and right_compressed is not None
        and right_compressed[2] > right_rest[2] + 0.05,
        details=f"rest={right_rest}, compressed={right_compressed}",
    )

    with ctx.pose({steer_joint: 0.45}):
        bar_turned = ctx.part_world_position(handlebar)
    ctx.check(
        "stem steers the handlebar around the steerer axis",
        bar_rest is not None
        and bar_turned is not None
        and abs(bar_turned[0] - bar_rest[0]) > 0.025
        and abs(bar_turned[2] - bar_rest[2]) < 0.005,
        details=f"rest={bar_rest}, turned={bar_turned}",
    )

    def _aabb_center(bounds):
        if bounds is None:
            return None
        (min_pt, max_pt) = bounds
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))

    left_grip_rest = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="left_grip"))
    with ctx.pose({bar_roll: 0.35}):
        left_grip_rolled = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="left_grip"))
    ctx.check(
        "handlebar rolls inside the stem clamp",
        left_grip_rest is not None
        and left_grip_rolled is not None
        and abs(left_grip_rolled[2] - left_grip_rest[2]) > 0.006,
        details=f"rest={left_grip_rest}, rolled={left_grip_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
