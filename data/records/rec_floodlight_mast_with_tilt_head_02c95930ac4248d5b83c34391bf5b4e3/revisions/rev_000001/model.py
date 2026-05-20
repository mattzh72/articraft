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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_led_floodlight_stand")

    stand_black = model.material("stand_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    worklight_yellow = model.material("worklight_yellow", rgba=(0.86, 0.73, 0.14, 1.0))
    diffuser = model.material("diffuser", rgba=(0.95, 0.96, 0.86, 0.92))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")

    hub_radius = 0.060
    base_ring_outer_radius = 0.21
    base_ring_inner_radius = 0.165
    base_ring_height = 0.024
    base_ring_segments = 72
    base_ring_outer_profile = [
        (
            base_ring_outer_radius * math.cos(index * 2.0 * math.pi / base_ring_segments),
            base_ring_outer_radius * math.sin(index * 2.0 * math.pi / base_ring_segments),
        )
        for index in range(base_ring_segments)
    ]
    base_ring_inner_profile = [
        (
            base_ring_inner_radius * math.cos(index * 2.0 * math.pi / base_ring_segments),
            base_ring_inner_radius * math.sin(index * 2.0 * math.pi / base_ring_segments),
        )
        for index in reversed(range(base_ring_segments))
    ]
    base_ring_shell = ExtrudeWithHolesGeometry(
        base_ring_outer_profile,
        [base_ring_inner_profile],
        base_ring_height,
        cap=True,
        center=True,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(base_ring_shell, "base_floor_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=stand_black,
        name="base_ring",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=graphite,
        name="base_top_plate",
    )
    base.visual(
        Cylinder(radius=hub_radius, length=0.21),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=graphite,
        name="center_hub",
    )
    spoke_length = base_ring_inner_radius - hub_radius + 0.012
    spoke_center_radius = 0.5 * (base_ring_inner_radius + hub_radius)
    for spoke_index, spoke_angle in enumerate(
        (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    ):
        base.visual(
            Box((spoke_length, 0.030, base_ring_height)),
            origin=Origin(
                xyz=(
                    spoke_center_radius * math.cos(spoke_angle),
                    spoke_center_radius * math.sin(spoke_angle),
                    0.012,
                ),
                rpy=(0.0, 0.0, spoke_angle),
            ),
            material=graphite,
            name=f"spoke_{spoke_index}",
        )

    inner_post_radius = 0.024
    sleeve_outer_radius = 0.034
    sleeve_inner_radius = 0.0248
    sleeve_height = 0.50
    sleeve_segments = 72
    outer_profile = [
        (
            sleeve_outer_radius * math.cos(index * 2.0 * math.pi / sleeve_segments),
            sleeve_outer_radius * math.sin(index * 2.0 * math.pi / sleeve_segments),
        )
        for index in range(sleeve_segments)
    ]
    inner_profile = [
        (
            sleeve_inner_radius * math.cos(index * 2.0 * math.pi / sleeve_segments),
            sleeve_inner_radius * math.sin(index * 2.0 * math.pi / sleeve_segments),
        )
        for index in reversed(range(sleeve_segments))
    ]
    sleeve_shell = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        sleeve_height,
        cap=True,
        center=True,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(sleeve_shell, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=stand_black,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.041, 0.0, 0.53), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_stem",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.060, 0.0, 0.53), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="clamp_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 0.70)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    upper_post = model.part("upper_post")
    upper_post.visual(
        Cylinder(radius=inner_post_radius, length=0.84),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=graphite,
        name="inner_post",
    )
    upper_post.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_black,
        name="stop_collar",
    )
    upper_post.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=stand_black,
        name="top_head",
    )
    upper_post.visual(
        Box((0.10, 0.024, 0.070)),
        origin=Origin(xyz=(0.0, -0.027, 0.44)),
        material=stand_black,
        name="rear_cradle",
    )
    upper_post.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.92)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    model.articulation(
        "base_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_post,
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=0.0,
            upper=0.32,
        ),
    )

    lamp_bar = model.part("lamp_bar")
    lamp_bar.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="pivot_knuckle",
    )
    lamp_bar.visual(
        Box((0.10, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=graphite,
        name="center_spine",
    )
    lamp_bar.visual(
        Box((0.50, 0.045, 0.034)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=stand_black,
        name="crossbar",
    )

    for side_name, x_center in (("left", -0.15), ("right", 0.15)):
        lamp_bar.visual(
            Box((0.060, 0.025, 0.070)),
            origin=Origin(xyz=(x_center, 0.083, 0.0)),
            material=graphite,
            name=f"{side_name}_mount",
        )
        lamp_bar.visual(
            Box((0.150, 0.050, 0.120)),
            origin=Origin(xyz=(x_center, 0.120, 0.0)),
            material=worklight_yellow,
            name=f"{side_name}_housing",
        )
        lamp_bar.visual(
            Box((0.162, 0.012, 0.132)),
            origin=Origin(xyz=(x_center, 0.151, 0.0)),
            material=stand_black,
            name=f"{side_name}_bezel",
        )
        lamp_bar.visual(
            Box((0.146, 0.004, 0.116)),
            origin=Origin(xyz=(x_center, 0.159, 0.0)),
            material=diffuser,
            name=f"{side_name}_diffuser",
        )

    lamp_bar.inertial = Inertial.from_geometry(
        Box((0.54, 0.18, 0.16)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
    )

    model.articulation(
        "upper_post_to_lamp_bar",
        ArticulationType.REVOLUTE,
        parent=upper_post,
        child=lamp_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(60.0),
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

    base = object_model.get_part("base")
    upper_post = object_model.get_part("upper_post")
    lamp_bar = object_model.get_part("lamp_bar")
    lift_joint = object_model.get_articulation("base_to_upper_post")
    tilt_joint = object_model.get_articulation("upper_post_to_lamp_bar")

    lift_upper = (
        lift_joint.motion_limits.upper
        if lift_joint.motion_limits is not None and lift_joint.motion_limits.upper is not None
        else 0.0
    )
    tilt_upper = (
        tilt_joint.motion_limits.upper
        if tilt_joint.motion_limits is not None and tilt_joint.motion_limits.upper is not None
        else 0.0
    )

    with ctx.pose({lift_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_sleeve",
            margin=0.006,
            name="inner post stays centered in the outer sleeve at rest",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.41,
            name="rest pose keeps substantial post insertion",
        )
        ctx.expect_contact(
            upper_post,
            base,
            elem_a="stop_collar",
            elem_b="outer_sleeve",
            name="mast stop collar seats on the sleeve rim at rest",
        )
        ctx.expect_contact(
            upper_post,
            lamp_bar,
            elem_a="rear_cradle",
            elem_b="pivot_knuckle",
            name="lamp bar hinge knuckle seats on the mount cradle",
        )

    rest_post_position = ctx.part_world_position(upper_post)
    with ctx.pose({lift_joint: lift_upper, tilt_joint: 0.0}):
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_sleeve",
            margin=0.006,
            name="extended post stays centered in the outer sleeve",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_sleeve",
            min_overlap=0.09,
            name="extended post retains insertion in the outer sleeve",
        )
        extended_post_position = ctx.part_world_position(upper_post)

    ctx.check(
        "prismatic mast extends upward",
        rest_post_position is not None
        and extended_post_position is not None
        and extended_post_position[2] > rest_post_position[2] + 0.25,
        details=f"rest={rest_post_position}, extended={extended_post_position}",
    )

    with ctx.pose({lift_joint: 0.0, tilt_joint: 0.0}):
        rest_left_diffuser = ctx.part_element_world_aabb(lamp_bar, elem="left_diffuser")
    with ctx.pose({lift_joint: 0.0, tilt_joint: tilt_upper}):
        tilted_left_diffuser = ctx.part_element_world_aabb(lamp_bar, elem="left_diffuser")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    rest_center = _aabb_center(rest_left_diffuser)
    tilted_center = _aabb_center(tilted_left_diffuser)
    ctx.check(
        "lamp bar tilts upward around the mount bracket",
        rest_center is not None
        and tilted_center is not None
        and tilted_center[2] > rest_center[2] + 0.06
        and tilted_center[1] < rest_center[1] - 0.04,
        details=f"rest={rest_center}, tilted={tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
