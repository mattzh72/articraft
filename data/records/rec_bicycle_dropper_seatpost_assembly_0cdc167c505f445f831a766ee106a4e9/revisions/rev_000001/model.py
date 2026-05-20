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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def _build_outer_post_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0158, 0.000),
            (0.0158, 0.228),
            (0.0166, 0.241),
            (0.0188, 0.257),
            (0.0188, 0.286),
            (0.0172, 0.292),
        ],
        [
            (0.0133, 0.003),
            (0.0133, 0.248),
            (0.01285, 0.272),
            (0.01285, 0.286),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_inner_tube_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.01285, -0.165),
            (0.01285, 0.088),
            (0.01360, 0.096),
        ],
        [
            (0.01095, -0.162),
            (0.01095, 0.091),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_post_head_mesh():
    head = BoxGeometry((0.016, 0.018, 0.022)).translate(0.0, 0.0, 0.103)
    head.merge(BoxGeometry((0.040, 0.014, 0.010)).translate(0.0, 0.0, 0.118))
    head.merge(BoxGeometry((0.0045, 0.016, 0.016)).translate(0.0175, 0.0, 0.125))
    head.merge(BoxGeometry((0.0045, 0.016, 0.016)).translate(-0.0175, 0.0, 0.125))
    return head


def _build_clamp_head_mesh():
    clamp = BoxGeometry((0.012, 0.012, 0.018)).translate(0.0, 0.0, 0.012)
    clamp.merge(BoxGeometry((0.048, 0.022, 0.010)).translate(0.0, 0.0, 0.024))
    clamp.merge(BoxGeometry((0.012, 0.024, 0.010)).translate(0.018, 0.0, 0.029))
    clamp.merge(BoxGeometry((0.012, 0.024, 0.010)).translate(-0.018, 0.0, 0.029))
    clamp.merge(BoxGeometry((0.050, 0.004, 0.016)).translate(0.0, 0.010, 0.036))
    clamp.merge(BoxGeometry((0.050, 0.004, 0.016)).translate(0.0, -0.010, 0.036))
    clamp.merge(BoxGeometry((0.050, 0.028, 0.005)).translate(0.0, 0.0, 0.044))
    return clamp


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trail_dropper_seatpost")

    outer_black = model.material("outer_black", rgba=(0.11, 0.11, 0.12, 1.0))
    stanchion_black = model.material("stanchion_black", rgba=(0.16, 0.16, 0.17, 1.0))
    hardware = model.material("hardware", rgba=(0.56, 0.58, 0.61, 1.0))

    outer_post = model.part("outer_post")
    outer_post.visual(
        _mesh("outer_post_shell", _build_outer_post_mesh()),
        material=outer_black,
        name="outer_tube_shell",
    )
    outer_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.292),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        _mesh("inner_post_tube", _build_inner_tube_mesh()),
        material=stanchion_black,
        name="inner_tube",
    )
    inner_post.visual(
        _mesh("inner_post_head", _build_post_head_mesh()),
        material=hardware,
        name="post_head",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.300)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.0054, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="pivot_barrel",
    )
    saddle_clamp.visual(
        _mesh("saddle_clamp_head", _build_clamp_head_mesh()),
        material=hardware,
        name="clamp_head",
    )
    saddle_clamp.visual(
        Box((0.012, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.014, 0.0460)),
        material=hardware,
        name="front_tab",
    )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.055, 0.030, 0.055)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_post,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.15,
            lower=0.0,
            upper=0.060,
        ),
    )
    model.articulation(
        "inner_to_clamp",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.35,
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

    outer_post = object_model.get_part("outer_post")
    inner_post = object_model.get_part("inner_post")
    saddle_clamp = object_model.get_part("saddle_clamp")
    drop = object_model.get_articulation("outer_to_inner")
    tilt = object_model.get_articulation("inner_to_clamp")

    ctx.allow_overlap(
        outer_post,
        inner_post,
        elem_a="outer_tube_shell",
        elem_b="inner_tube",
        reason="The lower body is authored as a thin-walled sleeve visual whose compiled overlap proxy does not preserve the sliding cavity.",
    )

    with ctx.pose({drop: 0.0, tilt: 0.0}):
        ctx.expect_within(
            inner_post,
            outer_post,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube_shell",
            margin=0.003,
            name="inner tube stays centered in the outer tube at rest",
        )
        ctx.expect_overlap(
            inner_post,
            outer_post,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube_shell",
            min_overlap=0.16,
            name="inner tube remains inserted at rest",
        )
        ctx.expect_origin_distance(
            saddle_clamp,
            inner_post,
            axes="xy",
            max_dist=0.001,
            name="saddle clamp stays centered over the post",
        )
        ctx.expect_origin_gap(
            saddle_clamp,
            inner_post,
            axis="z",
            min_gap=0.123,
            max_gap=0.125,
            name="saddle clamp hinge sits on top of the post head",
        )
        rest_inner_pos = ctx.part_world_position(inner_post)
        neutral_front = _aabb_center(ctx.part_element_world_aabb(saddle_clamp, elem="front_tab"))

    with ctx.pose({drop: 0.060, tilt: 0.0}):
        ctx.expect_within(
            inner_post,
            outer_post,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube_shell",
            margin=0.003,
            name="inner tube stays centered at full extension",
        )
        ctx.expect_overlap(
            inner_post,
            outer_post,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube_shell",
            min_overlap=0.10,
            name="inner tube retains insertion at full extension",
        )
        extended_inner_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "dropper post extends upward",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[2] > rest_inner_pos[2] + 0.055,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )

    with ctx.pose({drop: 0.030, tilt: 0.25}):
        nose_up_front = _aabb_center(ctx.part_element_world_aabb(saddle_clamp, elem="front_tab"))

    ctx.check(
        "positive clamp tilt raises the front of the rail clamp",
        neutral_front is not None
        and nose_up_front is not None
        and nose_up_front[2] > neutral_front[2] + 0.003,
        details=f"neutral={neutral_front}, nose_up={nose_up_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
