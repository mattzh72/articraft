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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annular_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 56,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            height=height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_post_artist_easel")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    warm_beech = model.material("warm_beech", rgba=(0.73, 0.60, 0.42, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base_radius = 0.235
    base_thickness = 0.030
    sleeve_height = 0.700
    sleeve_outer_radius = 0.034
    sleeve_inner_radius = 0.0285
    collar_height = 0.035
    collar_outer_radius = 0.041
    post_radius = 0.0245
    collar_segments = 56
    collar_inner_radius = 0.0265
    sleeve_top_z = base_thickness + sleeve_height

    post_length = 1.100
    post_center_z = 0.150
    post_top_z = post_center_z + (post_length * 0.5)
    hinge_y = 0.048
    hinge_z = post_top_z - 0.002

    base = model.part("base")
    base.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=powder_black,
        name="base_disc",
    )
    base.visual(
        _annular_tube_mesh(
            "easel_base_sleeve",
            outer_radius=sleeve_outer_radius,
            inner_radius=sleeve_inner_radius,
            height=sleeve_height,
        ),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + sleeve_height * 0.5)),
        material=powder_black,
        name="base_sleeve",
    )
    base.visual(
        _annular_tube_mesh(
            "easel_top_collar",
            outer_radius=collar_outer_radius,
            inner_radius=collar_inner_radius,
            height=collar_height,
            segments=collar_segments,
        ),
        origin=Origin(xyz=(0.0, 0.0, sleeve_top_z - collar_height * 0.5)),
        material=graphite,
        name="top_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(
            xyz=(collar_outer_radius + 0.013, 0.0, sleeve_top_z - collar_height * 0.45),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="height_lock_stem",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(
            xyz=(collar_outer_radius + 0.033, 0.0, sleeve_top_z - collar_height * 0.45),
        ),
        material=rubber,
        name="height_lock_knob",
    )
    guide_pad_z = sleeve_top_z - collar_height * 0.5
    base.visual(
        Box((0.004, 0.014, 0.024)),
        origin=Origin(xyz=(post_radius + 0.002, 0.0, guide_pad_z)),
        material=graphite,
        name="guide_pad_pos_x",
    )
    base.visual(
        Box((0.004, 0.014, 0.024)),
        origin=Origin(xyz=(-(post_radius + 0.002), 0.0, guide_pad_z)),
        material=graphite,
        name="guide_pad_neg_x",
    )
    base.visual(
        Box((0.014, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, post_radius + 0.002, guide_pad_z)),
        material=graphite,
        name="guide_pad_pos_y",
    )
    base.visual(
        Box((0.014, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -(post_radius + 0.002), guide_pad_z)),
        material=graphite,
        name="guide_pad_neg_y",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.470, 0.470, 0.765)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.3825)),
    )

    center_post = model.part("center_post")
    center_post.visual(
        Cylinder(radius=post_radius, length=post_length),
        origin=Origin(xyz=(0.0, 0.0, post_center_z)),
        material=brushed_steel,
        name="post_shaft",
    )
    center_post.visual(
        Cylinder(radius=0.029, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, post_top_z - 0.030)),
        material=graphite,
        name="tilt_head_collar",
    )
    center_post.visual(
        Box((0.090, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, 0.020, post_top_z - 0.020)),
        material=graphite,
        name="yoke_block",
    )
    center_post.visual(
        Box((0.012, 0.024, 0.050)),
        origin=Origin(xyz=(-0.037, hinge_y, hinge_z + 0.005)),
        material=graphite,
        name="yoke_left",
    )
    center_post.visual(
        Box((0.012, 0.024, 0.050)),
        origin=Origin(xyz=(0.037, hinge_y, hinge_z + 0.005)),
        material=graphite,
        name="yoke_right",
    )
    center_post.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 1.120)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.020, 0.160)),
    )

    model.articulation(
        "base_to_center_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=center_post,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.220,
        ),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.008, length=0.058),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    cradle.visual(
        Box((0.060, 0.030, 0.120)),
        origin=Origin(xyz=(0.0, 0.015, -0.045)),
        material=warm_beech,
        name="hinge_strap",
    )
    cradle.visual(
        Box((0.440, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.030, -0.060)),
        material=warm_beech,
        name="top_crossbar",
    )
    cradle.visual(
        Box((0.030, 0.022, 0.740)),
        origin=Origin(xyz=(-0.205, 0.030, -0.430)),
        material=warm_beech,
        name="left_rail",
    )
    cradle.visual(
        Box((0.030, 0.022, 0.740)),
        origin=Origin(xyz=(0.205, 0.030, -0.430)),
        material=warm_beech,
        name="right_rail",
    )
    cradle.visual(
        Box((0.060, 0.022, 0.740)),
        origin=Origin(xyz=(0.0, 0.025, -0.430)),
        material=warm_beech,
        name="center_spine",
    )
    cradle.visual(
        Box((0.440, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, 0.030, -0.800)),
        material=warm_beech,
        name="bottom_crossbar",
    )
    cradle.visual(
        Box((0.400, 0.085, 0.018)),
        origin=Origin(xyz=(0.0, 0.068, -0.785)),
        material=warm_beech,
        name="canvas_shelf",
    )
    cradle.visual(
        Box((0.400, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.110, -0.772)),
        material=warm_beech,
        name="canvas_lip",
    )
    cradle.visual(
        Box((0.220, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.040, -0.079)),
        material=warm_beech,
        name="top_retainer",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.470, 0.140, 0.830)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.055, -0.400)),
    )

    model.articulation(
        "post_to_cradle",
        ArticulationType.REVOLUTE,
        parent=center_post,
        child=cradle,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.18,
            upper=0.70,
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
    center_post = object_model.get_part("center_post")
    cradle = object_model.get_part("cradle")
    post_slide = object_model.get_articulation("base_to_center_post")
    cradle_tilt = object_model.get_articulation("post_to_cradle")

    ctx.expect_within(
        center_post,
        base,
        axes="xy",
        inner_elem="post_shaft",
        outer_elem="base_sleeve",
        margin=0.001,
        name="center post stays concentric in the base sleeve at rest",
    )
    ctx.expect_overlap(
        center_post,
        base,
        axes="z",
        elem_a="post_shaft",
        elem_b="base_sleeve",
        min_overlap=0.390,
        name="rest pose keeps substantial post insertion inside the sleeve",
    )

    rest_post_pos = ctx.part_world_position(center_post)
    with ctx.pose({post_slide: 0.220}):
        ctx.expect_within(
            center_post,
            base,
            axes="xy",
            inner_elem="post_shaft",
            outer_elem="top_collar",
            margin=0.001,
            name="extended post remains guided by the upper collar",
        )
        ctx.expect_overlap(
            center_post,
            base,
            axes="z",
            elem_a="post_shaft",
            elem_b="base_sleeve",
            min_overlap=0.175,
            name="extended post still retains insertion inside the sleeve",
        )
        extended_post_pos = ctx.part_world_position(center_post)

    ctx.check(
        "center post extends upward from the base",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.20,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    rest_shelf_aabb = ctx.part_element_world_aabb(cradle, elem="canvas_shelf")
    with ctx.pose({cradle_tilt: 0.50}):
        tilted_shelf_aabb = ctx.part_element_world_aabb(cradle, elem="canvas_shelf")

    rest_shelf_center = (
        None
        if rest_shelf_aabb is None
        else tuple(
            (rest_shelf_aabb[0][axis] + rest_shelf_aabb[1][axis]) * 0.5
            for axis in range(3)
        )
    )
    tilted_shelf_center = (
        None
        if tilted_shelf_aabb is None
        else tuple(
            (tilted_shelf_aabb[0][axis] + tilted_shelf_aabb[1][axis]) * 0.5
            for axis in range(3)
        )
    )
    ctx.check(
        "cradle tilt swings the shelf forward",
        rest_shelf_center is not None
        and tilted_shelf_center is not None
        and tilted_shelf_center[1] > rest_shelf_center[1] + 0.22
        and tilted_shelf_center[2] > rest_shelf_center[2] + 0.10,
        details=f"rest={rest_shelf_center}, tilted={tilted_shelf_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
