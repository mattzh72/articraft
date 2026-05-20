from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    superellipse_profile,
)


def _annulus_mesh(
    name: str,
    *,
    outer_diameter: float,
    inner_diameter: float,
    thickness: float,
    rotate_x_angle: float = 0.0,
):
    outer = superellipse_profile(
        outer_diameter,
        outer_diameter,
        exponent=2.0,
        segments=72,
    )
    inner = superellipse_profile(
        inner_diameter,
        inner_diameter,
        exponent=2.0,
        segments=72,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        height=thickness,
        center=True,
    )
    if rotate_x_angle:
        geom.rotate_x(rotate_x_angle)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_ring_light")

    matte_white = model.material("matte_white", rgba=(0.94, 0.94, 0.93, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.98, 0.97, 0.94, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.71, 0.73, 0.76, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.76, 0.81, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.115, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_white,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=satin_silver,
        name="base_neck",
    )
    base.visual(
        _annulus_mesh(
            "outer_sleeve_mesh",
            outer_diameter=0.034,
            inner_diameter=0.028,
            thickness=0.160,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=satin_silver,
        name="outer_sleeve",
    )
    base.visual(
        _annulus_mesh(
            "clamp_collar_mesh",
            outer_diameter=0.048,
            inner_diameter=0.028,
            thickness=0.028,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        material=charcoal,
        name="clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.024, 0.0, 0.198), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="adjustment_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.22)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0125, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=satin_silver,
        name="inner_post",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=charcoal,
        name="post_stop_collar",
    )
    mast.visual(
        Box((0.030, 0.020, 0.114)),
        origin=Origin(xyz=(0.0, 0.0, 0.327)),
        material=charcoal,
        name="top_bracket_stem",
    )
    mast.visual(
        Box((0.110, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.382)),
        material=charcoal,
        name="top_bracket_bridge",
    )
    mast.visual(
        Box((0.018, 0.022, 0.070)),
        origin=Origin(xyz=(-0.040, 0.0, 0.347)),
        material=charcoal,
        name="left_top_bracket_tab",
    )
    mast.visual(
        Box((0.018, 0.022, 0.070)),
        origin=Origin(xyz=(0.040, 0.0, 0.347)),
        material=charcoal,
        name="right_top_bracket_tab",
    )
    mast.visual(
        _annulus_mesh(
            "ring_housing_mesh",
            outer_diameter=0.272,
            inner_diameter=0.196,
            thickness=0.024,
            rotate_x_angle=pi / 2.0,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=matte_white,
        name="ring_housing",
    )
    mast.visual(
        _annulus_mesh(
            "ring_diffuser_mesh",
            outer_diameter=0.252,
            inner_diameter=0.206,
            thickness=0.004,
            rotate_x_angle=pi / 2.0,
        ),
        origin=Origin(xyz=(0.0, 0.014, 0.240)),
        material=diffuser_white,
        name="ring_diffuser",
    )
    mast.visual(
        Box((0.060, 0.052, 0.014)),
        origin=Origin(xyz=(0.0, 0.018, 0.085)),
        material=charcoal,
        name="mirror_hinge_support",
    )
    mast.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(-0.021, 0.035, 0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="left_hinge_barrel",
    )
    mast.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.021, 0.035, 0.072), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="right_hinge_barrel",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.30, 0.05, 0.58)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    mirror = model.part("mirror")
    mirror.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="mirror_hinge_barrel",
    )
    mirror.visual(
        Box((0.008, 0.012, 0.020)),
        origin=Origin(xyz=(-0.006, 0.0, -0.012)),
        material=charcoal,
        name="left_hinge_strap",
    )
    mirror.visual(
        Box((0.008, 0.012, 0.020)),
        origin=Origin(xyz=(0.006, 0.0, -0.012)),
        material=charcoal,
        name="right_hinge_strap",
    )
    mirror.visual(
        Box((0.154, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=charcoal,
        name="top_frame",
    )
    mirror.visual(
        Box((0.154, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=charcoal,
        name="bottom_frame",
    )
    mirror.visual(
        Box((0.012, 0.010, 0.112)),
        origin=Origin(xyz=(-0.071, 0.0, -0.069)),
        material=charcoal,
        name="left_frame",
    )
    mirror.visual(
        Box((0.012, 0.010, 0.112)),
        origin=Origin(xyz=(0.071, 0.0, -0.069)),
        material=charcoal,
        name="right_frame",
    )
    mirror.visual(
        Box((0.148, 0.004, 0.120)),
        origin=Origin(xyz=(0.0, 0.001, -0.069)),
        material=mirror_glass,
        name="mirror_panel",
    )
    mirror.inertial = Inertial.from_geometry(
        Box((0.16, 0.02, 0.14)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=0.10,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "mast_to_mirror",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=mirror,
        origin=Origin(xyz=(0.0, 0.035, 0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.85,
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
    mast = object_model.get_part("mast")
    mirror = object_model.get_part("mirror")
    slide_joint = object_model.get_articulation("base_to_mast")
    mirror_joint = object_model.get_articulation("mast_to_mirror")

    outer_sleeve = base.get_visual("outer_sleeve")
    inner_post = mast.get_visual("inner_post")
    ring_housing = mast.get_visual("ring_housing")
    mirror_panel = mirror.get_visual("mirror_panel")

    slide_upper = slide_joint.motion_limits.upper if slide_joint.motion_limits is not None else 0.12

    with ctx.pose({slide_joint: 0.0, mirror_joint: 0.0}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem=inner_post,
            outer_elem=outer_sleeve,
            margin=0.002,
            name="inner post stays centered in outer sleeve at rest",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a=inner_post,
            elem_b=outer_sleeve,
            min_overlap=0.150,
            name="inner post remains well inserted at rest",
        )
        ctx.expect_gap(
            mast,
            mirror,
            axis="z",
            positive_elem=ring_housing,
            negative_elem=mirror_panel,
            min_gap=0.030,
            name="mirror panel hangs below the ring light",
        )
        rest_mast_pos = ctx.part_world_position(mast)
        rest_panel_aabb = ctx.part_element_world_aabb(mirror, elem="mirror_panel")

    with ctx.pose({slide_joint: slide_upper, mirror_joint: 0.0}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem=inner_post,
            outer_elem=outer_sleeve,
            margin=0.002,
            name="inner post stays centered in outer sleeve when raised",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a=inner_post,
            elem_b=outer_sleeve,
            min_overlap=0.035,
            name="raised post keeps retained insertion in the sleeve",
        )
        raised_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.10,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    with ctx.pose({slide_joint: 0.0, mirror_joint: 0.60}):
        tilted_panel_aabb = ctx.part_element_world_aabb(mirror, elem="mirror_panel")
        ctx.expect_gap(
            mast,
            mirror,
            axis="z",
            positive_elem=ring_housing,
            negative_elem=mirror_panel,
            min_gap=0.008,
            name="tilted mirror still clears the ring housing",
        )

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return 0.5 * (min_corner[1] + max_corner[1])

    rest_center_y = _aabb_center_y(rest_panel_aabb)
    tilted_center_y = _aabb_center_y(tilted_panel_aabb)
    ctx.check(
        "mirror panel tilts forward on its hinge",
        rest_center_y is not None
        and tilted_center_y is not None
        and tilted_center_y > rest_center_y + 0.030,
        details=f"rest_center_y={rest_center_y}, tilted_center_y={tilted_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
