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
    Sphere,
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
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalance_drafting_floor_lamp")

    cast_iron = model.material("cast_iron", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.20, 0.20, 0.21, 1.0))
    brushed_brass = model.material("brushed_brass", rgba=(0.70, 0.58, 0.32, 1.0))
    warm_white = model.material("warm_white", rgba=(0.90, 0.89, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.185, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cast_iron,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.132, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=cast_iron,
        name="base_riser",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=satin_black,
        name="base_collar",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.185, length=0.078),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=satin_black,
        name="post_lower_sleeve",
    )
    post.visual(
        Cylinder(radius=0.016, length=1.360),
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        material=satin_black,
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.450)),
        material=steel,
        name="post_upper_collar",
    )
    post.visual(
        Box((0.100, 0.010, 0.160)),
        origin=Origin(xyz=(0.048, 0.026, 1.540)),
        material=steel,
        name="left_yoke_plate",
    )
    post.visual(
        Box((0.100, 0.010, 0.160)),
        origin=Origin(xyz=(0.048, -0.026, 1.540)),
        material=steel,
        name="right_yoke_plate",
    )
    post.visual(
        Box((0.070, 0.042, 0.028)),
        origin=Origin(xyz=(0.048, 0.000, 1.614)),
        material=steel,
        name="yoke_bridge",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 1.620)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.027, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    boom.visual(
        Cylinder(radius=0.012, length=0.880),
        origin=Origin(xyz=(0.440, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_boom_tube",
    )
    boom.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(-0.180, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_boom_tube",
    )
    boom.visual(
        Box((0.080, 0.038, 0.024)),
        origin=Origin(xyz=(0.800, 0.000, -0.015)),
        material=satin_black,
        name="shade_drop_link",
    )
    boom.visual(
        Box((0.050, 0.008, 0.065)),
        origin=Origin(xyz=(0.855, 0.019, -0.025)),
        material=steel,
        name="shade_fork_left",
    )
    boom.visual(
        Box((0.050, 0.008, 0.065)),
        origin=Origin(xyz=(0.855, -0.019, -0.025)),
        material=steel,
        name="shade_fork_right",
    )
    boom.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(-0.110, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_brass,
        name="counterweight_collar",
    )
    boom.visual(
        Cylinder(radius=0.085, length=0.140),
        origin=Origin(xyz=(-0.280, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_brass,
        name="counterweight_mass",
    )
    boom.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(-0.365, 0.0, 0.000)),
        material=steel,
        name="counterweight_endcap",
    )
    boom.inertial = Inertial.from_geometry(
        Box((1.260, 0.190, 0.180)),
        mass=3.2,
        origin=Origin(xyz=(0.245, 0.0, -0.010)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shade_knuckle",
    )
    shade.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(0.000, 0.000, -0.095)),
        material=steel,
        name="shade_cap",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(xyz=(0.000, 0.000, -0.040)),
        material=satin_black,
        name="shade_socket",
    )
    shade.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, -0.120)),
        material=steel,
        name="shade_neck",
    )
    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.122, 0.000),
            (0.116, 0.028),
            (0.095, 0.100),
            (0.070, 0.158),
            (0.034, 0.190),
        ],
        [
            (0.112, 0.008),
            (0.107, 0.030),
            (0.087, 0.100),
            (0.062, 0.154),
            (0.022, 0.182),
        ],
        segments=56,
    )
    shade.visual(
        _mesh("shade_shell", shade_shell),
        origin=Origin(xyz=(0.000, 0.000, -0.300)),
        material=warm_white,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.220),
        mass=1.1,
        origin=Origin(xyz=(0.000, 0.000, -0.205)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
    )
    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=post,
        child=boom,
        origin=Origin(xyz=(0.048, 0.0, 1.540)),
        # The boom extends mostly along local +X from the pivot, so -Y makes
        # positive motion raise the lamp head upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-0.38,
            upper=0.80,
        ),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(0.855, 0.0, -0.025)),
        # The shade points downward at q=0; -Y makes positive motion tip it
        # forward/outward from the boom end.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.65,
            upper=0.95,
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
    post = object_model.get_part("post")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")
    boom_joint = object_model.get_articulation("post_to_boom")
    shade_joint = object_model.get_articulation("boom_to_shade")

    ctx.expect_contact(
        post,
        base,
        elem_a="post_lower_sleeve",
        elem_b="base_collar",
        contact_tol=0.001,
        name="post sleeve seats on the base collar",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="xy",
        elem_a="post_lower_sleeve",
        elem_b="base_collar",
        min_overlap=0.050,
        name="post remains centered on the round base",
    )
    ctx.expect_contact(
        boom,
        post,
        elem_a="pivot_hub",
        elem_b="left_yoke_plate",
        contact_tol=0.0001,
        name="boom pivot hub bears against the left yoke plate",
    )
    ctx.expect_contact(
        boom,
        post,
        elem_a="pivot_hub",
        elem_b="right_yoke_plate",
        contact_tol=0.0001,
        name="boom pivot hub bears against the right yoke plate",
    )

    with ctx.pose({boom_joint: 0.0, shade_joint: 0.0}):
        ctx.expect_origin_gap(
            shade,
            post,
            axis="x",
            min_gap=0.75,
            name="lamp head projects forward from the post at rest",
        )
        ctx.expect_gap(
            boom,
            base,
            axis="z",
            min_gap=1.20,
            name="horizontal boom clears the base at rest",
        )
        rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    boom_upper = boom_joint.motion_limits.upper if boom_joint.motion_limits is not None else 0.80
    boom_lower = boom_joint.motion_limits.lower if boom_joint.motion_limits is not None else -0.45
    with ctx.pose({boom_joint: boom_upper, shade_joint: 0.0}):
        raised_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    rest_center = _aabb_center(rest_shell_aabb)
    raised_center = _aabb_center(raised_shell_aabb)
    ctx.check(
        "boom articulation raises the lamp head",
        rest_center is not None
        and raised_center is not None
        and raised_center[2] > rest_center[2] + 0.40,
        details=f"rest_center={rest_center}, raised_center={raised_center}",
    )

    with ctx.pose({boom_joint: boom_lower, shade_joint: 0.0}):
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.85,
            name="lowered boom still keeps the shade well above the base",
        )

    shade_upper = shade_joint.motion_limits.upper if shade_joint.motion_limits is not None else 0.95
    with ctx.pose({boom_joint: 0.15, shade_joint: 0.0}):
        neutral_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({boom_joint: 0.15, shade_joint: shade_upper}):
        tipped_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

    neutral_center = _aabb_center(neutral_shade_aabb)
    tipped_center = _aabb_center(tipped_shade_aabb)
    ctx.check(
        "shade joint tips the shade forward and upward",
        neutral_center is not None
        and tipped_center is not None
        and tipped_center[0] > neutral_center[0] + 0.07
        and tipped_center[2] > neutral_center[2] + 0.05,
        details=f"neutral_center={neutral_center}, tipped_center={tipped_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
