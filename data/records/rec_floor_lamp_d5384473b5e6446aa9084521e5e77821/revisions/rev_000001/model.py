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


def _outer_sleeve_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.035),
            (0.026, 0.855),
            (0.031, 0.865),
            (0.031, 0.915),
        ],
        [
            (0.020, 0.035),
            (0.020, 0.855),
            (0.024, 0.865),
            (0.024, 0.915),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _shade_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.022, 0.020),
            (0.033, 0.050),
            (0.058, 0.115),
            (0.102, 0.210),
        ],
        [
            (0.018, 0.024),
            (0.027, 0.052),
            (0.048, 0.115),
            (0.091, 0.206),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_floor_lamp")

    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    linen = model.material("linen", rgba=(0.86, 0.82, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.110, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=charcoal,
        name="top_plate",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=charcoal,
        name="lower_collar",
    )
    base.visual(
        _mesh("floor_lamp_outer_sleeve", _outer_sleeve_mesh()),
        material=satin_black,
        name="outer_tube",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.037, 0.0, 0.760), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="adjuster_stem",
    )
    base.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.055, 0.0, 0.760)),
        material=rubber,
        name="adjuster_knob",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.915),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, 0.4575)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.018, length=1.020),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=steel,
        name="inner_tube",
    )
    inner_post.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=satin_black,
        name="shade_mount_body",
    )
    inner_post.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=charcoal,
        name="stop_collar",
    )
    inner_post.visual(
        Box((0.046, 0.018, 0.018)),
        origin=Origin(xyz=(0.023, 0.0, 0.635)),
        material=satin_black,
        name="hinge_arm",
    )
    inner_post.visual(
        Box((0.010, 0.009, 0.028)),
        origin=Origin(xyz=(0.046, 0.010, 0.635)),
        material=satin_black,
        name="clevis_left",
    )
    inner_post.visual(
        Box((0.010, 0.009, 0.028)),
        origin=Origin(xyz=(0.046, -0.010, 0.635)),
        material=satin_black,
        name="clevis_right",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 1.120)),
        mass=1.7,
        origin=Origin(xyz=(0.020, 0.0, 0.280)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    shade.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_boss",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="socket_housing",
    )
    shade.visual(
        _mesh("floor_lamp_shade_shell", _shade_shell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linen,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.230, 0.220, 0.220)),
        mass=0.9,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.260,
        ),
    )
    model.articulation(
        "inner_post_to_shade",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=shade,
        origin=Origin(xyz=(0.046, 0.0, 0.635)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.75,
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
    inner_post = object_model.get_part("inner_post")
    shade = object_model.get_part("shade")
    lift = object_model.get_articulation("base_to_inner_post")
    tilt = object_model.get_articulation("inner_post_to_shade")

    lift_limits = lift.motion_limits
    tilt_limits = tilt.motion_limits
    lift_upper = 0.260 if lift_limits is None or lift_limits.upper is None else lift_limits.upper
    tilt_lower = -0.55 if tilt_limits is None or tilt_limits.lower is None else tilt_limits.lower
    tilt_upper = 0.75 if tilt_limits is None or tilt_limits.upper is None else tilt_limits.upper

    ctx.expect_within(
        inner_post,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube",
        margin=0.001,
        name="inner post stays centered inside the base sleeve",
    )
    ctx.expect_overlap(
        inner_post,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube",
        min_overlap=0.36,
        name="collapsed post remains deeply inserted in the sleeve",
    )

    rest_post_pos = ctx.part_world_position(inner_post)
    with ctx.pose({lift: lift_upper}):
        ctx.expect_within(
            inner_post,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube",
            margin=0.001,
            name="extended post remains centered in the sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube",
            min_overlap=0.10,
            name="extended post keeps retained insertion in the sleeve",
        )
        extended_post_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "post extends upward from the base",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.20,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    with ctx.pose({tilt: tilt_lower}):
        lowered_shade = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({tilt: tilt_upper}):
        raised_shade = ctx.part_element_world_aabb(shade, elem="shade_shell")

    ctx.check(
        "shade hinge lifts and lowers the cone",
        lowered_shade is not None
        and raised_shade is not None
        and raised_shade[1][2] > lowered_shade[1][2] + 0.08,
        details=f"lowered={lowered_shade}, raised={raised_shade}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
