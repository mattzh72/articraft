from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    marble = model.material("marble", rgba=(0.90, 0.91, 0.89, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    shade_charcoal = model.material("shade_charcoal", rgba=(0.16, 0.16, 0.18, 1.0))
    shade_inner = model.material("shade_inner", rgba=(0.92, 0.91, 0.87, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.28, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=marble,
        name="marble_block",
    )
    base.visual(
        Box((0.12, 0.10, 0.012)),
        origin=Origin(xyz=(-0.16, 0.0, 0.071)),
        material=brushed_steel,
        name="mount_plate",
    )
    base.visual(
        Box((0.055, 0.060, 0.032)),
        origin=Origin(xyz=(-0.16, 0.0, 0.093)),
        material=dark_steel,
        name="pivot_saddle",
    )
    base.visual(
        Box((0.030, 0.012, 0.110)),
        origin=Origin(xyz=(-0.16, 0.038, 0.120)),
        material=dark_steel,
        name="left_yoke_plate",
    )
    base.visual(
        Box((0.030, 0.012, 0.110)),
        origin=Origin(xyz=(-0.16, -0.038, 0.120)),
        material=dark_steel,
        name="right_yoke_plate",
    )
    for index, foot_xy in enumerate(((-0.17, -0.10), (-0.17, 0.10), (0.17, -0.10), (0.17, 0.10))):
        base.visual(
            Box((0.030, 0.030, 0.008)),
            origin=Origin(xyz=(foot_xy[0], foot_xy[1], 0.004)),
            material=rubber,
            name=f"foot_pad_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.28, 0.18)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.015, length=0.064),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_barrel",
    )
    post.visual(
        Box((0.058, 0.042, 0.038)),
        origin=Origin(xyz=(0.032, 0.0, 0.016)),
        material=dark_steel,
        name="pivot_housing",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(0.058, 0.0, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="lower_socket",
    )
    post.visual(
        _mesh(
            "arc_post_tube",
            tube_from_spline_points(
                [
                    (0.055, 0.0, 0.050),
                    (0.200, 0.0, 0.420),
                    (0.520, 0.0, 0.960),
                    (0.900, 0.0, 1.360),
                    (1.240, 0.0, 1.530),
                ],
                radius=0.017,
                samples_per_segment=20,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=brushed_steel,
        name="arc_tube",
    )
    post.visual(
        Box((0.070, 0.046, 0.050)),
        origin=Origin(xyz=(1.250, 0.0, 1.570)),
        material=dark_steel,
        name="shade_receiver_block",
    )
    post.visual(
        Box((0.034, 0.060, 0.124)),
        origin=Origin(xyz=(1.292, 0.0, 1.649)),
        material=dark_steel,
        name="shade_bracket_riser",
    )
    post.visual(
        Box((0.032, 0.010, 0.060)),
        origin=Origin(xyz=(1.316, 0.029, 1.720)),
        material=dark_steel,
        name="left_shade_bracket_plate",
    )
    post.visual(
        Box((0.032, 0.010, 0.060)),
        origin=Origin(xyz=(1.316, -0.029, 1.720)),
        material=dark_steel,
        name="right_shade_bracket_plate",
    )
    post.inertial = Inertial.from_geometry(
        Box((1.34, 0.10, 1.64)),
        mass=8.5,
        origin=Origin(xyz=(0.67, 0.0, 0.82)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shade_hinge_barrel",
    )
    shade.visual(
        Box((0.022, 0.028, 0.052)),
        origin=Origin(xyz=(0.010, 0.0, -0.034)),
        material=dark_steel,
        name="shade_hinge_drop",
    )
    shade.visual(
        Box((0.092, 0.028, 0.020)),
        origin=Origin(xyz=(0.066, 0.0, -0.062)),
        material=dark_steel,
        name="shade_hinge_arm",
    )
    shade.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.122, 0.0, -0.078)),
        material=dark_steel,
        name="shade_cap",
    )
    shade.visual(
        _mesh("shade_outer_shell", DomeGeometry(radius=0.160, closed=False)),
        origin=Origin(xyz=(0.195, 0.0, -0.228)),
        material=shade_charcoal,
        name="shade_outer_shell",
    )
    shade.visual(
        _mesh("shade_inner_shell", DomeGeometry(radius=0.152, closed=False)),
        origin=Origin(xyz=(0.195, 0.0, -0.224)),
        material=shade_inner,
        name="shade_inner_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.20)),
        mass=1.2,
        origin=Origin(xyz=(0.180, 0.0, -0.140)),
    )

    model.articulation(
        "base_to_post_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(-0.16, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.55,
            lower=radians(-14.0),
            upper=radians(24.0),
        ),
    )
    model.articulation(
        "post_to_shade_tilt",
        ArticulationType.REVOLUTE,
        parent=post,
        child=shade,
        origin=Origin(xyz=(1.316, 0.0, 1.720)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=radians(-35.0),
            upper=radians(25.0),
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
    shade = object_model.get_part("shade")
    post_tilt = object_model.get_articulation("base_to_post_tilt")
    shade_tilt = object_model.get_articulation("post_to_shade_tilt")

    ctx.check(
        "post tilt axis pitches in the lamp arc plane",
        post_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={post_tilt.axis}",
    )
    ctx.check(
        "shade hinge axis pitches the round shade",
        shade_tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={shade_tilt.axis}",
    )

    with ctx.pose({post_tilt: 0.0, shade_tilt: 0.0}):
        ctx.expect_contact(
            post,
            base,
            elem_a="pivot_barrel",
            elem_b="left_yoke_plate",
            name="post barrel seats against left base yoke",
        )
        ctx.expect_contact(
            post,
            base,
            elem_a="pivot_barrel",
            elem_b="right_yoke_plate",
            name="post barrel seats against right base yoke",
        )
        ctx.expect_contact(
            shade,
            post,
            elem_a="shade_hinge_barrel",
            elem_b="left_shade_bracket_plate",
            name="shade hinge seats against left bracket plate",
        )
        ctx.expect_contact(
            shade,
            post,
            elem_a="shade_hinge_barrel",
            elem_b="right_shade_bracket_plate",
            name="shade hinge seats against right bracket plate",
        )
        rest_shade_origin = ctx.part_world_position(shade)
        rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_outer_shell")

    with ctx.pose({post_tilt: post_tilt.motion_limits.upper, shade_tilt: 0.0}):
        raised_shade_origin = ctx.part_world_position(shade)

    ctx.check(
        "post tilt raises the arc tip",
        rest_shade_origin is not None
        and raised_shade_origin is not None
        and raised_shade_origin[2] > rest_shade_origin[2] + 0.16,
        details=f"rest={rest_shade_origin}, raised={raised_shade_origin}",
    )

    with ctx.pose({post_tilt: 0.0, shade_tilt: shade_tilt.motion_limits.upper}):
        aimed_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_outer_shell")

    rest_shell_center = _aabb_center(rest_shell_aabb)
    aimed_shell_center = _aabb_center(aimed_shell_aabb)
    ctx.check(
        "shade hinge swings the shell forward",
        rest_shell_center is not None
        and aimed_shell_center is not None
        and aimed_shell_center[0] > rest_shell_center[0] + 0.025,
        details=f"rest={rest_shell_center}, aimed={aimed_shell_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
