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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="designer_arc_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.71, 0.72, 0.74, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.72, 0.60, 0.34, 1.0))

    pivot_x = -0.11
    tube_radius = 0.018
    axle_radius = 0.012
    axle_length = 0.056

    arc_points = [
        (0.00, 0.00, 0.055),
        (0.08, 0.00, 0.31),
        (0.28, 0.00, 0.92),
        (0.66, 0.00, 1.55),
        (0.96, 0.00, 1.78),
        (1.16, 0.00, 1.84),
    ]
    arc_mesh = _save_mesh(
        "arc_floor_lamp_post",
        tube_from_spline_points(
            arc_points,
            radius=tube_radius,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )

    tip_dx = arc_points[-1][0] - arc_points[-2][0]
    tip_dz = arc_points[-1][2] - arc_points[-2][2]
    tip_length = math.hypot(tip_dx, tip_dz)
    tip_dir = (tip_dx / tip_length, 0.0, tip_dz / tip_length)
    tip_pitch = math.atan2(tip_dz, tip_dx)
    tip_cylinder_pitch = math.atan2(tip_dir[0], tip_dir[2])
    tip_origin = (
        arc_points[-1][0] + (0.018 * tip_dir[0]),
        0.0,
        arc_points[-1][2] + (0.018 * tip_dir[2]),
    )
    tip_stem_center = (
        arc_points[-1][0] + (0.009 * tip_dir[0]),
        0.0,
        arc_points[-1][2] + (0.009 * tip_dir[2]),
    )

    shade_shell = _save_mesh(
        "arc_floor_lamp_shade",
        LatheGeometry.from_shell_profiles(
            [
                (0.125, 0.000),
                (0.119, 0.030),
                (0.090, 0.105),
                (0.060, 0.170),
                (0.032, 0.212),
                (0.010, 0.225),
            ],
            [
                (0.118, 0.000),
                (0.112, 0.030),
                (0.084, 0.105),
                (0.055, 0.170),
                (0.028, 0.212),
                (0.008, 0.225),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    base = model.part("base")
    base.visual(
        Box((0.42, 0.42, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="base_plate",
    )
    base.visual(
        Box((0.072, 0.070, 0.018)),
        origin=Origin(xyz=(pivot_x, 0.0, 0.029)),
        material=matte_black,
        name="pivot_pedestal",
    )
    base.visual(
        Box((0.030, 0.014, 0.060)),
        origin=Origin(xyz=(pivot_x, 0.035, 0.050)),
        material=matte_black,
        name="left_yoke",
    )
    base.visual(
        Box((0.030, 0.014, 0.060)),
        origin=Origin(xyz=(pivot_x, -0.035, 0.050)),
        material=matte_black,
        name="right_yoke",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="tilt_axle",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=matte_black,
        name="pivot_hub",
    )
    post.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(0.010, 0.0, 0.065)),
        material=matte_black,
        name="hub_riser",
    )
    post.visual(
        arc_mesh,
        material=matte_black,
        name="arc_tube",
    )
    post.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=tip_stem_center, rpy=(0.0, tip_cylinder_pitch, 0.0)),
        material=satin_steel,
        name="tip_stem",
    )
    post.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=tip_origin),
        material=satin_steel,
        name="tip_spindle",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="swivel_collar",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.110),
        origin=Origin(xyz=(0.024, 0.0, -0.045)),
        material=satin_steel,
        name="shade_neck",
    )
    shade.visual(
        shade_shell,
        origin=Origin(xyz=(0.030, 0.0, -0.255)),
        material=warm_brass,
        name="shade_shell",
    )

    model.articulation(
        "base_to_post_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(pivot_x, 0.0, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.9,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "post_to_shade_swivel",
        ArticulationType.REVOLUTE,
        parent=post,
        child=shade,
        origin=Origin(xyz=tip_origin, rpy=(0.0, -tip_pitch, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-1.10,
            upper=1.10,
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
    tilt = object_model.get_articulation("base_to_post_tilt")
    swivel = object_model.get_articulation("post_to_shade_swivel")

    ctx.check(
        "tilt joint uses a horizontal axis",
        tuple(round(value, 6) for value in tilt.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.check(
        "shade joint rotates about the local tip axis",
        tuple(round(value, 6) for value in swivel.axis) == (1.0, 0.0, 0.0),
        details=f"axis={swivel.axis}",
    )

    ctx.expect_contact(
        post,
        base,
        elem_a="tilt_axle",
        elem_b="left_yoke",
        contact_tol=0.0005,
        name="left yoke captures the tilt axle",
    )
    ctx.expect_contact(
        post,
        base,
        elem_a="tilt_axle",
        elem_b="right_yoke",
        contact_tol=0.0005,
        name="right yoke captures the tilt axle",
    )
    ctx.expect_contact(
        shade,
        post,
        elem_a="swivel_collar",
        elem_b="tip_spindle",
        contact_tol=0.003,
        name="shade collar seats against the tip spindle",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        min_gap=1.45,
        name="shade hangs well above the square base",
    )

    rest_shade_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits and tilt.motion_limits.upper is not None else 0.35
    with ctx.pose({tilt: tilt_upper}):
        raised_shade_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "tilting the post raises the arc tip",
        rest_shade_center is not None
        and raised_shade_center is not None
        and raised_shade_center[2] > rest_shade_center[2] + 0.12,
        details=f"rest={rest_shade_center}, raised={raised_shade_center}",
    )

    swivel_upper = swivel.motion_limits.upper if swivel.motion_limits and swivel.motion_limits.upper is not None else 1.10
    rest_shell_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    with ctx.pose({swivel: swivel_upper}):
        swiveled_shell_center = _aabb_center(ctx.part_element_world_aabb(shade, elem="shade_shell"))
    ctx.check(
        "shade swivel visibly re-aims the shade",
        rest_shell_center is not None
        and swiveled_shell_center is not None
        and abs(swiveled_shell_center[1] - rest_shell_center[1]) > 0.06,
        details=f"rest={rest_shell_center}, swiveled={swiveled_shell_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
