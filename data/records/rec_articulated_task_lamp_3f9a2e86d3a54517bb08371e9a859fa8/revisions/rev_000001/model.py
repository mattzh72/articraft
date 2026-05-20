from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


SHOULDER_Z = 1.10
ARM_TIP = (0.455, 0.0, 0.055)
SHADE_REST_PITCH = 0.55


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _sleeve_geometry():
    return LatheGeometry.from_shell_profiles(
        outer_profile=[(0.032, -0.028), (0.032, 0.028)],
        inner_profile=[(0.020, -0.028), (0.020, 0.028)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _main_shade_geometry():
    # A thin, up-facing torchiere bowl: small neck at the post, flared open rim.
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.052, 0.000),
            (0.076, 0.026),
            (0.130, 0.078),
            (0.190, 0.132),
            (0.222, 0.168),
        ],
        inner_profile=[
            (0.045, 0.006),
            (0.069, 0.032),
            (0.123, 0.083),
            (0.181, 0.136),
            (0.212, 0.160),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _reading_shade_geometry():
    # Local +Z is revolved axis, then rotate so the shade opens along local +X.
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.032, 0.000),
            (0.041, 0.025),
            (0.057, 0.075),
            (0.072, 0.125),
        ],
        inner_profile=[
            (0.025, 0.006),
            (0.034, 0.030),
            (0.050, 0.078),
            (0.064, 0.117),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    ).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torchiere_floor_lamp")

    model.material("dark_bronze", rgba=(0.12, 0.10, 0.075, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.58, 0.50, 1.0))
    model.material("warm_cream", rgba=(0.94, 0.90, 0.78, 1.0))
    model.material("warm_light", rgba=(1.00, 0.78, 0.36, 0.65))
    model.material("black_rubber", rgba=(0.035, 0.035, 0.032, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.165, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="dark_bronze",
        name="round_base",
    )
    stand.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material="brushed_metal",
        name="base_step",
    )
    stand.visual(
        _mesh("base_bead", TorusGeometry(0.145, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material="brushed_metal",
        name="base_bead",
    )
    stand.visual(
        Cylinder(radius=0.014, length=1.585),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material="dark_bronze",
        name="post",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z - 0.034)),
        material="brushed_metal",
        name="lower_stop",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z + 0.040)),
        material="brushed_metal",
        name="upper_stop",
    )
    stand.visual(
        Cylinder(radius=0.029, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.610)),
        material="brushed_metal",
        name="top_socket",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 1.624)),
        material="brushed_metal",
        name="shade_neck_ring",
    )
    stand.visual(
        _mesh("main_dome_shade", _main_shade_geometry()),
        origin=Origin(xyz=(0.0, 0.0, 1.620)),
        material="warm_cream",
        name="main_dome_shade",
    )
    stand.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(0.0, 0.0, 1.666)),
        material="warm_light",
        name="main_bulb",
    )

    reading_arm = model.part("reading_arm")
    reading_arm.visual(
        _mesh("pivot_sleeve", _sleeve_geometry()),
        material="brushed_metal",
        name="pivot_sleeve",
    )
    reading_arm.visual(
        _mesh(
            "curved_reading_arm",
            tube_from_spline_points(
                [
                    (0.032, 0.0, 0.000),
                    (0.120, 0.0, 0.026),
                    (0.295, 0.0, 0.052),
                    (0.438, 0.0, 0.055),
                ],
                radius=0.0105,
                samples_per_segment=18,
                radial_segments=24,
                cap_ends=True,
            ),
        ),
        material="dark_bronze",
        name="curved_arm",
    )
    reading_arm.visual(
        Cylinder(radius=0.017, length=0.068),
        origin=Origin(xyz=ARM_TIP, rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="tip_barrel",
    )

    reading_shade = model.part("reading_shade")
    d = (cos(SHADE_REST_PITCH), 0.0, -sin(SHADE_REST_PITCH))
    axis_rpy = (0.0, pi / 2.0 + SHADE_REST_PITCH, 0.0)
    shell_origin = Origin(
        xyz=(d[0] * 0.088, 0.0, d[2] * 0.088),
        rpy=(0.0, SHADE_REST_PITCH, 0.0),
    )
    reading_shade.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="hinge_lug_0",
    )
    reading_shade.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="hinge_lug_1",
    )
    reading_shade.visual(
        Box((0.055, 0.012, 0.014)),
        origin=Origin(
            xyz=(d[0] * 0.029, 0.040, d[2] * 0.029),
            rpy=(0.0, SHADE_REST_PITCH, 0.0),
        ),
        material="brushed_metal",
        name="side_yoke_0",
    )
    reading_shade.visual(
        Box((0.055, 0.012, 0.014)),
        origin=Origin(
            xyz=(d[0] * 0.029, -0.040, d[2] * 0.029),
            rpy=(0.0, SHADE_REST_PITCH, 0.0),
        ),
        material="brushed_metal",
        name="side_yoke_1",
    )
    reading_shade.visual(
        Box((0.020, 0.086, 0.014)),
        origin=Origin(
            xyz=(d[0] * 0.058, 0.0, d[2] * 0.058),
            rpy=(0.0, SHADE_REST_PITCH, 0.0),
        ),
        material="brushed_metal",
        name="shade_yoke",
    )
    reading_shade.visual(
        Cylinder(radius=0.015, length=0.045),
        origin=Origin(xyz=(d[0] * 0.070, 0.0, d[2] * 0.070), rpy=axis_rpy),
        material="brushed_metal",
        name="shade_socket",
    )
    reading_shade.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(d[0] * 0.088, 0.0, d[2] * 0.088), rpy=axis_rpy),
        material="brushed_metal",
        name="reading_fitter",
    )
    reading_shade.visual(
        _mesh("reading_shade_shell", _reading_shade_geometry()),
        origin=shell_origin,
        material="warm_cream",
        name="reading_shade_shell",
    )
    reading_shade.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(d[0] * 0.135, 0.0, d[2] * 0.135), rpy=axis_rpy),
        material="brushed_metal",
        name="bulb_stem",
    )
    reading_shade.visual(
        Sphere(radius=0.023),
        origin=Origin(xyz=(d[0] * 0.175, 0.0, d[2] * 0.175)),
        material="warm_light",
        name="reading_bulb",
    )

    model.articulation(
        "shoulder_swivel",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=reading_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=reading_arm,
        child=reading_shade,
        origin=Origin(xyz=ARM_TIP),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=-0.65, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    arm = object_model.get_part("reading_arm")
    shade = object_model.get_part("reading_shade")
    shoulder = object_model.get_articulation("shoulder_swivel")
    tilt = object_model.get_articulation("shade_tilt")

    ctx.expect_within(
        stand,
        arm,
        axes="xy",
        inner_elem="post",
        outer_elem="pivot_sleeve",
        margin=0.001,
        name="pivot sleeve surrounds the post",
    )
    ctx.expect_overlap(
        stand,
        arm,
        axes="z",
        elem_a="post",
        elem_b="pivot_sleeve",
        min_overlap=0.045,
        name="sleeve has vertical bearing engagement",
    )
    ctx.expect_gap(
        stand,
        arm,
        axis="z",
        positive_elem="upper_stop",
        negative_elem="pivot_sleeve",
        min_gap=0.002,
        max_gap=0.015,
        name="upper stop clears rotating sleeve",
    )
    ctx.expect_contact(
        arm,
        stand,
        elem_a="pivot_sleeve",
        elem_b="lower_stop",
        contact_tol=0.001,
        name="rotating sleeve rests on lower stop collar",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a="hinge_lug_0",
        elem_b="tip_barrel",
        contact_tol=0.001,
        name="shade lug seats on arm hinge barrel",
    )

    def _aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({shoulder: 0.0}):
        tip_aabb_rest = ctx.part_element_world_aabb(arm, elem="tip_barrel")
    with ctx.pose({shoulder: 1.10}):
        tip_aabb_swept = ctx.part_element_world_aabb(arm, elem="tip_barrel")
    rest_tip = _aabb_center(tip_aabb_rest) if tip_aabb_rest is not None else None
    swept_tip = _aabb_center(tip_aabb_swept) if tip_aabb_swept is not None else None
    ctx.check(
        "shoulder swivel carries arm around post",
        rest_tip is not None
        and swept_tip is not None
        and swept_tip[1] > rest_tip[1] + 0.20,
        details=f"rest_tip={rest_tip}, swept_tip={swept_tip}",
    )

    with ctx.pose({tilt: -0.55}):
        shade_aabb_up = ctx.part_element_world_aabb(shade, elem="reading_bulb")
    with ctx.pose({tilt: 0.65}):
        shade_aabb_down = ctx.part_element_world_aabb(shade, elem="reading_bulb")
    up_bulb = _aabb_center(shade_aabb_up) if shade_aabb_up is not None else None
    down_bulb = _aabb_center(shade_aabb_down) if shade_aabb_down is not None else None
    ctx.check(
        "shade tilt changes beam pitch",
        up_bulb is not None
        and down_bulb is not None
        and down_bulb[2] < up_bulb[2] - 0.08,
        details=f"up_bulb={up_bulb}, down_bulb={down_bulb}",
    )

    return ctx.report()


object_model = build_object_model()
