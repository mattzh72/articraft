from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * i) / segments),
            radius * sin((2.0 * pi * i) / segments),
        )
        for i in range(segments)
    ]


def _ring_segment_profile(
    outer_radius: float,
    inner_radius: float,
    start_deg: float,
    end_deg: float,
    samples: int = 36,
) -> list[tuple[float, float]]:
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for i in range(samples + 1):
        angle = (start_deg + (end_deg - start_deg) * (i / samples)) * pi / 180.0
        outer.append((outer_radius * cos(angle), outer_radius * sin(angle)))
    for i in range(samples, -1, -1):
        angle = (start_deg + (end_deg - start_deg) * (i / samples)) * pi / 180.0
        inner.append((inner_radius * cos(angle), inner_radius * sin(angle)))
    return outer + inner


def _blade_profile(
    tip_radius: float,
    root_radius: float,
    teeth: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for tooth in range(teeth):
        base_angle = (2.0 * pi * tooth) / teeth
        tip_a = base_angle - (0.28 * pi / teeth)
        tip_b = base_angle + (0.28 * pi / teeth)
        valley = base_angle + (0.5 * pi / teeth)
        points.append((tip_radius * cos(tip_a), tip_radius * sin(tip_a)))
        points.append((tip_radius * cos(tip_b), tip_radius * sin(tip_b)))
        points.append((root_radius * cos(valley), root_radius * sin(valley)))
    return points


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="miter_saw_arm_assembly", assets=ASSETS)

    cast_aluminum = _material("cast_aluminum", (0.74, 0.75, 0.77, 1.0))
    brushed_steel = _material("brushed_steel", (0.72, 0.74, 0.76, 1.0))
    saw_red = _material("tool_red", (0.67, 0.16, 0.12, 1.0))
    charcoal = _material("charcoal_plastic", (0.18, 0.19, 0.21, 1.0))
    rubber_black = _material("rubber_black", (0.08, 0.08, 0.09, 1.0))
    blade_steel = _material("blade_steel", (0.82, 0.84, 0.86, 1.0))
    amber_guard = _material("amber_guard", (0.78, 0.60, 0.18, 0.32))

    model.materials.extend(
        [
            cast_aluminum,
            brushed_steel,
            saw_red,
            charcoal,
            rubber_black,
            blade_steel,
            amber_guard,
        ]
    )

    base_profile = [
        (-0.31, -0.27),
        (0.31, -0.27),
        (0.29, -0.09),
        (0.24, 0.11),
        (0.17, 0.23),
        (-0.17, 0.23),
        (-0.24, 0.11),
        (-0.29, -0.09),
    ]
    base_shell = _mesh("base_shell.obj", ExtrudeGeometry.from_z0(base_profile, 0.055))

    turntable_blade = _mesh(
        "turntable_blade.obj",
        ExtrudeWithHolesGeometry(
            _blade_profile(tip_radius=0.115, root_radius=0.108, teeth=48),
            [_circle_profile(0.016, segments=28)],
            height=0.004,
            center=True,
        ),
    )
    upper_guard = _mesh(
        "upper_guard.obj",
        ExtrudeGeometry.centered(
            _ring_segment_profile(
                outer_radius=0.145,
                inner_radius=0.112,
                start_deg=25.0,
                end_deg=240.0,
                samples=48,
            ),
            0.062,
        ),
    )
    lower_guard = _mesh(
        "lower_guard.obj",
        ExtrudeGeometry.centered(
            _ring_segment_profile(
                outer_radius=0.126,
                inner_radius=0.108,
                start_deg=40.0,
                end_deg=125.0,
                samples=24,
            ),
            0.058,
        ),
    )
    handle_tube = _mesh(
        "handle_tube.obj",
        tube_from_spline_points(
            [
                (0.0, 0.205, 0.095),
                (0.0, 0.228, 0.145),
                (0.0, 0.286, 0.138),
                (0.0, 0.328, 0.090),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    base = model.part("base")
    base.visual(base_shell, material=cast_aluminum)
    base.visual(
        Cylinder(radius=0.13, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=brushed_steel,
    )
    base.visual(
        Box((0.22, 0.028, 0.090)),
        origin=Origin(xyz=(-0.145, -0.206, 0.100)),
        material=cast_aluminum,
    )
    base.visual(
        Box((0.22, 0.028, 0.090)),
        origin=Origin(xyz=(0.145, -0.206, 0.100)),
        material=cast_aluminum,
    )
    base.visual(
        Box((0.03, 0.058, 0.220)),
        origin=Origin(xyz=(-0.075, -0.246, 0.165)),
        material=cast_aluminum,
    )
    base.visual(
        Box((0.03, 0.058, 0.220)),
        origin=Origin(xyz=(0.075, -0.246, 0.165)),
        material=cast_aluminum,
    )
    base.visual(
        Box((0.19, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, -0.245, 0.110)),
        material=cast_aluminum,
    )
    base.visual(
        Box((0.11, 0.028, 0.040)),
        origin=Origin(xyz=(0.0, -0.219, 0.250)),
        material=brushed_steel,
    )
    base.visual(
        Box((0.18, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.165, 0.064)),
        material=charcoal,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.54, 0.27)),
        mass=16.0,
        origin=Origin(xyz=(0.0, -0.02, 0.135)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.07, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_steel,
    )
    turntable.visual(
        Cylinder(radius=0.18, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=cast_aluminum,
    )
    turntable.visual(
        Cylinder(radius=0.19, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=charcoal,
    )
    turntable.visual(
        Box((0.072, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.146, 0.012)),
        material=charcoal,
    )
    turntable.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(xyz=(0.0, 0.204, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.020),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    saw_arm = model.part("saw_arm")
    saw_arm.visual(
        Box((0.11, 0.036, 0.060)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=brushed_steel,
    )
    saw_arm.visual(
        Box((0.13, 0.075, 0.085)),
        origin=Origin(xyz=(0.0, 0.061, 0.028)),
        material=charcoal,
    )
    saw_arm.visual(
        Cylinder(radius=0.012, length=0.190),
        origin=Origin(xyz=(-0.042, 0.150, 0.040), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
    )
    saw_arm.visual(
        Cylinder(radius=0.012, length=0.190),
        origin=Origin(xyz=(0.042, 0.150, 0.040), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
    )
    saw_arm.visual(
        Box((0.11, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.120, 0.058)),
        material=charcoal,
    )
    saw_arm.visual(
        Box((0.18, 0.090, 0.090)),
        origin=Origin(xyz=(0.0, 0.245, 0.045)),
        material=saw_red,
    )
    saw_arm.visual(
        Box((0.075, 0.125, 0.110)),
        origin=Origin(xyz=(0.0, 0.185, -0.005)),
        material=charcoal,
    )
    saw_arm.visual(
        Cylinder(radius=0.057, length=0.110),
        origin=Origin(xyz=(0.0, 0.245, 0.032), rpy=(0.0, pi / 2.0, 0.0)),
        material=saw_red,
    )
    saw_arm.visual(
        Cylinder(radius=0.034, length=0.075),
        origin=Origin(xyz=(0.0, 0.167, -0.012), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    saw_arm.visual(
        turntable_blade,
        origin=Origin(xyz=(0.0, 0.165, -0.015), rpy=(0.0, pi / 2.0, 0.0)),
        material=blade_steel,
    )
    saw_arm.visual(
        upper_guard,
        origin=Origin(xyz=(0.0, 0.165, -0.015), rpy=(0.0, pi / 2.0, 0.0)),
        material=saw_red,
    )
    saw_arm.visual(
        lower_guard,
        origin=Origin(xyz=(0.0, 0.176, -0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=amber_guard,
    )
    saw_arm.visual(handle_tube, material=rubber_black)
    saw_arm.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(xyz=(0.0, 0.328, 0.087), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_black,
    )
    saw_arm.visual(
        Box((0.032, 0.040, 0.075)),
        origin=Origin(xyz=(0.0, 0.309, 0.066)),
        material=charcoal,
    )
    saw_arm.inertial = Inertial.from_geometry(
        Box((0.32, 0.38, 0.20)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.165, 0.035)),
    )

    model.articulation(
        "miter_rotation",
        ArticulationType.REVOLUTE,
        parent="base",
        child="turntable",
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.82,
            upper=0.82,
        ),
    )
    model.articulation(
        "saw_arm_tilt",
        ArticulationType.REVOLUTE,
        parent="base",
        child="saw_arm",
        origin=Origin(xyz=(0.0, -0.204, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-0.07,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "saw_arm",
        "turntable",
        reason="the lowered blade and guard sweep into the turntable kerf region during the cut stroke",
    )
    ctx.allow_overlap(
        "base",
        "saw_arm",
        reason="the arm carriage nests tightly inside the rear yoke around the hinge pivot",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("turntable", "base", axes="xy", min_overlap=0.22)
    ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.025)
    ctx.expect_aabb_overlap("saw_arm", "base", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("saw_arm", "turntable", axes="xy", min_overlap=0.05)
    ctx.expect_joint_motion_axis(
        "saw_arm_tilt",
        "saw_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(saw_arm_tilt=-0.07):
        ctx.expect_aabb_overlap("saw_arm", "turntable", axes="xy", min_overlap=0.06)
        ctx.expect_origin_distance("saw_arm", "turntable", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("saw_arm", "base", axes="xy", min_overlap=0.04)

    with ctx.pose(saw_arm_tilt=0.95):
        ctx.expect_aabb_overlap("saw_arm", "base", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("saw_arm", "base", axes="xy", max_dist=0.22)
        ctx.expect_aabb_overlap("saw_arm", "turntable", axes="xy", min_overlap=0.03)

    with ctx.pose(miter_rotation=-0.82):
        ctx.expect_aabb_overlap("turntable", "base", axes="xy", min_overlap=0.22)
        ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.03)

    with ctx.pose(miter_rotation=0.82):
        ctx.expect_aabb_overlap("turntable", "base", axes="xy", min_overlap=0.22)
        ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.03)

    with ctx.pose(miter_rotation=-0.82, saw_arm_tilt=-0.07):
        ctx.expect_aabb_overlap("saw_arm", "turntable", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("saw_arm", "turntable", axes="xy", max_dist=0.22)

    with ctx.pose(miter_rotation=0.82, saw_arm_tilt=-0.07):
        ctx.expect_aabb_overlap("saw_arm", "turntable", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("saw_arm", "turntable", axes="xy", max_dist=0.22)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
