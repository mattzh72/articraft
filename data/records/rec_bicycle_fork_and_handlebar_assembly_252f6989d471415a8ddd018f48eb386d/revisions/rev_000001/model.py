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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
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
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recumbent_trike_front_fork")

    frame_gray = model.material("frame_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    fork_black = model.material("fork_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.71, 0.73, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.34, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.08, 1.0))

    head_tube = model.part("head_tube")
    head_tube_shell = LatheGeometry.from_shell_profiles(
        [
            (0.033, 0.000),
            (0.032, 0.010),
            (0.031, 0.022),
            (0.031, 0.123),
            (0.032, 0.135),
            (0.033, 0.145),
        ],
        [
            (0.0225, 0.000),
            (0.0220, 0.010),
            (0.0215, 0.022),
            (0.0215, 0.123),
            (0.0220, 0.135),
            (0.0225, 0.145),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    head_tube.visual(
        _save_mesh("head_tube_shell", head_tube_shell),
        material=frame_gray,
        name="head_tube_shell",
    )
    head_tube.visual(
        Box((0.072, 0.050, 0.032)),
        origin=Origin(xyz=(-0.056, 0.0, 0.040)),
        material=frame_gray,
        name="boom_lug",
    )
    head_tube.visual(
        Box((0.036, 0.060, 0.026)),
        origin=Origin(xyz=(-0.036, 0.0, 0.016)),
        material=frame_gray,
        name="lower_gusset",
    )
    head_tube.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.170)),
        mass=1.4,
        origin=Origin(xyz=(-0.018, 0.0, 0.072)),
    )

    steering = model.part("steering_assembly")
    steering.visual(
        Cylinder(radius=0.014, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=satin_steel,
        name="steerer_shaft",
    )
    steering.visual(
        Cylinder(radius=0.013, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.2475)),
        material=satin_steel,
        name="threaded_steerer",
    )
    for idx in range(6):
        steering.visual(
            Cylinder(radius=0.0152, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.224 + idx * 0.008)),
            material=satin_steel,
            name=f"thread_ring_{idx}",
        )
    steering.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=dark_steel,
        name="lower_headset_race",
    )
    steering.visual(
        Cylinder(radius=0.0175, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=fork_black,
        name="crown_steerer_socket",
    )
    steering.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=dark_steel,
        name="upper_locknut",
    )
    steering.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=dark_steel,
        name="stem_riser",
    )
    steering.visual(
        Box((0.064, 0.164, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, -0.042)),
        material=fork_black,
        name="crown_bridge",
    )
    steering.visual(
        Box((0.034, 0.022, 0.044)),
        origin=Origin(xyz=(0.014, 0.056, -0.060)),
        material=fork_black,
        name="left_crown_cheek",
    )
    steering.visual(
        Box((0.034, 0.022, 0.044)),
        origin=Origin(xyz=(0.014, -0.056, -0.060)),
        material=fork_black,
        name="right_crown_cheek",
    )
    steering.visual(
        Cylinder(radius=0.011, length=0.308),
        origin=Origin(xyz=(0.014, 0.056, -0.214)),
        material=fork_black,
        name="left_blade_leg",
    )
    steering.visual(
        Cylinder(radius=0.011, length=0.308),
        origin=Origin(xyz=(0.014, -0.056, -0.214)),
        material=fork_black,
        name="right_blade_leg",
    )
    steering.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(0.014, 0.056, -0.378)),
        material=dark_steel,
        name="left_dropout",
    )
    steering.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(0.014, -0.056, -0.378)),
        material=dark_steel,
        name="right_dropout",
    )
    steering.visual(
        Cylinder(radius=0.015, length=0.100),
        origin=Origin(xyz=(0.050, 0.0, 0.185), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="stem_extension",
    )
    steering.visual(
        Cylinder(radius=0.019, length=0.054),
        origin=Origin(xyz=(0.105, 0.0, 0.185), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pinch_collar",
    )
    steering.visual(
        Box((0.018, 0.014, 0.024)),
        origin=Origin(xyz=(0.120, 0.0, 0.203)),
        material=satin_steel,
        name="pinch_bolt_head",
    )

    bullhorn_bar = tube_from_spline_points(
        [
            (0.255, -0.188, 0.222),
            (0.228, -0.180, 0.220),
            (0.205, -0.164, 0.214),
            (0.168, -0.132, 0.202),
            (0.132, -0.090, 0.190),
            (0.105, -0.040, 0.185),
            (0.105, 0.000, 0.185),
            (0.105, 0.040, 0.185),
            (0.132, 0.090, 0.190),
            (0.168, 0.132, 0.202),
            (0.205, 0.164, 0.214),
            (0.228, 0.180, 0.220),
            (0.255, 0.188, 0.222),
        ],
        radius=0.0105,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    steering.visual(
        _save_mesh("bullhorn_bar", bullhorn_bar),
        material=satin_steel,
        name="bullhorn_bar",
    )
    steering.visual(
        Cylinder(radius=0.013, length=0.072),
        origin=Origin(xyz=(0.228, 0.182, 0.220), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    steering.visual(
        Cylinder(radius=0.013, length=0.072),
        origin=Origin(xyz=(0.228, -0.182, 0.220), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.300, 0.410, 0.680)),
        mass=3.4,
        origin=Origin(xyz=(0.100, 0.0, -0.060)),
    )

    model.articulation(
        "head_tube_to_steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steering,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.3,
            lower=-0.70,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    steering = object_model.get_part("steering_assembly")
    steer_joint = object_model.get_articulation("head_tube_to_steering")
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    steer_axis = steer_joint.axis
    steer_limits = steer_joint.motion_limits
    ctx.check(
        "steering axis is nearly vertical",
        steer_axis is not None and abs(steer_axis[2]) > 0.97 and abs(steer_axis[0]) < 0.10 and abs(steer_axis[1]) < 0.10,
        details=f"axis={steer_axis}",
    )
    ctx.check(
        "steering range spans left and right lock",
        steer_limits is not None
        and steer_limits.lower is not None
        and steer_limits.upper is not None
        and steer_limits.lower < -0.5
        and steer_limits.upper > 0.5,
        details=f"limits={steer_limits}",
    )
    ctx.check(
        "bullhorn assembly is wide enough to read as paired handlebars",
        (
            lambda aabb: aabb is not None and (aabb[1][1] - aabb[0][1]) > 0.34
        )(ctx.part_world_aabb(steering)),
        details=f"aabb={ctx.part_world_aabb(steering)}",
    )
    ctx.expect_within(
        steering,
        head_tube,
        axes="xy",
        inner_elem="steerer_shaft",
        outer_elem="head_tube_shell",
        margin=0.012,
        name="steerer stays centered in the head tube",
    )
    ctx.expect_overlap(
        steering,
        head_tube,
        axes="z",
        elem_a="steerer_shaft",
        elem_b="head_tube_shell",
        min_overlap=0.135,
        name="steerer spans the full head tube bore",
    )
    ctx.expect_gap(
        steering,
        head_tube,
        axis="z",
        positive_elem="upper_locknut",
        negative_elem="head_tube_shell",
        max_gap=0.0015,
        max_penetration=0.0,
        name="upper locknut seats on the top headset face",
    )
    ctx.expect_gap(
        head_tube,
        steering,
        axis="z",
        positive_elem="head_tube_shell",
        negative_elem="lower_headset_race",
        max_gap=0.0015,
        max_penetration=0.0,
        name="lower headset race seats on the bottom headset face",
    )

    upper_limit = 0.70
    if steer_limits is not None and steer_limits.upper is not None:
        upper_limit = steer_limits.upper
    rest_dropout_aabb = ctx.part_element_world_aabb(steering, elem="left_dropout")
    rest_dropout_center = _aabb_center(rest_dropout_aabb)
    turned_dropout_center = None
    with ctx.pose({steer_joint: upper_limit}):
        turned_dropout_center = _aabb_center(ctx.part_element_world_aabb(steering, elem="left_dropout"))
    ctx.check(
        "positive steer rotates the fork around the head tube axis",
        rest_dropout_center is not None
        and turned_dropout_center is not None
        and turned_dropout_center[0] < rest_dropout_center[0] - 0.015,
        details=f"rest={rest_dropout_center}, turned={turned_dropout_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
