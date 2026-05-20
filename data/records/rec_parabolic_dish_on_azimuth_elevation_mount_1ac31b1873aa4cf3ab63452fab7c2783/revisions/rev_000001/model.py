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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _build_reflector_shell():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.050, -0.400),
            (0.160, -0.396),
            (0.340, -0.374),
            (0.560, -0.315),
            (0.760, -0.218),
            (0.900, -0.110),
            (0.980, 0.000),
        ],
        [
            (0.022, -0.378),
            (0.120, -0.368),
            (0.300, -0.348),
            (0.510, -0.294),
            (0.700, -0.206),
            (0.860, -0.112),
            (0.940, -0.018),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    shell.rotate_y(math.pi / 2.0).translate(0.28, 0.0, 0.0)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracking_dish")

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.69, 1.0))
    pedestal_gray = model.material("pedestal_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    fork_gray = model.material("fork_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    dish_white = model.material("dish_white", rgba=(0.89, 0.91, 0.93, 1.0))
    alloy = model.material("alloy", rgba=(0.70, 0.73, 0.76, 1.0))
    counterweight_black = model.material(
        "counterweight_black",
        rgba=(0.10, 0.11, 0.12, 1.0),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.78, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="foundation_disc",
    )
    base.visual(
        Cylinder(radius=0.48, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=pedestal_gray,
        name="bearing_plinth",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.78, length=0.18),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.55, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=pedestal_gray,
        name="yaw_table",
    )
    pedestal.visual(
        Cylinder(radius=0.20, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=pedestal_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.28, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=fork_gray,
        name="fork_seat",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.55, length=0.80),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )

    fork = model.part("fork")
    fork.visual(
        Box((0.16, 2.30, 0.12)),
        origin=Origin(xyz=(-0.24, 0.0, 0.06)),
        material=fork_gray,
        name="lower_cradle_beam",
    )
    fork.visual(
        Box((0.18, 0.52, 0.16)),
        origin=Origin(xyz=(-0.24, 0.0, 0.08)),
        material=pedestal_gray,
        name="fork_saddle",
    )
    fork.visual(
        Box((0.16, 0.12, 0.92)),
        origin=Origin(xyz=(-0.24, 1.09, 0.58)),
        material=fork_gray,
        name="left_fork_arm",
    )
    fork.visual(
        Box((0.16, 0.12, 0.92)),
        origin=Origin(xyz=(-0.24, -1.09, 0.58)),
        material=fork_gray,
        name="right_fork_arm",
    )
    fork.visual(
        Box((0.22, 0.12, 0.20)),
        origin=Origin(xyz=(-0.09, 1.09, 1.04)),
        material=pedestal_gray,
        name="left_bearing_block",
    )
    fork.visual(
        Box((0.22, 0.12, 0.20)),
        origin=Origin(xyz=(-0.09, -1.09, 1.04)),
        material=pedestal_gray,
        name="right_bearing_block",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.38, 2.30, 1.16)),
        mass=520.0,
        origin=Origin(xyz=(-0.17, 0.0, 0.58)),
    )

    reflector = model.part("reflector")
    reflector.visual(
        _mesh(_build_reflector_shell(), "reflector_shell"),
        material=dish_white,
        name="reflector_shell",
    )
    reflector.visual(
        Cylinder(radius=0.045, length=2.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="trunnion_shaft",
    )
    reflector.visual(
        Box((0.30, 0.38, 0.38)),
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
        material=pedestal_gray,
        name="rear_hub",
    )
    reflector.visual(
        Box((0.42, 0.08, 0.10)),
        origin=Origin(xyz=(-0.31, 0.0, 0.0)),
        material=fork_gray,
        name="counterweight_boom",
    )
    reflector.visual(
        Cylinder(radius=0.13, length=0.46),
        origin=Origin(xyz=(-0.64, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=counterweight_black,
        name="counterweight_mass",
    )
    reflector.visual(
        Cylinder(radius=0.025, length=0.50),
        origin=Origin(xyz=(0.39, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="feed_support_boom",
    )
    reflector.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=Origin(xyz=(0.67, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="feed_horn",
    )
    reflector.inertial = Inertial.from_geometry(
        Box((1.70, 2.10, 1.10)),
        mass=230.0,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "pedestal_to_fork",
        ArticulationType.FIXED,
        parent=pedestal,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
    )
    model.articulation(
        "fork_to_reflector",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.5,
            lower=-0.25,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    fork = object_model.get_part("fork")
    reflector = object_model.get_part("reflector")
    yaw = object_model.get_articulation("base_yaw")
    elevation = object_model.get_articulation("fork_to_reflector")

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

    ctx.check(
        "primary_parts_present",
        {part.name for part in object_model.parts} == {"base", "pedestal", "fork", "reflector"},
        details=str([part.name for part in object_model.parts]),
    )
    ctx.check(
        "primary_joints_present",
        {joint.name for joint in object_model.articulations}
        == {"base_yaw", "pedestal_to_fork", "fork_to_reflector"},
        details=str([joint.name for joint in object_model.articulations]),
    )
    ctx.check(
        "yaw_axis_vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "elevation_axis_horizontal",
        elevation.axis == (0.0, -1.0, 0.0),
        details=f"axis={elevation.axis}",
    )
    ctx.check(
        "elevation_limits_readable",
        elevation.motion_limits is not None
        and elevation.motion_limits.lower is not None
        and elevation.motion_limits.upper is not None
        and elevation.motion_limits.lower < 0.0
        and elevation.motion_limits.upper > 1.0,
        details=str(elevation.motion_limits),
    )

    ctx.expect_contact(
        pedestal,
        base,
        elem_a="yaw_table",
        elem_b="bearing_plinth",
        name="pedestal_turntable_seated_on_base",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="xy",
        elem_a="yaw_table",
        elem_b="bearing_plinth",
        min_overlap=0.90,
        name="pedestal_supported_by_base_footprint",
    )
    ctx.expect_contact(
        fork,
        pedestal,
        elem_a="fork_saddle",
        elem_b="fork_seat",
        name="fork_seated_on_pedestal",
    )

    with ctx.pose({elevation: 0.0}):
        ctx.expect_contact(
            reflector,
            fork,
            elem_a="trunnion_shaft",
            elem_b="left_bearing_block",
            name="left_trunnion_bearing_contact",
        )
        ctx.expect_contact(
            reflector,
            fork,
            elem_a="trunnion_shaft",
            elem_b="right_bearing_block",
            name="right_trunnion_bearing_contact",
        )
        ctx.expect_gap(
            reflector,
            fork,
            axis="x",
            positive_elem="reflector_shell",
            negative_elem="lower_cradle_beam",
            min_gap=0.02,
            name="dish_shell_clears_rear_cradle_beam",
        )

    with ctx.pose({elevation: 1.20}):
        ctx.expect_gap(
            reflector,
            fork,
            axis="z",
            positive_elem="counterweight_mass",
            negative_elem="lower_cradle_beam",
            min_gap=0.10,
            name="counterweight_stays_above_cradle_beam_at_high_elevation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
