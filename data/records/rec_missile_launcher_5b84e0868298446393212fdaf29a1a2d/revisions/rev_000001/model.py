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


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, name: str, a, b, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_missile_launcher")

    tripod_paint = model.material("tripod_paint", rgba=(0.24, 0.28, 0.24, 1.0))
    launcher_paint = model.material("launcher_paint", rgba=(0.34, 0.38, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    optic_black = model.material("optic_black", rgba=(0.08, 0.09, 0.10, 1.0))

    tripod_base = model.part("tripod_base")
    tripod_base.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 0.92)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    apex = (0.0, 0.0, 0.80)
    foot_radius = 0.027
    feet = [
        (0.58, 0.0, foot_radius),
        (-0.29, 0.50, foot_radius),
        (-0.29, -0.50, foot_radius),
    ]

    tripod_base.visual(
        Cylinder(radius=0.055, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.8425)),
        material=tripod_paint,
        name="apex_housing",
    )
    tripod_base.visual(
        Cylinder(radius=0.032, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.849)),
        material=dark_steel,
        name="mast_post",
    )
    tripod_base.visual(
        Cylinder(radius=0.072, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
        material=dark_steel,
        name="yaw_seat",
    )
    tripod_base.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=tripod_paint,
        name="spreader_hub",
    )

    for index, foot in enumerate(feet):
        _add_member(
            tripod_base,
            f"leg_{index}",
            apex,
            foot,
            radius=0.018,
            material=tripod_paint,
        )
        tripod_base.visual(
            Cylinder(radius=foot_radius, length=0.024),
            origin=Origin(xyz=(foot[0], foot[1], 0.012)),
            material=black_rubber,
            name=f"foot_pad_{index}",
        )
        brace_anchor = (
            apex[0] + (foot[0] - apex[0]) * 0.57,
            apex[1] + (foot[1] - apex[1]) * 0.57,
            apex[2] + (foot[2] - apex[2]) * 0.57,
        )
        _add_member(
            tripod_base,
            f"brace_{index}",
            (0.0, 0.0, 0.440),
            brace_anchor,
            radius=0.010,
            material=dark_steel,
        )

    yaw_head = model.part("yaw_head")
    yaw_head.inertial = Inertial.from_geometry(
        Box((0.18, 0.24, 0.18)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )
    yaw_head.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="yaw_plate",
    )
    yaw_head.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=dark_steel,
        name="yaw_stem",
    )
    yaw_head.visual(
        Box((0.026, 0.060, 0.070)),
        origin=Origin(xyz=(-0.040, 0.0, 0.075)),
        material=tripod_paint,
        name="head_core",
    )
    yaw_head.visual(
        Box((0.016, 0.056, 0.024)),
        origin=Origin(xyz=(-0.030, -0.058, 0.075)),
        material=tripod_paint,
        name="left_core_beam",
    )
    yaw_head.visual(
        Box((0.016, 0.056, 0.024)),
        origin=Origin(xyz=(-0.030, 0.058, 0.075)),
        material=tripod_paint,
        name="right_core_beam",
    )

    for side_name, y in (("left", -0.084), ("right", 0.084)):
        yaw_head.visual(
            Box((0.038, 0.016, 0.016)),
            origin=Origin(xyz=(-0.002, y, 0.103)),
            material=tripod_paint,
            name=f"{side_name}_clip_upper",
        )
        yaw_head.visual(
            Box((0.038, 0.016, 0.016)),
            origin=Origin(xyz=(-0.002, y, 0.047)),
            material=tripod_paint,
            name=f"{side_name}_clip_lower",
        )
        yaw_head.visual(
            Box((0.014, 0.016, 0.050)),
            origin=Origin(xyz=(-0.027, y, 0.075)),
            material=dark_steel,
            name=f"{side_name}_clip_bridge",
        )

    tube_outer_profile = [
        (0.090, 0.000),
        (0.090, 0.070),
        (0.083, 0.170),
        (0.080, 0.640),
        (0.078, 1.070),
        (0.082, 1.190),
        (0.082, 1.240),
    ]
    tube_inner_profile = [
        (0.072, 0.006),
        (0.072, 0.150),
        (0.068, 0.640),
        (0.066, 1.090),
        (0.070, 1.234),
    ]
    tube_shell_mesh = _save_mesh(
        "launcher_tube_shell",
        LatheGeometry.from_shell_profiles(
            tube_outer_profile,
            tube_inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )

    launcher_tube = model.part("launcher_tube")
    launcher_tube.inertial = Inertial.from_geometry(
        Box((1.28, 0.30, 0.38)),
        mass=18.0,
        origin=Origin(xyz=(0.28, 0.0, 0.190)),
    )
    launcher_tube.visual(
        Box((0.040, 0.038, 0.030)),
        origin=Origin(xyz=(0.020, -0.064, 0.000)),
        material=dark_steel,
        name="left_trunnion_lug",
    )
    launcher_tube.visual(
        Box((0.040, 0.038, 0.030)),
        origin=Origin(xyz=(0.020, 0.064, 0.000)),
        material=dark_steel,
        name="right_trunnion_lug",
    )
    launcher_tube.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    launcher_tube.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    launcher_tube.visual(
        Box((0.090, 0.090, 0.060)),
        origin=Origin(xyz=(0.045, 0.0, 0.040)),
        material=tripod_paint,
        name="cradle_pedestal",
    )
    launcher_tube.visual(
        Box((0.180, 0.100, 0.040)),
        origin=Origin(xyz=(0.090, 0.0, 0.085)),
        material=tripod_paint,
        name="saddle_beam",
    )
    launcher_tube.visual(
        Box((0.110, 0.050, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, 0.105)),
        material=tripod_paint,
        name="tube_support_web",
    )
    launcher_tube.visual(
        Cylinder(radius=0.085, length=0.020),
        origin=Origin(xyz=(0.020, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_clamp_band",
    )
    launcher_tube.visual(
        Cylinder(radius=0.083, length=0.020),
        origin=Origin(xyz=(0.105, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_clamp_band",
    )
    launcher_tube.visual(
        tube_shell_mesh,
        origin=Origin(xyz=(-0.340, 0.0, 0.200), rpy=(0.0, 0.0, 0.0)),
        material=launcher_paint,
        name="tube_shell",
    )
    launcher_tube.visual(
        Cylinder(radius=0.094, length=0.110),
        origin=Origin(xyz=(-0.285, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_collar",
    )
    launcher_tube.visual(
        Cylinder(radius=0.086, length=0.040),
        origin=Origin(xyz=(0.880, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="muzzle_ring",
    )
    launcher_tube.visual(
        Box((0.160, 0.045, 0.045)),
        origin=Origin(xyz=(-0.115, -0.100, 0.205)),
        material=optic_black,
        name="sight_housing",
    )
    launcher_tube.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(xyz=(-0.225, -0.100, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_black,
        name="eyepiece",
    )
    launcher_tube.visual(
        Box((0.055, 0.018, 0.026)),
        origin=Origin(xyz=(-0.090, -0.072, 0.205)),
        material=dark_steel,
        name="sight_mount",
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_head,
        child=launcher_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.8,
            lower=math.radians(-10.0),
            upper=math.radians(55.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    yaw_head = object_model.get_part("yaw_head")
    launcher_tube = object_model.get_part("launcher_tube")

    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_axis = object_model.get_articulation("elevation_axis")

    yaw_seat = tripod_base.get_visual("yaw_seat")
    yaw_plate = yaw_head.get_visual("yaw_plate")
    left_clip_bridge = yaw_head.get_visual("left_clip_bridge")
    right_clip_bridge = yaw_head.get_visual("right_clip_bridge")
    left_trunnion = launcher_tube.get_visual("left_trunnion")
    right_trunnion = launcher_tube.get_visual("right_trunnion")
    tube_shell = launcher_tube.get_visual("tube_shell")
    muzzle_ring = launcher_tube.get_visual("muzzle_ring")

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
        "azimuth_axis_is_vertical",
        tuple(azimuth_rotation.axis) == (0.0, 0.0, 1.0),
        f"axis={azimuth_rotation.axis}",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        tuple(elevation_axis.axis) == (0.0, -1.0, 0.0),
        f"axis={elevation_axis.axis}",
    )

    ctx.expect_contact(yaw_head, tripod_base, elem_a=yaw_plate, elem_b=yaw_seat)
    ctx.expect_contact(
        launcher_tube,
        yaw_head,
        elem_a=left_trunnion,
        elem_b=left_clip_bridge,
        contact_tol=1e-5,
    )
    ctx.expect_contact(
        launcher_tube,
        yaw_head,
        elem_a=right_trunnion,
        elem_b=right_clip_bridge,
        contact_tol=1e-5,
    )
    ctx.expect_gap(
        launcher_tube,
        tripod_base,
        axis="z",
        min_gap=0.09,
        positive_elem=tube_shell,
        negative_elem=yaw_seat,
        name="tube_clears_tripod_apex",
    )

    def aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    muzzle_rest = ctx.part_element_world_aabb(launcher_tube, elem=muzzle_ring)
    assert muzzle_rest is not None
    muzzle_rest_center = aabb_center(muzzle_rest)

    with ctx.pose({azimuth_rotation: math.radians(40.0)}):
        muzzle_yawed = ctx.part_element_world_aabb(launcher_tube, elem=muzzle_ring)
        assert muzzle_yawed is not None
        muzzle_yawed_center = aabb_center(muzzle_yawed)
        ctx.check(
            "azimuth_swings_launcher_sideways",
            abs(muzzle_yawed_center[1] - muzzle_rest_center[1]) > 0.45
            and muzzle_yawed_center[0] < muzzle_rest_center[0] - 0.18
            and abs(muzzle_yawed_center[2] - muzzle_rest_center[2]) < 0.02,
            f"rest={muzzle_rest_center}, yawed={muzzle_yawed_center}",
        )

    with ctx.pose({elevation_axis: math.radians(35.0)}):
        muzzle_raised = ctx.part_element_world_aabb(launcher_tube, elem=muzzle_ring)
        assert muzzle_raised is not None
        muzzle_raised_center = aabb_center(muzzle_raised)
        ctx.check(
            "elevation_raises_launcher_muzzle",
            muzzle_raised_center[2] > muzzle_rest_center[2] + 0.45
            and muzzle_raised_center[0] < muzzle_rest_center[0] - 0.12,
            f"rest={muzzle_rest_center}, raised={muzzle_raised_center}",
        )
        ctx.expect_contact(
            launcher_tube,
            yaw_head,
            elem_a=left_trunnion,
            elem_b=left_clip_bridge,
            contact_tol=1e-5,
        )
        ctx.expect_contact(
            launcher_tube,
            yaw_head,
            elem_a=right_trunnion,
            elem_b=right_clip_bridge,
            contact_tol=1e-5,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
