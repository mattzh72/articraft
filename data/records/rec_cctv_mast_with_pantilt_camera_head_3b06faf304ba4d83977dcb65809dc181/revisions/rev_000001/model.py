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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broadcast_camera_column")

    base_paint = model.material("base_paint", rgba=(0.20, 0.21, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    head_black = model.material("head_black", rgba=(0.10, 0.10, 0.11, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.08, 0.12, 0.16, 1.0))

    base_column = model.part("base_column")
    base_column.visual(
        Box((0.58, 0.58, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=base_paint,
        name="base_plate",
    )
    base_column.visual(
        Cylinder(radius=0.055, length=1.42),
        origin=Origin(xyz=(0.0, 0.0, 0.732)),
        material=steel,
        name="pole",
    )
    base_column.visual(
        Cylinder(radius=0.075, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0395)),
        material=steel,
        name="pole_collar",
    )
    base_column.visual(
        Cylinder(radius=0.090, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.451)),
        material=head_black,
        name="pole_cap",
    )
    base_column.inertial = Inertial.from_geometry(
        Box((0.58, 0.58, 1.46)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.085, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=head_black,
        name="pan_bearing",
    )
    pan_yoke.visual(
        Cylinder(radius=0.100, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0665)),
        material=head_black,
        name="pan_motor",
    )
    pan_yoke.visual(
        Box((0.10, 0.30, 0.030)),
        origin=Origin(xyz=(-0.03, 0.0, 0.124)),
        material=head_black,
        name="rear_bridge",
    )
    pan_yoke.visual(
        Box((0.20, 0.020, 0.22)),
        origin=Origin(xyz=(0.08, 0.14, 0.205)),
        material=head_black,
        name="left_arm",
    )
    pan_yoke.visual(
        Box((0.20, 0.020, 0.22)),
        origin=Origin(xyz=(0.08, -0.14, 0.205)),
        material=head_black,
        name="right_arm",
    )
    pan_yoke.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.04, 0.16, 0.215), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=head_black,
        name="left_tilt_motor",
    )
    pan_yoke.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.04, -0.16, 0.215), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=head_black,
        name="right_tilt_motor",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.24, 0.34, 0.32)),
        mass=6.5,
        origin=Origin(xyz=(0.04, 0.0, 0.16)),
    )

    camera_housing = model.part("camera_housing")
    camera_housing.visual(
        Box((0.30, 0.22, 0.17)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=base_paint,
        name="body_shell",
    )
    camera_housing.visual(
        Box((0.10, 0.18, 0.14)),
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        material=head_black,
        name="lens_hood",
    )
    camera_housing.visual(
        Cylinder(radius=0.050, length=0.16),
        origin=Origin(xyz=(0.38, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_barrel",
    )
    camera_housing.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    camera_housing.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, -0.12, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    camera_housing.visual(
        Box((0.05, 0.12, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, -0.07)),
        material=head_black,
        name="underslung_mount",
    )
    camera_housing.inertial = Inertial.from_geometry(
        Box((0.46, 0.22, 0.19)),
        mass=4.2,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
    )

    model.articulation(
        "pole_to_pan",
        ArticulationType.REVOLUTE,
        parent=base_column,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_housing,
        origin=Origin(xyz=(0.04, 0.0, 0.215)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.4,
            lower=math.radians(-40.0),
            upper=math.radians(65.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_column = object_model.get_part("base_column")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_housing = object_model.get_part("camera_housing")
    pole_to_pan = object_model.get_articulation("pole_to_pan")
    pan_to_tilt = object_model.get_articulation("pan_to_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.check(
        "pan_axis_is_vertical",
        tuple(pole_to_pan.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pole_to_pan.axis}",
    )
    ctx.check(
        "tilt_axis_is_horizontal",
        abs(tuple(pan_to_tilt.axis)[0]) < 1e-9
        and abs(abs(tuple(pan_to_tilt.axis)[1]) - 1.0) < 1e-9
        and abs(tuple(pan_to_tilt.axis)[2]) < 1e-9,
        details=f"axis={pan_to_tilt.axis}",
    )

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

    ctx.expect_contact(
        pan_yoke,
        base_column,
        elem_a="pan_bearing",
        elem_b="pole_cap",
        name="pan_bearing_seated_on_pole_cap",
    )
    ctx.expect_contact(
        camera_housing,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left_trunnion_supported_by_left_arm",
    )
    ctx.expect_contact(
        camera_housing,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right_trunnion_supported_by_right_arm",
    )
    ctx.expect_gap(
        camera_housing,
        base_column,
        axis="z",
        min_gap=0.10,
        name="camera_clears_column",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    hood_rest = ctx.part_element_world_aabb(camera_housing, elem="lens_hood")
    assert hood_rest is not None
    hood_rest_center = aabb_center(hood_rest)

    with ctx.pose({pole_to_pan: math.pi / 2.0}):
        hood_panned = ctx.part_element_world_aabb(camera_housing, elem="lens_hood")
        assert hood_panned is not None
        hood_panned_center = aabb_center(hood_panned)
        ctx.check(
            "pan_axis_rotates_camera_about_vertical_column",
            abs(hood_panned_center[1]) > 0.25
            and abs(hood_panned_center[0]) < 0.12
            and abs(hood_panned_center[2] - hood_rest_center[2]) < 0.01,
            details=(
                f"rest_center={hood_rest_center}, panned_center={hood_panned_center}"
            ),
        )

    lens_rest = ctx.part_element_world_aabb(camera_housing, elem="lens_barrel")
    assert lens_rest is not None
    lens_rest_center = aabb_center(lens_rest)

    with ctx.pose({pan_to_tilt: math.radians(45.0)}):
        lens_tilted = ctx.part_element_world_aabb(camera_housing, elem="lens_barrel")
        assert lens_tilted is not None
        lens_tilted_center = aabb_center(lens_tilted)
        ctx.check(
            "tilt_axis_raises_lens_about_horizontal_axis",
            lens_tilted_center[2] > lens_rest_center[2] + 0.12
            and lens_tilted_center[0] < lens_rest_center[0] - 0.07,
            details=(
                f"rest_center={lens_rest_center}, tilted_center={lens_tilted_center}"
            ),
        )
        ctx.expect_contact(
            camera_housing,
            pan_yoke,
            elem_a="left_trunnion",
            elem_b="left_arm",
            name="left_trunnion_remains_supported_when_tilted",
        )
        ctx.expect_contact(
            camera_housing,
            pan_yoke,
            elem_a="right_trunnion",
            elem_b="right_arm",
            name="right_trunnion_remains_supported_when_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
