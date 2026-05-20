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
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arm_plate_mesh():
    profile = [
        (-0.11, 0.00),
        (0.02, 0.00),
        (0.07, 0.24),
        (0.06, 0.58),
        (0.02, 0.82),
        (-0.08, 0.94),
        (-0.18, 0.90),
        (-0.16, 0.18),
    ]
    thickness = 0.028
    arm = ExtrudeGeometry.from_z0(profile, thickness)
    arm.rotate_x(math.pi / 2.0)
    arm.translate(0.0, -0.5 * thickness, 0.0)
    return mesh_from_geometry(arm, "yoke_arm_plate")


def _head_shell_mesh():
    outer_profile = [
        (0.15, -0.28),
        (0.18, -0.18),
        (0.20, -0.08),
        (0.21, 0.02),
        (0.24, 0.18),
        (0.26, 0.34),
        (0.25, 0.44),
    ]
    inner_profile = [
        (0.00, -0.26),
        (0.10, -0.18),
        (0.14, -0.08),
        (0.165, 0.02),
        (0.195, 0.18),
        (0.215, 0.34),
        (0.223, 0.435),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, "searchlight_head_shell")


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
    model = ArticulatedObject(name="cost_optimized_searchlight_tower")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.64, 0.67, 0.70, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.33, 0.36, 0.40, 1.0))
    head_paint = model.material("head_paint", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.14, 0.15, 0.17, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.63, 0.73, 0.84, 0.45))

    arm_plate_mesh = _arm_plate_mesh()
    head_shell_mesh = _head_shell_mesh()

    tower = model.part("tower_support")
    tower.visual(
        Box((1.34, 1.34, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_hardware,
        name="anchor_flange",
    )
    tower.visual(
        Box((1.18, 1.18, 0.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=machinery_gray,
        name="ballast_cabinet",
    )
    tower.visual(
        Box((0.82, 0.03, 0.30)),
        origin=Origin(xyz=(0.0, 0.575, 0.28)),
        material=dark_hardware,
        name="service_door",
    )
    tower.visual(
        Box((0.24, 0.24, 3.56)),
        origin=Origin(xyz=(0.0, 0.0, 2.26)),
        material=galvanized_steel,
        name="mast_tube",
    )
    tower.visual(
        Box((0.42, 0.42, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 4.07)),
        material=machinery_gray,
        name="top_bearing_plate",
    )
    tower.visual(
        Box((0.12, 0.30, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 3.94)),
        material=dark_hardware,
        name="pan_drive_box",
    )

    yoke = model.part("pan_yoke")
    yoke.visual(
        Cylinder(radius=0.22, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_hardware,
        name="turntable_ring",
    )
    yoke.visual(
        Cylinder(radius=0.16, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=machinery_gray,
        name="slew_drum",
    )
    yoke.visual(
        Box((0.22, 0.30, 0.16)),
        origin=Origin(xyz=(-0.02, 0.0, 0.18)),
        material=dark_hardware,
        name="pan_gearbox",
    )
    yoke.visual(
        Box((0.18, 0.82, 0.12)),
        origin=Origin(xyz=(-0.05, 0.0, 0.24)),
        material=machinery_gray,
        name="lower_crossmember",
    )
    yoke.visual(
        Box((0.16, 0.26, 0.28)),
        origin=Origin(xyz=(-0.06, 0.0, 0.38)),
        material=machinery_gray,
        name="center_column",
    )
    yoke.visual(
        arm_plate_mesh,
        origin=Origin(xyz=(0.0, 0.34, 0.18)),
        material=galvanized_steel,
        name="left_arm",
    )
    yoke.visual(
        arm_plate_mesh,
        origin=Origin(xyz=(0.0, -0.34, 0.18)),
        material=galvanized_steel,
        name="right_arm",
    )
    yoke.visual(
        Box((0.14, 0.76, 0.08)),
        origin=Origin(xyz=(-0.16, 0.0, 1.08)),
        material=machinery_gray,
        name="top_tie_bar",
    )
    yoke.visual(
        Box((0.14, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, 0.33, 0.84)),
        material=dark_hardware,
        name="left_bearing_block",
    )
    yoke.visual(
        Box((0.14, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, -0.33, 0.84)),
        material=dark_hardware,
        name="right_bearing_block",
    )
    yoke.visual(
        Cylinder(radius=0.075, length=0.09),
        origin=Origin(xyz=(0.0, 0.285, 0.84), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="left_bearing_boss",
    )
    yoke.visual(
        Cylinder(radius=0.075, length=0.09),
        origin=Origin(xyz=(0.0, -0.285, 0.84), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="right_bearing_boss",
    )

    head = model.part("spotlight_head")
    head.visual(
        head_shell_mesh,
        material=head_paint,
        name="head_shell",
    )
    head.visual(
        Box((0.22, 0.44, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machinery_gray,
        name="trunnion_band",
    )
    head.visual(
        Cylinder(radius=0.055, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="trunnion_tube",
    )
    head.visual(
        Box((0.18, 0.18, 0.12)),
        origin=Origin(xyz=(-0.18, 0.0, -0.04)),
        material=dark_hardware,
        name="rear_service_housing",
    )
    head.visual(
        Cylinder(radius=0.26, length=0.03),
        origin=Origin(xyz=(0.432, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.223, length=0.008),
        origin=Origin(xyz=(0.434, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )

    model.articulation(
        "tower_pan",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 4.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.55),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.60,
            lower=-0.20,
            upper=0.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower_support")
    yoke = object_model.get_part("pan_yoke")
    head = object_model.get_part("spotlight_head")
    pan = object_model.get_articulation("tower_pan")
    tilt = object_model.get_articulation("yoke_tilt")

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
        "pan_axis_is_vertical",
        tuple(pan.axis) == (0.0, 0.0, 1.0)
        and pan.articulation_type == ArticulationType.CONTINUOUS,
        details=f"pan axis/type was {pan.axis!r} / {pan.articulation_type!r}",
    )
    ctx.check(
        "tilt_axis_is_pitch",
        tuple(tilt.axis) == (0.0, -1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper,
        details=f"tilt axis/limits were {tilt.axis!r} / {tilt.motion_limits!r}",
    )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        ctx.expect_contact(
            yoke,
            tower,
            contact_tol=0.001,
            name="turntable_seats_on_tower_plate",
        )
        ctx.expect_contact(
            head,
            yoke,
            contact_tol=0.001,
            name="head_trunnions_contact_bearing_blocks",
        )
        ctx.expect_gap(
            head,
            tower,
            axis="z",
            min_gap=0.45,
            name="head_clears_tower_in_rest_pose",
        )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        lens_home = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({pan: 0.0, tilt: 0.55}):
        lens_tilted = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "tilt_raises_lens",
        lens_home is not None
        and lens_tilted is not None
        and lens_tilted[2] > lens_home[2] + 0.14,
        details=f"neutral={lens_home!r} tilted={lens_tilted!r}",
    )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        lens_pan_home = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    with ctx.pose({pan: math.pi / 2.0, tilt: 0.0}):
        lens_pan_quarter = _aabb_center(ctx.part_element_world_aabb(head, elem="front_lens"))
    ctx.check(
        "pan_swings_beam_sideways",
        lens_pan_home is not None
        and lens_pan_quarter is not None
        and abs(lens_pan_home[1]) < 0.05
        and lens_pan_quarter[1] > 0.30
        and abs(lens_pan_quarter[0]) < 0.10,
        details=f"pan0={lens_pan_home!r} pan90={lens_pan_quarter!r}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
