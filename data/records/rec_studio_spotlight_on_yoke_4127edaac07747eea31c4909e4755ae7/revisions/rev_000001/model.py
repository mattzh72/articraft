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


def _head_shell_mesh(name: str):
    outer_profile = [
        (0.018, -0.106),
        (0.048, -0.100),
        (0.078, -0.084),
        (0.082, -0.010),
        (0.082, 0.082),
        (0.086, 0.110),
        (0.092, 0.123),
    ]
    inner_profile = [
        (0.000, -0.094),
        (0.034, -0.091),
        (0.072, -0.072),
        (0.076, -0.004),
        (0.076, 0.090),
        (0.078, 0.116),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ),
        name,
    )


def _front_bezel_mesh(name: str):
    outer_profile = [
        (0.078, -0.009),
        (0.084, -0.006),
        (0.089, 0.003),
        (0.089, 0.018),
        (0.084, 0.025),
        (0.079, 0.028),
    ]
    inner_profile = [
        (0.0765, -0.006),
        (0.0765, 0.022),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_on_yoke")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    hardware = model.material("hardware", rgba=(0.57, 0.59, 0.62, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.82, 0.90, 0.28))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    shell_mesh = _head_shell_mesh("studio_spotlight_head_shell")
    bezel_mesh = _front_bezel_mesh("studio_spotlight_front_bezel")

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.160, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=powder_black,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.148, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="foot_ring",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=powder_black,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=0.017, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=charcoal,
        name="stem",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.332)),
        material=powder_black,
        name="top_spigot",
    )
    stand.visual(
        Box((0.054, 0.006, 0.092)),
        origin=Origin(xyz=(0.0, 0.040, 0.066), rpy=(0.72, 0.0, 0.0)),
        material=powder_black,
        name="front_gusset",
    )
    stand.visual(
        Box((0.054, 0.006, 0.092)),
        origin=Origin(xyz=(0.0, -0.040, 0.066), rpy=(-0.72, 0.0, 0.0)),
        material=powder_black,
        name="rear_gusset",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, 0.346)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.173)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.032, 0.228, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=powder_black,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.032, 0.008, 0.192)),
        origin=Origin(xyz=(0.0, 0.112, -0.018)),
        material=powder_black,
        name="left_arm",
    )
    yoke.visual(
        Box((0.032, 0.008, 0.192)),
        origin=Origin(xyz=(0.0, -0.112, -0.018)),
        material=powder_black,
        name="right_arm",
    )
    yoke.visual(
        Box((0.046, 0.056, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=powder_black,
        name="mount_block",
    )
    yoke.visual(
        Box((0.018, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, 0.028, -0.104)),
        material=powder_black,
        name="left_clamp_cheek",
    )
    yoke.visual(
        Box((0.018, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, -0.028, -0.104)),
        material=powder_black,
        name="right_clamp_cheek",
    )
    yoke.visual(
        Cylinder(radius=0.004, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="clamp_bolt",
    )
    yoke.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.104, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_friction_plate",
    )
    yoke.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, -0.104, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_friction_plate",
    )
    yoke.visual(
        Box((0.020, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, 0.112, 0.0)),
        material=powder_black,
        name="left_side_boss",
    )
    yoke.visual(
        Box((0.020, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, -0.112, 0.0)),
        material=powder_black,
        name="right_side_boss",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.126, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_lock_knob",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, -0.126, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_lock_knob",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.050, 0.274, 0.182)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    head = model.part("head")
    head.visual(
        shell_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="body_shell",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(-0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="rear_cap",
    )
    head.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.0765, length=0.006),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(-0.010, 0.088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(-0.010, -0.088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.242, 0.184, 0.184)),
        mass=2.1,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.FIXED,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.465)),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=math.radians(-50.0),
            upper=math.radians(68.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
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

    ctx.expect_contact(
        yoke,
        stand,
        elem_a="mount_block",
        elem_b="top_spigot",
        name="yoke_mount_block_seats_on_spigot",
    )
    ctx.expect_contact(
        head,
        yoke,
        elem_a="left_trunnion",
        elem_b="left_friction_plate",
        name="left_trunnion_carried_by_yoke",
    )
    ctx.expect_contact(
        head,
        yoke,
        elem_a="right_trunnion",
        elem_b="right_friction_plate",
        name="right_trunnion_carried_by_yoke",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="z",
        positive_elem="body_shell",
        negative_elem="lower_bridge",
        min_gap=0.012,
        max_gap=0.040,
        name="head_clears_yoke_bridge_at_rest",
    )

    def _center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    closed_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        opened_lens_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    closed_z = _center_z(closed_lens_aabb)
    opened_z = _center_z(opened_lens_aabb)
    ctx.check(
        "positive_tilt_raises_front_lens",
        closed_z is not None and opened_z is not None and opened_z > closed_z + 0.055,
        details=f"closed_z={closed_z}, opened_z={opened_z}",
    )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.expect_gap(
            head,
            yoke,
            axis="z",
            positive_elem="rear_cap",
            negative_elem="lower_bridge",
            min_gap=0.004,
            name="down_tilt_rear_cap_still_clears_bridge",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
