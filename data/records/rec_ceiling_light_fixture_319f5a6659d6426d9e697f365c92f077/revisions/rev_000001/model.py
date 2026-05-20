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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_shade_mesh(*, rest_pitch: float) -> object:
    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0, 0.0),
            (0.018, 0.002),
            (0.046, 0.008),
            (0.050, 0.018),
            (0.050, 0.140),
            (0.052, 0.146),
        ],
        [
            (0.0, 0.006),
            (0.016, 0.007),
            (0.041, 0.014),
            (0.046, 0.020),
            (0.046, 0.139),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    shade_shell.rotate_y(math.pi / 2.0).translate(0.016, 0.0, 0.0)

    trunnion_rod = CylinderGeometry(
        radius=0.0045,
        height=0.100,
        radial_segments=28,
    ).rotate_x(math.pi / 2.0)

    rear_hub = CylinderGeometry(
        radius=0.017,
        height=0.032,
        radial_segments=24,
    ).rotate_y(math.pi / 2.0).translate(0.012, 0.0, 0.0)

    shade_shell.merge(trunnion_rod)
    shade_shell.merge(rear_hub)
    shade_shell.rotate_y(rest_pitch)
    return mesh_from_geometry(shade_shell, "tilting_shade")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_boom_ceiling_light")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    soft_white = model.material("soft_white", rgba=(0.93, 0.92, 0.89, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=0.085, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=charcoal,
        name="canopy_shell",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.026),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    boom_arm = model.part("boom_arm")
    boom_arm.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=charcoal,
        name="pivot_collar",
    )
    boom_arm.visual(
        Box((0.066, 0.020, 0.014)),
        origin=Origin(xyz=(0.051, 0.0, -0.014)),
        material=charcoal,
        name="arm_knuckle",
    )
    boom_arm.visual(
        Cylinder(radius=0.011, length=0.566),
        origin=Origin(xyz=(0.367, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="boom_tube",
    )
    boom_arm.visual(
        Box((0.016, 0.116, 0.018)),
        origin=Origin(xyz=(0.658, 0.0, -0.006)),
        material=charcoal,
        name="yoke_block",
    )
    boom_arm.visual(
        Box((0.020, 0.014, 0.028)),
        origin=Origin(xyz=(0.676, 0.057, -0.006)),
        material=charcoal,
        name="yoke_ear_left",
    )
    boom_arm.visual(
        Box((0.020, 0.014, 0.028)),
        origin=Origin(xyz=(0.676, -0.057, -0.006)),
        material=charcoal,
        name="yoke_ear_right",
    )
    boom_arm.inertial = Inertial.from_geometry(
        Box((0.698, 0.116, 0.050)),
        mass=1.4,
        origin=Origin(xyz=(0.349, 0.0, -0.010)),
    )

    rest_pitch = -0.35
    shade = model.part("shade")
    shade.visual(
        _build_shade_mesh(rest_pitch=rest_pitch),
        material=soft_white,
        name="shade_body",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.160, 0.126, 0.126)),
        mass=0.8,
        origin=Origin(xyz=(0.072, 0.0, -0.020)),
    )

    model.articulation(
        "canopy_to_boom",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=boom_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-2.3,
            upper=2.3,
        ),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom_arm,
        child=shade,
        origin=Origin(xyz=(0.690, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=-1.20,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    boom_arm = object_model.get_part("boom_arm")
    shade = object_model.get_part("shade")
    canopy_to_boom = object_model.get_articulation("canopy_to_boom")
    boom_to_shade = object_model.get_articulation("boom_to_shade")

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
        canopy,
        boom_arm,
        elem_a="canopy_shell",
        elem_b="pivot_collar",
        name="canopy contacts boom collar",
    )
    ctx.expect_contact(boom_arm, shade, name="boom yoke contacts shade trunnion")
    ctx.expect_gap(
        canopy,
        boom_arm,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem="canopy_shell",
        negative_elem="pivot_collar",
        name="boom collar sits directly below canopy",
    )

    boom_axis_ok = all(
        abs(component - expected) < 1e-9
        for component, expected in zip(canopy_to_boom.axis, (0.0, 0.0, 1.0))
    )
    ctx.check(
        "boom articulation axis is vertical",
        boom_axis_ok,
        details=f"Expected (0, 0, 1), got {canopy_to_boom.axis}",
    )
    shade_axis_ok = all(
        abs(component - expected) < 1e-9
        for component, expected in zip(boom_to_shade.axis, (0.0, 1.0, 0.0))
    )
    ctx.check(
        "shade articulation axis is horizontal",
        shade_axis_ok,
        details=f"Expected (0, 1, 0), got {boom_to_shade.axis}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    assert rest_shade_pos is not None
    with ctx.pose({canopy_to_boom: math.pi / 2.0}):
        swung_shade_pos = ctx.part_world_position(shade)
        assert swung_shade_pos is not None
        ctx.expect_contact(canopy, boom_arm, name="boom remains seated when rotated")
        ctx.expect_contact(boom_arm, shade, name="shade stays mounted when boom rotates")
        ctx.check(
            "boom swings shade through the room",
            swung_shade_pos[1] > 0.62 and swung_shade_pos[0] < 0.08,
            details=f"Unexpected swung shade origin {swung_shade_pos}",
        )

    rest_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_body")
    assert rest_shade_aabb is not None
    with ctx.pose({boom_to_shade: 0.70}):
        tilted_shade_aabb = ctx.part_element_world_aabb(shade, elem="shade_body")
        assert tilted_shade_aabb is not None
        ctx.expect_contact(boom_arm, shade, name="shade stays mounted when tilted")
        ctx.check(
            "shade tips downward",
            tilted_shade_aabb[0][2] < rest_shade_aabb[0][2] - 0.045,
            details=(
                f"Rest min z {rest_shade_aabb[0][2]:.4f}, "
                f"tilted min z {tilted_shade_aabb[0][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
