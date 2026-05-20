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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_X = 0.150
BASE_Y = 0.125
BASE_H = 0.028
PIVOT_Z = 0.082

TOWER_X = 0.024
TOWER_Y = 0.046
TOWER_Z = 0.090
TOWER_CENTER_X = 0.063

OUTER_ARM_X = 0.012
OUTER_ARM_Y = 0.036
OUTER_ARM_Z = 0.086
OUTER_ARM_CENTER_X = 0.034
OUTER_HUB_RADIUS = 0.011
OUTER_HUB_LEN = 0.018

BEARING_BAR_Y = 0.006
BEARING_TOPBOT_X = 0.028
BEARING_TOPBOT_Z = 0.006
BEARING_SIDE_X = 0.020
BEARING_SIDE_Z = 0.028
BEARING_OUTER_CENTER_Y = 0.020
BEARING_INNER_CENTER_Y = 0.011

STEM_RADIUS = 0.007
STEM_HEIGHT = 0.145
GRIP_RADIUS = 0.012
GRIP_HEIGHT = 0.040
LOWER_STUB = 0.018
COLLAR_RADIUS = 0.008

OUTER_LIMIT = 0.55
INNER_LIMIT = 0.50
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_joystick_gimbal")

    model.material("base_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_frame", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("inner_frame", rgba=(0.56, 0.60, 0.64, 1.0))
    model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_X, BASE_Y, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material="base_coat",
        name="base_block",
    )
    for x_sign in (-1.0, 1.0):
        base.visual(
            Box((TOWER_X, TOWER_Y, TOWER_Z)),
            origin=Origin(xyz=(x_sign * TOWER_CENTER_X, 0.0, BASE_H + TOWER_Z / 2.0)),
            material="base_coat",
            name=f"{'left' if x_sign < 0 else 'right'}_tower",
        )
        base.visual(
            Box((0.020, TOWER_Y, 0.026)),
            origin=Origin(xyz=(x_sign * 0.066, 0.0, BASE_H + 0.013)),
            material="base_coat",
            name=f"{'left' if x_sign < 0 else 'right'}_gusset",
        )
        base.visual(
            Cylinder(radius=0.015, length=0.016),
            origin=Origin(xyz=(x_sign * 0.058, 0.0, PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material="machined_frame",
            name=f"{'left' if x_sign < 0 else 'right'}_base_bearing",
        )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        Box((OUTER_ARM_X, OUTER_ARM_Y, OUTER_ARM_Z)),
        origin=Origin(xyz=(-OUTER_ARM_CENTER_X, 0.0, 0.0)),
        material="machined_frame",
        name="left_arm",
    )
    outer_ring.visual(
        Box((OUTER_ARM_X, OUTER_ARM_Y, OUTER_ARM_Z)),
        origin=Origin(xyz=(OUTER_ARM_CENTER_X, 0.0, 0.0)),
        material="machined_frame",
        name="right_arm",
    )
    outer_ring.visual(
        Box((0.056, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material="machined_frame",
        name="bottom_bridge",
    )
    for x_sign in (-1.0, 1.0):
        outer_ring.visual(
            Cylinder(radius=OUTER_HUB_RADIUS, length=OUTER_HUB_LEN),
            origin=Origin(xyz=(x_sign * 0.041, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="machined_frame",
            name=f"{'left' if x_sign < 0 else 'right'}_hub",
        )
    for prefix, y_pos in (("front", BEARING_OUTER_CENTER_Y), ("rear", -BEARING_OUTER_CENTER_Y)):
        outer_ring.visual(
            Box((BEARING_TOPBOT_X, BEARING_BAR_Y, BEARING_TOPBOT_Z)),
            origin=Origin(xyz=(0.0, y_pos, 0.014)),
            material="machined_frame",
            name=f"{prefix}_top_bar",
        )
        outer_ring.visual(
            Box((BEARING_TOPBOT_X, BEARING_BAR_Y, BEARING_TOPBOT_Z)),
            origin=Origin(xyz=(0.0, y_pos, -0.014)),
            material="machined_frame",
            name=f"{prefix}_bottom_bar",
        )
        outer_ring.visual(
            Box((BEARING_SIDE_X, BEARING_BAR_Y, BEARING_SIDE_Z)),
            origin=Origin(xyz=(-0.024, y_pos, 0.0)),
            material="machined_frame",
            name=f"{prefix}_left_bar",
        )
        outer_ring.visual(
            Box((BEARING_SIDE_X, BEARING_BAR_Y, BEARING_SIDE_Z)),
            origin=Origin(xyz=(0.024, y_pos, 0.0)),
            material="machined_frame",
            name=f"{prefix}_right_bar",
        )

    inner_assembly = model.part("inner_assembly")
    inner_assembly.visual(
        Box((0.018, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material="inner_frame",
        name="lower_yoke",
    )
    inner_assembly.visual(
        Box((0.010, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material="inner_frame",
        name="stem_post",
    )
    inner_assembly.visual(
        Cylinder(radius=COLLAR_RADIUS, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="inner_frame",
        name="pivot_collar",
    )
    inner_assembly.visual(
        Box((0.024, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, BEARING_INNER_CENTER_Y, 0.0)),
        material="inner_frame",
        name="front_bearing_block",
    )
    inner_assembly.visual(
        Box((0.024, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -BEARING_INNER_CENTER_Y, 0.0)),
        material="inner_frame",
        name="rear_bearing_block",
    )
    inner_assembly.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, STEM_HEIGHT / 2.0)),
        material="inner_frame",
        name="stick_stem",
    )
    inner_assembly.visual(
        Cylinder(radius=0.006, length=LOWER_STUB),
        origin=Origin(xyz=(0.0, 0.0, -LOWER_STUB / 2.0)),
        material="inner_frame",
        name="lower_stub",
    )
    inner_assembly.visual(
        Cylinder(radius=GRIP_RADIUS, length=GRIP_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, STEM_HEIGHT + GRIP_HEIGHT / 2.0),
            rpy=(0.0, 0.0, 0.0),
        ),
        material="grip_black",
        name="grip",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-OUTER_LIMIT,
            upper=OUTER_LIMIT,
            effort=25.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_assembly,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-INNER_LIMIT,
            upper=INNER_LIMIT,
            effort=18.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_ring = object_model.get_part("outer_ring")
    inner_assembly = object_model.get_part("inner_assembly")
    outer_joint = object_model.get_articulation("base_to_outer")
    inner_joint = object_model.get_articulation("outer_to_inner")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.allow_overlap(
        outer_ring,
        inner_assembly,
        reason="The inner gimbal axis is represented by simplified rectangular bearing blocks captured inside the outer yoke support frame.",
    )
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

    ctx.expect_contact(base, outer_ring, name="base supports the outer gimbal ring")
    ctx.expect_overlap(
        outer_ring,
        inner_assembly,
        axes="y",
        min_overlap=0.006,
        name="outer ring captures the inner gimbal bearing blocks",
    )
    ctx.expect_overlap(
        outer_ring,
        inner_assembly,
        axes="z",
        min_overlap=0.060,
        name="nested gimbals share a substantial vertical working envelope",
    )

    grip_rest = ctx.part_element_world_aabb(inner_assembly, elem="grip")
    with ctx.pose({outer_joint: 0.35}):
        grip_outer = ctx.part_element_world_aabb(inner_assembly, elem="grip")
    with ctx.pose({inner_joint: 0.35}):
        grip_inner = ctx.part_element_world_aabb(inner_assembly, elem="grip")

    def _center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    def _center_y(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    ctx.check(
        "outer gimbal joint tilts grip along y",
        grip_rest is not None
        and grip_outer is not None
        and _center_y(grip_outer) is not None
        and _center_y(grip_rest) is not None
        and _center_y(grip_outer) > _center_y(grip_rest) + 0.020,
        details=f"rest={grip_rest}, outer_tilt={grip_outer}",
    )
    ctx.check(
        "inner gimbal joint tilts grip along x",
        grip_rest is not None
        and grip_inner is not None
        and _center_x(grip_inner) is not None
        and _center_x(grip_rest) is not None
        and _center_x(grip_inner) > _center_x(grip_rest) + 0.020,
        details=f"rest={grip_rest}, inner_tilt={grip_inner}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
