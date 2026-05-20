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


PIVOT_X = 0.0
PIVOT_Z = 0.62


def _make_barrel_shell_mesh() -> object:
    outer_profile = [
        (0.00, -0.34),
        (0.10, -0.335),
        (0.20, -0.31),
        (0.30, -0.245),
        (0.34, -0.17),
        (0.33, -0.08),
        (0.31, 0.00),
        (0.29, 0.18),
        (0.26, 0.44),
        (0.24, 0.68),
        (0.27, 0.80),
        (0.24, 0.88),
    ]
    inner_profile = [
        (0.00, -0.23),
        (0.06, -0.225),
        (0.10, -0.18),
        (0.118, -0.02),
        (0.122, 0.24),
        (0.125, 0.58),
        (0.128, 0.80),
        (0.132, 0.88),
    ]
    barrel_geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        lip_samples=8,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(barrel_geom, "bombard_barrel_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_bombard")

    weathered_oak = model.material("weathered_oak", rgba=(0.46, 0.33, 0.20, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.33, 0.22, 0.13, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.22, 0.22, 0.24, 1.0))
    blackened_iron = model.material("blackened_iron", rgba=(0.12, 0.12, 0.13, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.92, 0.14, 0.14)),
        origin=Origin(xyz=(0.02, 0.40, 0.07)),
        material=weathered_oak,
        name="left_runner",
    )
    carriage.visual(
        Box((1.92, 0.14, 0.14)),
        origin=Origin(xyz=(0.02, -0.40, 0.07)),
        material=weathered_oak,
        name="right_runner",
    )
    carriage.visual(
        Box((0.30, 0.92, 0.16)),
        origin=Origin(xyz=(-0.52, 0.0, 0.10)),
        material=dark_oak,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.24, 0.92, 0.16)),
        origin=Origin(xyz=(0.66, 0.0, 0.10)),
        material=dark_oak,
        name="front_transom",
    )
    carriage.visual(
        Box((1.18, 0.62, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.19)),
        material=weathered_oak,
        name="bed_deck",
    )
    carriage.visual(
        Box((1.00, 0.08, 0.08)),
        origin=Origin(xyz=(0.06, 0.29, 0.18)),
        material=dark_oak,
        name="left_inner_rail",
    )
    carriage.visual(
        Box((1.00, 0.08, 0.08)),
        origin=Origin(xyz=(0.06, -0.29, 0.18)),
        material=dark_oak,
        name="right_inner_rail",
    )
    carriage.visual(
        Box((0.08, 0.46, 0.08)),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.259)),
        material=dark_oak,
        name="center_saddle",
    )

    carriage.visual(
        Box((1.22, 0.07, 0.34)),
        origin=Origin(xyz=(0.0, 0.38, 0.37)),
        material=weathered_oak,
        name="left_lower_cheek",
    )
    carriage.visual(
        Box((0.26, 0.07, 0.28)),
        origin=Origin(xyz=(-0.30, 0.38, 0.68)),
        material=weathered_oak,
        name="left_rear_upright",
    )
    carriage.visual(
        Box((0.22, 0.07, 0.22)),
        origin=Origin(xyz=(0.26, 0.38, 0.65)),
        material=weathered_oak,
        name="left_front_upright",
    )
    carriage.visual(
        Box((0.62, 0.07, 0.08)),
        origin=Origin(xyz=(-0.02, 0.38, 0.78)),
        material=weathered_oak,
        name="left_top_rail",
    )
    carriage.visual(
        Box((0.18, 0.07, 0.08)),
        origin=Origin(xyz=(0.0, 0.38, 0.51)),
        material=dark_oak,
        name="left_journal_seat",
    )
    carriage.visual(
        Box((1.10, 0.10, 0.14)),
        origin=Origin(xyz=(0.02, 0.345, 0.17)),
        material=dark_oak,
        name="left_cheek_binder",
    )
    carriage.visual(
        Box((1.22, 0.07, 0.34)),
        origin=Origin(xyz=(0.0, -0.38, 0.37)),
        material=weathered_oak,
        name="right_lower_cheek",
    )
    carriage.visual(
        Box((0.26, 0.07, 0.28)),
        origin=Origin(xyz=(-0.30, -0.38, 0.68)),
        material=weathered_oak,
        name="right_rear_upright",
    )
    carriage.visual(
        Box((0.22, 0.07, 0.22)),
        origin=Origin(xyz=(0.26, -0.38, 0.65)),
        material=weathered_oak,
        name="right_front_upright",
    )
    carriage.visual(
        Box((0.62, 0.07, 0.08)),
        origin=Origin(xyz=(-0.02, -0.38, 0.78)),
        material=weathered_oak,
        name="right_top_rail",
    )
    carriage.visual(
        Box((0.18, 0.07, 0.08)),
        origin=Origin(xyz=(0.0, -0.38, 0.51)),
        material=dark_oak,
        name="right_journal_seat",
    )
    carriage.visual(
        Box((1.10, 0.10, 0.14)),
        origin=Origin(xyz=(0.02, -0.345, 0.17)),
        material=dark_oak,
        name="right_cheek_binder",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.96, 1.02, 0.82)),
        mass=520.0,
        origin=Origin(xyz=(0.02, 0.0, 0.41)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _make_barrel_shell_mesh(),
        material=cast_iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.070, length=0.88),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_iron,
        name="pivot_shaft",
    )
    barrel.visual(
        Cylinder(radius=0.285, length=0.10),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="breech_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.255, length=0.08),
        origin=Origin(xyz=(0.82, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blackened_iron,
        name="muzzle_ring",
    )
    barrel.visual(
        Box((0.10, 0.22, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.309)),
        material=blackened_iron,
        name="support_shoe",
    )
    barrel.visual(
        Box((0.13, 0.14, 0.014)),
        origin=Origin(xyz=(-0.18, 0.0, 0.347)),
        material=cast_iron,
        name="vent_pad",
    )
    barrel.visual(
        Box((0.030, 0.024, 0.026)),
        origin=Origin(xyz=(-0.10, -0.036, 0.357)),
        material=blackened_iron,
        name="left_hinge_ear_block",
    )
    barrel.visual(
        Box((0.030, 0.024, 0.026)),
        origin=Origin(xyz=(-0.10, 0.036, 0.357)),
        material=blackened_iron,
        name="right_hinge_ear_block",
    )
    barrel.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.10, -0.036, 0.370), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_iron,
        name="left_hinge_ear",
    )
    barrel.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.10, 0.036, 0.370), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_iron,
        name="right_hinge_ear",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=1.22),
        mass=1350.0,
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    touch_hole_cover = model.part("touch_hole_cover")
    touch_hole_cover.visual(
        Cylinder(radius=0.012, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_iron,
        name="hinge_knuckle",
    )
    touch_hole_cover.visual(
        Box((0.032, 0.040, 0.024)),
        origin=Origin(xyz=(-0.014, 0.0, -0.004)),
        material=blackened_iron,
        name="hinge_strap",
    )
    touch_hole_cover.visual(
        Box((0.086, 0.108, 0.008)),
        origin=Origin(xyz=(-0.068, 0.0, -0.012)),
        material=cast_iron,
        name="cover_plate",
    )
    touch_hole_cover.inertial = Inertial.from_geometry(
        Box((0.100, 0.110, 0.026)),
        mass=6.0,
        origin=Origin(xyz=(-0.052, 0.0, -0.004)),
    )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.30,
            lower=math.radians(-8.0),
            upper=math.radians(26.0),
        ),
    )
    model.articulation(
        "barrel_to_touch_hole_cover",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=touch_hole_cover,
        origin=Origin(xyz=(-0.10, 0.0, 0.370)),
        # The cover plate lies along local -X when closed, so +Y lifts it upward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    touch_hole_cover = object_model.get_part("touch_hole_cover")
    elevation_joint = object_model.get_articulation("carriage_to_barrel")
    cover_joint = object_model.get_articulation("barrel_to_touch_hole_cover")

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

    with ctx.pose({elevation_joint: 0.0, cover_joint: 0.0}):
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="pivot_shaft",
            elem_b="left_journal_seat",
            name="left trunnion bears on the cradle seat",
        )
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="pivot_shaft",
            elem_b="right_journal_seat",
            name="right trunnion bears on the cradle seat",
        )
        ctx.expect_contact(
            touch_hole_cover,
            barrel,
            elem_a="cover_plate",
            elem_b="vent_pad",
            name="touch-hole cover seats on the breech vent pad",
        )

    rest_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_ring")
    with ctx.pose({elevation_joint: elevation_joint.motion_limits.upper, cover_joint: 0.0}):
        raised_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_ring")
    ctx.check(
        "barrel elevation raises the muzzle",
        rest_muzzle_aabb is not None
        and raised_muzzle_aabb is not None
        and raised_muzzle_aabb[1][2] > rest_muzzle_aabb[1][2] + 0.14,
        details=f"rest={rest_muzzle_aabb}, raised={raised_muzzle_aabb}",
    )

    rest_cover_aabb = ctx.part_element_world_aabb(touch_hole_cover, elem="cover_plate")
    with ctx.pose({elevation_joint: 0.0, cover_joint: cover_joint.motion_limits.upper}):
        opened_cover_aabb = ctx.part_element_world_aabb(touch_hole_cover, elem="cover_plate")
    ctx.check(
        "touch-hole cover opens upward",
        rest_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.05,
        details=f"rest={rest_cover_aabb}, opened={opened_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
