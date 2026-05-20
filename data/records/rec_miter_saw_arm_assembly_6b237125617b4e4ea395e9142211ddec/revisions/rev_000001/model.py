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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jobsite_miter_saw")

    cast_metal = model.material("cast_metal", rgba=(0.27, 0.29, 0.31, 1.0))
    machined_metal = model.material("machined_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    black = model.material("black_plastic", rgba=(0.10, 0.10, 0.10, 1.0))
    guard_orange = model.material("guard_orange", rgba=(0.88, 0.48, 0.15, 1.0))
    accent_red = model.material("accent_red", rgba=(0.74, 0.10, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.40, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_metal,
        name="elem_base_deck",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=machined_metal,
        name="elem_turntable_pedestal",
    )
    base.visual(
        Box((0.22, 0.028, 0.12)),
        origin=Origin(xyz=(-0.16, -0.185, 0.110)),
        material=machined_metal,
        name="elem_fence_left",
    )
    base.visual(
        Box((0.22, 0.028, 0.12)),
        origin=Origin(xyz=(0.16, -0.185, 0.110)),
        material=machined_metal,
        name="elem_fence_right",
    )
    base.visual(
        Box((0.018, 0.130, 0.050)),
        origin=Origin(xyz=(-0.319, 0.0, 0.075)),
        material=cast_metal,
        name="elem_wing_hinge_bracket",
    )
    for name, y_center in (
        ("elem_wing_knuckle_front", -0.035),
        ("elem_wing_knuckle_rear", 0.035),
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(-0.340, y_center, 0.084), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=machined_metal,
            name=name,
        )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.16, length=0.018),
        origin=Origin(),
        material=machined_metal,
        name="elem_turntable_disc",
    )
    table.visual(
        Box((0.016, 0.180, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=black,
        name="elem_insert_slot",
    )
    table.visual(
        Box((0.028, 0.032, 0.139)),
        origin=Origin(xyz=(-0.055, -0.092, 0.0785)),
        material=cast_metal,
        name="elem_pivot_post_left",
    )
    table.visual(
        Box((0.028, 0.032, 0.139)),
        origin=Origin(xyz=(0.055, -0.092, 0.0785)),
        material=cast_metal,
        name="elem_pivot_post_right",
    )
    table.visual(
        Box((0.030, 0.028, 0.016)),
        origin=Origin(xyz=(-0.055, -0.092, 0.156)),
        material=cast_metal,
        name="elem_pivot_cap_left",
    )
    table.visual(
        Box((0.030, 0.028, 0.016)),
        origin=Origin(xyz=(0.055, -0.092, 0.156)),
        material=cast_metal,
        name="elem_pivot_cap_right",
    )
    table.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(-0.046, -0.092, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="elem_arm_knuckle_left",
    )
    table.visual(
        Cylinder(radius=0.017, length=0.028),
        origin=Origin(xyz=(0.046, -0.092, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="elem_arm_knuckle_right",
    )
    table.visual(
        Box((0.024, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.170, 0.016)),
        material=black,
        name="elem_miter_handle",
    )

    guard_profile = [
        (-0.100, -0.010),
        (-0.110, 0.045),
        (-0.088, 0.110),
        (-0.038, 0.160),
        (0.045, 0.164),
        (0.105, 0.115),
        (0.126, 0.040),
        (0.108, -0.012),
        (0.072, -0.045),
        (-0.012, -0.052),
    ]
    guard_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(guard_profile, 0.030),
        "upper_blade_guard",
    )
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.120, 0.108),
                (0.0, 0.110, 0.155),
                (0.0, 0.170, 0.182),
                (0.0, 0.235, 0.160),
                (0.0, 0.280, 0.122),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "saw_handle_loop",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.0155, length=0.064),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="elem_arm_barrel",
    )
    arm.visual(
        Box((0.050, 0.096, 0.038)),
        origin=Origin(xyz=(0.0, 0.050, 0.010)),
        material=guard_orange,
        name="elem_arm_link",
    )
    arm.visual(
        Box((0.068, 0.150, 0.050)),
        origin=Origin(xyz=(0.0, 0.155, 0.015)),
        material=guard_orange,
        name="elem_arm_beam",
    )
    arm.visual(
        guard_mesh,
        origin=Origin(xyz=(0.0, 0.210, -0.068), rpy=(0.0, pi / 2.0, 0.0)),
        material=guard_orange,
        name="elem_guard",
    )
    arm.visual(
        Cylinder(radius=0.127, length=0.004),
        origin=Origin(xyz=(0.0, 0.210, -0.068), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="elem_blade",
    )
    arm.visual(
        Box((0.094, 0.080, 0.090)),
        origin=Origin(xyz=(0.080, 0.175, -0.030)),
        material=black,
        name="elem_motor",
    )
    arm.visual(
        Box((0.028, 0.150, 0.040)),
        origin=Origin(xyz=(0.0, 0.195, 0.108)),
        material=black,
        name="elem_handle_spine",
    )
    arm.visual(
        Box((0.040, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, 0.175, 0.070)),
        material=black,
        name="elem_handle_mount",
    )
    arm.visual(
        handle_mesh,
        material=black,
        name="elem_handle",
    )
    arm.visual(
        Box((0.030, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.262, 0.116)),
        material=accent_red,
        name="elem_trigger_grip",
    )

    wing = model.part("wing")
    wing.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined_metal,
        name="elem_wing_knuckle",
    )
    wing.visual(
        Box((0.220, 0.125, 0.018)),
        origin=Origin(xyz=(-0.120, 0.0, 0.0)),
        material=machined_metal,
        name="elem_wing_deck",
    )
    wing.visual(
        Box((0.060, 0.100, 0.042)),
        origin=Origin(xyz=(-0.060, 0.0, -0.024)),
        material=cast_metal,
        name="elem_wing_brace",
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-0.80,
            upper=0.80,
        ),
    )
    model.articulation(
        "table_to_arm",
        ArticulationType.REVOLUTE,
        parent=table,
        child=arm,
        origin=Origin(xyz=(0.0, -0.092, 0.115), rpy=(0.75, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=-0.42,
            upper=0.35,
        ),
    )
    model.articulation(
        "base_to_wing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing,
        origin=Origin(xyz=(-0.340, 0.0, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.6,
            lower=-1.35,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    table = object_model.get_part("table")
    arm = object_model.get_part("arm")
    wing = object_model.get_part("wing")
    table_joint = object_model.get_articulation("base_to_table")
    arm_joint = object_model.get_articulation("table_to_arm")
    wing_joint = object_model.get_articulation("base_to_wing")

    ctx.check(
        "all primary parts exist",
        all(part is not None for part in (base, table, arm, wing)),
        details="Expected base, table, arm, and wing parts.",
    )
    ctx.check(
        "all primary articulations exist",
        all(joint is not None for joint in (table_joint, arm_joint, wing_joint)),
        details="Expected table, chop arm, and support wing articulations.",
    )

    ctx.expect_gap(
        table,
        base,
        axis="z",
        positive_elem="elem_turntable_disc",
        negative_elem="elem_turntable_pedestal",
        min_gap=-0.0005,
        max_gap=0.0015,
        name="turntable seats on the base pedestal",
    )
    ctx.expect_overlap(
        table,
        base,
        axes="xy",
        elem_a="elem_turntable_disc",
        elem_b="elem_turntable_pedestal",
        min_overlap=0.18,
        name="turntable stays centered over the pedestal",
    )

    arm_rest = ctx.part_world_position(arm)
    with ctx.pose({table_joint: 0.60}):
        arm_swiveled = ctx.part_world_position(arm)
    ctx.check(
        "table rotation swings the saw body around the vertical axis",
        arm_rest is not None
        and arm_swiveled is not None
        and arm_swiveled[0] > arm_rest[0] + 0.04,
        details=f"rest={arm_rest}, swiveled={arm_swiveled}",
    )

    def elem_center_z(part_obj, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    blade_rest_z = elem_center_z(arm, "elem_blade")
    with ctx.pose({arm_joint: -0.34}):
        blade_lowered_z = elem_center_z(arm, "elem_blade")
        ctx.expect_gap(
            arm,
            table,
            axis="z",
            positive_elem="elem_blade",
            negative_elem="elem_turntable_disc",
            min_gap=0.0,
            max_gap=0.018,
            name="lowered blade approaches the turntable without interpenetration",
        )
    ctx.check(
        "chop arm lowers the blade toward the work table",
        blade_rest_z is not None
        and blade_lowered_z is not None
        and blade_lowered_z < blade_rest_z - 0.06,
        details=f"rest_z={blade_rest_z}, lowered_z={blade_lowered_z}",
    )

    wing_rest_z = elem_center_z(wing, "elem_wing_deck")
    with ctx.pose({wing_joint: -1.20}):
        wing_folded_z = elem_center_z(wing, "elem_wing_deck")
    ctx.check(
        "support wing folds down beside the base",
        wing_rest_z is not None
        and wing_folded_z is not None
        and wing_folded_z < wing_rest_z - 0.08,
        details=f"rest_z={wing_rest_z}, folded_z={wing_folded_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
