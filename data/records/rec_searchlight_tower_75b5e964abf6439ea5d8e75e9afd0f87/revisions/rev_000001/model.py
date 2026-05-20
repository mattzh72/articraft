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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_searchlight_tower")

    graphite = model.material("graphite", rgba=(0.15, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    anodized = model.material("anodized_black", rgba=(0.11, 0.12, 0.13, 1.0))
    datum = model.material("datum_amber", rgba=(0.84, 0.55, 0.16, 1.0))
    lens = model.material("dark_glass", rgba=(0.18, 0.21, 0.25, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.72, 0.72, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=graphite,
        name="base_plate",
    )
    tower.visual(
        Box((0.28, 0.24, 0.92)),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=graphite,
        name="mast",
    )
    tower.visual(
        Box((0.34, 0.30, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
        material=graphite,
        name="upper_mast_block",
    )
    tower.visual(
        Box((0.46, 0.40, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.11)),
        material=steel,
        name="top_deck",
    )
    tower.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.165)),
        material=steel,
        name="pan_seat",
    )
    tower.visual(
        Box((0.16, 0.08, 0.012)),
        origin=Origin(xyz=(0.15, 0.0, 1.156)),
        material=datum,
        name="pan_datum_pad",
    )
    tower.visual(
        Box((0.012, 0.03, 0.028)),
        origin=Origin(xyz=(0.194, 0.0, 1.176)),
        material=steel,
        name="pan_index_pointer",
    )
    tower.visual(
        Box((0.14, 0.08, 0.012)),
        origin=Origin(xyz=(0.171, 0.0, 0.056)),
        material=steel,
        name="front_datum_pad",
    )
    tower.visual(
        Cylinder(radius=0.04, length=0.028),
        origin=Origin(xyz=(0.25, 0.25, 0.011)),
        material=steel,
        name="front_left_level_foot",
    )
    tower.visual(
        Cylinder(radius=0.04, length=0.028),
        origin=Origin(xyz=(0.25, -0.25, 0.011)),
        material=steel,
        name="front_right_level_foot",
    )
    tower.visual(
        Cylinder(radius=0.04, length=0.028),
        origin=Origin(xyz=(-0.25, 0.25, 0.011)),
        material=steel,
        name="rear_left_level_foot",
    )
    tower.visual(
        Cylinder(radius=0.04, length=0.028),
        origin=Origin(xyz=(-0.25, -0.25, 0.011)),
        material=steel,
        name="rear_right_level_foot",
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=steel,
        name="turntable",
    )
    pan_stage.visual(
        Box((0.22, 0.18, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=graphite,
        name="pan_pedestal",
    )
    pan_stage.visual(
        Box((0.18, 0.52, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=graphite,
        name="lower_yoke_bridge",
    )
    pan_stage.visual(
        Box((0.08, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, 0.29, 0.55)),
        material=graphite,
        name="left_arm",
    )
    pan_stage.visual(
        Box((0.08, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.29, 0.205)),
        material=graphite,
        name="left_arm_lower",
    )
    pan_stage.visual(
        Box((0.08, 0.10, 0.24)),
        origin=Origin(xyz=(0.0, -0.29, 0.55)),
        material=graphite,
        name="right_arm",
    )
    pan_stage.visual(
        Box((0.08, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, -0.29, 0.205)),
        material=graphite,
        name="right_arm_lower",
    )
    pan_stage.visual(
        Box((0.14, 0.58, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=graphite,
        name="top_bridge",
    )
    pan_stage.visual(
        Box((0.10, 0.09, 0.14)),
        origin=Origin(xyz=(0.0, 0.33, 0.36)),
        material=steel,
        name="left_bearing_block",
    )
    pan_stage.visual(
        Box((0.10, 0.09, 0.14)),
        origin=Origin(xyz=(0.0, -0.33, 0.36)),
        material=steel,
        name="right_bearing_block",
    )
    pan_stage.visual(
        Box((0.08, 0.02, 0.008)),
        origin=Origin(xyz=(0.17, 0.0, 0.044)),
        material=datum,
        name="pan_index_bar",
    )
    pan_stage.visual(
        Box((0.012, 0.02, 0.03)),
        origin=Origin(xyz=(0.194, 0.0, 0.063)),
        material=steel,
        name="pan_index_blade",
    )
    pan_stage.visual(
        Box((0.05, 0.008, 0.18)),
        origin=Origin(xyz=(0.028, 0.379, 0.36)),
        material=datum,
        name="tilt_scale_plate",
    )

    head = model.part("spotlight_head")
    head.visual(
        Cylinder(radius=0.19, length=0.38),
        origin=Origin(xyz=(0.17, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=anodized,
        name="barrel",
    )
    head.visual(
        Cylinder(radius=0.21, length=0.05),
        origin=Origin(xyz=(0.385, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.175, length=0.01),
        origin=Origin(xyz=(0.415, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens,
        name="lens_disc",
    )
    head.visual(
        Box((0.18, 0.24, 0.18)),
        origin=Origin(xyz=(-0.10, 0.0, 0.0)),
        material=graphite,
        name="rear_housing",
    )
    head.visual(
        Box((0.18, 0.12, 0.035)),
        origin=Origin(xyz=(-0.07, 0.0, 0.1075)),
        material=steel,
        name="service_box",
    )
    head.visual(
        Box((0.26, 0.10, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, -0.13)),
        material=steel,
        name="lower_datum_rail",
    )
    head.visual(
        Box((0.06, 0.02, 0.10)),
        origin=Origin(xyz=(-0.08, 0.13, 0.0)),
        material=steel,
        name="left_side_datum_pad",
    )
    head.visual(
        Box((0.06, 0.02, 0.10)),
        origin=Origin(xyz=(-0.08, -0.13, 0.0)),
        material=steel,
        name="right_side_datum_pad",
    )
    head.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, 0.235, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, -0.235, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    head.visual(
        Box((0.04, 0.12, 0.03)),
        origin=Origin(xyz=(-0.06, 0.19, 0.045)),
        material=datum,
        name="tilt_pointer_fin",
    )

    model.articulation(
        "tower_to_pan_stage",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-2.3,
            upper=2.3,
        ),
    )
    model.articulation(
        "pan_stage_to_head",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.7,
            lower=-0.35,
            upper=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_stage = object_model.get_part("pan_stage")
    head = object_model.get_part("spotlight_head")
    pan = object_model.get_articulation("tower_to_pan_stage")
    tilt = object_model.get_articulation("pan_stage_to_head")

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
        tuple(pan.axis) == (0.0, 0.0, 1.0),
        f"expected vertical pan axis, got {pan.axis}",
    )
    ctx.check(
        "tilt_axis_is_cross_yoke",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        f"expected horizontal tilt axis through yoke blocks, got {tilt.axis}",
    )

    ctx.expect_contact(
        pan_stage,
        tower,
        elem_a="turntable",
        elem_b="pan_seat",
        name="turntable_seats_on_pan_bearing",
    )
    ctx.expect_contact(
        head,
        pan_stage,
        elem_a="left_trunnion",
        elem_b="left_bearing_block",
        name="left_trunnion_carried_by_left_bearing_block",
    )
    ctx.expect_contact(
        head,
        pan_stage,
        elem_a="right_trunnion",
        elem_b="right_bearing_block",
        name="right_trunnion_carried_by_right_bearing_block",
    )
    ctx.expect_gap(
        pan_stage,
        head,
        axis="y",
        positive_elem="left_arm",
        negative_elem="barrel",
        min_gap=0.035,
        max_gap=0.08,
        name="left_yoke_clearance_is_controlled",
    )
    ctx.expect_gap(
        head,
        pan_stage,
        axis="y",
        positive_elem="barrel",
        negative_elem="right_arm",
        min_gap=0.035,
        max_gap=0.08,
        name="right_yoke_clearance_is_controlled",
    )
    ctx.expect_gap(
        pan_stage,
        tower,
        axis="z",
        positive_elem="turntable",
        negative_elem="pan_seat",
        min_gap=0.0,
        max_gap=0.001,
        name="pan_stage_gap_language_is_seated",
    )

    with ctx.pose({tilt: 0.0}):
        bezel_closed = ctx.part_element_world_aabb(head, elem="front_bezel")
    with ctx.pose({tilt: 0.75}):
        bezel_raised = ctx.part_element_world_aabb(head, elem="front_bezel")
    ctx.check(
        "positive_tilt_raises_spotlight_nose",
        bezel_closed is not None
        and bezel_raised is not None
        and aabb_center(bezel_raised)[2] > aabb_center(bezel_closed)[2] + 0.18,
        "positive tilt should lift the front bezel for repeatable elevation calibration",
    )

    with ctx.pose({pan: 0.0}):
        bezel_forward = ctx.part_element_world_aabb(head, elem="front_bezel")
    with ctx.pose({pan: 1.0}):
        bezel_panned = ctx.part_element_world_aabb(head, elem="front_bezel")
    ctx.check(
        "positive_pan_swings_bezel_toward_positive_y",
        bezel_forward is not None
        and bezel_panned is not None
        and aabb_center(bezel_panned)[1] > aabb_center(bezel_forward)[1] + 0.20,
        "positive pan should move the spotlight nose toward +Y around the tower axis",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
