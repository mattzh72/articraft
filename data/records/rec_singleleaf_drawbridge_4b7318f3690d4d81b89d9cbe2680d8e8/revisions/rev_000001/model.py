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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_singleleaf_drawbridge")

    frame_steel = model.material("frame_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    bridge_steel = model.material("bridge_steel", rgba=(0.54, 0.56, 0.58, 1.0))
    datum_finish = model.material("datum_finish", rgba=(0.80, 0.82, 0.84, 1.0))
    safety_mark = model.material("safety_mark", rgba=(0.86, 0.57, 0.16, 1.0))
    adjuster_blue = model.material("adjuster_blue", rgba=(0.26, 0.39, 0.64, 1.0))

    support_frame = model.part("support_frame")
    bridge_leaf = model.part("bridge_leaf")

    axis_height = 0.98
    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    # Fixed support frame: base, pedestals, bearing stack, closed-position stops,
    # and calibration features. Everything is physically tied together.
    support_frame.visual(
        Box((0.96, 1.82, 0.18)),
        origin=Origin(xyz=(0.05, 0.0, 0.09)),
        material=frame_steel,
        name="foundation",
    )
    support_frame.visual(
        Box((0.40, 0.18, 0.66)),
        origin=Origin(xyz=(-0.04, -0.77, 0.51)),
        material=frame_steel,
        name="left_pedestal",
    )
    support_frame.visual(
        Box((0.40, 0.18, 0.66)),
        origin=Origin(xyz=(-0.04, 0.77, 0.51)),
        material=frame_steel,
        name="right_pedestal",
    )
    support_frame.visual(
        Box((0.26, 1.46, 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, 0.44)),
        material=frame_steel,
        name="lower_tie",
    )
    support_frame.visual(
        Box((0.24, 1.52, 0.10)),
        origin=Origin(xyz=(-0.18, 0.0, 0.86)),
        material=frame_steel,
        name="rear_tie",
    )
    support_frame.visual(
        Box((0.20, 0.86, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.80)),
        material=frame_steel,
        name="front_sill",
    )
    support_frame.visual(
        Box((0.10, 0.10, 0.62)),
        origin=Origin(xyz=(0.18, -0.46, 0.49)),
        material=frame_steel,
        name="left_front_upright",
    )
    support_frame.visual(
        Box((0.10, 0.10, 0.62)),
        origin=Origin(xyz=(0.18, 0.46, 0.49)),
        material=frame_steel,
        name="right_front_upright",
    )
    support_frame.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=(0.00, -0.73, 0.87)),
        material=adjuster_blue,
        name="left_bearing_riser",
    )
    support_frame.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=(0.00, 0.73, 0.87)),
        material=adjuster_blue,
        name="right_bearing_riser",
    )
    support_frame.visual(
        Box((0.24, 0.10, 0.22)),
        origin=Origin(xyz=(0.02, -0.73, axis_height)),
        material=frame_steel,
        name="left_bearing_block",
    )
    support_frame.visual(
        Box((0.24, 0.10, 0.22)),
        origin=Origin(xyz=(0.02, 0.73, axis_height)),
        material=frame_steel,
        name="right_bearing_block",
    )
    support_frame.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(
            xyz=(0.0, -0.64, axis_height),
            rpy=cyl_y.rpy,
        ),
        material=datum_finish,
        name="left_bearing",
    )
    support_frame.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(
            xyz=(0.0, 0.64, axis_height),
            rpy=cyl_y.rpy,
        ),
        material=datum_finish,
        name="right_bearing",
    )
    support_frame.visual(
        Box((0.12, 0.18, 0.015)),
        origin=Origin(xyz=(0.20, -0.32, 0.8375)),
        material=datum_finish,
        name="left_stop_pad",
    )
    support_frame.visual(
        Box((0.12, 0.18, 0.015)),
        origin=Origin(xyz=(0.20, 0.32, 0.8375)),
        material=datum_finish,
        name="right_stop_pad",
    )
    support_frame.visual(
        Box((0.14, 0.08, 0.01)),
        origin=Origin(xyz=(0.10, -0.73, 1.095)),
        material=datum_finish,
        name="left_datum_pad",
    )
    support_frame.visual(
        Box((0.14, 0.08, 0.01)),
        origin=Origin(xyz=(0.10, 0.73, 1.095)),
        material=datum_finish,
        name="right_datum_pad",
    )
    support_frame.visual(
        Box((0.16, 0.006, 0.22)),
        origin=Origin(xyz=(0.08, -0.677, axis_height + 0.01)),
        material=datum_finish,
        name="left_scale_plate",
    )
    support_frame.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.12, -0.670, axis_height + 0.085)),
        material=safety_mark,
        name="left_scale_tick_high",
    )
    support_frame.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.12, -0.670, axis_height + 0.010)),
        material=safety_mark,
        name="left_scale_tick_zero",
    )
    support_frame.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.12, -0.670, axis_height - 0.065)),
        material=safety_mark,
        name="left_scale_tick_low",
    )
    support_frame.visual(
        Box((0.14, 0.10, 0.10)),
        origin=Origin(xyz=(0.02, -0.87, 0.82)),
        material=adjuster_blue,
        name="left_adjuster_housing",
    )
    support_frame.visual(
        Box((0.14, 0.10, 0.10)),
        origin=Origin(xyz=(0.02, 0.87, 0.82)),
        material=adjuster_blue,
        name="right_adjuster_housing",
    )
    support_frame.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(
            xyz=(0.05, -0.87, 0.91),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=datum_finish,
        name="left_adjuster_post",
    )
    support_frame.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(
            xyz=(0.05, 0.87, 0.91),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=datum_finish,
        name="right_adjuster_post",
    )

    # Rotating bridge leaf: stiff deck, cheek plates, trunnions, stop shoes,
    # datum pads, and an alignment pointer. All details are attached to the leaf.
    bridge_leaf.visual(
        Box((1.64, 1.00, 0.04)),
        origin=Origin(xyz=(0.82, 0.0, 0.08)),
        material=bridge_steel,
        name="deck_panel",
    )
    bridge_leaf.visual(
        Box((0.18, 1.00, 0.20)),
        origin=Origin(xyz=(0.09, 0.0, -0.03)),
        material=bridge_steel,
        name="root_beam",
    )
    bridge_leaf.visual(
        Box((1.52, 0.06, 0.22)),
        origin=Origin(xyz=(0.78, -0.49, -0.03)),
        material=bridge_steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((1.52, 0.06, 0.22)),
        origin=Origin(xyz=(0.78, 0.49, -0.03)),
        material=bridge_steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((0.10, 1.02, 0.12)),
        origin=Origin(xyz=(1.59, 0.0, -0.02)),
        material=bridge_steel,
        name="nose_beam",
    )
    bridge_leaf.visual(
        Box((0.18, 0.06, 0.28)),
        origin=Origin(xyz=(0.09, -0.49, -0.02)),
        material=bridge_steel,
        name="left_cheek",
    )
    bridge_leaf.visual(
        Box((0.18, 0.06, 0.28)),
        origin=Origin(xyz=(0.09, 0.49, -0.02)),
        material=bridge_steel,
        name="right_cheek",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(
            xyz=(0.0, -0.56, 0.0),
            rpy=cyl_y.rpy,
        ),
        material=datum_finish,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.08, length=0.08),
        origin=Origin(
            xyz=(0.0, 0.56, 0.0),
            rpy=cyl_y.rpy,
        ),
        material=datum_finish,
        name="right_trunnion",
    )
    bridge_leaf.visual(
        Box((0.16, 0.14, 0.015)),
        origin=Origin(xyz=(0.20, -0.32, -0.1275)),
        material=datum_finish,
        name="left_stop_shoe",
    )
    bridge_leaf.visual(
        Box((0.16, 0.14, 0.015)),
        origin=Origin(xyz=(0.20, 0.32, -0.1275)),
        material=datum_finish,
        name="right_stop_shoe",
    )
    bridge_leaf.visual(
        Box((0.16, 0.12, 0.01)),
        origin=Origin(xyz=(0.34, -0.24, 0.105)),
        material=datum_finish,
        name="left_root_datum",
    )
    bridge_leaf.visual(
        Box((0.16, 0.12, 0.01)),
        origin=Origin(xyz=(0.34, 0.24, 0.105)),
        material=datum_finish,
        name="right_root_datum",
    )
    bridge_leaf.visual(
        Box((0.12, 0.16, 0.01)),
        origin=Origin(xyz=(1.46, 0.0, 0.105)),
        material=datum_finish,
        name="nose_datum",
    )
    bridge_leaf.visual(
        Box((0.12, 0.010, 0.024)),
        origin=Origin(xyz=(0.11, -0.515, 0.010)),
        material=safety_mark,
        name="left_index_pointer",
    )
    bridge_leaf.visual(
        Box((0.06, 0.12, 0.02)),
        origin=Origin(xyz=(0.56, -0.26, 0.10)),
        material=adjuster_blue,
        name="left_trim_block",
    )
    bridge_leaf.visual(
        Box((0.06, 0.12, 0.02)),
        origin=Origin(xyz=(0.56, 0.26, 0.10)),
        material=adjuster_blue,
        name="right_trim_block",
    )

    model.articulation(
        "support_frame_to_bridge_leaf",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, axis_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("support_frame_to_bridge_leaf")

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

    limits = hinge.motion_limits
    axis_ok = hinge.axis == (0.0, -1.0, 0.0)
    limits_ok = (
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.20 <= limits.upper <= 1.30
    )
    ctx.check(
        "bridge_leaf_hinge_definition",
        axis_ok and limits_ok,
        details=(
            f"Expected -Y opening hinge with 0.0..~1.25 rad travel; "
            f"got axis={hinge.axis}, limits={limits}"
        ),
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            support_frame,
            bridge_leaf,
            elem_a="left_bearing",
            elem_b="left_trunnion",
            name="left_trunnion_supported_by_left_bearing",
        )
        ctx.expect_contact(
            support_frame,
            bridge_leaf,
            elem_a="right_bearing",
            elem_b="right_trunnion",
            name="right_trunnion_supported_by_right_bearing",
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="left_stop_shoe",
            negative_elem="left_stop_pad",
            max_gap=0.001,
            max_penetration=0.0,
            name="left_closed_stop_has_controlled_gap",
        )
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="right_stop_shoe",
            negative_elem="right_stop_pad",
            max_gap=0.001,
            max_penetration=0.0,
            name="right_closed_stop_has_controlled_gap",
        )
        ctx.expect_overlap(
            bridge_leaf,
            support_frame,
            axes="y",
            min_overlap=0.82,
            elem_a="deck_panel",
            elem_b="front_sill",
            name="leaf_is_centered_over_sill_width",
        )

    with ctx.pose({hinge: 1.0}):
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="nose_datum",
            negative_elem="front_sill",
            min_gap=0.80,
            name="open_leaf_clears_support_frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
