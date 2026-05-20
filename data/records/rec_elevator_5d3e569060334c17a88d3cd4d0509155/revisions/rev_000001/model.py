from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(part, name, size, xyz, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mine_cage_hoist_elevator")

    timber = model.material("timber_lining", rgba=(0.48, 0.32, 0.18, 1.0))
    steel = model.material("weathered_steel", rgba=(0.25, 0.27, 0.29, 1.0))
    dark_steel = model.material("guide_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.72, 0.64, 0.18, 1.0))
    safety_red = model.material("safety_red", rgba=(0.67, 0.16, 0.12, 1.0))

    shaft = model.part("shaft")
    _add_box(shaft, "shaft_floor", (1.95, 1.63, 0.12), (0.0, -0.04, 0.06), timber)
    _add_box(shaft, "back_wall", (1.95, 0.08, 4.80), (0.0, -0.815, 2.40), timber)
    _add_box(shaft, "left_wall", (0.08, 1.63, 4.80), (-0.915, -0.04, 2.40), timber)
    _add_box(shaft, "right_wall", (0.08, 1.63, 4.80), (0.915, -0.04, 2.40), timber)
    _add_box(shaft, "front_lintel", (1.95, 0.14, 0.10), (0.0, 0.705, 4.75), timber)
    _add_box(shaft, "left_guide_rail", (0.05, 0.16, 4.60), (-0.85, -0.28, 2.40), dark_steel)
    _add_box(shaft, "right_guide_rail", (0.05, 0.16, 4.60), (0.85, -0.28, 2.40), dark_steel)

    cage = model.part("cage")
    _add_box(cage, "floor_plate", (1.46, 0.96, 0.05), (0.0, 0.0, 0.025), steel)

    for name, x, y in (
        ("front_left_post", -0.72, 0.47),
        ("front_right_post", 0.72, 0.47),
        ("rear_left_post", -0.72, -0.47),
        ("rear_right_post", 0.72, -0.47),
    ):
        _add_box(cage, name, (0.06, 0.06, 2.10), (x, y, 1.10), steel)

    _add_box(cage, "roof_front_rail", (1.38, 0.06, 0.06), (0.0, 0.47, 2.12), steel)
    _add_box(cage, "roof_back_rail", (1.38, 0.06, 0.06), (0.0, -0.47, 2.12), steel)
    _add_box(cage, "roof_left_rail", (0.06, 0.88, 0.06), (-0.72, 0.0, 2.12), steel)
    _add_box(cage, "roof_right_rail", (0.06, 0.88, 0.06), (0.72, 0.0, 2.12), steel)
    _add_box(cage, "roof_cross_tie", (0.06, 0.88, 0.05), (-0.10, 0.0, 2.12), steel)

    _add_box(cage, "left_lower_side_rail", (0.06, 0.88, 0.04), (-0.72, 0.0, 0.55), steel)
    _add_box(cage, "right_lower_side_rail", (0.06, 0.88, 0.04), (0.72, 0.0, 0.55), steel)
    _add_box(cage, "left_mid_side_rail", (0.06, 0.88, 0.04), (-0.72, 0.0, 1.25), steel)
    _add_box(cage, "right_mid_side_rail", (0.06, 0.88, 0.04), (0.72, 0.0, 1.25), steel)
    _add_box(cage, "left_upper_side_rail", (0.06, 0.88, 0.04), (-0.72, 0.0, 1.75), steel)
    _add_box(cage, "right_upper_side_rail", (0.06, 0.88, 0.04), (0.72, 0.0, 1.75), steel)

    _add_box(cage, "back_lower_rail", (1.38, 0.06, 0.04), (0.0, -0.47, 0.70), steel)
    _add_box(cage, "back_mid_rail", (1.38, 0.06, 0.04), (0.0, -0.47, 1.25), steel)
    _add_box(cage, "back_upper_rail", (1.38, 0.06, 0.04), (0.0, -0.47, 1.75), steel)

    _add_box(cage, "front_opening_sill", (1.38, 0.06, 0.04), (0.0, 0.47, 0.12), steel)
    _add_box(cage, "front_track", (1.48, 0.08, 0.08), (0.0, 0.50, 2.12), steel)
    _add_box(cage, "front_divider_post", (0.06, 0.06, 1.96), (0.05, 0.47, 1.10), steel)
    _add_box(cage, "fixed_panel_mid_rail", (0.64, 0.06, 0.04), (0.40, 0.47, 1.05), steel)
    _add_box(cage, "fixed_panel_bar_1", (0.04, 0.04, 1.94), (0.25, 0.47, 1.11), steel)
    _add_box(cage, "fixed_panel_bar_2", (0.04, 0.04, 1.94), (0.53, 0.47, 1.11), steel)

    _add_box(cage, "left_lower_shoe", (0.075, 0.12, 0.10), (-0.7875, -0.28, 0.55), dark_steel)
    _add_box(cage, "right_lower_shoe", (0.075, 0.12, 0.10), (0.7875, -0.28, 0.55), dark_steel)
    _add_box(cage, "left_upper_shoe", (0.075, 0.12, 0.10), (-0.7875, -0.28, 1.55), dark_steel)
    _add_box(cage, "right_upper_shoe", (0.075, 0.12, 0.10), (0.7875, -0.28, 1.55), dark_steel)
    _add_box(cage, "left_upper_shoe_bracket", (0.03, 0.10, 0.13), (-0.75, -0.28, 1.665), dark_steel)
    _add_box(cage, "right_upper_shoe_bracket", (0.03, 0.10, 0.13), (0.75, -0.28, 1.665), dark_steel)
    _add_box(cage, "pawl_mount_pad", (0.10, 0.12, 0.16), (0.70, -0.28, 1.25), dark_steel)

    gate = model.part("gate")
    _add_box(gate, "bottom_gate_rail", (0.69, 0.04, 0.04), (0.0, 0.0, 0.02), gate_paint)
    _add_box(gate, "top_gate_rail", (0.69, 0.04, 0.04), (0.0, 0.0, 1.74), gate_paint)
    _add_box(gate, "left_gate_stile", (0.04, 0.04, 1.76), (-0.325, 0.0, 0.88), gate_paint)
    _add_box(gate, "right_gate_stile", (0.04, 0.04, 1.76), (0.325, 0.0, 0.88), gate_paint)
    _add_box(gate, "gate_bar_1", (0.03, 0.03, 1.68), (-0.11, 0.0, 0.88), gate_paint)
    _add_box(gate, "gate_bar_2", (0.03, 0.03, 1.68), (0.11, 0.0, 0.88), gate_paint)
    _add_box(gate, "hanger_left", (0.05, 0.06, 0.16), (-0.22, 0.0, 1.82), dark_steel)
    _add_box(gate, "hanger_right", (0.05, 0.06, 0.16), (0.22, 0.0, 1.82), dark_steel)
    _add_box(gate, "bottom_guide_left", (0.05, 0.06, 0.04), (-0.20, -0.03, -0.02), dark_steel)
    _add_box(gate, "bottom_guide_right", (0.05, 0.06, 0.04), (0.20, -0.03, -0.02), dark_steel)

    pawl = model.part("pawl")
    _add_box(pawl, "pawl_hub", (0.04, 0.08, 0.08), (0.02, 0.0, 0.0), dark_steel)
    _add_box(pawl, "pawl_arm", (0.03, 0.05, 0.12), (0.025, 0.0, -0.06), safety_red)
    _add_box(pawl, "pawl_tooth", (0.04, 0.05, 0.04), (0.0, 0.0, -0.12), safety_red)

    model.articulation(
        "shaft_to_cage",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=cage,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.7,
            lower=0.0,
            upper=2.20,
        ),
    )
    model.articulation(
        "cage_to_gate",
        ArticulationType.PRISMATIC,
        parent=cage,
        child=gate,
        origin=Origin(xyz=(-0.335, 0.53, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.5,
            lower=0.0,
            upper=0.76,
        ),
    )
    model.articulation(
        "cage_to_pawl",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=pawl,
        origin=Origin(xyz=(0.75, -0.28, 1.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=1.5,
            lower=0.0,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    cage = object_model.get_part("cage")
    gate = object_model.get_part("gate")
    pawl = object_model.get_part("pawl")
    cage_lift = object_model.get_articulation("shaft_to_cage")
    gate_slide = object_model.get_articulation("cage_to_gate")
    pawl_hinge = object_model.get_articulation("cage_to_pawl")

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
        cage,
        shaft,
        elem_a="left_lower_shoe",
        elem_b="left_guide_rail",
        contact_tol=1e-5,
        name="left lower guide shoe bears on left guide rail",
    )
    ctx.expect_contact(
        cage,
        shaft,
        elem_a="right_lower_shoe",
        elem_b="right_guide_rail",
        contact_tol=1e-5,
        name="right lower guide shoe bears on right guide rail",
    )
    ctx.expect_contact(
        gate,
        cage,
        elem_a="hanger_left",
        elem_b="front_track",
        contact_tol=1e-5,
        name="left gate hanger rides the front track",
    )
    ctx.expect_contact(
        gate,
        cage,
        elem_a="bottom_guide_left",
        elem_b="front_opening_sill",
        contact_tol=1e-5,
        name="gate bottom guide rides on the sill",
    )
    ctx.expect_gap(
        shaft,
        pawl,
        axis="x",
        positive_elem="right_guide_rail",
        negative_elem="pawl_tooth",
        min_gap=0.04,
        max_gap=0.07,
        name="retracted pawl tooth stays clear of the guide rail",
    )

    cage_rest = ctx.part_world_position(cage)
    with ctx.pose({cage_lift: 1.40}):
        cage_raised = ctx.part_world_position(cage)
        ctx.expect_contact(
            cage,
            shaft,
            elem_a="left_upper_shoe",
            elem_b="left_guide_rail",
            contact_tol=1e-5,
            name="upper left guide shoe remains on the guide rail when raised",
        )
        ctx.expect_contact(
            cage,
            shaft,
            elem_a="right_upper_shoe",
            elem_b="right_guide_rail",
            contact_tol=1e-5,
            name="upper right guide shoe remains on the guide rail when raised",
        )
    ctx.check(
        "cage lifts upward in the shaft",
        cage_rest is not None and cage_raised is not None and cage_raised[2] > cage_rest[2] + 1.0,
        details=f"rest={cage_rest}, raised={cage_raised}",
    )

    gate_open_q = gate_slide.motion_limits.upper if gate_slide.motion_limits is not None else 0.76
    gate_rest = ctx.part_world_position(gate)
    with ctx.pose({gate_slide: gate_open_q}):
        gate_open = ctx.part_world_position(gate)
        ctx.expect_gap(
            gate,
            cage,
            axis="x",
            positive_elem="bottom_gate_rail",
            negative_elem="front_divider_post",
            min_gap=0.0,
            max_gap=0.03,
            name="open gate clears the left opening edge",
        )
        ctx.expect_contact(
            gate,
            cage,
            elem_a="hanger_right",
            elem_b="front_track",
            contact_tol=1e-5,
            name="right gate hanger stays engaged in the track when open",
        )
    ctx.check(
        "gate slides sideways to open the cage entry",
        gate_rest is not None and gate_open is not None and gate_open[0] > gate_rest[0] + 0.7,
        details=f"rest={gate_rest}, open={gate_open}",
    )

    pawl_tooth_rest = ctx.part_element_world_aabb(pawl, elem="pawl_tooth")
    with ctx.pose({pawl_hinge: 0.35}):
        pawl_tooth_engaged = ctx.part_element_world_aabb(pawl, elem="pawl_tooth")
        ctx.expect_gap(
            shaft,
            pawl,
            axis="x",
            positive_elem="right_guide_rail",
            negative_elem="pawl_tooth",
            min_gap=0.0,
            max_gap=0.03,
            name="engaged pawl swings out toward the arrest guide",
        )
    rest_max_x = pawl_tooth_rest[1][0] if pawl_tooth_rest is not None else None
    engaged_max_x = pawl_tooth_engaged[1][0] if pawl_tooth_engaged is not None else None
    ctx.check(
        "pawl tooth rotates outward",
        rest_max_x is not None and engaged_max_x is not None and engaged_max_x > rest_max_x + 0.03,
        details=f"rest_max_x={rest_max_x}, engaged_max_x={engaged_max_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
