from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _add_box_visual(part, name, size, center, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _make_frame_body() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.46, 0.60, 0.04, centered=(True, True, False))

    tower = (
        cq.Workplane("XY")
        .box(0.10, 0.60, 0.98, centered=(True, True, False))
        .translate((-0.16, 0.0, 0.04))
    )
    tower = (
        tower.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.40, 0.62)
        .cutBlind(-0.07)
    )

    beam = (
        cq.Workplane("XY")
        .box(0.076, 0.64, 0.18, centered=(True, True, True))
        .translate((-0.072, 0.0, 0.74))
    )

    left_cheek = (
        cq.Workplane("XY")
        .box(0.08, 0.05, 0.78, centered=(True, True, False))
        .translate((-0.10, -0.275, 0.04))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.08, 0.05, 0.78, centered=(True, True, False))
        .translate((-0.10, 0.275, 0.04))
    )

    lower_rib = (
        cq.Workplane("XY")
        .box(0.05, 0.64, 0.09, centered=(True, True, True))
        .translate((-0.12, 0.0, 0.58))
    )

    return (
        base.union(tower)
        .union(beam)
        .union(left_cheek)
        .union(right_cheek)
        .union(lower_rib)
    )


def _make_crosshead_body() -> cq.Workplane:
    upper_runner_bridge = (
        cq.Workplane("XY")
        .box(0.03, 0.26, 0.026, centered=(True, True, True))
        .translate((0.015, 0.0, 0.05))
    )
    lower_runner_bridge = (
        cq.Workplane("XY")
        .box(0.03, 0.26, 0.026, centered=(True, True, True))
        .translate((0.015, 0.0, -0.05))
    )
    carriage_spine = (
        cq.Workplane("XY")
        .box(0.028, 0.30, 0.16, centered=(True, True, True))
        .translate((0.028, 0.0, 0.0))
    )
    z_stage_backbone = (
        cq.Workplane("XY")
        .box(0.032, 0.14, 0.50, centered=(True, True, True))
        .translate((0.056, 0.0, -0.10))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.05, 0.18, 0.05, centered=(True, True, True))
        .translate((0.055, 0.0, 0.17))
    )

    body = (
        upper_runner_bridge.union(lower_runner_bridge)
        .union(carriage_spine)
        .union(z_stage_backbone)
        .union(top_cap)
    )

    return (
        body.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.07, 0.26)
        .cutBlind(-0.012)
    )


def _make_tool_body() -> cq.Workplane:
    back_plate = (
        cq.Workplane("XY")
        .box(0.032, 0.14, 0.30, centered=(True, True, True))
        .translate((0.020, 0.0, -0.18))
    )
    front_plate = (
        cq.Workplane("XY")
        .box(0.018, 0.22, 0.18, centered=(True, True, True))
        .translate((0.045, 0.0, -0.24))
    )
    lower_neck = (
        cq.Workplane("XY")
        .box(0.040, 0.08, 0.10, centered=(True, True, True))
        .translate((0.060, 0.0, -0.32))
    )
    tool_flange = (
        cq.Workplane("XY")
        .box(0.060, 0.10, 0.028, centered=(True, True, True))
        .translate((0.070, 0.0, -0.37))
    )

    body = back_plate.union(front_plate).union(lower_neck).union(tool_flange)

    return (
        body.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.10, 0.10)
        .cutBlind(-0.010)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_transfer_stage")

    frame_paint = model.material("frame_paint", color=(0.21, 0.23, 0.26))
    machine_steel = model.material("machine_steel", color=(0.72, 0.74, 0.77))
    carriage_paint = model.material("carriage_paint", color=(0.84, 0.85, 0.86))
    tool_paint = model.material("tool_paint", color=(0.18, 0.42, 0.73))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_body(), "frame_body"),
        material=frame_paint,
        name="frame_body",
    )
    _add_box_visual(
        frame,
        "y_upper_rail",
        (0.016, 0.58, 0.022),
        (-0.026, 0.0, 0.79),
        machine_steel,
    )
    _add_box_visual(
        frame,
        "y_lower_rail",
        (0.016, 0.58, 0.022),
        (-0.026, 0.0, 0.69),
        machine_steel,
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_make_crosshead_body(), "crosshead_body"),
        material=carriage_paint,
        name="crosshead_body",
    )
    _add_box_visual(
        crosshead,
        "y_upper_runner",
        (0.03, 0.26, 0.024),
        (0.015, 0.0, 0.05),
        carriage_paint,
    )
    _add_box_visual(
        crosshead,
        "y_lower_runner",
        (0.03, 0.26, 0.024),
        (0.015, 0.0, -0.05),
        carriage_paint,
    )
    _add_box_visual(
        crosshead,
        "z_left_rail",
        (0.014, 0.022, 0.36),
        (0.079, -0.052, -0.03),
        machine_steel,
    )
    _add_box_visual(
        crosshead,
        "z_right_rail",
        (0.014, 0.022, 0.36),
        (0.079, 0.052, -0.03),
        machine_steel,
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        mesh_from_cadquery(_make_tool_body(), "tool_body"),
        material=tool_paint,
        name="tool_body",
    )
    _add_box_visual(
        tool_plate,
        "z_left_runner",
        (0.028, 0.022, 0.10),
        (0.014, -0.052, -0.06),
        carriage_paint,
    )
    _add_box_visual(
        tool_plate,
        "z_right_runner",
        (0.028, 0.022, 0.10),
        (0.014, 0.052, -0.06),
        carriage_paint,
    )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(-0.018, 0.0, 0.74)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.35,
            lower=-0.14,
            upper=0.14,
        ),
    )
    model.articulation(
        "crosshead_to_tool_plate",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=tool_plate,
        origin=Origin(xyz=(0.086, 0.0, 0.15)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.30,
            lower=0.0,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crosshead = object_model.get_part("crosshead")
    tool_plate = object_model.get_part("tool_plate")

    y_slide = object_model.get_articulation("frame_to_crosshead")
    z_slide = object_model.get_articulation("crosshead_to_tool_plate")

    y_upper_rail = frame.get_visual("y_upper_rail")
    y_lower_rail = frame.get_visual("y_lower_rail")
    y_upper_runner = crosshead.get_visual("y_upper_runner")
    y_lower_runner = crosshead.get_visual("y_lower_runner")
    z_left_rail = crosshead.get_visual("z_left_rail")
    z_right_rail = crosshead.get_visual("z_right_rail")
    z_left_runner = tool_plate.get_visual("z_left_runner")
    z_right_runner = tool_plate.get_visual("z_right_runner")

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
        "crosshead_joint_is_y_prismatic",
        y_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0)
        and y_slide.motion_limits is not None
        and y_slide.motion_limits.lower == -0.14
        and y_slide.motion_limits.upper == 0.14,
        details=f"unexpected crosshead joint configuration: axis={y_slide.axis}, limits={y_slide.motion_limits}",
    )
    ctx.check(
        "tool_joint_is_descending_z_prismatic",
        z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(z_slide.axis) == (0.0, 0.0, -1.0)
        and z_slide.motion_limits is not None
        and z_slide.motion_limits.lower == 0.0
        and z_slide.motion_limits.upper == 0.22,
        details=f"unexpected tool joint configuration: axis={z_slide.axis}, limits={z_slide.motion_limits}",
    )

    ctx.expect_contact(
        crosshead,
        frame,
        elem_a=y_upper_runner,
        elem_b=y_upper_rail,
        contact_tol=0.001,
        name="upper_y_runner_contacts_upper_rail",
    )
    ctx.expect_contact(
        crosshead,
        frame,
        elem_a=y_lower_runner,
        elem_b=y_lower_rail,
        contact_tol=0.001,
        name="lower_y_runner_contacts_lower_rail",
    )
    ctx.expect_overlap(
        crosshead,
        frame,
        elem_a=y_upper_runner,
        elem_b=y_upper_rail,
        axes="yz",
        min_overlap=0.02,
        name="upper_y_guide_has_supported_bearing_overlap",
    )
    ctx.expect_overlap(
        crosshead,
        frame,
        elem_a=y_lower_runner,
        elem_b=y_lower_rail,
        axes="yz",
        min_overlap=0.02,
        name="lower_y_guide_has_supported_bearing_overlap",
    )

    ctx.expect_contact(
        tool_plate,
        crosshead,
        elem_a=z_left_runner,
        elem_b=z_left_rail,
        contact_tol=0.001,
        name="left_z_runner_contacts_left_rail",
    )
    ctx.expect_contact(
        tool_plate,
        crosshead,
        elem_a=z_right_runner,
        elem_b=z_right_rail,
        contact_tol=0.001,
        name="right_z_runner_contacts_right_rail",
    )
    ctx.expect_overlap(
        tool_plate,
        crosshead,
        elem_a=z_left_runner,
        elem_b=z_left_rail,
        axes="yz",
        min_overlap=0.02,
        name="left_z_guide_has_supported_bearing_overlap",
    )
    ctx.expect_overlap(
        tool_plate,
        crosshead,
        elem_a=z_right_runner,
        elem_b=z_right_rail,
        axes="yz",
        min_overlap=0.02,
        name="right_z_guide_has_supported_bearing_overlap",
    )

    rest_crosshead_y = ctx.part_world_position(crosshead)[1]
    rest_tool_y = ctx.part_world_position(tool_plate)[1]
    rest_tool_z = ctx.part_world_position(tool_plate)[2]

    with ctx.pose({y_slide: 0.10}):
        moved_crosshead_y = ctx.part_world_position(crosshead)[1]
        moved_tool_y = ctx.part_world_position(tool_plate)[1]
        ctx.check(
            "crosshead_translates_along_y",
            abs((moved_crosshead_y - rest_crosshead_y) - 0.10) < 1e-6,
            details=f"expected +0.10 m Y motion, got {moved_crosshead_y - rest_crosshead_y}",
        )
        ctx.check(
            "tool_plate_follows_crosshead_in_y",
            abs(moved_tool_y - moved_crosshead_y) < 1e-6,
            details=f"tool Y {moved_tool_y} did not follow crosshead Y {moved_crosshead_y}",
        )

    with ctx.pose({z_slide: 0.18}):
        moved_tool_y = ctx.part_world_position(tool_plate)[1]
        moved_tool_z = ctx.part_world_position(tool_plate)[2]
        ctx.check(
            "tool_plate_descends_along_z",
            abs((rest_tool_z - moved_tool_z) - 0.18) < 1e-6,
            details=f"expected 0.18 m descent, got {rest_tool_z - moved_tool_z}",
        )
        ctx.check(
            "z_axis_motion_preserves_y_alignment",
            abs(moved_tool_y - rest_tool_y) < 1e-6,
            details=f"tool Y drifted from {rest_tool_y} to {moved_tool_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
