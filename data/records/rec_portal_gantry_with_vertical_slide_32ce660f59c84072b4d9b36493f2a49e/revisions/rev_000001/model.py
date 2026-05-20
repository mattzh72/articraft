from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def rectangular_tube(size: tuple[float, float, float], wall: float) -> cq.Workplane:
    sx, sy, sz = size
    outer = cq.Workplane("XY").box(sx, sy, sz)
    inner = cq.Workplane("XY").box(sx - 2.0 * wall, sy - 2.0 * wall, sz + 0.004)
    return outer.cut(inner)


def build_portal_frame() -> cq.Workplane:
    leg_width = 0.18
    leg_depth = 0.24
    leg_shell_height = 1.43
    beam_length = 1.58
    beam_depth = 0.24
    beam_height = 0.20
    wall = 0.02
    foot_width = 0.28
    foot_depth = 0.34
    foot_height = 0.05
    leg_center_x = 0.70

    left_leg = rectangular_tube((leg_width, leg_depth, leg_shell_height), wall).translate(
        (-leg_center_x, 0.0, foot_height + leg_shell_height / 2.0)
    )
    right_leg = rectangular_tube((leg_width, leg_depth, leg_shell_height), wall).translate(
        (leg_center_x, 0.0, foot_height + leg_shell_height / 2.0)
    )
    beam = rectangular_tube((beam_length, beam_depth, beam_height), wall).translate(
        (0.0, 0.0, 1.58)
    )
    left_foot = cq.Workplane("XY").box(foot_width, foot_depth, foot_height).translate(
        (-leg_center_x, 0.0, foot_height / 2.0)
    )
    right_foot = cq.Workplane("XY").box(foot_width, foot_depth, foot_height).translate(
        (leg_center_x, 0.0, foot_height / 2.0)
    )
    return left_leg.union(right_leg).union(beam).union(left_foot).union(right_foot)


def build_shuttle_body() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.26, 0.18, 0.055)
        .translate((0.0, 0.0, -0.055))
        .edges("|Z")
        .fillet(0.008)
    )


def build_tool_carriage_body() -> cq.Workplane:
    head = cq.Workplane("XY").box(0.07, 0.06, 0.16).translate((0.0, 0.0, -0.13))
    ram = cq.Workplane("XY").box(0.05, 0.05, 0.22).translate((0.0, 0.0, -0.32))
    tool_block = cq.Workplane("XY").box(0.055, 0.055, 0.10).translate((0.0, 0.0, -0.48))
    return head.union(ram).union(tool_block).edges("|Z").fillet(0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_portal")

    frame_color = model.material("frame_color", color=(0.83, 0.85, 0.88, 1.0))
    rail_color = model.material("rail_color", color=(0.18, 0.18, 0.20, 1.0))
    shuttle_color = model.material("shuttle_color", color=(0.23, 0.27, 0.31, 1.0))
    carriage_color = model.material("carriage_color", color=(0.28, 0.43, 0.67, 1.0))
    tool_color = model.material("tool_color", color=(0.12, 0.12, 0.14, 1.0))

    portal_frame = model.part("portal_frame")
    portal_frame.visual(
        mesh_from_cadquery(build_portal_frame(), "portal_structure"),
        material=frame_color,
        name="portal_structure",
    )
    portal_frame.visual(
        Box((1.18, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.075, 1.47)),
        material=rail_color,
        name="front_rail",
    )
    portal_frame.visual(
        Box((1.18, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.075, 1.47)),
        material=rail_color,
        name="rear_rail",
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        mesh_from_cadquery(build_shuttle_body(), "shuttle_body"),
        material=shuttle_color,
        name="shuttle_body",
    )
    shuttle.visual(
        Box((0.05, 0.16, 0.195)),
        origin=Origin(xyz=(-0.105, 0.0, -0.18)),
        material=shuttle_color,
        name="left_cheek",
    )
    shuttle.visual(
        Box((0.05, 0.16, 0.195)),
        origin=Origin(xyz=(0.105, 0.0, -0.18)),
        material=shuttle_color,
        name="right_cheek",
    )
    shuttle.visual(
        Box((0.22, 0.035, 0.0275)),
        origin=Origin(xyz=(0.0, 0.07, -0.01375)),
        material=rail_color,
        name="front_pad",
    )
    shuttle.visual(
        Box((0.22, 0.035, 0.0275)),
        origin=Origin(xyz=(0.0, -0.07, -0.01375)),
        material=rail_color,
        name="rear_pad",
    )
    shuttle.visual(
        Box((0.014, 0.08, 0.52)),
        origin=Origin(xyz=(-0.056, 0.0, -0.3425)),
        material=rail_color,
        name="left_guide",
    )
    shuttle.visual(
        Box((0.014, 0.08, 0.52)),
        origin=Origin(xyz=(0.056, 0.0, -0.3425)),
        material=rail_color,
        name="right_guide",
    )

    tool_carriage = model.part("tool_carriage")
    tool_carriage.visual(
        mesh_from_cadquery(build_tool_carriage_body(), "tool_carriage_body"),
        material=carriage_color,
        name="carriage_body",
    )
    tool_carriage.visual(
        Box((0.014, 0.08, 0.18)),
        origin=Origin(xyz=(-0.042, 0.0, -0.15)),
        material=tool_color,
        name="left_shoe",
    )
    tool_carriage.visual(
        Box((0.014, 0.08, 0.18)),
        origin=Origin(xyz=(0.042, 0.0, -0.15)),
        material=tool_color,
        name="right_shoe",
    )
    tool_carriage.visual(
        Cylinder(radius=0.02, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.62)),
        material=tool_color,
        name="probe_tip",
    )

    shuttle_slide = model.articulation(
        "shuttle_slide",
        ArticulationType.PRISMATIC,
        parent=portal_frame,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.6,
            lower=-0.48,
            upper=0.48,
        ),
    )
    model.articulation(
        "tool_slide",
        ArticulationType.PRISMATIC,
        parent=shuttle,
        child=tool_carriage,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    portal_frame = object_model.get_part("portal_frame")
    shuttle = object_model.get_part("shuttle")
    tool_carriage = object_model.get_part("tool_carriage")
    shuttle_slide = object_model.get_articulation("shuttle_slide")
    tool_slide = object_model.get_articulation("tool_slide")

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
        "portal_parts_present",
        {part.name for part in object_model.parts}
        == {"portal_frame", "shuttle", "tool_carriage"},
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "shuttle_joint_axis",
        shuttle_slide.axis == (1.0, 0.0, 0.0)
        and shuttle_slide.motion_limits is not None
        and shuttle_slide.motion_limits.lower == -0.48
        and shuttle_slide.motion_limits.upper == 0.48,
        details=f"axis={shuttle_slide.axis}, limits={shuttle_slide.motion_limits}",
    )
    ctx.check(
        "tool_joint_axis",
        tool_slide.axis == (0.0, 0.0, -1.0)
        and tool_slide.motion_limits is not None
        and tool_slide.motion_limits.lower == 0.0
        and tool_slide.motion_limits.upper == 0.26,
        details=f"axis={tool_slide.axis}, limits={tool_slide.motion_limits}",
    )

    ctx.expect_contact(
        shuttle,
        portal_frame,
        elem_a="front_pad",
        elem_b="front_rail",
        name="front_pad_contacts_front_rail",
    )
    ctx.expect_contact(
        shuttle,
        portal_frame,
        elem_a="rear_pad",
        elem_b="rear_rail",
        name="rear_pad_contacts_rear_rail",
    )
    ctx.expect_contact(
        tool_carriage,
        shuttle,
        elem_a="left_shoe",
        elem_b="left_guide",
        name="left_shoe_contacts_left_guide",
    )
    ctx.expect_contact(
        tool_carriage,
        shuttle,
        elem_a="right_shoe",
        elem_b="right_guide",
        name="right_shoe_contacts_right_guide",
    )

    with ctx.pose({shuttle_slide: -0.48, tool_slide: 0.0}):
        ctx.expect_within(
            shuttle,
            portal_frame,
            axes="x",
            inner_elem="front_pad",
            outer_elem="front_rail",
            margin=0.001,
            name="front_pad_stays_on_front_rail_left",
        )
        ctx.expect_within(
            shuttle,
            portal_frame,
            axes="x",
            inner_elem="rear_pad",
            outer_elem="rear_rail",
            margin=0.001,
            name="rear_pad_stays_on_rear_rail_left",
        )

    with ctx.pose({shuttle_slide: 0.48, tool_slide: 0.0}):
        ctx.expect_within(
            shuttle,
            portal_frame,
            axes="x",
            inner_elem="front_pad",
            outer_elem="front_rail",
            margin=0.001,
            name="front_pad_stays_on_front_rail_right",
        )
        ctx.expect_within(
            shuttle,
            portal_frame,
            axes="x",
            inner_elem="rear_pad",
            outer_elem="rear_rail",
            margin=0.001,
            name="rear_pad_stays_on_rear_rail_right",
        )

    with ctx.pose({shuttle_slide: 0.0, tool_slide: 0.0}):
        ctx.expect_within(
            tool_carriage,
            shuttle,
            axes="z",
            inner_elem="left_shoe",
            outer_elem="left_guide",
            margin=0.001,
            name="left_shoe_engaged_at_top",
        )
        ctx.expect_within(
            tool_carriage,
            shuttle,
            axes="z",
            inner_elem="right_shoe",
            outer_elem="right_guide",
            margin=0.001,
            name="right_shoe_engaged_at_top",
        )

    with ctx.pose({shuttle_slide: 0.0, tool_slide: 0.26}):
        ctx.expect_within(
            tool_carriage,
            shuttle,
            axes="z",
            inner_elem="left_shoe",
            outer_elem="left_guide",
            margin=0.001,
            name="left_shoe_engaged_at_bottom",
        )
        ctx.expect_within(
            tool_carriage,
            shuttle,
            axes="z",
            inner_elem="right_shoe",
            outer_elem="right_guide",
            margin=0.001,
            name="right_shoe_engaged_at_bottom",
        )

    shuttle_body_aabb = ctx.part_element_world_aabb(shuttle, elem="shuttle_body")
    carriage_body_aabb = ctx.part_element_world_aabb(tool_carriage, elem="carriage_body")
    if shuttle_body_aabb is None or carriage_body_aabb is None:
        ctx.fail("distinct_carriage_scales", "missing shuttle or carriage body AABB")
    else:
        shuttle_dx = shuttle_body_aabb[1][0] - shuttle_body_aabb[0][0]
        shuttle_dz = shuttle_body_aabb[1][2] - shuttle_body_aabb[0][2]
        carriage_dx = carriage_body_aabb[1][0] - carriage_body_aabb[0][0]
        carriage_dz = carriage_body_aabb[1][2] - carriage_body_aabb[0][2]
        ctx.check(
            "distinct_carriage_scales",
            shuttle_dx > carriage_dx * 2.5 and carriage_dz > shuttle_dz * 2.5,
            details=(
                f"shuttle_dx={shuttle_dx:.3f}, shuttle_dz={shuttle_dz:.3f}, "
                f"carriage_dx={carriage_dx:.3f}, carriage_dz={carriage_dz:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
