from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_inspection_portal")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.22, 0.25, 1.0))
    dark_rail = model.material("dark_linear_rail", rgba=(0.035, 0.040, 0.045, 1.0))
    blue_carriage = model.material("blue_moving_carriage", rgba=(0.08, 0.18, 0.32, 1.0))
    tool_gray = model.material("tool_carriage_gray", rgba=(0.36, 0.39, 0.40, 1.0))
    cover = model.material("removable_service_cover", rgba=(0.55, 0.58, 0.58, 1.0))
    safety = model.material("yellow_safety_stops", rgba=(0.95, 0.68, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    def add_box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    portal = model.part("portal_frame")

    # A broad rigid box-section gantry: two legs and a continuous crossbeam create
    # a clear inspection opening below the span.
    add_box(portal, "crossbeam_tube", (3.20, 0.40, 0.24), (0.0, 0.0, 2.00), painted_steel)
    add_box(portal, "top_cap", (3.32, 0.44, 0.035), (0.0, 0.0, 2.137), painted_steel)
    add_box(portal, "leg_0", (0.20, 0.36, 1.88), (-1.45, 0.0, 0.94), painted_steel)
    add_box(portal, "leg_1", (0.20, 0.36, 1.88), (1.45, 0.0, 0.94), painted_steel)
    add_box(portal, "foot_0", (0.55, 0.62, 0.10), (-1.45, 0.0, 0.05), painted_steel)
    add_box(portal, "foot_1", (0.55, 0.62, 0.10), (1.45, 0.0, 0.05), painted_steel)
    add_box(portal, "front_floor_sill", (2.45, 0.08, 0.11), (0.0, -0.25, 0.055), painted_steel)
    add_box(portal, "rear_floor_sill", (2.45, 0.08, 0.11), (0.0, 0.25, 0.055), painted_steel)

    # Corner gussets and bolted pads make the crossbeam/leg junctions read as a
    # welded portal frame rather than isolated boxes.
    for i, x in enumerate((-1.31, 1.31)):
        add_box(portal, f"front_gusset_{i}", (0.28, 0.028, 0.34), (x, -0.214, 1.80), painted_steel)
        add_box(portal, f"rear_gusset_{i}", (0.28, 0.028, 0.34), (x, 0.214, 1.80), painted_steel)
        add_box(portal, f"base_bolt_pad_{i}", (0.35, 0.50, 0.025), (x, 0.0, 0.115), dark_rail)

    # Fixed linear rails under the beam.  Hanger blocks connect each rail to the
    # beam shell; end stops sit beyond the commanded shuttle travel.
    add_box(portal, "front_rail", (2.70, 0.055, 0.055), (0.0, -0.235, 1.800), dark_rail)
    add_box(portal, "rear_rail", (2.70, 0.055, 0.055), (0.0, 0.235, 1.800), dark_rail)
    for j, x in enumerate((-1.05, -0.52, 0.0, 0.52, 1.05)):
        add_box(portal, f"front_rail_hanger_{j}", (0.080, 0.055, 0.070), (x, -0.235, 1.858), painted_steel)
        add_box(portal, f"rear_rail_hanger_{j}", (0.080, 0.055, 0.070), (x, 0.235, 1.858), painted_steel)
    add_box(portal, "travel_stop_neg", (0.10, 0.60, 0.13), (-1.32, 0.0, 1.745), safety)
    add_box(portal, "travel_stop_pos", (0.10, 0.60, 0.13), (1.32, 0.0, 1.745), safety)
    add_box(portal, "front_scale_strip", (2.35, 0.018, 0.030), (0.0, -0.226, 1.915), cover)

    shuttle = model.part("shuttle")
    # The moving beam carriage is a squat truck with guide shoes tucked just
    # under the two fixed rails.
    add_box(shuttle, "upper_carriage", (0.52, 0.42, 0.20), (0.0, 0.0, 0.035), blue_carriage)
    add_box(shuttle, "front_rail_shoe", (0.44, 0.052, 0.058), (0.0, -0.235, 0.1635), dark_rail)
    add_box(shuttle, "rear_rail_shoe", (0.44, 0.052, 0.058), (0.0, 0.235, 0.1635), dark_rail)
    add_box(shuttle, "front_service_cover", (0.30, 0.018, 0.105), (0.0, -0.218, 0.045), cover)
    add_box(shuttle, "rear_service_cover", (0.30, 0.018, 0.105), (0.0, 0.218, 0.045), cover)
    for k, x in enumerate((-0.18, 0.18)):
        add_box(shuttle, f"front_rib_{k}", (0.035, 0.040, 0.18), (x, -0.225, 0.020), blue_carriage)
        add_box(shuttle, f"rear_rib_{k}", (0.035, 0.040, 0.18), (x, 0.225, 0.020), blue_carriage)

    # A rigid yoke below the shuttle carries the vertical guide pair for the
    # narrower hanging tool carriage.
    add_box(shuttle, "guide_bridge", (0.48, 0.32, 0.060), (0.0, 0.0, -0.095), blue_carriage)
    add_box(shuttle, "vertical_guide_neg", (0.045, 0.24, 0.960), (-0.18, 0.0, -0.605), dark_rail)
    add_box(shuttle, "vertical_guide_pos", (0.045, 0.24, 0.960), (0.18, 0.0, -0.605), dark_rail)
    add_box(shuttle, "front_guide_cover", (0.43, 0.050, 0.42), (0.0, -0.145, -0.42), cover)
    add_box(shuttle, "rear_guide_cover", (0.43, 0.050, 0.42), (0.0, 0.145, -0.42), cover)
    for x in (-0.18, 0.18):
        add_box(shuttle, f"upper_guide_gusset_{'neg' if x < 0 else 'pos'}", (0.12, 0.20, 0.070), (x, 0.0, -0.155), blue_carriage)

    tool = model.part("tool_carriage")
    # The hanging tool carriage is intentionally much narrower than the moving
    # shuttle.  Its side shoes sit inside the vertical guide pair with visible
    # running clearance.
    add_box(tool, "tool_slide_bar", (0.14, 0.16, 0.62), (0.0, 0.0, -0.36), tool_gray)
    add_box(tool, "upper_crosshead", (0.26, 0.14, 0.035), (0.0, 0.0, -0.18), tool_gray)
    add_box(tool, "lower_crosshead", (0.26, 0.14, 0.035), (0.0, 0.0, -0.52), tool_gray)
    add_box(tool, "slider_shoe_neg", (0.055, 0.18, 0.13), (-0.130, 0.0, -0.18), dark_rail)
    add_box(tool, "slider_shoe_pos", (0.055, 0.18, 0.13), (0.130, 0.0, -0.18), dark_rail)
    add_box(tool, "lower_slider_shoe_neg", (0.055, 0.18, 0.13), (-0.130, 0.0, -0.52), dark_rail)
    add_box(tool, "lower_slider_shoe_pos", (0.055, 0.18, 0.13), (0.130, 0.0, -0.52), dark_rail)
    add_box(tool, "tool_service_cover", (0.105, 0.014, 0.33), (0.0, -0.087, -0.38), cover)
    add_box(tool, "tool_front_rib", (0.025, 0.024, 0.52), (0.0, -0.097, -0.36), tool_gray)
    tool.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.76)),
        material=black,
        name="probe_sleeve",
    )
    tool.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.91)),
        material=black,
        name="probe_tip",
    )

    model.articulation(
        "portal_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=portal,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, 1.58)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.35, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "shuttle_to_tool",
        ArticulationType.PRISMATIC,
        parent=shuttle,
        child=tool,
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.20, lower=0.0, upper=0.34),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    portal = object_model.get_part("portal_frame")
    shuttle = object_model.get_part("shuttle")
    tool = object_model.get_part("tool_carriage")
    shuttle_slide = object_model.get_articulation("portal_to_shuttle")
    tool_slide = object_model.get_articulation("shuttle_to_tool")

    ctx.check(
        "two prismatic inspection motions",
        shuttle_slide.articulation_type == ArticulationType.PRISMATIC
        and tool_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"shuttle={shuttle_slide.articulation_type}, tool={tool_slide.articulation_type}",
    )

    ctx.expect_contact(
        portal,
        shuttle,
        elem_a="front_rail",
        elem_b="front_rail_shoe",
        contact_tol=0.0005,
        name="front shuttle shoe rides on fixed rail",
    )
    ctx.expect_contact(
        portal,
        shuttle,
        elem_a="rear_rail",
        elem_b="rear_rail_shoe",
        contact_tol=0.0005,
        name="rear shuttle shoe rides on fixed rail",
    )

    rest_shuttle_pos = ctx.part_world_position(shuttle)
    with ctx.pose({shuttle_slide: 0.85}):
        ctx.expect_within(
            shuttle,
            portal,
            axes="x",
            inner_elem="front_rail_shoe",
            outer_elem="front_rail",
            margin=0.0,
            name="front shoe remains on rail at positive travel",
        )
        ctx.expect_gap(
            portal,
            shuttle,
            axis="x",
            positive_elem="travel_stop_pos",
            negative_elem="upper_carriage",
            min_gap=0.05,
            name="positive shuttle travel stops before end block",
        )
        max_shuttle_pos = ctx.part_world_position(shuttle)
    with ctx.pose({shuttle_slide: -0.85}):
        ctx.expect_within(
            shuttle,
            portal,
            axes="x",
            inner_elem="rear_rail_shoe",
            outer_elem="rear_rail",
            margin=0.0,
            name="rear shoe remains on rail at negative travel",
        )
        ctx.expect_gap(
            shuttle,
            portal,
            axis="x",
            positive_elem="upper_carriage",
            negative_elem="travel_stop_neg",
            min_gap=0.05,
            name="negative shuttle travel stops before end block",
        )
        min_shuttle_pos = ctx.part_world_position(shuttle)
    ctx.check(
        "shuttle traverses the beam span",
        rest_shuttle_pos is not None
        and max_shuttle_pos is not None
        and min_shuttle_pos is not None
        and max_shuttle_pos[0] > rest_shuttle_pos[0] + 0.80
        and min_shuttle_pos[0] < rest_shuttle_pos[0] - 0.80,
        details=f"rest={rest_shuttle_pos}, max={max_shuttle_pos}, min={min_shuttle_pos}",
    )

    for q, label in ((0.0, "raised"), (0.34, "lowered")):
        with ctx.pose({tool_slide: q}):
            ctx.expect_contact(
                shuttle,
                tool,
                elem_a="vertical_guide_pos",
                elem_b="slider_shoe_pos",
                contact_tol=0.0005,
                name=f"right vertical guide supports tool when {label}",
            )
            ctx.expect_contact(
                tool,
                shuttle,
                elem_a="slider_shoe_neg",
                elem_b="vertical_guide_neg",
                contact_tol=0.0005,
                name=f"left vertical guide supports tool when {label}",
            )
            ctx.expect_within(
                tool,
                shuttle,
                axes="z",
                inner_elem="lower_slider_shoe_pos",
                outer_elem="vertical_guide_pos",
                margin=0.005,
                name=f"lower tool shoe retained in guide when {label}",
            )

    rest_tool_pos = ctx.part_world_position(tool)
    with ctx.pose({tool_slide: 0.34}):
        lowered_tool_pos = ctx.part_world_position(tool)
    ctx.check(
        "tool carriage slides downward",
        rest_tool_pos is not None
        and lowered_tool_pos is not None
        and lowered_tool_pos[2] < rest_tool_pos[2] - 0.30,
        details=f"rest={rest_tool_pos}, lowered={lowered_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
