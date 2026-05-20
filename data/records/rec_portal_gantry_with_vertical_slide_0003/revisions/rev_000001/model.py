from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.23, 0.28, 0.33, 1.0))
    extrusion_gray = model.material("extrusion_gray", rgba=(0.64, 0.68, 0.72, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.12, 0.33, 0.66, 1.0))
    dark_tool = model.material("dark_tool", rgba=(0.16, 0.17, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.11, 0.20, 0.025)),
        origin=Origin(xyz=(-0.215, 0.0, 0.0125)),
        material=painted_steel,
        name="left_foot",
    )
    frame.visual(
        Box((0.11, 0.20, 0.025)),
        origin=Origin(xyz=(0.215, 0.0, 0.0125)),
        material=painted_steel,
        name="right_foot",
    )
    frame.visual(
        Box((0.06, 0.10, 0.44)),
        origin=Origin(xyz=(-0.215, 0.0, 0.245)),
        material=extrusion_gray,
        name="left_upright",
    )
    frame.visual(
        Box((0.06, 0.10, 0.44)),
        origin=Origin(xyz=(0.215, 0.0, 0.245)),
        material=extrusion_gray,
        name="right_upright",
    )
    frame.visual(
        Box((0.50, 0.11, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=extrusion_gray,
        name="top_beam",
    )
    frame.visual(
        Box((0.05, 0.11, 0.05)),
        origin=Origin(xyz=(-0.19, 0.0, 0.44)),
        material=painted_steel,
        name="left_gusset",
    )
    frame.visual(
        Box((0.05, 0.11, 0.05)),
        origin=Origin(xyz=(0.19, 0.0, 0.44)),
        material=painted_steel,
        name="right_gusset",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.08, 0.016, 0.13)),
        origin=Origin(xyz=(0.0, -0.063, -0.005)),
        material=carriage_gray,
        name="rear_plate",
    )
    carriage.visual(
        Box((0.08, 0.016, 0.13)),
        origin=Origin(xyz=(0.0, 0.063, -0.005)),
        material=carriage_gray,
        name="front_plate",
    )
    carriage.visual(
        Box((0.08, 0.15, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=carriage_gray,
        name="bridge_block",
    )
    carriage.visual(
        Box((0.07, 0.09, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.1275)),
        material=safety_blue,
        name="slide_saddle",
    )

    z_slide = model.part("z_slide")
    z_slide.visual(
        Box((0.074, 0.09, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=safety_blue,
        name="head",
    )
    z_slide.visual(
        Box((0.044, 0.064, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        material=carriage_gray,
        name="mast",
    )
    z_slide.visual(
        Box((0.09, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.266)),
        material=painted_steel,
        name="tool_plate",
    )
    z_slide.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.292)),
        material=dark_tool,
        name="tool_nose",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.5,
            lower=-0.125,
            upper=0.125,
        ),
    )
    model.articulation(
        "carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.35,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    z_slide = object_model.get_part("z_slide")
    frame_to_carriage = object_model.get_articulation("frame_to_carriage")
    carriage_to_z_slide = object_model.get_articulation("carriage_to_z_slide")

    left_upright = frame.get_visual("left_upright")
    right_upright = frame.get_visual("right_upright")
    top_beam = frame.get_visual("top_beam")
    rear_plate = carriage.get_visual("rear_plate")
    front_plate = carriage.get_visual("front_plate")
    bridge_block = carriage.get_visual("bridge_block")
    slide_saddle = carriage.get_visual("slide_saddle")
    slide_mast = z_slide.get_visual("mast")
    slide_head = z_slide.get_visual("head")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        carriage,
        frame,
        elem_a=rear_plate,
        elem_b=top_beam,
        name="rear_plate_contacts_top_beam_at_rest",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_a=front_plate,
        elem_b=top_beam,
        name="front_plate_contacts_top_beam_at_rest",
    )
    ctx.expect_contact(
        z_slide,
        carriage,
        elem_a=slide_head,
        elem_b=slide_saddle,
        name="z_slide_head_seats_against_carriage_at_rest",
    )
    ctx.expect_within(
        z_slide,
        carriage,
        axes="xy",
        inner_elem=slide_mast,
        outer_elem=slide_saddle,
        name="slide_mast_stays_laterally_within_carriage",
    )
    ctx.expect_origin_distance(
        z_slide,
        carriage,
        axes="xy",
        max_dist=0.001,
        name="z_slide_stays_centered_under_carriage",
    )

    with ctx.pose({frame_to_carriage: -0.125}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=rear_plate,
            elem_b=top_beam,
            name="rear_plate_remains_guided_at_left_travel",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=front_plate,
            elem_b=top_beam,
            name="front_plate_remains_guided_at_left_travel",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            positive_elem=bridge_block,
            negative_elem=left_upright,
            min_gap=0.0195,
            name="carriage_clears_left_upright",
        )

    with ctx.pose({frame_to_carriage: 0.125}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=rear_plate,
            elem_b=top_beam,
            name="rear_plate_remains_guided_at_right_travel",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=front_plate,
            elem_b=top_beam,
            name="front_plate_remains_guided_at_right_travel",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="x",
            positive_elem=right_upright,
            negative_elem=bridge_block,
            min_gap=0.0195,
            name="carriage_clears_right_upright",
        )

    with ctx.pose({carriage_to_z_slide: 0.18}):
        ctx.expect_gap(
            carriage,
            z_slide,
            axis="z",
            positive_elem=slide_saddle,
            negative_elem=slide_head,
            min_gap=0.179,
            max_gap=0.181,
            name="z_slide_vertical_travel_is_180mm",
        )
        ctx.expect_within(
            z_slide,
            carriage,
            axes="xy",
            inner_elem=slide_mast,
            outer_elem=slide_saddle,
            name="slide_mast_stays_guided_at_full_extension",
        )

    left_pose = None
    right_pose = None
    rest_slide = None
    extended_slide = None
    with ctx.pose({frame_to_carriage: -0.125}):
        left_pose = ctx.part_world_position(carriage)
    with ctx.pose({frame_to_carriage: 0.125}):
        right_pose = ctx.part_world_position(carriage)
    with ctx.pose({carriage_to_z_slide: 0.0}):
        rest_slide = ctx.part_world_position(z_slide)
    with ctx.pose({carriage_to_z_slide: 0.18}):
        extended_slide = ctx.part_world_position(z_slide)

    ctx.check(
        "carriage_prismatic_axis_moves_250mm_across_gantry",
        left_pose is not None
        and right_pose is not None
        and abs((right_pose[0] - left_pose[0]) - 0.25) <= 1e-6
        and abs(right_pose[1] - left_pose[1]) <= 1e-6
        and abs(right_pose[2] - left_pose[2]) <= 1e-6,
        details=f"left_pose={left_pose}, right_pose={right_pose}",
    )
    ctx.check(
        "z_slide_prismatic_axis_moves_180mm_vertically",
        rest_slide is not None
        and extended_slide is not None
        and abs((rest_slide[2] - extended_slide[2]) - 0.18) <= 1e-6
        and abs(rest_slide[0] - extended_slide[0]) <= 1e-6
        and abs(rest_slide[1] - extended_slide[1]) <= 1e-6,
        details=f"rest_slide={rest_slide}, extended_slide={extended_slide}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
