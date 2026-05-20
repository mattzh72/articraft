from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TOTAL_WIDTH = 10.8
PANEL_WIDTH = 2.45
PANEL_HEIGHT = 1.70
PANEL_THICKNESS = 0.06
PANEL_CENTER_Z = 1.07
CENTER_POST_WIDTH = 0.08
OUTER_POST_WIDTH = 0.18
OUTER_POST_DEPTH = 0.18
STRUCTURE_TOP_Z = 2.17
GROUND_STRIP_Z = 0.025
GROUND_STRIP_HEIGHT = 0.05
GROUND_RAIL_HEAD_HEIGHT = 0.03
GROUND_RAIL_HEAD_TOP_Z = 0.075
GROUND_RAIL_BASE_HEIGHT = 0.07
TRACK_FLANGE_THICKNESS = 0.012
TRACK_FLANGE_CENTER_Y = 0.082
TRACK_INNER_FACE_Y = TRACK_FLANGE_CENTER_Y - TRACK_FLANGE_THICKNESS * 0.5
TRACK_FLANGE_HEIGHT = 0.22
TRACK_FLANGE_CENTER_Z = 2.03
TOP_TRACK_CAP_Z = 2.12
WHEEL_RADIUS = 0.042
WHEEL_CENTER_WORLD_Z = GROUND_RAIL_HEAD_TOP_Z + WHEEL_RADIUS
GUIDE_ROLLER_RADIUS = 0.018
GUIDE_ROLLER_CENTER_Y = TRACK_INNER_FACE_Y - GUIDE_ROLLER_RADIUS
GUIDE_ROLLER_CENTER_WORLD_Z = 2.025
PANEL_CLOSED_CENTER_X = CENTER_POST_WIDTH * 0.5 + PANEL_WIDTH * 0.5
PANEL_TRAVEL = 2.55


def _add_panel_visuals(part, *, material_panel, material_mesh, material_hardware) -> None:
    half_w = PANEL_WIDTH * 0.5
    half_h = PANEL_HEIGHT * 0.5
    stile_w = 0.08
    rail_h = 0.08
    mesh_bar = 0.012
    mesh_depth = 0.012
    inner_w = PANEL_WIDTH - 2.0 * stile_w
    inner_h = PANEL_HEIGHT - 2.0 * rail_h

    part.visual(
        Box((stile_w, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(-half_w + stile_w * 0.5, 0.0, 0.0)),
        material=material_panel,
        name="outer_stile",
    )
    part.visual(
        Box((stile_w, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(half_w - stile_w * 0.5, 0.0, 0.0)),
        material=material_panel,
        name="inner_stile",
    )
    part.visual(
        Box((inner_w, PANEL_THICKNESS, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, half_h - rail_h * 0.5)),
        material=material_panel,
        name="top_rail",
    )
    part.visual(
        Box((inner_w, PANEL_THICKNESS, rail_h)),
        origin=Origin(xyz=(0.0, 0.0, -half_h + rail_h * 0.5)),
        material=material_panel,
        name="bottom_rail",
    )

    vertical_count = 10
    for index in range(vertical_count):
        t = (index + 1) / (vertical_count + 1)
        x = -inner_w * 0.5 + inner_w * t
        part.visual(
            Box((mesh_bar, mesh_depth, inner_h)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=material_mesh,
            name=f"vertical_mesh_{index}",
        )

    horizontal_count = 7
    for index in range(horizontal_count):
        t = (index + 1) / (horizontal_count + 1)
        z = -inner_h * 0.5 + inner_h * t
        part.visual(
            Box((inner_w, mesh_depth, mesh_bar)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=material_mesh,
            name=f"horizontal_mesh_{index}",
        )

    carriage_xs = (-0.72, 0.72)
    wheel_local_z = WHEEL_CENTER_WORLD_Z - PANEL_CENTER_Z
    housing_z = -half_h - 0.02
    for index, x in enumerate(carriage_xs):
        part.visual(
            Box((0.18, 0.055, 0.14)),
            origin=Origin(xyz=(x, 0.0, housing_z)),
            material=material_panel,
            name=f"carriage_housing_{index}",
        )
        part.visual(
            Cylinder(radius=0.011, length=0.08),
            origin=Origin(xyz=(x, 0.0, wheel_local_z), rpy=(1.57079632679, 0.0, 0.0)),
            material=material_hardware,
            name=f"carriage_axle_{index}",
        )
        part.visual(
            Cylinder(radius=WHEEL_RADIUS, length=0.05),
            origin=Origin(xyz=(x, 0.0, wheel_local_z), rpy=(1.57079632679, 0.0, 0.0)),
            material=material_hardware,
            name="guide_wheel" if index == 0 else f"guide_wheel_{index}",
        )

    bracket_xs = (-0.66, 0.66)
    roller_local_z = GUIDE_ROLLER_CENTER_WORLD_Z - PANEL_CENTER_Z
    stem_z = half_h + 0.03
    crosshead_z = half_h + 0.10
    for index, x in enumerate(bracket_xs):
        part.visual(
            Box((0.05, 0.03, 0.12)),
            origin=Origin(xyz=(x, 0.0, stem_z)),
            material=material_panel,
            name=f"guide_stem_{index}",
        )
        part.visual(
            Box((0.16, 0.086, 0.03)),
            origin=Origin(xyz=(x, 0.0, crosshead_z)),
            material=material_panel,
            name=f"guide_crosshead_{index}",
        )

    part.visual(
        Cylinder(radius=GUIDE_ROLLER_RADIUS, length=0.05),
        origin=Origin(xyz=(bracket_xs[0], -GUIDE_ROLLER_CENTER_Y, roller_local_z)),
        material=material_hardware,
        name="front_guide_roller",
    )
    part.visual(
        Cylinder(radius=GUIDE_ROLLER_RADIUS, length=0.05),
        origin=Origin(xyz=(bracket_xs[0], GUIDE_ROLLER_CENTER_Y, roller_local_z)),
        material=material_hardware,
        name="rear_guide_roller",
    )
    part.visual(
        Cylinder(radius=GUIDE_ROLLER_RADIUS, length=0.05),
        origin=Origin(xyz=(bracket_xs[1], -GUIDE_ROLLER_CENTER_Y, roller_local_z)),
        material=material_hardware,
        name="front_guide_roller_1",
    )
    part.visual(
        Cylinder(radius=GUIDE_ROLLER_RADIUS, length=0.05),
        origin=Origin(xyz=(bracket_xs[1], GUIDE_ROLLER_CENTER_Y, roller_local_z)),
        material=material_hardware,
        name="rear_guide_roller_1",
    )

    part.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, 0.20, 1.95)),
        mass=140.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bi_parting_sliding_road_gate")

    coated_steel = model.material("coated_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.71, 1.0))
    concrete = model.material("concrete", rgba=(0.60, 0.61, 0.60, 1.0))

    gate_frame = model.part("gate_frame")
    gate_frame.visual(
        Box((TOTAL_WIDTH, 0.34, GROUND_STRIP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GROUND_STRIP_Z)),
        material=concrete,
        name="ground_strip",
    )
    gate_frame.visual(
        Box((TOTAL_WIDTH - 0.18, 0.18, GROUND_RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GROUND_RAIL_BASE_HEIGHT * 0.5)),
        material=galvanized,
        name="ground_rail_base",
    )
    gate_frame.visual(
        Box((TOTAL_WIDTH - 0.28, 0.06, GROUND_RAIL_HEAD_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                GROUND_RAIL_HEAD_TOP_Z - GROUND_RAIL_HEAD_HEIGHT * 0.5,
            )
        ),
        material=galvanized,
        name="ground_rail_head",
    )

    post_center_z = STRUCTURE_TOP_Z * 0.5
    outer_post_x = TOTAL_WIDTH * 0.5 - OUTER_POST_WIDTH * 0.5
    for side_name, x in (("left_end_post", -outer_post_x), ("right_end_post", outer_post_x)):
        gate_frame.visual(
            Box((OUTER_POST_WIDTH, OUTER_POST_DEPTH, STRUCTURE_TOP_Z)),
            origin=Origin(xyz=(x, 0.0, post_center_z)),
            material=coated_steel,
            name=side_name,
        )

    gate_frame.visual(
        Box((CENTER_POST_WIDTH, 0.16, STRUCTURE_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.0, post_center_z)),
        material=coated_steel,
        name="center_latch_post",
    )
    gate_frame.visual(
        Box((TOTAL_WIDTH - 0.16, 0.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, TOP_TRACK_CAP_Z)),
        material=coated_steel,
        name="upper_track_cap",
    )
    gate_frame.visual(
        Box((TOTAL_WIDTH - 0.16, TRACK_FLANGE_THICKNESS, TRACK_FLANGE_HEIGHT)),
        origin=Origin(xyz=(0.0, -TRACK_FLANGE_CENTER_Y, TRACK_FLANGE_CENTER_Z)),
        material=coated_steel,
        name="front_track_flange",
    )
    gate_frame.visual(
        Box((TOTAL_WIDTH - 0.16, TRACK_FLANGE_THICKNESS, TRACK_FLANGE_HEIGHT)),
        origin=Origin(xyz=(0.0, TRACK_FLANGE_CENTER_Y, TRACK_FLANGE_CENTER_Z)),
        material=coated_steel,
        name="rear_track_flange",
    )

    gate_frame.inertial = Inertial.from_geometry(
        Box((TOTAL_WIDTH, 0.40, STRUCTURE_TOP_Z)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, STRUCTURE_TOP_Z * 0.5)),
    )

    left_panel = model.part("left_panel")
    _add_panel_visuals(
        left_panel,
        material_panel=coated_steel,
        material_mesh=galvanized,
        material_hardware=galvanized,
    )

    right_panel = model.part("right_panel")
    _add_panel_visuals(
        right_panel,
        material_panel=coated_steel,
        material_mesh=galvanized,
        material_hardware=galvanized,
    )

    model.articulation(
        "frame_to_left_panel",
        ArticulationType.PRISMATIC,
        parent=gate_frame,
        child=left_panel,
        origin=Origin(xyz=(-PANEL_CLOSED_CENTER_X, 0.0, PANEL_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.30,
            lower=0.0,
            upper=PANEL_TRAVEL,
        ),
    )
    model.articulation(
        "frame_to_right_panel",
        ArticulationType.PRISMATIC,
        parent=gate_frame,
        child=right_panel,
        origin=Origin(xyz=(PANEL_CLOSED_CENTER_X, 0.0, PANEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.30,
            lower=0.0,
            upper=PANEL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    gate_frame = object_model.get_part("gate_frame")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")
    left_slide = object_model.get_articulation("frame_to_left_panel")
    right_slide = object_model.get_articulation("frame_to_right_panel")

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
        "both leaves use opposed prismatic joints",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.articulation_type == ArticulationType.PRISMATIC
        and left_slide.axis == (-1.0, 0.0, 0.0)
        and right_slide.axis == (1.0, 0.0, 0.0),
        details=f"left={left_slide.articulation_type, left_slide.axis}, right={right_slide.articulation_type, right_slide.axis}",
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            gate_frame,
            left_panel,
            axis="x",
            positive_elem="center_latch_post",
            max_gap=0.002,
            max_penetration=0.0,
            name="left leaf closes against the center latch post",
        )
        ctx.expect_gap(
            right_panel,
            gate_frame,
            axis="x",
            negative_elem="center_latch_post",
            max_gap=0.002,
            max_penetration=0.0,
            name="right leaf closes against the center latch post",
        )
        ctx.expect_contact(
            left_panel,
            gate_frame,
            elem_a="guide_wheel",
            elem_b="ground_rail_head",
            name="left leaf rides on the ground rail",
        )
        ctx.expect_contact(
            right_panel,
            gate_frame,
            elem_a="guide_wheel",
            elem_b="ground_rail_head",
            name="right leaf rides on the ground rail",
        )
        ctx.expect_contact(
            left_panel,
            gate_frame,
            elem_a="front_guide_roller",
            elem_b="front_track_flange",
            name="left leaf stays captured by the front guide track",
        )
        ctx.expect_contact(
            right_panel,
            gate_frame,
            elem_a="rear_guide_roller",
            elem_b="rear_track_flange",
            name="right leaf stays captured by the rear guide track",
        )

    left_rest = ctx.part_world_position(left_panel)
    right_rest = ctx.part_world_position(right_panel)
    with ctx.pose({left_slide: PANEL_TRAVEL, right_slide: PANEL_TRAVEL}):
        left_open = ctx.part_world_position(left_panel)
        right_open = ctx.part_world_position(right_panel)
        ctx.expect_contact(
            left_panel,
            gate_frame,
            elem_a="guide_wheel",
            elem_b="ground_rail_head",
            name="left leaf wheel remains on the rail when open",
        )
        ctx.expect_contact(
            right_panel,
            gate_frame,
            elem_a="guide_wheel",
            elem_b="ground_rail_head",
            name="right leaf wheel remains on the rail when open",
        )
        ctx.expect_contact(
            left_panel,
            gate_frame,
            elem_a="front_guide_roller",
            elem_b="front_track_flange",
            name="left leaf front guide remains engaged when open",
        )
        ctx.expect_contact(
            right_panel,
            gate_frame,
            elem_a="rear_guide_roller",
            elem_b="rear_track_flange",
            name="right leaf rear guide remains engaged when open",
        )

    ctx.check(
        "left leaf slides outward toward the left storage bay",
        left_rest is not None and left_open is not None and left_open[0] < left_rest[0] - 2.0,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right leaf slides outward toward the right storage bay",
        right_rest is not None and right_open is not None and right_open[0] > right_rest[0] + 2.0,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
