from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.09
CANOPY_TOP_THICKNESS = 0.012
CANOPY_SIDE_THICKNESS = 0.012
CANOPY_BACK_THICKNESS = 0.010
FRONT_PANEL_THICKNESS = 0.0015
FRONT_FACE_Y = CANOPY_DEPTH * 0.5
FILTER_WIDTH = CANOPY_WIDTH - 2.0 * CANOPY_SIDE_THICKNESS - 0.040
FILTER_DEPTH = 0.400
FILTER_THICKNESS = 0.004
FILTER_SIDE_RAIL_WIDTH = 0.020
FILTER_FRONT_RAIL_DEPTH = 0.050

CHIMNEY_WIDTH = 0.30
CHIMNEY_DEPTH = 0.24
CHIMNEY_HEIGHT = 0.72
CHIMNEY_WALL_THICKNESS = 0.008
CHIMNEY_BOTTOM_Z = CANOPY_HEIGHT - 0.004

BUTTON_CAP_WIDTH = 0.018
BUTTON_CAP_DEPTH = 0.003
BUTTON_CAP_HEIGHT = 0.010
BUTTON_STEM_WIDTH = 0.011
BUTTON_STEM_DEPTH = 0.010
BUTTON_STEM_HEIGHT = 0.006
BUTTON_TRAVEL = 0.002
BUTTON_CLUSTER_X = 0.18

BUTTON_SPECS = (
    {
        "part_name": "button_upper_0",
        "joint_name": "button_upper_0_slide",
        "x": BUTTON_CLUSTER_X - 0.078,
        "z": 0.055,
        "row": "upper",
        "index": 0,
    },
    {
        "part_name": "button_upper_1",
        "joint_name": "button_upper_1_slide",
        "x": BUTTON_CLUSTER_X - 0.026,
        "z": 0.055,
        "row": "upper",
        "index": 1,
    },
    {
        "part_name": "button_upper_2",
        "joint_name": "button_upper_2_slide",
        "x": BUTTON_CLUSTER_X + 0.026,
        "z": 0.055,
        "row": "upper",
        "index": 2,
    },
    {
        "part_name": "button_upper_3",
        "joint_name": "button_upper_3_slide",
        "x": BUTTON_CLUSTER_X + 0.078,
        "z": 0.055,
        "row": "upper",
        "index": 3,
    },
    {
        "part_name": "button_lower_0",
        "joint_name": "button_lower_0_slide",
        "x": BUTTON_CLUSTER_X - 0.052,
        "z": 0.031,
        "row": "lower",
        "index": 0,
    },
    {
        "part_name": "button_lower_1",
        "joint_name": "button_lower_1_slide",
        "x": BUTTON_CLUSTER_X + 0.000,
        "z": 0.031,
        "row": "lower",
        "index": 1,
    },
    {
        "part_name": "button_lower_2",
        "joint_name": "button_lower_2_slide",
        "x": BUTTON_CLUSTER_X + 0.052,
        "z": 0.031,
        "row": "lower",
        "index": 2,
    },
)


def _rect_profile(width: float, height: float, *, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def _build_front_panel_mesh():
    outer_profile = _rect_profile(CANOPY_WIDTH, CANOPY_HEIGHT)
    hole_profiles = [
        _rect_profile(
            BUTTON_STEM_WIDTH,
            BUTTON_STEM_HEIGHT,
            center=(spec["x"], spec["z"] - CANOPY_HEIGHT * 0.5),
        )
        for spec in BUTTON_SPECS
    ]
    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=FRONT_PANEL_THICKNESS,
        center=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(geometry, ASSETS.mesh_path("range_hood_front_panel.obj"))


def _build_grease_filter_mesh():
    outer_profile = _rect_profile(FILTER_WIDTH, FILTER_DEPTH)
    hole_profiles: list[list[tuple[float, float]]] = []
    slot_width = 0.102
    slot_height = 0.018
    x_centers = (-0.315, -0.189, -0.063, 0.063, 0.189, 0.315)
    y_centers = (-0.144, -0.048, 0.048, 0.144)
    for row_index, y_center in enumerate(y_centers):
        row_shift = 0.018 if row_index % 2 else 0.0
        for x_center in x_centers:
            shifted_x = x_center + row_shift
            if abs(shifted_x) + slot_width * 0.5 <= FILTER_WIDTH * 0.5 - 0.020:
                hole_profiles.append(
                    _rect_profile(slot_width, slot_height, center=(shifted_x, y_center))
                )

    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=FILTER_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("range_hood_grease_filter.obj"))


def _aabb_size(aabb):
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _aabb_center(aabb):
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.83, 1.0))
    filter_dark = model.material("filter_dark", rgba=(0.30, 0.31, 0.33, 1.0))
    button_black = model.material("button_black", rgba=(0.10, 0.10, 0.11, 1.0))

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - CANOPY_TOP_THICKNESS * 0.5)),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        Box((CANOPY_SIDE_THICKNESS, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(
                -CANOPY_WIDTH * 0.5 + CANOPY_SIDE_THICKNESS * 0.5,
                0.0,
                CANOPY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="left_skirt",
    )
    hood_body.visual(
        Box((CANOPY_SIDE_THICKNESS, CANOPY_DEPTH, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(
                CANOPY_WIDTH * 0.5 - CANOPY_SIDE_THICKNESS * 0.5,
                0.0,
                CANOPY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="right_skirt",
    )
    hood_body.visual(
        Box((CANOPY_WIDTH, CANOPY_BACK_THICKNESS, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CANOPY_DEPTH * 0.5 + CANOPY_BACK_THICKNESS * 0.5,
                CANOPY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="back_panel",
    )
    hood_body.visual(
        _build_front_panel_mesh(),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_FACE_Y - FRONT_PANEL_THICKNESS * 0.5,
                CANOPY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_panel",
    )
    hood_body.visual(
        _build_grease_filter_mesh(),
        origin=Origin(xyz=(0.0, -0.015, 0.009)),
        material=filter_dark,
        name="grease_filter",
    )
    hood_body.visual(
        Box((FILTER_SIDE_RAIL_WIDTH, FILTER_DEPTH, FILTER_THICKNESS)),
        origin=Origin(
            xyz=(
                -CANOPY_WIDTH * 0.5 + CANOPY_SIDE_THICKNESS + FILTER_SIDE_RAIL_WIDTH * 0.5,
                -0.015,
                0.009,
            )
        ),
        material=filter_dark,
        name="left_filter_rail",
    )
    hood_body.visual(
        Box((FILTER_SIDE_RAIL_WIDTH, FILTER_DEPTH, FILTER_THICKNESS)),
        origin=Origin(
            xyz=(
                CANOPY_WIDTH * 0.5 - CANOPY_SIDE_THICKNESS - FILTER_SIDE_RAIL_WIDTH * 0.5,
                -0.015,
                0.009,
            )
        ),
        material=filter_dark,
        name="right_filter_rail",
    )
    hood_body.visual(
        Box((FILTER_WIDTH, FILTER_FRONT_RAIL_DEPTH, FILTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.015 + FILTER_DEPTH * 0.5 - FILTER_FRONT_RAIL_DEPTH * 0.5,
                0.009,
            )
        ),
        material=filter_dark,
        name="front_filter_rail",
    )
    hood_body.visual(
        Box((FILTER_WIDTH, FILTER_FRONT_RAIL_DEPTH, FILTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.015 - FILTER_DEPTH * 0.5 + FILTER_FRONT_RAIL_DEPTH * 0.5,
                0.009,
            )
        ),
        material=filter_dark,
        name="rear_filter_rail",
    )
    hood_body.visual(
        Box((0.14, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, CANOPY_DEPTH * 0.5 - 0.070, 0.008)),
        material=filter_dark,
        name="filter_handle",
    )

    chimney_center_z = CHIMNEY_BOTTOM_Z + CHIMNEY_HEIGHT * 0.5
    hood_body.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_WALL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_DEPTH * 0.5 - CHIMNEY_WALL_THICKNESS * 0.5,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_WALL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CHIMNEY_DEPTH * 0.5 + CHIMNEY_WALL_THICKNESS * 0.5,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((CHIMNEY_WALL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                -CHIMNEY_WIDTH * 0.5 + CHIMNEY_WALL_THICKNESS * 0.5,
                0.0,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    hood_body.visual(
        Box((CHIMNEY_WALL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                CHIMNEY_WIDTH * 0.5 - CHIMNEY_WALL_THICKNESS * 0.5,
                0.0,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    hood_body.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CHIMNEY_BOTTOM_Z + CHIMNEY_HEIGHT - CHIMNEY_WALL_THICKNESS * 0.5,
            )
        ),
        material=stainless,
        name="chimney_top",
    )

    for spec in BUTTON_SPECS:
        button = model.part(spec["part_name"])
        button.visual(
            Box((BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)),
            origin=Origin(xyz=(0.0, BUTTON_CAP_DEPTH * 0.5, 0.0)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH * 0.5, 0.0)),
            material=button_black,
            name="button_stem",
        )
        model.articulation(
            spec["joint_name"],
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(spec["x"], FRONT_FACE_Y + BUTTON_TRAVEL, spec["z"])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    front_panel = hood_body.get_visual("front_panel")
    chimney_front = hood_body.get_visual("chimney_front")
    button_parts = [object_model.get_part(spec["part_name"]) for spec in BUTTON_SPECS]
    button_joints = [object_model.get_articulation(spec["joint_name"]) for spec in BUTTON_SPECS]
    button_caps = [part.get_visual("button_cap") for part in button_parts]
    button_stems = [part.get_visual("button_stem") for part in button_parts]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    hood_aabb = ctx.part_world_aabb(hood_body)
    if hood_aabb is None:
        ctx.fail("hood_body_has_geometry", "hood_body has no measurable geometry")
    else:
        hood_width, hood_depth, hood_height = _aabb_size(hood_aabb)
        ctx.check("hood_width_realistic", 0.85 <= hood_width <= 0.95, f"width={hood_width:.3f} m")
        ctx.check("hood_depth_realistic", 0.47 <= hood_depth <= 0.53, f"depth={hood_depth:.3f} m")
        ctx.check("hood_height_realistic", 0.79 <= hood_height <= 0.83, f"height={hood_height:.3f} m")

    front_panel_aabb = ctx.part_element_world_aabb(hood_body, elem=front_panel)
    chimney_front_aabb = ctx.part_element_world_aabb(hood_body, elem=chimney_front)
    if front_panel_aabb is None or chimney_front_aabb is None:
        ctx.fail("named_visual_aabbs_available", "front_panel or chimney_front AABB could not be resolved")
    else:
        front_width = _aabb_size(front_panel_aabb)[0]
        chimney_width = _aabb_size(chimney_front_aabb)[0]
        front_center = _aabb_center(front_panel_aabb)
        chimney_center = _aabb_center(chimney_front_aabb)
        ctx.check(
            "chimney_narrower_than_canopy",
            chimney_width < front_width * 0.40,
            f"chimney_width={chimney_width:.3f}, canopy_width={front_width:.3f}",
        )
        ctx.check(
            "chimney_centered_over_canopy",
            abs(chimney_center[0] - front_center[0]) <= 0.01,
            f"center_dx={abs(chimney_center[0] - front_center[0]):.4f}",
        )

    rest_positions = {
        part.name: ctx.part_world_position(part)
        for part in button_parts
    }
    upper_positions = [
        rest_positions[spec["part_name"]]
        for spec in BUTTON_SPECS
        if spec["row"] == "upper"
    ]
    lower_positions = [
        rest_positions[spec["part_name"]]
        for spec in BUTTON_SPECS
        if spec["row"] == "lower"
    ]
    if all(position is not None for position in upper_positions + lower_positions):
        ctx.check(
            "upper_row_above_lower_row",
            min(position[2] for position in upper_positions) > max(position[2] for position in lower_positions),
            "upper button row should sit above lower button row",
        )
        for stagger_index in range(3):
            ctx.check(
                f"staggered_button_layout_{stagger_index}",
                upper_positions[stagger_index][0] < lower_positions[stagger_index][0] < upper_positions[stagger_index + 1][0],
                (
                    f"lower_x={lower_positions[stagger_index][0]:.4f} should lie between "
                    f"{upper_positions[stagger_index][0]:.4f} and {upper_positions[stagger_index + 1][0]:.4f}"
                ),
            )
    else:
        ctx.fail("button_positions_available", "could not resolve all button world positions")

    for part, joint, cap, stem in zip(button_parts, button_joints, button_caps, button_stems):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis",
            joint.articulation_type == ArticulationType.PRISMATIC and joint.axis == (0.0, -1.0, 0.0),
            f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.check(
            f"{joint.name}_travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.0030,
            (
                "expected short button plunger travel in [0.0015, 0.0030] m; "
                f"got {None if limits is None else limits.upper}"
            ),
        )
        with ctx.pose({joint: 0.0}):
            ctx.expect_contact(
                part,
                hood_body,
                elem_a=stem,
                elem_b=front_panel,
                name=f"{part.name}_stem_contacts_panel_at_rest",
            )
            ctx.expect_gap(
                part,
                hood_body,
                axis="y",
                positive_elem=cap,
                negative_elem=front_panel,
                min_gap=BUTTON_TRAVEL - 0.0002,
                max_gap=BUTTON_TRAVEL + 0.0002,
                name=f"{part.name}_cap_proud_at_rest",
            )
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_position = ctx.part_world_position(part)
            rest_position = rest_positions[part.name]
            ctx.expect_contact(
                part,
                hood_body,
                elem_a=stem,
                elem_b=front_panel,
                name=f"{part.name}_stem_contacts_panel_when_pressed",
            )
            ctx.expect_gap(
                part,
                hood_body,
                axis="y",
                positive_elem=cap,
                negative_elem=front_panel,
                min_gap=0.0,
                max_gap=0.0002,
                name=f"{part.name}_cap_flush_when_pressed",
            )
            ctx.check(
                f"{joint.name}_moves_inward_only",
                pressed_position is not None
                and rest_position is not None
                and pressed_position[1] <= rest_position[1] - 0.0018
                and abs(pressed_position[0] - rest_position[0]) <= 1e-6
                and abs(pressed_position[2] - rest_position[2]) <= 1e-6,
                f"rest={rest_position}, pressed={pressed_position}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
