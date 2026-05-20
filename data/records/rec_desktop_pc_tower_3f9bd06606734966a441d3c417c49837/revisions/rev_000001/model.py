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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_filter_mesh(width: float, depth: float, thickness: float):
    outer = rounded_rect_profile(width, depth, 0.010, corner_segments=8)
    slot = rounded_rect_profile(0.018, 0.050, 0.004, corner_segments=6)
    hole_profiles: list[list[tuple[float, float]]] = []
    for x_pos in (-0.065, -0.022, 0.022, 0.065):
        for y_pos in (-0.095, -0.035, 0.025, 0.085):
            hole_profiles.append(_shift_profile(slot, dx=x_pos, dy=y_pos))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer,
            hole_profiles,
            height=thickness,
            center=True,
        ),
        "pc_case_top_filter",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_pc_case")

    steel = model.material("powder_coat_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    interior = model.material("interior_black", rgba=(0.08, 0.09, 0.10, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.24, 0.29, 0.32, 0.33))
    clear_glass = model.material("side_glass", rgba=(0.40, 0.46, 0.50, 0.24))
    mesh_black = model.material("mesh_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    outer_width = 0.270
    outer_depth = 0.500
    foot_height = 0.012
    shell_height = 0.528
    top_z = foot_height + shell_height
    wall_thickness = 0.010
    floor_thickness = 0.008
    front_frame_depth = 0.022
    top_rail_height = 0.024
    left_bottom_rail_height = 0.034
    front_post_width = 0.018

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, top_z)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, top_z / 2.0)),
    )

    for x_pos in (-0.088, 0.088):
        for y_pos in (-0.170, 0.170):
            body.visual(
                Box((0.024, 0.060, foot_height)),
                origin=Origin(xyz=(x_pos, y_pos, foot_height / 2.0)),
                material=rubber,
                name=f"foot_{'l' if x_pos < 0 else 'r'}_{'f' if y_pos < 0 else 'r'}",
            )

    body.visual(
        Box((outer_width, outer_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + floor_thickness / 2.0)),
        material=steel,
        name="floor_pan",
    )
    body.visual(
        Box((wall_thickness, outer_depth, shell_height)),
        origin=Origin(
            xyz=(outer_width / 2.0 - wall_thickness / 2.0, 0.0, foot_height + shell_height / 2.0)
        ),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((outer_width, wall_thickness, shell_height)),
        origin=Origin(
            xyz=(0.0, outer_depth / 2.0 - wall_thickness / 2.0, foot_height + shell_height / 2.0)
        ),
        material=steel,
        name="rear_wall",
    )
    body.visual(
        Box((front_post_width, front_frame_depth, shell_height)),
        origin=Origin(
            xyz=(
                -outer_width / 2.0 + front_post_width / 2.0,
                -outer_depth / 2.0 + front_frame_depth / 2.0,
                foot_height + shell_height / 2.0,
            )
        ),
        material=steel,
        name="front_left_post",
    )
    body.visual(
        Box((front_post_width, front_frame_depth, shell_height)),
        origin=Origin(
            xyz=(
                outer_width / 2.0 - front_post_width / 2.0,
                -outer_depth / 2.0 + front_frame_depth / 2.0,
                foot_height + shell_height / 2.0,
            )
        ),
        material=steel,
        name="front_right_post",
    )
    body.visual(
        Box((outer_width, front_frame_depth, top_rail_height)),
        origin=Origin(
            xyz=(0.0, -outer_depth / 2.0 + front_frame_depth / 2.0, top_z - top_rail_height / 2.0)
        ),
        material=steel,
        name="top_front_rail",
    )
    body.visual(
        Box((wall_thickness, outer_depth, top_rail_height)),
        origin=Origin(
            xyz=(-outer_width / 2.0 + wall_thickness / 2.0, 0.0, top_z - top_rail_height / 2.0)
        ),
        material=steel,
        name="left_top_rail",
    )
    body.visual(
        Box((wall_thickness, outer_depth, left_bottom_rail_height)),
        origin=Origin(
            xyz=(
                -outer_width / 2.0 + wall_thickness / 2.0,
                0.0,
                foot_height + left_bottom_rail_height / 2.0,
            )
        ),
        material=steel,
        name="left_bottom_rail",
    )
    body.visual(
        Box((wall_thickness, 0.050, 0.446)),
        origin=Origin(
            xyz=(
                -outer_width / 2.0 + wall_thickness / 2.0,
                0.225,
                0.275,
            )
        ),
        material=steel,
        name="side_hinge_post",
    )
    body.visual(
        Box((0.016, 0.338, 0.010)),
        origin=Origin(xyz=(-0.117, -0.019, top_z - 0.005)),
        material=steel,
        name="top_filter_left_ledge",
    )
    body.visual(
        Box((0.016, 0.338, 0.010)),
        origin=Origin(xyz=(0.117, -0.019, top_z - 0.005)),
        material=steel,
        name="top_filter_right_ledge",
    )
    body.visual(
        Box((outer_width - 0.030, 0.012, 0.390)),
        origin=Origin(xyz=(0.0, -0.225, 0.278)),
        material=interior,
        name="front_fan_bracket",
    )
    for idx, fan_z in enumerate((0.154, 0.274, 0.394), start=1):
        body.visual(
            Cylinder(radius=0.052, length=0.012),
            origin=Origin(xyz=(0.0, -0.221, fan_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mesh_black,
            name=f"front_fan_{idx}",
        )
    body.visual(
        Box((outer_width - 0.038, 0.212, 0.180)),
        origin=Origin(xyz=(0.0, 0.120, 0.102)),
        material=steel,
        name="psu_shroud",
    )
    body.visual(
        Box((0.146, 0.012, 0.146)),
        origin=Origin(xyz=(-0.030, 0.244, 0.395)),
        material=interior,
        name="rear_exhaust_frame",
    )
    body.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(-0.030, 0.241, 0.395), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mesh_black,
        name="rear_exhaust_fan",
    )
    body.visual(
        Box((0.086, 0.012, 0.114)),
        origin=Origin(xyz=(0.080, 0.244, 0.194)),
        material=steel,
        name="pci_slot_stack",
    )

    door_width = 0.246
    door_height = 0.498
    front_door = model.part("front_door")
    front_door.visual(
        Box((door_width, 0.004, door_height)),
        origin=Origin(xyz=(-door_width / 2.0, 0.002, door_height / 2.0)),
        material=smoked_glass,
        name="door_glass",
    )
    front_door.visual(
        Box((0.020, 0.008, door_height)),
        origin=Origin(xyz=(-0.010, 0.0, door_height / 2.0)),
        material=steel,
        name="hinge_spine",
    )
    front_door.visual(
        Box((door_width, 0.008, 0.012)),
        origin=Origin(xyz=(-door_width / 2.0, 0.0, 0.006)),
        material=steel,
        name="door_bottom_cap",
    )
    front_door.visual(
        Box((door_width, 0.008, 0.012)),
        origin=Origin(xyz=(-door_width / 2.0, 0.0, door_height - 0.006)),
        material=steel,
        name="door_top_cap",
    )
    front_door.visual(
        Box((0.008, 0.012, 0.170)),
        origin=Origin(xyz=(-door_width + 0.004, -0.002, 0.285)),
        material=steel,
        name="door_latch_bar",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.012, door_height)),
        mass=3.4,
        origin=Origin(xyz=(-door_width / 2.0, 0.0, door_height / 2.0)),
    )

    side_panel_depth = 0.452
    side_panel_height = 0.446
    side_panel = model.part("side_glass_panel")
    side_panel.visual(
        Box((0.004, side_panel_depth, side_panel_height)),
        origin=Origin(xyz=(0.002, -side_panel_depth / 2.0, side_panel_height / 2.0)),
        material=clear_glass,
        name="side_glass",
    )
    side_panel.visual(
        Box((0.008, side_panel_depth, 0.012)),
        origin=Origin(xyz=(0.0, -side_panel_depth / 2.0, 0.006)),
        material=steel,
        name="side_bottom_frame",
    )
    side_panel.visual(
        Box((0.008, side_panel_depth, 0.012)),
        origin=Origin(xyz=(0.0, -side_panel_depth / 2.0, side_panel_height - 0.006)),
        material=steel,
        name="side_top_frame",
    )
    side_panel.visual(
        Box((0.008, 0.022, side_panel_height)),
        origin=Origin(xyz=(0.0, -0.011, side_panel_height / 2.0)),
        material=steel,
        name="rear_hinge_rail",
    )
    side_panel.visual(
        Box((0.010, 0.020, 0.140)),
        origin=Origin(xyz=(-0.001, -side_panel_depth + 0.010, 0.255)),
        material=steel,
        name="side_latch_tab",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((0.008, side_panel_depth, side_panel_height)),
        mass=3.1,
        origin=Origin(xyz=(0.0, -side_panel_depth / 2.0, side_panel_height / 2.0)),
    )

    filter_width = 0.218
    filter_depth = 0.338
    filter_thickness = 0.004
    top_filter = model.part("top_filter_panel")
    top_filter.visual(
        _build_filter_mesh(filter_width, filter_depth, filter_thickness),
        origin=Origin(xyz=(0.0, filter_depth / 2.0, 0.0)),
        material=mesh_black,
        name="filter_mesh",
    )
    top_filter.visual(
        Box((0.050, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, filter_depth - 0.012, 0.003)),
        material=steel,
        name="filter_pull_tab",
    )
    top_filter.inertial = Inertial.from_geometry(
        Box((filter_width, filter_depth, 0.010)),
        mass=0.45,
        origin=Origin(xyz=(0.0, filter_depth / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_door,
        origin=Origin(xyz=(outer_width / 2.0, -outer_depth / 2.0 - 0.004, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "body_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_panel,
        origin=Origin(xyz=(-outer_width / 2.0 - 0.004, 0.228, 0.052)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "body_to_top_filter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_filter,
        origin=Origin(xyz=(0.0, -0.188, top_z + filter_thickness / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_door = object_model.get_part("front_door")
    side_panel = object_model.get_part("side_glass_panel")
    top_filter = object_model.get_part("top_filter_panel")

    front_hinge = object_model.get_articulation("body_to_front_door")
    side_hinge = object_model.get_articulation("body_to_side_panel")
    top_hinge = object_model.get_articulation("body_to_top_filter")

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

    ctx.expect_gap(
        body,
        front_door,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="front_door_seats_flush",
    )
    ctx.expect_overlap(
        front_door,
        body,
        axes="xz",
        min_overlap=0.20,
        name="front_door_covers_case_front",
    )
    ctx.expect_gap(
        body,
        side_panel,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="side_panel_seats_flush",
    )
    ctx.expect_overlap(
        side_panel,
        body,
        axes="yz",
        min_overlap=0.20,
        name="side_panel_covers_side_opening",
    )
    ctx.expect_gap(
        top_filter,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="top_filter_sits_on_top_rail",
    )
    ctx.expect_overlap(
        top_filter,
        body,
        axes="xy",
        min_overlap=0.18,
        name="top_filter_covers_top_vent",
    )

    ctx.check(
        "front_hinge_axis_is_vertical",
        front_hinge.axis == (0.0, 0.0, 1.0),
        details=f"Expected (0, 0, 1), got {front_hinge.axis}",
    )
    ctx.check(
        "side_hinge_axis_is_vertical_rear_edge",
        side_hinge.axis == (0.0, 0.0, -1.0),
        details=f"Expected (0, 0, -1), got {side_hinge.axis}",
    )
    ctx.check(
        "top_filter_hinge_axis_runs_left_right",
        top_hinge.axis == (1.0, 0.0, 0.0),
        details=f"Expected (1, 0, 0), got {top_hinge.axis}",
    )
    ctx.check(
        "front_door_open_limit_is_wide",
        front_hinge.motion_limits is not None and front_hinge.motion_limits.upper is not None and front_hinge.motion_limits.upper >= 1.9,
        details="Front glass door should open to at least about 109 degrees.",
    )
    ctx.check(
        "side_panel_open_limit_is_wide",
        side_hinge.motion_limits is not None and side_hinge.motion_limits.upper is not None and side_hinge.motion_limits.upper >= 1.7,
        details="Side glass panel should swing substantially outward.",
    )
    ctx.check(
        "top_filter_open_limit_is_serviceable",
        top_hinge.motion_limits is not None and top_hinge.motion_limits.upper is not None and 1.1 <= top_hinge.motion_limits.upper <= 1.4,
        details="Top filter should tip open to a realistic service angle.",
    )

    front_rest = ctx.part_world_aabb(front_door)
    side_rest = ctx.part_world_aabb(side_panel)
    filter_rest = ctx.part_world_aabb(top_filter)

    if front_rest is None:
        ctx.fail("front_door_rest_aabb_available", "Front door AABB missing in rest pose.")
    else:
        with ctx.pose({front_hinge: math.radians(100)}):
            front_open = ctx.part_world_aabb(front_door)
            if front_open is None:
                ctx.fail("front_door_open_aabb_available", "Front door AABB missing in open pose.")
            else:
                ctx.check(
                    "front_door_swings_outboard",
                    front_open[0][0] > front_rest[0][0] + 0.20,
                    details=f"Expected open door xmax to increase by > 0.10 m, got rest={front_rest}, open={front_open}",
                )

    if side_rest is None:
        ctx.fail("side_panel_rest_aabb_available", "Side panel AABB missing in rest pose.")
    else:
        with ctx.pose({side_hinge: math.radians(100)}):
            side_open = ctx.part_world_aabb(side_panel)
            if side_open is None:
                ctx.fail("side_panel_open_aabb_available", "Side panel AABB missing in open pose.")
            else:
                ctx.check(
                    "side_panel_swings_left_outward",
                    side_open[0][0] < side_rest[0][0] - 0.10,
                    details=f"Expected open panel xmin to decrease by > 0.10 m, got rest={side_rest}, open={side_open}",
                )

    if filter_rest is None:
        ctx.fail("top_filter_rest_aabb_available", "Top filter AABB missing in rest pose.")
    else:
        with ctx.pose({top_hinge: math.radians(65)}):
            filter_open = ctx.part_world_aabb(top_filter)
            if filter_open is None:
                ctx.fail("top_filter_open_aabb_available", "Top filter AABB missing in open pose.")
            else:
                ctx.check(
                    "top_filter_lifts_up",
                    filter_open[1][2] > filter_rest[1][2] + 0.12,
                    details=f"Expected open filter zmax to increase by > 0.12 m, got rest={filter_rest}, open={filter_open}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
