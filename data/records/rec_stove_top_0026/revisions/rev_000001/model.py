from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_hob_open_shelving", assets=ASSETS)

    stone = model.material("stone_quartz", rgba=(0.78, 0.79, 0.77, 1.0))
    wood = model.material("oak_veneer", rgba=(0.63, 0.50, 0.36, 1.0))
    hob_glass_mat = model.material("hob_glass", rgba=(0.07, 0.07, 0.08, 0.95))
    zone_mark = model.material("zone_mark", rgba=(0.42, 0.44, 0.47, 0.42))
    body_metal = model.material("body_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    knob_metal = model.material("knob_metal", rgba=(0.18, 0.18, 0.19, 1.0))
    knob_indicator = model.material("knob_indicator", rgba=(0.84, 0.84, 0.86, 1.0))

    counter_width = 0.90
    counter_depth = 0.60
    counter_thickness = 0.04
    counter_top_z = 0.76
    counter_center_z = counter_top_z - counter_thickness * 0.5

    cutout_width = 0.56
    cutout_depth = 0.49
    hob_center_y = 0.02

    cabinet_height = counter_top_z - counter_thickness
    side_thickness = 0.018
    carcass_depth = 0.58
    inner_width = counter_width - 2.0 * side_thickness
    shelf_depth = 0.556
    shelf_thickness = 0.022

    glass_width = 0.60
    glass_depth = 0.52
    glass_thickness = 0.006

    body_width = 0.54
    body_depth = 0.48
    body_height = 0.05

    panel_width = 0.50
    panel_height = 0.10
    panel_thickness = 0.012

    knob_x = 0.182
    knob_z = 0.025
    knob_hole_radius = 0.009
    knob_shaft_radius = 0.0082
    knob_shaft_length = 0.020
    knob_flange_radius = 0.015
    knob_flange_length = 0.004
    knob_body_radius = 0.021
    knob_body_length = 0.018

    def _rect_profile(width: float, height: float, *, center=(0.0, 0.0)) -> list[tuple[float, float]]:
        cx, cy = center
        half_w = width * 0.5
        half_h = height * 0.5
        return [
            (cx - half_w, cy - half_h),
            (cx + half_w, cy - half_h),
            (cx + half_w, cy + half_h),
            (cx - half_w, cy + half_h),
        ]

    def _circle_profile(
        radius: float,
        *,
        center: tuple[float, float] = (0.0, 0.0),
        segments: int = 28,
    ) -> list[tuple[float, float]]:
        cx, cy = center
        return [
            (
                cx + radius * math.cos(2.0 * math.pi * index / segments),
                cy + radius * math.sin(2.0 * math.pi * index / segments),
            )
            for index in range(segments)
        ]

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, thickness: float):
        return _save_mesh(
            name,
            ExtrudeWithHolesGeometry(
                _circle_profile(outer_radius, segments=48),
                [_circle_profile(inner_radius, segments=40)],
                height=thickness,
                center=True,
            ),
        )

    countertop = model.part("countertop")
    counter_geom = ExtrudeWithHolesGeometry(
        _rect_profile(counter_width, counter_depth),
        [_rect_profile(cutout_width, cutout_depth, center=(0.0, hob_center_y))],
        height=counter_thickness,
        center=True,
    )
    countertop.visual(
        _save_mesh("countertop_ring.obj", counter_geom),
        origin=Origin(xyz=(0.0, 0.0, counter_center_z)),
        material=stone,
        name="counter_slab",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((counter_width, counter_depth, counter_thickness)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, counter_center_z)),
    )

    left_panel = model.part("left_panel")
    left_panel.visual(
        Box((side_thickness, carcass_depth, cabinet_height)),
        material=wood,
        name="left_side",
    )
    left_panel.inertial = Inertial.from_geometry(
        Box((side_thickness, carcass_depth, cabinet_height)),
        mass=10.0,
    )

    right_panel = model.part("right_panel")
    right_panel.visual(
        Box((side_thickness, carcass_depth, cabinet_height)),
        material=wood,
        name="right_side",
    )
    right_panel.inertial = Inertial.from_geometry(
        Box((side_thickness, carcass_depth, cabinet_height)),
        mass=10.0,
    )

    back_panel = model.part("back_panel")
    back_panel.visual(
        Box((inner_width, 0.012, cabinet_height)),
        material=wood,
        name="back_board",
    )
    back_panel.inertial = Inertial.from_geometry(
        Box((inner_width, 0.012, cabinet_height)),
        mass=8.0,
    )

    bottom_shelf = model.part("bottom_shelf")
    bottom_shelf.visual(
        Box((inner_width, shelf_depth, shelf_thickness)),
        material=wood,
        name="bottom_board",
    )
    bottom_shelf.inertial = Inertial.from_geometry(
        Box((inner_width, shelf_depth, shelf_thickness)),
        mass=9.0,
    )

    middle_shelf = model.part("middle_shelf")
    middle_shelf.visual(
        Box((inner_width, shelf_depth, shelf_thickness)),
        material=wood,
        name="middle_board",
    )
    middle_shelf.inertial = Inertial.from_geometry(
        Box((inner_width, shelf_depth, shelf_thickness)),
        mass=7.5,
    )

    hob_glass = model.part("hob_glass")
    hob_glass.visual(
        Box((glass_width, glass_depth, glass_thickness)),
        material=hob_glass_mat,
        name="glass_surface",
    )
    small_zone_ring = _annulus_mesh(
        "zone_ring_small.obj",
        outer_radius=0.080,
        inner_radius=0.074,
        thickness=0.0005,
    )
    large_zone_ring = _annulus_mesh(
        "zone_ring_large.obj",
        outer_radius=0.095,
        inner_radius=0.088,
        thickness=0.0005,
    )
    zone_specs = [
        ("zone_front_left", (-0.13, -0.10, small_zone_ring)),
        ("zone_front_right", (0.13, -0.10, small_zone_ring)),
        ("zone_rear_left", (-0.13, 0.11, large_zone_ring)),
        ("zone_rear_right", (0.13, 0.11, large_zone_ring)),
    ]
    for zone_name, (x_pos, y_pos, zone_mesh) in zone_specs:
        hob_glass.visual(
            zone_mesh,
            origin=Origin(
                xyz=(x_pos, y_pos, glass_thickness * 0.5 + 0.00025),
            ),
            material=zone_mark,
            name=zone_name,
        )
        hob_glass.visual(
            Box((0.040, 0.0012, 0.0005)),
            origin=Origin(
                xyz=(x_pos, y_pos - 0.055, glass_thickness * 0.5 + 0.00025),
            ),
            material=zone_mark,
            name=f"{zone_name}_index_mark",
        )
    hob_glass.inertial = Inertial.from_geometry(
        Box((glass_width, glass_depth, glass_thickness)),
        mass=5.0,
    )

    hob_body = model.part("hob_body")
    hob_body.visual(
        Box((body_width, body_depth, body_height)),
        material=body_metal,
        name="body_shell",
    )
    hob_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=6.5,
    )

    control_panel = model.part("control_panel")
    panel_geom = ExtrudeWithHolesGeometry(
        _rect_profile(panel_width, panel_height),
        [
            _circle_profile(knob_hole_radius, center=(-knob_x, knob_z)),
            _circle_profile(knob_hole_radius, center=(-knob_x, -knob_z)),
            _circle_profile(knob_hole_radius, center=(knob_x, knob_z)),
            _circle_profile(knob_hole_radius, center=(knob_x, -knob_z)),
        ],
        height=panel_thickness,
        center=True,
    ).rotate_x(math.pi * 0.5)
    control_panel.visual(
        _save_mesh("hob_control_panel.obj", panel_geom),
        material=body_metal,
        name="panel_shell",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness, panel_height)),
        mass=1.2,
    )

    def _build_knob(part_name: str) -> None:
        knob_part = model.part(part_name)
        knob_part.visual(
            Cylinder(radius=knob_shaft_radius, length=knob_shaft_length),
            origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_metal,
            name="shaft",
        )
        knob_part.visual(
            Cylinder(radius=knob_flange_radius, length=knob_flange_length),
            origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_metal,
            name="flange",
        )
        knob_part.visual(
            Cylinder(radius=knob_body_radius, length=knob_body_length),
            origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_metal,
            name="body",
        )
        knob_part.visual(
            Box((0.004, 0.002, 0.014)),
            origin=Origin(xyz=(0.0, -0.029, 0.010)),
            material=knob_indicator,
            name="indicator",
        )
        knob_part.inertial = Inertial.from_geometry(
            Box((0.050, 0.044, 0.044)),
            mass=0.12,
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
        )

    for knob_name in (
        "knob_left_upper",
        "knob_left_lower",
        "knob_right_upper",
        "knob_right_lower",
    ):
        _build_knob(knob_name)

    def _fixed(name: str, parent, child, xyz: tuple[float, float, float]) -> None:
        model.articulation(
            name,
            ArticulationType.FIXED,
            parent=parent,
            child=child,
            origin=Origin(xyz=xyz),
        )

    _fixed(
        "countertop_to_left_panel",
        countertop,
        left_panel,
        (-counter_width * 0.5 + side_thickness * 0.5, 0.0, cabinet_height * 0.5),
    )
    _fixed(
        "countertop_to_right_panel",
        countertop,
        right_panel,
        (counter_width * 0.5 - side_thickness * 0.5, 0.0, cabinet_height * 0.5),
    )
    _fixed(
        "countertop_to_back_panel",
        countertop,
        back_panel,
        (0.0, carcass_depth * 0.5 - 0.006, cabinet_height * 0.5),
    )
    _fixed(
        "left_panel_to_bottom_shelf",
        left_panel,
        bottom_shelf,
        (side_thickness * 0.5 + inner_width * 0.5, 0.0, -cabinet_height * 0.5 + shelf_thickness * 0.5),
    )
    _fixed(
        "left_panel_to_middle_shelf",
        left_panel,
        middle_shelf,
        (side_thickness * 0.5 + inner_width * 0.5, 0.0, -0.01),
    )
    _fixed(
        "countertop_to_hob_glass",
        countertop,
        hob_glass,
        (0.0, hob_center_y, counter_top_z + glass_thickness * 0.5),
    )
    _fixed(
        "countertop_to_hob_body",
        countertop,
        hob_body,
        (0.0, hob_center_y, counter_top_z - body_height * 0.5),
    )
    _fixed(
        "hob_body_to_control_panel",
        hob_body,
        control_panel,
        (0.0, -0.234, -(body_height * 0.5 + panel_height * 0.5)),
    )

    knob_positions = {
        "knob_left_upper": (-knob_x, knob_z),
        "knob_left_lower": (-knob_x, -knob_z),
        "knob_right_upper": (knob_x, knob_z),
        "knob_right_lower": (knob_x, -knob_z),
    }
    for knob_name, (x_pos, z_pos) in knob_positions.items():
        model.articulation(
            f"control_panel_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=control_panel,
            child=knob_name,
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=10.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")
    back_panel = object_model.get_part("back_panel")
    bottom_shelf = object_model.get_part("bottom_shelf")
    middle_shelf = object_model.get_part("middle_shelf")
    hob_glass = object_model.get_part("hob_glass")
    hob_body = object_model.get_part("hob_body")
    control_panel = object_model.get_part("control_panel")
    knob_left_upper = object_model.get_part("knob_left_upper")
    knob_left_lower = object_model.get_part("knob_left_lower")
    knob_right_upper = object_model.get_part("knob_right_upper")
    knob_right_lower = object_model.get_part("knob_right_lower")

    knob_joints = {
        "control_panel_to_knob_left_upper": object_model.get_articulation("control_panel_to_knob_left_upper"),
        "control_panel_to_knob_left_lower": object_model.get_articulation("control_panel_to_knob_left_lower"),
        "control_panel_to_knob_right_upper": object_model.get_articulation("control_panel_to_knob_right_upper"),
        "control_panel_to_knob_right_lower": object_model.get_articulation("control_panel_to_knob_right_lower"),
    }

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

    ctx.expect_contact(left_panel, countertop)
    ctx.expect_contact(right_panel, countertop)
    ctx.expect_contact(back_panel, countertop)
    ctx.expect_contact(bottom_shelf, left_panel)
    ctx.expect_contact(bottom_shelf, right_panel)
    ctx.expect_contact(bottom_shelf, back_panel)
    ctx.expect_contact(middle_shelf, left_panel)
    ctx.expect_contact(middle_shelf, right_panel)
    ctx.expect_contact(middle_shelf, back_panel)

    ctx.expect_gap(hob_glass, countertop, axis="z", max_gap=0.0001, max_penetration=0.0)
    ctx.expect_gap(hob_glass, hob_body, axis="z", max_gap=0.0001, max_penetration=0.0)
    ctx.expect_gap(hob_body, control_panel, axis="z", max_gap=0.0001, max_penetration=0.0)
    ctx.expect_contact(knob_left_upper, control_panel)
    ctx.expect_contact(knob_left_lower, control_panel)
    ctx.expect_contact(knob_right_upper, control_panel)
    ctx.expect_contact(knob_right_lower, control_panel)

    left_upper_pos = ctx.part_world_position(knob_left_upper)
    left_lower_pos = ctx.part_world_position(knob_left_lower)
    right_upper_pos = ctx.part_world_position(knob_right_upper)
    right_lower_pos = ctx.part_world_position(knob_right_lower)
    control_panel_pos = ctx.part_world_position(control_panel)

    assert left_upper_pos is not None
    assert left_lower_pos is not None
    assert right_upper_pos is not None
    assert right_lower_pos is not None
    assert control_panel_pos is not None

    ctx.check(
        "left_pair_vertical_alignment",
        abs(left_upper_pos[0] - left_lower_pos[0]) < 1e-6
        and 0.045 < left_upper_pos[2] - left_lower_pos[2] < 0.055,
        details=f"left pair positions={left_upper_pos}, {left_lower_pos}",
    )
    ctx.check(
        "right_pair_vertical_alignment",
        abs(right_upper_pos[0] - right_lower_pos[0]) < 1e-6
        and 0.045 < right_upper_pos[2] - right_lower_pos[2] < 0.055,
        details=f"right pair positions={right_upper_pos}, {right_lower_pos}",
    )
    ctx.check(
        "left_right_knob_pair_spacing",
        0.34 < right_upper_pos[0] - left_upper_pos[0] < 0.38,
        details=f"upper pair x positions={left_upper_pos[0]}, {right_upper_pos[0]}",
    )
    ctx.check(
        "knobs_are_front_mounted",
        max(
            left_upper_pos[1],
            left_lower_pos[1],
            right_upper_pos[1],
            right_lower_pos[1],
        )
        < control_panel_pos[1] + 1e-6,
        details=(
            f"knob y positions={[left_upper_pos[1], left_lower_pos[1], right_upper_pos[1], right_lower_pos[1]]}, "
            f"panel={control_panel_pos[1]}"
        ),
    )

    for joint_name, joint in knob_joints.items():
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name}_axis_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{joint_name}_unbounded_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )

    with ctx.pose(
        {
            knob_joints["control_panel_to_knob_left_upper"]: math.pi * 0.5,
            knob_joints["control_panel_to_knob_left_lower"]: math.pi,
            knob_joints["control_panel_to_knob_right_upper"]: -math.pi * 0.5,
            knob_joints["control_panel_to_knob_right_lower"]: math.pi * 0.25,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_rotated_no_floating")
        ctx.expect_contact(knob_left_upper, control_panel, name="left_upper_knob_rotated_contact")
        ctx.expect_contact(knob_left_lower, control_panel, name="left_lower_knob_rotated_contact")
        ctx.expect_contact(knob_right_upper, control_panel, name="right_upper_knob_rotated_contact")
        ctx.expect_contact(knob_right_lower, control_panel, name="right_lower_knob_rotated_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
