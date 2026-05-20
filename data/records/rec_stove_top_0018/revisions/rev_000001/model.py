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


def _circle_profile(radius: float, *, segments: int = 20) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_cooktop_cabinet", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.93, 0.91, 1.0))
    toe_black = model.material("toe_black", rgba=(0.18, 0.18, 0.17, 1.0))
    counter_stone = model.material("counter_stone", rgba=(0.62, 0.64, 0.66, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.08, 0.08, 0.09, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.33, 0.36, 0.39, 0.95))
    switch_body = model.material("switch_body", rgba=(0.14, 0.14, 0.15, 1.0))
    button_metal = model.material("button_metal", rgba=(0.82, 0.83, 0.84, 1.0))
    pull_metal = model.material("pull_metal", rgba=(0.73, 0.74, 0.76, 1.0))

    cabinet_width = 0.88
    cabinet_depth = 0.56
    cabinet_height = 0.86
    side_thickness = 0.018
    face_thickness = 0.018
    back_thickness = 0.012
    floor_thickness = 0.018
    toe_height = 0.10
    toe_setback = 0.06
    opening_width = 0.77
    opening_bottom = 0.12
    opening_top = 0.79
    door_height = 0.664
    door_width = 0.762
    door_thickness = 0.018

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((side_thickness, cabinet_depth, cabinet_height - toe_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + side_thickness / 2.0,
                0.0,
                toe_height + (cabinet_height - toe_height) / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet_body.visual(
        Box((side_thickness, cabinet_depth, cabinet_height - toe_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - side_thickness / 2.0,
                0.0,
                toe_height + (cabinet_height - toe_height) / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet_body.visual(
        Box((cabinet_width - 2.0 * side_thickness + 0.004, back_thickness, cabinet_height - toe_height)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - back_thickness / 2.0,
                toe_height + (cabinet_height - toe_height) / 2.0,
            )
        ),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet_body.visual(
        Box((cabinet_width - 2.0 * side_thickness + 0.002, cabinet_depth - 0.06, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.03, toe_height + floor_thickness / 2.0)),
        material=cabinet_white,
        name="cabinet_floor",
    )
    cabinet_body.visual(
        Box((0.055, face_thickness, opening_top - opening_bottom)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 + 0.055 / 2.0),
                -cabinet_depth / 2.0 + face_thickness / 2.0,
                (opening_bottom + opening_top) / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_stile",
    )
    cabinet_body.visual(
        Box((0.055, face_thickness, opening_top - opening_bottom)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + 0.055 / 2.0,
                -cabinet_depth / 2.0 + face_thickness / 2.0,
                (opening_bottom + opening_top) / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_stile",
    )
    cabinet_body.visual(
        Box((opening_width, face_thickness, cabinet_height - opening_top)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + face_thickness / 2.0,
                opening_top + (cabinet_height - opening_top) / 2.0,
            )
        ),
        material=cabinet_white,
        name="top_rail",
    )
    cabinet_body.visual(
        Box((opening_width, face_thickness, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + 0.004,
                toe_height + 0.005,
            )
        ),
        material=cabinet_white,
        name="bottom_rail",
    )
    hinge_barrel_span = door_width - 0.080
    hinge_pin_span = door_width + 0.040
    hinge_cheek_width = 0.028
    hinge_cheek_center_x = hinge_pin_span / 2.0 + hinge_cheek_width / 2.0
    cabinet_body.visual(
        Box((hinge_cheek_width, 0.014, 0.024)),
        origin=Origin(
            xyz=(
                -hinge_cheek_center_x,
                -cabinet_depth / 2.0 - 0.003,
                opening_bottom + 0.002,
            )
        ),
        material=cabinet_white,
        name="left_hinge_support",
    )
    cabinet_body.visual(
        Box((hinge_cheek_width, 0.014, 0.024)),
        origin=Origin(
            xyz=(
                hinge_cheek_center_x,
                -cabinet_depth / 2.0 - 0.003,
                opening_bottom + 0.002,
            )
        ),
        material=cabinet_white,
        name="right_hinge_support",
    )
    cabinet_body.visual(
        Cylinder(radius=0.0035, length=hinge_pin_span),
        origin=Origin(
            xyz=(0.0, -cabinet_depth / 2.0 - 0.003, opening_bottom),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pull_metal,
        name="hinge_pin",
    )
    cabinet_body.visual(
        Box((cabinet_width - 0.12, face_thickness, toe_height)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + toe_setback + face_thickness / 2.0,
                toe_height / 2.0,
            )
        ),
        material=toe_black,
        name="toe_kick",
    )
    cabinet_body.visual(
        Box((0.06, toe_setback, toe_height)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + 0.03,
                -cabinet_depth / 2.0 + toe_setback / 2.0,
                toe_height / 2.0,
            )
        ),
        material=toe_black,
        name="left_toe_return",
    )
    cabinet_body.visual(
        Box((0.06, toe_setback, toe_height)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - 0.03,
                -cabinet_depth / 2.0 + toe_setback / 2.0,
                toe_height / 2.0,
            )
        ),
        material=toe_black,
        name="right_toe_return",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    countertop_width = 0.92
    countertop_depth = 0.62
    countertop_thickness = 0.04
    counter_opening_width = 0.74
    counter_opening_depth = 0.50
    side_strip_width = (countertop_width - counter_opening_width) / 2.0
    end_strip_depth = (countertop_depth - counter_opening_depth) / 2.0

    countertop = model.part("countertop")
    countertop.visual(
        Box((side_strip_width, countertop_depth, countertop_thickness)),
        origin=Origin(
            xyz=(
                -countertop_width / 2.0 + side_strip_width / 2.0,
                0.0,
                countertop_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="left_counter_strip",
    )
    countertop.visual(
        Box((side_strip_width, countertop_depth, countertop_thickness)),
        origin=Origin(
            xyz=(
                countertop_width / 2.0 - side_strip_width / 2.0,
                0.0,
                countertop_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="right_counter_strip",
    )
    countertop.visual(
        Box((counter_opening_width + 0.004, end_strip_depth, countertop_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -countertop_depth / 2.0 + end_strip_depth / 2.0,
                countertop_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="front_counter_strip",
    )
    countertop.visual(
        Box((counter_opening_width + 0.004, end_strip_depth, countertop_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                countertop_depth / 2.0 - end_strip_depth / 2.0,
                countertop_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="rear_counter_strip",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((countertop_width, countertop_depth, countertop_thickness)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, countertop_thickness / 2.0)),
    )

    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height)),
    )

    glass_width = 0.76
    glass_depth = 0.52
    glass_thickness = 0.006
    cooktop_body_width = 0.72
    cooktop_body_depth = 0.46
    cooktop_body_height = 0.062

    cooktop = model.part("cooktop")
    button_holes = [
        [(x + px, y + py) for x, y in _circle_profile(0.0055, segments=22)]
        for px, py in (
            (0.0, -0.14),
            (0.0, -0.18),
            (0.0, -0.22),
            (-0.034, -0.18),
            (0.034, -0.18),
        )
    ]
    glass_top_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            [
                (-glass_width / 2.0, -glass_depth / 2.0),
                (glass_width / 2.0, -glass_depth / 2.0),
                (glass_width / 2.0, glass_depth / 2.0),
                (-glass_width / 2.0, glass_depth / 2.0),
            ],
            button_holes,
            glass_thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("cooktop_glass_with_button_holes.obj"),
    )
    cooktop.visual(
        glass_top_mesh,
        origin=Origin(xyz=(0.0, 0.0, glass_thickness / 2.0)),
        material=cooktop_glass,
        name="glass_top",
    )
    cooktop.visual(
        Box((cooktop_body_width, 0.22, cooktop_body_height)),
        origin=Origin(xyz=(0.0, 0.12, -cooktop_body_height / 2.0)),
        material=switch_body,
        name="rear_power_module",
    )
    cooktop.visual(
        Box((0.18, 0.14, 0.050)),
        origin=Origin(xyz=(-0.22, -0.13, -0.025)),
        material=switch_body,
        name="left_front_module",
    )
    cooktop.visual(
        Box((0.18, 0.14, 0.050)),
        origin=Origin(xyz=(0.22, -0.13, -0.025)),
        material=switch_body,
        name="right_front_module",
    )
    cooktop.visual(
        Box((0.14, 0.06, 0.020)),
        origin=Origin(xyz=(0.0, -0.095, -0.010)),
        material=switch_body,
        name="control_bridge",
    )
    cooktop.visual(
        Box((0.11, 0.11, 0.018)),
        origin=Origin(xyz=(0.0, -0.18, -0.013)),
        material=switch_body,
        name="button_switch_cluster",
    )
    for zone_name, zone_x, zone_y, zone_radius in (
        ("zone_rear_left", -0.20, 0.12, 0.105),
        ("zone_front_left", -0.21, -0.07, 0.090),
        ("zone_rear_center", 0.00, 0.14, 0.125),
        ("zone_rear_right", 0.20, 0.12, 0.105),
        ("zone_front_right", 0.21, -0.07, 0.090),
    ):
        cooktop.visual(
            Cylinder(radius=zone_radius, length=0.0006),
            origin=Origin(xyz=(zone_x, zone_y, glass_thickness + 0.0003)),
            material=zone_mark,
            name=zone_name,
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((glass_width, glass_depth, glass_thickness + cooktop_body_height)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, (glass_thickness - cooktop_body_height) / 2.0)),
    )

    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, countertop_thickness)),
    )

    for button_name, button_x, button_y in (
        ("button_top", 0.0, -0.14),
        ("button_center", 0.0, -0.18),
        ("button_bottom", 0.0, -0.22),
        ("button_left", -0.034, -0.18),
        ("button_right", 0.034, -0.18),
    ):
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=button_metal,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.0045, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=switch_body,
            name="stem",
        )
        button.visual(
            Cylinder(radius=0.0065, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, -0.0155)),
            material=switch_body,
            name="plunger",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.018, 0.018, 0.025)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
        )
        model.articulation(
            f"cooktop_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(button_x, button_y, glass_thickness)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0015,
            ),
        )

    drop_panel = model.part("drop_panel")
    drop_panel.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0 + 0.003, door_height / 2.0)),
        material=cabinet_white,
        name="panel_leaf",
    )
    drop_panel.visual(
        Box((0.18, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.003, door_height - 0.07)),
        material=pull_metal,
        name="pull_bar",
    )
    drop_panel.visual(
        Cylinder(radius=0.005, length=hinge_barrel_span),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pull_metal,
        name="panel_hinge_barrel",
    )
    drop_panel.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=7.5,
        origin=Origin(xyz=(0.0, door_thickness / 2.0, door_height / 2.0)),
    )

    model.articulation(
        "cabinet_to_drop_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=drop_panel,
        origin=Origin(xyz=(0.0, -cabinet_depth / 2.0 - 0.003, opening_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    cabinet_body = object_model.get_part("cabinet_body")
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    drop_panel = object_model.get_part("drop_panel")

    button_names = (
        "button_top",
        "button_center",
        "button_bottom",
        "button_left",
        "button_right",
    )
    buttons = [object_model.get_part(name) for name in button_names]
    button_joints = [object_model.get_articulation(f"cooktop_to_{name}") for name in button_names]
    panel_joint = object_model.get_articulation("cabinet_to_drop_panel")

    ctx.allow_overlap(
        drop_panel,
        cabinet_body,
        elem_a="panel_hinge_barrel",
        elem_b="hinge_pin",
        reason="The drop panel rotates on a captured bottom hinge pin.",
    )
    for button in buttons:
        ctx.allow_overlap(
            button,
            cooktop,
            elem_a="stem",
            elem_b="button_switch_cluster",
            reason="The button stem plunges into the concealed switch module.",
        )
        ctx.allow_overlap(
            button,
            cooktop,
            elem_a="plunger",
            elem_b="button_switch_cluster",
            reason="The internal plunger remains inside the concealed switch module through travel.",
        )

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

    ctx.expect_contact(countertop, cabinet_body, name="countertop_is_seated_on_cabinet")
    ctx.expect_contact(cooktop, countertop, name="cooktop_is_seated_in_countertop")
    ctx.expect_overlap(cooktop, countertop, axes="xy", min_overlap=0.50, name="cooktop_centered_in_counter")
    ctx.expect_overlap(drop_panel, cabinet_body, axes="x", min_overlap=0.70, name="drop_panel_spans_cabinet_front")

    for zone_name in (
        "zone_rear_left",
        "zone_front_left",
        "zone_rear_center",
        "zone_rear_right",
        "zone_front_right",
    ):
        ctx.check(
            f"{zone_name}_present",
            cooktop.get_visual(zone_name) is not None,
            f"Missing cooktop zone visual {zone_name}.",
        )

    for button_name, button, button_joint in zip(button_names, buttons, button_joints):
        ctx.expect_overlap(button, cooktop, axes="xy", min_overlap=0.016, name=f"{button_name}_within_control_area")
        rest_position = ctx.part_world_position(button)
        ctx.check(
            f"{button_name}_rest_position_known",
            rest_position is not None,
            f"Could not resolve rest position for {button_name}.",
        )
        limits = button_joint.motion_limits
        assert limits is not None
        with ctx.pose({button_joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{button_name}_lower_no_floating")
        with ctx.pose({button_joint: limits.upper}):
            pressed_position = ctx.part_world_position(button)
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{button_name}_upper_no_floating")
            ctx.expect_overlap(
                button,
                cooktop,
                axes="xy",
                min_overlap=0.016,
                name=f"{button_name}_pressed_stays_over_glass",
            )
            ctx.check(
                f"{button_name}_moves_inward",
                rest_position is not None
                and pressed_position is not None
                and pressed_position[2] < rest_position[2] - 0.0012,
                f"{button_name} should move inward by about 1.5 mm when pressed.",
            )

    panel_limits = panel_joint.motion_limits
    assert panel_limits is not None
    panel_rest_aabb = ctx.part_world_aabb(drop_panel)
    ctx.check(
        "drop_panel_rest_aabb_known",
        panel_rest_aabb is not None,
        "Could not resolve closed-panel bounds.",
    )
    with ctx.pose({panel_joint: panel_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="drop_panel_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="drop_panel_lower_no_floating")
    with ctx.pose({panel_joint: panel_limits.upper}):
        panel_open_aabb = ctx.part_world_aabb(drop_panel)
        ctx.fail_if_parts_overlap_in_current_pose(name="drop_panel_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="drop_panel_upper_no_floating")
        ctx.check(
            "drop_panel_swings_forward",
            panel_rest_aabb is not None
            and panel_open_aabb is not None
            and panel_open_aabb[0][1] < panel_rest_aabb[0][1] - 0.45,
            "The panel should swing outward from the cabinet front when opened.",
        )
        ctx.check(
            "drop_panel_top_lowers",
            panel_rest_aabb is not None
            and panel_open_aabb is not None
            and panel_open_aabb[1][2] < panel_rest_aabb[1][2] - 0.55,
            "The top edge of the panel should lower substantially in the open pose.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
