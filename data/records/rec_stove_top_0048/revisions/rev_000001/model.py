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
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smooth_top_electric_stove", assets=ASSETS)

    steel = model.material("steel", rgba=(0.76, 0.77, 0.78, 1.0))
    dark_enamel = model.material("dark_enamel", rgba=(0.18, 0.19, 0.21, 1.0))
    black_glass = model.material("black_glass", rgba=(0.08, 0.08, 0.09, 1.0))
    oven_glass = model.material("oven_glass", rgba=(0.14, 0.18, 0.22, 0.72))
    radiant = model.material("radiant", rgba=(0.75, 0.22, 0.10, 0.78))
    button_finish = model.material("button_finish", rgba=(0.80, 0.80, 0.82, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.86, 0.86, 0.87, 1.0))

    width = 0.76
    depth = 0.68
    body_height = 0.89
    side_thickness = 0.04
    back_thickness = 0.02
    bottom_thickness = 0.03
    top_thickness = 0.03
    toe_kick_height = 0.088
    toe_kick_depth = 0.04
    panel_width = width - (2.0 * side_thickness)
    panel_thickness = 0.035
    panel_height = 0.13
    panel_center_z = 0.825
    cooktop_thickness = 0.012
    cooktop_width = 0.778
    cooktop_depth = 0.702
    front_y = -(depth / 2.0)
    inner_front_y = front_y + panel_thickness
    inner_back_y = (depth / 2.0) - back_thickness
    top_support_depth = inner_back_y - inner_front_y
    top_support_center_y = (inner_back_y + inner_front_y) / 2.0
    front_frame_thickness = 0.025

    cooktop_glass = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(
                cooktop_width,
                cooktop_depth,
                0.028,
                corner_segments=10,
            ),
            cooktop_thickness,
            cap=True,
        ),
        "assets/meshes/cooktop_glass.obj",
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_thickness, depth, body_height)),
        origin=Origin(
            xyz=(
                -(width / 2.0) + (side_thickness / 2.0),
                0.0,
                body_height / 2.0,
            )
        ),
        material=steel,
        name="left_side",
    )
    cabinet.visual(
        Box((side_thickness, depth, body_height)),
        origin=Origin(
            xyz=(
                (width / 2.0) - (side_thickness / 2.0),
                0.0,
                body_height / 2.0,
            )
        ),
        material=steel,
        name="right_side",
    )
    cabinet.visual(
        Box((panel_width, back_thickness, body_height)),
        origin=Origin(
            xyz=(
                0.0,
                (depth / 2.0) - (back_thickness / 2.0),
                body_height / 2.0,
            )
        ),
        material=steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((panel_width, depth - back_thickness, bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                front_y + ((depth - back_thickness) / 2.0),
                bottom_thickness / 2.0,
            )
        ),
        material=dark_enamel,
        name="oven_floor",
    )
    cabinet.visual(
        Box((panel_width, top_support_depth, top_thickness)),
        origin=Origin(
            xyz=(0.0, top_support_center_y, body_height - (top_thickness / 2.0))
        ),
        material=steel,
        name="top_support",
    )
    cabinet.visual(
        Box((panel_width, toe_kick_depth, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y + (toe_kick_depth / 2.0),
                toe_kick_height / 2.0,
            )
        ),
        material=dark_enamel,
        name="toe_kick",
    )
    cabinet.visual(
        Box((side_thickness, front_frame_thickness, 0.658)),
        origin=Origin(
            xyz=(
                -(width / 2.0) + (side_thickness / 2.0),
                front_y + (front_frame_thickness / 2.0),
                0.102 + (0.658 / 2.0),
            )
        ),
        material=dark_enamel,
        name="front_left_stile",
    )
    cabinet.visual(
        Box((side_thickness, front_frame_thickness, 0.658)),
        origin=Origin(
            xyz=(
                (width / 2.0) - (side_thickness / 2.0),
                front_y + (front_frame_thickness / 2.0),
                0.102 + (0.658 / 2.0),
            )
        ),
        material=dark_enamel,
        name="front_right_stile",
    )
    control_panel = model.part("control_panel")
    opening_band_height = 0.028
    cap_strip_height = (panel_height - opening_band_height) / 2.0
    slot_width = 0.102
    slot_left_1 = -0.271
    slot_right_1 = slot_left_1 + slot_width
    slot_left_2 = -0.141
    slot_right_2 = slot_left_2 + slot_width
    left_strip_width = slot_left_1 - (-(panel_width / 2.0))
    center_strip_width = slot_left_2 - slot_right_1
    right_strip_width = (panel_width / 2.0) - slot_right_2
    control_panel.visual(
        Box((panel_width, panel_thickness, cap_strip_height)),
        origin=Origin(xyz=(0.0, 0.0, (panel_height / 2.0) - (cap_strip_height / 2.0))),
        material=dark_enamel,
        name="panel_top_strip",
    )
    control_panel.visual(
        Box((panel_width, panel_thickness, cap_strip_height)),
        origin=Origin(
            xyz=(0.0, 0.0, -(panel_height / 2.0) + (cap_strip_height / 2.0))
        ),
        material=dark_enamel,
        name="panel_bottom_strip",
    )
    control_panel.visual(
        Box((left_strip_width, panel_thickness, opening_band_height)),
        origin=Origin(
            xyz=(
                -(panel_width / 2.0) + (left_strip_width / 2.0),
                0.0,
                0.0,
            )
        ),
        material=dark_enamel,
        name="panel_left_strip",
    )
    control_panel.visual(
        Box((center_strip_width, panel_thickness, opening_band_height)),
        origin=Origin(
            xyz=(
                slot_right_1 + (center_strip_width / 2.0),
                0.0,
                0.0,
            )
        ),
        material=dark_enamel,
        name="panel_center_strip",
    )
    control_panel.visual(
        Box((right_strip_width, panel_thickness, opening_band_height)),
        origin=Origin(
            xyz=(
                slot_right_2 + (right_strip_width / 2.0),
                0.0,
                0.0,
            )
        ),
        material=dark_enamel,
        name="panel_right_strip",
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        cooktop_glass,
        origin=Origin(xyz=(0.0, 0.0, cooktop_thickness / 2.0)),
        material=black_glass,
        name="glass_top",
    )
    zone_z = cooktop_thickness - 0.0005
    cooktop.visual(
        Cylinder(radius=0.090, length=0.001),
        origin=Origin(xyz=(-0.20, -0.17, zone_z)),
        material=radiant,
        name="zone_front_left",
    )
    cooktop.visual(
        Cylinder(radius=0.082, length=0.001),
        origin=Origin(xyz=(0.19, -0.17, zone_z)),
        material=radiant,
        name="zone_front_right",
    )
    cooktop.visual(
        Cylinder(radius=0.095, length=0.001),
        origin=Origin(xyz=(-0.20, 0.17, zone_z)),
        material=radiant,
        name="zone_rear_left",
    )
    cooktop.visual(
        Cylinder(radius=0.090, length=0.001),
        origin=Origin(xyz=(0.19, 0.17, zone_z)),
        material=radiant,
        name="zone_rear_right",
    )

    hinge_height = 0.102
    door_height = 0.658
    door_width = 0.68
    door_thickness = 0.035
    handle_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (-0.18, -0.008, 0.56),
                (-0.18, -0.042, 0.56),
                (0.18, -0.042, 0.56),
                (0.18, -0.008, 0.56),
            ],
            radius=0.009,
            radial_segments=18,
            cap_ends=False,
            corner_mode="fillet",
            corner_radius=0.020,
            corner_segments=10,
        ),
        "assets/meshes/oven_handle.obj",
    )
    oven_door = model.part("oven_door")
    oven_door.visual(
        Cylinder(radius=0.012, length=0.64),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_enamel,
        name="hinge_barrel",
    )
    oven_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.0, -(door_thickness / 2.0), door_height / 2.0)),
        material=dark_enamel,
        name="door_panel",
    )
    oven_door.visual(
        Box((0.44, 0.008, 0.32)),
        origin=Origin(xyz=(0.0, -0.010, 0.36)),
        material=oven_glass,
        name="door_window",
    )
    oven_door.visual(
        handle_mesh,
        origin=Origin(),
        material=handle_finish,
        name="door_handle",
    )

    door_hinge_pin = model.part("door_hinge_pin")
    door_hinge_pin.visual(
        Cylinder(radius=0.005, length=panel_width),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_finish,
        name="hinge_pin",
    )

    button_left = model.part("button_left")
    button_left.visual(
        Box((slot_width, 0.028, opening_band_height)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=button_finish,
        name="button_body",
    )

    button_right = model.part("button_right")
    button_right.visual(
        Box((slot_width, 0.028, opening_band_height)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=button_finish,
        name="button_body",
    )

    knob_left = model.part("knob_left")
    knob_left.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_body",
    )
    knob_left.visual(
        Box((0.004, 0.002, 0.016)),
        origin=Origin(xyz=(0.0, -0.025, 0.012)),
        material=button_finish,
        name="indicator",
    )

    knob_right = model.part("knob_right")
    knob_right.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=knob_finish,
        name="knob_body",
    )
    knob_right.visual(
        Box((0.004, 0.002, 0.016)),
        origin=Origin(xyz=(0.0, -0.025, 0.012)),
        material=button_finish,
        name="indicator",
    )

    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(0.0, front_y + (panel_thickness / 2.0), panel_center_z)),
    )
    model.articulation(
        "cabinet_to_cooktop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, body_height)),
    )
    model.articulation(
        "cabinet_to_hinge_pin",
        ArticulationType.FIXED,
        parent=cabinet,
        child=door_hinge_pin,
        origin=Origin(xyz=(0.0, front_y, hinge_height)),
    )
    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=oven_door,
        origin=Origin(xyz=(0.0, front_y, hinge_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "left_button_slide",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=button_left,
        origin=Origin(xyz=(-0.220, -(panel_thickness / 2.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "right_button_slide",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=button_right,
        origin=Origin(xyz=(-0.090, -(panel_thickness / 2.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "left_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=knob_left,
        origin=Origin(xyz=(0.120, -(panel_thickness / 2.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    model.articulation(
        "right_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=knob_right,
        origin=Origin(xyz=(0.240, -(panel_thickness / 2.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    control_panel = object_model.get_part("control_panel")
    cooktop = object_model.get_part("cooktop")
    oven_door = object_model.get_part("oven_door")
    door_hinge_pin = object_model.get_part("door_hinge_pin")
    button_left = object_model.get_part("button_left")
    button_right = object_model.get_part("button_right")
    knob_left = object_model.get_part("knob_left")
    knob_right = object_model.get_part("knob_right")

    oven_door_hinge = object_model.get_articulation("oven_door_hinge")
    left_button_slide = object_model.get_articulation("left_button_slide")
    right_button_slide = object_model.get_articulation("right_button_slide")
    left_knob_spin = object_model.get_articulation("left_knob_spin")
    right_knob_spin = object_model.get_articulation("right_knob_spin")

    glass_top = cooktop.get_visual("glass_top")
    zone_front_left = cooktop.get_visual("zone_front_left")
    zone_front_right = cooktop.get_visual("zone_front_right")
    zone_rear_left = cooktop.get_visual("zone_rear_left")
    zone_rear_right = cooktop.get_visual("zone_rear_right")

    ctx.allow_overlap(
        oven_door,
        door_hinge_pin,
        reason="The oven door uses a solid hinge barrel around a solid hinge-pin proxy so the articulated connection remains physically grounded through the full swing.",
    )

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

    ctx.check(
        "four_radiant_zones_present",
        len(
            [
                zone_front_left,
                zone_front_right,
                zone_rear_left,
                zone_rear_right,
            ]
        )
        == 4,
        "Expected four named radiant-zone visuals on the cooktop.",
    )
    ctx.check(
        "door_hinge_axis_is_left_right",
        tuple(round(v, 3) for v in oven_door_hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected oven door hinge axis (1, 0, 0), got {oven_door_hinge.axis!r}.",
    )
    ctx.check(
        "button_axes_push_inward",
        tuple(round(v, 3) for v in left_button_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in right_button_slide.axis) == (0.0, 1.0, 0.0),
        "Expected both push-buttons to move inward along +Y.",
    )
    ctx.check(
        "knob_axes_rotate_about_panel_normal",
        tuple(round(v, 3) for v in left_knob_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in right_knob_spin.axis) == (0.0, 1.0, 0.0),
        "Expected both knobs to rotate about the front-panel normal.",
    )
    ctx.check(
        "button_travel_is_short",
        left_button_slide.motion_limits is not None
        and right_button_slide.motion_limits is not None
        and left_button_slide.motion_limits.upper == 0.006
        and right_button_slide.motion_limits.upper == 0.006,
        "Push-buttons should have short 6 mm travel.",
    )
    ctx.check(
        "knobs_are_continuous",
        left_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        "Knob joints should be continuous.",
    )

    ctx.expect_contact(control_panel, cabinet, name="control_panel_contacts_cabinet")
    ctx.expect_contact(cooktop, cabinet, name="cooktop_contacts_cabinet")
    ctx.expect_contact(
        door_hinge_pin, cabinet, name="hinge_pin_contacts_cabinet_structure"
    )
    ctx.expect_within(
        cooktop,
        cooktop,
        axes="xy",
        inner_elem=zone_front_left,
        outer_elem=glass_top,
        margin=0.0,
        name="front_left_zone_within_glass",
    )
    ctx.expect_within(
        cooktop,
        cooktop,
        axes="xy",
        inner_elem=zone_front_right,
        outer_elem=glass_top,
        margin=0.0,
        name="front_right_zone_within_glass",
    )
    ctx.expect_within(
        cooktop,
        cooktop,
        axes="xy",
        inner_elem=zone_rear_left,
        outer_elem=glass_top,
        margin=0.0,
        name="rear_left_zone_within_glass",
    )
    ctx.expect_within(
        cooktop,
        cooktop,
        axes="xy",
        inner_elem=zone_rear_right,
        outer_elem=glass_top,
        margin=0.0,
        name="rear_right_zone_within_glass",
    )

    with ctx.pose({oven_door_hinge: 0.0}):
        ctx.expect_contact(oven_door, cabinet, name="door_closed_contacts_cabinet")
        ctx.expect_gap(
            control_panel,
            oven_door,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="door_closed_meets_panel_cleanly",
        )
        ctx.expect_overlap(
            oven_door,
            cabinet,
            axes="x",
            min_overlap=0.62,
            name="door_spans_broad_oven_width",
        )

    door_limits = oven_door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({oven_door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")
            ctx.expect_gap(
                control_panel,
                oven_door,
                axis="z",
                min_gap=0.50,
                name="door_swings_well_below_control_panel",
            )

    for slide_joint, button_part, prefix in (
        (left_button_slide, button_left, "left_button"),
        (right_button_slide, button_right, "right_button"),
    ):
        limits = slide_joint.motion_limits
        with ctx.pose({slide_joint: 0.0}):
            ctx.expect_contact(
                button_part,
                control_panel,
                name=f"{prefix}_resting_contact",
            )
        if limits is not None and limits.upper is not None:
            with ctx.pose({slide_joint: limits.upper}):
                ctx.expect_contact(
                    button_part,
                    control_panel,
                    name=f"{prefix}_pressed_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{prefix}_pressed_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{prefix}_pressed_no_floating")

    with ctx.pose({left_knob_spin: pi / 2.0, right_knob_spin: -pi / 2.0}):
        ctx.expect_contact(knob_left, control_panel, name="left_knob_contacts_panel")
        ctx.expect_contact(knob_right, control_panel, name="right_knob_contacts_panel")
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="articulated_parent_child_clearance",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
