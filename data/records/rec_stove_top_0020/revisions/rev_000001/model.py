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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_cooktop_cabinet", assets=ASSETS)

    cabinet_paint = model.material("cabinet_paint", rgba=(0.93, 0.93, 0.91, 1.0))
    countertop_stone = model.material("countertop_stone", rgba=(0.73, 0.75, 0.76, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.07, 0.08, 0.09, 0.92))
    cooktop_frame = model.material("cooktop_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    radiant_zone = model.material("radiant_zone", rgba=(0.26, 0.18, 0.17, 0.45))
    zone_mark = model.material("zone_mark", rgba=(0.63, 0.23, 0.15, 0.22))
    hardware_dark = model.material("hardware_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    hardware_light = model.material("hardware_light", rgba=(0.72, 0.74, 0.76, 1.0))

    cabinet_width = 0.80
    cabinet_depth = 0.58
    cabinet_height = 0.86
    side_thickness = 0.018
    back_thickness = 0.012
    panel_thickness = 0.018
    toe_kick_height = 0.10
    door_height = 0.738
    door_width = 0.389
    door_thickness = 0.019
    door_bottom = 0.102

    counter_width = 0.90
    counter_depth = 0.64
    counter_thickness = 0.04
    counter_side_strip = 0.171
    counter_end_strip = 0.076
    opening_width = counter_width - 2.0 * counter_side_strip
    opening_depth = counter_depth - 2.0 * counter_end_strip

    cooktop_glass_width = 0.556
    cooktop_glass_depth = 0.486
    cooktop_glass_thickness = 0.006
    cooktop_flange_width = 0.600
    cooktop_flange_depth = 0.530
    cooktop_flange_thickness = 0.006
    cooktop_body_width = 0.535
    cooktop_body_depth = 0.465
    cooktop_body_height = 0.060
    frame_wall_thickness = 0.010
    front_bezel_depth = 0.018
    front_opening_y = -opening_depth / 2.0

    cabinet = model.part("cabinet_carcass")
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + side_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=cabinet_paint,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0 - side_thickness / 2.0, 0.0, cabinet_height / 2.0)),
        material=cabinet_paint,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, cabinet_depth - back_thickness, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -back_thickness / 2.0,
                toe_kick_height + panel_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, back_thickness, cabinet_height - toe_kick_height - panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth / 2.0 - back_thickness / 2.0,
                toe_kick_height + panel_thickness + (cabinet_height - toe_kick_height - panel_thickness) / 2.0,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth - 0.060, panel_thickness)),
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0 + side_thickness + side_thickness / 2.0,
                0.0,
                cabinet_height - panel_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="left_counter_ledger",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth - 0.060, panel_thickness)),
        origin=Origin(
            xyz=(
                cabinet_width / 2.0 - side_thickness - side_thickness / 2.0,
                0.0,
                cabinet_height - panel_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="right_counter_ledger",
    )
    cabinet.visual(
        Box((cabinet_width - 0.140, 0.018, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_depth / 2.0 + 0.070 + 0.009,
                toe_kick_height / 2.0,
            )
        ),
        material=cabinet_paint,
        name="toe_kick_board",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0)),
    )

    countertop = model.part("countertop")
    countertop.visual(
        Box((counter_side_strip, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                -counter_width / 2.0 + counter_side_strip / 2.0,
                0.0,
                counter_thickness / 2.0,
            )
        ),
        material=countertop_stone,
        name="left_counter_strip",
    )
    countertop.visual(
        Box((counter_side_strip, counter_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                counter_width / 2.0 - counter_side_strip / 2.0,
                0.0,
                counter_thickness / 2.0,
            )
        ),
        material=countertop_stone,
        name="right_counter_strip",
    )
    countertop.visual(
        Box((opening_width + 0.004, counter_end_strip, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                counter_depth / 2.0 - counter_end_strip / 2.0,
                counter_thickness / 2.0,
            )
        ),
        material=countertop_stone,
        name="back_counter_strip",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((counter_width, counter_depth, counter_thickness)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, counter_thickness / 2.0)),
    )

    cooktop = model.part("cooktop_body")
    cooktop.visual(
        Box((cooktop_glass_width, cooktop_glass_depth, cooktop_glass_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                counter_thickness - cooktop_glass_thickness / 2.0,
            )
        ),
        material=cooktop_glass,
        name="glass_top",
    )
    for zone_name, zone_x, zone_y, zone_radius in (
        ("left_front_zone", -0.145, -0.115, 0.090),
        ("right_front_zone", 0.145, -0.115, 0.090),
        ("left_rear_zone", -0.145, 0.115, 0.100),
        ("right_rear_zone", 0.145, 0.115, 0.100),
    ):
        cooktop.visual(
            Cylinder(radius=zone_radius, length=0.0012),
            origin=Origin(
                xyz=(zone_x, zone_y, counter_thickness - cooktop_glass_thickness - 0.0006),
            ),
            material=radiant_zone,
            name=zone_name,
        )
        cooktop.visual(
            Cylinder(radius=zone_radius * 0.56, length=0.0008),
            origin=Origin(
                xyz=(zone_x, zone_y, counter_thickness - cooktop_glass_thickness - 0.0016),
            ),
            material=zone_mark,
            name=f"{zone_name}_mark",
        )
    cooktop.visual(
        Box((frame_wall_thickness, cooktop_body_depth + 0.001, 0.034)),
        origin=Origin(xyz=(-cooktop_glass_width / 2.0 + frame_wall_thickness / 2.0, 0.0, 0.017)),
        material=cooktop_frame,
        name="left_frame_wall",
    )
    cooktop.visual(
        Box((frame_wall_thickness, cooktop_body_depth + 0.001, 0.034)),
        origin=Origin(xyz=(cooktop_glass_width / 2.0 - frame_wall_thickness / 2.0, 0.0, 0.017)),
        material=cooktop_frame,
        name="right_frame_wall",
    )
    cooktop.visual(
        Box((cooktop_body_width, frame_wall_thickness, 0.034)),
        origin=Origin(xyz=(0.0, cooktop_glass_depth / 2.0 - frame_wall_thickness / 2.0, 0.017)),
        material=cooktop_frame,
        name="back_frame_wall",
    )
    cooktop.visual(
        Box((0.145, front_bezel_depth, 0.034)),
        origin=Origin(xyz=(-0.205, front_opening_y + front_bezel_depth / 2.0, 0.017)),
        material=cooktop_frame,
        name="left_front_bezel",
    )
    cooktop.visual(
        Box((0.145, front_bezel_depth, 0.034)),
        origin=Origin(xyz=(0.205, front_opening_y + front_bezel_depth / 2.0, 0.017)),
        material=cooktop_frame,
        name="right_front_bezel",
    )
    cooktop.visual(
        Box((0.240, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, front_opening_y + 0.006, 0.029)),
        material=cooktop_frame,
        name="button_slot_top",
    )
    cooktop.visual(
        Box((0.240, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, front_opening_y + 0.006, 0.005)),
        material=cooktop_frame,
        name="button_slot_bottom",
    )
    cooktop.visual(
        Box((0.012, front_bezel_depth, 0.034)),
        origin=Origin(xyz=(-0.117, front_opening_y + front_bezel_depth / 2.0, 0.017)),
        material=cooktop_frame,
        name="button_slot_left_post",
    )
    cooktop.visual(
        Box((0.012, front_bezel_depth, 0.034)),
        origin=Origin(xyz=(0.117, front_opening_y + front_bezel_depth / 2.0, 0.017)),
        material=cooktop_frame,
        name="button_slot_right_post",
    )
    cooktop.visual(
        Box((cooktop_flange_width, cooktop_flange_depth, cooktop_flange_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -cooktop_flange_thickness / 2.0)),
        material=hardware_light,
        name="mounting_flange",
    )
    cooktop.visual(
        Box((cooktop_body_width, cooktop_body_depth, cooktop_body_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -cooktop_flange_thickness - cooktop_body_height / 2.0,
            )
        ),
        material=hardware_dark,
        name="undertray",
    )
    cooktop.inertial = Inertial.from_geometry(
        Box((cooktop_flange_width, cooktop_flange_depth, counter_thickness + cooktop_body_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="knob_body",
    )
    left_knob.visual(
        Box((0.014, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.002, 0.012)),
        material=hardware_light,
        name="pointer",
    )
    left_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.026),
        mass=0.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="knob_body",
    )
    right_knob.visual(
        Box((0.014, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.002, 0.012)),
        material=hardware_light,
        name="pointer",
    )
    right_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.026),
        mass=0.05,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    center_button = model.part("center_button")
    center_button.visual(
        Box((0.220, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=hardware_light,
        name="button_cap",
    )
    center_button.inertial = Inertial.from_geometry(
        Box((0.220, 0.014, 0.014)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, -door_thickness / 2.0, door_height / 2.0)),
        material=cabinet_paint,
        name="door_panel",
    )
    left_door.visual(
        Box((0.014, 0.010, 0.160)),
        origin=Origin(xyz=(door_width - 0.034, -door_thickness - 0.005, 0.510)),
        material=hardware_light,
        name="pull_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=5.8,
        origin=Origin(xyz=(door_width / 2.0, -door_thickness / 2.0, door_height / 2.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width / 2.0, -door_thickness / 2.0, door_height / 2.0)),
        material=cabinet_paint,
        name="door_panel",
    )
    right_door.visual(
        Box((0.014, 0.010, 0.160)),
        origin=Origin(xyz=(-door_width + 0.034, -door_thickness - 0.005, 0.510)),
        material=hardware_light,
        name="pull_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=5.8,
        origin=Origin(xyz=(-door_width / 2.0, -door_thickness / 2.0, door_height / 2.0)),
    )

    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height)),
    )
    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(),
    )
    model.articulation(
        "cooktop_to_center_button",
        ArticulationType.PRISMATIC,
        parent=cooktop,
        child=center_button,
        origin=Origin(xyz=(0.0, front_opening_y, 0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.06,
            lower=0.0,
            upper=0.008,
        ),
    )
    model.articulation(
        "cooktop_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=left_knob,
        origin=Origin(xyz=(-0.224, front_opening_y - 0.013, 0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )
    model.articulation(
        "cooktop_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=right_knob,
        origin=Origin(xyz=(0.224, front_opening_y - 0.013, 0.017)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )
    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(
            xyz=(
                -cabinet_width / 2.0,
                -cabinet_depth / 2.0,
                door_bottom,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=-1.92,
            upper=0.0,
        ),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(
            xyz=(
                cabinet_width / 2.0,
                -cabinet_depth / 2.0,
                door_bottom,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=0.0,
            upper=1.92,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet_carcass")
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop_body")
    center_button = object_model.get_part("center_button")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    button_slide = object_model.get_articulation("cooktop_to_center_button")
    left_knob_spin = object_model.get_articulation("cooktop_to_left_knob")
    right_knob_spin = object_model.get_articulation("cooktop_to_right_knob")
    left_door_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_door_hinge = object_model.get_articulation("cabinet_to_right_door")

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

    def _elem_aabb(part, elem: str):
        return ctx.part_element_world_aabb(part, elem=elem)

    def _top_z(part, elem: str) -> float | None:
        aabb = _elem_aabb(part, elem)
        if aabb is None:
            return None
        return aabb[1][2]

    def _front_y(part, elem: str) -> float | None:
        aabb = _elem_aabb(part, elem)
        if aabb is None:
            return None
        return aabb[0][1]

    ctx.expect_gap(countertop, cabinet, axis="z", max_gap=0.0, max_penetration=0.0, name="countertop_seated_on_cabinet")
    ctx.expect_contact(countertop, cabinet, name="countertop_contacts_cabinet")
    ctx.expect_contact(cooktop, countertop, name="cooktop_clamped_to_countertop")
    ctx.expect_contact(left_door, cabinet, name="left_door_closed_contacts_cabinet")
    ctx.expect_contact(right_door, cabinet, name="right_door_closed_contacts_cabinet")

    glass_top_z = _top_z(cooktop, "glass_top")
    counter_top_z = _top_z(countertop, "left_counter_strip")
    ctx.check(
        "cooktop_glass_flush_with_countertop",
        glass_top_z is not None and counter_top_z is not None and abs(glass_top_z - counter_top_z) <= 0.001,
        details=f"glass_top_z={glass_top_z}, counter_top_z={counter_top_z}",
    )

    left_strip = _elem_aabb(countertop, "left_counter_strip")
    right_strip = _elem_aabb(countertop, "right_counter_strip")
    back_strip = _elem_aabb(countertop, "back_counter_strip")
    countertop_box = ctx.part_world_aabb(countertop)
    undertray = _elem_aabb(cooktop, "undertray")
    opening_fit_ok = False
    opening_details = "missing AABB"
    if left_strip and right_strip and back_strip and undertray and countertop_box:
        opening_min_x = left_strip[1][0]
        opening_max_x = right_strip[0][0]
        opening_min_y = countertop_box[0][1]
        opening_max_y = back_strip[0][1]
        opening_fit_ok = (
            undertray[0][0] >= opening_min_x + 0.004
            and undertray[1][0] <= opening_max_x - 0.004
            and undertray[0][1] >= opening_min_y + 0.040
            and undertray[1][1] <= opening_max_y - 0.004
        )
        opening_details = (
            f"undertray={undertray}, "
            f"opening=(({opening_min_x}, {opening_min_y}), ({opening_max_x}, {opening_max_y}))"
        )
    ctx.check("cooktop_undertray_fits_cutout", opening_fit_ok, opening_details)

    for zone_name in (
        "left_front_zone",
        "right_front_zone",
        "left_rear_zone",
        "right_rear_zone",
    ):
        zone_box = _elem_aabb(cooktop, zone_name)
        glass_box = _elem_aabb(cooktop, "glass_top")
        zone_ok = False
        zone_details = "missing AABB"
        if zone_box and glass_box:
            zone_ok = (
                zone_box[0][0] >= glass_box[0][0]
                and zone_box[1][0] <= glass_box[1][0]
                and zone_box[0][1] >= glass_box[0][1]
                and zone_box[1][1] <= glass_box[1][1]
                and zone_box[1][2] <= glass_box[1][2]
            )
            zone_details = f"{zone_name}={zone_box}, glass={glass_box}"
        ctx.check(f"{zone_name}_within_glass", zone_ok, zone_details)

    ctx.check(
        "button_prismatic_axis",
        button_slide.axis == (0.0, 1.0, 0.0)
        and button_slide.motion_limits is not None
        and button_slide.motion_limits.lower == 0.0
        and button_slide.motion_limits.upper is not None
        and 0.006 <= button_slide.motion_limits.upper <= 0.010,
        details=f"axis={button_slide.axis}, limits={button_slide.motion_limits}",
    )
    ctx.check(
        "left_knob_continuous_axis",
        left_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_knob_spin.axis == (0.0, 1.0, 0.0)
        and left_knob_spin.motion_limits is not None
        and left_knob_spin.motion_limits.lower is None
        and left_knob_spin.motion_limits.upper is None,
        details=f"type={left_knob_spin.articulation_type}, axis={left_knob_spin.axis}",
    )
    ctx.check(
        "right_knob_continuous_axis",
        right_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_spin.axis == (0.0, 1.0, 0.0)
        and right_knob_spin.motion_limits is not None
        and right_knob_spin.motion_limits.lower is None
        and right_knob_spin.motion_limits.upper is None,
        details=f"type={right_knob_spin.articulation_type}, axis={right_knob_spin.axis}",
    )
    ctx.check(
        "door_hinge_axes_and_ranges",
        left_door_hinge.axis == (0.0, 0.0, 1.0)
        and right_door_hinge.axis == (0.0, 0.0, 1.0)
        and left_door_hinge.motion_limits is not None
        and right_door_hinge.motion_limits is not None
        and left_door_hinge.motion_limits.lower is not None
        and right_door_hinge.motion_limits.upper is not None
        and left_door_hinge.motion_limits.lower <= -1.75
        and right_door_hinge.motion_limits.upper >= 1.75,
        details=(
            f"left={left_door_hinge.motion_limits}, "
            f"right={right_door_hinge.motion_limits}"
        ),
    )

    with ctx.pose({button_slide: 0.0}):
        button_front = _front_y(center_button, "button_cap")
        bezel_front = _front_y(cooktop, "left_front_bezel")
        ctx.check(
            "button_rest_flush_with_bezel",
            button_front is not None and bezel_front is not None and abs(button_front - bezel_front) <= 0.001,
            details=f"button_front={button_front}, bezel_front={bezel_front}",
        )

    if button_slide.motion_limits is not None and button_slide.motion_limits.upper is not None:
        with ctx.pose({button_slide: button_slide.motion_limits.upper}):
            button_front = _front_y(center_button, "button_cap")
            bezel_front = _front_y(cooktop, "left_front_bezel")
            ctx.check(
                "button_pressed_inward_travel",
                button_front is not None and bezel_front is not None and 0.006 <= button_front - bezel_front <= 0.010,
                details=f"button_front={button_front}, bezel_front={bezel_front}",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="button_pressed_no_floating")

    with ctx.pose({left_knob_spin: 1.3, right_knob_spin: -2.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")

    if (
        left_door_hinge.motion_limits is not None
        and left_door_hinge.motion_limits.lower is not None
        and right_door_hinge.motion_limits is not None
        and right_door_hinge.motion_limits.upper is not None
    ):
        with ctx.pose(
            {
                left_door_hinge: left_door_hinge.motion_limits.lower,
                right_door_hinge: right_door_hinge.motion_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="doors_open_no_overlap")
            ctx.fail_if_isolated_parts(name="doors_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
