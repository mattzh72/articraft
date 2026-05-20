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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insulated_pet_flap", assets=ASSETS)

    frame_charcoal = model.material("frame_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))
    flap_trim = model.material("flap_trim", rgba=(0.23, 0.24, 0.26, 1.0))
    clear_polycarbonate = model.material("clear_polycarbonate", rgba=(0.80, 0.91, 0.98, 0.30))

    frame_width = 0.38
    frame_height = 0.46
    frame_depth = 0.065
    side_border = 0.055
    bottom_border = 0.065
    top_hood = 0.085
    opening_width = frame_width - 2.0 * side_border
    opening_height = frame_height - bottom_border - top_hood
    opening_bottom = bottom_border
    opening_top = opening_bottom + opening_height

    hood_roof_thickness = 0.012
    hood_lip_thickness = 0.010
    hood_lip_bottom = 0.330
    hood_lip_height = frame_height - hood_roof_thickness - hood_lip_bottom
    bearing_tab_width = 0.008
    bearing_tab_depth = 0.008
    bearing_tab_height = 0.026

    stop_width = 0.004
    stop_depth = 0.010
    stop_height = opening_height

    seal_height = 0.010
    seal_depth = 0.012

    hinge_z = 0.360

    door_width = 0.258
    door_height = 0.295
    door_thickness = 0.018
    border_width = 0.016
    top_trim_height = 0.028
    bottom_trim_height = 0.016
    side_trim_height = door_height - top_trim_height - bottom_trim_height
    clear_width = door_width - 2.0 * border_width
    clear_height = door_height - top_trim_height - bottom_trim_height

    frame = model.part("outer_frame")
    frame.visual(
        Box((side_border, frame_depth, frame_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + side_border * 0.5), 0.0, frame_height * 0.5)),
        material=frame_charcoal,
        name="left_jamb",
    )
    frame.visual(
        Box((side_border, frame_depth, frame_height)),
        origin=Origin(xyz=((opening_width * 0.5 + side_border * 0.5), 0.0, frame_height * 0.5)),
        material=frame_charcoal,
        name="right_jamb",
    )
    frame.visual(
        Box((frame_width, frame_depth, bottom_border)),
        origin=Origin(xyz=(0.0, 0.0, bottom_border * 0.5)),
        material=frame_charcoal,
        name="bottom_sill",
    )
    frame.visual(
        Box((frame_width, frame_depth, hood_roof_thickness)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - hood_roof_thickness * 0.5)),
        material=frame_charcoal,
        name="hood_roof",
    )
    frame.visual(
        Box((frame_width, hood_lip_thickness, hood_lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                frame_depth * 0.5 - hood_lip_thickness * 0.5,
                hood_lip_bottom + hood_lip_height * 0.5,
            )
        ),
        material=frame_charcoal,
        name="hood_front_lip",
    )
    frame.visual(
        Box((frame_width, hood_lip_thickness, hood_lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                -frame_depth * 0.5 + hood_lip_thickness * 0.5,
                hood_lip_bottom + hood_lip_height * 0.5,
            )
        ),
        material=frame_charcoal,
        name="hood_back_lip",
    )
    frame.visual(
        Box((bearing_tab_width, bearing_tab_depth, bearing_tab_height)),
        origin=Origin(
            xyz=(
                -(opening_width * 0.5 - bearing_tab_width * 0.5),
                frame_depth * 0.5 - hood_lip_thickness - bearing_tab_depth * 0.5,
                hinge_z - 0.004,
            )
        ),
        material=frame_charcoal,
        name="left_front_bearing_tab",
    )
    frame.visual(
        Box((bearing_tab_width, bearing_tab_depth, bearing_tab_height)),
        origin=Origin(
            xyz=(
                -(opening_width * 0.5 - bearing_tab_width * 0.5),
                -frame_depth * 0.5 + hood_lip_thickness + bearing_tab_depth * 0.5,
                hinge_z - 0.004,
            )
        ),
        material=frame_charcoal,
        name="left_rear_bearing_tab",
    )
    frame.visual(
        Box((bearing_tab_width, bearing_tab_depth, bearing_tab_height)),
        origin=Origin(
            xyz=(
                opening_width * 0.5 - bearing_tab_width * 0.5,
                frame_depth * 0.5 - hood_lip_thickness - bearing_tab_depth * 0.5,
                hinge_z - 0.004,
            )
        ),
        material=frame_charcoal,
        name="right_front_bearing_tab",
    )
    frame.visual(
        Box((bearing_tab_width, bearing_tab_depth, bearing_tab_height)),
        origin=Origin(
            xyz=(
                opening_width * 0.5 - bearing_tab_width * 0.5,
                -frame_depth * 0.5 + hood_lip_thickness + bearing_tab_depth * 0.5,
                hinge_z - 0.004,
            )
        ),
        material=frame_charcoal,
        name="right_rear_bearing_tab",
    )
    frame.visual(
        Box((stop_width, stop_depth, stop_height)),
        origin=Origin(
            xyz=(
                -(opening_width * 0.5 - stop_width * 0.5),
                0.0,
                opening_bottom + stop_height * 0.5,
            )
        ),
        material=gasket_black,
        name="left_stop",
    )
    frame.visual(
        Box((stop_width, stop_depth, stop_height)),
        origin=Origin(
            xyz=(
                opening_width * 0.5 - stop_width * 0.5,
                0.0,
                opening_bottom + stop_height * 0.5,
            )
        ),
        material=gasket_black,
        name="right_stop",
    )
    frame.visual(
        Box((door_width, seal_depth, seal_height)),
        origin=Origin(xyz=(0.0, 0.0, opening_bottom - seal_height * 0.5)),
        material=gasket_black,
        name="bottom_seal",
    )
    frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, frame_height * 0.5)),
    )

    flap = model.part("door_flap")
    flap.visual(
        Box((door_width, door_thickness, top_trim_height)),
        origin=Origin(xyz=(0.0, 0.0, -top_trim_height * 0.5)),
        material=flap_trim,
        name="top_trim",
    )
    flap.visual(
        Box((door_width, door_thickness, bottom_trim_height)),
        origin=Origin(xyz=(0.0, 0.0, -(door_height - bottom_trim_height * 0.5))),
        material=flap_trim,
        name="bottom_trim",
    )
    flap.visual(
        Box((border_width, door_thickness, side_trim_height)),
        origin=Origin(
            xyz=(
                -(door_width * 0.5 - border_width * 0.5),
                0.0,
                -(top_trim_height + side_trim_height * 0.5),
            )
        ),
        material=flap_trim,
        name="left_trim",
    )
    flap.visual(
        Box((border_width, door_thickness, side_trim_height)),
        origin=Origin(
            xyz=(
                door_width * 0.5 - border_width * 0.5,
                0.0,
                -(top_trim_height + side_trim_height * 0.5),
            )
        ),
        material=flap_trim,
        name="right_trim",
    )
    flap.visual(
        Box((0.024, door_thickness, 0.018)),
        origin=Origin(xyz=(-0.113, 0.0, -0.010)),
        material=flap_trim,
        name="left_hinge_boss",
    )
    flap.visual(
        Box((0.024, door_thickness, 0.018)),
        origin=Origin(xyz=(0.113, 0.0, -0.010)),
        material=flap_trim,
        name="right_hinge_boss",
    )
    flap.visual(
        Box((clear_width, 0.012, clear_height)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -(top_trim_height + side_trim_height * 0.5),
            )
        ),
        material=clear_polycarbonate,
        name="clear_panel",
    )
    flap.visual(
        Cylinder(radius=0.0055, length=0.015),
        origin=Origin(xyz=(-0.118, 0.0, -0.004), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=flap_trim,
        name="left_pivot_pin",
    )
    flap.visual(
        Cylinder(radius=0.0055, length=0.015),
        origin=Origin(xyz=(0.118, 0.0, -0.004), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=flap_trim,
        name="right_pivot_pin",
    )
    flap.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -door_height * 0.5)),
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-math.radians(45.0),
            upper=math.radians(45.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("outer_frame")
    flap = object_model.get_part("door_flap")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    hood_roof = frame.get_visual("hood_roof")
    left_front_bearing_tab = frame.get_visual("left_front_bearing_tab")
    left_rear_bearing_tab = frame.get_visual("left_rear_bearing_tab")
    right_front_bearing_tab = frame.get_visual("right_front_bearing_tab")
    right_rear_bearing_tab = frame.get_visual("right_rear_bearing_tab")
    left_stop = frame.get_visual("left_stop")
    right_stop = frame.get_visual("right_stop")
    bottom_seal = frame.get_visual("bottom_seal")
    top_trim = flap.get_visual("top_trim")
    left_trim = flap.get_visual("left_trim")
    right_trim = flap.get_visual("right_trim")
    bottom_trim = flap.get_visual("bottom_trim")
    clear_panel = flap.get_visual("clear_panel")
    left_pin = flap.get_visual("left_pivot_pin")
    right_pin = flap.get_visual("right_pivot_pin")

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

    ctx.expect_origin_distance(flap, frame, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        flap,
        frame,
        axis="z",
        positive_elem=bottom_trim,
        negative_elem=bottom_seal,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="x",
        positive_elem=left_trim,
        negative_elem=left_stop,
        min_gap=0.0015,
        max_gap=0.0035,
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="x",
        positive_elem=right_stop,
        negative_elem=right_trim,
        min_gap=0.0015,
        max_gap=0.0035,
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="z",
        positive_elem=hood_roof,
        negative_elem=top_trim,
        min_gap=0.085,
        max_gap=0.095,
    )
    ctx.expect_within(flap, frame, axes="x", inner_elem=clear_panel, outer_elem=bottom_seal)
    ctx.expect_overlap(flap, frame, axes="x", elem_a=left_pin, elem_b=hood_roof, min_overlap=0.010)
    ctx.expect_overlap(flap, frame, axes="x", elem_a=right_pin, elem_b=hood_roof, min_overlap=0.010)
    ctx.expect_overlap(flap, frame, axes="z", elem_a=left_pin, elem_b=left_front_bearing_tab, min_overlap=0.010)
    ctx.expect_overlap(flap, frame, axes="z", elem_a=right_pin, elem_b=right_front_bearing_tab, min_overlap=0.010)
    ctx.expect_gap(
        frame,
        flap,
        axis="y",
        positive_elem=left_front_bearing_tab,
        negative_elem=left_pin,
        min_gap=0.008,
        max_gap=0.010,
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="y",
        positive_elem=left_pin,
        negative_elem=left_rear_bearing_tab,
        min_gap=0.008,
        max_gap=0.010,
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="y",
        positive_elem=right_front_bearing_tab,
        negative_elem=right_pin,
        min_gap=0.008,
        max_gap=0.010,
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="y",
        positive_elem=right_pin,
        negative_elem=right_rear_bearing_tab,
        min_gap=0.008,
        max_gap=0.010,
    )

    rest_bottom_aabb = ctx.part_element_world_aabb(flap, elem=bottom_trim)
    with ctx.pose({flap_hinge: math.radians(42.0)}):
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem=bottom_trim,
            negative_elem=bottom_seal,
            min_gap=0.068,
        )
        open_forward_aabb = ctx.part_element_world_aabb(flap, elem=bottom_trim)
        if rest_bottom_aabb is not None and open_forward_aabb is not None:
            rest_center_y = 0.5 * (rest_bottom_aabb[0][1] + rest_bottom_aabb[1][1])
            open_center_y = 0.5 * (open_forward_aabb[0][1] + open_forward_aabb[1][1])
            rest_center_x = 0.5 * (rest_bottom_aabb[0][0] + rest_bottom_aabb[1][0])
            open_center_x = 0.5 * (open_forward_aabb[0][0] + open_forward_aabb[1][0])
            ctx.check(
                "flap_opens_forward_from_top_axis",
                open_center_y > rest_center_y + 0.11,
                details=f"bottom trim center y only moved from {rest_center_y:.4f} to {open_center_y:.4f}",
            )
            ctx.check(
                "flap_keeps_horizontal_hinge_alignment_when_opening_forward",
                abs(open_center_x - rest_center_x) < 0.002,
                details=f"bottom trim center x shifted from {rest_center_x:.4f} to {open_center_x:.4f}",
            )

    with ctx.pose({flap_hinge: -math.radians(42.0)}):
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem=bottom_trim,
            negative_elem=bottom_seal,
            min_gap=0.068,
        )
        open_rear_aabb = ctx.part_element_world_aabb(flap, elem=bottom_trim)
        if rest_bottom_aabb is not None and open_rear_aabb is not None:
            rest_center_y = 0.5 * (rest_bottom_aabb[0][1] + rest_bottom_aabb[1][1])
            rear_center_y = 0.5 * (open_rear_aabb[0][1] + open_rear_aabb[1][1])
            ctx.check(
                "flap_opens_backward_from_top_axis",
                rear_center_y < rest_center_y - 0.11,
                details=f"bottom trim center y only moved from {rest_center_y:.4f} to {rear_center_y:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
