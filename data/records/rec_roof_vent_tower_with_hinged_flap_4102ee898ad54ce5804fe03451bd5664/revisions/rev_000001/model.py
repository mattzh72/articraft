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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    body_paint = model.material("body_paint", rgba=(0.62, 0.66, 0.69, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.50, 0.54, 0.57, 1.0))
    flap_paint = model.material("flap_paint", rgba=(0.72, 0.75, 0.78, 1.0))
    hardware = model.material("hardware", rgba=(0.24, 0.26, 0.28, 1.0))

    curb_width = 0.66
    curb_depth = 0.54
    curb_height = 0.16
    curb_wall = 0.03

    tower_width = 0.58
    tower_depth = 0.44
    tower_height = 0.84
    wall_t = 0.025
    tower_base_z = curb_height - 0.012
    tower_center_z = tower_base_z + tower_height * 0.5
    tower_front_y = tower_depth * 0.5

    opening_width = 0.26
    opening_height = 0.26
    opening_center_x = -0.055
    opening_bottom_z = 0.53
    opening_top_z = opening_bottom_z + opening_height
    opening_center_z = (opening_bottom_z + opening_top_z) * 0.5

    half_tower_w = tower_width * 0.5
    opening_left_x = opening_center_x - opening_width * 0.5
    opening_right_x = opening_center_x + opening_width * 0.5
    left_jamb_width = opening_left_x - (-half_tower_w)
    right_jamb_width = half_tower_w - opening_right_x

    outlet_outer_width = 0.32
    outlet_outer_height = 0.32
    outlet_frame_depth = 0.065
    outlet_frame_y = tower_front_y + outlet_frame_depth * 0.5 - 0.005
    outlet_border = (outlet_outer_width - opening_width) * 0.5

    housing = model.part("housing")

    # Roof curb / base ring.
    housing.visual(
        Box((curb_width, curb_wall, curb_height)),
        origin=Origin(xyz=(0.0, curb_depth * 0.5 - curb_wall * 0.5, curb_height * 0.5)),
        material=body_paint,
        name="curb_front",
    )
    housing.visual(
        Box((curb_width, curb_wall, curb_height)),
        origin=Origin(xyz=(0.0, -curb_depth * 0.5 + curb_wall * 0.5, curb_height * 0.5)),
        material=body_paint,
        name="curb_back",
    )
    housing.visual(
        Box((curb_wall, curb_depth - 2.0 * curb_wall, curb_height)),
        origin=Origin(
            xyz=(-curb_width * 0.5 + curb_wall * 0.5, 0.0, curb_height * 0.5)
        ),
        material=body_paint,
        name="curb_left",
    )
    housing.visual(
        Box((curb_wall, curb_depth - 2.0 * curb_wall, curb_height)),
        origin=Origin(
            xyz=(curb_width * 0.5 - curb_wall * 0.5, 0.0, curb_height * 0.5)
        ),
        material=body_paint,
        name="curb_right",
    )
    housing.visual(
        Box((tower_width + 0.02, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.233, 0.155)),
        material=body_paint,
        name="base_transition_front",
    )
    housing.visual(
        Box((tower_width + 0.02, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, -0.233, 0.155)),
        material=body_paint,
        name="base_transition_back",
    )
    housing.visual(
        Box((0.06, tower_depth + 0.02, 0.05)),
        origin=Origin(xyz=(-0.285, 0.0, 0.155)),
        material=body_paint,
        name="base_transition_left",
    )
    housing.visual(
        Box((0.06, tower_depth + 0.02, 0.05)),
        origin=Origin(xyz=(0.285, 0.0, 0.155)),
        material=body_paint,
        name="base_transition_right",
    )

    # Upright housing shell.
    housing.visual(
        Box((wall_t, tower_depth, tower_height)),
        origin=Origin(xyz=(-half_tower_w + wall_t * 0.5, 0.0, tower_center_z)),
        material=body_paint,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, tower_depth, tower_height)),
        origin=Origin(xyz=(half_tower_w - wall_t * 0.5, 0.0, tower_center_z)),
        material=body_paint,
        name="right_wall",
    )
    housing.visual(
        Box((tower_width, wall_t, tower_height)),
        origin=Origin(xyz=(0.0, -tower_front_y + wall_t * 0.5, tower_center_z)),
        material=body_paint,
        name="back_wall",
    )
    housing.visual(
        Box((tower_width + 0.04, tower_depth + 0.05, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, tower_base_z + tower_height - 0.001)),
        material=frame_paint,
        name="roof_cap",
    )

    front_lower_height = opening_bottom_z - tower_base_z
    housing.visual(
        Box((tower_width, wall_t, front_lower_height)),
        origin=Origin(
            xyz=(
                0.0,
                tower_front_y - wall_t * 0.5,
                tower_base_z + front_lower_height * 0.5,
            )
        ),
        material=body_paint,
        name="front_lower_panel",
    )

    front_upper_bottom = opening_top_z
    front_upper_height = tower_base_z + tower_height - 0.02 - front_upper_bottom
    housing.visual(
        Box((tower_width, wall_t, front_upper_height)),
        origin=Origin(
            xyz=(
                0.0,
                tower_front_y - wall_t * 0.5,
                front_upper_bottom + front_upper_height * 0.5,
            )
        ),
        material=body_paint,
        name="front_upper_panel",
    )

    housing.visual(
        Box((left_jamb_width, wall_t, opening_height)),
        origin=Origin(
            xyz=(
                -half_tower_w + left_jamb_width * 0.5,
                tower_front_y - wall_t * 0.5,
                opening_center_z,
            )
        ),
        material=body_paint,
        name="front_left_jamb",
    )
    housing.visual(
        Box((right_jamb_width, wall_t, opening_height)),
        origin=Origin(
            xyz=(
                opening_right_x + right_jamb_width * 0.5,
                tower_front_y - wall_t * 0.5,
                opening_center_z,
            )
        ),
        material=body_paint,
        name="front_right_jamb",
    )

    # Projecting outlet frame.
    housing.visual(
        Box((outlet_outer_width, outlet_frame_depth, outlet_border)),
        origin=Origin(
            xyz=(
                opening_center_x,
                outlet_frame_y,
                opening_top_z + outlet_border * 0.5,
            )
        ),
        material=frame_paint,
        name="outlet_trim_header",
    )
    housing.visual(
        Box((outlet_outer_width, outlet_frame_depth, outlet_border)),
        origin=Origin(
            xyz=(
                opening_center_x,
                outlet_frame_y,
                opening_bottom_z - outlet_border * 0.5,
            )
        ),
        material=frame_paint,
        name="outlet_trim_sill",
    )
    housing.visual(
        Box((outlet_border, outlet_frame_depth, outlet_outer_height)),
        origin=Origin(
            xyz=(
                opening_left_x - outlet_border * 0.5,
                outlet_frame_y,
                opening_center_z,
            )
        ),
        material=frame_paint,
        name="outlet_trim_left",
    )
    housing.visual(
        Box((outlet_border, outlet_frame_depth, outlet_outer_height)),
        origin=Origin(
            xyz=(
                opening_right_x + outlet_border * 0.5,
                outlet_frame_y,
                opening_center_z,
            )
        ),
        material=frame_paint,
        name="outlet_trim_right",
    )
    housing.visual(
        Box((outlet_outer_width + 0.03, 0.03, 0.016)),
        origin=Origin(
            xyz=(
                opening_center_x,
                outlet_frame_y + outlet_frame_depth * 0.15,
                opening_top_z + outlet_border + 0.01,
            )
        ),
        material=frame_paint,
        name="rain_lip",
    )
    housing.visual(
        Box((0.24, 0.014, 0.018)),
        origin=Origin(
            xyz=(
                opening_center_x,
                outlet_frame_y + outlet_frame_depth * 0.5 - 0.007,
                opening_top_z + outlet_border - 0.011,
            )
        ),
        material=hardware,
        name="hinge_mount_rail",
    )
    housing.visual(
        Box((0.018, 0.03, 0.028)),
        origin=Origin(
            xyz=(
                opening_center_x - outlet_outer_width * 0.5 + 0.02,
                outlet_frame_y + outlet_frame_depth * 0.12,
                opening_top_z + outlet_border - 0.002,
            )
        ),
        material=frame_paint,
        name="rain_lip_left_brace",
    )
    housing.visual(
        Box((0.018, 0.03, 0.028)),
        origin=Origin(
            xyz=(
                opening_center_x + outlet_outer_width * 0.5 - 0.02,
                outlet_frame_y + outlet_frame_depth * 0.12,
                opening_top_z + outlet_border - 0.002,
            )
        ),
        material=frame_paint,
        name="rain_lip_right_brace",
    )

    housing.inertial = Inertial.from_geometry(
        Box((curb_width, curb_depth, tower_base_z + tower_height)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, (tower_base_z + tower_height) * 0.5)),
    )

    flap = model.part("weather_flap")
    flap_width = outlet_outer_width + 0.02
    flap_height = outlet_outer_height - 0.01

    flap.visual(
        Box((flap_width, 0.006, flap_height)),
        origin=Origin(xyz=(0.0, 0.006, -flap_height * 0.5)),
        material=flap_paint,
        name="flap_sheet",
    )
    flap.visual(
        Box((flap_width, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.011, -0.022)),
        material=frame_paint,
        name="flap_top_stiffener",
    )
    flap.visual(
        Box((0.02, 0.014, flap_height - 0.03)),
        origin=Origin(
            xyz=(-flap_width * 0.5 + 0.01, 0.010, -(flap_height - 0.03) * 0.5 - 0.015)
        ),
        material=frame_paint,
        name="flap_left_edge",
    )
    flap.visual(
        Box((0.02, 0.014, flap_height - 0.03)),
        origin=Origin(
            xyz=(flap_width * 0.5 - 0.01, 0.010, -(flap_height - 0.03) * 0.5 - 0.015)
        ),
        material=frame_paint,
        name="flap_right_edge",
    )
    flap.visual(
        Box((flap_width - 0.03, 0.014, 0.02)),
        origin=Origin(xyz=(0.0, 0.010, -flap_height + 0.01)),
        material=frame_paint,
        name="flap_bottom_edge",
    )
    flap.visual(
        Cylinder(radius=0.008, length=flap_width - 0.02),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.03, flap_height)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.012, -flap_height * 0.48)),
    )

    model.articulation(
        "housing_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(
            xyz=(
                opening_center_x,
                outlet_frame_y + outlet_frame_depth * 0.5 + 0.001,
                opening_top_z + outlet_border,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("housing_to_weather_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            housing,
            axis="y",
            positive_elem="flap_sheet",
            negative_elem="outlet_trim_header",
            min_gap=0.004,
            max_gap=0.015,
            name="closed flap sits just ahead of outlet frame",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="x",
            elem_a="flap_sheet",
            elem_b="outlet_trim_header",
            min_overlap=0.30,
            name="flap spans the framed outlet width",
        )

        panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_sheet")
        header_aabb = ctx.part_element_world_aabb(housing, elem="outlet_trim_header")
        sill_aabb = ctx.part_element_world_aabb(housing, elem="outlet_trim_sill")
        covers_height = (
            panel_aabb is not None
            and header_aabb is not None
            and sill_aabb is not None
            and panel_aabb[0][2] < sill_aabb[1][2] - 0.01
            and panel_aabb[1][2] > header_aabb[0][2] + 0.01
        )
        ctx.check(
            "closed flap covers outlet height",
            covers_height,
            details=f"panel={panel_aabb}, header={header_aabb}, sill={sill_aabb}",
        )

    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({hinge: 1.1}):
        open_aabb = ctx.part_world_aabb(flap)
    opens_outward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.20
        and open_aabb[0][2] > closed_aabb[0][2] + 0.08
    )
    ctx.check(
        "flap opens outward and upward",
        opens_outward,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    left_jamb = ctx.part_element_world_aabb(housing, elem="front_left_jamb")
    right_jamb = ctx.part_element_world_aabb(housing, elem="front_right_jamb")
    asymmetric_body = (
        left_jamb is not None
        and right_jamb is not None
        and (right_jamb[1][0] - right_jamb[0][0]) > (left_jamb[1][0] - left_jamb[0][0]) + 0.09
    )
    ctx.check(
        "fixed housing is visibly heavier on the right side of the outlet",
        asymmetric_body,
        details=f"left={left_jamb}, right={right_jamb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
