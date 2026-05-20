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

    steel = model.material("painted_steel", rgba=(0.62, 0.66, 0.70, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.46, 0.49, 0.53, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.27, 0.28, 0.30, 1.0))

    flange_w = 0.44
    flange_d = 0.36
    flange_t = 0.006

    body_w = 0.34
    body_d = 0.26
    curb_h = 0.09
    curb_wall = 0.018

    tower_h = 0.44
    wall_t = 0.012
    tower_top_z = curb_h + tower_h
    front_face_y = body_d * 0.5

    outlet_w = 0.24
    outlet_h = 0.16
    outlet_bottom_z = 0.27
    outlet_top_z = outlet_bottom_z + outlet_h
    outlet_center_z = outlet_bottom_z + outlet_h * 0.5

    frame_depth = 0.026
    frame_trim_depth = 0.020
    frame_side_w = (body_w - outlet_w) * 0.5
    frame_sill_h = 0.018
    frame_header_h = 0.020

    hinge_axis_y = front_face_y + 0.030
    hinge_axis_z = outlet_top_z + 0.014

    housing = model.part("housing")
    housing.visual(
        Box((flange_w, flange_d, flange_t)),
        origin=Origin(xyz=(0.0, 0.0, flange_t * 0.5)),
        material=shadow_steel,
        name="roof_flange",
    )

    housing.visual(
        Box((body_w, curb_wall, curb_h)),
        origin=Origin(xyz=(0.0, front_face_y - curb_wall * 0.5, curb_h * 0.5)),
        material=steel,
        name="curb_front",
    )
    housing.visual(
        Box((body_w, curb_wall, curb_h)),
        origin=Origin(xyz=(0.0, -front_face_y + curb_wall * 0.5, curb_h * 0.5)),
        material=steel,
        name="curb_back",
    )
    housing.visual(
        Box((curb_wall, body_d - 2.0 * curb_wall, curb_h)),
        origin=Origin(
            xyz=((body_w - curb_wall) * 0.5, 0.0, curb_h * 0.5),
        ),
        material=steel,
        name="curb_right",
    )
    housing.visual(
        Box((curb_wall, body_d - 2.0 * curb_wall, curb_h)),
        origin=Origin(
            xyz=(-(body_w - curb_wall) * 0.5, 0.0, curb_h * 0.5),
        ),
        material=steel,
        name="curb_left",
    )

    upper_center_z = curb_h + tower_h * 0.5
    housing.visual(
        Box((body_w, wall_t, tower_h)),
        origin=Origin(
            xyz=(0.0, -front_face_y + wall_t * 0.5, upper_center_z),
        ),
        material=steel,
        name="back_wall",
    )
    housing.visual(
        Box((wall_t, body_d - wall_t, tower_h)),
        origin=Origin(
            xyz=((body_w - wall_t) * 0.5, -wall_t * 0.25, upper_center_z),
        ),
        material=steel,
        name="right_wall",
    )
    housing.visual(
        Box((wall_t, body_d - wall_t, tower_h)),
        origin=Origin(
            xyz=(-(body_w - wall_t) * 0.5, -wall_t * 0.25, upper_center_z),
        ),
        material=steel,
        name="left_wall",
    )
    housing.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, tower_top_z - wall_t * 0.5)),
        material=steel,
        name="roof_cap",
    )
    housing.visual(
        Box((body_w, wall_t, outlet_bottom_z - curb_h)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y - wall_t * 0.5,
                curb_h + (outlet_bottom_z - curb_h) * 0.5,
            ),
        ),
        material=steel,
        name="front_lower_wall",
    )

    frame_center_y = front_face_y - frame_depth * 0.5
    housing.visual(
        Box((body_w, frame_depth, frame_header_h)),
        origin=Origin(
            xyz=(0.0, frame_center_y, outlet_top_z + frame_header_h * 0.5),
        ),
        material=shadow_steel,
        name="outlet_header",
    )
    housing.visual(
        Box((body_w, frame_depth, frame_sill_h)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                outlet_bottom_z - frame_sill_h * 0.5,
            ),
        ),
        material=shadow_steel,
        name="outlet_sill",
    )
    housing.visual(
        Box((frame_side_w, frame_depth, outlet_h)),
        origin=Origin(
            xyz=((body_w - frame_side_w) * 0.5, frame_center_y, outlet_center_z),
        ),
        material=shadow_steel,
        name="outlet_right_jamb",
    )
    housing.visual(
        Box((frame_side_w, frame_depth, outlet_h)),
        origin=Origin(
            xyz=(-(body_w - frame_side_w) * 0.5, frame_center_y, outlet_center_z),
        ),
        material=shadow_steel,
        name="outlet_left_jamb",
    )

    trim_center_y = front_face_y - frame_trim_depth * 0.5 + 0.003
    housing.visual(
        Box((body_w, frame_trim_depth, 0.012)),
        origin=Origin(xyz=(0.0, trim_center_y, outlet_top_z + 0.006)),
        material=dark_hardware,
        name="outlet_top_trim",
    )
    housing.visual(
        Box((body_w, frame_trim_depth, 0.010)),
        origin=Origin(xyz=(0.0, trim_center_y, outlet_bottom_z - 0.005)),
        material=dark_hardware,
        name="outlet_bottom_trim",
    )
    housing.visual(
        Box((0.030, frame_trim_depth, outlet_h + 0.014)),
        origin=Origin(
            xyz=((outlet_w * 0.5) + 0.015, trim_center_y, outlet_center_z),
        ),
        material=dark_hardware,
        name="outlet_right_trim",
    )
    housing.visual(
        Box((0.030, frame_trim_depth, outlet_h + 0.014)),
        origin=Origin(
            xyz=(-(outlet_w * 0.5) - 0.015, trim_center_y, outlet_center_z),
        ),
        material=dark_hardware,
        name="outlet_left_trim",
    )

    bracket_plate_w = 0.012
    bracket_depth = hinge_axis_y - front_face_y + 0.010
    bracket_h = 0.052
    bracket_center_y = front_face_y + bracket_depth * 0.5 - 0.004
    bracket_center_z = hinge_axis_z - 0.010
    housing.visual(
        Box((bracket_plate_w, bracket_depth, bracket_h)),
        origin=Origin(
            xyz=((body_w - bracket_plate_w) * 0.5, bracket_center_y, bracket_center_z),
        ),
        material=dark_hardware,
        name="right_hinge_bracket",
    )
    housing.visual(
        Box((bracket_plate_w, bracket_depth, bracket_h)),
        origin=Origin(
            xyz=(
                -(body_w - bracket_plate_w) * 0.5,
                bracket_center_y,
                bracket_center_z,
            ),
        ),
        material=dark_hardware,
        name="left_hinge_bracket",
    )

    lug_radius = 0.012
    lug_length = 0.020
    lug_x_center = outlet_w * 0.5 + 0.030
    housing.visual(
        Cylinder(radius=lug_radius, length=lug_length),
        origin=Origin(
            xyz=(lug_x_center, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_hardware,
        name="right_hinge_lug",
    )
    housing.visual(
        Cylinder(radius=lug_radius, length=lug_length),
        origin=Origin(
            xyz=(-lug_x_center, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_hardware,
        name="left_hinge_lug",
    )

    housing.inertial = Inertial.from_geometry(
        Box((flange_w, flange_d, tower_top_z)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, tower_top_z * 0.5)),
    )

    flap = model.part("weather_flap")
    flap_panel_w = 0.28
    flap_panel_h = 0.19
    flap_panel_t = 0.008
    flap.visual(
        Cylinder(radius=0.011, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_hardware,
        name="flap_barrel",
    )
    flap.visual(
        Box((flap_panel_w, flap_panel_t, flap_panel_h)),
        origin=Origin(xyz=(0.0, -0.013, -flap_panel_h * 0.5)),
        material=steel,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_panel_w * 0.92, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, -flap_panel_h + 0.009)),
        material=shadow_steel,
        name="flap_bottom_stiffener",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_panel_w, 0.030, flap_panel_h)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.010, -flap_panel_h * 0.5)),
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
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
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.expect_gap(
        flap,
        housing,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="front_lower_wall",
        min_gap=0.008,
        max_gap=0.028,
        name="closed flap hangs just ahead of the outlet face",
    )
    ctx.expect_overlap(
        flap,
        housing,
        axes="x",
        elem_a="flap_panel",
        elem_b="front_lower_wall",
        min_overlap=0.26,
        name="flap spans the outlet width",
    )

    front_wall_aabb = ctx.part_element_world_aabb(housing, elem="front_lower_wall")
    hinge_y = hinge.origin.xyz[1]
    front_face_max_y = None if front_wall_aabb is None else front_wall_aabb[1][1]
    ctx.check(
        "hinge axis is visibly pulled forward from the outlet frame",
        front_face_max_y is not None and hinge_y > front_face_max_y + 0.020,
        details=f"hinge_y={hinge_y}, front_face_max_y={front_face_max_y}",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 1.0}):
        opened_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "flap opens upward and outward",
        closed_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.06
        and opened_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.08,
        details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
