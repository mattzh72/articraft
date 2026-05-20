from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.60, 0.62, 0.66, 1.0))
    interior_dark = model.material("interior_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    flap_finish = model.material("flap_finish", rgba=(0.66, 0.68, 0.72, 1.0))

    flashing_width = 0.88
    flashing_depth = 0.68
    flashing_thickness = 0.018

    tower_width = 0.56
    tower_depth = 0.42
    wall_thickness = 0.018
    wall_height = 1.10

    outlet_width = 0.42
    outlet_height = 0.32
    outlet_bottom_z = flashing_thickness + 0.70
    outlet_top_z = outlet_bottom_z + outlet_height

    front_face_y = tower_depth * 0.5
    front_wall_center_y = front_face_y - wall_thickness * 0.5
    inner_front_face_y = front_face_y - wall_thickness

    housing = model.part("housing")
    housing.visual(
        Box((flashing_width, flashing_depth, flashing_thickness)),
        origin=Origin(xyz=(0.0, 0.0, flashing_thickness * 0.5)),
        material=galvanized,
        name="roof_flashing",
    )
    housing.visual(
        Box((wall_thickness, tower_depth, wall_height)),
        origin=Origin(
            xyz=(
                -(tower_width * 0.5 - wall_thickness * 0.5),
                0.0,
                flashing_thickness + wall_height * 0.5,
            )
        ),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, tower_depth, wall_height)),
        origin=Origin(
            xyz=(
                tower_width * 0.5 - wall_thickness * 0.5,
                0.0,
                flashing_thickness + wall_height * 0.5,
            )
        ),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((tower_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(tower_depth * 0.5 - wall_thickness * 0.5),
                flashing_thickness + wall_height * 0.5,
            )
        ),
        material=galvanized,
        name="back_wall",
    )

    lower_front_height = outlet_bottom_z - flashing_thickness
    housing.visual(
        Box((tower_width - 2.0 * wall_thickness, wall_thickness, lower_front_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_wall_center_y,
                flashing_thickness + lower_front_height * 0.5,
            )
        ),
        material=galvanized,
        name="lower_front_wall",
    )

    front_jamb_width = ((tower_width - 2.0 * wall_thickness) - outlet_width) * 0.5
    upper_front_height = wall_height - lower_front_height
    jamb_center_z = outlet_bottom_z + upper_front_height * 0.5
    housing.visual(
        Box((front_jamb_width, wall_thickness, upper_front_height)),
        origin=Origin(
            xyz=(
                -(outlet_width * 0.5 + front_jamb_width * 0.5),
                front_wall_center_y,
                jamb_center_z,
            )
        ),
        material=galvanized,
        name="left_front_jamb",
    )
    housing.visual(
        Box((front_jamb_width, wall_thickness, upper_front_height)),
        origin=Origin(
            xyz=(
                outlet_width * 0.5 + front_jamb_width * 0.5,
                front_wall_center_y,
                jamb_center_z,
            )
        ),
        material=galvanized,
        name="right_front_jamb",
    )

    header_height = flashing_thickness + wall_height - outlet_top_z
    housing.visual(
        Box((outlet_width, wall_thickness, header_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_wall_center_y,
                outlet_top_z + header_height * 0.5,
            )
        ),
        material=galvanized,
        name="front_header",
    )

    roof_cap_thickness = 0.020
    housing.visual(
        Box((0.60, 0.46, roof_cap_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                flashing_thickness + wall_height + roof_cap_thickness * 0.5,
            )
        ),
        material=frame_finish,
        name="roof_cap",
    )

    outer_frame_width = 0.50
    frame_depth = 0.032
    frame_side_width = (outer_frame_width - outlet_width) * 0.5
    top_frame_height = 0.062
    sill_height = 0.050
    frame_center_y = front_face_y + frame_depth * 0.5
    side_frame_height = outlet_height + top_frame_height + sill_height
    housing.visual(
        Box((frame_side_width, frame_depth, side_frame_height)),
        origin=Origin(
            xyz=(
                -(outlet_width * 0.5 + frame_side_width * 0.5),
                frame_center_y,
                outlet_bottom_z - sill_height * 0.5 + side_frame_height * 0.5,
            )
        ),
        material=frame_finish,
        name="left_outlet_frame",
    )
    housing.visual(
        Box((frame_side_width, frame_depth, side_frame_height)),
        origin=Origin(
            xyz=(
                outlet_width * 0.5 + frame_side_width * 0.5,
                frame_center_y,
                outlet_bottom_z - sill_height * 0.5 + side_frame_height * 0.5,
            )
        ),
        material=frame_finish,
        name="right_outlet_frame",
    )
    housing.visual(
        Box((outer_frame_width, frame_depth, top_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                frame_center_y,
                outlet_top_z + top_frame_height * 0.5,
            )
        ),
        material=frame_finish,
        name="top_outlet_frame",
    )
    housing.visual(
        Box((outer_frame_width, 0.044, sill_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + 0.022,
                outlet_bottom_z - sill_height * 0.5,
            )
        ),
        material=frame_finish,
        name="outlet_sill",
    )
    housing.visual(
        Box((0.52, 0.008, 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + 0.036,
                outlet_top_z + 0.010,
            )
        ),
        material=frame_finish,
        name="hinge_mount",
    )

    throat_depth = 0.10
    throat_thickness = 0.016
    throat_center_y = inner_front_face_y - throat_depth * 0.5
    housing.visual(
        Box((throat_thickness, throat_depth, outlet_height)),
        origin=Origin(
            xyz=(-outlet_width * 0.5 + throat_thickness * 0.5, throat_center_y, outlet_bottom_z + outlet_height * 0.5)
        ),
        material=interior_dark,
        name="left_throat_wall",
    )
    housing.visual(
        Box((throat_thickness, throat_depth, outlet_height)),
        origin=Origin(
            xyz=(outlet_width * 0.5 - throat_thickness * 0.5, throat_center_y, outlet_bottom_z + outlet_height * 0.5)
        ),
        material=interior_dark,
        name="right_throat_wall",
    )
    housing.visual(
        Box((outlet_width, throat_depth, throat_thickness)),
        origin=Origin(
            xyz=(0.0, throat_center_y, outlet_top_z - throat_thickness * 0.5)
        ),
        material=interior_dark,
        name="top_throat",
    )
    housing.visual(
        Box((outlet_width, throat_depth, throat_thickness)),
        origin=Origin(
            xyz=(0.0, throat_center_y, outlet_bottom_z + throat_thickness * 0.5)
        ),
        material=interior_dark,
        name="bottom_throat",
    )

    housing.inertial = Inertial.from_geometry(
        Box((flashing_width, flashing_depth, flashing_thickness + wall_height + roof_cap_thickness)),
        mass=62.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (flashing_thickness + wall_height + roof_cap_thickness) * 0.5,
            )
        ),
    )

    flap = model.part("weather_flap")
    flap_width = 0.50
    flap_height = 0.40
    flap_thickness = 0.012
    flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, flap_thickness * 0.5, -flap_height * 0.5)),
        material=flap_finish,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_width, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, -0.010)),
        material=flap_finish,
        name="top_hem",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_thickness, flap_height)),
        mass=5.5,
        origin=Origin(xyz=(0.0, flap_thickness * 0.5, -flap_height * 0.5)),
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + 0.044 + 0.004,
                outlet_top_z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("housing_to_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            flap,
            housing,
            elem_a="top_hem",
            elem_b="hinge_mount",
            name="flap remains physically hung from the top hinge mount",
        )
        ctx.expect_gap(
            flap,
            housing,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="top_outlet_frame",
            min_gap=0.014,
            max_gap=0.018,
            name="closed flap sits just proud of the framed outlet",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="x",
            elem_a="flap_panel",
            elem_b="top_outlet_frame",
            min_overlap=0.49,
            name="flap spans the outlet frame width",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="z",
            elem_a="flap_panel",
            elem_b="outlet_sill",
            min_overlap=0.045,
            name="closed flap reaches down over the sill",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({hinge: 1.30}):
        open_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    outward_opens = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.25
        and open_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.20
    )
    ctx.check(
        "flap opens outward and lifts clear of the outlet",
        outward_opens,
        details=f"closed_panel_aabb={closed_panel_aabb}, open_panel_aabb={open_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
