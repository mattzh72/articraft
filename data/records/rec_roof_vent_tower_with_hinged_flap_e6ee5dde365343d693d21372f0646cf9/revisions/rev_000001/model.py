from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    throat_dark = model.material("throat_shadow", rgba=(0.18, 0.19, 0.20, 1.0))

    curb_width = 0.52
    curb_depth = 0.44
    curb_height = 0.10

    tower_width = 0.36
    tower_depth = 0.26
    tower_height = 0.78
    wall_thickness = 0.02
    roof_thickness = 0.025

    outlet_outer_width = 0.34
    outlet_inner_width = 0.30
    outlet_height = 0.26
    outlet_bottom_z = 0.58
    outlet_center_z = outlet_bottom_z + outlet_height * 0.5
    outlet_top_z = outlet_bottom_z + outlet_height
    frame_depth = 0.050
    frame_front_y = 0.145
    frame_center_y = frame_front_y - frame_depth * 0.5
    cheek_depth = 0.090
    cheek_center_y = 0.115

    housing = model.part("housing")
    housing.visual(
        Box((curb_width, curb_depth, curb_height)),
        origin=Origin(xyz=(0.0, 0.0, curb_height * 0.5)),
        material=steel_dark,
        name="roof_curb",
    )
    housing.visual(
        Box((wall_thickness, tower_depth, tower_height)),
        origin=Origin(
            xyz=(
                -(tower_width - wall_thickness) * 0.5,
                0.0,
                curb_height + tower_height * 0.5,
            )
        ),
        material=galvanized,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, tower_depth, tower_height)),
        origin=Origin(
            xyz=(
                (tower_width - wall_thickness) * 0.5,
                0.0,
                curb_height + tower_height * 0.5,
            )
        ),
        material=galvanized,
        name="right_wall",
    )
    housing.visual(
        Box((tower_width, wall_thickness, tower_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(tower_depth - wall_thickness) * 0.5,
                curb_height + tower_height * 0.5,
            )
        ),
        material=galvanized,
        name="back_wall",
    )
    housing.visual(
        Box((tower_width, wall_thickness, 0.46)),
        origin=Origin(xyz=(0.0, (tower_depth - wall_thickness) * 0.5, 0.33)),
        material=galvanized,
        name="front_apron",
    )
    housing.visual(
        Box((tower_width, tower_depth, roof_thickness)),
        origin=Origin(
            xyz=(0.0, -0.005, curb_height + tower_height - roof_thickness * 0.5)
        ),
        material=galvanized,
        name="roof_cap",
    )

    housing.visual(
        Box((outlet_outer_width, frame_depth, 0.04)),
        origin=Origin(xyz=(0.0, frame_center_y, outlet_bottom_z - 0.02)),
        material=galvanized,
        name="outlet_sill",
    )
    housing.visual(
        Box((outlet_outer_width, frame_depth, 0.04)),
        origin=Origin(xyz=(0.0, frame_center_y, outlet_top_z + 0.01)),
        material=galvanized,
        name="outlet_header",
    )
    housing.visual(
        Box((0.02, frame_depth, outlet_height)),
        origin=Origin(
            xyz=(
                -(outlet_outer_width - 0.02) * 0.5,
                frame_center_y,
                outlet_center_z,
            )
        ),
        material=galvanized,
        name="left_jamb",
    )
    housing.visual(
        Box((0.02, frame_depth, outlet_height)),
        origin=Origin(
            xyz=(
                (outlet_outer_width - 0.02) * 0.5,
                frame_center_y,
                outlet_center_z,
            )
        ),
        material=galvanized,
        name="right_jamb",
    )
    housing.visual(
        Box((0.03, cheek_depth, 0.32)),
        origin=Origin(xyz=(-0.165, cheek_center_y, 0.70)),
        material=galvanized,
        name="left_cheek",
    )
    housing.visual(
        Box((0.03, cheek_depth, 0.32)),
        origin=Origin(xyz=(0.165, cheek_center_y, 0.70)),
        material=galvanized,
        name="right_cheek",
    )

    throat_depth = 0.10
    throat_center_y = 0.045
    housing.visual(
        Box((outlet_inner_width - 0.03, 0.012, outlet_height + 0.02)),
        origin=Origin(xyz=(0.0, throat_center_y - throat_depth * 0.5, outlet_center_z)),
        material=throat_dark,
        name="throat_back",
    )
    housing.visual(
        Box((0.018, throat_depth, outlet_height + 0.02)),
        origin=Origin(xyz=(-0.141, throat_center_y, outlet_center_z)),
        material=throat_dark,
        name="throat_left_liner",
    )
    housing.visual(
        Box((0.018, throat_depth, outlet_height + 0.02)),
        origin=Origin(xyz=(0.141, throat_center_y, outlet_center_z)),
        material=throat_dark,
        name="throat_right_liner",
    )
    housing.visual(
        Box((outlet_inner_width - 0.03, throat_depth, 0.018)),
        origin=Origin(xyz=(0.0, throat_center_y, outlet_top_z - 0.005)),
        material=throat_dark,
        name="throat_top_liner",
    )
    housing.visual(
        Box((outlet_inner_width - 0.03, throat_depth, 0.018)),
        origin=Origin(xyz=(0.0, throat_center_y, outlet_bottom_z + 0.005)),
        material=throat_dark,
        name="throat_bottom_liner",
    )
    housing.visual(
        Box((outlet_inner_width - 0.012, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.141, outlet_top_z - 0.020)),
        material=galvanized,
        name="stop_top",
    )
    housing.visual(
        Box((outlet_inner_width - 0.012, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.141, outlet_bottom_z + 0.020)),
        material=galvanized,
        name="stop_bottom",
    )
    housing.visual(
        Box((0.018, 0.006, outlet_height - 0.04)),
        origin=Origin(xyz=(-0.141, 0.141, outlet_center_z)),
        material=galvanized,
        name="stop_left",
    )
    housing.visual(
        Box((0.018, 0.006, outlet_height - 0.04)),
        origin=Origin(xyz=(0.141, 0.141, outlet_center_z)),
        material=galvanized,
        name="stop_right",
    )
    housing.inertial = Inertial.from_geometry(
        Box((curb_width, curb_depth, curb_height + tower_height)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, (curb_height + tower_height) * 0.5)),
    )

    flap_width = 0.28
    flap_height = 0.24
    flap_thickness = 0.012
    hinge_barrel_radius = 0.012
    hinge_barrel_length = 0.26

    flap = model.part("weather_flap")
    flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        origin=Origin(xyz=(0.0, flap_thickness * 0.5, -flap_height * 0.5)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_width, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.009, -flap_height + 0.012)),
        material=steel_dark,
        name="bottom_hem",
    )
    flap.visual(
        Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel_dark,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, 0.024, flap_height)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.012, -flap_height * 0.5)),
    )

    model.articulation(
        "housing_to_weather_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(0.0, frame_front_y, outlet_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("housing_to_weather_flap")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            flap,
            housing,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="stop_top",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed flap sits just ahead of the outlet stops",
        )
        ctx.expect_overlap(
            flap,
            housing,
            axes="x",
            elem_a="flap_panel",
            elem_b="stop_top",
            min_overlap=0.24,
            name="closed flap covers the outlet width",
        )

        closed_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({hinge: 1.10}):
        open_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    closed_max_y = closed_panel_aabb[1][1] if closed_panel_aabb is not None else None
    open_max_y = open_panel_aabb[1][1] if open_panel_aabb is not None else None
    closed_min_z = closed_panel_aabb[0][2] if closed_panel_aabb is not None else None
    open_min_z = open_panel_aabb[0][2] if open_panel_aabb is not None else None
    ctx.check(
        "flap swings outward when opened",
        closed_max_y is not None
        and open_max_y is not None
        and open_max_y > closed_max_y + 0.18,
        details=f"closed_max_y={closed_max_y}, open_max_y={open_max_y}",
    )
    ctx.check(
        "flap lower edge lifts when opened",
        closed_min_z is not None
        and open_min_z is not None
        and open_min_z > closed_min_z + 0.10,
        details=f"closed_min_z={closed_min_z}, open_min_z={open_min_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
