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
    model = ArticulatedObject(name="residential_home_elevator")

    powder_coat = model.material("powder_coat", rgba=(0.86, 0.86, 0.88, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.72, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.58, 0.70, 0.78, 0.38))
    rail_metal = model.material("rail_metal", rgba=(0.44, 0.46, 0.50, 1.0))
    floor_finish = model.material("floor_finish", rgba=(0.42, 0.31, 0.22, 1.0))

    shaft_width = 1.26
    shaft_depth = 1.08
    shaft_height = 3.05
    base_thickness = 0.06
    top_thickness = 0.08

    car_width = 1.10
    car_depth = 0.80
    car_height = 2.15
    wall_thickness = 0.035
    floor_thickness = 0.05
    roof_thickness = 0.04

    opening_width = 0.52
    opening_height = 1.95
    front_frame_depth = 0.035
    side_pocket_width = (car_width - opening_width) / 2.0

    panel_width = opening_width / 2.0
    panel_height = 1.92
    panel_thickness = 0.022
    panel_travel = panel_width

    shaft_frame = model.part("shaft_frame")
    shaft_frame.visual(
        Box((shaft_width, shaft_depth, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=dark_trim,
        name="base_plinth",
    )
    shaft_frame.visual(
        Box((shaft_width, shaft_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, shaft_height - top_thickness / 2.0)),
        material=dark_trim,
        name="top_crosshead",
    )
    shaft_frame.visual(
        Box((shaft_width - 0.10, 0.05, shaft_height - base_thickness - top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                shaft_depth / 2.0 - 0.045,
                base_thickness + (shaft_height - base_thickness - top_thickness) / 2.0,
            )
        ),
        material=powder_coat,
        name="rear_backboard",
    )

    front_jamb_width = 0.08
    front_jamb_depth = 0.06
    front_jamb_height = shaft_height - base_thickness - top_thickness
    front_jamb_y = -shaft_depth / 2.0 + front_jamb_depth / 2.0
    front_jamb_z = base_thickness + front_jamb_height / 2.0
    front_jamb_x = shaft_width / 2.0 - front_jamb_width / 2.0
    shaft_frame.visual(
        Box((front_jamb_width, front_jamb_depth, front_jamb_height)),
        origin=Origin(xyz=(-front_jamb_x, front_jamb_y, front_jamb_z)),
        material=powder_coat,
        name="left_front_jamb",
    )
    shaft_frame.visual(
        Box((front_jamb_width, front_jamb_depth, front_jamb_height)),
        origin=Origin(xyz=(front_jamb_x, front_jamb_y, front_jamb_z)),
        material=powder_coat,
        name="right_front_jamb",
    )

    rear_post_width = 0.08
    rear_post_depth = 0.08
    rear_post_x = shaft_width / 2.0 - rear_post_width / 2.0
    rear_post_y = shaft_depth / 2.0 - rear_post_depth / 2.0
    shaft_frame.visual(
        Box((rear_post_width, rear_post_depth, front_jamb_height)),
        origin=Origin(xyz=(-rear_post_x, rear_post_y, front_jamb_z)),
        material=powder_coat,
        name="left_rear_post",
    )
    shaft_frame.visual(
        Box((rear_post_width, rear_post_depth, front_jamb_height)),
        origin=Origin(xyz=(rear_post_x, rear_post_y, front_jamb_z)),
        material=powder_coat,
        name="right_rear_post",
    )

    rail_height = shaft_height - 0.16
    rail_width = 0.04
    rail_depth = 0.03
    rail_y = car_depth / 2.0 + 0.045
    rail_x = 0.32
    rail_z = 0.10 + rail_height / 2.0
    shaft_frame.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(-rail_x, rail_y, rail_z)),
        material=rail_metal,
        name="left_guide_rail",
    )
    shaft_frame.visual(
        Box((rail_width, rail_depth, rail_height)),
        origin=Origin(xyz=(rail_x, rail_y, rail_z)),
        material=rail_metal,
        name="right_guide_rail",
    )
    shaft_frame.inertial = Inertial.from_geometry(
        Box((shaft_width, shaft_depth, shaft_height)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, shaft_height / 2.0)),
    )

    car = model.part("car")
    car.visual(
        Box((car_width, car_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=floor_finish,
        name="car_floor",
    )
    shell_height = car_height - floor_thickness
    wall_center_z = floor_thickness + shell_height / 2.0
    side_wall_x = car_width / 2.0 - wall_thickness / 2.0
    rear_wall_y = car_depth / 2.0 - wall_thickness / 2.0
    car.visual(
        Box((wall_thickness, car_depth, shell_height)),
        origin=Origin(xyz=(-side_wall_x, 0.0, wall_center_z)),
        material=powder_coat,
        name="left_wall",
    )
    car.visual(
        Box((wall_thickness, car_depth, shell_height)),
        origin=Origin(xyz=(side_wall_x, 0.0, wall_center_z)),
        material=powder_coat,
        name="right_wall",
    )
    car.visual(
        Box((car_width, wall_thickness, shell_height)),
        origin=Origin(xyz=(0.0, rear_wall_y, wall_center_z)),
        material=powder_coat,
        name="rear_wall",
    )
    car.visual(
        Box((car_width, car_depth, roof_thickness)),
        origin=Origin(xyz=(0.0, 0.0, car_height - roof_thickness / 2.0)),
        material=powder_coat,
        name="car_roof",
    )

    front_frame_y = -car_depth / 2.0 + front_frame_depth / 2.0
    front_opening_z = floor_thickness + opening_height / 2.0
    front_transom_height = car_height - floor_thickness - roof_thickness - opening_height
    front_transom_z = floor_thickness + opening_height + front_transom_height / 2.0
    front_return_x = car_width / 2.0 - side_pocket_width / 2.0
    car.visual(
        Box((side_pocket_width, front_frame_depth, opening_height)),
        origin=Origin(xyz=(-front_return_x, front_frame_y, front_opening_z)),
        material=brushed_steel,
        name="left_front_return",
    )
    car.visual(
        Box((side_pocket_width, front_frame_depth, opening_height)),
        origin=Origin(xyz=(front_return_x, front_frame_y, front_opening_z)),
        material=brushed_steel,
        name="right_front_return",
    )
    car.visual(
        Box((opening_width, front_frame_depth, front_transom_height)),
        origin=Origin(xyz=(0.0, front_frame_y, front_transom_z)),
        material=brushed_steel,
        name="front_transom",
    )

    car.visual(
        Box((car_width, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -car_depth / 2.0 + 0.07, car_height - 0.12)),
        material=dark_trim,
        name="upper_door_track",
    )
    car.visual(
        Box((opening_width + 0.12, 0.06, 0.025)),
        origin=Origin(xyz=(0.0, -car_depth / 2.0 + 0.07, floor_thickness + 0.0125)),
        material=dark_trim,
        name="door_sill",
    )

    rear_stile_width = 0.06
    rear_stile_depth = 0.05
    rear_stile_x = rail_x
    rear_stile_y = rear_wall_y + 0.01
    rear_stile_height = car_height - 0.10
    rear_stile_z = 0.05 + rear_stile_height / 2.0
    car.visual(
        Box((rear_stile_width, rear_stile_depth, rear_stile_height)),
        origin=Origin(xyz=(-rear_stile_x, rear_stile_y, rear_stile_z)),
        material=dark_trim,
        name="left_carriage_stile",
    )
    car.visual(
        Box((rear_stile_width, rear_stile_depth, rear_stile_height)),
        origin=Origin(xyz=(rear_stile_x, rear_stile_y, rear_stile_z)),
        material=dark_trim,
        name="right_carriage_stile",
    )

    shoe_depth = 0.03
    shoe_height = 0.14
    shoe_y = rail_y - rail_depth / 2.0 - shoe_depth / 2.0
    car.visual(
        Box((0.07, shoe_depth, shoe_height)),
        origin=Origin(xyz=(-rear_stile_x, shoe_y, 0.24)),
        material=rail_metal,
        name="left_lower_guide_shoe",
    )
    car.visual(
        Box((0.07, shoe_depth, shoe_height)),
        origin=Origin(xyz=(-rear_stile_x, shoe_y, car_height - 0.26)),
        material=rail_metal,
        name="left_upper_guide_shoe",
    )
    car.visual(
        Box((0.07, shoe_depth, shoe_height)),
        origin=Origin(xyz=(rear_stile_x, shoe_y, 0.24)),
        material=rail_metal,
        name="right_lower_guide_shoe",
    )
    car.visual(
        Box((0.07, shoe_depth, shoe_height)),
        origin=Origin(xyz=(rear_stile_x, shoe_y, car_height - 0.26)),
        material=rail_metal,
        name="right_upper_guide_shoe",
    )

    car.inertial = Inertial.from_geometry(
        Box((car_width, car_depth, car_height)),
        mass=175.0,
        origin=Origin(xyz=(0.0, 0.0, car_height / 2.0)),
    )

    car_lift = model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft_frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.6, lower=0.0, upper=0.72),
    )

    def add_door_panel(part_name: str, handle_x: float) -> None:
        panel = model.part(part_name)
        panel.visual(
            Box((panel_width, panel_thickness, panel_height)),
            origin=Origin(xyz=(0.0, 0.0, panel_height / 2.0)),
            material=brushed_steel,
            name="door_leaf",
        )
        panel.visual(
            Box((panel_width - 0.08, 0.004, 0.78)),
            origin=Origin(xyz=(0.0, 0.0, 1.12)),
            material=smoked_glass,
            name="glass_lite",
        )
        panel.visual(
            Box((0.018, 0.014, 0.48)),
            origin=Origin(xyz=(handle_x, 0.0, 1.08)),
            material=dark_trim,
            name="pull_stile",
        )
        panel.inertial = Inertial.from_geometry(
            Box((panel_width, panel_thickness, panel_height)),
            mass=22.0,
            origin=Origin(xyz=(0.0, 0.0, panel_height / 2.0)),
        )

    add_door_panel("left_door_panel", handle_x=0.09)
    add_door_panel("right_door_panel", handle_x=-0.09)

    door_bottom_z = floor_thickness + 0.028
    door_y = front_frame_y + front_frame_depth / 2.0 + panel_thickness / 2.0
    left_door = model.get_part("left_door_panel")
    right_door = model.get_part("right_door_panel")
    left_slide = model.articulation(
        "car_to_left_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-panel_width / 2.0, door_y, door_bottom_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=panel_travel),
    )
    right_slide = model.articulation(
        "car_to_right_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(panel_width / 2.0, door_y, door_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=panel_travel),
    )

    model.meta["primary_joints"] = [car_lift.name, left_slide.name, right_slide.name]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft_frame = object_model.get_part("shaft_frame")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door_panel")
    right_door = object_model.get_part("right_door_panel")

    car_lift = object_model.get_articulation("shaft_to_car")
    left_slide = object_model.get_articulation("car_to_left_door")
    right_slide = object_model.get_articulation("car_to_right_door")
    door_clear_opening = 0.50

    with ctx.pose({car_lift: 0.0, left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="door_leaf",
            negative_elem="door_leaf",
            name="center-opening door panels meet cleanly at the center seam",
        )
        ctx.expect_gap(
            shaft_frame,
            car,
            axis="y",
            min_gap=0.0,
            max_gap=0.03,
            positive_elem="left_guide_rail",
            negative_elem="left_lower_guide_shoe",
            name="left lower guide shoe runs on the left guide rail",
        )
        ctx.expect_gap(
            shaft_frame,
            car,
            axis="y",
            min_gap=0.0,
            max_gap=0.03,
            positive_elem="right_guide_rail",
            negative_elem="right_lower_guide_shoe",
            name="right lower guide shoe runs on the right guide rail",
        )
        ctx.expect_overlap(
            shaft_frame,
            car,
            axes="z",
            min_overlap=0.12,
            elem_a="left_guide_rail",
            elem_b="left_upper_guide_shoe",
            name="upper left shoe remains vertically engaged on the rail",
        )
        ctx.expect_overlap(
            shaft_frame,
            car,
            axes="z",
            min_overlap=0.12,
            elem_a="right_guide_rail",
            elem_b="right_upper_guide_shoe",
            name="upper right shoe remains vertically engaged on the rail",
        )

    rest_car_pos = ctx.part_world_position(car)
    rest_left_pos = ctx.part_world_position(left_door)
    rest_right_pos = ctx.part_world_position(right_door)

    with ctx.pose({car_lift: 0.72}):
        raised_car_pos = ctx.part_world_position(car)
        ctx.expect_overlap(
            shaft_frame,
            car,
            axes="z",
            min_overlap=0.12,
            elem_a="left_guide_rail",
            elem_b="left_upper_guide_shoe",
            name="left upper guide shoe stays captured at the top landing pose",
        )
        ctx.expect_overlap(
            shaft_frame,
            car,
            axes="z",
            min_overlap=0.12,
            elem_a="right_guide_rail",
            elem_b="right_upper_guide_shoe",
            name="right upper guide shoe stays captured at the top landing pose",
        )

    ctx.check(
        "car lifts upward on the rail axis",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 0.65,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )

    with ctx.pose({left_slide: 0.26, right_slide: 0.26}):
        open_left_pos = ctx.part_world_position(left_door)
        open_right_pos = ctx.part_world_position(right_door)
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=door_clear_opening,
            positive_elem="door_leaf",
            negative_elem="door_leaf",
            name="bi-parting panels clear the central opening when opened",
        )

    ctx.check(
        "left panel slides to the left pocket",
        rest_left_pos is not None
        and open_left_pos is not None
        and open_left_pos[0] < rest_left_pos[0] - 0.22,
        details=f"rest={rest_left_pos}, open={open_left_pos}",
    )
    ctx.check(
        "right panel slides to the right pocket",
        rest_right_pos is not None
        and open_right_pos is not None
        and open_right_pos[0] > rest_right_pos[0] + 0.22,
        details=f"rest={rest_right_pos}, open={open_right_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
