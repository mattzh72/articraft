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
    model = ArticulatedObject(name="hospital_bed_elevator")

    shaft_paint = model.material("shaft_paint", rgba=(0.42, 0.45, 0.48, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    stainless = model.material("stainless", rgba=(0.83, 0.85, 0.87, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    glass = model.material("glass", rgba=(0.35, 0.49, 0.56, 0.45))
    floor_vinyl = model.material("floor_vinyl", rgba=(0.24, 0.27, 0.31, 1.0))

    shaft_width = 4.40
    shaft_depth = 4.40
    shaft_height = 4.45
    column_size = 0.16
    beam_depth = 0.18

    car_width = 3.75
    car_depth = 3.75
    car_height = 2.65
    wall_thickness = 0.08
    floor_thickness = 0.12
    roof_thickness = 0.08
    opening_width = 1.80
    door_panel_width = 0.94
    door_panel_thickness = 0.045
    door_panel_height = 2.00
    door_closed_center_x = door_panel_width / 2.0
    door_travel = 0.86
    front_frame_y = -car_depth / 2.0 + 0.035
    door_y = -car_depth / 2.0 + 0.115
    door_center_z = 0.12 + door_panel_height / 2.0
    lift_travel = 1.40

    inner_half_width = car_width / 2.0 - wall_thickness

    shaft = model.part("shaft")
    shaft.visual(
        Box((shaft_width, shaft_depth, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=shaft_paint,
        name="base_slab",
    )

    column_xy = shaft_width / 2.0 - column_size / 2.0
    column_z = 0.18 + shaft_height / 2.0
    for name, x, y in (
        ("front_left_column", -column_xy, -column_xy),
        ("front_right_column", column_xy, -column_xy),
        ("rear_left_column", -column_xy, column_xy),
        ("rear_right_column", column_xy, column_xy),
    ):
        shaft.visual(
            Box((column_size, column_size, shaft_height)),
            origin=Origin(xyz=(x, y, column_z)),
            material=shaft_paint,
            name=name,
        )

    top_z = 0.18 + shaft_height + beam_depth / 2.0
    shaft.visual(
        Box((shaft_width, beam_depth, beam_depth)),
        origin=Origin(xyz=(0.0, -column_xy, top_z)),
        material=shaft_paint,
        name="front_top_beam",
    )
    shaft.visual(
        Box((shaft_width, beam_depth, beam_depth)),
        origin=Origin(xyz=(0.0, column_xy, top_z)),
        material=shaft_paint,
        name="rear_top_beam",
    )
    shaft.visual(
        Box((beam_depth, shaft_depth - 2.0 * column_size, beam_depth)),
        origin=Origin(xyz=(-column_xy, 0.0, top_z)),
        material=shaft_paint,
        name="left_top_beam",
    )
    shaft.visual(
        Box((beam_depth, shaft_depth - 2.0 * column_size, beam_depth)),
        origin=Origin(xyz=(column_xy, 0.0, top_z)),
        material=shaft_paint,
        name="right_top_beam",
    )

    rail_height = shaft_height - 0.08
    rail_center_z = 0.18 + rail_height / 2.0
    rail_y = car_depth / 2.0 + 0.11
    rail_x = 1.45
    for name, x in (("left_guide_rail", -rail_x), ("right_guide_rail", rail_x)):
        shaft.visual(
            Box((0.08, 0.12, rail_height)),
            origin=Origin(xyz=(x, rail_y, rail_center_z)),
            material=dark_trim,
            name=name,
        )

    shaft.inertial = Inertial.from_geometry(
        Box((shaft_width, shaft_depth, shaft_height + 0.36)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, (shaft_height + 0.36) / 2.0)),
    )

    car = model.part("car")
    car.visual(
        Box((car_width, car_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=floor_vinyl,
        name="floor",
    )
    car.visual(
        Box((car_width, car_depth, roof_thickness)),
        origin=Origin(xyz=(0.0, 0.0, car_height - roof_thickness / 2.0)),
        material=stainless,
        name="roof",
    )
    side_wall_height = car_height - floor_thickness - roof_thickness
    wall_center_z = floor_thickness + side_wall_height / 2.0
    car.visual(
        Box((wall_thickness, car_depth, side_wall_height)),
        origin=Origin(xyz=(-car_width / 2.0 + wall_thickness / 2.0, 0.0, wall_center_z)),
        material=stainless,
        name="left_wall",
    )
    car.visual(
        Box((wall_thickness, car_depth, side_wall_height)),
        origin=Origin(xyz=(car_width / 2.0 - wall_thickness / 2.0, 0.0, wall_center_z)),
        material=stainless,
        name="right_wall",
    )
    car.visual(
        Box((car_width - 2.0 * wall_thickness, wall_thickness, side_wall_height)),
        origin=Origin(xyz=(0.0, car_depth / 2.0 - wall_thickness / 2.0, wall_center_z)),
        material=stainless,
        name="back_wall",
    )

    return_width = inner_half_width - opening_width / 2.0
    return_center_x = opening_width / 2.0 + return_width / 2.0
    front_return_height = 2.25
    front_return_z = floor_thickness + front_return_height / 2.0
    car.visual(
        Box((return_width, 0.07, front_return_height)),
        origin=Origin(xyz=(-return_center_x, front_frame_y, front_return_z)),
        material=stainless,
        name="left_front_return",
    )
    car.visual(
        Box((return_width, 0.07, front_return_height)),
        origin=Origin(xyz=(return_center_x, front_frame_y, front_return_z)),
        material=stainless,
        name="right_front_return",
    )
    car.visual(
        Box((opening_width, 0.07, 0.34)),
        origin=Origin(xyz=(0.0, front_frame_y, 2.38)),
        material=stainless,
        name="front_header",
    )
    car.visual(
        Box((opening_width, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, -car_depth / 2.0 + 0.06, 0.05)),
        material=dark_trim,
        name="threshold",
    )
    car.visual(
        Box((opening_width + 0.12, 0.12, 0.04)),
        origin=Origin(xyz=(0.0, door_y, 2.23)),
        material=dark_trim,
        name="door_track",
    )

    rail_mount_y = 0.15
    rail_mount_z = 0.95
    car.visual(
        Box((0.01, 0.16, 0.20)),
        origin=Origin(xyz=(-inner_half_width - 0.005, rail_mount_y, rail_mount_z)),
        material=dark_trim,
        name="left_rail_mount_pad",
    )
    car.visual(
        Box((0.01, 0.16, 0.20)),
        origin=Origin(xyz=(inner_half_width + 0.005, rail_mount_y, rail_mount_z)),
        material=dark_trim,
        name="right_rail_mount_pad",
    )

    for name, x, z in (
        ("left_lower_guide_shoe", -rail_x, 0.55),
        ("left_upper_guide_shoe", -rail_x, 2.10),
        ("right_lower_guide_shoe", rail_x, 0.55),
        ("right_upper_guide_shoe", rail_x, 2.10),
    ):
        car.visual(
            Box((0.16, 0.06, 0.18)),
            origin=Origin(xyz=(x, car_depth / 2.0 + 0.02, z)),
            material=dark_trim,
            name=name,
        )

    car.inertial = Inertial.from_geometry(
        Box((car_width, car_depth, car_height)),
        mass=1300.0,
        origin=Origin(xyz=(0.0, 0.0, car_height / 2.0)),
    )

    def add_door(panel_name: str, handedness: str, closed_center_x: float):
        door = model.part(panel_name)
        panel = door.visual(
            Box((door_panel_width, door_panel_thickness, door_panel_height)),
            material=steel,
            name="panel",
        )
        door.visual(
            Box((door_panel_width - 0.12, 0.010, door_panel_height - 0.20)),
            origin=Origin(xyz=(0.0, door_panel_thickness / 2.0 - 0.002, 0.0)),
            material=stainless,
            name="face_inset",
        )
        door.visual(
            Box((0.16, 0.008, 0.60)),
            origin=Origin(xyz=(0.0, door_panel_thickness / 2.0 - 0.001, 0.18)),
            material=glass,
            name="vision_panel",
        )
        door.visual(
            Box((0.28, 0.035, 0.06)),
            origin=Origin(xyz=(0.0, 0.0, door_panel_height / 2.0 + 0.03)),
            material=dark_trim,
            name="hanger",
        )
        door.inertial = Inertial.from_geometry(
            Box((door_panel_width, door_panel_thickness, door_panel_height + 0.08)),
            mass=140.0,
            origin=Origin(xyz=(0.0, 0.0, 0.03)),
        )

        if handedness == "left":
            axis = (-1.0, 0.0, 0.0)
        else:
            axis = (1.0, 0.0, 0.0)

        model.articulation(
            f"car_to_{panel_name}",
            ArticulationType.PRISMATIC,
            parent=car,
            child=door,
            origin=Origin(xyz=(closed_center_x, door_y, door_center_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=350.0,
                velocity=0.45,
                lower=0.0,
                upper=door_travel,
            ),
        )

        return panel_name

    add_door("left_door", "left", -door_closed_center_x)
    add_door("right_door", "right", door_closed_center_x)

    left_rail = model.part("left_grab_rail")
    left_rail.visual(
        Box((0.025, 0.14, 0.18)),
        origin=Origin(xyz=(0.0125, 0.0, 0.0)),
        material=dark_trim,
        name="mount_plate",
    )
    left_rail.visual(
        Cylinder(radius=0.03, length=0.18),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    left_rail.visual(
        Cylinder(radius=0.022, length=0.55),
        origin=Origin(xyz=(0.275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="grab_bar",
    )
    left_rail.inertial = Inertial.from_geometry(
        Box((0.58, 0.16, 0.20)),
        mass=12.0,
        origin=Origin(xyz=(0.29, 0.0, 0.0)),
    )

    right_rail = model.part("right_grab_rail")
    right_rail.visual(
        Box((0.025, 0.14, 0.18)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.0)),
        material=dark_trim,
        name="mount_plate",
    )
    right_rail.visual(
        Cylinder(radius=0.03, length=0.18),
        origin=Origin(xyz=(-0.03, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    right_rail.visual(
        Cylinder(radius=0.022, length=0.55),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="grab_bar",
    )
    right_rail.inertial = Inertial.from_geometry(
        Box((0.58, 0.16, 0.20)),
        mass=12.0,
        origin=Origin(xyz=(-0.29, 0.0, 0.0)),
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=0.6,
            lower=0.0,
            upper=lift_travel,
        ),
    )
    model.articulation(
        "car_to_left_grab_rail",
        ArticulationType.REVOLUTE,
        parent=car,
        child=left_rail,
        origin=Origin(xyz=(-inner_half_width, rail_mount_y, rail_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "car_to_right_grab_rail",
        ArticulationType.REVOLUTE,
        parent=car,
        child=right_rail,
        origin=Origin(xyz=(inner_half_width, rail_mount_y, rail_mount_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_rail = object_model.get_part("left_grab_rail")
    right_rail = object_model.get_part("right_grab_rail")

    lift_joint = object_model.get_articulation("shaft_to_car")
    left_door_joint = object_model.get_articulation("car_to_left_door")
    right_door_joint = object_model.get_articulation("car_to_right_door")
    left_rail_joint = object_model.get_articulation("car_to_left_grab_rail")
    right_rail_joint = object_model.get_articulation("car_to_right_grab_rail")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_gap(
        shaft,
        car,
        axis="y",
        positive_elem="left_guide_rail",
        negative_elem="left_lower_guide_shoe",
        min_gap=0.0,
        max_gap=0.01,
        name="car left guide shoe runs close to left shaft rail",
    )
    ctx.expect_gap(
        shaft,
        car,
        axis="y",
        positive_elem="right_guide_rail",
        negative_elem="right_lower_guide_shoe",
        min_gap=0.0,
        max_gap=0.01,
        name="car right guide shoe runs close to right shaft rail",
    )
    ctx.expect_contact(
        left_rail,
        car,
        elem_a="mount_plate",
        elem_b="left_rail_mount_pad",
        name="left grab rail mount plate seats on left wall pad",
    )
    ctx.expect_contact(
        right_rail,
        car,
        elem_a="mount_plate",
        elem_b="right_rail_mount_pad",
        name="right grab rail mount plate seats on right wall pad",
    )

    with ctx.pose({left_door_joint: 0.0, right_door_joint: 0.0}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            positive_elem="panel",
            negative_elem="panel",
            min_gap=0.0,
            max_gap=0.002,
            name="closed bi-parting doors meet at the center seam",
        )
        ctx.expect_gap(
            car,
            left_door,
            axis="z",
            positive_elem="door_track",
            negative_elem="hanger",
            min_gap=0.0,
            max_gap=0.035,
            name="left door hanger stays under the car track",
        )
        ctx.expect_gap(
            car,
            right_door,
            axis="z",
            positive_elem="door_track",
            negative_elem="hanger",
            min_gap=0.0,
            max_gap=0.035,
            name="right door hanger stays under the car track",
        )

    car_rest = ctx.part_world_position(car)
    left_door_rest = ctx.part_world_position(left_door)
    right_door_rest = ctx.part_world_position(right_door)
    left_bar_rest = ctx.part_element_world_aabb(left_rail, elem="grab_bar")
    right_bar_rest = ctx.part_element_world_aabb(right_rail, elem="grab_bar")

    with ctx.pose({lift_joint: lift_joint.motion_limits.upper}):
        car_raised = ctx.part_world_position(car)
        ctx.check(
            "car rises on the vertical prismatic lift",
            car_rest is not None
            and car_raised is not None
            and car_raised[2] > car_rest[2] + 1.0,
            details=f"rest={car_rest}, raised={car_raised}",
        )
        ctx.expect_gap(
            shaft,
            car,
            axis="y",
            positive_elem="left_guide_rail",
            negative_elem="left_upper_guide_shoe",
            min_gap=0.0,
            max_gap=0.01,
            name="upper left guide shoe stays aligned with left shaft rail at raised pose",
        )

    with ctx.pose({left_door_joint: left_door_joint.motion_limits.upper, right_door_joint: right_door_joint.motion_limits.upper}):
        left_door_open = ctx.part_world_position(left_door)
        right_door_open = ctx.part_world_position(right_door)
        ctx.check(
            "left door slides toward the left pocket",
            left_door_rest is not None
            and left_door_open is not None
            and left_door_open[0] < left_door_rest[0] - 0.70,
            details=f"rest={left_door_rest}, open={left_door_open}",
        )
        ctx.check(
            "right door slides toward the right pocket",
            right_door_rest is not None
            and right_door_open is not None
            and right_door_open[0] > right_door_rest[0] + 0.70,
            details=f"rest={right_door_rest}, open={right_door_open}",
        )
        ctx.expect_origin_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=2.4,
            name="opened door panel centers separate widely enough for bed access",
        )

    with ctx.pose({left_rail_joint: left_rail_joint.motion_limits.upper, right_rail_joint: right_rail_joint.motion_limits.upper}):
        left_bar_folded = ctx.part_element_world_aabb(left_rail, elem="grab_bar")
        right_bar_folded = ctx.part_element_world_aabb(right_rail, elem="grab_bar")
        ctx.check(
            "left grab rail folds toward the rear wall",
            left_bar_rest is not None
            and left_bar_folded is not None
            and left_bar_folded[1][1] > left_bar_rest[1][1] + 0.25,
            details=f"rest={left_bar_rest}, folded={left_bar_folded}",
        )
        ctx.check(
            "right grab rail folds toward the rear wall",
            right_bar_rest is not None
            and right_bar_folded is not None
            and right_bar_folded[1][1] > right_bar_rest[1][1] + 0.25,
            details=f"rest={right_bar_rest}, folded={right_bar_folded}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
