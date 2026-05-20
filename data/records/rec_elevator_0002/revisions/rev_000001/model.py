from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traction_passenger_elevator")

    shaft_width = 2.10
    shaft_depth = 1.95
    shaft_height = 4.60
    wall_thickness = 0.08

    car_width = 1.44
    car_depth = 1.20
    car_height = 2.30
    car_wall = 0.04
    doorway_width = 0.94
    doorway_height = 2.00
    door_width = doorway_width / 2.0
    door_thickness = 0.04
    door_plane_y = 0.62
    car_rest_y = 0.28
    car_rest_z = 0.20
    car_travel = 1.80
    hanger_height = 0.03
    track_height = 0.03
    frame_depth = 0.10
    frame_front_y = 0.60
    frame_center_y = frame_front_y - (frame_depth / 2.0)

    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    shaft_gray = model.material("shaft_gray", rgba=(0.56, 0.58, 0.62, 1.0))
    cable_black = model.material("cable_black", rgba=(0.12, 0.12, 0.13, 1.0))
    counterweight_gray = model.material("counterweight_gray", rgba=(0.45, 0.46, 0.48, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Box((shaft_width, shaft_depth, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="pit_floor",
    )
    shaft.visual(
        Box((wall_thickness, shaft_depth, shaft_height)),
        origin=Origin(xyz=(-(shaft_width / 2.0) + wall_thickness / 2.0, 0.0, shaft_height / 2.0)),
        material=shaft_gray,
        name="left_wall",
    )
    shaft.visual(
        Box((wall_thickness, shaft_depth, shaft_height)),
        origin=Origin(xyz=((shaft_width / 2.0) - wall_thickness / 2.0, 0.0, shaft_height / 2.0)),
        material=shaft_gray,
        name="right_wall",
    )
    shaft.visual(
        Box((shaft_width, wall_thickness, shaft_height)),
        origin=Origin(xyz=(0.0, -(shaft_depth / 2.0) + wall_thickness / 2.0, shaft_height / 2.0)),
        material=shaft_gray,
        name="back_wall",
    )
    shaft.visual(
        Box((shaft_width, shaft_depth, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, shaft_height - 0.06)),
        material=dark_steel,
        name="top_slab",
    )
    shaft.visual(
        Box((shaft_width, 0.06, 0.20)),
        origin=Origin(xyz=(0.0, (shaft_depth / 2.0) - 0.03, shaft_height - 0.30)),
        material=dark_steel,
        name="front_header",
    )
    shaft.visual(
        Box((0.03, 0.08, shaft_height - 0.20)),
        origin=Origin(xyz=(-0.765, car_rest_y - 0.10, (shaft_height - 0.20) / 2.0 + 0.10)),
        material=dark_steel,
        name="left_rail",
    )
    shaft.visual(
        Box((0.03, 0.08, shaft_height - 0.20)),
        origin=Origin(xyz=(0.765, car_rest_y - 0.10, (shaft_height - 0.20) / 2.0 + 0.10)),
        material=dark_steel,
        name="right_rail",
    )
    shaft.visual(
        Box((0.96, 0.22, 0.16)),
        origin=Origin(xyz=(0.57, -0.60, shaft_height - 0.30)),
        material=dark_steel,
        name="machine_beam",
    )
    shaft.visual(
        Cylinder(radius=0.10, length=0.18),
        origin=Origin(xyz=(0.56, -0.60, shaft_height - 0.30), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drive_sheave",
    )
    for idx, rope_x in enumerate((0.40, 0.48, 0.56, 0.64), start=1):
        shaft.visual(
            Cylinder(radius=0.01, length=shaft_height - 0.44),
            origin=Origin(xyz=(rope_x, -0.60, (shaft_height - 0.44) / 2.0 + 0.18)),
            material=cable_black,
            name=f"rope_{idx}",
        )
    shaft.visual(
        Box((0.18, 0.26, 1.45)),
        origin=Origin(xyz=(0.91, -0.60, 1.95)),
        material=counterweight_gray,
        name="counterweight",
    )
    shaft.visual(
        Box((0.025, 0.08, shaft_height - 0.48)),
        origin=Origin(xyz=(0.84, -0.60, (shaft_height - 0.48) / 2.0 + 0.18)),
        material=dark_steel,
        name="counterweight_left_rail",
    )
    shaft.visual(
        Box((0.025, 0.08, shaft_height - 0.48)),
        origin=Origin(xyz=(0.98, -0.60, (shaft_height - 0.48) / 2.0 + 0.18)),
        material=dark_steel,
        name="counterweight_right_rail",
    )

    car = model.part("car")
    car.visual(
        Box((car_width, car_depth, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel,
        name="car_floor",
    )
    car.visual(
        Box((car_width, car_depth, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, car_height - 0.025)),
        material=steel,
        name="car_roof",
    )
    car.visual(
        Box((car_wall, car_depth, car_height)),
        origin=Origin(xyz=(-(car_width / 2.0) + car_wall / 2.0, 0.0, car_height / 2.0)),
        material=steel,
        name="left_side_wall",
    )
    car.visual(
        Box((car_wall, car_depth, car_height)),
        origin=Origin(xyz=((car_width / 2.0) - car_wall / 2.0, 0.0, car_height / 2.0)),
        material=steel,
        name="right_side_wall",
    )
    car.visual(
        Box((car_width - 2.0 * car_wall, car_wall, car_height)),
        origin=Origin(xyz=(0.0, -(car_depth / 2.0) + car_wall / 2.0, car_height / 2.0)),
        material=steel,
        name="back_wall",
    )
    car.visual(
        Box(((car_width - doorway_width) / 2.0, frame_depth, doorway_height + 0.16)),
        origin=Origin(
            xyz=(
                -(doorway_width / 2.0) - (car_width - doorway_width) / 4.0,
                frame_center_y,
                (doorway_height + 0.16) / 2.0,
            )
        ),
        material=steel,
        name="left_jamb",
    )
    car.visual(
        Box(((car_width - doorway_width) / 2.0, frame_depth, doorway_height + 0.16)),
        origin=Origin(
            xyz=(
                (doorway_width / 2.0) + (car_width - doorway_width) / 4.0,
                frame_center_y,
                (doorway_height + 0.16) / 2.0,
            )
        ),
        material=steel,
        name="right_jamb",
    )
    car.visual(
        Box((doorway_width, frame_depth, 0.08)),
        origin=Origin(xyz=(0.0, frame_center_y, 2.20)),
        material=steel,
        name="door_header",
    )
    car.visual(
        Box((doorway_width + 0.14, frame_depth, 0.08)),
        origin=Origin(xyz=(0.0, frame_center_y, 0.04)),
        material=dark_steel,
        name="threshold",
    )
    car.visual(
        Box((doorway_width + 0.14, frame_depth, track_height)),
        origin=Origin(xyz=(0.0, frame_center_y, 2.125)),
        material=dark_steel,
        name="top_track",
    )
    car.visual(
        Box((0.03, 0.12, 0.08)),
        origin=Origin(xyz=(-0.735, -0.10, 0.45)),
        material=dark_steel,
        name="left_lower_shoe",
    )
    car.visual(
        Box((0.03, 0.12, 0.08)),
        origin=Origin(xyz=(-0.735, -0.10, 1.90)),
        material=dark_steel,
        name="left_upper_shoe",
    )
    car.visual(
        Box((0.03, 0.12, 0.08)),
        origin=Origin(xyz=(0.735, -0.10, 0.45)),
        material=dark_steel,
        name="right_lower_shoe",
    )
    car.visual(
        Box((0.03, 0.12, 0.08)),
        origin=Origin(xyz=(0.735, -0.10, 1.90)),
        material=dark_steel,
        name="right_upper_shoe",
    )
    car.visual(
        Box((car_width - 0.12, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, -(car_depth / 2.0) + car_wall + 0.015, 0.96)),
        material=dark_steel,
        name="rear_handrail",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, doorway_height)),
        material=steel,
        name="panel",
    )
    left_door.visual(
        Box((0.08, 0.04, hanger_height)),
        origin=Origin(xyz=(0.16, 0.0, (doorway_height / 2.0) + (hanger_height / 2.0))),
        material=dark_steel,
        name="hanger",
    )
    left_door.visual(
        Box((0.04, 0.04, 0.015)),
        origin=Origin(xyz=(0.16, 0.0, -(doorway_height / 2.0) + 0.0075)),
        material=dark_steel,
        name="guide",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, doorway_height)),
        material=steel,
        name="panel",
    )
    right_door.visual(
        Box((0.08, 0.04, hanger_height)),
        origin=Origin(xyz=(-0.16, 0.0, (doorway_height / 2.0) + (hanger_height / 2.0))),
        material=dark_steel,
        name="hanger",
    )
    right_door.visual(
        Box((0.04, 0.04, 0.015)),
        origin=Origin(xyz=(-0.16, 0.0, -(doorway_height / 2.0) + 0.0075)),
        material=dark_steel,
        name="guide",
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, car_rest_y, car_rest_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=1.2,
            lower=0.0,
            upper=car_travel,
        ),
    )
    model.articulation(
        "car_to_left_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-door_width / 2.0, door_plane_y, 1.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.6,
            lower=-0.23,
            upper=0.0,
        ),
    )
    model.articulation(
        "car_to_right_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(door_width / 2.0, door_plane_y, 1.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.6,
            lower=0.0,
            upper=0.23,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    car_lift = object_model.get_articulation("shaft_to_car")
    left_slide = object_model.get_articulation("car_to_left_door")
    right_slide = object_model.get_articulation("car_to_right_door")
    car_upper_travel = (
        car_lift.motion_limits.upper
        if car_lift.motion_limits is not None and car_lift.motion_limits.upper is not None
        else 1.80
    )

    pit_floor = shaft.get_visual("pit_floor")
    top_slab = shaft.get_visual("top_slab")
    left_rail = shaft.get_visual("left_rail")
    right_rail = shaft.get_visual("right_rail")
    car_floor = car.get_visual("car_floor")
    car_roof = car.get_visual("car_roof")
    threshold = car.get_visual("threshold")
    top_track = car.get_visual("top_track")
    left_jamb = car.get_visual("left_jamb")
    right_jamb = car.get_visual("right_jamb")
    left_lower_shoe = car.get_visual("left_lower_shoe")
    left_upper_shoe = car.get_visual("left_upper_shoe")
    right_lower_shoe = car.get_visual("right_lower_shoe")
    right_upper_shoe = car.get_visual("right_upper_shoe")
    left_hanger = left_door.get_visual("hanger")
    right_hanger = right_door.get_visual("hanger")
    left_guide = left_door.get_visual("guide")
    right_guide = right_door.get_visual("guide")
    left_panel = left_door.get_visual("panel")
    right_panel = right_door.get_visual("panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=16, contact_tol=1e-5)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    with ctx.pose({car_lift: 0.0, left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_within(car, shaft, axes="xy", name="car_stays_inside_shaft_plan")
        ctx.expect_origin_distance(car, shaft, axes="x", max_dist=0.02, name="car_centered_in_shaft")
        ctx.expect_gap(
            car,
            shaft,
            axis="z",
            min_gap=0.09,
            max_gap=0.12,
            positive_elem=car_floor,
            negative_elem=pit_floor,
            name="car_floor_above_pit_floor",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=left_lower_shoe,
            elem_b=left_rail,
            contact_tol=1e-5,
            name="left_lower_guide_shoe_contacts_left_rail",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=left_upper_shoe,
            elem_b=left_rail,
            contact_tol=1e-5,
            name="left_upper_guide_shoe_contacts_left_rail",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=right_lower_shoe,
            elem_b=right_rail,
            contact_tol=1e-5,
            name="right_lower_guide_shoe_contacts_right_rail",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=right_upper_shoe,
            elem_b=right_rail,
            contact_tol=1e-5,
            name="right_upper_guide_shoe_contacts_right_rail",
        )
        ctx.expect_contact(
            left_door,
            car,
            elem_a=left_hanger,
            elem_b=top_track,
            contact_tol=1e-5,
            name="left_hanger_supported_by_top_track",
        )
        ctx.expect_contact(
            right_door,
            car,
            elem_a=right_hanger,
            elem_b=top_track,
            contact_tol=1e-5,
            name="right_hanger_supported_by_top_track",
        )
        ctx.expect_contact(
            left_door,
            car,
            elem_a=left_guide,
            elem_b=threshold,
            contact_tol=1e-5,
            name="left_bottom_guide_supported_by_threshold",
        )
        ctx.expect_contact(
            right_door,
            car,
            elem_a=right_guide,
            elem_b=threshold,
            contact_tol=1e-5,
            name="right_bottom_guide_supported_by_threshold",
        )
        ctx.expect_contact(
            left_door,
            car,
            elem_a=left_panel,
            elem_b=left_jamb,
            contact_tol=1e-5,
            name="left_panel_closes_against_left_jamb",
        )
        ctx.expect_contact(
            right_door,
            car,
            elem_a=right_panel,
            elem_b=right_jamb,
            contact_tol=1e-5,
            name="right_panel_closes_against_right_jamb",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem=right_panel,
            negative_elem=left_panel,
            name="door_panels_meet_at_center_when_closed",
        )

    with ctx.pose({left_slide: -0.23, right_slide: 0.23}):
        ctx.fail_if_parts_overlap_in_current_pose(name="doors_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=1e-5, name="doors_open_pose_no_floating")
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.44,
            positive_elem=right_panel,
            negative_elem=left_panel,
            name="door_panels_clear_a_wide_center_opening",
        )
        ctx.expect_contact(
            left_door,
            car,
            elem_a=left_hanger,
            elem_b=top_track,
            contact_tol=1e-5,
            name="left_hanger_stays_in_track_when_open",
        )
        ctx.expect_contact(
            right_door,
            car,
            elem_a=right_hanger,
            elem_b=top_track,
            contact_tol=1e-5,
            name="right_hanger_stays_in_track_when_open",
        )
        ctx.expect_contact(
            left_door,
            car,
            elem_a=left_guide,
            elem_b=threshold,
            contact_tol=1e-5,
            name="left_guide_stays_on_threshold_when_open",
        )
        ctx.expect_contact(
            right_door,
            car,
            elem_a=right_guide,
            elem_b=threshold,
            contact_tol=1e-5,
            name="right_guide_stays_on_threshold_when_open",
        )
        ctx.expect_overlap(
            left_door,
            car,
            axes="xz",
            elem_a=left_panel,
            elem_b=left_jamb,
            min_overlap=0.22,
            name="left_panel_retracts_behind_left_jamb",
        )
        ctx.expect_overlap(
            right_door,
            car,
            axes="xz",
            elem_a=right_panel,
            elem_b=right_jamb,
            min_overlap=0.22,
            name="right_panel_retracts_behind_right_jamb",
        )

    with ctx.pose({car_lift: car_upper_travel}):
        ctx.fail_if_parts_overlap_in_current_pose(name="car_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=1e-5, name="car_upper_pose_no_floating")
        ctx.expect_within(car, shaft, axes="xy", name="car_stays_inside_shaft_plan_at_top")
        ctx.expect_contact(
            car,
            shaft,
            elem_a=left_upper_shoe,
            elem_b=left_rail,
            contact_tol=1e-5,
            name="left_upper_shoe_stays_on_rail_at_top",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=right_upper_shoe,
            elem_b=right_rail,
            contact_tol=1e-5,
            name="right_upper_shoe_stays_on_rail_at_top",
        )
        ctx.expect_gap(
            shaft,
            car,
            axis="z",
            min_gap=0.16,
            max_gap=0.22,
            positive_elem=top_slab,
            negative_elem=car_roof,
            name="car_roof_stays_below_top_slab_at_upper_landing",
        )

    for articulation, label in (
        (car_lift, "shaft_to_car"),
        (left_slide, "car_to_left_door"),
        (right_slide, "car_to_right_door"),
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=1e-5, name=f"{label}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
                ctx.fail_if_isolated_parts(contact_tol=1e-5, name=f"{label}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
