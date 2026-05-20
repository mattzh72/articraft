from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

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
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_freight_elevator", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.62, 0.62, 0.60, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    guide_steel = model.material("guide_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    platform_steel = model.material("platform_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.74, 0.75, 0.77, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.76, 0.19, 1.0))
    hydraulic_black = model.material("hydraulic_black", rgba=(0.12, 0.12, 0.14, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.82, 0.84, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Box((0.70, 2.10, 0.10)),
        origin=Origin(xyz=(-1.20, 0.00, -0.05)),
        material=concrete,
        name="left_floor_pad",
    )
    shaft.visual(
        Box((0.70, 2.10, 0.10)),
        origin=Origin(xyz=(1.20, 0.00, -0.05)),
        material=concrete,
        name="right_floor_pad",
    )
    shaft.visual(
        Box((1.70, 0.48, 0.10)),
        origin=Origin(xyz=(0.00, -0.81, -0.05)),
        material=concrete,
        name="rear_floor_pad",
    )
    shaft.visual(
        Box((3.10, 0.38, 0.10)),
        origin=Origin(xyz=(0.00, 0.86, -0.05)),
        material=concrete,
        name="front_landing_pad",
    )
    shaft.visual(
        Box((1.42, 0.20, 0.001)),
        origin=Origin(xyz=(0.00, 0.61, 0.0005)),
        material=panel_gray,
        name="landing_plate",
    )
    shaft.visual(
        Box((2.10, 0.08, 3.20)),
        origin=Origin(xyz=(0.00, -0.89, 1.60)),
        material=frame_steel,
        name="rear_wall",
    )
    shaft.visual(
        Box((0.08, 1.48, 3.20)),
        origin=Origin(xyz=(-0.97, -0.19, 1.60)),
        material=frame_steel,
        name="left_hoistway_wall",
    )
    shaft.visual(
        Box((0.08, 1.48, 3.20)),
        origin=Origin(xyz=(0.97, -0.19, 1.60)),
        material=frame_steel,
        name="right_hoistway_wall",
    )
    shaft.visual(
        Box((1.54, 0.10, 0.20)),
        origin=Origin(xyz=(0.00, 0.73, 1.84)),
        material=frame_steel,
        name="landing_header",
    )
    shaft.visual(
        Box((0.10, 0.10, 1.78)),
        origin=Origin(xyz=(-0.72, 0.73, 0.89)),
        material=frame_steel,
        name="left_jamb",
    )
    shaft.visual(
        Box((0.10, 0.10, 1.78)),
        origin=Origin(xyz=(0.72, 0.73, 0.89)),
        material=frame_steel,
        name="right_jamb",
    )
    shaft.visual(
        Box((2.49, 0.08, 0.06)),
        origin=Origin(xyz=(0.745, 0.64, 1.75)),
        material=frame_steel,
        name="door_track",
    )
    shaft.visual(
        Box((1.34, 0.04, 1.72)),
        origin=Origin(xyz=(1.32, 0.56, 0.86)),
        material=panel_gray,
        name="door_pocket_backer",
    )
    shaft.visual(
        Box((0.10, 0.10, 1.78)),
        origin=Origin(xyz=(2.04, 0.73, 0.89)),
        material=frame_steel,
        name="pocket_end_post",
    )
    shaft.visual(
        Box((0.02, 0.06, 1.70)),
        origin=Origin(xyz=(-0.66, 0.66, 0.85)),
        material=safety_yellow,
        name="door_stop",
    )
    shaft.visual(
        Box((0.04, 0.18, 2.90)),
        origin=Origin(xyz=(-0.91, -0.03, 1.45)),
        material=guide_steel,
        name="left_guide_rail",
    )
    shaft.visual(
        Box((0.04, 0.18, 2.90)),
        origin=Origin(xyz=(0.91, -0.03, 1.45)),
        material=guide_steel,
        name="right_guide_rail",
    )
    shaft.visual(
        Cylinder(radius=0.14, length=0.66),
        origin=Origin(xyz=(1.15, -0.35, -0.33)),
        material=hydraulic_black,
        name="hydraulic_cylinder",
    )
    shaft.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(1.15, -0.35, 0.04)),
        material=chrome,
        name="ram_gland",
    )
    shaft.visual(
        Box((0.32, 0.22, 0.26)),
        origin=Origin(xyz=(1.08, -0.72, 0.13)),
        material=hydraulic_black,
        name="power_unit",
    )
    shaft.visual(
        Box((0.06, 0.58, 0.06)),
        origin=Origin(xyz=(1.12, -0.54, 0.03)),
        material=hydraulic_black,
        name="hydraulic_hose_trunk",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((3.10, 2.10, 3.30)),
        mass=240.0,
        origin=Origin(xyz=(0.00, 0.00, 1.55)),
    )

    car = model.part("car")
    car.visual(
        Box((1.66, 1.08, 0.09)),
        origin=Origin(xyz=(0.00, 0.00, -0.045)),
        material=platform_steel,
        name="platform_base",
    )
    car.visual(
        Box((1.62, 1.04, 0.001)),
        origin=Origin(xyz=(0.00, 0.00, 0.0005)),
        material=panel_gray,
        name="deck_surface",
    )
    car.visual(
        Box((1.56, 0.05, 1.62)),
        origin=Origin(xyz=(0.00, -0.515, 0.81)),
        material=platform_steel,
        name="back_wall",
    )
    car.visual(
        Box((0.04, 0.96, 1.10)),
        origin=Origin(xyz=(-0.81, -0.03, 0.55)),
        material=platform_steel,
        name="left_side_guard",
    )
    car.visual(
        Box((0.04, 0.96, 1.10)),
        origin=Origin(xyz=(0.81, -0.03, 0.55)),
        material=platform_steel,
        name="right_side_guard",
    )
    car.visual(
        Box((1.66, 0.06, 0.12)),
        origin=Origin(xyz=(0.00, 0.48, 0.06)),
        material=safety_yellow,
        name="toe_guard",
    )
    car.visual(
        Box((0.06, 0.14, 0.96)),
        origin=Origin(xyz=(-0.86, -0.03, 0.48)),
        material=guide_steel,
        name="left_guide_runner",
    )
    car.visual(
        Box((0.06, 0.14, 0.96)),
        origin=Origin(xyz=(0.86, -0.03, 0.48)),
        material=guide_steel,
        name="right_guide_runner",
    )
    car.visual(
        Box((0.64, 0.34, 0.10)),
        origin=Origin(xyz=(0.00, -0.18, -0.14)),
        material=frame_steel,
        name="underframe",
    )
    car.inertial = Inertial.from_geometry(
        Box((1.66, 1.08, 1.72)),
        mass=62.0,
        origin=Origin(xyz=(0.00, 0.00, 0.81)),
    )

    landing_door = model.part("landing_door")
    landing_door.visual(
        Box((1.30, 0.04, 1.64)),
        origin=Origin(xyz=(0.00, 0.00, 0.82)),
        material=panel_gray,
        name="door_panel",
    )
    landing_door.visual(
        Box((1.30, 0.02, 0.06)),
        origin=Origin(xyz=(0.00, -0.01, 1.67)),
        material=frame_steel,
        name="hanger_bar",
    )
    landing_door.visual(
        Box((0.16, 0.06, 0.02)),
        origin=Origin(xyz=(-0.41, 0.00, 1.71)),
        material=guide_steel,
        name="left_hanger_shoe",
    )
    landing_door.visual(
        Box((0.16, 0.06, 0.02)),
        origin=Origin(xyz=(0.41, 0.00, 1.71)),
        material=guide_steel,
        name="right_hanger_shoe",
    )
    landing_door.inertial = Inertial.from_geometry(
        Box((1.30, 0.06, 1.72)),
        mass=28.0,
        origin=Origin(xyz=(0.00, 0.00, 0.86)),
    )

    model.articulation(
        "car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.00, -0.03, 0.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.35,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "landing_door_slide",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=landing_door,
        origin=Origin(xyz=(0.00, 0.64, 0.00)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.50,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    landing_door = object_model.get_part("landing_door")

    car_lift = object_model.get_articulation("car_lift")
    landing_door_slide = object_model.get_articulation("landing_door_slide")

    landing_plate = shaft.get_visual("landing_plate")
    rear_wall = shaft.get_visual("rear_wall")
    left_rail = shaft.get_visual("left_guide_rail")
    right_rail = shaft.get_visual("right_guide_rail")
    door_track = shaft.get_visual("door_track")
    door_stop = shaft.get_visual("door_stop")
    pocket_backer = shaft.get_visual("door_pocket_backer")

    deck_surface = car.get_visual("deck_surface")
    back_wall = car.get_visual("back_wall")
    toe_guard = car.get_visual("toe_guard")
    left_runner = car.get_visual("left_guide_runner")
    right_runner = car.get_visual("right_guide_runner")

    door_panel = landing_door.get_visual("door_panel")
    left_shoe = landing_door.get_visual("left_hanger_shoe")
    right_shoe = landing_door.get_visual("right_hanger_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    car_limits = car_lift.motion_limits
    door_limits = landing_door_slide.motion_limits

    ctx.check(
        "car_lift_vertical_axis",
        tuple(car_lift.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical prismatic axis, got {car_lift.axis!r}",
    )
    ctx.check(
        "landing_door_horizontal_axis",
        tuple(landing_door_slide.axis) == (1.0, 0.0, 0.0),
        details=f"Expected horizontal sliding axis, got {landing_door_slide.axis!r}",
    )
    ctx.check(
        "car_lift_realistic_travel",
        car_limits is not None
        and car_limits.lower == 0.0
        and car_limits.upper is not None
        and 1.20 <= car_limits.upper <= 1.50,
        details=f"Expected realistic freight-lift travel, got {car_limits!r}",
    )
    ctx.check(
        "landing_door_travel_matches_single_panel",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.20 <= door_limits.upper <= 1.35,
        details=f"Expected a single-panel side-slide stroke, got {door_limits!r}",
    )

    ctx.expect_gap(
        car,
        shaft,
        axis="z",
        positive_elem=deck_surface,
        negative_elem=landing_plate,
        max_gap=0.001,
        max_penetration=0.0011,
        name="car_floor_flush_with_landing",
    )
    ctx.expect_contact(car, shaft, elem_a=left_runner, elem_b=left_rail)
    ctx.expect_contact(car, shaft, elem_a=right_runner, elem_b=right_rail)
    ctx.expect_gap(
        car,
        shaft,
        axis="y",
        positive_elem=back_wall,
        negative_elem=rear_wall,
        min_gap=0.25,
        max_gap=0.40,
        name="car_has_rear_hoistway_clearance",
    )
    ctx.expect_gap(
        landing_door,
        car,
        axis="y",
        positive_elem=door_panel,
        negative_elem=toe_guard,
        min_gap=0.10,
        max_gap=0.16,
        name="landing_door_stays_forward_of_platform",
    )
    ctx.expect_gap(
        landing_door,
        shaft,
        axis="x",
        positive_elem=door_panel,
        negative_elem=door_stop,
        max_gap=0.0,
        max_penetration=0.0,
        name="landing_door_closes_against_left_stop",
    )
    ctx.expect_contact(landing_door, shaft, elem_a=left_shoe, elem_b=door_track)
    ctx.expect_contact(landing_door, shaft, elem_a=right_shoe, elem_b=door_track)

    if car_limits is not None and car_limits.upper is not None:
        with ctx.pose({car_lift: car_limits.upper}):
            ctx.expect_contact(car, shaft, elem_a=left_runner, elem_b=left_rail)
            ctx.expect_contact(car, shaft, elem_a=right_runner, elem_b=right_rail)
            ctx.expect_gap(
                car,
                shaft,
                axis="z",
                positive_elem=deck_surface,
                negative_elem=landing_plate,
                min_gap=1.34,
                max_gap=1.36,
                name="car_rises_through_full_freight_travel",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="car_lift_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="car_lift_upper_no_floating")

    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({landing_door_slide: door_limits.upper}):
            ctx.expect_contact(landing_door, shaft, elem_a=left_shoe, elem_b=door_track)
            ctx.expect_contact(landing_door, shaft, elem_a=right_shoe, elem_b=door_track)
            ctx.expect_gap(
                landing_door,
                shaft,
                axis="x",
                positive_elem=door_panel,
                negative_elem=door_stop,
                min_gap=1.29,
                max_gap=1.31,
                name="single_landing_door_slides_full_width",
            )
            ctx.expect_within(
                landing_door,
                shaft,
                axes="xz",
                inner_elem=door_panel,
                outer_elem=pocket_backer,
                name="door_stows_inside_right_pocket",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="landing_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="landing_door_upper_no_floating")

    if (
        car_limits is not None
        and car_limits.upper is not None
        and door_limits is not None
        and door_limits.upper is not None
    ):
        with ctx.pose({car_lift: car_limits.upper, landing_door_slide: door_limits.upper}):
            ctx.expect_contact(car, shaft, elem_a=left_runner, elem_b=left_rail)
            ctx.expect_contact(car, shaft, elem_a=right_runner, elem_b=right_rail)
            ctx.expect_contact(landing_door, shaft, elem_a=left_shoe, elem_b=door_track)
            ctx.expect_contact(landing_door, shaft, elem_a=right_shoe, elem_b=door_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="combined_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="combined_upper_pose_no_floating")

    for articulation, prefix in (
        (car_lift, "car_lift"),
        (landing_door_slide, "landing_door_slide"),
    ):
        limits = articulation.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({articulation: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_lower_no_floating")
            with ctx.pose({articulation: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_upper_pose_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_upper_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
