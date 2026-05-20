from __future__ import annotations

import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _ORIG_GETCWD()


os.getcwd = _safe_getcwd

try:
    os.chdir("/")
except OSError:
    pass

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

HERE = Path(__file__).resolve().parent
ASSETS = AssetContext.from_script(__file__)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_glass_elevator", assets=ASSETS)

    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    brushed_bronze = model.material("brushed_bronze", rgba=(0.54, 0.46, 0.34, 1.0))
    podium_stone = model.material("podium_stone", rgba=(0.73, 0.75, 0.76, 1.0))
    shaft_glass = model.material("shaft_glass", rgba=(0.70, 0.84, 0.92, 0.22))
    car_glass = model.material("car_glass", rgba=(0.67, 0.86, 0.95, 0.28))
    door_glass = model.material("door_glass", rgba=(0.73, 0.90, 0.98, 0.25))
    floor_dark = model.material("floor_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    light_panel = model.material("light_panel", rgba=(0.92, 0.94, 0.95, 0.95))

    shaft = model.part("shaft")
    shaft.visual(
        Box((1.64, 1.30, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=podium_stone,
        name="podium",
    )
    shaft.visual(
        Box((1.10, 0.28, 0.045)),
        origin=Origin(xyz=(0.0, 0.71, 0.0225)),
        material=podium_stone,
        name="landing_threshold",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            post_name = (
                f"{'front' if y_sign > 0.0 else 'rear'}_"
                f"{'right' if x_sign > 0.0 else 'left'}_tower_post"
            )
            shaft.visual(
                Box((0.06, 0.06, 3.30)),
                origin=Origin(xyz=(0.68 * x_sign, 0.56 * y_sign, 1.65)),
                material=brushed_bronze,
                name=post_name,
            )
    shaft.visual(
        Box((1.42, 1.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 3.34)),
        material=brushed_bronze,
        name="crown",
    )
    shaft.visual(
        Box((0.04, 0.72, 3.29)),
        origin=Origin(xyz=(-0.63, -0.17, 1.645)),
        material=shaft_glass,
        name="left_shaft_glass",
    )
    shaft.visual(
        Box((0.04, 0.72, 3.29)),
        origin=Origin(xyz=(0.63, -0.17, 1.645)),
        material=shaft_glass,
        name="right_shaft_glass",
    )
    shaft.visual(
        Box((1.22, 0.08, 3.29)),
        origin=Origin(xyz=(0.0, -0.49, 1.645)),
        material=shaft_glass,
        name="rear_shaft_glass",
    )
    shaft.visual(
        Box((1.30, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.56, 3.06)),
        material=brushed_bronze,
        name="front_header",
    )
    shaft.visual(
        Box((0.02, 0.05, 3.29)),
        origin=Origin(xyz=(-0.45, -0.465, 1.645)),
        material=dark_steel,
        name="left_guide_rail",
    )
    shaft.visual(
        Box((0.02, 0.05, 3.29)),
        origin=Origin(xyz=(0.45, -0.465, 1.645)),
        material=dark_steel,
        name="right_guide_rail",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((1.64, 1.30, 3.50)),
        mass=80.0,
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
    )

    car = model.part("car")
    car.visual(
        Box((1.02, 0.86, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=floor_dark,
        name="floor",
    )
    car.visual(
        Box((0.92, 0.76, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=brushed_bronze,
        name="floor_inlay",
    )
    car.visual(
        Box((1.04, 0.88, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 2.095)),
        material=steel,
        name="roof",
    )
    car.visual(
        Box((0.74, 0.20, 0.01)),
        origin=Origin(xyz=(0.0, -0.18, 2.06)),
        material=light_panel,
        name="ceiling_light",
    )
    car.visual(
        Box((0.05, 0.05, 2.02)),
        origin=Origin(xyz=(-0.485, -0.405, 1.055)),
        material=steel,
        name="rear_left_post",
    )
    car.visual(
        Box((0.05, 0.05, 2.02)),
        origin=Origin(xyz=(0.485, -0.405, 1.055)),
        material=steel,
        name="rear_right_post",
    )
    car.visual(
        Box((0.06, 0.06, 2.00)),
        origin=Origin(xyz=(-0.48, 0.41, 1.045)),
        material=steel,
        name="front_left_jamb",
    )
    car.visual(
        Box((0.06, 0.06, 2.00)),
        origin=Origin(xyz=(0.48, 0.41, 1.045)),
        material=steel,
        name="front_right_jamb",
    )
    car.visual(
        Box((1.18, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.44, 2.03)),
        material=dark_steel,
        name="door_header_guide",
    )
    car.visual(
        Box((1.18, 0.02, 0.022)),
        origin=Origin(xyz=(0.0, 0.44, 0.056)),
        material=dark_steel,
        name="door_sill_guide",
    )
    car.visual(
        Box((0.88, 0.03, 0.31)),
        origin=Origin(xyz=(0.0, 0.405, 1.91)),
        material=car_glass,
        name="front_transom_glass",
    )
    car.visual(
        Box((0.91, 0.012, 2.02)),
        origin=Origin(xyz=(0.0, -0.39, 1.055)),
        material=car_glass,
        name="rear_glass",
    )
    car.visual(
        Box((0.012, 0.76, 2.02)),
        origin=Origin(xyz=(-0.45, -0.01, 1.055)),
        material=car_glass,
        name="left_glass",
    )
    car.visual(
        Box((0.012, 0.76, 2.02)),
        origin=Origin(xyz=(0.45, -0.01, 1.055)),
        material=car_glass,
        name="right_glass",
    )
    car.visual(
        Cylinder(radius=0.012, length=0.72),
        origin=Origin(xyz=(0.0, -0.372, 0.94), rpy=(0.0, 1.57079632679, 0.0)),
        material=steel,
        name="rear_handrail",
    )
    car.visual(
        Cylinder(radius=0.012, length=0.48),
        origin=Origin(xyz=(-0.432, 0.0, 0.94), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="left_handrail",
    )
    car.visual(
        Cylinder(radius=0.012, length=0.48),
        origin=Origin(xyz=(0.432, 0.0, 0.94), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="right_handrail",
    )
    car.visual(
        Box((0.02, 0.044, 1.55)),
        origin=Origin(xyz=(-0.43, -0.418, 1.05)),
        material=dark_steel,
        name="left_guide_pad",
    )
    car.visual(
        Box((0.02, 0.044, 1.55)),
        origin=Origin(xyz=(0.43, -0.418, 1.05)),
        material=dark_steel,
        name="right_guide_pad",
    )
    car.inertial = Inertial.from_geometry(
        Box((1.06, 0.90, 2.20)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((0.425, 0.012, 1.66)),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=door_glass,
        name="glass",
    )
    left_door.visual(
        Box((0.425, 0.028, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="bottom_rail",
    )
    left_door.visual(
        Box((0.425, 0.028, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 1.84)),
        material=steel,
        name="top_rail",
    )
    left_door.visual(
        Box((0.022, 0.028, 1.78)),
        origin=Origin(xyz=(-0.2015, 0.0, 0.93)),
        material=steel,
        name="outer_stile",
    )
    left_door.visual(
        Box((0.020, 0.028, 1.78)),
        origin=Origin(xyz=(0.2015, 0.0, 0.93)),
        material=steel,
        name="meeting_stile",
    )
    left_door.visual(
        Box((0.34, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.915)),
        material=dark_steel,
        name="hanger_bar",
    )
    left_door.visual(
        Box((0.26, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="bottom_guide_bar",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((0.425, 0.03, 1.94)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((0.425, 0.012, 1.66)),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=door_glass,
        name="glass",
    )
    right_door.visual(
        Box((0.425, 0.028, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="bottom_rail",
    )
    right_door.visual(
        Box((0.425, 0.028, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 1.84)),
        material=steel,
        name="top_rail",
    )
    right_door.visual(
        Box((0.022, 0.028, 1.78)),
        origin=Origin(xyz=(0.2015, 0.0, 0.93)),
        material=steel,
        name="outer_stile",
    )
    right_door.visual(
        Box((0.020, 0.028, 1.78)),
        origin=Origin(xyz=(-0.2015, 0.0, 0.93)),
        material=steel,
        name="meeting_stile",
    )
    right_door.visual(
        Box((0.34, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.915)),
        material=dark_steel,
        name="hanger_bar",
    )
    right_door.visual(
        Box((0.26, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="bottom_guide_bar",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((0.425, 0.03, 1.94)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
    )

    model.articulation(
        "car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.7, lower=0.0, upper=1.05),
    )
    model.articulation(
        "left_door_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-0.2125, 0.465, 0.07)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.7, lower=0.0, upper=0.2475),
    )
    model.articulation(
        "right_door_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(0.2125, 0.465, 0.07)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.7, lower=0.0, upper=0.2475),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, seed=0)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    car_lift = object_model.get_articulation("car_lift")
    left_door_slide = object_model.get_articulation("left_door_slide")
    right_door_slide = object_model.get_articulation("right_door_slide")

    podium = shaft.get_visual("podium")
    left_guide_rail = shaft.get_visual("left_guide_rail")
    right_guide_rail = shaft.get_visual("right_guide_rail")
    car_floor = car.get_visual("floor")
    header_guide = car.get_visual("door_header_guide")
    sill_guide = car.get_visual("door_sill_guide")
    left_guide_pad = car.get_visual("left_guide_pad")
    right_guide_pad = car.get_visual("right_guide_pad")
    left_hanger_bar = left_door.get_visual("hanger_bar")
    right_hanger_bar = right_door.get_visual("hanger_bar")
    left_bottom_guide = left_door.get_visual("bottom_guide_bar")
    right_bottom_guide = right_door.get_visual("bottom_guide_bar")
    left_meeting_stile = left_door.get_visual("meeting_stile")
    right_meeting_stile = right_door.get_visual("meeting_stile")
    car_top = car_lift.motion_limits.upper if car_lift.motion_limits is not None else None
    left_open = (
        left_door_slide.motion_limits.upper
        if left_door_slide.motion_limits is not None
        else None
    )
    right_open = (
        right_door_slide.motion_limits.upper
        if right_door_slide.motion_limits is not None
        else None
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_origin_distance(
        car,
        shaft,
        axes="xy",
        max_dist=0.01,
        name="car_centered_in_shaft",
    )

    with ctx.pose({car_lift: 0.0, left_door_slide: 0.0, right_door_slide: 0.0}):
        ctx.expect_within(
            car,
            shaft,
            axes="xy",
            name="car_plan_within_shaft_at_landing",
        )
        ctx.expect_gap(
            car,
            shaft,
            axis="z",
            positive_elem=car_floor,
            negative_elem=podium,
            max_gap=0.0,
            max_penetration=0.0,
            name="car_floor_flush_with_podium_at_landing",
        )
        ctx.expect_overlap(
            car,
            shaft,
            axes="xy",
            elem_a=car_floor,
            elem_b=podium,
            min_overlap=0.80,
            name="car_floor_supported_by_podium",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=left_guide_pad,
            elem_b=left_guide_rail,
            name="left_guide_pad_contacts_left_rail_at_landing",
        )
        ctx.expect_contact(
            car,
            shaft,
            elem_a=right_guide_pad,
            elem_b=right_guide_rail,
            name="right_guide_pad_contacts_right_rail_at_landing",
        )
        ctx.expect_contact(
            left_door,
            car,
            elem_a=left_hanger_bar,
            elem_b=header_guide,
            name="left_door_hanger_supported_at_landing",
        )
        ctx.expect_contact(
            right_door,
            car,
            elem_a=right_hanger_bar,
            elem_b=header_guide,
            name="right_door_hanger_supported_at_landing",
        )
        ctx.expect_gap(
            left_door,
            car,
            axis="z",
            positive_elem=left_bottom_guide,
            negative_elem=sill_guide,
            min_gap=0.002,
            max_gap=0.004,
            name="left_bottom_guide_clears_sill_at_landing",
        )
        ctx.expect_gap(
            right_door,
            car,
            axis="z",
            positive_elem=right_bottom_guide,
            negative_elem=sill_guide,
            min_gap=0.002,
            max_gap=0.004,
            name="right_bottom_guide_clears_sill_at_landing",
        )
        ctx.expect_overlap(
            left_door,
            car,
            axes="x",
            elem_a=left_bottom_guide,
            elem_b=sill_guide,
            min_overlap=0.20,
            name="left_bottom_guide_stays_over_sill_at_landing",
        )
        ctx.expect_overlap(
            right_door,
            car,
            axes="x",
            elem_a=right_bottom_guide,
            elem_b=sill_guide,
            min_overlap=0.20,
            name="right_bottom_guide_stays_over_sill_at_landing",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.001,
            max_gap=0.003,
            positive_elem=right_meeting_stile,
            negative_elem=left_meeting_stile,
            name="door_meeting_stiles_leave_center_seal_gap",
        )

    if left_open is not None and right_open is not None:
        with ctx.pose({left_door_slide: left_open, right_door_slide: right_open}):
            ctx.fail_if_parts_overlap_in_current_pose(name="doors_open_no_overlap")
            ctx.fail_if_isolated_parts(name="doors_open_no_floating")
            ctx.expect_gap(
                right_door,
                left_door,
                axis="x",
                min_gap=0.49,
                positive_elem=right_meeting_stile,
                negative_elem=left_meeting_stile,
                name="doors_create_clear_center_opening",
            )
            ctx.expect_contact(
                left_door,
                car,
                elem_a=left_hanger_bar,
                elem_b=header_guide,
                name="left_door_remains_hung_when_open",
            )
            ctx.expect_contact(
                right_door,
                car,
                elem_a=right_hanger_bar,
                elem_b=header_guide,
                name="right_door_remains_hung_when_open",
            )
            ctx.expect_gap(
                left_door,
                car,
                axis="z",
                positive_elem=left_bottom_guide,
                negative_elem=sill_guide,
                min_gap=0.002,
                max_gap=0.004,
                name="left_bottom_guide_clears_sill_when_open",
            )
            ctx.expect_gap(
                right_door,
                car,
                axis="z",
                positive_elem=right_bottom_guide,
                negative_elem=sill_guide,
                min_gap=0.002,
                max_gap=0.004,
                name="right_bottom_guide_clears_sill_when_open",
            )

    if car_top is not None:
        with ctx.pose({car_lift: car_top}):
            ctx.fail_if_parts_overlap_in_current_pose(name="car_top_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="car_top_pose_no_floating")
            ctx.expect_contact(
                car,
                shaft,
                elem_a=left_guide_pad,
                elem_b=left_guide_rail,
                name="left_guide_pad_contacts_left_rail_at_top",
            )
            ctx.expect_contact(
                car,
                shaft,
                elem_a=right_guide_pad,
                elem_b=right_guide_rail,
                name="right_guide_pad_contacts_right_rail_at_top",
            )
            ctx.expect_within(
                car,
                shaft,
                axes="xy",
                name="car_plan_within_shaft_at_top",
            )
            ctx.expect_gap(
                car,
                shaft,
                axis="z",
                positive_elem=car_floor,
                negative_elem=podium,
                min_gap=1.00,
                name="car_floor_lifts_clear_of_podium",
            )

            if left_open is not None and right_open is not None:
                with ctx.pose({left_door_slide: left_open, right_door_slide: right_open}):
                    ctx.fail_if_parts_overlap_in_current_pose(
                        name="top_open_pose_no_overlap"
                    )
                    ctx.fail_if_isolated_parts(name="top_open_pose_no_floating")
                    ctx.expect_gap(
                        right_door,
                        left_door,
                        axis="x",
                        min_gap=0.49,
                        positive_elem=right_meeting_stile,
                        negative_elem=left_meeting_stile,
                        name="top_open_pose_retains_clear_doorway",
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
