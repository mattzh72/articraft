from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

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

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_barrier_gate")

    housing_paint = model.material("housing_paint", rgba=(0.86, 0.88, 0.90, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.21, 0.23, 0.25, 1.0))
    door_paint = model.material("door_paint", rgba=(0.79, 0.81, 0.83, 1.0))
    cabinet_interior = model.material("cabinet_interior", rgba=(0.33, 0.35, 0.37, 1.0))
    boom_white = model.material("boom_white", rgba=(0.95, 0.96, 0.96, 1.0))
    safety_red = model.material("safety_red", rgba=(0.87, 0.14, 0.14, 1.0))
    warning_amber = model.material("warning_amber", rgba=(0.93, 0.64, 0.10, 1.0))
    counterweight_dark = model.material("counterweight_dark", rgba=(0.27, 0.29, 0.31, 1.0))

    plinth_w = 0.46
    plinth_d = 0.38
    plinth_h = 0.06
    cabinet_w = 0.28
    cabinet_d = 0.34
    cabinet_h = 0.84
    wall_t = 0.02
    top_cap_h = 0.03
    door_t = 0.018
    door_w = cabinet_w - (2.0 * wall_t) - 0.006
    door_h = 0.72
    door_bottom = plinth_h + 0.08

    housing = model.part("housing")
    housing.visual(
        Box((plinth_w, plinth_d, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h / 2.0)),
        material=housing_trim,
        name="foundation_plinth",
    )
    housing.visual(
        Box((cabinet_w - 0.02, cabinet_d - 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + 0.01)),
        material=cabinet_interior,
        name="floor_pan",
    )
    housing.visual(
        Box((wall_t, cabinet_d, cabinet_h)),
        origin=Origin(
            xyz=(
                -(cabinet_w / 2.0) + (wall_t / 2.0),
                0.0,
                plinth_h + (cabinet_h / 2.0),
            )
        ),
        material=housing_paint,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, cabinet_d, cabinet_h)),
        origin=Origin(
            xyz=(
                (cabinet_w / 2.0) - (wall_t / 2.0),
                0.0,
                plinth_h + (cabinet_h / 2.0),
            )
        ),
        material=housing_paint,
        name="right_wall",
    )
    housing.visual(
        Box((cabinet_w - (2.0 * wall_t), wall_t, cabinet_h)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_d / 2.0) - (wall_t / 2.0),
                plinth_h + (cabinet_h / 2.0),
            )
        ),
        material=housing_paint,
        name="back_wall",
    )
    housing.visual(
        Box((cabinet_w - (2.0 * wall_t), wall_t, 0.03)),
        origin=Origin(
            xyz=(0.0, -(cabinet_d / 2.0) + (wall_t / 2.0), door_bottom - 0.015)
        ),
        material=housing_trim,
        name="front_threshold",
    )
    housing.visual(
        Box((cabinet_w - (2.0 * wall_t), wall_t, 0.06)),
        origin=Origin(
            xyz=(0.0, -(cabinet_d / 2.0) + (wall_t / 2.0), door_bottom + door_h + 0.03)
        ),
        material=housing_trim,
        name="front_header",
    )
    housing.visual(
        Box((cabinet_w + 0.04, cabinet_d + 0.04, top_cap_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + cabinet_h + (top_cap_h / 2.0))),
        material=housing_trim,
        name="top_cap",
    )
    housing.visual(
        Box((0.08, 0.10, 0.40)),
        origin=Origin(xyz=(-0.06, 0.08, plinth_h + 0.20)),
        material=cabinet_interior,
        name="control_stack",
    )
    housing.visual(
        Box((0.14, 0.18, 0.18)),
        origin=Origin(xyz=(0.03, 0.05, plinth_h + 0.09)),
        material=cabinet_interior,
        name="drive_motor",
    )
    housing.visual(
        Box((0.06, 0.10, 0.10)),
        origin=Origin(xyz=(0.17, 0.22, 0.97)),
        material=housing_trim,
        name="hinge_pod",
    )
    housing.visual(
        Box((0.03, 0.08, 0.12)),
        origin=Origin(xyz=(0.215, 0.22, 0.98)),
        material=housing_trim,
        name="hinge_cheek",
    )
    housing.inertial = Inertial.from_geometry(
        Box((plinth_w, plinth_d, 1.04)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h / 2.0)),
        material=door_paint,
        name="door_panel",
    )
    service_door.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(
            xyz=(door_w - 0.040, -0.015, 0.42),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_trim,
        name="door_handle",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=5.0,
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h / 2.0)),
    )

    boom = model.part("boom_arm")
    boom.visual(
        Cylinder(radius=0.045, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=counterweight_dark,
        name="pivot_drum",
    )
    boom.visual(
        Box((0.10, 0.12, 0.09)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=housing_trim,
        name="pivot_mount",
    )
    boom.visual(
        Box((3.45, 0.09, 0.07)),
        origin=Origin(xyz=(1.825, 0.0, 0.0)),
        material=boom_white,
        name="main_arm",
    )
    boom.visual(
        Box((0.40, 0.10, 0.10)),
        origin=Origin(xyz=(-0.05, 0.11, -0.045)),
        material=housing_trim,
        name="rear_stub",
    )
    boom.visual(
        Box((0.06, 0.06, 0.22)),
        origin=Origin(xyz=(-0.23, 0.11, -0.17)),
        material=housing_trim,
        name="counterweight_post",
    )
    boom.visual(
        Box((0.18, 0.12, 0.14)),
        origin=Origin(xyz=(-0.34, 0.11, -0.31)),
        material=counterweight_dark,
        name="counterweight",
    )
    for idx, x_center in enumerate((0.45, 0.90, 1.35, 1.80, 2.25, 2.70, 3.15), start=1):
        boom.visual(
            Box((0.22, 0.094, 0.006)),
            origin=Origin(xyz=(x_center, 0.0, 0.038)),
            material=safety_red,
            name=f"stripe_{idx}",
        )
    boom.visual(
        Box((0.06, 0.10, 0.08)),
        origin=Origin(xyz=(3.58, 0.0, 0.0)),
        material=boom_white,
        name="tip_cap",
    )
    boom.visual(
        Box((0.06, 0.06, 0.025)),
        origin=Origin(xyz=(3.56, 0.0, 0.048)),
        material=warning_amber,
        name="tip_light",
    )
    boom.inertial = Inertial.from_geometry(
        Box((3.95, 0.16, 0.38)),
        mass=19.0,
        origin=Origin(xyz=(1.62, 0.0, -0.02)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=service_door,
        origin=Origin(xyz=(-(door_w / 2.0), -(cabinet_d / 2.0) - (door_t / 2.0), door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "boom_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=boom,
        origin=Origin(xyz=(0.275, 0.22, 0.98)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(84.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    service_door = object_model.get_part("service_door")
    boom = object_model.get_part("boom_arm")
    door_hinge = object_model.get_articulation("door_hinge")
    boom_hinge = object_model.get_articulation("boom_hinge")
    top_cap = housing.get_visual("top_cap")
    front_header = housing.get_visual("front_header")
    hinge_cheek = housing.get_visual("hinge_cheek")
    door_panel = service_door.get_visual("door_panel")
    pivot_drum = boom.get_visual("pivot_drum")
    main_arm = boom.get_visual("main_arm")
    counterweight = boom.get_visual("counterweight")
    tip_light = boom.get_visual("tip_light")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    door_limits = door_hinge.motion_limits
    boom_limits = boom_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_lower_no_floating")
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")
    if boom_limits is not None and boom_limits.lower is not None and boom_limits.upper is not None:
        with ctx.pose({boom_hinge: boom_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_hinge_lower_no_floating")
        with ctx.pose({boom_hinge: boom_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="boom_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="boom_hinge_upper_no_floating")
        with ctx.pose(
            {
                door_hinge: door_limits.upper if door_limits is not None else 0.0,
                boom_hinge: boom_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="all_articulations_open_no_overlap")
            ctx.fail_if_isolated_parts(name="all_articulations_open_no_floating")

    ctx.expect_gap(
        housing,
        service_door,
        axis="y",
        positive_elem=front_header,
        negative_elem=door_panel,
        min_gap=0.0,
        max_gap=0.001,
        name="service_door_sits_flush_with_front_frame",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="x",
        positive_elem=pivot_drum,
        negative_elem=hinge_cheek,
        max_penetration=0.001,
        max_gap=0.001,
        name="pivot_drum_sits_against_hinge_cheek",
    )
    ctx.expect_overlap(
        boom,
        housing,
        axes="yz",
        elem_a=pivot_drum,
        elem_b=hinge_cheek,
        min_overlap=0.055,
        name="pivot_drum_aligned_with_hinge_cheek",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="z",
        positive_elem=main_arm,
        negative_elem=top_cap,
        min_gap=0.01,
        max_gap=0.03,
        name="resting_boom_runs_just_above_housing_roof",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="x",
        positive_elem=main_arm,
        negative_elem=hinge_cheek,
        min_gap=0.10,
        name="boom_projects_forward_of_housing",
    )
    ctx.expect_gap(
        housing,
        boom,
        axis="x",
        positive_elem=hinge_cheek,
        negative_elem=counterweight,
        min_gap=0.14,
        name="counterweight_sits_behind_hinge",
    )
    ctx.expect_gap(
        housing,
        boom,
        axis="z",
        positive_elem=top_cap,
        negative_elem=counterweight,
        min_gap=0.08,
        name="counterweight_hangs_below_roofline",
    )

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem=door_panel)
    housing_aabb = ctx.part_world_aabb(housing)
    if closed_door_aabb is not None and housing_aabb is not None:
        ctx.check(
            "service_door_fills_front_opening",
            closed_door_aabb[0][0] >= housing_aabb[0][0] + 0.10
            and closed_door_aabb[1][0] <= housing_aabb[1][0] - 0.10
            and closed_door_aabb[0][2] >= 0.14
            and closed_door_aabb[1][2] <= 0.92,
            details=f"door_aabb={closed_door_aabb}, housing_aabb={housing_aabb}",
        )

    rest_tip_aabb = ctx.part_element_world_aabb(boom, elem=tip_light)
    with ctx.pose({boom_hinge: math.radians(80.0)}):
        ctx.expect_gap(
            boom,
            housing,
            axis="z",
            positive_elem=tip_light,
            negative_elem=top_cap,
            min_gap=3.2,
            name="raised_boom_tip_clears_far_above_housing",
        )
        ctx.expect_gap(
            boom,
            housing,
            axis="x",
            positive_elem=pivot_drum,
            negative_elem=hinge_cheek,
            max_penetration=0.001,
            max_gap=0.001,
            name="raised_boom_keeps_pivot_seated",
        )
        raised_tip_aabb = ctx.part_element_world_aabb(boom, elem=tip_light)
        if rest_tip_aabb is not None and raised_tip_aabb is not None:
            ctx.check(
                "boom_tip_rises_by_more_than_three_meters",
                raised_tip_aabb[0][2] >= rest_tip_aabb[0][2] + 3.2,
                details=f"rest_tip={rest_tip_aabb}, raised_tip={raised_tip_aabb}",
            )

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem=door_panel)
    with ctx.pose({door_hinge: math.radians(100.0)}):
        open_door_aabb = ctx.part_element_world_aabb(service_door, elem=door_panel)
        if closed_door_aabb is not None and open_door_aabb is not None:
            ctx.check(
                "service_door_swings_outward",
                open_door_aabb[0][1] <= closed_door_aabb[0][1] - 0.20,
                details=f"closed_door={closed_door_aabb}, open_door={open_door_aabb}",
            )
        ctx.expect_gap(
            housing,
            service_door,
            axis="x",
            positive_elem=hinge_cheek,
            negative_elem=door_panel,
            min_gap=0.05,
            name="open_service_door_stays_clear_of_boom_pod",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
