from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


CABINET_WIDTH = 0.80
CABINET_DEPTH = 0.56
CABINET_HEIGHT = 0.87
SIDE_THICKNESS = 0.018
BOTTOM_THICKNESS = 0.018
BACK_THICKNESS = 0.018
PLINTH_HEIGHT = 0.10

COUNTER_WIDTH = 0.84
COUNTER_DEPTH = 0.64
COUNTER_THICKNESS = 0.03
COOKTOP_OPENING_WIDTH = 0.56
COOKTOP_OPENING_DEPTH = 0.49
COOKTOP_OPENING_CENTER_Y = -0.02

DOOR_HEIGHT = 0.72
DOOR_WIDTH = 0.396
DOOR_THICKNESS = 0.021
HINGE_RADIUS = 0.008
HINGE_AXIS_Y = -(CABINET_DEPTH * 0.5)
LEFT_HINGE_X = -(CABINET_WIDTH * 0.5) - 0.001
RIGHT_HINGE_X = (CABINET_WIDTH * 0.5) + 0.001
DOOR_BOTTOM_Z = PLINTH_HEIGHT + BOTTOM_THICKNESS + 0.004

BUTTON_TRAVEL = 0.004
BUTTON_POSITIONS = (
    (-0.180, -0.028),
    (-0.100, -0.021),
    (-0.020, -0.014),
    (0.060, -0.0075),
)
KNOB_POSITION = (0.205, -0.027)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_cooktop_cabinet_bay", assets=ASSETS)

    stone = model.material("stone_quartz", rgba=(0.70, 0.71, 0.69, 1.0))
    cabinet_paint = model.material("cabinet_paint", rgba=(0.94, 0.95, 0.93, 1.0))
    plinth_finish = model.material("plinth_finish", rgba=(0.32, 0.32, 0.31, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.06, 0.07, 0.08, 1.0))
    cooktop_body_finish = model.material("cooktop_body_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.25, 0.27, 0.29, 1.0))
    control_finish = model.material("control_finish", rgba=(0.78, 0.79, 0.80, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    pull_finish = model.material("pull_finish", rgba=(0.66, 0.67, 0.69, 1.0))

    cabinet_body = model.part("cabinet_body")
    side_height = CABINET_HEIGHT - PLINTH_HEIGHT
    side_center_z = PLINTH_HEIGHT + (side_height * 0.5)
    side_center_x = (CABINET_WIDTH * 0.5) - (SIDE_THICKNESS * 0.5)
    cabinet_body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, side_height)),
        origin=Origin(xyz=(-side_center_x, 0.0, side_center_z)),
        material=cabinet_paint,
        name="left_side",
    )
    cabinet_body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, side_height)),
        origin=Origin(xyz=(side_center_x, 0.0, side_center_z)),
        material=cabinet_paint,
        name="right_side",
    )
    cabinet_body.visual(
        Box(
            (
                CABINET_WIDTH - (2.0 * SIDE_THICKNESS),
                CABINET_DEPTH,
                BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + (BOTTOM_THICKNESS * 0.5))),
        material=cabinet_paint,
        name="cabinet_bottom",
    )
    cabinet_body.visual(
        Box(
            (
                CABINET_WIDTH - (2.0 * SIDE_THICKNESS),
                BACK_THICKNESS,
                side_height,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                (CABINET_DEPTH * 0.5) - (BACK_THICKNESS * 0.5),
                side_center_z,
            )
        ),
        material=cabinet_paint,
        name="back_panel",
    )
    cabinet_body.visual(
        Box((CABINET_WIDTH - (2.0 * SIDE_THICKNESS), BACK_THICKNESS, PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH * 0.5) + 0.060 + (BACK_THICKNESS * 0.5),
                PLINTH_HEIGHT * 0.5,
            )
        ),
        material=plinth_finish,
        name="toe_kick",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT * 0.5)),
    )

    countertop = model.part("countertop")
    left_right_strip_width = (COUNTER_WIDTH - COOKTOP_OPENING_WIDTH) * 0.5
    front_strip_depth = 0.055
    back_strip_depth = COUNTER_DEPTH - front_strip_depth - COOKTOP_OPENING_DEPTH
    strip_center_z = COUNTER_THICKNESS * 0.5
    countertop.visual(
        Box((left_right_strip_width, COOKTOP_OPENING_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                -((COUNTER_WIDTH * 0.5) - (left_right_strip_width * 0.5)),
                COOKTOP_OPENING_CENTER_Y,
                strip_center_z,
            )
        ),
        material=stone,
        name="left_counter_run",
    )
    countertop.visual(
        Box((left_right_strip_width, COOKTOP_OPENING_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                (COUNTER_WIDTH * 0.5) - (left_right_strip_width * 0.5),
                COOKTOP_OPENING_CENTER_Y,
                strip_center_z,
            )
        ),
        material=stone,
        name="right_counter_run",
    )
    countertop.visual(
        Box((COUNTER_WIDTH, front_strip_depth, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(COUNTER_DEPTH * 0.5) + (front_strip_depth * 0.5),
                strip_center_z,
            )
        ),
        material=stone,
        name="front_counter_run",
    )
    countertop.visual(
        Box((COUNTER_WIDTH, back_strip_depth, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (COUNTER_DEPTH * 0.5) - (back_strip_depth * 0.5),
                strip_center_z,
            )
        ),
        material=stone,
        name="back_counter_run",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, strip_center_z)),
    )
    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT)),
    )

    cooktop_body = model.part("cooktop_body")
    cooktop_body.visual(
        Box((COOKTOP_OPENING_WIDTH, 0.390, 0.110)),
        origin=Origin(xyz=(0.0, 0.030, -0.033)),
        material=cooktop_body_finish,
        name="cooktop_housing",
    )
    cooktop_body.visual(
        Box((COOKTOP_OPENING_WIDTH, COOKTOP_OPENING_DEPTH, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cooktop_glass,
        name="glass_top",
    )

    for zone_name, zone_x, zone_y in (
        ("zone_front_left", -0.145, -0.110),
        ("zone_front_right", 0.145, -0.110),
        ("zone_rear_left", -0.145, 0.110),
        ("zone_rear_right", 0.145, 0.110),
    ):
        cooktop_body.visual(
            Cylinder(radius=0.090, length=0.008),
            origin=Origin(xyz=(zone_x, zone_y, 0.026)),
            material=zone_mark,
            name=zone_name,
        )

    button_stem_width = 0.018
    button_stem_depth = 0.040
    button_stem_height = 0.014
    button_cap_width = 0.030
    button_cap_depth = 0.014
    button_cap_height = 0.006
    guide_wall = 0.004
    guide_depth = 0.060
    guide_height = button_stem_height + (2.0 * guide_wall)
    button_origin_y = -0.183

    for index, (button_x, button_z) in enumerate(BUTTON_POSITIONS, start=1):
        cooktop_body.visual(
            Box((guide_wall, guide_depth, guide_height)),
            origin=Origin(xyz=(button_x - 0.011, -0.195, button_z)),
            material=cooktop_body_finish,
            name=f"button_{index}_left_guide",
        )
        cooktop_body.visual(
            Box((guide_wall, guide_depth, guide_height)),
            origin=Origin(xyz=(button_x + 0.011, -0.195, button_z)),
            material=cooktop_body_finish,
            name=f"button_{index}_right_guide",
        )
        cooktop_body.visual(
            Box((button_stem_width, guide_depth, guide_wall)),
            origin=Origin(xyz=(button_x, -0.195, button_z - 0.009)),
            material=cooktop_body_finish,
            name=f"button_{index}_lower_guide",
        )
        cooktop_body.visual(
            Box((button_stem_width, guide_depth, guide_wall)),
            origin=Origin(xyz=(button_x, -0.195, button_z + 0.009)),
            material=cooktop_body_finish,
            name=f"button_{index}_upper_guide",
        )

        button_part = model.part(f"button_{index}")
        button_part.visual(
            Box((button_stem_width, button_stem_depth, button_stem_height)),
            origin=Origin(xyz=(0.0, -0.020, 0.0)),
            material=control_finish,
            name="button_stem",
        )
        button_part.visual(
            Box((button_cap_width, button_cap_depth, button_cap_height)),
            origin=Origin(xyz=(0.0, -0.047, 0.0)),
            material=control_finish,
            name="button_cap",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((button_cap_width, 0.054, button_stem_height)),
            mass=0.030,
            origin=Origin(xyz=(0.0, -0.027, 0.0)),
        )
        model.articulation(
            f"cooktop_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cooktop_body,
            child=button_part,
            origin=Origin(xyz=(button_x, button_origin_y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_finish,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.028, 0.014)),
        material=control_finish,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.030),
        mass=0.07,
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    for suffix, x_offset in (("left", -0.020), ("right", 0.020)):
        cooktop_body.visual(
            Box((guide_wall, guide_depth, 0.044)),
            origin=Origin(xyz=(KNOB_POSITION[0] + x_offset, -0.195, KNOB_POSITION[1])),
            material=cooktop_body_finish,
            name=f"knob_{suffix}_guide",
        )
    for suffix, z_offset in (("lower", -0.020), ("upper", 0.020)):
        cooktop_body.visual(
            Box((0.036, guide_depth, guide_wall)),
            origin=Origin(xyz=(KNOB_POSITION[0], -0.195, KNOB_POSITION[1] + z_offset)),
            material=cooktop_body_finish,
            name=f"knob_{suffix}_guide",
        )
    model.articulation(
        "cooktop_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=cooktop_body,
        child=knob,
        origin=Origin(xyz=(KNOB_POSITION[0], button_origin_y, KNOB_POSITION[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    cooktop_body.inertial = Inertial.from_geometry(
        Box((COOKTOP_OPENING_WIDTH, COOKTOP_OPENING_DEPTH, 0.118)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
    )
    model.articulation(
        "countertop_to_cooktop_body",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop_body,
        origin=Origin(xyz=(0.0, COOKTOP_OPENING_CENTER_Y, 0.0)),
    )

    def add_door(
        *,
        name: str,
        hinge_name: str,
        hinge_x: float,
        panel_center_x: float,
        lower: float,
        upper: float,
    ) -> None:
        door = model.part(name)
        door.visual(
            Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
            origin=Origin(xyz=(panel_center_x, -(DOOR_THICKNESS * 0.5), DOOR_HEIGHT * 0.5)),
            material=cabinet_paint,
            name="door_panel",
        )
        door.visual(
            Box((0.018, 0.012, 0.200)),
            origin=Origin(
                xyz=(
                    panel_center_x - math.copysign((DOOR_WIDTH * 0.5) - 0.048, panel_center_x),
                    -0.016,
                    0.410,
                )
            ),
            material=pull_finish,
            name="door_pull",
        )
        door.inertial = Inertial.from_geometry(
            Box((DOOR_WIDTH, 0.033, DOOR_HEIGHT)),
            mass=4.2,
            origin=Origin(xyz=(panel_center_x, -0.010, DOOR_HEIGHT * 0.5)),
        )
        model.articulation(
            hinge_name,
            ArticulationType.REVOLUTE,
            parent=cabinet_body,
            child=door,
            origin=Origin(xyz=(hinge_x, HINGE_AXIS_Y, DOOR_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.8,
                lower=lower,
                upper=upper,
            ),
        )

    add_door(
        name="left_door",
        hinge_name="cabinet_to_left_door",
        hinge_x=LEFT_HINGE_X,
        panel_center_x=DOOR_WIDTH * 0.5,
        lower=-1.55,
        upper=0.0,
    )
    add_door(
        name="right_door",
        hinge_name="cabinet_to_right_door",
        hinge_x=RIGHT_HINGE_X,
        panel_center_x=-(DOOR_WIDTH * 0.5),
        lower=0.0,
        upper=1.55,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet_body = object_model.get_part("cabinet_body")
    countertop = object_model.get_part("countertop")
    cooktop_body = object_model.get_part("cooktop_body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 5)]
    knob = object_model.get_part("control_knob")

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    button_joints = [
        object_model.get_articulation(f"cooktop_to_button_{index}") for index in range(1, 5)
    ]
    knob_joint = object_model.get_articulation("cooktop_to_control_knob")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_gap(countertop, cabinet_body, axis="z", min_gap=0.0, max_gap=0.0)
    ctx.expect_contact(countertop, cooktop_body)
    ctx.expect_gap(countertop, left_door, axis="z", min_gap=0.015, max_gap=0.040, negative_elem="door_panel")
    ctx.expect_gap(countertop, right_door, axis="z", min_gap=0.015, max_gap=0.040, negative_elem="door_panel")
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.004,
        max_gap=0.0145,
        positive_elem="door_panel",
        negative_elem="door_panel",
    )
    ctx.expect_contact(left_door, cabinet_body, contact_tol=0.0015)
    ctx.expect_contact(right_door, cabinet_body, contact_tol=0.0015)
    ctx.expect_contact(knob, cooktop_body)

    for button in buttons:
        ctx.expect_contact(button, cooktop_body)

    glass_aabb = ctx.part_element_world_aabb(cooktop_body, elem="glass_top")
    countertop_aabb = ctx.part_world_aabb(countertop)
    if glass_aabb is not None and countertop_aabb is not None:
        top_delta = abs(glass_aabb[1][2] - countertop_aabb[1][2])
        ctx.check(
            "cooktop_glass_flush_with_countertop",
            top_delta <= 0.001,
            f"glass top and countertop top differ by {top_delta:.4f} m",
        )
    else:
        ctx.fail("cooktop_glass_flush_with_countertop", "missing AABB for glass top or countertop")

    zone_names = (
        "zone_front_left",
        "zone_front_right",
        "zone_rear_left",
        "zone_rear_right",
    )
    for zone_name in zone_names:
        zone_aabb = ctx.part_element_world_aabb(cooktop_body, elem=zone_name)
        if zone_aabb is None or glass_aabb is None:
            ctx.fail(f"{zone_name}_within_glass_top", "missing zone or glass AABB")
            continue
        within_xy = (
            zone_aabb[0][0] >= glass_aabb[0][0] - 1e-6
            and zone_aabb[1][0] <= glass_aabb[1][0] + 1e-6
            and zone_aabb[0][1] >= glass_aabb[0][1] - 1e-6
            and zone_aabb[1][1] <= glass_aabb[1][1] + 1e-6
        )
        ctx.check(
            f"{zone_name}_within_glass_top",
            within_xy,
            f"{zone_name} must stay inside the glass footprint",
        )

    button_cap_centers = []
    for button in buttons:
        button_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
        if button_aabb is None:
            ctx.fail(f"{button.name}_cap_present", "button cap AABB missing")
            continue
        button_cap_centers.append(_aabb_center(button_aabb))

    if len(button_cap_centers) == 4 and all(center is not None for center in button_cap_centers):
        diagonal_ok = all(
            button_cap_centers[index][0] < button_cap_centers[index + 1][0]
            and button_cap_centers[index][2] < button_cap_centers[index + 1][2]
            for index in range(3)
        )
        total_rise = button_cap_centers[-1][2] - button_cap_centers[0][2]
        ctx.check(
            "button_caps_form_rising_diagonal",
            diagonal_ok and total_rise >= 0.015,
            f"button cap centers were {button_cap_centers}",
        )
    else:
        ctx.fail("button_caps_form_rising_diagonal", "missing one or more button cap centers")

    knob_aabb = ctx.part_element_world_aabb(knob, elem="knob_body")
    if knob_aabb is not None and button_cap_centers and button_cap_centers[-1] is not None:
        knob_center = _aabb_center(knob_aabb)
        far_right_ok = knob_center is not None and knob_center[0] >= button_cap_centers[-1][0] + 0.10
        ctx.check(
            "knob_is_far_right_of_button_row",
            far_right_ok,
            f"knob center {knob_center} should sit well right of button row {button_cap_centers[-1]}",
        )
    else:
        ctx.fail("knob_is_far_right_of_button_row", "missing knob center or button row centers")

    button_axes_ok = all(
        joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (0.0, 1.0, 0.0)
        for joint in button_joints
    )
    ctx.check(
        "button_joint_axes_and_types",
        button_axes_ok,
        "all four buttons must be short-travel inward prismatic joints",
    )
    ctx.check(
        "knob_joint_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        "knob must rotate continuously around a front-to-back axis",
    )
    ctx.check(
        "door_joints_are_vertical_revolutes",
        left_hinge.articulation_type == ArticulationType.REVOLUTE
        and right_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_hinge.axis) == (0.0, 0.0, 1.0)
        and tuple(right_hinge.axis) == (0.0, 0.0, 1.0),
        "cabinet doors must swing on vertical hinges",
    )

    closed_left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    closed_right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")

    for joint in button_joints:
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        button_index = button_joints.index(joint) + 1
        button_part = object_model.get_part(f"button_{button_index}")
        rest_aabb = ctx.part_element_world_aabb(button_part, elem="button_cap")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
            ctx.expect_contact(button_part, cooktop_body, name=f"{joint.name}_upper_contact")
            pressed_aabb = ctx.part_element_world_aabb(button_part, elem="button_cap")
            if rest_aabb is not None and pressed_aabb is not None:
                delta_y = _aabb_center(pressed_aabb)[1] - _aabb_center(rest_aabb)[1]
                ctx.check(
                    f"{joint.name}_moves_inward",
                    abs(delta_y - BUTTON_TRAVEL) <= 0.0005,
                    f"{joint.name} moved {delta_y:.4f} m instead of about {BUTTON_TRAVEL:.4f} m",
                )
            else:
                ctx.fail(f"{joint.name}_moves_inward", "missing button cap AABB during press test")

    with ctx.pose({knob_joint: 1.7}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_pose_no_floating")
        ctx.expect_contact(knob, cooktop_body, name="knob_pose_contact")

    left_limits = left_hinge.motion_limits
    if left_limits is not None and left_limits.lower is not None and closed_left_panel is not None:
        with ctx.pose({left_hinge: left_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="left_door_open_no_floating")
            ctx.expect_contact(
                left_door,
                cabinet_body,
                contact_tol=0.0015,
                name="left_door_open_hinge_contact",
            )
            open_left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
            if open_left_panel is not None:
                swung_out = open_left_panel[0][1] <= closed_left_panel[0][1] - 0.10
                ctx.check(
                    "left_door_swings_forward",
                    swung_out,
                    f"left door panel min-y {open_left_panel[0][1]:.4f} did not move forward enough from {closed_left_panel[0][1]:.4f}",
                )
            else:
                ctx.fail("left_door_swings_forward", "missing left door panel AABB in open pose")

    right_limits = right_hinge.motion_limits
    if right_limits is not None and right_limits.upper is not None and closed_right_panel is not None:
        with ctx.pose({right_hinge: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="right_door_open_no_floating")
            ctx.expect_contact(
                right_door,
                cabinet_body,
                contact_tol=0.0015,
                name="right_door_open_hinge_contact",
            )
            open_right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
            if open_right_panel is not None:
                swung_out = open_right_panel[0][1] <= closed_right_panel[0][1] - 0.10
                ctx.check(
                    "right_door_swings_forward",
                    swung_out,
                    f"right door panel min-y {open_right_panel[0][1]:.4f} did not move forward enough from {closed_right_panel[0][1]:.4f}",
                )
            else:
                ctx.fail("right_door_swings_forward", "missing right door panel AABB in open pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
