from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

COUNTER_WIDTH = 1.80
COUNTER_DEPTH = 0.95
COUNTER_THICKNESS = 0.04

LEG_SIZE = 0.08
LEG_HEIGHT = 0.88
LEG_X_INSET = 0.14
LEG_Y_INSET = 0.12

COOKTOP_WIDTH = 0.60
COOKTOP_DEPTH = 0.52
COOKTOP_GLASS_THICKNESS = 0.008
COOKTOP_SUPPORT_LIP = 0.015
COOKTOP_SUPPORT_THICKNESS = 0.004

ZONE_RADIUS = 0.095
ZONE_X = 0.155
ZONE_Y = 0.115

BUTTON_SIZE = 0.018
BUTTON_DEPTH = 0.018
BUTTON_PROUD = 0.0025
BUTTON_TRAVEL = 0.0035
BUTTON_APERTURE = BUTTON_SIZE

BUTTON_LAYOUT: tuple[tuple[str, float, float], ...] = (
    ("button_left_high", 0.132, -0.176),
    ("button_mid_left", 0.166, -0.194),
    ("button_mid_right", 0.200, -0.212),
    ("button_right_low", 0.234, -0.230),
)


def _rect_profile(width: float, depth: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (cx - half_w, cy - half_d),
        (cx + half_w, cy - half_d),
        (cx + half_w, cy + half_d),
        (cx - half_w, cy + half_d),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_island_induction_cooktop", assets=ASSETS)

    stone = model.material("stone", rgba=(0.83, 0.82, 0.79, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    cooktop_black = model.material("cooktop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.34, 0.36, 0.39, 0.65))
    button_finish = model.material("button_finish", rgba=(0.24, 0.25, 0.27, 1.0))

    countertop = model.part("countertop")
    front_back_depth = (COUNTER_DEPTH - COOKTOP_DEPTH) * 0.5
    side_band_width = (COUNTER_WIDTH - COOKTOP_WIDTH) * 0.5
    countertop.visual(
        Box((COUNTER_WIDTH, front_back_depth, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -((COUNTER_DEPTH * 0.5) - (front_back_depth * 0.5)),
                COUNTER_THICKNESS * 0.5,
            )
        ),
        material=stone,
        name="counter_front_band",
    )
    countertop.visual(
        Box((COUNTER_WIDTH, front_back_depth, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (COUNTER_DEPTH * 0.5) - (front_back_depth * 0.5),
                COUNTER_THICKNESS * 0.5,
            )
        ),
        material=stone,
        name="counter_back_band",
    )
    countertop.visual(
        Box((side_band_width, COOKTOP_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                -((COOKTOP_WIDTH * 0.5) + (side_band_width * 0.5)),
                0.0,
                COUNTER_THICKNESS * 0.5,
            )
        ),
        material=stone,
        name="counter_left_band",
    )
    countertop.visual(
        Box((side_band_width, COOKTOP_DEPTH, COUNTER_THICKNESS)),
        origin=Origin(
            xyz=(
                (COOKTOP_WIDTH * 0.5) + (side_band_width * 0.5),
                0.0,
                COUNTER_THICKNESS * 0.5,
            )
        ),
        material=stone,
        name="counter_right_band",
    )

    support_z = COUNTER_THICKNESS - COOKTOP_GLASS_THICKNESS - (COOKTOP_SUPPORT_THICKNESS * 0.5)
    lip_y = (COOKTOP_DEPTH * 0.5) - (COOKTOP_SUPPORT_LIP * 0.5)
    lip_x = (COOKTOP_WIDTH * 0.5) - (COOKTOP_SUPPORT_LIP * 0.5)
    countertop.visual(
        Box((COOKTOP_WIDTH, COOKTOP_SUPPORT_LIP, COOKTOP_SUPPORT_THICKNESS)),
        origin=Origin(xyz=(0.0, -lip_y, support_z)),
        material=stone,
        name="cooktop_support_front",
    )
    countertop.visual(
        Box((COOKTOP_WIDTH, COOKTOP_SUPPORT_LIP, COOKTOP_SUPPORT_THICKNESS)),
        origin=Origin(xyz=(0.0, lip_y, support_z)),
        material=stone,
        name="cooktop_support_back",
    )
    countertop.visual(
        Box((COOKTOP_SUPPORT_LIP, COOKTOP_DEPTH, COOKTOP_SUPPORT_THICKNESS)),
        origin=Origin(xyz=(-lip_x, 0.0, support_z)),
        material=stone,
        name="cooktop_support_left",
    )
    countertop.visual(
        Box((COOKTOP_SUPPORT_LIP, COOKTOP_DEPTH, COOKTOP_SUPPORT_THICKNESS)),
        origin=Origin(xyz=(lip_x, 0.0, support_z)),
        material=stone,
        name="cooktop_support_right",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTER_WIDTH, COUNTER_DEPTH, COUNTER_THICKNESS)),
        mass=75.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_THICKNESS * 0.5)),
    )

    leg_positions = {
        "front_left_leg": (
            -(COUNTER_WIDTH * 0.5 - LEG_X_INSET),
            -(COUNTER_DEPTH * 0.5 - LEG_Y_INSET),
        ),
        "front_right_leg": (
            COUNTER_WIDTH * 0.5 - LEG_X_INSET,
            -(COUNTER_DEPTH * 0.5 - LEG_Y_INSET),
        ),
        "rear_left_leg": (
            -(COUNTER_WIDTH * 0.5 - LEG_X_INSET),
            COUNTER_DEPTH * 0.5 - LEG_Y_INSET,
        ),
        "rear_right_leg": (
            COUNTER_WIDTH * 0.5 - LEG_X_INSET,
            COUNTER_DEPTH * 0.5 - LEG_Y_INSET,
        ),
    }

    for leg_name, (leg_x, leg_y) in leg_positions.items():
        leg = model.part(leg_name)
        leg.visual(
            Box((LEG_SIZE, LEG_SIZE, LEG_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, -LEG_HEIGHT * 0.5)),
            material=brushed_steel,
            name="leg_shaft",
        )
        leg.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -LEG_HEIGHT - 0.005)),
            material=brushed_steel,
            name="leveling_foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((LEG_SIZE, LEG_SIZE, LEG_HEIGHT + 0.01)),
            mass=6.0,
            origin=Origin(xyz=(0.0, 0.0, -(LEG_HEIGHT + 0.01) * 0.5)),
        )
        model.articulation(
            f"countertop_to_{leg_name}",
            ArticulationType.FIXED,
            parent=countertop,
            child=leg,
            origin=Origin(xyz=(leg_x, leg_y, 0.0)),
        )

    cooktop_glass = model.part("cooktop_glass")
    glass_half_w = COOKTOP_WIDTH * 0.5
    glass_half_d = COOKTOP_DEPTH * 0.5
    button_openings = [
        (
            button_x - (BUTTON_APERTURE * 0.5),
            button_x + (BUTTON_APERTURE * 0.5),
            button_y - (BUTTON_APERTURE * 0.5),
            button_y + (BUTTON_APERTURE * 0.5),
        )
        for _, button_x, button_y in BUTTON_LAYOUT
    ]
    x_edges = sorted(
        {
            -glass_half_w,
            glass_half_w,
            *[edge for opening in button_openings for edge in opening[:2]],
        }
    )
    y_edges = sorted(
        {
            -glass_half_d,
            glass_half_d,
            *[edge for opening in button_openings for edge in opening[2:]],
        }
    )
    panel_index = 0
    for x0, x1 in zip(x_edges[:-1], x_edges[1:]):
        for y0, y1 in zip(y_edges[:-1], y_edges[1:]):
            center_x = (x0 + x1) * 0.5
            center_y = (y0 + y1) * 0.5
            inside_opening = any(
                opening[0] < center_x < opening[1] and opening[2] < center_y < opening[3]
                for opening in button_openings
            )
            if inside_opening:
                continue
            cooktop_glass.visual(
                Box((x1 - x0, y1 - y0, COOKTOP_GLASS_THICKNESS)),
                origin=Origin(
                    xyz=(center_x, center_y, COOKTOP_GLASS_THICKNESS * 0.5)
                ),
                material=cooktop_black,
                name=f"glass_panel_{panel_index}",
            )
            panel_index += 1

    zone_specs = (
        ("zone_front_left", -ZONE_X, -ZONE_Y),
        ("zone_front_right", ZONE_X, -ZONE_Y),
        ("zone_rear_left", -ZONE_X, ZONE_Y),
        ("zone_rear_right", ZONE_X, ZONE_Y),
    )
    for zone_name, zone_x, zone_y in zone_specs:
        cooktop_glass.visual(
            Cylinder(radius=ZONE_RADIUS, length=0.0006),
            origin=Origin(xyz=(zone_x, zone_y, COOKTOP_GLASS_THICKNESS - 0.0003)),
            material=zone_mark,
            name=zone_name,
        )
        cooktop_glass.visual(
            Cylinder(radius=0.011, length=0.0004),
            origin=Origin(xyz=(zone_x, zone_y, COOKTOP_GLASS_THICKNESS - 0.0002)),
            material=zone_mark,
            name=f"{zone_name}_center",
        )

    cooktop_glass.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, COOKTOP_GLASS_THICKNESS)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_GLASS_THICKNESS * 0.5)),
    )
    model.articulation(
        "countertop_to_cooktop_glass",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop_glass,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_THICKNESS - COOKTOP_GLASS_THICKNESS)),
    )

    for button_name, button_x, button_y in BUTTON_LAYOUT:
        button = model.part(button_name)
        button.visual(
            Box((BUTTON_SIZE, BUTTON_SIZE, BUTTON_DEPTH)),
            origin=Origin(xyz=(0.0, 0.0, -BUTTON_DEPTH * 0.5)),
            material=button_finish,
            name="button_plunger",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_SIZE, BUTTON_SIZE, BUTTON_DEPTH)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, -BUTTON_DEPTH * 0.5)),
        )
        model.articulation(
            f"cooktop_glass_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=cooktop_glass,
            child=button,
            origin=Origin(xyz=(button_x, button_y, COOKTOP_GLASS_THICKNESS + BUTTON_PROUD)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    cooktop_glass = object_model.get_part("cooktop_glass")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")

    leg_parts = (front_left_leg, front_right_leg, rear_left_leg, rear_right_leg)
    button_parts = [object_model.get_part(name) for name, _, _ in BUTTON_LAYOUT]
    button_joints = [
        object_model.get_articulation(f"cooktop_glass_to_{name}") for name, _, _ in BUTTON_LAYOUT
    ]
    fixed_joint_names = (
        "countertop_to_front_left_leg",
        "countertop_to_front_right_leg",
        "countertop_to_rear_left_leg",
        "countertop_to_rear_right_leg",
        "countertop_to_cooktop_glass",
    )
    fixed_joints = [object_model.get_articulation(name) for name in fixed_joint_names]

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

    ctx.expect_contact(cooktop_glass, countertop, name="cooktop_glass_supported_by_countertop")
    countertop_aabb = ctx.part_world_aabb(countertop)
    cooktop_aabb = ctx.part_world_aabb(cooktop_glass)
    flush_top_ok = (
        countertop_aabb is not None
        and cooktop_aabb is not None
        and abs(countertop_aabb[1][2] - cooktop_aabb[1][2]) < 1e-6
    )
    ctx.check(
        "cooktop_glass_is_flush_with_countertop_top",
        flush_top_ok,
        details="Cooktop glass should finish flush with the top surface of the island.",
    )
    for leg in leg_parts:
        ctx.expect_contact(leg, countertop, name=f"{leg.name}_attached_to_countertop")

    for zone_name in ("zone_front_left", "zone_front_right", "zone_rear_left", "zone_rear_right"):
        zone_aabb = ctx.part_element_world_aabb(cooktop_glass, elem=zone_name)
        ctx.check(
            f"{zone_name}_exists",
            zone_aabb is not None,
            details=f"Expected named zone visual {zone_name} on cooktop_glass.",
        )

    for joint in fixed_joints:
        ctx.check(
            f"{joint.name}_is_fixed",
            joint.articulation_type == ArticulationType.FIXED,
            details=f"{joint.name} should be fixed, got {joint.articulation_type!r}.",
        )

    rest_positions: list[tuple[float, float, float]] = []
    for button, joint in zip(button_parts, button_joints):
        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"{joint.name} should be prismatic, got {joint.articulation_type!r}.",
        )
        ctx.check(
            f"{joint.name}_axis_is_downward",
            tuple(joint.axis) == (0.0, 0.0, -1.0),
            details=f"{joint.name} axis should be (0, 0, -1), got {joint.axis!r}.",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_travel_limit",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.003 <= limits.upper <= 0.005,
            details=f"{joint.name} should have a short inward travel, got {limits!r}.",
        )
        ctx.expect_within(button, cooktop_glass, axes="xy", margin=0.0, name=f"{button.name}_within_glass_footprint")
        button_pos = ctx.part_world_position(button)
        button_aabb = ctx.part_world_aabb(button)
        ctx.check(
            f"{button.name}_position_available",
            button_pos is not None and button_aabb is not None and cooktop_aabb is not None,
            details=f"Expected world position and bounds for {button.name}.",
        )
        if button_pos is not None and button_aabb is not None and cooktop_aabb is not None:
            rest_positions.append(button_pos)
            rest_top_ok = (
                button_aabb[1][2] > cooktop_aabb[1][2] + 0.001
                and button_aabb[1][2] < cooktop_aabb[1][2] + 0.0035
            )
            ctx.check(
                f"{button.name}_rests_slightly_proud",
                rest_top_ok,
                details=f"{button.name} should sit slightly proud of the glass at rest.",
            )

    stair_step_ok = len(rest_positions) == 4 and all(
        rest_positions[index][0] < rest_positions[index + 1][0]
        and rest_positions[index][1] > rest_positions[index + 1][1]
        and rest_positions[index][0] > 0.0
        and rest_positions[index][1] < 0.0
        for index in range(3)
    ) and rest_positions[-1][0] > 0.0 and rest_positions[-1][1] < 0.0
    ctx.check(
        "buttons_form_descending_front_right_stair_step",
        stair_step_ok,
        details="Expected button centers to step rightward and toward the front across the front-right control area.",
    )

    for button, joint, rest_position in zip(button_parts, button_joints, rest_positions):
        limits = joint.motion_limits
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                pressed_position = ctx.part_world_position(button)
                pressed_aabb = ctx.part_world_aabb(button)
                motion_ok = (
                    pressed_position is not None
                    and pressed_aabb is not None
                    and cooktop_aabb is not None
                    and abs(pressed_position[0] - rest_position[0]) < 1e-6
                    and abs(pressed_position[1] - rest_position[1]) < 1e-6
                    and pressed_position[2] < rest_position[2] - 0.003
                    and pressed_aabb[1][2] < cooktop_aabb[1][2] - 0.0005
                )
                ctx.check(
                    f"{button.name}_presses_inward_only",
                    motion_ok,
                    details=f"{button.name} should translate inward along -z without lateral drift.",
                )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
