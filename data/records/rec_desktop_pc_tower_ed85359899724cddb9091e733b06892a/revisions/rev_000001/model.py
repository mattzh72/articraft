from __future__ import annotations

import math

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
    model = ArticulatedObject(name="low_profile_htpc_case")

    dark_metal = model.material("dark_anodized_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    vent_black = model.material("deep_black_vents", rgba=(0.0, 0.0, 0.0, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    hinge_steel = model.material("stainless_hinge_steel", rgba=(0.74, 0.72, 0.68, 1.0))
    lens_smoke = model.material("smoked_display_lens", rgba=(0.02, 0.08, 0.10, 0.85))
    button_blue = model.material("blue_power_button", rgba=(0.05, 0.23, 0.95, 1.0))

    case = model.part("case")

    # A real HTPC chassis is broad and shallow: about a 430 mm component width,
    # AV rack depth, and under-90 mm height.  The root part is a connected tray
    # and front bezel frame; the top center is left as an access opening.
    case.visual(
        Box((0.460, 0.350, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_metal,
        name="bottom_tray",
    )
    case.visual(
        Box((0.012, 0.350, 0.080)),
        origin=Origin(xyz=(-0.224, 0.0, 0.045)),
        material=dark_metal,
        name="side_wall_0",
    )
    case.visual(
        Box((0.012, 0.350, 0.080)),
        origin=Origin(xyz=(0.224, 0.0, 0.045)),
        material=dark_metal,
        name="side_wall_1",
    )
    case.visual(
        Box((0.460, 0.012, 0.080)),
        origin=Origin(xyz=(0.0, 0.169, 0.045)),
        material=dark_metal,
        name="rear_wall",
    )
    case.visual(
        Box((0.460, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.170, 0.019)),
        material=satin_black,
        name="front_lower_rail",
    )
    case.visual(
        Box((0.460, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.170, 0.077)),
        material=satin_black,
        name="front_upper_rail",
    )
    case.visual(
        Box((0.014, 0.010, 0.060)),
        origin=Origin(xyz=(-0.188, -0.170, 0.050)),
        material=satin_black,
        name="door_hinge_post",
    )
    case.visual(
        Box((0.014, 0.010, 0.060)),
        origin=Origin(xyz=(0.075, -0.170, 0.050)),
        material=satin_black,
        name="door_latch_post",
    )
    case.visual(
        Box((0.086, 0.010, 0.058)),
        origin=Origin(xyz=(0.128, -0.170, 0.050)),
        material=satin_black,
        name="front_control_backing",
    )
    case.visual(
        Box((0.044, 0.010, 0.018)),
        origin=Origin(xyz=(0.193, -0.170, 0.030)),
        material=satin_black,
        name="button_lower_surround",
    )
    case.visual(
        Box((0.044, 0.010, 0.014)),
        origin=Origin(xyz=(0.193, -0.170, 0.072)),
        material=satin_black,
        name="button_upper_surround",
    )
    case.visual(
        Box((0.010, 0.010, 0.058)),
        origin=Origin(xyz=(0.220, -0.170, 0.050)),
        material=satin_black,
        name="button_side_surround",
    )
    case.visual(
        Box((0.245, 0.003, 0.053)),
        origin=Origin(xyz=(-0.050, -0.164, 0.050)),
        material=vent_black,
        name="drive_bay_shadow",
    )
    case.visual(
        Box((0.072, 0.002, 0.018)),
        origin=Origin(xyz=(0.132, -0.176, 0.050)),
        material=lens_smoke,
        name="display_lens",
    )

    # Side vent openings are represented by dark recessed slits on both side
    # walls, not as floating decals: each is infinitesimally embedded into the
    # metal wall surface.
    for side_index, x in enumerate((-0.231, 0.231)):
        for slot_index, y in enumerate((-0.100, -0.070, -0.040, -0.010, 0.020, 0.050, 0.080)):
            case.visual(
                Box((0.002, 0.018, 0.030)),
                origin=Origin(xyz=(x, y, 0.053)),
                material=vent_black,
                name=f"side_vent_{side_index}_{slot_index}",
            )

    # Two exposed rear hinge stations: each has two fixed outer knuckles and
    # small rear-wall leaves.  The moving center knuckles live on the top panel.
    for hinge_index, center_x in enumerate((-0.130, 0.130)):
        for knuckle_index, offset_x in enumerate((-0.020, 0.020)):
            x = center_x + offset_x
            case.visual(
                Cylinder(radius=0.006, length=0.014),
                origin=Origin(xyz=(x, 0.160, 0.091), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=hinge_steel,
                name=f"top_fixed_knuckle_{hinge_index}_{knuckle_index}",
            )
            case.visual(
                Box((0.014, 0.004, 0.024)),
                origin=Origin(xyz=(x, 0.163, 0.077)),
                material=hinge_steel,
                name=f"top_hinge_leaf_{hinge_index}_{knuckle_index}",
            )

    # Front bezel door fixed hinge knuckles.  They are outside the door panel
    # edge, alternating with the moving center knuckle on the door.
    for knuckle_index, z in enumerate((0.032, 0.068)):
        case.visual(
            Cylinder(radius=0.0055, length=0.018),
            origin=Origin(xyz=(-0.175, -0.181, z)),
            material=hinge_steel,
            name=f"door_fixed_knuckle_{knuckle_index}",
        )
        case.visual(
            Box((0.012, 0.004, 0.018)),
            origin=Origin(xyz=(-0.175, -0.174, z)),
            material=hinge_steel,
            name=f"door_hinge_leaf_{knuckle_index}",
        )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((0.420, 0.305, 0.006)),
        # Local y=0 is the rear hinge line; the closed access panel extends
        # forward along -Y and sits just above the case rim.
        origin=Origin(xyz=(0.0, -0.1575, 0.0)),
        material=brushed_aluminum,
        name="top_skin",
    )
    top_panel.visual(
        Box((0.390, 0.008, 0.002)),
        origin=Origin(xyz=(0.0, -0.303, 0.0038)),
        material=satin_black,
        name="front_reveal_line",
    )
    for slot_index, x in enumerate((-0.120, -0.090, -0.060, -0.030, 0.030, 0.060, 0.090, 0.120)):
        top_panel.visual(
            Box((0.012, 0.090, 0.0012)),
            origin=Origin(xyz=(x, -0.155, 0.0030)),
            material=vent_black,
            name=f"top_vent_{slot_index}",
        )
    for hinge_index, center_x in enumerate((-0.130, 0.130)):
        top_panel.visual(
            Box((0.058, 0.044, 0.003)),
            origin=Origin(xyz=(center_x, -0.024, 0.0055)),
            material=hinge_steel,
            name=f"top_moving_leaf_{hinge_index}",
        )
        top_panel.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"top_moving_knuckle_{hinge_index}",
        )

    front_door = model.part("front_door")
    front_door.visual(
        Box((0.235, 0.010, 0.058)),
        # The slab begins to the right of the hinge pin, so it can swing without
        # passing through the fixed front hinge knuckles.
        origin=Origin(xyz=(0.1255, 0.0, 0.0)),
        material=satin_black,
        name="door_slab",
    )
    front_door.visual(
        Box((0.180, 0.002, 0.005)),
        origin=Origin(xyz=(0.133, -0.006, -0.018)),
        material=brushed_aluminum,
        name="door_pull_groove",
    )
    front_door.visual(
        Box((0.040, 0.003, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=hinge_steel,
        name="door_moving_leaf",
    )
    front_door.visual(
        Cylinder(radius=0.0055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="door_moving_knuckle",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.013, length=0.007),
        origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_blue,
        name="button_cap",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=top_panel,
        origin=Origin(xyz=(0.0, 0.160, 0.091)),
        # The panel extends along local -Y from the rear hinge line; rotation
        # about -X lifts its free front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=front_door,
        origin=Origin(xyz=(-0.175, -0.181, 0.050)),
        # Negative Z makes positive motion swing the closed front door outward
        # toward the viewer/front (-Y), like a left-edge equipment bezel door.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "power_button_press",
        ArticulationType.PRISMATIC,
        parent=case,
        child=power_button,
        origin=Origin(xyz=(0.195, -0.175, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.004),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    top_panel = object_model.get_part("top_panel")
    front_door = object_model.get_part("front_door")
    power_button = object_model.get_part("power_button")
    top_hinge = object_model.get_articulation("top_hinge")
    door_hinge = object_model.get_articulation("front_door_hinge")
    button_press = object_model.get_articulation("power_button_press")

    case_aabb = ctx.part_world_aabb(case)
    if case_aabb is not None:
        (mn, mx) = case_aabb
        width = mx[0] - mn[0]
        depth = mx[1] - mn[1]
        height = mx[2] - mn[2]
        ctx.check(
            "case is wide and low profile",
            width > 0.42 and depth > 0.30 and height < 0.11,
            details=f"width={width:.3f}, depth={depth:.3f}, height={height:.3f}",
        )

    top_hinge_names = {visual.name for visual in case.visuals if visual.name and visual.name.startswith("top_fixed_knuckle")}
    ctx.check(
        "two rear hinge stations are visible",
        len(top_hinge_names) == 4,
        details=f"fixed rear hinge knuckles={sorted(top_hinge_names)}",
    )

    ctx.expect_gap(
        top_panel,
        case,
        axis="z",
        positive_elem="top_skin",
        negative_elem="side_wall_0",
        min_gap=0.001,
        max_gap=0.006,
        name="closed top access panel sits above chassis rim",
    )
    ctx.expect_overlap(
        top_panel,
        case,
        axes="xy",
        elem_a="top_skin",
        elem_b="bottom_tray",
        min_overlap=0.25,
        name="top access panel covers broad top opening",
    )
    ctx.expect_gap(
        case,
        front_door,
        axis="y",
        positive_elem="front_control_backing",
        negative_elem="door_slab",
        min_gap=0.0005,
        max_gap=0.006,
        name="front bezel door closes just proud of front bezel",
    )

    closed_top_aabb = ctx.part_element_world_aabb(top_panel, elem="top_skin")
    with ctx.pose({top_hinge: 1.25}):
        open_top_aabb = ctx.part_element_world_aabb(top_panel, elem="top_skin")
    if closed_top_aabb is not None and open_top_aabb is not None:
        ctx.check(
            "top panel opens upward on rear hinge",
            open_top_aabb[1][2] > closed_top_aabb[1][2] + 0.20,
            details=f"closed_max_z={closed_top_aabb[1][2]:.3f}, open_max_z={open_top_aabb[1][2]:.3f}",
        )

    closed_door_aabb = ctx.part_world_aabb(front_door)
    with ctx.pose({door_hinge: 1.75}):
        open_door_aabb = ctx.part_world_aabb(front_door)
    if closed_door_aabb is not None and open_door_aabb is not None:
        ctx.check(
            "front bezel door swings outward from left edge",
            open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
            details=f"closed_min_y={closed_door_aabb[0][1]:.3f}, open_min_y={open_door_aabb[0][1]:.3f}",
        )

    button_rest = ctx.part_world_position(power_button)
    with ctx.pose({button_press: 0.004}):
        button_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button presses inward",
        button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
