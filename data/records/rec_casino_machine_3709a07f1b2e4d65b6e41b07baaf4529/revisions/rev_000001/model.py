from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_top_slot_machine")

    black = model.material("black_enamel", rgba=(0.015, 0.012, 0.010, 1.0))
    deep_red = model.material("deep_red_lacquer", rgba=(0.45, 0.025, 0.018, 1.0))
    gold = model.material("brushed_gold", rgba=(0.92, 0.66, 0.20, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.08, 0.11, 0.14, 0.38))
    cream = model.material("warm_reel_ivory", rgba=(0.96, 0.90, 0.72, 1.0))
    red = model.material("reel_red", rgba=(0.86, 0.04, 0.03, 1.0))
    blue = model.material("reel_blue", rgba=(0.05, 0.22, 0.78, 1.0))
    green = model.material("reel_green", rgba=(0.05, 0.55, 0.18, 1.0))

    cabinet = model.part("cabinet")

    # Coordinate frame: +Z is up, -Y is the front face, +X is the lever flank.
    lower_w = 0.58
    lower_d = 0.50
    lower_h = 1.30
    wall = 0.04
    front_y = -lower_d / 2.0
    back_y = lower_d / 2.0
    front_panel_y = front_y + wall / 2.0

    # Narrow lower cabinet shell.  It is built from real panels so the reel
    # window and service-door opening are not hidden by a solid block.
    cabinet.visual(
        Box((wall, lower_d, lower_h)),
        origin=Origin(xyz=(-lower_w / 2.0 + wall / 2.0, 0.0, lower_h / 2.0)),
        material=black,
        name="lower_side_0",
    )
    cabinet.visual(
        Box((wall, lower_d, lower_h)),
        origin=Origin(xyz=(lower_w / 2.0 - wall / 2.0, 0.0, lower_h / 2.0)),
        material=black,
        name="lower_side_1",
    )
    cabinet.visual(
        Box((lower_w, wall, lower_h)),
        origin=Origin(xyz=(0.0, back_y - wall / 2.0, lower_h / 2.0)),
        material=black,
        name="lower_back",
    )
    cabinet.visual(
        Box((lower_w, lower_d, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=black,
        name="plinth",
    )
    cabinet.visual(
        Box((lower_w, lower_d, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, lower_h - 0.0275)),
        material=black,
        name="lower_top_deck",
    )

    # Lower front strips around the service door.
    door_w = 0.34
    door_h = 0.52
    door_bottom = 0.18
    door_left = -door_w / 2.0
    door_right = door_w / 2.0
    door_top = door_bottom + door_h
    side_strip_w = (lower_w - door_w) / 2.0 + 0.012
    cabinet.visual(
        Box((side_strip_w, wall, 0.66)),
        origin=Origin(xyz=(-lower_w / 2.0 + side_strip_w / 2.0, front_panel_y, 0.43)),
        material=deep_red,
        name="front_lower_side_0",
    )
    cabinet.visual(
        Box((side_strip_w, wall, 0.66)),
        origin=Origin(xyz=(lower_w / 2.0 - side_strip_w / 2.0, front_panel_y, 0.43)),
        material=deep_red,
        name="front_lower_side_1",
    )
    cabinet.visual(
        Box((lower_w, wall, 0.12)),
        origin=Origin(xyz=(0.0, front_panel_y, 0.10)),
        material=deep_red,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((lower_w, wall, 0.12)),
        origin=Origin(xyz=(0.0, front_panel_y, door_top + 0.06)),
        material=deep_red,
        name="front_mid_rail",
    )

    # Reel-window frame and glass in the lower cabinet.
    window_w = 0.50
    window_h = 0.28
    window_z = 1.00
    window_bottom = window_z - window_h / 2.0
    window_top = window_z + window_h / 2.0
    cabinet.visual(
        Box(((lower_w - window_w) / 2.0 + 0.012, wall, window_h + 0.08)),
        origin=Origin(
            xyz=(
                -lower_w / 2.0 + ((lower_w - window_w) / 2.0 + 0.012) / 2.0,
                front_panel_y,
                window_z,
            )
        ),
        material=deep_red,
        name="window_side_0",
    )
    cabinet.visual(
        Box(((lower_w - window_w) / 2.0 + 0.012, wall, window_h + 0.08)),
        origin=Origin(
            xyz=(
                lower_w / 2.0 - ((lower_w - window_w) / 2.0 + 0.012) / 2.0,
                front_panel_y,
                window_z,
            )
        ),
        material=deep_red,
        name="window_side_1",
    )
    cabinet.visual(
        Box((lower_w, wall, 0.085)),
        origin=Origin(xyz=(0.0, front_panel_y, window_bottom - 0.042)),
        material=deep_red,
        name="window_bottom_rail",
    )
    cabinet.visual(
        Box((lower_w, wall, 0.075)),
        origin=Origin(xyz=(0.0, front_panel_y, window_top + 0.038)),
        material=deep_red,
        name="window_top_rail",
    )
    reel_bezel = BezelGeometry(
        (window_w, window_h),
        (window_w + 0.065, window_h + 0.065),
        0.025,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.025,
        outer_corner_radius=0.045,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.003),
    )
    cabinet.visual(
        mesh_from_geometry(reel_bezel, "reel_window_bezel"),
        origin=Origin(xyz=(0.0, front_y - 0.010, window_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=gold,
        name="reel_window_bezel",
    )
    cabinet.visual(
        Box((window_w + 0.020, 0.006, window_h + 0.020)),
        origin=Origin(xyz=(0.0, front_y - 0.014, window_z)),
        material=dark_glass,
        name="reel_window_glass",
    )

    # Back-lit high-top housing, wider than the narrow lower body.
    topper_w = 0.74
    topper_d = 0.54
    topper_h = 0.82
    topper_bottom = lower_h - 0.005
    cabinet.visual(
        Box((topper_w, topper_d, topper_h)),
        origin=Origin(xyz=(0.0, 0.0, topper_bottom + topper_h / 2.0)),
        material=black,
        name="topper_housing",
    )
    cabinet.visual(
        Box((topper_w - 0.10, 0.018, 0.47)),
        origin=Origin(xyz=(0.0, -topper_d / 2.0 - 0.008, topper_bottom + 0.51)),
        material=gold,
        name="topper_marquee",
    )
    cabinet.visual(
        Box((topper_w - 0.17, 0.020, 0.31)),
        origin=Origin(xyz=(0.0, -topper_d / 2.0 - 0.020, topper_bottom + 0.51)),
        material=deep_red,
        name="topper_lit_panel",
    )
    # Stylized "777" sign made from raised bars, not texture.
    for i, x in enumerate((-0.21, 0.0, 0.21)):
        cabinet.visual(
            Box((0.115, 0.014, 0.030)),
            origin=Origin(xyz=(x, -topper_d / 2.0 - 0.034, topper_bottom + 0.61)),
            material=gold,
            name=f"seven_top_{i}",
        )
        cabinet.visual(
            Box((0.032, 0.014, 0.205)),
            origin=Origin(
                xyz=(x + 0.025, -topper_d / 2.0 - 0.034, topper_bottom + 0.515),
                rpy=(0.0, 0.0, -0.35),
            ),
            material=gold,
            name=f"seven_stem_{i}",
        )

    # Player details on the lower front.
    cabinet.visual(
        Box((0.15, 0.012, 0.045)),
        origin=Origin(xyz=(-0.14, front_y - 0.018, 0.81)),
        material=chrome,
        name="coin_slot_plate",
    )
    cabinet.visual(
        Box((0.075, 0.010, 0.012)),
        origin=Origin(xyz=(-0.14, front_y - 0.026, 0.815)),
        material=black,
        name="coin_slot",
    )
    for i, x in enumerate((0.065, 0.135, 0.205)):
        cabinet.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x, front_y - 0.018, 0.81), rpy=(pi / 2.0, 0.0, 0.0)),
            material=(red, blue, green)[i],
            name=f"play_button_{i}",
        )

    # A visible axle ties the rotating reel set back into the cabinet sidewalls.
    # It intentionally passes through the reel hubs and is allowed in tests.
    axle_y = -0.090
    axle_z = window_z
    cabinet.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="reel_axle",
    )
    cabinet.visual(
        Box((0.05, 0.045, 0.24)),
        origin=Origin(xyz=(-0.265, axle_y, axle_z)),
        material=chrome,
        name="reel_side_bracket_0",
    )
    cabinet.visual(
        Box((0.05, 0.045, 0.24)),
        origin=Origin(xyz=(0.265, axle_y, axle_z)),
        material=chrome,
        name="reel_side_bracket_1",
    )

    # Cabinet hinge leaves for the lower service door.
    for i, z in enumerate((door_bottom + 0.09, door_bottom + 0.26, door_bottom + 0.43)):
        cabinet.visual(
            Box((0.038, 0.012, 0.095)),
            origin=Origin(xyz=(door_left - 0.017, front_y - 0.005, z)),
            material=chrome,
            name=f"service_hinge_leaf_{i}",
        )

    # Rotating reel set: three independent reels, each on a horizontal X axis.
    reel_centers = (-0.17, 0.0, 0.17)
    decal_mats = (red, blue, green)
    for i, x in enumerate(reel_centers):
        reel = model.part(f"reel_{i}")
        reel.visual(
            Cylinder(radius=0.130, length=0.110),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=cream,
            name="reel_core",
        )
        reel.visual(
            mesh_from_geometry(TorusGeometry(0.128, 0.006), f"reel_{i}_rim_0_mesh"),
            origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=gold,
            name="reel_rim_0",
        )
        reel.visual(
            mesh_from_geometry(TorusGeometry(0.128, 0.006), f"reel_{i}_rim_1_mesh"),
            origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=gold,
            name="reel_rim_1",
        )
        # Raised symbol cards are embedded slightly into the reel surface so
        # they rotate with the cylinder instead of reading as separate labels.
        reel.visual(
            Box((0.080, 0.008, 0.055)),
            origin=Origin(xyz=(0.0, -0.128, 0.000)),
            material=decal_mats[i],
            name="front_symbol",
        )
        reel.visual(
            Box((0.080, 0.008, 0.045)),
            origin=Origin(xyz=(0.0, -0.106, 0.080)),
            material=gold,
            name="upper_symbol",
        )
        reel.visual(
            Box((0.080, 0.008, 0.045)),
            origin=Origin(xyz=(0.0, -0.106, -0.080)),
            material=black,
            name="lower_symbol",
        )
        model.articulation(
            f"cabinet_to_reel_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=reel,
            origin=Origin(xyz=(x, axle_y, axle_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    # Side pull lever, mounted to the cabinet's right flank.
    lever = model.part("pull_lever")
    lever.visual(
        Cylinder(radius=0.058, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_hub",
    )
    lever.visual(
        Box((0.034, 0.035, 0.390)),
        origin=Origin(xyz=(0.066, 0.0, -0.230)),
        material=chrome,
        name="lever_arm",
    )
    lever.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.066, 0.0, -0.460)),
        material=deep_red,
        name="lever_knob",
    )
    model.articulation(
        "cabinet_to_pull_lever",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lever,
        origin=Origin(xyz=(lower_w / 2.0, -0.055, 1.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.75, upper=0.90),
    )

    # Hinged service door on the lower front.  Its part frame is the bottom of
    # the vertical hinge line, so positive motion swings the free edge outward.
    service_door = model.part("service_door")
    service_door.visual(
        Box((door_w, 0.022, door_h)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h / 2.0)),
        material=deep_red,
        name="door_panel",
    )
    service_door.visual(
        Box((door_w - 0.055, 0.010, 0.040)),
        origin=Origin(xyz=(door_w / 2.0 + 0.010, -0.016, door_h - 0.075)),
        material=chrome,
        name="door_pull",
    )
    service_door.visual(
        Cylinder(radius=0.014, length=door_h + 0.025),
        origin=Origin(xyz=(0.0, 0.0, door_h / 2.0)),
        material=chrome,
        name="hinge_barrel",
    )
    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(xyz=(door_left, front_y - 0.016, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("service_door")
    lever = object_model.get_part("pull_lever")
    door_joint = object_model.get_articulation("cabinet_to_service_door")
    lever_joint = object_model.get_articulation("cabinet_to_pull_lever")

    for i in range(3):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"service_hinge_leaf_{i}",
            elem_b="hinge_barrel",
            reason="The exposed service-door hinge knuckle is intentionally captured around the hinge barrel.",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="z",
            elem_a=f"service_hinge_leaf_{i}",
            elem_b="hinge_barrel",
            min_overlap=0.06,
            name=f"service hinge leaf {i} engaged with barrel",
        )

    for i in range(3):
        reel = object_model.get_part(f"reel_{i}")
        ctx.allow_overlap(
            cabinet,
            reel,
            elem_a="reel_axle",
            elem_b="reel_core",
            reason="The fixed cabinet axle is intentionally captured through the rotating reel hub.",
        )
        ctx.expect_within(
            cabinet,
            reel,
            axes="yz",
            inner_elem="reel_axle",
            outer_elem="reel_core",
            margin=0.002,
            name=f"reel_{i} axle centered in hub",
        )
        ctx.expect_overlap(
            cabinet,
            reel,
            axes="x",
            elem_a="reel_axle",
            elem_b="reel_core",
            min_overlap=0.08,
            name=f"reel_{i} hub retained on axle",
        )
        ctx.expect_overlap(
            reel,
            cabinet,
            axes="xz",
            elem_a="reel_core",
            elem_b="reel_window_glass",
            min_overlap=0.08,
            name=f"reel_{i} visible through window",
        )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            max_gap=0.006,
            max_penetration=0.0,
            positive_elem="front_lower_side_0",
            negative_elem="door_panel",
            name="service door sits just proud of lower front",
        )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "service door swings outward from vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.08,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
    with ctx.pose({lever_joint: 0.70}):
        pulled_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
    ctx.check(
        "side pull lever rotates on flank pivot",
        closed_knob_aabb is not None
        and pulled_knob_aabb is not None
        and abs(pulled_knob_aabb[0][2] - closed_knob_aabb[0][2]) > 0.03,
        details=f"rest={closed_knob_aabb}, pulled={pulled_knob_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
