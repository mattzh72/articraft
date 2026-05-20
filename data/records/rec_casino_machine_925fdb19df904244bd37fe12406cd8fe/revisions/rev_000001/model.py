from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_video_slot_machine")

    cabinet_red = model.material("cabinet_red", rgba=(0.55, 0.035, 0.035, 1.0))
    black = model.material("black_powdercoat", rgba=(0.015, 0.014, 0.014, 1.0))
    dark_glass = model.material("dark_smoked_glass", rgba=(0.02, 0.04, 0.07, 0.82))
    blue_glass = model.material("lit_video_screen", rgba=(0.05, 0.20, 0.44, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.76, 0.72, 0.63, 1.0))
    gold = model.material("gold_trim", rgba=(0.95, 0.67, 0.20, 1.0))
    button_blue = model.material("button_blue", rgba=(0.0, 0.22, 0.70, 1.0))
    button_red = model.material("button_red", rgba=(0.86, 0.03, 0.02, 1.0))
    white = model.material("button_white", rgba=(0.95, 0.93, 0.84, 1.0))

    cabinet = model.part("cabinet")

    # Casino-floor scale: a broad, waist-high belly cabinet with the display
    # pitched back toward the player.
    cabinet.visual(
        Box((0.86, 0.66, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=black,
        name="plinth",
    )
    cabinet.visual(
        Box((0.82, 0.60, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=cabinet_red,
        name="lower_body",
    )
    cabinet.visual(
        Box((0.64, 0.008, 0.46)),
        origin=Origin(xyz=(0.0, -0.294, 0.49)),
        material=black,
        name="belly_recess",
    )
    cabinet.visual(
        Box((0.80, 0.29, 0.055)),
        origin=Origin(xyz=(0.0, -0.265, 0.875)),
        material=black,
        name="button_deck",
    )
    cabinet.visual(
        Box((0.82, 0.22, 0.22)),
        origin=Origin(xyz=(0.0, 0.150, 0.930)),
        material=cabinet_red,
        name="rear_riser",
    )
    cabinet.visual(
        Box((0.86, 0.34, 0.43)),
        origin=Origin(xyz=(0.0, -0.060, 1.205), rpy=(math.radians(-20.0), 0.0, 0.0)),
        material=cabinet_red,
        name="slant_shell",
    )

    # Slanted video play face and raised trim.  Positions are derived in the
    # same tilted local frame so the face reads as one angled panel.
    display_roll = math.radians(-20.0)
    display_center = (0.0, -0.238, 1.180)

    def display_xyz(dx: float, dy: float, dz: float) -> tuple[float, float, float]:
        ca = math.cos(display_roll)
        sa = math.sin(display_roll)
        return (
            display_center[0] + dx,
            display_center[1] + dy * ca - dz * sa,
            display_center[2] + dy * sa + dz * ca,
        )

    cabinet.visual(
        Box((0.79, 0.030, 0.47)),
        origin=Origin(xyz=display_center, rpy=(display_roll, 0.0, 0.0)),
        material=dark_glass,
        name="display_face",
    )
    cabinet.visual(
        Box((0.58, 0.035, 0.30)),
        origin=Origin(xyz=display_xyz(0.0, -0.020, 0.025), rpy=(display_roll, 0.0, 0.0)),
        material=blue_glass,
        name="video_screen",
    )
    cabinet.visual(
        Box((0.78, 0.050, 0.045)),
        origin=Origin(xyz=display_xyz(0.0, -0.030, 0.235), rpy=(display_roll, 0.0, 0.0)),
        material=gold,
        name="top_bezel",
    )
    cabinet.visual(
        Box((0.78, 0.050, 0.045)),
        origin=Origin(xyz=display_xyz(0.0, -0.030, -0.235), rpy=(display_roll, 0.0, 0.0)),
        material=gold,
        name="bottom_bezel",
    )
    cabinet.visual(
        Box((0.045, 0.050, 0.49)),
        origin=Origin(xyz=display_xyz(-0.390, -0.030, 0.0), rpy=(display_roll, 0.0, 0.0)),
        material=gold,
        name="side_bezel_0",
    )
    cabinet.visual(
        Box((0.045, 0.050, 0.49)),
        origin=Origin(xyz=display_xyz(0.390, -0.030, 0.0), rpy=(display_roll, 0.0, 0.0)),
        material=gold,
        name="side_bezel_1",
    )
    cabinet.visual(
        Box((0.78, 0.16, 0.11)),
        origin=Origin(xyz=(0.0, 0.070, 1.445)),
        material=black,
        name="top_marquee",
    )
    cabinet.visual(
        Box((0.60, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, 0.008, 1.485)),
        material=blue_glass,
        name="marquee_glass",
    )
    cabinet.visual(
        Box((0.45, 0.020, 0.045)),
        origin=Origin(xyz=(0.0, -0.310, 0.765)),
        material=chrome,
        name="ticket_trim",
    )

    # Lockable belly door below the play area.  The child frame is the hinge
    # line on the front-left edge, so positive motion opens it outward.
    belly_door = model.part("belly_door")
    belly_door.visual(
        Box((0.62, 0.034, 0.48)),
        origin=Origin(xyz=(0.310, -0.018, 0.0)),
        material=cabinet_red,
        name="door_panel",
    )
    belly_door.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(-0.004, -0.021, -0.170)),
        material=chrome,
        name="hinge_barrel_0",
    )
    belly_door.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(-0.004, -0.021, 0.000)),
        material=chrome,
        name="hinge_barrel_1",
    )
    belly_door.visual(
        Cylinder(radius=0.018, length=0.13),
        origin=Origin(xyz=(-0.004, -0.021, 0.170)),
        material=chrome,
        name="hinge_barrel_2",
    )
    belly_door.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(0.480, -0.040, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lock_cylinder",
    )
    belly_door.visual(
        Box((0.040, 0.006, 0.010)),
        origin=Origin(xyz=(0.480, -0.056, 0.055)),
        material=black,
        name="key_slot",
    )

    door_hinge = model.articulation(
        "cabinet_to_belly_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=belly_door,
        origin=Origin(xyz=(-0.360, -0.299, 0.490)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    door_hinge.meta["purpose"] = "vertical side hinge for service belly door"

    # Five separate player buttons on the shallow deck.  They depress downward
    # just a few millimeters like casino button caps.
    button_positions = [-0.285, -0.145, 0.0, 0.145, 0.285]
    button_materials = [button_blue, button_blue, button_red, button_blue, white]
    for idx, (x_pos, mat) in enumerate(zip(button_positions, button_materials)):
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(radius=0.043, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=black,
            name="button_shadow",
        )
        model.articulation(
            f"cabinet_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, -0.300, 0.9014)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.18, lower=0.0, upper=0.008),
        )

    # A side-mounted pull handle.  The chrome bracket is a separate fixed part
    # with visible stand-offs so it does not read as blended into the cabinet.
    handle_bracket = model.part("handle_bracket")
    handle_bracket.visual(
        Box((0.025, 0.22, 0.22)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=chrome,
        name="backplate",
    )
    handle_bracket.visual(
        Cylinder(radius=0.014, length=0.039),
        origin=Origin(xyz=(-0.070, -0.070, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="standoff_0",
    )
    handle_bracket.visual(
        Cylinder(radius=0.014, length=0.039),
        origin=Origin(xyz=(-0.070, 0.070, -0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="standoff_1",
    )
    handle_bracket.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_hub",
    )
    model.articulation(
        "cabinet_to_handle_bracket",
        ArticulationType.FIXED,
        parent=cabinet,
        child=handle_bracket,
        origin=Origin(xyz=(0.4995, -0.080, 0.800)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.024, length=0.150),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_shaft",
    )
    handle.visual(
        Cylinder(radius=0.017, length=0.335),
        origin=Origin(xyz=(0.105, 0.0, -0.180), rpy=(math.pi, 0.0, 0.0)),
        material=chrome,
        name="lever_rod",
    )
    handle.visual(
        Sphere(radius=0.048),
        origin=Origin(xyz=(0.105, 0.0, -0.365)),
        material=button_red,
        name="handle_knob",
    )
    model.articulation(
        "bracket_to_handle",
        ArticulationType.REVOLUTE,
        parent=handle_bracket,
        child=handle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.90, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    belly_door = object_model.get_part("belly_door")
    bracket = object_model.get_part("handle_bracket")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("cabinet_to_belly_door")
    handle_joint = object_model.get_articulation("bracket_to_handle")

    ctx.allow_overlap(
        handle,
        bracket,
        elem_a="pivot_shaft",
        elem_b="pivot_hub",
        reason="The handle shaft is intentionally captured inside the side bracket hub.",
    )
    ctx.expect_within(
        handle,
        bracket,
        axes="yz",
        inner_elem="pivot_shaft",
        outer_elem="pivot_hub",
        margin=0.0,
        name="handle shaft is centered inside bracket hub",
    )
    ctx.expect_overlap(
        handle,
        bracket,
        axes="x",
        elem_a="pivot_shaft",
        elem_b="pivot_hub",
        min_overlap=0.045,
        name="handle shaft has retained insertion through hub",
    )

    ctx.expect_gap(
        bracket,
        cabinet,
        axis="x",
        positive_elem="backplate",
        negative_elem="lower_body",
        min_gap=0.025,
        name="handle bracket backplate stands proud of side wall",
    )
    ctx.expect_gap(
        bracket,
        cabinet,
        axis="x",
        positive_elem="standoff_0",
        negative_elem="lower_body",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper bracket standoff seats on side wall",
    )

    ctx.expect_gap(
        cabinet,
        belly_door,
        axis="y",
        positive_elem="lower_body",
        negative_elem="door_panel",
        max_gap=0.004,
        max_penetration=0.0,
        name="belly door closes against front cabinet face",
    )
    ctx.expect_overlap(
        belly_door,
        cabinet,
        axes="z",
        elem_a="door_panel",
        elem_b="belly_recess",
        min_overlap=0.20,
        name="belly door covers the service opening height",
    )

    closed_door_aabb = ctx.part_world_aabb(belly_door)
    with ctx.pose({door_hinge: 1.10}):
        open_door_aabb = ctx.part_world_aabb(belly_door)
    ctx.check(
        "belly door swings outward from front",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: -0.75}):
        pulled_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "pull handle rotates about side pivot",
        rest_handle_aabb is not None
        and pulled_handle_aabb is not None
        and pulled_handle_aabb[0][1] < rest_handle_aabb[0][1] - 0.05,
        details=f"rest={rest_handle_aabb}, pulled={pulled_handle_aabb}",
    )

    button_joint = object_model.get_articulation("cabinet_to_button_2")
    button = object_model.get_part("button_2")
    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.008}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "center spin button depresses into shallow deck",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.006,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
