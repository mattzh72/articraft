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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_D = 0.38
BODY_W = 0.48
BODY_H = 0.29
WALL = 0.012
FRONT_T = 0.008
FRONT_FACE_X = BODY_D / 2.0
FRONT_PANEL_X = FRONT_FACE_X - FRONT_T / 2.0

MAIN_SECTION_W = 0.372
CONTROL_W = BODY_W - MAIN_SECTION_W
CONTROL_Y_MIN = BODY_W / 2.0 - CONTROL_W
CONTROL_Y_MAX = BODY_W / 2.0
CONTROL_Y_CENTER = (CONTROL_Y_MIN + CONTROL_Y_MAX) / 2.0

DOOR_T = 0.028
DOOR_W = 0.352
DOOR_H = 0.226
DOOR_HINGE_Y = -0.226
DOOR_BOTTOM_Z = 0.024
DOOR_INNER_FACE_X = FRONT_FACE_X

COVER_T = 0.008
COVER_W = 0.096
COVER_H = 0.094
COVER_HINGE_Y = 0.238
COVER_BOTTOM_Z = 0.144
COVER_INNER_FACE_X = FRONT_FACE_X

BUTTON_DEPTH = 0.007
BUTTON_WIDTH = 0.070
BUTTON_HEIGHT = 0.018
BUTTON_JOINT_X = FRONT_FACE_X
BUTTON_TRAVEL = 0.0025
BUTTON_Y = CONTROL_Y_CENTER
BUTTON_ZS = (0.111, 0.084, 0.057)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_microwave")

    stainless = model.material("stainless", rgba=(0.70, 0.71, 0.73, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.10, 0.10, 0.11, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.12, 0.14, 0.16, 0.55))
    button_gray = model.material("button_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    indicator = model.material("indicator", rgba=(0.45, 0.63, 0.78, 1.0))

    body = model.part("body")

    # Outer shell.
    body.visual(
        Box((BODY_D, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + WALL / 2.0, BODY_H / 2.0)),
        material=stainless,
        name="shell_left",
    )
    body.visual(
        Box((BODY_D, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - WALL / 2.0, BODY_H / 2.0)),
        material=stainless,
        name="shell_right",
    )
    body.visual(
        Box((BODY_D, BODY_W - 2.0 * WALL, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=stainless,
        name="shell_floor",
    )
    body.visual(
        Box((BODY_D, BODY_W - 2.0 * WALL, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - WALL / 2.0)),
        material=stainless,
        name="shell_roof",
    )
    body.visual(
        Box((WALL, BODY_W - 2.0 * WALL, BODY_H - 2.0 * WALL)),
        origin=Origin(xyz=(-BODY_D / 2.0 + WALL / 2.0, 0.0, BODY_H / 2.0)),
        material=stainless,
        name="shell_back",
    )

    # Main front surround around the oven cavity.
    body.visual(
        Box((FRONT_T, MAIN_SECTION_W, 0.028)),
        origin=Origin(xyz=(FRONT_PANEL_X, -0.054, 0.014)),
        material=graphite,
        name="front_bottom_rail",
    )
    body.visual(
        Box((FRONT_T, MAIN_SECTION_W, 0.034)),
        origin=Origin(xyz=(FRONT_PANEL_X, -0.054, BODY_H - WALL - 0.017)),
        material=graphite,
        name="front_top_rail",
    )
    body.visual(
        Box((FRONT_T, 0.020, 0.216)),
        origin=Origin(xyz=(FRONT_PANEL_X, -0.230, 0.136)),
        material=graphite,
        name="front_left_jamb",
    )
    body.visual(
        Box((FRONT_T, 0.012, 0.216)),
        origin=Origin(xyz=(FRONT_PANEL_X, 0.126, 0.136)),
        material=graphite,
        name="front_center_post",
    )

    # Control-side fascia with an upper hidden-settings bay.
    body.visual(
        Box((FRONT_T, CONTROL_W, 0.126)),
        origin=Origin(xyz=(FRONT_PANEL_X, CONTROL_Y_CENTER, 0.075)),
        material=graphite,
        name="control_lower_fascia",
    )
    body.visual(
        Box((FRONT_T, CONTROL_W, 0.040)),
        origin=Origin(xyz=(FRONT_PANEL_X, CONTROL_Y_CENTER, 0.258)),
        material=graphite,
        name="control_upper_cap",
    )
    body.visual(
        Box((FRONT_T, CONTROL_W, 0.010)),
        origin=Origin(xyz=(FRONT_PANEL_X, CONTROL_Y_CENTER, 0.143)),
        material=graphite,
        name="control_compartment_sill",
    )
    body.visual(
        Box((FRONT_T, 0.010, 0.090)),
        origin=Origin(xyz=(FRONT_PANEL_X, 0.137, 0.193)),
        material=graphite,
        name="control_compartment_left_stile",
    )
    body.visual(
        Box((FRONT_T, 0.010, 0.090)),
        origin=Origin(xyz=(FRONT_PANEL_X, 0.235, 0.193)),
        material=graphite,
        name="control_compartment_right_stile",
    )
    body.visual(
        Box((0.004, 0.084, 0.078)),
        origin=Origin(xyz=(FRONT_FACE_X - 0.030, CONTROL_Y_CENTER, 0.193)),
        material=charcoal,
        name="secondary_settings_backplate",
    )

    # Secondary controls behind the cover.
    body.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(FRONT_FACE_X - 0.023, 0.166, 0.208),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=button_gray,
        name="secondary_knob_upper",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(FRONT_FACE_X - 0.023, 0.206, 0.182),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=button_gray,
        name="secondary_knob_lower",
    )
    body.visual(
        Box((0.010, 0.020, 0.010)),
        origin=Origin(xyz=(FRONT_FACE_X - 0.023, 0.190, 0.158)),
        material=indicator,
        name="secondary_toggle",
    )

    # Interior turntable for visual credibility.
    body.visual(
        Cylinder(radius=0.115, length=0.006),
        origin=Origin(xyz=(0.010, -0.050, 0.015)),
        material=smoked_glass,
        name="turntable",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_T, 0.026, DOOR_H)),
        origin=Origin(xyz=(DOOR_T / 2.0, 0.013, DOOR_H / 2.0)),
        material=graphite,
        name="door_hinge_stile",
    )
    door.visual(
        Box((DOOR_T, 0.026, DOOR_H)),
        origin=Origin(xyz=(DOOR_T / 2.0, DOOR_W - 0.013, DOOR_H / 2.0)),
        material=graphite,
        name="door_free_stile",
    )
    door.visual(
        Box((DOOR_T, DOOR_W, 0.026)),
        origin=Origin(xyz=(DOOR_T / 2.0, DOOR_W / 2.0, DOOR_H - 0.013)),
        material=graphite,
        name="door_top_rail",
    )
    door.visual(
        Box((DOOR_T, DOOR_W, 0.032)),
        origin=Origin(xyz=(DOOR_T / 2.0, DOOR_W / 2.0, 0.016)),
        material=graphite,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.004, DOOR_W - 0.048, DOOR_H - 0.050)),
        origin=Origin(xyz=(0.010, DOOR_W / 2.0, DOOR_H / 2.0)),
        material=smoked_glass,
        name="door_window",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.130),
        origin=Origin(xyz=(DOOR_T + 0.026, DOOR_W - 0.032, 0.128)),
        material=stainless,
        name="door_handle_bar",
    )
    door.visual(
        Box((0.024, 0.014, 0.014)),
        origin=Origin(xyz=(DOOR_T + 0.012, DOOR_W - 0.032, 0.170)),
        material=stainless,
        name="door_handle_upper_mount",
    )
    door.visual(
        Box((0.024, 0.014, 0.014)),
        origin=Origin(xyz=(DOOR_T + 0.012, DOOR_W - 0.032, 0.086)),
        material=stainless,
        name="door_handle_lower_mount",
    )

    cover = model.part("control_cover")
    cover.visual(
        Box((COVER_T, COVER_W, COVER_H)),
        origin=Origin(xyz=(COVER_T / 2.0, -COVER_W / 2.0, COVER_H / 2.0)),
        material=graphite,
        name="cover_shell",
    )
    cover.visual(
        Box((0.003, COVER_W - 0.020, COVER_H - 0.030)),
        origin=Origin(xyz=(COVER_T / 2.0 + 0.001, -COVER_W / 2.0, COVER_H / 2.0)),
        material=smoked_glass,
        name="cover_window",
    )
    cover.visual(
        Box((0.010, 0.018, 0.028)),
        origin=Origin(xyz=(COVER_T + 0.005, -COVER_W + 0.010, COVER_H / 2.0)),
        material=stainless,
        name="cover_pull_tab",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_INNER_FACE_X, DOOR_HINGE_Y, DOOR_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.92,
        ),
    )
    model.articulation(
        "body_to_control_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(COVER_INNER_FACE_X, COVER_HINGE_Y, COVER_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    for index, button_z in enumerate(BUTTON_ZS, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_DEPTH, BUTTON_WIDTH, BUTTON_HEIGHT)),
            origin=Origin(xyz=(BUTTON_DEPTH / 2.0, 0.0, BUTTON_HEIGHT / 2.0)),
            material=button_gray if index < 3 else indicator,
            name="button_cap",
        )
        button.visual(
            Box((0.0015, BUTTON_WIDTH - 0.014, BUTTON_HEIGHT - 0.006)),
            origin=Origin(
                xyz=(BUTTON_DEPTH - 0.00075, 0.0, BUTTON_HEIGHT / 2.0),
            ),
            material=soft_black,
            name="button_face_shadow",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BUTTON_JOINT_X, BUTTON_Y, button_z - BUTTON_HEIGHT / 2.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    cover = object_model.get_part("control_cover")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 4)]

    door_hinge = object_model.get_articulation("body_to_door")
    cover_hinge = object_model.get_articulation("body_to_control_cover")
    button_joints = [
        object_model.get_articulation(f"body_to_button_{index}")
        for index in range(1, 4)
    ]

    ctx.check(
        "door hinge uses vertical opening axis",
        tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "control cover uses its own vertical hinge axis",
        tuple(cover_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={cover_hinge.axis}",
    )
    for index, joint in enumerate(button_joints, start=1):
        ctx.check(
            f"button {index} plunges inward",
            tuple(joint.axis) == (-1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    with ctx.pose({door_hinge: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            min_gap=-0.0001,
            max_gap=0.0001,
            name="door closes against the body front",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.20,
            name="door aligns over the oven front opening",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            min_gap=-0.0001,
            max_gap=0.0001,
            name="cover panel closes flush over hidden settings",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="yz",
            min_overlap=0.08,
            name="cover panel stays on the control strip footprint",
        )
        for index, button in enumerate(buttons, start=1):
            ctx.expect_gap(
                button,
                body,
                axis="x",
                min_gap=-0.0001,
                max_gap=0.0001,
                name=f"button {index} rests against the control face",
            )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.30}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "main door swings outward from the front face",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.00}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "secondary cover panel pops outward on its own hinge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][0] > closed_cover_aabb[1][0] + 0.05,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    for index, (button, joint) in enumerate(zip(buttons, button_joints, strict=True), start=1):
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index} retracts on a short plunger stroke",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] < rest_pos[0] - 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
