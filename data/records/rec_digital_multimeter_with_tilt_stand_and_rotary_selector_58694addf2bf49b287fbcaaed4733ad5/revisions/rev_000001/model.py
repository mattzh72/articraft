from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


BODY_WIDTH = 0.083
BODY_DEPTH = 0.045
BODY_HEIGHT = 0.168
FRONT_Y = BODY_DEPTH * 0.5
BACK_Y = -FRONT_Y

DISPLAY_CENTER_Z = 0.046
SELECTOR_CENTER_Z = -0.004
KEY_ROW_Z = -0.036
JACK_PANEL_Z = -0.058
FLASHLIGHT_X = 0.028
FLASHLIGHT_Z = 0.067


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
    shell = shell.edges("|Z").fillet(0.009)

    front_pocket = (
        cq.Workplane("XY")
        .box(BODY_WIDTH - 0.010, 0.0022, BODY_HEIGHT - 0.014)
        .translate((0.0, FRONT_Y - 0.0005, 0.0))
    )
    rear_pocket = (
        cq.Workplane("XY")
        .box(0.038, 0.0022, 0.072)
        .translate((0.0, BACK_Y + 0.0005, -0.016))
    )
    return shell.cut(front_pocket).cut(rear_pocket)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_multimeter")

    housing = model.material("housing", rgba=(0.20, 0.21, 0.22, 1.0))
    bumper = model.material("bumper", rgba=(0.14, 0.15, 0.16, 1.0))
    screen = model.material("screen", rgba=(0.14, 0.27, 0.20, 0.65))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    key_rubber = model.material("key_rubber", rgba=(0.24, 0.25, 0.27, 1.0))
    flashlight = model.material("flashlight", rgba=(0.92, 0.73, 0.20, 1.0))
    jack_ring = model.material("jack_ring", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "multimeter_body_shell"),
        material=housing,
        name="shell",
    )
    body.visual(
        Box((0.054, 0.0018, 0.032)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0010, DISPLAY_CENTER_Z)),
        material=dark_trim,
        name="display_bezel",
    )
    body.visual(
        Box((0.046, 0.0014, 0.024)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0011, DISPLAY_CENTER_Z)),
        material=screen,
        name="display",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.0018),
        origin=Origin(
            xyz=(0.0, FRONT_Y - 0.0009, SELECTOR_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="dial_well",
    )
    body.visual(
        Box((0.046, 0.0018, 0.010)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0007, KEY_ROW_Z)),
        material=dark_trim,
        name="key_panel",
    )
    body.visual(
        Cylinder(radius=0.0052, length=0.0018),
        origin=Origin(
            xyz=(FLASHLIGHT_X, FRONT_Y - 0.0007, FLASHLIGHT_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="flashlight_pad",
    )
    body.visual(
        Box((0.060, 0.0018, 0.022)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0010, JACK_PANEL_Z)),
        material=bumper,
        name="jack_panel",
    )
    for jack_index, jack_x in enumerate((-0.018, 0.0, 0.018)):
        body.visual(
            Cylinder(radius=0.006, length=0.0028),
            origin=Origin(
                xyz=(jack_x, FRONT_Y - 0.0006, JACK_PANEL_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=jack_ring,
            name=f"jack_{jack_index}",
        )
    for lug_index, lug_x in enumerate((-0.020, 0.020)):
        body.visual(
            Box((0.010, 0.005, 0.010)),
            origin=Origin(xyz=(lug_x, BACK_Y - 0.0012, -0.076)),
            material=bumper,
            name=f"hinge_lug_{lug_index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.58,
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.0175, length=0.0104),
        origin=Origin(xyz=(0.0, 0.0052, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="dial",
    )
    selector.visual(
        Cylinder(radius=0.0215, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bumper,
        name="skirt",
    )
    selector.visual(
        Box((0.003, 0.0016, 0.010)),
        origin=Origin(xyz=(0.0, 0.0096, 0.0090)),
        material=flashlight,
        name="pointer",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.044, 0.012, 0.044)),
        mass=0.04,
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_Y, SELECTOR_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.18, velocity=8.0),
    )

    key_x_positions = (-0.016, 0.0, 0.016)
    for key_index, key_x in enumerate(key_x_positions):
        key = model.part(f"key_{key_index}")
        key.visual(
            Box((0.011, 0.0030, 0.006)),
            origin=Origin(),
            material=key_rubber,
            name="cap",
        )
        key.inertial = Inertial.from_geometry(Box((0.011, 0.0030, 0.006)), mass=0.006)
        model.articulation(
            f"body_to_key_{key_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(key_x, FRONT_Y + 0.0017, KEY_ROW_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0012,
            ),
        )

    flashlight_button = model.part("flashlight_button")
    flashlight_button.visual(
        Cylinder(radius=0.0045, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flashlight,
        name="cap",
    )
    flashlight_button.inertial = Inertial.from_geometry(
        Box((0.010, 0.004, 0.010)),
        mass=0.004,
    )
    model.articulation(
        "body_to_flashlight_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flashlight_button,
        origin=Origin(xyz=(FLASHLIGHT_X, FRONT_Y + 0.0018, FLASHLIGHT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0012,
        ),
    )

    stand = model.part("stand")
    stand.visual(
        Box((0.028, 0.0032, 0.072)),
        origin=Origin(xyz=(0.0, -0.0014, 0.036)),
        material=bumper,
        name="panel",
    )
    stand.visual(
        Box((0.014, 0.0060, 0.050)),
        origin=Origin(xyz=(0.0, -0.0030, 0.034)),
        material=dark_trim,
        name="rib",
    )
    stand.visual(
        Cylinder(radius=0.0034, length=0.020),
        origin=Origin(xyz=(0.0, -0.0032, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bumper,
        name="barrel",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.028, 0.008, 0.072)),
        mass=0.035,
        origin=Origin(xyz=(0.0, -0.002, 0.036)),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BACK_Y - 0.0002, -0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    flashlight_button = object_model.get_part("flashlight_button")

    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")
    flashlight_joint = object_model.get_articulation("body_to_flashlight_button")

    ctx.check(
        "selector_is_continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.motion_limits is not None
        and selector_joint.motion_limits.lower is None
        and selector_joint.motion_limits.upper is None,
        details=(
            f"type={selector_joint.articulation_type}, "
            f"limits={selector_joint.motion_limits!r}"
        ),
    )

    ctx.expect_gap(
        selector,
        body,
        axis="y",
        max_gap=0.004,
        max_penetration=0.0,
        elem_b="dial_well",
        name="selector_sits_proud_of_face",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="xz",
        min_overlap=0.028,
        name="selector_stays_centered_on_body_face",
    )

    for key_index in range(3):
        key = object_model.get_part(f"key_{key_index}")
        joint = object_model.get_articulation(f"body_to_key_{key_index}")
        limits = joint.motion_limits

        ctx.expect_gap(
            key,
            body,
            axis="y",
            max_gap=0.004,
            max_penetration=1e-6,
            elem_b="key_panel",
            name=f"key_{key_index}_rests_proud_of_face",
        )
        ctx.expect_overlap(
            key,
            body,
            axes="xz",
            min_overlap=0.006,
            name=f"key_{key_index}_sits_on_front_panel",
        )

        rest_pos = ctx.part_world_position(key)
        pressed_pos = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(key)
        ctx.check(
            f"key_{key_index}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0008,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.expect_gap(
        flashlight_button,
        body,
        axis="y",
        max_gap=0.004,
        max_penetration=1e-6,
        elem_b="flashlight_pad",
        name="flashlight_button_sits_proud_of_face",
    )
    ctx.expect_overlap(
        flashlight_button,
        body,
        axes="xz",
        min_overlap=0.007,
        name="flashlight_button_mounts_on_upper_face",
    )

    flash_rest = ctx.part_world_position(flashlight_button)
    flash_pressed = None
    flash_limits = flashlight_joint.motion_limits
    if flash_limits is not None and flash_limits.upper is not None:
        with ctx.pose({flashlight_joint: flash_limits.upper}):
            flash_pressed = ctx.part_world_position(flashlight_button)
    ctx.check(
        "flashlight_button_presses_inward",
        flash_rest is not None
        and flash_pressed is not None
        and flash_pressed[1] < flash_rest[1] - 0.0008,
        details=f"rest={flash_rest}, pressed={flash_pressed}",
    )

    ctx.expect_gap(
        body,
        stand,
        axis="y",
        max_gap=0.004,
        max_penetration=1e-6,
        positive_elem="shell",
        negative_elem="panel",
        name="stand_stows_close_to_back",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(stand, elem="panel")
    open_panel_aabb = None
    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_panel_aabb = ctx.part_element_world_aabb(stand, elem="panel")
    closed_panel_center = _aabb_center(closed_panel_aabb)
    open_panel_center = _aabb_center(open_panel_aabb)
    ctx.check(
        "stand_flips_outward",
        closed_panel_center is not None
        and open_panel_center is not None
        and open_panel_center[1] < closed_panel_center[1] - 0.020
        and open_panel_center[2] < closed_panel_center[2] - 0.010,
        details=f"closed={closed_panel_center}, open={open_panel_center}",
    )

    display_aabb = ctx.part_element_world_aabb(body, elem="display")
    selector_aabb = ctx.part_element_world_aabb(selector, elem="dial")
    flash_aabb = ctx.part_element_world_aabb(flashlight_button, elem="cap")
    display_center = _aabb_center(display_aabb)
    selector_center = _aabb_center(selector_aabb)
    flash_center = _aabb_center(flash_aabb)

    ctx.check(
        "layout_display_above_selector",
        display_center is not None
        and selector_center is not None
        and display_center[2] > selector_center[2] + 0.035,
        details=f"display={display_center}, selector={selector_center}",
    )
    ctx.check(
        "layout_flashlight_in_top_corner",
        display_center is not None
        and flash_center is not None
        and flash_center[0] > display_center[0] + 0.020
        and flash_center[2] > display_center[2] + 0.010,
        details=f"display={display_center}, flashlight={flash_center}",
    )

    return ctx.report()


object_model = build_object_model()
