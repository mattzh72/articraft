from __future__ import annotations

from math import pi

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


BODY_W = 0.078
BODY_D = 0.040
BODY_H = 0.158

BUTTON_TRAVEL = 0.0012

BUTTON_LAYOUT = (
    ("hold_button", -0.0135, 0.103),
    ("range_button", 0.0135, 0.103),
    ("minmax_button", -0.0135, 0.092),
    ("backlight_button", 0.0135, 0.092),
)


def _add_front_button(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    x: float,
    z: float,
    material,
) -> None:
    button = model.part(name)
    button.visual(
        Box((0.015, 0.0016, 0.007)),
        origin=Origin(xyz=(0.0, 0.0008, 0.0)),
        material=material,
        name="cap",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(x, 0.0214, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_multimeter")

    bumper = model.material("bumper", rgba=(0.89, 0.60, 0.14, 1.0))
    panel = model.material("panel", rgba=(0.16, 0.17, 0.18, 1.0))
    jack_panel = model.material("jack_panel", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.25, 0.40, 0.43, 1.0))
    control = model.material("control", rgba=(0.23, 0.24, 0.25, 1.0))
    control_dark = model.material("control_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    socket = model.material("socket", rgba=(0.04, 0.04, 0.04, 1.0))
    metal = model.material("metal", rgba=(0.65, 0.67, 0.70, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
        material=bumper,
        name="body_shell",
    )
    body.visual(
        Box((0.066, 0.006, 0.142)),
        origin=Origin(xyz=(0.0, 0.0165, 0.081)),
        material=panel,
        name="front_panel",
    )
    body.visual(
        Box((0.051, 0.002, 0.031)),
        origin=Origin(xyz=(0.0, 0.0190, 0.126)),
        material=panel,
        name="display_bezel",
    )
    body.visual(
        Box((0.047, 0.0015, 0.024)),
        origin=Origin(xyz=(0.0, 0.0196, 0.126)),
        material=glass,
        name="display_glass",
    )
    body.visual(
        Box((0.060, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, 0.0174, 0.022)),
        material=jack_panel,
        name="jack_panel",
    )
    for name, x, z in BUTTON_LAYOUT:
        for side, x_sign in (("left", -1.0), ("right", 1.0)):
            body.visual(
                Box((0.0015, 0.0024, 0.0085)),
                origin=Origin(xyz=(x + x_sign * 0.00825, 0.0207, z)),
                material=panel,
                name=f"{name}_{side}_guide",
            )
    for index, x in enumerate((-0.016, 0.0, 0.016)):
        body.visual(
            Cylinder(radius=0.0042, length=0.0018),
            origin=Origin(xyz=(x, 0.0193, 0.022), rpy=(pi * 0.5, 0.0, 0.0)),
            material=socket,
            name=f"jack_{index}",
        )

    body.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(-0.023, -0.023, 0.014), rpy=(0.0, pi * 0.5, 0.0)),
        material=panel,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.023, -0.023, 0.014), rpy=(0.0, pi * 0.5, 0.0)),
        material=panel,
        name="hinge_barrel_1",
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.0215, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=control,
        name="selector_cap",
    )
    selector.visual(
        Cylinder(radius=0.0155, length=0.012),
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=control_dark,
        name="selector_hub",
    )
    selector.visual(
        Cylinder(radius=0.0036, length=0.014),
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=metal,
        name="selector_shaft",
    )
    selector.visual(
        Box((0.004, 0.0012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0104, 0.013)),
        material=bumper,
        name="pointer",
    )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, 0.020, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0),
    )

    for name, x, z in BUTTON_LAYOUT:
        _add_front_button(model, body, name=name, x=x, z=z, material=control)

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=panel,
        name="stand_barrel",
    )
    stand.visual(
        Box((0.050, 0.004, 0.070)),
        origin=Origin(xyz=(0.0, 0.001, 0.035)),
        material=control_dark,
        name="stand_panel",
    )
    stand.visual(
        Box((0.024, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0005, 0.063)),
        material=control_dark,
        name="stand_foot",
    )

    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.023, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    stand = object_model.get_part("stand")
    hold_button = object_model.get_part("hold_button")
    range_button = object_model.get_part("range_button")
    minmax_button = object_model.get_part("minmax_button")
    backlight_button = object_model.get_part("backlight_button")

    selector_joint = object_model.get_articulation("body_to_selector")
    stand_joint = object_model.get_articulation("body_to_stand")

    ctx.expect_gap(
        body,
        selector,
        axis="z",
        positive_elem="display_glass",
        negative_elem="selector_cap",
        min_gap=0.024,
        name="display sits clearly above the selector",
    )
    ctx.expect_gap(
        range_button,
        hold_button,
        axis="x",
        min_gap=0.008,
        name="upper buttons remain individually separated",
    )
    ctx.expect_gap(
        backlight_button,
        minmax_button,
        axis="x",
        min_gap=0.008,
        name="lower buttons remain individually separated",
    )
    ctx.expect_gap(
        hold_button,
        minmax_button,
        axis="z",
        min_gap=0.003,
        max_gap=0.020,
        name="button rows remain distinct",
    )
    ctx.expect_gap(
        body,
        stand,
        axis="y",
        positive_elem="body_shell",
        negative_elem="stand_panel",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="stand stores flush to the rear housing",
    )

    for part_name in ("hold_button", "range_button", "minmax_button", "backlight_button"):
        part = object_model.get_part(part_name)
        joint = object_model.get_articulation(f"body_to_{part_name}")
        limits = joint.motion_limits
        rest_pos = ctx.part_world_position(part)
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(part)
            ctx.check(
                f"{part_name} presses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] < rest_pos[1] - 0.0008,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    pointer_rest = ctx.part_element_world_aabb(selector, elem="pointer")
    with ctx.pose({selector_joint: pi * 0.5}):
        pointer_quarter = ctx.part_element_world_aabb(selector, elem="pointer")
    pointer_rest_center = _aabb_center(pointer_rest)
    pointer_quarter_center = _aabb_center(pointer_quarter)
    ctx.check(
        "selector pointer rotates around the central shaft",
        pointer_rest_center is not None
        and pointer_quarter_center is not None
        and pointer_quarter_center[0] > pointer_rest_center[0] + 0.010
        and pointer_quarter_center[2] < pointer_rest_center[2] - 0.008,
        details=f"rest={pointer_rest_center}, quarter_turn={pointer_quarter_center}",
    )

    closed_panel = ctx.part_element_world_aabb(stand, elem="stand_panel")
    with ctx.pose({stand_joint: 0.95}):
        open_panel = ctx.part_element_world_aabb(stand, elem="stand_panel")
    ctx.check(
        "rear stand flips outward from the case base",
        closed_panel is not None
        and open_panel is not None
        and open_panel[0][1] < closed_panel[0][1] - 0.030
        and open_panel[1][2] < closed_panel[1][2] - 0.010,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    return ctx.report()


object_model = build_object_model()
