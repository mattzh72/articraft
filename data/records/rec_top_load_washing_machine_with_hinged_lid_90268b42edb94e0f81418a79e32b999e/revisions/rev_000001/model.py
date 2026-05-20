from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.69
BODY_D = 0.72
BODY_H = 0.94
CONSOLE_H = 0.14
DECK_Z = BODY_H
OPENING_Y = -0.03
BODY_CAVITY_R = 0.255
TUB_OUTER_R = 0.225
TUB_INNER_R = 0.192
TUB_H = 0.58
TUB_BOTTOM_Z = 0.33
CONSOLE_FRONT_Y = 0.20
LID_W = 0.57
LID_D = 0.55
LID_TH = 0.024
LID_HINGE_Y = 0.19
LID_HINGE_Z = DECK_Z


def _centered_box(sx: float, sy: float, sz: float, *, xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate(xyz)


def _build_body_shell() -> cq.Workplane:
    shell = _centered_box(BODY_W, BODY_D, BODY_H, xyz=(0.0, 0.0, BODY_H * 0.5))
    shell = shell.edges("|Z").fillet(0.024)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.08)
        .center(0.0, OPENING_Y)
        .circle(BODY_CAVITY_R)
        .extrude(BODY_H - 0.08 + 0.002)
    )
    return shell.cut(cavity)


def _build_console_shell() -> cq.Workplane:
    console = _centered_box(
        BODY_W - 0.04,
        0.16,
        CONSOLE_H,
        xyz=(0.0, 0.28, DECK_Z + CONSOLE_H * 0.5),
    )
    return console.edges("|Z").fillet(0.012)


def _build_lid_frame() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .center(0.0, -LID_D * 0.5)
        .rect(LID_W, LID_D)
        .extrude(LID_TH)
    )
    window = (
        cq.Workplane("XY")
        .workplane(offset=-0.002)
        .center(0.0, -0.29)
        .rect(0.45, 0.43)
        .extrude(LID_TH + 0.004)
    )
    frame = frame.cut(window)
    rear_bar = (
        cq.Workplane("YZ")
        .circle(0.008)
        .extrude(LID_W - 0.10)
        .translate((-(LID_W - 0.10) * 0.5, 0.0, 0.010))
    )
    front_pull = _centered_box(
        LID_W - 0.12,
        0.026,
        0.010,
        xyz=(0.0, -LID_D + 0.018, 0.012),
    )
    return frame.union(rear_bar).union(front_pull)


def _build_basket_shell() -> cq.Workplane:
    basket = cq.Workplane("XY").circle(TUB_OUTER_R).extrude(TUB_H)
    inner_cut = (
        cq.Workplane("XY")
        .workplane(offset=0.018)
        .circle(TUB_INNER_R)
        .extrude(TUB_H - 0.018)
    )
    basket = basket.cut(inner_cut)
    rim_outer = (
        cq.Workplane("XY")
        .workplane(offset=TUB_H - 0.03)
        .circle(0.246)
        .extrude(0.03)
    )
    rim_inner = (
        cq.Workplane("XY")
        .workplane(offset=TUB_H - 0.03)
        .circle(0.223)
        .extrude(0.03)
    )
    return basket.union(rim_outer.cut(rim_inner))


def _build_tub_support_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=0.85)
        .circle(0.260)
        .circle(0.226)
        .extrude(0.03)
    )


def _visual_top_z(ctx: TestContext, part, elem: str) -> float | None:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    _, maxs = aabb
    return float(maxs[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washer")

    enamel = model.material("enamel", rgba=(0.95, 0.96, 0.97, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    smoky_glass = model.material("smoky_glass", rgba=(0.30, 0.38, 0.42, 0.42))
    stainless = model.material("stainless", rgba=(0.76, 0.79, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.21, 0.24, 0.27, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.28, 0.31, 0.34, 1.0))
    button_finish = model.material("button_finish", rgba=(0.86, 0.88, 0.90, 1.0))
    detergent_finish = model.material("detergent_finish", rgba=(0.58, 0.74, 0.88, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_body_shell(), "washer_body_shell"),
        material=enamel,
        name="body_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_console_shell(), "washer_console_shell"),
        material=warm_gray,
        name="rear_console",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_tub_support_ring(), "washer_tub_support"),
        material=warm_gray,
        name="tub_support",
    )
    cabinet.visual(
        Box((BODY_W - 0.02, 0.05, 0.006)),
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.012, 0.18)),
        material=warm_gray,
        name="front_trim",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_frame(), "washer_lid_frame"),
        material=enamel,
        name="lid_frame",
    )
    lid.visual(
        Box((0.522, 0.432, 0.006)),
        origin=Origin(xyz=(0.0, -0.29, 0.008)),
        material=smoky_glass,
        name="lid_window",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_build_basket_shell(), "washer_basket_shell"),
        material=stainless,
        name="basket_shell",
    )
    tub.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=enamel,
        name="agitator_base",
    )
    tub.visual(
        Cylinder(radius=0.055, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=enamel,
        name="agitator_mid",
    )
    tub.visual(
        Cylinder(radius=0.042, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.393)),
        material=enamel,
        name="agitator_upper",
    )
    tub.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.483)),
        material=enamel,
        name="softener_cup",
    )
    tub.visual(
        Box((0.030, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.042, 0.500)),
        material=dark_trim,
        name="softener_hinge_mount",
    )
    for vane_index, yaw in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        tub.visual(
            Box((0.014, 0.082, 0.190)),
            origin=Origin(
                xyz=(0.032, 0.0, 0.208),
                rpy=(0.0, 0.0, yaw),
            ),
            material=enamel,
            name=f"vane_{vane_index}",
        )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_Y, TUB_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=10.0,
        ),
    )

    softener_lid = model.part("softener_lid")
    softener_lid.visual(
        Box((0.066, 0.054, 0.004)),
        origin=Origin(xyz=(0.0, -0.027, 0.004)),
        material=smoky_glass,
        name="softener_cover",
    )
    softener_lid.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(
            xyz=(0.0, 0.0, 0.004),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_trim,
        name="softener_hinge",
    )
    model.articulation(
        "tub_to_softener_lid",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=softener_lid,
        origin=Origin(xyz=(0.0, 0.054, 0.499)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    detergent_cap = model.part("detergent_cap")
    detergent_cap.visual(
        Box((0.052, 0.048, 0.004)),
        origin=Origin(xyz=(0.026, 0.0, 0.002)),
        material=detergent_finish,
        name="detergent_cover",
    )
    detergent_cap.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.0, 0.004),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_trim,
        name="detergent_hinge",
    )
    detergent_cap.visual(
        Box((0.010, 0.020, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.004)),
        material=dark_trim,
        name="detergent_pull",
    )
    model.articulation(
        "cabinet_to_detergent_cap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=detergent_cap,
        origin=Origin(xyz=(0.286, -0.018, DECK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.051, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.003, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_trim,
        name="dial_collar",
    )
    selector_dial.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.016, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=knob_finish,
        name="dial_knob",
    )
    selector_dial.visual(
        Box((0.005, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.027, 0.018)),
        material=button_finish,
        name="dial_pointer",
    )
    model.articulation(
        "cabinet_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_dial,
        origin=Origin(xyz=(0.0, CONSOLE_FRONT_Y, 1.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
        ),
    )

    button_xs = (-0.132, -0.066, 0.0, 0.066, 0.132)
    for index, x_pos in enumerate(button_xs):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.050, 0.008, 0.022)),
            origin=Origin(xyz=(0.0, -0.010, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.038, 0.006, 0.016)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=warm_gray,
            name="button_base",
        )
        model.articulation(
            f"cabinet_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, CONSOLE_FRONT_Y, 0.978)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0018,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    detergent_cap = object_model.get_part("detergent_cap")
    selector_dial = object_model.get_part("selector_dial")
    softener_lid = object_model.get_part("softener_lid")
    center_button = object_model.get_part("mode_button_2")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    dial_spin = object_model.get_articulation("cabinet_to_selector_dial")
    button_press = object_model.get_articulation("cabinet_to_mode_button_2")
    detergent_hinge = object_model.get_articulation("cabinet_to_detergent_cap")
    softener_hinge = object_model.get_articulation("tub_to_softener_lid")

    ctx.allow_overlap(
        cabinet,
        tub,
        elem_a="tub_support",
        elem_b="basket_shell",
        reason="The cabinet support collar intentionally nests under the basket flange as a simplified suspension support.",
    )

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="body_shell",
        max_gap=0.006,
        max_penetration=0.0,
        name="lid rests just above the washer deck",
    )
    ctx.expect_gap(
        detergent_cap,
        cabinet,
        axis="z",
        positive_elem="detergent_cover",
        negative_elem="body_shell",
        max_gap=0.010,
        max_penetration=0.0,
        name="detergent cap folds flat on the top deck",
    )

    basket_aabb = ctx.part_element_world_aabb(tub, elem="basket_shell")
    basket_depth = None
    if basket_aabb is not None:
        mins, maxs = basket_aabb
        basket_depth = float(maxs[2] - mins[2])
    ctx.check(
        "basket remains deep",
        basket_depth is not None and basket_depth >= 0.57,
        details=f"basket_depth={basket_depth}",
    )

    ctx.check(
        "tub uses continuous rotation",
        tub_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={tub_spin.articulation_type}",
    )
    ctx.check(
        "selector dial uses continuous rotation",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_spin.articulation_type}",
    )
    tub_aabb = ctx.part_element_world_aabb(tub, elem="agitator_upper")
    ctx.check(
        "agitator rises from basket center",
        tub_aabb is not None,
        details="Expected agitator geometry integrated into the rotating tub assembly.",
    )
    ctx.check(
        "five separate mode buttons present",
        all(object_model.get_part(f"mode_button_{index}") is not None for index in range(5)),
        details="Expected five separate prismatic mode button parts.",
    )

    lid_closed_top = _visual_top_z(ctx, lid, "lid_frame")
    lid_open_top = None
    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            lid_open_top = _visual_top_z(ctx, lid, "lid_frame")
    ctx.check(
        "lid opens upward",
        lid_closed_top is not None and lid_open_top is not None and lid_open_top > lid_closed_top + 0.22,
        details=f"closed_top={lid_closed_top}, open_top={lid_open_top}",
    )

    button_rest = ctx.part_world_position(center_button)
    button_pressed = None
    button_limits = button_press.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_press: button_limits.upper}):
            button_pressed = ctx.part_world_position(center_button)
    ctx.check(
        "mode button presses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] > button_rest[1] + 0.0012,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    detergent_closed_top = _visual_top_z(ctx, detergent_cap, "detergent_cover")
    detergent_open_top = None
    detergent_limits = detergent_hinge.motion_limits
    if detergent_limits is not None and detergent_limits.upper is not None:
        with ctx.pose({detergent_hinge: detergent_limits.upper}):
            detergent_open_top = _visual_top_z(ctx, detergent_cap, "detergent_cover")
    ctx.check(
        "detergent cap flips upward",
        detergent_closed_top is not None
        and detergent_open_top is not None
        and detergent_open_top > detergent_closed_top + 0.04,
        details=f"closed_top={detergent_closed_top}, open_top={detergent_open_top}",
    )

    softener_closed_top = _visual_top_z(ctx, softener_lid, "softener_cover")
    softener_open_top = None
    softener_limits = softener_hinge.motion_limits
    if softener_limits is not None and softener_limits.upper is not None:
        with ctx.pose({softener_hinge: softener_limits.upper}):
            softener_open_top = _visual_top_z(ctx, softener_lid, "softener_cover")
    ctx.check(
        "softener lid opens from agitator top",
        softener_closed_top is not None
        and softener_open_top is not None
        and softener_open_top > softener_closed_top + 0.03,
        details=f"closed_top={softener_closed_top}, open_top={softener_open_top}",
    )

    ctx.expect_gap(
        softener_lid,
        tub,
        axis="z",
        positive_elem="softener_cover",
        negative_elem="softener_cup",
        max_gap=0.010,
        max_penetration=0.0,
        name="softener lid sits over the agitator cup",
    )
    ctx.check(
        "selector dial part present",
        selector_dial is not None,
        details="Expected a separate selector dial part.",
    )

    return ctx.report()


object_model = build_object_model()
