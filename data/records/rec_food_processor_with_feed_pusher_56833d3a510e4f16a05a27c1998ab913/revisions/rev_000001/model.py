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

BASE_WIDTH = 0.270
BASE_DEPTH = 0.220
BASE_HEIGHT = 0.134
BASE_FRONT_X = 0.118

BOWL_JOINT_Z = 0.126
BOWL_HEIGHT = 0.182
BOWL_TOP_RADIUS = 0.101
BOWL_INNER_TOP_RADIUS = 0.097

LID_CHUTE_X = 0.045
CHUTE_OUTER_RADIUS = 0.043
CHUTE_INNER_RADIUS = 0.032
CHUTE_HEIGHT = 0.145
PUSHER_TRAVEL = 0.100
BUTTON_TRAVEL = 0.006


def _ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _build_base_shell() -> cq.Workplane:
    lower = cq.Workplane("XY").box(BASE_WIDTH, BASE_DEPTH, 0.040, centered=(True, True, False))
    mid = (
        cq.Workplane("XY")
        .box(0.244, 0.194, 0.055, centered=(True, True, False))
        .translate((0.0, 0.0, 0.040))
    )
    upper = (
        cq.Workplane("XY")
        .box(0.216, 0.170, 0.039, centered=(True, True, False))
        .translate((0.0, 0.0, 0.095))
    )
    front_panel = (
        cq.Workplane("XY")
        .box(0.012, 0.118, 0.050, centered=(True, True, False))
        .translate((0.108, 0.0, 0.054))
    )
    shell = lower.union(mid).union(upper).union(front_panel)
    seat_cut = cq.Workplane("XY").circle(0.077).extrude(0.034).translate((0.0, 0.0, 0.104))
    return shell.cut(seat_cut)


def _build_bowl_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.074)
        .workplane(offset=0.165)
        .circle(BOWL_TOP_RADIUS)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.018))
    )
    inner = (
        cq.Workplane("XY")
        .circle(0.068)
        .workplane(offset=0.159)
        .circle(BOWL_INNER_TOP_RADIUS)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.024))
    )
    rim = _ring(0.104, 0.096, 0.009).translate((0.0, 0.0, 0.173))
    return outer.cut(inner).union(rim)


def _build_bowl_handle() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.016, 0.040, 0.124).translate((0.0, 0.114, 0.090))
    slot = cq.Workplane("XY").box(0.024, 0.022, 0.060).translate((0.0, 0.120, 0.090))
    return outer.cut(slot)


def _build_lid_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(0.108).extrude(0.006)
    return plate.cut(cq.Workplane("XY").circle(0.033).extrude(0.014).translate((LID_CHUTE_X, 0.0, -0.004)))


def _build_lid_lip() -> cq.Workplane:
    return _ring(0.095, 0.087, 0.010).translate((0.0, 0.0, -0.010))


def _build_chute_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(CHUTE_OUTER_RADIUS).extrude(CHUTE_HEIGHT)
    inner = cq.Workplane("XY").circle(CHUTE_INNER_RADIUS).extrude(CHUTE_HEIGHT)
    flange = _ring(0.048, 0.039, 0.010).translate((0.0, 0.0, -0.004))
    return outer.cut(inner).union(flange).translate((LID_CHUTE_X, 0.0, 0.0))


def _build_pusher_sleeve() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.029).extrude(0.145).translate((0.0, 0.0, -0.145))
    inner = cq.Workplane("XY").circle(0.026).extrude(0.145).translate((0.0, 0.0, -0.145))
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_processor")

    base_white = model.material("base_white", rgba=(0.93, 0.93, 0.91, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.20, 0.22, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.84, 0.88, 0.92, 0.42))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shell(), "base_shell"),
        material=base_white,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(_ring(0.080, 0.064, 0.022), "bowl_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=dark_gray,
        name="bowl_collar",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.008, 0.110, 0.080)),
        origin=Origin(xyz=(0.004, 0.0, 0.040)),
        material=dark_gray,
        name="panel",
    )

    model.articulation(
        "base_to_control_panel",
        ArticulationType.FIXED,
        parent=base,
        child=control_panel,
        origin=Origin(xyz=(0.135, -0.006, 0.024)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_build_bowl_shell(), "bowl_shell"),
        material=clear_smoke,
        name="bowl_shell",
    )
    bowl.visual(
        mesh_from_cadquery(_build_bowl_handle(), "bowl_handle"),
        material=clear_smoke,
        name="handle",
    )
    bowl.visual(
        mesh_from_cadquery(_ring(0.061, 0.045, 0.018), "bowl_coupling"),
        material=dark_gray,
        name="coupling_ring",
    )
    bowl.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_gray,
        name="center_shaft",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, BOWL_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_plate(), "lid_plate"),
        material=clear_smoke,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_build_lid_lip(), "lid_lip"),
        material=clear_smoke,
        name="lip",
    )
    lid.visual(
        mesh_from_cadquery(_build_chute_shell(), "feed_chute"),
        material=clear_smoke,
        name="chute",
    )

    model.articulation(
        "bowl_to_lid",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BOWL_HEIGHT + 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_build_pusher_sleeve(), "pusher_sleeve"),
        material=clear_smoke,
        name="sleeve",
    )
    pusher.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_gray,
        name="cap",
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(LID_CHUTE_X, 0.0, CHUTE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PUSHER_TRAVEL, effort=15.0, velocity=0.25),
    )

    blade_hub = model.part("blade_hub")
    blade_hub.visual(
        mesh_from_cadquery(_ring(0.024, 0.012, 0.026), "hub_sleeve"),
        material=dark_gray,
        name="hub_sleeve",
    )
    blade_hub.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_gray,
        name="hub_cap",
    )
    blade_hub.visual(
        Box((0.054, 0.012, 0.003)),
        origin=Origin(xyz=(0.046, 0.0, 0.020), rpy=(0.0, math.radians(8.0), math.radians(14.0))),
        material=dark_gray,
        name="upper_blade",
    )
    blade_hub.visual(
        Box((0.054, 0.012, 0.003)),
        origin=Origin(
            xyz=(-0.046, 0.0, 0.028),
            rpy=(0.0, math.radians(-6.0), math.pi + math.radians(14.0)),
        ),
        material=dark_gray,
        name="lower_blade",
    )

    model.articulation(
        "bowl_to_blade_hub",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade_hub,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="knob_skirt",
    )
    selector_knob.visual(
        Cylinder(radius=0.025, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="knob_body",
    )
    selector_knob.visual(
        Box((0.006, 0.004, 0.011)),
        origin=Origin(xyz=(0.024, 0.0, 0.017)),
        material=base_white,
        name="pointer",
    )

    model.articulation(
        "base_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=selector_knob,
        origin=Origin(xyz=(0.008, 0.032, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    for index, y in enumerate((-0.012, -0.040)):
        button = model.part(f"pulse_button_{index}")
        button.visual(
            Box((0.010, 0.020, 0.012)),
            origin=Origin(xyz=(0.005, 0.0, 0.0)),
            material=dark_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.006, 0.010, 0.010)),
            origin=Origin(xyz=(-0.003, 0.0, 0.0)),
            material=dark_gray,
            name="button_stem",
        )

        model.articulation(
            f"base_to_pulse_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(0.014, y, 0.040)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=8.0, velocity=0.08),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    blade_hub = object_model.get_part("blade_hub")
    selector_knob = object_model.get_part("selector_knob")
    pulse_button_0 = object_model.get_part("pulse_button_0")
    pulse_button_1 = object_model.get_part("pulse_button_1")

    bowl_lock = object_model.get_articulation("base_to_bowl")
    lid_lock = object_model.get_articulation("bowl_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    blade_spin = object_model.get_articulation("bowl_to_blade_hub")
    knob_spin = object_model.get_articulation("base_to_selector_knob")
    button_press_0 = object_model.get_articulation("base_to_pulse_button_0")
    button_press_1 = object_model.get_articulation("base_to_pulse_button_1")

    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.10, name="bowl stays centered over base")
    ctx.expect_within(
        bowl,
        base,
        axes="xy",
        inner_elem="coupling_ring",
        outer_elem="bowl_collar",
        margin=0.0,
        name="bowl coupling stays inside base collar",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="z",
        elem_a="coupling_ring",
        elem_b="bowl_collar",
        min_overlap=0.014,
        name="bowl coupling remains vertically engaged with collar",
    )
    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        elem_a="lip",
        elem_b="bowl_shell",
        min_overlap=0.18,
        name="lid lip overlaps bowl rim footprint",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="sleeve",
        outer_elem="chute",
        margin=0.0015,
        name="pusher sleeve stays centered in chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="sleeve",
        elem_b="chute",
        min_overlap=0.090,
        name="pusher remains deeply inserted at rest",
    )
    ctx.expect_overlap(
        blade_hub,
        bowl,
        axes="x",
        elem_a="upper_blade",
        elem_b="bowl_shell",
        min_overlap=0.05,
        name="blade sits over processing cavity",
    )
    ctx.expect_gap(
        blade_hub,
        bowl,
        axis="z",
        positive_elem="upper_blade",
        negative_elem="coupling_ring",
        min_gap=0.035,
        max_gap=0.120,
        name="blade clears the bowl coupling base",
    )

    handle_rest = ctx.part_element_world_aabb(bowl, elem="handle")
    with ctx.pose({bowl_lock: math.radians(18.0)}):
        handle_turned = ctx.part_element_world_aabb(bowl, elem="handle")
        ctx.check(
            "bowl lock twist moves handle around the axis",
            handle_rest is not None
            and handle_turned is not None
            and abs(handle_turned[0][0] - handle_rest[0][0]) > 0.010,
            details=f"rest={handle_rest}, turned={handle_turned}",
        )

    chute_rest = ctx.part_element_world_aabb(lid, elem="chute")
    with ctx.pose({lid_lock: math.radians(16.0)}):
        chute_turned = ctx.part_element_world_aabb(lid, elem="chute")
        rest_center_y = (chute_rest[0][1] + chute_rest[1][1]) / 2.0 if chute_rest is not None else None
        turned_center_y = (chute_turned[0][1] + chute_turned[1][1]) / 2.0 if chute_turned is not None else None
        ctx.check(
            "lid lock twist moves chute around the bowl",
            rest_center_y is not None
            and turned_center_y is not None
            and abs(turned_center_y - rest_center_y) > 0.008,
            details=f"rest={chute_rest}, turned={chute_turned}",
        )

    pusher_rest = ctx.part_element_world_aabb(pusher, elem="sleeve")
    with ctx.pose({pusher_slide: PUSHER_TRAVEL}):
        pusher_extended = ctx.part_element_world_aabb(pusher, elem="sleeve")
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="sleeve",
            outer_elem="chute",
            margin=0.0015,
            name="extended pusher sleeve stays centered in chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="sleeve",
            elem_b="chute",
            min_overlap=0.040,
            name="extended pusher retains insertion in chute",
        )
        ctx.check(
            "pusher lifts upward when extended",
            pusher_rest is not None
            and pusher_extended is not None
            and pusher_extended[0][2] > pusher_rest[0][2] + 0.080,
            details=f"rest={pusher_rest}, extended={pusher_extended}",
        )

    blade_rest = aabb_center(ctx.part_element_world_aabb(blade_hub, elem="upper_blade"))
    with ctx.pose({blade_spin: math.pi / 2.0}):
        blade_turned = aabb_center(ctx.part_element_world_aabb(blade_hub, elem="upper_blade"))
        ctx.check(
            "blade hub rotation swings blade around shaft",
            blade_rest is not None
            and blade_turned is not None
            and abs(blade_turned[1] - blade_rest[1]) > 0.020,
            details=f"rest={blade_rest}, turned={blade_turned}",
        )

    pointer_rest = aabb_center(ctx.part_element_world_aabb(selector_knob, elem="pointer"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        pointer_turned = aabb_center(ctx.part_element_world_aabb(selector_knob, elem="pointer"))
        ctx.check(
            "selector knob rotates pointer around its axle",
            pointer_rest is not None
            and pointer_turned is not None
            and abs(pointer_turned[1] - pointer_rest[1]) > 0.010,
            details=f"rest={pointer_rest}, turned={pointer_turned}",
        )

    button_0_rest = aabb_center(ctx.part_element_world_aabb(pulse_button_0, elem="button_cap"))
    with ctx.pose({button_press_0: BUTTON_TRAVEL}):
        button_0_pressed = aabb_center(ctx.part_element_world_aabb(pulse_button_0, elem="button_cap"))
        ctx.check(
            "upper pulse button presses inward",
            button_0_rest is not None
            and button_0_pressed is not None
            and button_0_pressed[0] < button_0_rest[0] - 0.004,
            details=f"rest={button_0_rest}, pressed={button_0_pressed}",
        )

    button_1_rest = aabb_center(ctx.part_element_world_aabb(pulse_button_1, elem="button_cap"))
    with ctx.pose({button_press_1: BUTTON_TRAVEL}):
        button_1_pressed = aabb_center(ctx.part_element_world_aabb(pulse_button_1, elem="button_cap"))
        ctx.check(
            "lower pulse button presses inward",
            button_1_rest is not None
            and button_1_pressed is not None
            and button_1_pressed[0] < button_1_rest[0] - 0.004,
            details=f"rest={button_1_rest}, pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
