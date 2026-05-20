from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.240
BASE_DEPTH = 0.190
BASE_HEIGHT = 0.140

BOWL_HEIGHT = 0.158
BOWL_TOP_Z = 0.116

CHUTE_CENTER = (0.030, 0.014)
CHUTE_HEIGHT = 0.118
CHUTE_OUTER = (0.082, 0.102)
CHUTE_INNER = (0.052, 0.072)

FLAP_CENTER = (-0.045, -0.030)
FLAP_SIZE = (0.034, 0.026)
FLAP_HINGE_Y = -0.015


def _base_body_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, 0.050, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    upper = (
        cq.Workplane("XY")
        .workplane(offset=0.050)
        .rect(0.214, 0.166)
        .workplane(offset=0.046)
        .rect(0.176, 0.138)
        .loft(combine=True)
    )
    dock = cq.Workplane("XY").workplane(offset=0.096).circle(0.072).extrude(0.020)
    body = lower.union(upper).union(dock)

    panel_recess = (
        cq.Workplane("XZ")
        .rect(0.176, 0.100)
        .extrude(0.014)
        .translate((0.008, -BASE_DEPTH * 0.5 - 0.001, 0.074))
    )
    dial_hole = (
        cq.Workplane("XZ")
        .circle(0.015)
        .extrude(0.036)
        .translate((-0.055, -BASE_DEPTH * 0.5 - 0.006, 0.083))
    )

    button_holes = None
    for x_pos, z_pos in (
        (0.036, 0.088),
        (0.072, 0.088),
        (0.036, 0.056),
        (0.072, 0.056),
    ):
        hole = (
            cq.Workplane("XZ")
            .rect(0.020, 0.012)
            .extrude(0.038)
            .translate((x_pos, -BASE_DEPTH * 0.5 - 0.006, z_pos))
        )
        button_holes = hole if button_holes is None else button_holes.union(hole)

    return body.cut(panel_recess).cut(dial_hole).cut(button_holes)


def _bowl_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(0.058)
        .workplane(offset=0.018)
        .circle(0.080)
        .workplane(offset=0.050)
        .circle(0.098)
        .workplane(offset=0.090)
        .circle(0.104)
        .loft(combine=True)
    )
    shell = shell.union(cq.Workplane("XY").circle(0.060).extrude(0.016))
    shell = shell.faces(">Z").shell(-0.0035)
    drive_socket = cq.Workplane("XY").circle(0.026).extrude(0.028)
    return shell.cut(drive_socket)


def _lid_shell_shape() -> cq.Workplane:
    lid = cq.Workplane("XY").circle(0.108).circle(0.044).extrude(0.006)
    lid = lid.union(cq.Workplane("XY").circle(0.106).circle(0.100).extrude(-0.012))

    chute_cut = (
        cq.Workplane("XY")
        .box(0.056, 0.076, 0.024, centered=(True, True, False))
        .translate((CHUTE_CENTER[0], CHUTE_CENTER[1], -0.008))
    )
    ingredient_cut = (
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.018, centered=(True, True, False))
        .translate((FLAP_CENTER[0], FLAP_CENTER[1], -0.004))
    )
    collar = (
        cq.Workplane("XY")
        .box(0.040, 0.028, 0.004, centered=(True, True, False))
        .translate((FLAP_CENTER[0], FLAP_CENTER[1], 0.006))
    )
    collar = collar.cut(
        cq.Workplane("XY")
        .box(0.032, 0.020, 0.010, centered=(True, True, False))
        .translate((FLAP_CENTER[0], FLAP_CENTER[1], 0.004))
    )
    return lid.cut(chute_cut).cut(ingredient_cut).union(collar)


def _chute_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CHUTE_OUTER[0], CHUTE_OUTER[1], CHUTE_HEIGHT, centered=(True, True, False))
        .translate((CHUTE_CENTER[0], CHUTE_CENTER[1], 0.000))
    )
    inner = (
        cq.Workplane("XY")
        .box(CHUTE_INNER[0], CHUTE_INNER[1], CHUTE_HEIGHT + 0.010, centered=(True, True, False))
        .translate((CHUTE_CENTER[0], CHUTE_CENTER[1], -0.004))
    )
    rim = (
        cq.Workplane("XY")
        .box(0.088, 0.108, 0.006, centered=(True, True, False))
        .translate((CHUTE_CENTER[0], CHUTE_CENTER[1], CHUTE_HEIGHT))
    )
    rim_cut = (
        cq.Workplane("XY")
        .box(CHUTE_INNER[0], CHUTE_INNER[1], 0.010, centered=(True, True, False))
        .translate((CHUTE_CENTER[0], CHUTE_CENTER[1], CHUTE_HEIGHT - 0.002))
    )
    return outer.cut(inner).union(rim.cut(rim_cut))


def _flap_mount_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.044, 0.006, 0.008, centered=(True, True, False))
        .translate((FLAP_CENTER[0], FLAP_HINGE_Y + 0.003, 0.006))
    )


def _pusher_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.046, 0.066, 0.124, centered=(True, True, False))
        .translate((0.000, 0.000, -0.114))
    )
    return body


def _pusher_cap_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.068, 0.088, 0.012, centered=(True, True, False))
        .translate((0.000, 0.000, 0.002))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.050, 0.060, 0.020, centered=(True, True, False))
        .translate((0.000, 0.000, 0.014))
    )
    return cap.union(grip)


def _flap_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FLAP_SIZE[0], FLAP_SIZE[1], 0.004, centered=(True, False, False))
        .translate((0.000, -FLAP_SIZE[1], 0.000))
    )


def _disc_shape() -> cq.Workplane:
    disc = (
        cq.Workplane("XY")
        .circle(0.071)
        .circle(0.0105)
        .extrude(0.006)
    )
    for angle in (0.0, 120.0, 240.0):
        slot = (
            cq.Workplane("XY")
            .center(0.035 * math.cos(math.radians(angle)), 0.035 * math.sin(math.radians(angle)))
            .rect(0.030, 0.010)
            .extrude(0.010)
        )
        disc = disc.cut(slot)
    hub = cq.Workplane("XY").circle(0.020).circle(0.0105).extrude(0.018)
    return disc.union(hub)


def _button_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.026, 0.006, 0.016, centered=(True, True, False))
        .translate((0.000, -0.0045, 0.000))
    )
    stem = (
        cq.Workplane("XY")
        .box(0.016, 0.010, 0.010, centered=(True, True, False))
        .translate((0.000, 0.0025, 0.003))
    )
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_food_processor")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.13, 0.14, 1.0))
    bowl_clear = model.material("bowl_clear", rgba=(0.72, 0.78, 0.82, 0.36))
    lid_clear = model.material("lid_clear", rgba=(0.74, 0.80, 0.84, 0.32))
    pusher_grey = model.material("pusher_grey", rgba=(0.82, 0.84, 0.85, 1.0))
    metal = model.material("metal", rgba=(0.82, 0.84, 0.86, 1.0))
    shaft_grey = model.material("shaft_grey", rgba=(0.45, 0.47, 0.49, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "food_processor_base"),
        material=body_white,
        name="base_shell",
    )
    base.visual(
        Box((0.170, 0.003, 0.096)),
        origin=Origin(xyz=(0.008, -0.0965, 0.074)),
        material=trim_black,
        name="control_fascia",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_bowl_shape(), "food_processor_bowl"),
        material=bowl_clear,
        name="bowl_shell",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.000, 0.016, BOWL_TOP_Z)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "food_processor_lid"),
        material=lid_clear,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_chute_shape(), "food_processor_chute"),
        material=lid_clear,
        name="chute_shell",
    )
    lid.visual(
        mesh_from_cadquery(_flap_mount_shape(), "food_processor_flap_mount"),
        material=trim_black,
        name="flap_mount",
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.000, 0.000, BOWL_HEIGHT + 0.012)),
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_flap_panel_shape(), "food_processor_flap"),
        material=lid_clear,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(xyz=(-0.012, -0.003, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_black,
        name="flap_barrel_0",
    )
    flap.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(xyz=(0.012, -0.003, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_black,
        name="flap_barrel_1",
    )
    model.articulation(
        "lid_to_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=flap,
        origin=Origin(xyz=(FLAP_CENTER[0], FLAP_HINGE_Y, 0.0105)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        mesh_from_cadquery(_pusher_shape(), "food_processor_pusher"),
        material=pusher_grey,
        name="pusher_body",
    )
    pusher.visual(
        mesh_from_cadquery(_pusher_cap_shape(), "food_processor_pusher_cap"),
        material=pusher_grey,
        name="pusher_cap",
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_CENTER[0], CHUTE_CENTER[1], CHUTE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=shaft_grey,
        name="spindle_hub",
    )
    spindle.visual(
        Cylinder(radius=0.008, length=0.068),
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=shaft_grey,
        name="spindle_shaft",
    )
    model.articulation(
        "base_to_spindle",
        ArticulationType.FIXED,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.000, 0.016, 0.116)),
    )

    disc = model.part("disc")
    disc.visual(
        mesh_from_cadquery(_disc_shape(), "food_processor_disc"),
        material=metal,
        name="disc_blade",
    )
    model.articulation(
        "spindle_to_disc",
        ArticulationType.CONTINUOUS,
        parent=spindle,
        child=disc,
        origin=Origin(xyz=(0.000, 0.000, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.026,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.052, 0.006, flare=0.08),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "food_processor_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="dial_shell",
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(-0.055, -0.095, 0.083)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )

    button_mesh = mesh_from_cadquery(_button_shape(), "food_processor_button")
    button_positions = (
        (0.036, 0.088),
        (0.072, 0.088),
        (0.036, 0.056),
        (0.072, 0.056),
    )
    for index, (x_pos, z_pos) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=trim_black, name="button_shell")
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, -0.0935, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0035,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    flap = object_model.get_part("flap")
    pusher = object_model.get_part("pusher")
    disc = object_model.get_part("disc")
    dial = object_model.get_part("dial")

    flap_joint = object_model.get_articulation("lid_to_flap")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    disc_joint = object_model.get_articulation("spindle_to_disc")
    dial_joint = object_model.get_articulation("base_to_dial")

    ctx.expect_overlap(
        bowl,
        lid,
        axes="xy",
        min_overlap=0.180,
        name="lid covers the broad bowl opening",
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_body",
        outer_elem="chute_shell",
        margin=0.002,
        name="pusher stays centered in the chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_body",
        elem_b="chute_shell",
        min_overlap=0.090,
        name="resting pusher remains deeply inserted in the chute",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.085}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="chute_shell",
            margin=0.002,
            name="raised pusher stays inside the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_body",
            elem_b="chute_shell",
            min_overlap=0.030,
            name="raised pusher still retains insertion",
        )
        raised_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher lifts upward",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.06,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )

    ctx.expect_gap(
        flap,
        lid,
        axis="z",
        positive_elem="flap_panel",
        negative_elem="lid_shell",
        max_gap=0.006,
        max_penetration=0.0015,
        name="ingredient flap sits nearly flush when closed",
    )
    flap_closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: math.radians(85.0)}):
        flap_open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "ingredient flap lifts clear when opened",
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][2] > flap_closed_aabb[1][2] + 0.020,
        details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
    )

    disc_rest = ctx.part_world_position(disc)
    with ctx.pose({disc_joint: math.pi}):
        disc_turn = ctx.part_world_position(disc)
    ctx.check(
        "disc remains centered on the spindle while rotating",
        disc_rest is not None
        and disc_turn is not None
        and max(abs(disc_rest[i] - disc_turn[i]) for i in range(3)) < 1e-6,
        details=f"rest={disc_rest}, turned={disc_turn}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_turn = ctx.part_world_position(dial)
    ctx.check(
        "selector dial rotates in place",
        dial_rest is not None
        and dial_turn is not None
        and max(abs(dial_rest[i] - dial_turn[i]) for i in range(3)) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turn}",
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"base_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.0035}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
