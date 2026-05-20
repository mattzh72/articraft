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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BACKPLATE_WIDTH = 0.180
BACKPLATE_THICKNESS = 0.006
BACKPLATE_HEIGHT = 0.400

BODY_WIDTH = 0.160
BODY_DEPTH = 0.092
BODY_HEIGHT = 0.130

FRONT_PLATE_WIDTH = 0.146
FRONT_PLATE_THICKNESS = 0.004
FRONT_PLATE_HEIGHT = 0.114

HOPPER_WIDTH = 0.128
HOPPER_DEPTH = 0.118
HOPPER_HEIGHT = 0.220
HOPPER_WALL = 0.004
HOPPER_BASE_Z = 0.140

SELECTOR_Z = 0.082

CHUTE_OUTER_WIDTH = 0.090
CHUTE_OUTER_HEIGHT = 0.060
CHUTE_FRAME_DEPTH = 0.012
CHUTE_OPENING_WIDTH = 0.074
CHUTE_OPENING_HEIGHT = 0.044
CHUTE_FRONT_Y = 0.110
CHUTE_HINGE_Z = 0.010


def _build_hopper_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        HOPPER_WIDTH,
        HOPPER_DEPTH,
        HOPPER_HEIGHT,
        centered=(True, False, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            HOPPER_WIDTH - 2.0 * HOPPER_WALL,
            HOPPER_DEPTH - 2.0 * HOPPER_WALL,
            HOPPER_HEIGHT + 0.004,
            centered=(True, False, False),
        )
        .translate((0.0, HOPPER_WALL, -0.002))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_candy_dispenser")

    backplate_finish = model.material("backplate_finish", rgba=(0.69, 0.71, 0.74, 1.0))
    case_finish = model.material("case_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    front_metal = model.material("front_metal", rgba=(0.85, 0.86, 0.88, 1.0))
    shadow_dark = model.material("shadow_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.79, 0.80, 1.0))
    flap_finish = model.material("flap_finish", rgba=(0.82, 0.83, 0.85, 1.0))
    hopper_clear = model.material("hopper_clear", rgba=(0.72, 0.88, 0.96, 0.35))
    lid_finish = model.material("lid_finish", rgba=(0.84, 0.84, 0.86, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((BACKPLATE_WIDTH, BACKPLATE_THICKNESS, BACKPLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, BACKPLATE_THICKNESS * 0.5, BACKPLATE_HEIGHT * 0.5)),
        material=backplate_finish,
        name="backplate",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.047, BODY_HEIGHT * 0.5)),
        material=case_finish,
        name="body_case",
    )
    cabinet.visual(
        Box((0.086, 0.064, 0.024)),
        origin=Origin(xyz=(0.0, 0.038, 0.129)),
        material=case_finish,
        name="hopper_neck",
    )
    cabinet.visual(
        Box((FRONT_PLATE_WIDTH, FRONT_PLATE_THICKNESS, FRONT_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.098,
                0.067,
            )
        ),
        material=front_metal,
        name="front_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.023, length=0.0045),
        origin=Origin(
            xyz=(0.0, 0.098, SELECTOR_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="selector_escutcheon",
    )
    cabinet.visual(
        Box((0.036, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.101, 0.111)),
        material=chrome,
        name="coin_slot",
    )

    frame_center_z = CHUTE_OUTER_HEIGHT * 0.5
    side_x = (CHUTE_OUTER_WIDTH - 0.010) * 0.5
    cabinet.visual(
        Box((CHUTE_OUTER_WIDTH, CHUTE_FRAME_DEPTH, 0.010)),
        origin=Origin(xyz=(0.0, CHUTE_FRONT_Y - CHUTE_FRAME_DEPTH * 0.5, 0.055)),
        material=front_metal,
        name="chute_brow",
    )
    cabinet.visual(
        Box((0.010, CHUTE_FRAME_DEPTH, 0.050)),
        origin=Origin(xyz=(-side_x, CHUTE_FRONT_Y - CHUTE_FRAME_DEPTH * 0.5, 0.030)),
        material=front_metal,
        name="chute_side_0",
    )
    cabinet.visual(
        Box((0.010, CHUTE_FRAME_DEPTH, 0.050)),
        origin=Origin(xyz=(side_x, CHUTE_FRONT_Y - CHUTE_FRAME_DEPTH * 0.5, 0.030)),
        material=front_metal,
        name="chute_side_1",
    )
    cabinet.visual(
        Box((CHUTE_OUTER_WIDTH, CHUTE_FRAME_DEPTH, 0.010)),
        origin=Origin(xyz=(0.0, CHUTE_FRONT_Y - CHUTE_FRAME_DEPTH * 0.5, 0.005)),
        material=front_metal,
        name="chute_sill",
    )
    cabinet.visual(
        Box((CHUTE_OPENING_WIDTH, 0.024, CHUTE_OPENING_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.094, 0.028)),
        material=shadow_dark,
        name="chute_cavity",
    )

    hopper = model.part("hopper")
    hopper.visual(
        mesh_from_cadquery(_build_hopper_shell(), "clear_hopper"),
        material=hopper_clear,
        name="hopper_shell",
    )
    model.articulation(
        "cabinet_to_hopper",
        ArticulationType.FIXED,
        parent=cabinet,
        child=hopper,
        origin=Origin(xyz=(0.0, BACKPLATE_THICKNESS, HOPPER_BASE_Z)),
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="selector_shaft",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="selector_hub",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.024,
                body_style="skirted",
                top_diameter=0.040,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_knob_body",
        ),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="selector_body",
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.0, 0.100, SELECTOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=12.0),
    )

    flap = model.part("retrieval_flap")
    flap.visual(
        Cylinder(radius=0.0035, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="flap_barrel",
    )
    flap.visual(
        Box((0.076, 0.004, 0.045)),
        origin=Origin(xyz=(0.0, 0.002, 0.0225)),
        material=flap_finish,
        name="flap_panel",
    )
    flap.visual(
        Box((0.050, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.007, 0.039)),
        material=flap_finish,
        name="flap_lip",
    )
    model.articulation(
        "cabinet_to_retrieval_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(0.0, CHUTE_FRONT_Y + 0.0015, CHUTE_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    lid = model.part("refill_lid")
    lid.visual(
        Cylinder(radius=0.003, length=HOPPER_WIDTH + 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lid_barrel",
    )
    lid.visual(
        Box((HOPPER_WIDTH + 0.008, HOPPER_DEPTH + 0.006, 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                (HOPPER_DEPTH + 0.006) * 0.5 - 0.002,
                0.002,
            )
        ),
        material=lid_finish,
        name="lid_panel",
    )
    lid.visual(
        Box((HOPPER_WIDTH + 0.008, 0.010, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                HOPPER_DEPTH + 0.001,
                0.005,
            )
        ),
        material=lid_finish,
        name="lid_lip",
    )
    model.articulation(
        "hopper_to_refill_lid",
        ArticulationType.REVOLUTE,
        parent=hopper,
        child=lid,
        origin=Origin(xyz=(0.0, HOPPER_WALL + 0.0015, HOPPER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    hopper = object_model.get_part("hopper")
    knob = object_model.get_part("selector_knob")
    flap = object_model.get_part("retrieval_flap")
    lid = object_model.get_part("refill_lid")

    selector_joint = object_model.get_articulation("cabinet_to_selector_knob")
    flap_joint = object_model.get_articulation("cabinet_to_retrieval_flap")
    lid_joint = object_model.get_articulation("hopper_to_refill_lid")

    ctx.allow_overlap(
        cabinet,
        knob,
        elem_a="body_case",
        elem_b="selector_shaft",
        reason="The selector shaft intentionally enters the simplified solid mechanism body behind the front plate.",
    )

    ctx.check(
        "selector_joint_is_continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_joint.articulation_type!r}",
    )
    ctx.check(
        "selector_axis_is_horizontal",
        selector_joint.axis is not None
        and abs(float(selector_joint.axis[1])) > 0.9
        and abs(float(selector_joint.axis[2])) < 0.1,
        details=f"axis={selector_joint.axis!r}",
    )

    ctx.expect_overlap(
        knob,
        cabinet,
        axes="xz",
        elem_a="selector_body",
        elem_b="front_plate",
        min_overlap=0.035,
        name="selector knob stays centered on the front plate",
    )
    ctx.expect_gap(
        knob,
        cabinet,
        axis="y",
        positive_elem="selector_body",
        negative_elem="front_plate",
        min_gap=0.002,
        max_gap=0.032,
        name="selector knob stands proud of the front plate",
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            hopper,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="hopper_shell",
            max_gap=0.003,
            max_penetration=0.001,
            name="refill lid sits on the hopper rim when closed",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_joint: 1.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "refill lid lifts above the hopper",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.040,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_gap(
            flap,
            cabinet,
            axis="y",
            positive_elem="flap_panel",
            negative_elem="chute_cavity",
            min_gap=0.000,
            max_gap=0.020,
            name="retrieval flap closes the chute mouth",
        )
        closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 1.15}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "retrieval flap swings outward and downward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and float(open_flap_aabb[1][1]) > float(closed_flap_aabb[1][1]) + 0.020
        and float(open_flap_aabb[1][2]) < float(closed_flap_aabb[1][2]) - 0.015,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    hopper_aabb = ctx.part_world_aabb(hopper)
    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check(
        "wall_mounted_scale_reads_tall",
        hopper_aabb is not None
        and cabinet_aabb is not None
        and float(cabinet_aabb[1][2] - cabinet_aabb[0][2]) > 0.35
        and float(hopper_aabb[1][2] - hopper_aabb[0][2]) > 0.20,
        details=f"cabinet={cabinet_aabb}, hopper={hopper_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
