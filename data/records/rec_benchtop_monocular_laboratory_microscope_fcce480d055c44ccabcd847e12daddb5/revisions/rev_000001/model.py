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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def _rounded_base() -> cq.Workplane:
    """Low cast-metal foot with softened vertical corners."""
    base = cq.Workplane("XY").box(0.30, 0.22, 0.040)
    return base.edges("|Z").fillet(0.025)


def _arm_mesh():
    """A tapered cast arm lofted through several superellipse sections."""

    sections = []
    for z, center_y, width, depth in (
        (0.035, 0.105, 0.095, 0.060),
        (0.180, 0.130, 0.078, 0.050),
        (0.335, 0.125, 0.064, 0.042),
        (0.428, 0.065, 0.055, 0.038),
    ):
        loop = [
            (x, y + center_y, z)
            for x, y in superellipse_profile(width, depth, exponent=3.2, segments=40)
        ]
        sections.append(loop)
    return section_loft(sections)


def _stage_plate() -> cq.Workplane:
    """Round specimen platform with a real central transmitted-light aperture."""
    plate = cq.Workplane("XY").circle(0.085).circle(0.012).extrude(0.012)
    return plate.translate((0.0, 0.0, -0.006))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_monocular_microscope")

    cast = model.material("warm_cast_gray", rgba=(0.43, 0.45, 0.44, 1.0))
    dark = model.material("satin_black", rgba=(0.015, 0.016, 0.017, 1.0))
    blackened = model.material("blackened_metal", rgba=(0.06, 0.06, 0.065, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    glass = model.material("pale_optical_glass", rgba=(0.45, 0.68, 0.80, 0.72))
    white = model.material("white_index_paint", rgba=(0.92, 0.90, 0.84, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_rounded_base(), "cast_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast,
        name="cast_base",
    )
    stand.visual(
        mesh_from_geometry(_arm_mesh(), "tapered_arm"),
        material=cast,
        name="tapered_arm",
    )
    stand.visual(
        Box((0.090, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, 0.025, 0.410)),
        material=cast,
        name="head_block",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.392)),
        material=cast,
        name="nose_bearing",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.185),
        origin=Origin(xyz=(0.0, 0.105, 0.485), rpy=(-0.58, 0.0, 0.0)),
        material=blackened,
        name="eyepiece_tube",
    )
    stand.visual(
        Cylinder(radius=0.023, length=0.035),
        origin=Origin(xyz=(0.0, 0.165, 0.576), rpy=(-0.58, 0.0, 0.0)),
        material=dark,
        name="ocular_rim",
    )
    stand.visual(
        Box((0.030, 0.105, 0.166)),
        origin=Origin(xyz=(0.0, 0.035, 0.123)),
        material=cast,
        name="guide_pedestal",
    )
    stand.visual(
        Box((0.034, 0.180, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=brushed,
        name="guide_rail",
    )
    stand.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        material=blackened,
        name="substage_condenser",
    )
    stand.visual(
        Box((0.018, 0.045, 0.045)),
        origin=Origin(xyz=(-0.036, 0.115, 0.335)),
        material=cast,
        name="focus_boss_pad",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(-0.052, 0.115, 0.335), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast,
        name="focus_bushing",
    )

    stage = model.part("stage")
    stage.visual(
        mesh_from_cadquery(_stage_plate(), "round_stage_plate"),
        material=blackened,
        name="round_stage_plate",
    )
    stage.visual(
        Box((0.014, 0.050, 0.004)),
        origin=Origin(xyz=(-0.047, 0.024, 0.008)),
        material=brushed,
        name="stage_clip_0",
    )
    stage.visual(
        Box((0.014, 0.050, 0.004)),
        origin=Origin(xyz=(0.047, -0.024, 0.008)),
        material=brushed,
        name="stage_clip_1",
    )
    stage.visual(
        Box((0.065, 0.100, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=blackened,
        name="saddle_web",
    )
    stage.visual(
        Box((0.008, 0.100, 0.024)),
        origin=Origin(xyz=(-0.030, 0.0, -0.021)),
        material=blackened,
        name="saddle_cheek_0",
    )
    stage.visual(
        Box((0.008, 0.100, 0.024)),
        origin=Origin(xyz=(0.030, 0.0, -0.021)),
        material=blackened,
        name="saddle_cheek_1",
    )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=blackened,
        name="turret_disk",
    )
    nosepiece.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=brushed,
        name="center_hub",
    )
    for idx, (angle, length, radius) in enumerate(
        ((math.radians(90.0), 0.052, 0.0075), (math.radians(210.0), 0.063, 0.0085), (math.radians(330.0), 0.074, 0.0095))
    ):
        x = 0.026 * math.cos(angle)
        y = 0.026 * math.sin(angle)
        nosepiece.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, -0.012 - length / 2.0)),
            material=brushed,
            name=f"objective_{idx}",
        )
        nosepiece.visual(
            Cylinder(radius=max(radius - 0.002, 0.004), length=0.006),
            origin=Origin(xyz=(x, y, -0.012 - length - 0.003)),
            material=glass,
            name=f"front_lens_{idx}",
        )

    focus_knob = model.part("focus_knob")
    focus_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.026,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=28, depth=0.0012, width=0.0013),
        ),
        "ribbed_focus_knob",
    )
    focus_knob.visual(
        focus_mesh,
        origin=Origin(xyz=(-0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="knurled_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="focus_axle",
    )
    focus_knob.visual(
        Box((0.004, 0.006, 0.018)),
        origin=Origin(xyz=(-0.035, 0.012, 0.0)),
        material=white,
        name="focus_mark",
    )

    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.08, lower=-0.035, upper=0.035),
    )
    model.articulation(
        "nosepiece_spin",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "focus_knob_turn",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=focus_knob,
        origin=Origin(xyz=(-0.069, 0.115, 0.335)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage = object_model.get_part("stage")
    nosepiece = object_model.get_part("nosepiece")
    focus_knob = object_model.get_part("focus_knob")
    stage_slide = object_model.get_articulation("stage_slide")
    nosepiece_spin = object_model.get_articulation("nosepiece_spin")
    focus_knob_turn = object_model.get_articulation("focus_knob_turn")

    ctx.expect_gap(
        stage,
        stand,
        axis="z",
        positive_elem="saddle_web",
        negative_elem="guide_rail",
        min_gap=0.0005,
        max_gap=0.003,
        name="stage saddle rides just above the guide rail",
    )
    ctx.expect_overlap(
        stage,
        stand,
        axes="y",
        elem_a="saddle_web",
        elem_b="guide_rail",
        min_overlap=0.090,
        name="stage remains retained on the fore-aft rail at center",
    )
    stage_rest = ctx.part_world_position(stage)
    with ctx.pose({stage_slide: stage_slide.motion_limits.upper}):
        ctx.expect_overlap(
            stage,
            stand,
            axes="y",
            elem_a="saddle_web",
            elem_b="guide_rail",
            min_overlap=0.055,
            name="stage remains retained at forward travel",
        )
        stage_forward = ctx.part_world_position(stage)
    ctx.check(
        "stage slide moves fore and aft on y",
        stage_rest is not None
        and stage_forward is not None
        and stage_forward[1] > stage_rest[1] + 0.025,
        details=f"rest={stage_rest}, forward={stage_forward}",
    )

    ctx.expect_gap(
        stand,
        nosepiece,
        axis="z",
        positive_elem="nose_bearing",
        negative_elem="turret_disk",
        min_gap=0.0,
        max_gap=0.002,
        name="nosepiece sits immediately under the head bearing",
    )

    def _element_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    obj_rest = _element_center(nosepiece, "objective_0")
    with ctx.pose({nosepiece_spin: 0.8}):
        obj_rotated = _element_center(nosepiece, "objective_0")
    ctx.check(
        "nosepiece rotates an objective around the optical axis",
        obj_rest is not None
        and obj_rotated is not None
        and abs(obj_rotated[0] - obj_rest[0]) + abs(obj_rotated[1] - obj_rest[1]) > 0.018,
        details=f"rest={obj_rest}, rotated={obj_rotated}",
    )

    mark_rest = _element_center(focus_knob, "focus_mark")
    with ctx.pose({focus_knob_turn: math.pi / 2.0}):
        mark_rotated = _element_center(focus_knob, "focus_mark")
    ctx.check(
        "focus knob rotates about a horizontal side axis",
        mark_rest is not None
        and mark_rotated is not None
        and abs(mark_rotated[2] - mark_rest[2]) > 0.006,
        details=f"rest={mark_rest}, rotated={mark_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
