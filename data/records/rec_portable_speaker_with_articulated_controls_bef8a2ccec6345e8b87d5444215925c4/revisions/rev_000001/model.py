from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boombox_speaker")

    charcoal = Material("charcoal_molded_plastic", rgba=(0.03, 0.035, 0.045, 1.0))
    dark_panel = Material("dark_control_panel", rgba=(0.015, 0.017, 0.020, 1.0))
    black = Material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    graphite = Material("graphite_grille", rgba=(0.10, 0.105, 0.11, 1.0))
    gunmetal = Material("gunmetal_trim", rgba=(0.28, 0.30, 0.32, 1.0))
    blue_accent = Material("blue_tinted_display", rgba=(0.05, 0.16, 0.22, 1.0))
    white = Material("white_print", rgba=(0.88, 0.88, 0.80, 1.0))

    case = model.part("case")
    case.visual(
        Box((0.70, 0.22, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=charcoal,
        name="case_shell",
    )
    case.visual(
        Box((0.22, 0.014, 0.14)),
        origin=Origin(xyz=(0.0, -0.117, 0.205)),
        material=dark_panel,
        name="control_panel",
    )
    case.visual(
        Box((0.150, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, -0.126, 0.272)),
        material=blue_accent,
        name="tuning_window",
    )

    grille_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.185, 0.185),
            0.004,
            hole_diameter=0.008,
            pitch=(0.015, 0.015),
            frame=0.013,
            corner_radius=0.012,
            stagger=True,
        ),
        "speaker_grille",
    )
    for x, visual_name in ((-0.225, "speaker_grille_0"), (0.225, "speaker_grille_1")):
        case.visual(
            grille_mesh,
            origin=Origin(xyz=(x, -0.112, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=visual_name,
        )

    ring_mesh = mesh_from_geometry(TorusGeometry(0.101, 0.006, radial_segments=18, tubular_segments=64), "speaker_ring")
    for x, visual_name in ((-0.225, "speaker_ring_0"), (0.225, "speaker_ring_1")):
        case.visual(
            ring_mesh,
            origin=Origin(xyz=(x, -0.116, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=visual_name,
        )

    # Side bosses form the fixed half of the carry-handle pivots.
    for x, visual_name in ((-0.362, "pivot_boss_0"), (0.362, "pivot_boss_1")):
        case.visual(
            Cylinder(radius=0.045, length=0.025),
            origin=Origin(xyz=(x, 0.0, 0.325), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gunmetal,
            name=visual_name,
        )

    for i, (x, y) in enumerate(((-0.245, -0.075), (-0.245, 0.075), (0.245, -0.075), (0.245, 0.075))):
        case.visual(
            Box((0.095, 0.040, 0.020)),
            origin=Origin(xyz=(x, y, 0.0)),
            material=black,
            name=f"foot_{i}",
        )

    for i, x in enumerate((-0.063, -0.036, 0.036, 0.063)):
        case.visual(
            Box((0.004, 0.003, 0.020)),
            origin=Origin(xyz=(x, -0.1255, 0.233)),
            material=white,
            name=f"scale_tick_{i}",
        )

    handle = model.part("carry_handle")
    handle_arc = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.392, 0.0, 0.047),
                (-0.300, 0.0, 0.145),
                (0.0, 0.0, 0.190),
                (0.300, 0.0, 0.145),
                (0.392, 0.0, 0.047),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=24,
            cap_ends=True,
        ),
        "carry_handle_arc",
    )
    handle.visual(handle_arc, material=charcoal, name="handle_arc")
    for x, visual_name in ((-0.388, "pivot_lug_0"), (0.388, "pivot_lug_1")):
        handle.visual(
            Cylinder(radius=0.033, length=0.027),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=charcoal,
            name=visual_name,
        )
    for x, visual_name in ((-0.392, "handle_neck_0"), (0.392, "handle_neck_1")):
        handle.visual(
            Cylinder(radius=0.012, length=0.034),
            origin=Origin(xyz=(x, 0.0, 0.036)),
            material=charcoal,
            name=visual_name,
        )

    tuning_knob = model.part("tuning_knob")
    tuning_knob.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=gunmetal,
        name="shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.096,
            0.045,
            body_style="faceted",
            base_diameter=0.100,
            top_diameter=0.083,
            edge_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=28, depth=0.0018, width=0.0025),
            indicator=KnobIndicator(style="line", mode="engraved", angle_deg=90.0, depth=0.0008),
            center=False,
        ),
        "tuning_knob_body",
    )
    tuning_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=black,
        name="knob_body",
    )
    tuning_knob.visual(
        Box((0.009, 0.045, 0.003)),
        origin=Origin(xyz=(0.0, 0.023, 0.0725)),
        material=white,
        name="pointer_mark",
    )

    model.articulation(
        "case_to_handle",
        ArticulationType.REVOLUTE,
        parent=case,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "panel_to_tuning_knob",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=tuning_knob,
        origin=Origin(xyz=(0.0, -0.124, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=7.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _center(bounds):
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    def _size(bounds):
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple(hi[i] - lo[i] for i in range(3))

    case = object_model.get_part("case")
    handle = object_model.get_part("carry_handle")
    knob = object_model.get_part("tuning_knob")
    handle_joint = object_model.get_articulation("case_to_handle")
    knob_joint = object_model.get_articulation("panel_to_tuning_knob")

    ctx.check(
        "handle has realistic pivot limits",
        handle_joint.articulation_type == ArticulationType.REVOLUTE
        and handle_joint.motion_limits is not None
        and handle_joint.motion_limits.lower is not None
        and handle_joint.motion_limits.upper is not None
        and handle_joint.motion_limits.lower < -1.0
        and handle_joint.motion_limits.upper > 1.0,
        details=f"type={handle_joint.articulation_type}, limits={handle_joint.motion_limits}",
    )
    ctx.check(
        "tuning knob is continuously rotatable",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    case_shell = ctx.part_element_world_aabb(case, elem="case_shell")
    handle_arc = ctx.part_element_world_aabb(handle, elem="handle_arc")
    handle_arc_clearance = None
    if case_shell is not None and handle_arc is not None:
        handle_arc_clearance = handle_arc[0][2] - case_shell[1][2]
    ctx.check(
        "carry handle arches above the top",
        handle_arc_clearance is not None and handle_arc_clearance > 0.025,
        details=f"clearance={handle_arc_clearance}",
    )

    shaft = ctx.part_element_world_aabb(knob, elem="shaft")
    panel = ctx.part_element_world_aabb(case, elem="control_panel")
    shaft_mount_gap = None
    shaft_center = _center(shaft)
    if shaft is not None and panel is not None:
        shaft_mount_gap = shaft[1][1] - panel[0][1]
    ctx.check(
        "knob shaft seats on front panel",
        shaft is not None
        and panel is not None
        and abs(shaft_mount_gap) < 0.001
        and panel[0][0] < shaft_center[0] < panel[1][0]
        and panel[0][2] < shaft_center[2] < panel[1][2],
        details=f"shaft={shaft}, panel={panel}, gap={shaft_mount_gap}",
    )

    knob_body_size = _size(ctx.part_element_world_aabb(knob, elem="knob_body"))
    ctx.check(
        "front tuning knob is large and proud",
        knob_body_size is not None and knob_body_size[0] > 0.080 and knob_body_size[1] > 0.040 and knob_body_size[2] > 0.080,
        details=f"knob_body_size={knob_body_size}",
    )

    boss_0 = ctx.part_element_world_aabb(case, elem="pivot_boss_0")
    boss_1 = ctx.part_element_world_aabb(case, elem="pivot_boss_1")
    lug_0 = ctx.part_element_world_aabb(handle, elem="pivot_lug_0")
    lug_1 = ctx.part_element_world_aabb(handle, elem="pivot_lug_1")
    ctx.check(
        "handle lugs meet side pivot bosses",
        boss_0 is not None
        and boss_1 is not None
        and lug_0 is not None
        and lug_1 is not None
        and abs(lug_0[1][0] - boss_0[0][0]) < 0.001
        and abs(lug_1[0][0] - boss_1[1][0]) < 0.001,
        details=f"lug_0={lug_0}, boss_0={boss_0}, lug_1={lug_1}, boss_1={boss_1}",
    )

    rest_pointer = _center(ctx.part_element_world_aabb(knob, elem="pointer_mark"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_pointer = _center(ctx.part_element_world_aabb(knob, elem="pointer_mark"))
    ctx.check(
        "pointer mark rotates with tuning knob",
        rest_pointer is not None
        and turned_pointer is not None
        and abs(turned_pointer[0] - rest_pointer[0]) > 0.015
        and rest_pointer[2] - turned_pointer[2] > 0.015,
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    rest_handle_center = _center(ctx.part_element_world_aabb(handle, elem="handle_arc"))
    with ctx.pose({handle_joint: 1.0}):
        pivoted_handle_center = _center(ctx.part_element_world_aabb(handle, elem="handle_arc"))
    ctx.check(
        "carry handle pivots forward about side pins",
        rest_handle_center is not None
        and pivoted_handle_center is not None
        and pivoted_handle_center[1] < rest_handle_center[1] - 0.08,
        details=f"rest={rest_handle_center}, pivoted={pivoted_handle_center}",
    )

    return ctx.report()


object_model = build_object_model()
