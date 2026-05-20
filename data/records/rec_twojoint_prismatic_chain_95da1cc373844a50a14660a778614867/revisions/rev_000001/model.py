from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GUIDE_CLEARANCE_D = 0.0003

OUTER_LENGTH = 0.56
OUTER_OD = 0.100
OUTER_ID = 0.086
OUTER_FRONT_BUSH_X = 0.537
OUTER_FRONT_BUSH_L = 0.018
OUTER_REAR_BUSH_X = 0.122
OUTER_REAR_BUSH_L = 0.022

MID_HOME = 0.120
MID_LENGTH = 0.490
MID_OD = 0.078
MID_ID = 0.064
MID_STOP_X = 0.190
MID_STOP_L = 0.012
MID_STOP_OD = 0.083
MID_HEAD_X = 0.445
MID_HEAD_L = 0.045
MID_HEAD_OD = 0.088
MID_REAR_BUSH_X = 0.148
MID_REAR_BUSH_L = 0.022
MID_FRONT_BUSH_X = 0.452
MID_FRONT_BUSH_L = 0.020
MID_TRAVEL = OUTER_FRONT_BUSH_X - (MID_HOME + MID_STOP_X + MID_STOP_L)

OUTPUT_HOME = 0.140
OUTPUT_TUBE_L = 0.352
OUTPUT_OD = 0.056
OUTPUT_ID = 0.044
OUTPUT_STOP_X = 0.072
OUTPUT_STOP_L = 0.012
OUTPUT_STOP_OD = 0.060
OUTPUT_PLATE_X = OUTPUT_TUBE_L
OUTPUT_PLATE_T = 0.014
OUTPUT_PLATE_W = 0.104
OUTPUT_PLATE_H = 0.104
OUTPUT_PLATE_HOLE_D = 0.010
OUTPUT_PLATE_BOLT_OFFSET = 0.032
OUTPUT_TRAVEL = MID_FRONT_BUSH_X - (OUTPUT_HOME + OUTPUT_STOP_X + OUTPUT_STOP_L)


def _ring(length: float, outer_d: float, inner_d: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(outer_d / 2.0).circle(inner_d / 2.0).extrude(length)


def _ring_at(x0: float, length: float, outer_d: float, inner_d: float) -> cq.Workplane:
    return _ring(length, outer_d, inner_d).translate((x0, 0.0, 0.0))


def _solid_disk_at(x0: float, length: float, diameter: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(diameter / 2.0).extrude(length).translate((x0, 0.0, 0.0))


def _tube_at(x0: float, length: float, outer_d: float, inner_d: float) -> cq.Workplane:
    return _ring(length, outer_d, inner_d).translate((x0, 0.0, 0.0))


def _box(center_xyz: tuple[float, float, float], size_xyz: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size_xyz
    cx, cy, cz = center_xyz
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _output_plate() -> cq.Workplane:
    bolt = OUTPUT_PLATE_BOLT_OFFSET
    plate = (
        cq.Workplane("YZ")
        .rect(OUTPUT_PLATE_W, OUTPUT_PLATE_H)
        .extrude(OUTPUT_PLATE_T)
        .faces(">X")
        .workplane()
        .pushPoints(
            [
                (-bolt, -bolt),
                (-bolt, bolt),
                (bolt, -bolt),
                (bolt, bolt),
            ]
        )
        .hole(OUTPUT_PLATE_HOLE_D)
    )
    return plate.translate((OUTPUT_PLATE_X, 0.0, 0.0))


def _sleeve_shell() -> cq.Workplane:
    shell = _tube_at(0.0, OUTER_LENGTH, OUTER_OD, OUTER_ID)
    rear_collar = _ring_at(0.0, 0.035, 0.116, OUTER_OD)
    front_collar = _ring_at(0.515, 0.045, 0.112, OUTER_OD)
    rear_cap = _solid_disk_at(0.0, 0.012, OUTER_ID)
    return shell.union(rear_collar).union(front_collar).union(rear_cap)


def _sleeve_support() -> cq.Workplane:
    base = _box((0.180, 0.0, -0.090), (0.240, 0.180, 0.020))
    pedestal = _box((0.180, 0.0, -0.063), (0.160, 0.090, 0.034))
    return base.union(pedestal)


def _intermediate_tube() -> cq.Workplane:
    return _tube_at(0.0, MID_LENGTH, MID_OD, MID_ID)


def _intermediate_head() -> cq.Workplane:
    return _ring_at(MID_HEAD_X, MID_HEAD_L, MID_HEAD_OD, MID_OD)


def _intermediate_stop_collar() -> cq.Workplane:
    return _ring_at(MID_STOP_X, MID_STOP_L, MID_STOP_OD, MID_OD)


def _output_tube() -> cq.Workplane:
    return _tube_at(0.0, OUTPUT_TUBE_L, OUTPUT_OD, OUTPUT_ID)


def _output_stop_collar() -> cq.Workplane:
    return _ring_at(OUTPUT_STOP_X, OUTPUT_STOP_L, OUTPUT_STOP_OD, OUTPUT_OD)


def _add_visual(part, shape: cq.Workplane, name: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, name), origin=Origin(), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inline_telescoping_ram")

    model.material("outer_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("mid_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("output_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("bushing_bronze", rgba=(0.63, 0.47, 0.22, 1.0))

    outer = model.part("outer_sleeve")
    intermediate = model.part("intermediate_tube")
    output = model.part("output_tube")

    _add_visual(outer, _sleeve_shell(), "sleeve_shell", "outer_steel")
    _add_visual(outer, _sleeve_support(), "sleeve_support", "outer_steel")
    _add_visual(
        outer,
        _ring_at(OUTER_REAR_BUSH_X, OUTER_REAR_BUSH_L, OUTER_ID, MID_OD + GUIDE_CLEARANCE_D),
        "sleeve_rear_bushing",
        "bushing_bronze",
    )
    _add_visual(
        outer,
        _ring_at(OUTER_FRONT_BUSH_X, OUTER_FRONT_BUSH_L, OUTER_ID, MID_OD + GUIDE_CLEARANCE_D),
        "sleeve_front_bushing",
        "bushing_bronze",
    )

    _add_visual(intermediate, _intermediate_tube(), "stage_tube", "mid_steel")
    _add_visual(intermediate, _intermediate_head(), "stage_head", "mid_steel")
    _add_visual(intermediate, _intermediate_stop_collar(), "stage_stop_collar", "mid_steel")
    _add_visual(
        intermediate,
        _ring_at(MID_REAR_BUSH_X, MID_REAR_BUSH_L, MID_ID, OUTPUT_OD + GUIDE_CLEARANCE_D),
        "stage_rear_bushing",
        "bushing_bronze",
    )
    _add_visual(
        intermediate,
        _ring_at(MID_FRONT_BUSH_X, MID_FRONT_BUSH_L, MID_ID, OUTPUT_OD + GUIDE_CLEARANCE_D),
        "stage_front_bushing",
        "bushing_bronze",
    )

    _add_visual(output, _output_tube(), "output_tube_shell", "output_steel")
    _add_visual(output, _output_stop_collar(), "output_stop_collar", "output_steel")
    _add_visual(output, _output_plate(), "output_plate", "output_steel")

    model.articulation(
        "sleeve_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=intermediate,
        origin=Origin(xyz=(MID_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.28,
            lower=0.0,
            upper=MID_TRAVEL,
        ),
    )
    model.articulation(
        "intermediate_to_output",
        ArticulationType.PRISMATIC,
        parent=intermediate,
        child=output,
        origin=Origin(xyz=(OUTPUT_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1600.0,
            velocity=0.30,
            lower=0.0,
            upper=OUTPUT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    intermediate = object_model.get_part("intermediate_tube")
    output = object_model.get_part("output_tube")

    sleeve_joint = object_model.get_articulation("sleeve_to_intermediate")
    output_joint = object_model.get_articulation("intermediate_to_output")

    sleeve_rear_bushing = outer.get_visual("sleeve_rear_bushing")
    sleeve_front_bushing = outer.get_visual("sleeve_front_bushing")
    stage_tube = intermediate.get_visual("stage_tube")
    stage_head = intermediate.get_visual("stage_head")
    stage_stop = intermediate.get_visual("stage_stop_collar")
    stage_rear_bushing = intermediate.get_visual("stage_rear_bushing")
    stage_front_bushing = intermediate.get_visual("stage_front_bushing")
    output_tube = output.get_visual("output_tube_shell")
    output_stop = output.get_visual("output_stop_collar")
    output_plate = output.get_visual("output_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.00025)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.check(
        "shared_prismatic_axis",
        tuple(sleeve_joint.axis) == (1.0, 0.0, 0.0) and tuple(output_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected both joints on +X; got {sleeve_joint.axis} and {output_joint.axis}",
    )
    ctx.check(
        "positive_stage_travel",
        sleeve_joint.motion_limits is not None
        and output_joint.motion_limits is not None
        and sleeve_joint.motion_limits.lower == 0.0
        and output_joint.motion_limits.lower == 0.0
        and sleeve_joint.motion_limits.upper is not None
        and output_joint.motion_limits.upper is not None
        and sleeve_joint.motion_limits.upper > 0.18
        and output_joint.motion_limits.upper > 0.18,
        "Both telescoping stages should have realistic forward travel.",
    )

    ctx.expect_overlap(
        intermediate,
        outer,
        axes="yz",
        min_overlap=0.075,
        elem_a=stage_tube,
        elem_b=sleeve_rear_bushing,
        name="intermediate_guided_by_outer_rear_bushing",
    )
    ctx.expect_overlap(
        output,
        intermediate,
        axes="yz",
        min_overlap=0.054,
        elem_a=output_tube,
        elem_b=stage_rear_bushing,
        name="output_guided_by_intermediate_rear_bushing",
    )
    ctx.expect_gap(
        intermediate,
        outer,
        axis="x",
        positive_elem=stage_head,
        negative_elem=sleeve_front_bushing,
        min_gap=0.0,
        max_gap=0.015,
        name="intermediate_head_protrudes_from_sleeve",
    )
    ctx.expect_gap(
        output,
        intermediate,
        axis="x",
        positive_elem=output_plate,
        negative_elem=stage_head,
        min_gap=0.0,
        max_gap=0.012,
        name="output_plate_sits_ahead_of_front_head",
    )

    with ctx.pose({sleeve_joint: MID_TRAVEL}):
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="yz",
            min_overlap=0.075,
            elem_a=stage_tube,
            elem_b=sleeve_front_bushing,
            name="extended_intermediate_still_guided_by_outer_front_bushing",
        )
        ctx.expect_gap(
            outer,
            intermediate,
            axis="x",
            positive_elem=sleeve_front_bushing,
            negative_elem=stage_stop,
            min_gap=0.0,
            max_gap=0.004,
            max_penetration=0.0,
            name="intermediate_stop_catches_outer_bushing",
        )

    with ctx.pose({sleeve_joint: MID_TRAVEL, output_joint: OUTPUT_TRAVEL}):
        ctx.expect_overlap(
            output,
            intermediate,
            axes="yz",
            min_overlap=0.054,
            elem_a=output_tube,
            elem_b=stage_front_bushing,
            name="extended_output_still_guided_by_front_bushing",
        )
        ctx.expect_gap(
            intermediate,
            output,
            axis="x",
            positive_elem=stage_front_bushing,
            negative_elem=output_stop,
            min_gap=0.0,
            max_gap=0.004,
            max_penetration=0.0,
            name="output_stop_catches_front_bushing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
