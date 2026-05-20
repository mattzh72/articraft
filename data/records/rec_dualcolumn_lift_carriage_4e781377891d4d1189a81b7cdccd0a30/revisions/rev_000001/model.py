from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.42
BASE_D = 0.24
BASE_T = 0.04

RAIL_X = 0.13
RAIL_W = 0.04
RAIL_D = 0.05
RAIL_H = 1.06

LOWER_BRACE_W = 0.30
LOWER_BRACE_D = 0.04
LOWER_BRACE_H = 0.38
UPPER_BRACE_W = 0.30
UPPER_BRACE_D = 0.04
UPPER_BRACE_H = 0.34
BRACE_Y = -0.045

TOP_BEAM_W = 0.36
TOP_BEAM_D = 0.10
TOP_BEAM_T = 0.04

GUIDE_OUTER_W = 0.074
GUIDE_D = 0.06
GUIDE_H = 0.26
GUIDE_Y = (RAIL_D / 2.0) + (GUIDE_D / 2.0)

SADDLE_HOME_Z = 0.14
SADDLE_TRAVEL = 0.62

SADDLE_BRIDGE_W = (2.0 * RAIL_X) - GUIDE_OUTER_W
SADDLE_BACK_D = 0.03
SADDLE_BACK_H = 0.24
SADDLE_BACK_Y = 0.005
SADDLE_RIB_D = 0.11
SADDLE_RIB_T = 0.04
SADDLE_RIB_Y = 0.03
SADDLE_LOWER_RIB_Z = 0.05
SADDLE_UPPER_RIB_Z = 0.21
WORK_FACE_S = 0.22
WORK_FACE_D = 0.02
WORK_FACE_Y = 0.09
WORK_FACE_Z = 0.15


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _saddle_frame_shape():
    back = (
        cq.Workplane("XY")
        .box(SADDLE_BRIDGE_W, SADDLE_BACK_D, SADDLE_BACK_H)
        .translate((0.0, SADDLE_BACK_Y, SADDLE_BACK_H / 2.0))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(SADDLE_BRIDGE_W, SADDLE_RIB_D, SADDLE_RIB_T)
        .translate((0.0, SADDLE_RIB_Y, SADDLE_LOWER_RIB_Z))
    )
    upper_rib = (
        cq.Workplane("XY")
        .box(SADDLE_BRIDGE_W, SADDLE_RIB_D, SADDLE_RIB_T)
        .translate((0.0, SADDLE_RIB_Y, SADDLE_UPPER_RIB_Z))
    )
    return back.union(lower_rib).union(upper_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_service_lift")

    model.material("frame_gray", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("rail_steel", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("saddle_orange", rgba=(0.87, 0.48, 0.14, 1.0))
    model.material("workface_gray", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    _add_box_visual(
        frame,
        name="base_plate",
        size=(BASE_W, BASE_D, BASE_T),
        center=(0.0, 0.0, BASE_T / 2.0),
        material="frame_gray",
    )
    _add_box_visual(
        frame,
        name="left_rail",
        size=(RAIL_W, RAIL_D, RAIL_H),
        center=(-RAIL_X, 0.0, BASE_T + (RAIL_H / 2.0)),
        material="rail_steel",
    )
    _add_box_visual(
        frame,
        name="right_rail",
        size=(RAIL_W, RAIL_D, RAIL_H),
        center=(RAIL_X, 0.0, BASE_T + (RAIL_H / 2.0)),
        material="rail_steel",
    )
    _add_box_visual(
        frame,
        name="lower_brace",
        size=(LOWER_BRACE_W, LOWER_BRACE_D, LOWER_BRACE_H),
        center=(0.0, BRACE_Y, BASE_T + (LOWER_BRACE_H / 2.0)),
        material="frame_gray",
    )
    _add_box_visual(
        frame,
        name="upper_brace",
        size=(UPPER_BRACE_W, UPPER_BRACE_D, UPPER_BRACE_H),
        center=(0.0, BRACE_Y, BASE_T + RAIL_H - (UPPER_BRACE_H / 2.0)),
        material="frame_gray",
    )
    _add_box_visual(
        frame,
        name="top_beam",
        size=(TOP_BEAM_W, TOP_BEAM_D, TOP_BEAM_T),
        center=(0.0, -0.005, BASE_T + RAIL_H + (TOP_BEAM_T / 2.0)),
        material="frame_gray",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_T + RAIL_H + TOP_BEAM_T)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + RAIL_H + TOP_BEAM_T) / 2.0)),
    )

    saddle = model.part("saddle")
    saddle_frame_shape = _saddle_frame_shape()
    saddle.visual(
        Box((GUIDE_OUTER_W, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(-RAIL_X, GUIDE_Y, GUIDE_H / 2.0)),
        material="saddle_orange",
        name="left_guide",
    )
    saddle.visual(
        Box((GUIDE_OUTER_W, GUIDE_D, GUIDE_H)),
        origin=Origin(xyz=(RAIL_X, GUIDE_Y, GUIDE_H / 2.0)),
        material="saddle_orange",
        name="right_guide",
    )
    saddle.visual(
        mesh_from_cadquery(saddle_frame_shape, "saddle_frame"),
        material="saddle_orange",
        name="saddle_frame",
    )
    saddle.visual(
        Box((WORK_FACE_S, WORK_FACE_D, WORK_FACE_S)),
        origin=Origin(xyz=(0.0, WORK_FACE_Y, WORK_FACE_Z)),
        material="workface_gray",
        name="work_face",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((2.0 * RAIL_X + GUIDE_OUTER_W, 0.15, GUIDE_H)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.03, GUIDE_H / 2.0)),
    )

    model.articulation(
        "frame_to_saddle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, SADDLE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SADDLE_TRAVEL,
            effort=1400.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    saddle = object_model.get_part("saddle")
    lift = object_model.get_articulation("frame_to_saddle")
    left_rail = frame.get_visual("left_rail")
    right_rail = frame.get_visual("right_rail")
    left_guide = saddle.get_visual("left_guide")
    right_guide = saddle.get_visual("right_guide")
    work_face = saddle.get_visual("work_face")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
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

    ctx.expect_contact(
        saddle,
        frame,
        elem_a=left_guide,
        elem_b=left_rail,
        contact_tol=1e-4,
        name="left guide bears on left rail at rest",
    )
    ctx.expect_contact(
        saddle,
        frame,
        elem_a=right_guide,
        elem_b=right_rail,
        contact_tol=1e-4,
        name="right guide bears on right rail at rest",
    )
    ctx.expect_gap(
        saddle,
        frame,
        axis="y",
        max_gap=1e-4,
        max_penetration=0.0,
        positive_elem=left_guide,
        negative_elem=left_rail,
        name="left guide seats against the front of the left rail",
    )
    ctx.expect_gap(
        saddle,
        frame,
        axis="y",
        max_gap=1e-4,
        max_penetration=0.0,
        positive_elem=right_guide,
        negative_elem=right_rail,
        name="right guide seats against the front of the right rail",
    )
    ctx.expect_overlap(
        saddle,
        frame,
        axes="x",
        elem_a=left_guide,
        elem_b=left_rail,
        min_overlap=0.039,
        name="left guide fully covers the rail width",
    )
    ctx.expect_overlap(
        saddle,
        frame,
        axes="x",
        elem_a=right_guide,
        elem_b=right_rail,
        min_overlap=0.039,
        name="right guide fully covers the rail width",
    )
    ctx.expect_overlap(
        saddle,
        frame,
        axes="z",
        elem_a=left_guide,
        elem_b=left_rail,
        min_overlap=0.22,
        name="left guide has deep rail overlap at rest",
    )
    ctx.expect_overlap(
        saddle,
        frame,
        axes="z",
        elem_a=right_guide,
        elem_b=right_rail,
        min_overlap=0.22,
        name="right guide has deep rail overlap at rest",
    )
    ctx.expect_origin_gap(
        saddle,
        frame,
        axis="z",
        min_gap=0.10,
        name="saddle starts well above the base frame origin",
    )

    rest_pos = ctx.part_world_position(saddle)
    with ctx.pose({lift: SADDLE_TRAVEL}):
        ctx.expect_contact(
            saddle,
            frame,
            elem_a=left_guide,
            elem_b=left_rail,
            contact_tol=1e-4,
            name="left guide stays on the left rail at full lift",
        )
        ctx.expect_contact(
            saddle,
            frame,
            elem_a=right_guide,
            elem_b=right_rail,
            contact_tol=1e-4,
            name="right guide stays on the right rail at full lift",
        )
        ctx.expect_gap(
            saddle,
            frame,
            axis="y",
            max_gap=1e-4,
            max_penetration=0.0,
            positive_elem=left_guide,
            negative_elem=left_rail,
            name="left guide stays seated on the rail at full lift",
        )
        ctx.expect_gap(
            saddle,
            frame,
            axis="y",
            max_gap=1e-4,
            max_penetration=0.0,
            positive_elem=right_guide,
            negative_elem=right_rail,
            name="right guide stays seated on the rail at full lift",
        )
        ctx.expect_overlap(
            saddle,
            frame,
            axes="x",
            elem_a=left_guide,
            elem_b=left_rail,
            min_overlap=0.039,
            name="left guide still covers the rail width at full lift",
        )
        ctx.expect_overlap(
            saddle,
            frame,
            axes="x",
            elem_a=right_guide,
            elem_b=right_rail,
            min_overlap=0.039,
            name="right guide still covers the rail width at full lift",
        )
        ctx.expect_overlap(
            saddle,
            frame,
            axes="z",
            elem_a=left_guide,
            elem_b=left_rail,
            min_overlap=0.08,
            name="left guide retains insertion at full lift",
        )
        ctx.expect_overlap(
            saddle,
            frame,
            axes="z",
            elem_a=right_guide,
            elem_b=right_rail,
            min_overlap=0.08,
            name="right guide retains insertion at full lift",
        )
        raised_pos = ctx.part_world_position(saddle)

    ctx.check(
        "single stage lifts upward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.50,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.check(
        "square work face is present",
        work_face is not None,
        details="Expected named square work face visual on the saddle.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
