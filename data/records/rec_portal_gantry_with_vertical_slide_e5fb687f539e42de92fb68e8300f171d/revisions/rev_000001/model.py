from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.90
FRAME_DEPTH = 0.34
BASE_HEIGHT = 0.12
COLUMN_WIDTH = 0.12
COLUMN_DEPTH = 0.28
HEADER_HEIGHT = 0.14
PORTAL_HEIGHT = 1.10
TRACK_WIDTH = 0.62
TRACK_DEPTH = 0.016
TRACK_HEIGHT = 0.022
TRACK_Y = 0.118
TOP_TRACK_Z = 0.83
BOTTOM_TRACK_Z = 0.59

CARRIAGE_WIDTH = 0.58
CARRIAGE_DEPTH = 0.08
CARRIAGE_HEIGHT = 0.34
CARRIAGE_HALF_TRAVEL = 0.12
CARRIAGE_HOME = (0.0, 0.14, 0.71)

STAGE_TRAVEL = 0.055
STAGE_HOME = (0.0, 0.015, -0.02)


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _carriage_body_shape():
    outer_ring = cq.Workplane("XY").box(0.58, 0.06, 0.34).translate((0.0, 0.055, 0.0))
    inner_window = cq.Workplane("XY").box(0.30, 0.08, 0.20).translate((0.0, 0.055, 0.0))
    body = outer_ring.cut(inner_window)
    body = body.union(cq.Workplane("XY").box(0.48, 0.06, 0.02).translate((0.0, -0.02, 0.145)))
    body = body.union(cq.Workplane("XY").box(0.48, 0.06, 0.02).translate((0.0, -0.02, -0.145)))
    for x in (-0.24, 0.24):
        body = body.union(cq.Workplane("XY").box(0.05, 0.115, 0.03).translate((x, 0.0, 0.16)))
        body = body.union(cq.Workplane("XY").box(0.05, 0.115, 0.03).translate((x, 0.0, -0.16)))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_portal")

    model.material("frame_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("panel_graphite", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("track_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("carriage_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("stage_white", rgba=(0.93, 0.94, 0.95, 1.0))
    model.material("accent_blue", rgba=(0.18, 0.40, 0.77, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        (FRAME_WIDTH, FRAME_DEPTH, BASE_HEIGHT),
        (0.0, 0.0, BASE_HEIGHT / 2.0),
        "frame_graphite",
        "base_plinth",
    )
    column_height = PORTAL_HEIGHT - BASE_HEIGHT - HEADER_HEIGHT
    column_z = BASE_HEIGHT + column_height / 2.0
    column_x = FRAME_WIDTH / 2.0 - COLUMN_WIDTH / 2.0
    _add_box(
        frame,
        (COLUMN_WIDTH, COLUMN_DEPTH, column_height),
        (-column_x, 0.0, column_z),
        "frame_graphite",
        "left_column",
    )
    _add_box(
        frame,
        (COLUMN_WIDTH, COLUMN_DEPTH, column_height),
        (column_x, 0.0, column_z),
        "frame_graphite",
        "right_column",
    )
    _add_box(
        frame,
        (FRAME_WIDTH, COLUMN_DEPTH, HEADER_HEIGHT),
        (0.0, 0.0, PORTAL_HEIGHT - HEADER_HEIGHT / 2.0),
        "frame_graphite",
        "header",
    )
    _add_box(
        frame,
        (0.82, 0.04, 0.96),
        (0.0, -0.12, 0.60),
        "panel_graphite",
        "back_panel",
    )
    _add_box(
        frame,
        (TRACK_WIDTH, TRACK_DEPTH, TRACK_HEIGHT),
        (0.0, TRACK_Y, TOP_TRACK_Z),
        "track_steel",
        "top_track",
    )
    _add_box(
        frame,
        (TRACK_WIDTH, TRACK_DEPTH, TRACK_HEIGHT),
        (0.0, TRACK_Y, BOTTOM_TRACK_Z),
        "track_steel",
        "bottom_track",
    )
    for prefix, z in (("top", TOP_TRACK_Z), ("bottom", BOTTOM_TRACK_Z)):
        for side, x in (("left", -0.31), ("right", 0.31)):
            _add_box(
                frame,
                (0.03, 0.24, 0.04),
                (x, 0.0, z),
                "frame_graphite",
                f"{prefix}_{side}_rib",
            )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="carriage_gray",
        name="carriage_body",
    )
    _add_box(
        carriage,
        (0.018, 0.030, 0.20),
        (-0.110, 0.010, 0.0),
        "track_steel",
        "left_guide",
    )
    _add_box(
        carriage,
        (0.018, 0.030, 0.20),
        (0.110, 0.010, 0.0),
        "track_steel",
        "right_guide",
    )
    _add_box(
        carriage,
        (0.24, 0.04, 0.026),
        (0.0, -0.05, 0.144),
        "track_steel",
        "top_slider",
    )
    _add_box(
        carriage,
        (0.24, 0.04, 0.026),
        (0.0, -0.05, -0.144),
        "track_steel",
        "bottom_slider",
    )

    stage = model.part("lift_stage")
    _add_box(
        stage,
        (0.190, 0.04, 0.13),
        (0.0, 0.018, 0.0),
        "stage_white",
        "stage_face",
    )
    _add_box(
        stage,
        (0.190, 0.07, 0.020),
        (0.0, 0.002, 0.075),
        "accent_blue",
        "stage_top_cap",
    )
    _add_box(
        stage,
        (0.190, 0.08, 0.024),
        (0.0, 0.000, -0.060),
        "accent_blue",
        "stage_bottom_shelf",
    )
    _add_box(
        stage,
        (0.006, 0.030, 0.11),
        (-0.098, -0.010, 0.0),
        "track_steel",
        "left_shoe",
    )
    _add_box(
        stage,
        (0.006, 0.030, 0.11),
        (0.098, -0.010, 0.0),
        "track_steel",
        "right_shoe",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=CARRIAGE_HOME),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_HALF_TRAVEL,
            upper=CARRIAGE_HALF_TRAVEL,
            effort=280.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=STAGE_HOME),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_TRAVEL,
            effort=180.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("lift_stage")
    frame_to_carriage = object_model.get_articulation("frame_to_carriage")
    carriage_to_stage = object_model.get_articulation("carriage_to_stage")

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
        frame,
        carriage,
        elem_a="top_track",
        elem_b="top_slider",
        name="top carriage slider rides on top frame track",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="bottom_track",
        elem_b="bottom_slider",
        name="bottom carriage slider rides on bottom frame track",
    )
    ctx.expect_contact(
        carriage,
        stage,
        elem_a="left_guide",
        elem_b="left_shoe",
        name="left lift shoe bears on carriage guide",
    )
    ctx.expect_contact(
        carriage,
        stage,
        elem_a="right_guide",
        elem_b="right_shoe",
        name="right lift shoe bears on carriage guide",
    )
    ctx.expect_within(
        stage,
        carriage,
        axes="xy",
        inner_elem="stage_face",
        margin=0.03,
        name="stage remains nested within carriage face envelope",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({frame_to_carriage: CARRIAGE_HALF_TRAVEL}):
        carriage_extended = ctx.part_world_position(carriage)
        ctx.expect_contact(
            frame,
            carriage,
            elem_a="top_track",
            elem_b="top_slider",
            name="top carriage slider stays mounted at right travel limit",
        )
        ctx.expect_overlap(
            frame,
            carriage,
            axes="x",
            elem_a="top_track",
            elem_b="top_slider",
            min_overlap=0.14,
            name="top carriage slider retains right-side insertion",
        )
    ctx.check(
        "horizontal carriage extends along +X",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.08,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    stage_rest = ctx.part_world_position(stage)
    with ctx.pose({carriage_to_stage: STAGE_TRAVEL}):
        stage_raised = ctx.part_world_position(stage)
        ctx.expect_contact(
            carriage,
            stage,
            elem_a="left_guide",
            elem_b="left_shoe",
            name="left lift shoe stays engaged at top of stroke",
        )
        ctx.expect_contact(
            carriage,
            stage,
            elem_a="right_guide",
            elem_b="right_shoe",
            name="right lift shoe stays engaged at top of stroke",
        )
        ctx.expect_overlap(
            carriage,
            stage,
            axes="z",
            elem_a="left_guide",
            elem_b="left_shoe",
            min_overlap=0.05,
            name="left lift shoe retains guide overlap at top of stroke",
        )
        ctx.expect_within(
            stage,
            carriage,
            axes="xy",
            inner_elem="stage_face",
            margin=0.03,
            name="raised stage stays nested in carriage face envelope",
        )
    ctx.check(
        "vertical stage lifts along +Z",
        stage_rest is not None
        and stage_raised is not None
        and stage_raised[2] > stage_rest[2] + 0.04,
        details=f"rest={stage_rest}, raised={stage_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
