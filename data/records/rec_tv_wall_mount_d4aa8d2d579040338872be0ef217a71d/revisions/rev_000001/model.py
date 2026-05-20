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


WALL_PLATE_HEIGHT = 0.34
WALL_PLATE_WIDTH = 0.11
WALL_PLATE_THICKNESS = 0.008
WALL_AXIS_BACKSET = 0.032

PIVOT_OUTER_RADIUS = 0.020
PIVOT_BORE_RADIUS = 0.008
EYE_THICKNESS = 0.012
CLEVIS_EAR_THICKNESS = 0.010
CLEVIS_GAP = EYE_THICKNESS
CLEVIS_STACK_HEIGHT = 2.0 * CLEVIS_EAR_THICKNESS + CLEVIS_GAP

ARM_SECTION_WIDTH = 0.056
ARM_SECTION_HEIGHT = 0.026
ARM_WALL = 0.004
INNER_LINK_LENGTH = 0.33
OUTER_LINK_LENGTH = 0.31
HEAD_NECK_LENGTH = 0.088

TILT_PIVOT_OUTER_RADIUS = 0.016
TILT_PIVOT_BORE_RADIUS = 0.006
TILT_EYE_THICKNESS = 0.018
TILT_EAR_THICKNESS = 0.018
TILT_GAP = TILT_EYE_THICKNESS
TILT_STACK_WIDTH = 2.0 * TILT_EAR_THICKNESS + TILT_GAP


def _vertical_ring(center_x: float, thickness: float, outer_radius: float, bore_radius: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(thickness / 2.0, both=True)
        .translate((center_x, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("XY")
        .circle(bore_radius)
        .extrude((thickness + 0.01) / 2.0, both=True)
        .translate((center_x, 0.0, 0.0))
    )
    return outer.cut(bore)


def _horizontal_ring(center_x: float, thickness: float, outer_radius: float, bore_radius: float) -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .extrude(thickness / 2.0, both=True)
        .translate((center_x, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("XZ")
        .circle(bore_radius)
        .extrude((thickness + 0.01) / 2.0, both=True)
        .translate((center_x, 0.0, 0.0))
    )
    return outer.cut(bore)


def _wall_bracket_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(WALL_PLATE_THICKNESS, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)
        .translate((-WALL_AXIS_BACKSET, 0.0, 0.0))
        .edges("|X")
        .fillet(0.010)
    )
    bolt_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.028, -0.115),
                (0.028, -0.115),
                (-0.028, 0.115),
                (0.028, 0.115),
            ]
        )
        .circle(0.0055)
        .extrude(0.020, both=True)
    )
    support = (
        cq.Workplane("XY")
        .box(0.042, 0.062, 0.074)
        .translate((-0.018, 0.0, 0.0))
        .edges("|X")
        .fillet(0.008)
    )
    pivot = (
        cq.Workplane("XY")
        .circle(PIVOT_OUTER_RADIUS)
        .extrude(CLEVIS_STACK_HEIGHT / 2.0, both=True)
    )
    body = plate.union(support).union(pivot)
    clevis_gap = (
        cq.Workplane("XY")
        .box(0.060, 0.046, CLEVIS_GAP)
        .translate((0.0, 0.0, 0.0))
    )
    pivot_bore = (
        cq.Workplane("XY")
        .circle(PIVOT_BORE_RADIUS)
        .extrude((CLEVIS_STACK_HEIGHT + 0.020) / 2.0, both=True)
    )
    return body.cut(bolt_holes).cut(clevis_gap).cut(pivot_bore)


def _arm_link_shape(length: float) -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(length - 0.100, ARM_SECTION_WIDTH, ARM_SECTION_HEIGHT)
        .translate((length / 2.0, 0.0, 0.0))
        .edges("|X")
        .fillet(0.005)
    )
    proximal_lug = (
        cq.Workplane("XY")
        .box(0.054, 0.026, EYE_THICKNESS)
        .translate((0.027, 0.0, 0.0))
        .edges("|X")
        .fillet(0.003)
    )
    proximal_eye = _vertical_ring(
        center_x=0.0,
        thickness=EYE_THICKNESS,
        outer_radius=PIVOT_OUTER_RADIUS,
        bore_radius=PIVOT_BORE_RADIUS,
    )
    distal_clevis = (
        cq.Workplane("XY")
        .circle(PIVOT_OUTER_RADIUS)
        .extrude(CLEVIS_STACK_HEIGHT / 2.0, both=True)
        .translate((length, 0.0, 0.0))
    )
    upper_lug = (
        cq.Workplane("XY")
        .box(0.054, 0.024, CLEVIS_EAR_THICKNESS)
        .translate((length - 0.027, 0.0, 0.011))
        .edges("|X")
        .fillet(0.003)
    )
    lower_lug = (
        cq.Workplane("XY")
        .box(0.054, 0.024, CLEVIS_EAR_THICKNESS)
        .translate((length - 0.027, 0.0, -0.011))
        .edges("|X")
        .fillet(0.003)
    )
    distal_slot = (
        cq.Workplane("XY")
        .box(0.056, 0.046, CLEVIS_GAP)
        .translate((length, 0.0, 0.0))
    )
    distal_bore = (
        cq.Workplane("XY")
        .circle(PIVOT_BORE_RADIUS)
        .extrude((CLEVIS_STACK_HEIGHT + 0.020) / 2.0, both=True)
        .translate((length, 0.0, 0.0))
    )
    return (
        beam.union(proximal_lug)
        .union(proximal_eye)
        .union(upper_lug)
        .union(lower_lug)
        .union(distal_clevis)
        .cut(distal_slot)
        .cut(distal_bore)
    )


def _head_swivel_shape() -> cq.Workplane:
    neck = (
        cq.Workplane("XY")
        .box(HEAD_NECK_LENGTH - 0.070, 0.050, 0.028)
        .translate((HEAD_NECK_LENGTH / 2.0, 0.0, 0.0))
        .edges("|X")
        .fillet(0.004)
    )
    proximal_lug = (
        cq.Workplane("XY")
        .box(0.040, 0.024, EYE_THICKNESS)
        .translate((0.020, 0.0, 0.0))
        .edges("|X")
        .fillet(0.003)
    )
    swivel_eye = _vertical_ring(
        center_x=0.0,
        thickness=EYE_THICKNESS,
        outer_radius=PIVOT_OUTER_RADIUS,
        bore_radius=PIVOT_BORE_RADIUS,
    )
    tilt_clevis = (
        cq.Workplane("XZ")
        .circle(TILT_PIVOT_OUTER_RADIUS)
        .extrude(TILT_STACK_WIDTH / 2.0, both=True)
        .translate((HEAD_NECK_LENGTH, 0.0, 0.0))
    )
    upper_side_lug = (
        cq.Workplane("XY")
        .box(0.050, 0.018, 0.024)
        .translate((HEAD_NECK_LENGTH - 0.025, 0.020, 0.0))
        .edges("|X")
        .fillet(0.003)
    )
    lower_side_lug = (
        cq.Workplane("XY")
        .box(0.050, 0.018, 0.024)
        .translate((HEAD_NECK_LENGTH - 0.025, -0.020, 0.0))
        .edges("|X")
        .fillet(0.003)
    )
    tilt_slot = (
        cq.Workplane("XY")
        .box(0.050, TILT_GAP, 0.038)
        .translate((HEAD_NECK_LENGTH, 0.0, 0.0))
    )
    tilt_bore = (
        cq.Workplane("XZ")
        .circle(TILT_PIVOT_BORE_RADIUS)
        .extrude((TILT_STACK_WIDTH + 0.020) / 2.0, both=True)
        .translate((HEAD_NECK_LENGTH, 0.0, 0.0))
    )
    return (
        neck.union(proximal_lug)
        .union(swivel_eye)
        .union(upper_side_lug)
        .union(lower_side_lug)
        .union(tilt_clevis)
        .cut(tilt_slot)
        .cut(tilt_bore)
    )


def _tilt_frame_shape() -> cq.Workplane:
    tilt_eye = _horizontal_ring(
        center_x=0.0,
        thickness=TILT_EYE_THICKNESS,
        outer_radius=TILT_PIVOT_OUTER_RADIUS,
        bore_radius=TILT_PIVOT_BORE_RADIUS,
    )
    neck = (
        cq.Workplane("XY")
        .box(0.050, TILT_GAP, 0.036)
        .translate((0.025, 0.0, 0.0))
        .edges("|X")
        .fillet(0.004)
    )
    plate = (
        cq.Workplane("XY")
        .box(0.016, 0.124, 0.096)
        .translate((0.054, 0.0, 0.0))
        .edges("|X")
        .fillet(0.004)
    )
    left_rib = (
        cq.Workplane("XY")
        .box(0.034, 0.016, 0.048)
        .translate((0.040, -0.026, 0.0))
        .edges("|X")
        .fillet(0.003)
    )
    right_rib = (
        cq.Workplane("XY")
        .box(0.034, 0.016, 0.048)
        .translate((0.040, 0.026, 0.0))
        .edges("|X")
        .fillet(0.003)
    )
    return tilt_eye.union(neck).union(plate).union(left_rib).union(right_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_tv_wall_mount")

    model.material("wall_steel", rgba=(0.16, 0.16, 0.17, 1.0))
    model.material("arm_steel", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("head_cast", rgba=(0.20, 0.21, 0.23, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(_wall_bracket_shape(), "wall_bracket"),
        material="wall_steel",
        name="body",
    )

    inner_link = model.part("inner_link")
    inner_link.visual(
        mesh_from_cadquery(_arm_link_shape(INNER_LINK_LENGTH), "inner_link"),
        material="arm_steel",
        name="body",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_cadquery(_arm_link_shape(OUTER_LINK_LENGTH), "outer_link"),
        material="arm_steel",
        name="body",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        mesh_from_cadquery(_head_swivel_shape(), "head_swivel"),
        material="head_cast",
        name="body",
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        mesh_from_cadquery(_tilt_frame_shape(), "tilt_frame"),
        material="head_cast",
        name="body",
    )

    model.articulation(
        "base_fold",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=inner_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=outer_link,
        origin=Origin(xyz=(INNER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-2.70, upper=2.70),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=outer_link,
        child=head_swivel,
        origin=Origin(xyz=(OUTER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=tilt_frame,
        origin=Origin(xyz=(HEAD_NECK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.3, lower=-0.35, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    head_swivel = object_model.get_part("head_swivel")
    tilt_frame = object_model.get_part("tilt_frame")

    base_fold = object_model.get_articulation("base_fold")
    elbow_fold = object_model.get_articulation("elbow_fold")
    head_swivel_joint = object_model.get_articulation("head_swivel_joint")
    head_tilt_joint = object_model.get_articulation("head_tilt_joint")

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
    ctx.allow_overlap(
        wall_bracket,
        inner_link,
        elem_a="body",
        elem_b="body",
        reason="The first revolute hinge is represented with a shared coaxial knuckle envelope at the pivot stack.",
    )
    ctx.allow_overlap(
        inner_link,
        outer_link,
        elem_a="body",
        elem_b="body",
        reason="The elbow uses simplified interleaved hinge knuckles that intentionally share the pivot envelope.",
    )
    ctx.allow_overlap(
        outer_link,
        head_swivel,
        elem_a="body",
        elem_b="body",
        reason="The swivel head is modeled with a compact shared-pin hinge envelope at the arm tip.",
    )
    ctx.allow_overlap(
        head_swivel,
        tilt_frame,
        elem_a="body",
        elem_b="body",
        reason="The tilt trunnion is intentionally simplified as a shared-axis hinge envelope.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(wall_bracket, inner_link, contact_tol=0.0015, name="wall bracket carries inner link")
    ctx.expect_contact(inner_link, outer_link, contact_tol=0.0015, name="inner link carries outer link")
    ctx.expect_contact(outer_link, head_swivel, contact_tol=0.0015, name="outer link carries swivel head")
    ctx.expect_contact(head_swivel, tilt_frame, contact_tol=0.0015, name="swivel head carries tilt frame")

    ctx.check(
        "fold joints use vertical axes",
        base_fold.axis == (0.0, 0.0, 1.0)
        and elbow_fold.axis == (0.0, 0.0, 1.0)
        and head_swivel_joint.axis == (0.0, 0.0, 1.0),
        details=(
            f"base={base_fold.axis}, elbow={elbow_fold.axis}, swivel={head_swivel_joint.axis}"
        ),
    )
    ctx.check(
        "tilt joint uses horizontal pitch axis",
        head_tilt_joint.axis == (0.0, -1.0, 0.0),
        details=f"tilt axis={head_tilt_joint.axis}",
    )

    def span_x(part_name: str) -> float | None:
        aabb = ctx.part_world_aabb(part_name)
        if aabb is None:
            return None
        return aabb[1][0] - aabb[0][0]

    inner_span = span_x("inner_link")
    outer_span = span_x("outer_link")
    swivel_span = span_x("head_swivel")
    tilt_span = span_x("tilt_frame")
    ctx.check(
        "arm links are distinctly longer than the compact head support",
        inner_span is not None
        and outer_span is not None
        and swivel_span is not None
        and tilt_span is not None
        and inner_span > swivel_span + 0.16
        and outer_span > swivel_span + 0.14
        and inner_span > tilt_span + 0.20
        and outer_span > tilt_span + 0.18,
        details=(
            f"inner_span={inner_span}, outer_span={outer_span}, "
            f"swivel_span={swivel_span}, tilt_span={tilt_span}"
        ),
    )

    rest_head_pos = ctx.part_world_position("tilt_frame")
    with ctx.pose(base_fold=0.70):
        base_fold_pos = ctx.part_world_position("tilt_frame")
    ctx.check(
        "base fold swings the output frame laterally",
        rest_head_pos is not None
        and base_fold_pos is not None
        and abs(base_fold_pos[1]) > 0.20
        and base_fold_pos[0] < rest_head_pos[0] - 0.05,
        details=f"rest={rest_head_pos}, folded={base_fold_pos}",
    )

    rest_tilt_box = ctx.part_world_aabb("tilt_frame")
    with ctx.pose(head_tilt_joint=0.18):
        raised_tilt_box = ctx.part_world_aabb("tilt_frame")
    ctx.check(
        "positive head tilt lifts the front support",
        rest_tilt_box is not None
        and raised_tilt_box is not None
        and raised_tilt_box[1][2] > rest_tilt_box[1][2] + 0.005,
        details=f"rest={rest_tilt_box}, raised={raised_tilt_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
