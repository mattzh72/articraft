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


DESK_WIDTH = 1.20
DESK_DEPTH = 0.65
TOP_THICKNESS = 0.028
TOP_CORNER_RADIUS = 0.028

COLUMN_SPACING = 0.90
FRAME_CENTER_X = COLUMN_SPACING / 2.0
FRAME_RAIL_Y = 0.205
FRAME_RAIL_HEIGHT = 0.040
FRAME_RAIL_WIDTH = 0.055
FRAME_RAIL_LENGTH = 0.86
FRAME_SPINE_WIDTH = 0.045
FRAME_SPINE_HEIGHT = 0.040
FRAME_SPINE_LENGTH = 0.56
COLUMN_BRACKET_X = 0.080
COLUMN_BRACKET_Y = 0.420
COLUMN_BRACKET_Z = 0.024
MOUNT_PLATE_X = 0.160
MOUNT_PLATE_Y = 0.170
MOUNT_PLATE_Z = 0.012
TOP_CENTER_Z = 0.052

FOOT_X = 0.090
FOOT_Y = 0.700
FOOT_Z = 0.035
FOOT_CENTER_Z = -0.523
FOOT_FILLET = 0.040

BASE_COLLAR_RADIUS = 0.064
BASE_COLLAR_HEIGHT = 0.060
BASE_COLLAR_CENTER_Z = -0.512

OUTER_RADIUS = 0.045
OUTER_INNER_RADIUS = 0.039
OUTER_LENGTH = 0.540
OUTER_BOTTOM_Z = -OUTER_LENGTH

INNER_RADIUS = 0.034
INNER_INNER_RADIUS = 0.029
INNER_TOP_Z = -0.008
INNER_BOTTOM_Z = -0.590
INNER_PLATE_Z = 0.012
INNER_PLATE_X = 0.150
INNER_PLATE_Y = 0.180
INNER_CAP_RADIUS = 0.050
MOUNT_HEIGHT = 0.170
STAGE_TRAVEL = 0.320

POD_X = 0.985
POD_Y = -0.265
POD_MOUNT_Z = TOP_CENTER_Z - (TOP_THICKNESS / 2.0)
POD_WIDTH = 0.200
POD_DEPTH = 0.058
POD_HEIGHT = 0.030
POD_CORNER_RADIUS = 0.006
SLOT_LENGTH = 0.040
SLOT_HEIGHT = 0.008
SLOT_DEPTH = 0.012
SWITCH_TRAVEL = 0.018
SWITCH_THUMB_X = 0.022
SWITCH_THUMB_Y = 0.006
SWITCH_THUMB_Z = 0.014
SWITCH_STEM_X = 0.012
SWITCH_STEM_Y = 0.009
SWITCH_STEM_Z = 0.006
SWITCH_SURFACE_Y = -(POD_DEPTH / 2.0)
GUIDE_RING_HEIGHT = 0.045
GUIDE_RING_OUTER_RADIUS = OUTER_INNER_RADIUS + 0.001


def _tube(outer_radius: float, inner_radius: float, z_min: float, z_max: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _rounded_box(size_x: float, size_y: float, size_z: float, fillet: float) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).edges("|Z").fillet(fillet)


def _outer_base_visuals(side_name: str) -> dict[str, cq.Workplane]:
    foot = _rounded_box(FOOT_X, FOOT_Y, FOOT_Z, FOOT_FILLET).translate((0.0, 0.0, FOOT_CENTER_Z))
    collar = (
        cq.Workplane("XY")
        .circle(BASE_COLLAR_RADIUS)
        .extrude(BASE_COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_COLLAR_CENTER_Z - (BASE_COLLAR_HEIGHT / 2.0)))
    )
    sleeve = _tube(OUTER_RADIUS, OUTER_INNER_RADIUS, OUTER_BOTTOM_Z, 0.0)
    guide_ring = (
        cq.Workplane("XY")
        .circle(GUIDE_RING_OUTER_RADIUS)
        .circle(INNER_RADIUS)
        .extrude(GUIDE_RING_HEIGHT)
        .translate((0.0, 0.0, -GUIDE_RING_HEIGHT))
    )
    return {
        f"{side_name}_foot_support": foot.union(collar),
        f"{side_name}_outer_sleeve": sleeve.union(guide_ring),
    }


def _inner_stage_visuals(side_name: str) -> dict[str, cq.Workplane]:
    tube = _tube(INNER_RADIUS, INNER_INNER_RADIUS, INNER_BOTTOM_Z, INNER_TOP_Z)
    cap = (
        cq.Workplane("XY")
        .circle(INNER_CAP_RADIUS)
        .extrude(INNER_PLATE_Z)
        .translate((0.0, 0.0, -INNER_PLATE_Z))
    )
    plate = (
        cq.Workplane("XY")
        .box(INNER_PLATE_X, INNER_PLATE_Y, INNER_PLATE_Z)
        .translate((0.0, 0.0, -(INNER_PLATE_Z / 2.0)))
    )
    return {
        f"{side_name}_inner_tube": tube,
        f"{side_name}_top_cap": cap,
        f"{side_name}_mount_plate": plate,
    }


def _desk_frame_visuals() -> dict[str, cq.Workplane]:
    top = _rounded_box(DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS, TOP_CORNER_RADIUS).translate(
        (FRAME_CENTER_X, 0.0, TOP_CENTER_Z)
    )
    front_rail = cq.Workplane("XY").box(FRAME_RAIL_LENGTH, FRAME_RAIL_WIDTH, FRAME_RAIL_HEIGHT).translate(
        (FRAME_CENTER_X, -FRAME_RAIL_Y, FRAME_RAIL_HEIGHT / 2.0)
    )
    back_rail = cq.Workplane("XY").box(FRAME_RAIL_LENGTH, FRAME_RAIL_WIDTH, FRAME_RAIL_HEIGHT).translate(
        (FRAME_CENTER_X, FRAME_RAIL_Y, FRAME_RAIL_HEIGHT / 2.0)
    )
    spine = cq.Workplane("XY").box(FRAME_SPINE_LENGTH, FRAME_SPINE_WIDTH, FRAME_SPINE_HEIGHT).translate(
        (FRAME_CENTER_X, 0.0, FRAME_SPINE_HEIGHT / 2.0)
    )
    left_bracket = cq.Workplane("XY").box(COLUMN_BRACKET_X, COLUMN_BRACKET_Y, COLUMN_BRACKET_Z).translate(
        (0.0, 0.0, COLUMN_BRACKET_Z / 2.0)
    )
    right_bracket = cq.Workplane("XY").box(COLUMN_BRACKET_X, COLUMN_BRACKET_Y, COLUMN_BRACKET_Z).translate(
        (COLUMN_SPACING, 0.0, COLUMN_BRACKET_Z / 2.0)
    )
    left_mount = cq.Workplane("XY").box(MOUNT_PLATE_X, MOUNT_PLATE_Y, MOUNT_PLATE_Z).translate(
        (0.0, 0.0, MOUNT_PLATE_Z / 2.0)
    )
    right_mount = cq.Workplane("XY").box(MOUNT_PLATE_X, MOUNT_PLATE_Y, MOUNT_PLATE_Z).translate(
        (COLUMN_SPACING, 0.0, MOUNT_PLATE_Z / 2.0)
    )
    return {
        "top_panel": top,
        "front_frame_rail": front_rail,
        "rear_frame_rail": back_rail,
        "center_frame_spine": spine,
        "left_column_bracket": left_bracket,
        "right_column_bracket": right_bracket,
        "left_mount_plate": left_mount,
        "right_mount_plate": right_mount,
    }


def _control_pod_shell() -> cq.Workplane:
    shell = _rounded_box(POD_WIDTH, POD_DEPTH, POD_HEIGHT, POD_CORNER_RADIUS).translate(
        (0.0, 0.0, -(POD_HEIGHT / 2.0))
    )
    slot = cq.Workplane("XY").box(SLOT_LENGTH, SLOT_DEPTH, SLOT_HEIGHT).translate(
        (0.0, -(POD_DEPTH / 2.0) + (SLOT_DEPTH / 2.0), -(POD_HEIGHT / 2.0))
    )
    return shell.cut(slot)


def _switch_slider_shape() -> cq.Workplane:
    thumb = cq.Workplane("XY").box(SWITCH_THUMB_X, SWITCH_THUMB_Y, SWITCH_THUMB_Z).translate(
        (0.0, -(SWITCH_THUMB_Y / 2.0), 0.0)
    )
    stem = cq.Workplane("XY").box(SWITCH_STEM_X, SWITCH_STEM_Y, SWITCH_STEM_Z).translate(
        (0.0, SWITCH_STEM_Y / 2.0, 0.0)
    )
    return thumb.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_standing_desk")

    model.material("laminate_oak", rgba=(0.90, 0.86, 0.79, 1.0))
    model.material("powder_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("satin_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    model.material("switch_light", rgba=(0.82, 0.85, 0.88, 1.0))

    left_outer_shapes = _outer_base_visuals("left")
    left_outer = model.part("left_outer_base")
    left_outer.visual(
        mesh_from_cadquery(left_outer_shapes["left_foot_support"], "left_foot_support"),
        material="powder_black",
        name="left_foot_support",
    )
    left_outer.visual(
        mesh_from_cadquery(left_outer_shapes["left_outer_sleeve"], "left_outer_sleeve"),
        material="powder_black",
        name="left_outer_sleeve",
    )

    left_inner_shapes = _inner_stage_visuals("left")
    left_inner = model.part("left_inner_stage")
    left_inner.visual(
        mesh_from_cadquery(left_inner_shapes["left_inner_tube"], "left_inner_tube"),
        material="powder_black",
        name="left_inner_tube",
    )
    left_inner.visual(
        mesh_from_cadquery(left_inner_shapes["left_top_cap"], "left_top_cap"),
        material="powder_black",
        name="left_top_cap",
    )
    left_inner.visual(
        mesh_from_cadquery(left_inner_shapes["left_mount_plate"], "left_mount_plate"),
        material="powder_black",
        name="left_mount_plate",
    )

    frame_shapes = _desk_frame_visuals()
    frame = model.part("desk_frame")
    frame.visual(
        mesh_from_cadquery(frame_shapes["top_panel"], "top_panel"),
        material="laminate_oak",
        name="top_panel",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["front_frame_rail"], "front_frame_rail"),
        material="powder_black",
        name="front_frame_rail",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["rear_frame_rail"], "rear_frame_rail"),
        material="powder_black",
        name="rear_frame_rail",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["center_frame_spine"], "center_frame_spine"),
        material="powder_black",
        name="center_frame_spine",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["left_column_bracket"], "left_column_bracket"),
        material="powder_black",
        name="left_column_bracket",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["right_column_bracket"], "right_column_bracket"),
        material="powder_black",
        name="right_column_bracket",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["left_mount_plate"], "left_mount_plate"),
        material="powder_black",
        name="left_mount_plate",
    )
    frame.visual(
        mesh_from_cadquery(frame_shapes["right_mount_plate"], "right_mount_plate"),
        material="powder_black",
        name="right_mount_plate",
    )

    right_inner_shapes = _inner_stage_visuals("right")
    right_inner = model.part("right_inner_stage")
    right_inner.visual(
        mesh_from_cadquery(right_inner_shapes["right_inner_tube"], "right_inner_tube"),
        material="powder_black",
        name="right_inner_tube",
    )
    right_inner.visual(
        mesh_from_cadquery(right_inner_shapes["right_top_cap"], "right_top_cap"),
        material="powder_black",
        name="right_top_cap",
    )
    right_inner.visual(
        mesh_from_cadquery(right_inner_shapes["right_mount_plate"], "right_mount_plate"),
        material="powder_black",
        name="right_mount_plate",
    )

    right_outer_shapes = _outer_base_visuals("right")
    right_outer = model.part("right_outer_base")
    right_outer.visual(
        mesh_from_cadquery(right_outer_shapes["right_foot_support"], "right_foot_support"),
        material="powder_black",
        name="right_foot_support",
    )
    right_outer.visual(
        mesh_from_cadquery(right_outer_shapes["right_outer_sleeve"], "right_outer_sleeve"),
        material="powder_black",
        name="right_outer_sleeve",
    )

    control_pod = model.part("control_pod")
    control_pod.visual(
        mesh_from_cadquery(_control_pod_shell(), "control_pod_shell"),
        material="satin_graphite",
        name="pod_shell",
    )

    switch_slider = model.part("height_lock_switch")
    switch_slider.visual(
        mesh_from_cadquery(_switch_slider_shape(), "height_lock_switch"),
        material="switch_light",
        name="switch_thumb",
    )

    left_lift = model.articulation(
        "left_column_lift",
        ArticulationType.PRISMATIC,
        parent=left_outer,
        child=left_inner,
        origin=Origin(xyz=(0.0, 0.0, MOUNT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.07,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "left_inner_to_frame",
        ArticulationType.FIXED,
        parent=left_inner,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_right_inner",
        ArticulationType.FIXED,
        parent=frame,
        child=right_inner,
        origin=Origin(xyz=(COLUMN_SPACING, 0.0, 0.0)),
    )
    right_lift = model.articulation(
        "right_column_lift",
        ArticulationType.PRISMATIC,
        parent=right_inner,
        child=right_outer,
        origin=Origin(xyz=(0.0, 0.0, -MOUNT_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.07,
            lower=0.0,
            upper=STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "frame_to_control_pod",
        ArticulationType.FIXED,
        parent=frame,
        child=control_pod,
        origin=Origin(xyz=(POD_X, POD_Y, POD_MOUNT_Z)),
    )
    model.articulation(
        "pod_to_switch",
        ArticulationType.PRISMATIC,
        parent=control_pod,
        child=switch_slider,
        origin=Origin(xyz=(-SWITCH_TRAVEL / 2.0, SWITCH_SURFACE_Y, -(POD_HEIGHT / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=SWITCH_TRAVEL,
        ),
    )

    frame.meta["coordinated_lifts"] = [left_lift.name, right_lift.name]
    right_lift.meta["note"] = "Drive this joint with the same scalar value as left_column_lift to keep both feet planted."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_outer = object_model.get_part("left_outer_base")
    left_inner = object_model.get_part("left_inner_stage")
    frame = object_model.get_part("desk_frame")
    right_inner = object_model.get_part("right_inner_stage")
    right_outer = object_model.get_part("right_outer_base")
    control_pod = object_model.get_part("control_pod")
    switch_slider = object_model.get_part("height_lock_switch")

    left_lift = object_model.get_articulation("left_column_lift")
    right_lift = object_model.get_articulation("right_column_lift")
    switch_joint = object_model.get_articulation("pod_to_switch")

    ctx.check(
        "column joints use vertical prismatic motion",
        left_lift.axis == (0.0, 0.0, 1.0)
        and right_lift.axis == (0.0, 0.0, -1.0)
        and left_lift.motion_limits is not None
        and right_lift.motion_limits is not None
        and left_lift.motion_limits.upper == STAGE_TRAVEL
        and right_lift.motion_limits.upper == STAGE_TRAVEL,
        details=f"left_axis={left_lift.axis}, right_axis={right_lift.axis}",
    )
    ctx.check(
        "switch uses short horizontal prismatic travel",
        switch_joint.axis == (1.0, 0.0, 0.0)
        and switch_joint.motion_limits is not None
        and switch_joint.motion_limits.upper is not None
        and 0.0 < switch_joint.motion_limits.upper <= 0.025,
        details=f"switch_axis={switch_joint.axis}, upper={getattr(switch_joint.motion_limits, 'upper', None)}",
    )

    ctx.allow_overlap(
        left_inner,
        left_outer,
        elem_a="left_inner_tube",
        elem_b="left_outer_sleeve",
        reason="The telescoping leg is represented with a sleeve proxy around the inner column; the visible nested fit is intentional.",
    )
    ctx.allow_overlap(
        right_inner,
        right_outer,
        elem_a="right_inner_tube",
        elem_b="right_outer_sleeve",
        reason="The telescoping leg is represented with a sleeve proxy around the inner column; the visible nested fit is intentional.",
    )

    ctx.expect_within(
        left_inner,
        left_outer,
        axes="xy",
        inner_elem="left_inner_tube",
        outer_elem="left_outer_sleeve",
        name="left inner tube stays centered in left sleeve",
    )
    ctx.expect_overlap(
        left_inner,
        left_outer,
        axes="z",
        elem_a="left_inner_tube",
        elem_b="left_outer_sleeve",
        min_overlap=0.40,
        name="left inner tube remains inserted at seated height",
    )
    ctx.expect_within(
        right_inner,
        right_outer,
        axes="xy",
        inner_elem="right_inner_tube",
        outer_elem="right_outer_sleeve",
        name="right inner tube stays centered in right sleeve",
    )
    ctx.expect_overlap(
        right_inner,
        right_outer,
        axes="z",
        elem_a="right_inner_tube",
        elem_b="right_outer_sleeve",
        min_overlap=0.40,
        name="right inner tube remains inserted at seated height",
    )

    frame_rest = ctx.part_world_position(frame)
    right_outer_rest = ctx.part_world_position(right_outer)
    switch_rest = ctx.part_world_position(switch_slider)
    with ctx.pose({left_lift: STAGE_TRAVEL, right_lift: STAGE_TRAVEL, switch_joint: SWITCH_TRAVEL}):
        ctx.expect_within(
            left_inner,
            left_outer,
            axes="xy",
            inner_elem="left_inner_tube",
            outer_elem="left_outer_sleeve",
            name="left inner tube stays centered when extended",
        )
        ctx.expect_overlap(
            left_inner,
            left_outer,
            axes="z",
            elem_a="left_inner_tube",
            elem_b="left_outer_sleeve",
            min_overlap=0.09,
            name="left inner tube keeps retained insertion when extended",
        )
        ctx.expect_within(
            right_inner,
            right_outer,
            axes="xy",
            inner_elem="right_inner_tube",
            outer_elem="right_outer_sleeve",
            name="right inner tube stays centered when extended",
        )
        ctx.expect_overlap(
            right_inner,
            right_outer,
            axes="z",
            elem_a="right_inner_tube",
            elem_b="right_outer_sleeve",
            min_overlap=0.09,
            name="right inner tube keeps retained insertion when extended",
        )
        frame_extended = ctx.part_world_position(frame)
        right_outer_extended = ctx.part_world_position(right_outer)
        switch_extended = ctx.part_world_position(switch_slider)

    ctx.check(
        "desk frame rises during coordinated lift",
        frame_rest is not None
        and frame_extended is not None
        and frame_extended[2] > frame_rest[2] + 0.30,
        details=f"rest={frame_rest}, extended={frame_extended}",
    )
    ctx.check(
        "right foot stays planted during coordinated lift",
        right_outer_rest is not None
        and right_outer_extended is not None
        and abs(right_outer_extended[2] - right_outer_rest[2]) <= 1e-6,
        details=f"rest={right_outer_rest}, extended={right_outer_extended}",
    )
    ctx.check(
        "front control switch slides across the pod slot",
        switch_rest is not None
        and switch_extended is not None
        and switch_extended[0] > switch_rest[0] + 0.015,
        details=f"rest={switch_rest}, extended={switch_extended}",
    )
    ctx.expect_contact(
        control_pod,
        frame,
        elem_a="pod_shell",
        elem_b="top_panel",
        name="control pod mounts flush to the desktop underside",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
