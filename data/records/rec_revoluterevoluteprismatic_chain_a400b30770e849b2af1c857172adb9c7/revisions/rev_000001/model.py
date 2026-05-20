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


EAR_THICKNESS = 0.008
JOINT_LUG_THICKNESS = 0.018

INNER_LINK_LENGTH = 0.205
OUTER_STAGE_ORIGIN_X = 0.145
STAGE_GUIDE_LENGTH = 0.090
STAGE_CARRIAGE_LENGTH = 0.048
STAGE_MAX_EXTENSION = 0.032


def _union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _y_cylinder(*, x_center: float, radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_center, 0.0)
        .circle(radius)
        .extrude(length)
        .translate((0.0, -length / 2.0, 0.0))
    )


def _x_box(*, x_start: float, x_end: float, size_y: float, size_z: float) -> cq.Workplane:
    length = x_end - x_start
    return cq.Workplane("XY").box(length, size_y, size_z).translate(
        ((x_start + x_end) / 2.0, 0.0, 0.0)
    )


def _side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.012, 0.160, 0.300).translate((-0.056, 0.0, 0.0))
    spine = _x_box(x_start=-0.050, x_end=-0.010, size_y=0.052, size_z=0.096)
    shoulder_pad = _x_box(x_start=-0.010, x_end=0.000, size_y=0.040, size_z=0.050)
    return _union_all(plate, spine, shoulder_pad)


def _inner_link_shape() -> cq.Workplane:
    shoulder_pad = _x_box(x_start=0.000, x_end=0.028, size_y=JOINT_LUG_THICKNESS, size_z=0.032)
    beam = _x_box(x_start=0.028, x_end=0.176, size_y=0.016, size_z=0.022)
    elbow_housing = _x_box(x_start=0.168, x_end=INNER_LINK_LENGTH, size_y=0.030, size_z=0.038)
    return _union_all(shoulder_pad, beam, elbow_housing)


def _outer_link_shape() -> cq.Workplane:
    elbow_pad = _x_box(x_start=0.000, x_end=0.022, size_y=JOINT_LUG_THICKNESS, size_z=0.030)
    beam = _x_box(x_start=0.022, x_end=0.128, size_y=0.016, size_z=0.020)
    rail_support = _x_box(x_start=0.128, x_end=OUTER_STAGE_ORIGIN_X, size_y=0.022, size_z=0.018)
    rail = _x_box(
        x_start=OUTER_STAGE_ORIGIN_X,
        x_end=OUTER_STAGE_ORIGIN_X + STAGE_GUIDE_LENGTH,
        size_y=0.020,
        size_z=0.010,
    ).translate((0.0, 0.0, 0.010))
    nose_block = _x_box(
        x_start=OUTER_STAGE_ORIGIN_X + STAGE_GUIDE_LENGTH - 0.004,
        x_end=OUTER_STAGE_ORIGIN_X + STAGE_GUIDE_LENGTH + 0.012,
        size_y=0.028,
        size_z=0.022,
    ).translate((0.0, 0.0, 0.008))
    return _union_all(elbow_pad, beam, rail_support, rail, nose_block)


def _end_stage_shape() -> cq.Workplane:
    shoe = _x_box(x_start=0.000, x_end=STAGE_CARRIAGE_LENGTH, size_y=0.022, size_z=0.010).translate(
        (0.0, 0.0, 0.020)
    )
    carriage = _x_box(x_start=0.000, x_end=0.050, size_y=0.036, size_z=0.024).translate((0.0, 0.0, 0.032))
    tool_block = _x_box(x_start=0.048, x_end=0.072, size_y=0.028, size_z=0.022).translate((0.0, 0.0, 0.030))
    return _union_all(shoe, carriage, tool_block)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_linear_arm")

    model.material("wall_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("arm_alloy", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("stage_dark", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("stage_silver", rgba=(0.82, 0.84, 0.87, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_shape(), "side_plate"),
        material="wall_steel",
        name="plate_shell",
    )

    inner_link = model.part("inner_link")
    inner_link.visual(
        mesh_from_cadquery(_inner_link_shape(), "inner_link"),
        material="arm_alloy",
        name="inner_shell",
    )

    outer_link = model.part("outer_link")
    outer_link.visual(
        mesh_from_cadquery(_outer_link_shape(), "outer_link"),
        material="stage_dark",
        name="outer_shell",
    )

    end_stage = model.part("end_stage")
    end_stage.visual(
        mesh_from_cadquery(_end_stage_shape(), "end_stage"),
        material="stage_silver",
        name="stage_shell",
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=inner_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.1,
            lower=-0.35,
            upper=1.15,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=inner_link,
        child=outer_link,
        origin=Origin(xyz=(INNER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.3,
            lower=-0.20,
            upper=1.30,
        ),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=outer_link,
        child=end_stage,
        origin=Origin(xyz=(OUTER_STAGE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=STAGE_MAX_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    inner_link = object_model.get_part("inner_link")
    outer_link = object_model.get_part("outer_link")
    end_stage = object_model.get_part("end_stage")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    stage_slide = object_model.get_articulation("stage_slide")

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

    ctx.check("shoulder_axis_is_pitch", shoulder.axis == (0.0, -1.0, 0.0), f"axis={shoulder.axis}")
    ctx.check("elbow_axis_is_pitch", elbow.axis == (0.0, -1.0, 0.0), f"axis={elbow.axis}")
    ctx.check("stage_axis_is_outboard_x", stage_slide.axis == (1.0, 0.0, 0.0), f"axis={stage_slide.axis}")

    ctx.expect_contact(inner_link, side_plate, name="shoulder_joint_has_supported_contact")
    ctx.expect_contact(outer_link, inner_link, name="elbow_joint_has_supported_contact")
    with ctx.pose(stage_slide=0.0):
        ctx.expect_contact(end_stage, outer_link, name="stage_has_contact_when_retracted")
    with ctx.pose(stage_slide=STAGE_MAX_EXTENSION):
        ctx.expect_contact(end_stage, outer_link, name="stage_has_contact_when_extended")
        ctx.expect_overlap(
            end_stage,
            outer_link,
            axes="y",
            min_overlap=0.020,
            name="stage_remains_laterally_guided",
        )
        ctx.expect_gap(
            end_stage,
            outer_link,
            axis="z",
            max_gap=0.001,
            max_penetration=0.006,
            name="stage_remains_vertically_engaged",
        )

    outer_link_rest = ctx.part_world_position(outer_link)
    with ctx.pose(shoulder_pitch=0.85):
        outer_link_raised = ctx.part_world_position(outer_link)
    ctx.check(
        "positive_shoulder_raises_outer_link",
        outer_link_rest is not None
        and outer_link_raised is not None
        and outer_link_raised[2] > outer_link_rest[2] + 0.10,
        f"rest={outer_link_rest}, raised={outer_link_raised}",
    )

    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=0.0, stage_slide=0.0):
        stage_rest = ctx.part_world_position(end_stage)
    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=0.75, stage_slide=0.0):
        stage_elbow_up = ctx.part_world_position(end_stage)
    ctx.check(
        "positive_elbow_raises_end_stage",
        stage_rest is not None
        and stage_elbow_up is not None
        and stage_elbow_up[2] > stage_rest[2] + 0.06,
        f"rest={stage_rest}, elbow_up={stage_elbow_up}",
    )

    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=0.0, stage_slide=0.0):
        stage_retracted = ctx.part_world_position(end_stage)
    with ctx.pose(shoulder_pitch=0.0, elbow_pitch=0.0, stage_slide=STAGE_MAX_EXTENSION):
        stage_extended = ctx.part_world_position(end_stage)
    ctx.check(
        "positive_stage_motion_extends_outboard",
        stage_retracted is not None
        and stage_extended is not None
        and stage_extended[0] > stage_retracted[0] + 0.025,
        f"retracted={stage_retracted}, extended={stage_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
