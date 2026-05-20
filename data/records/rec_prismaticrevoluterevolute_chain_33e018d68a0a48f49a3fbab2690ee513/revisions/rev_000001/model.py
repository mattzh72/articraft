from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_W = 0.95
PLATE_H = 0.62
PLATE_T = 0.03

RAIL_LEN = 0.78
RAIL_T = 0.018
RAIL_H = 0.028
RAIL_Z_OFFSET = 0.085
RAIL_OUTER_Y = PLATE_T + RAIL_T

SLIDE_STROKE = 0.52
SLIDE_X0 = -0.35
SLIDE_Y = 0.11

CARRIAGE_W = 0.14
CARRIAGE_H = 0.22
GUIDE_L = 0.14
GUIDE_H = 0.04
CARRIAGE_HEAD_D = 0.028
CARRIAGE_SPINE_W = 0.06
CARRIAGE_SPINE_D = 0.06
CARRIAGE_SPINE_H = 0.06
CARRIAGE_PAD_D = 0.012
CARRIAGE_STRUT_W = 0.022
CARRIAGE_STRUT_D = 0.032
CARRIAGE_STRUT_H = 0.06

ARM_PLATE_T = 0.018
SHOULDER_ROOT_R = 0.028
SHOULDER_LEN = 0.31
SHOULDER_W = 0.08
ELBOW_R = 0.05
FOREARM_ROOT_R = 0.038
TIP_R = 0.042

FOREARM_LEN = 0.26
FOREARM_W = 0.065
TOOL_PAD_L = 0.08
TOOL_PAD_D = 0.05


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _side_plate_body_shape() -> cq.Workplane:
    body = _cq_box((PLATE_W, PLATE_T, PLATE_H), (0.0, PLATE_T / 2.0, PLATE_H / 2.0))
    body = body.union(_cq_box((PLATE_W * 0.82, 0.08, 0.09), (0.0, -0.04, 0.045)))
    body = body.union(_cq_box((PLATE_W * 0.30, 0.05, 0.08), (0.0, -0.025, PLATE_H - 0.04)))
    for rib_x in (-0.30, 0.30):
        body = body.union(_cq_box((0.055, 0.05, 0.42), (rib_x, -0.025, PLATE_H / 2.0)))

    body = body.cut(
        _cq_box((PLATE_W * 0.42, PLATE_T + 0.012, 0.15), (0.0, PLATE_T / 2.0, PLATE_H * 0.62))
    )
    body = body.cut(
        _cq_box((PLATE_W * 0.54, PLATE_T + 0.012, 0.08), (0.0, PLATE_T / 2.0, 0.13))
    )
    return body.edges("|Z").fillet(0.006)


def _carriage_body_shape() -> cq.Workplane:
    rail_center_local_y = (RAIL_OUTER_Y + RAIL_T / 2.0) - SLIDE_Y
    pad_center_y = rail_center_local_y + (RAIL_T + CARRIAGE_PAD_D) / 2.0

    body = _cq_box((CARRIAGE_W, CARRIAGE_HEAD_D, CARRIAGE_H), (0.0, 0.0, 0.0))
    body = body.union(_cq_box((CARRIAGE_SPINE_W, CARRIAGE_SPINE_D, CARRIAGE_SPINE_H), (0.0, pad_center_y / 2.0, 0.0)))
    body = body.union(_cq_box((GUIDE_L, CARRIAGE_PAD_D, GUIDE_H), (0.0, pad_center_y, RAIL_Z_OFFSET)))
    body = body.union(_cq_box((GUIDE_L, CARRIAGE_PAD_D, GUIDE_H), (0.0, pad_center_y, -RAIL_Z_OFFSET)))
    body = body.union(
        _cq_box((CARRIAGE_STRUT_W, CARRIAGE_STRUT_D, CARRIAGE_STRUT_H), (0.0, (pad_center_y - 0.01) / 2.0, 0.05))
    )
    body = body.union(
        _cq_box((CARRIAGE_STRUT_W, CARRIAGE_STRUT_D, CARRIAGE_STRUT_H), (0.0, (pad_center_y - 0.01) / 2.0, -0.05))
    )
    body = body.cut(_cq_box((0.05, 0.03, 0.10), (0.0, -0.004, 0.0)))
    return body.edges("|Z").fillet(0.005)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_transfer_slide")

    model.material("plate_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("rail_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("carriage_gray", rgba=(0.45, 0.48, 0.52, 1.0))
    model.material("arm_blue", rgba=(0.18, 0.36, 0.62, 1.0))
    model.material("arm_silver", rgba=(0.76, 0.78, 0.80, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((PLATE_W, PLATE_T, PLATE_H)),
        origin=Origin(xyz=(0.0, PLATE_T / 2.0, PLATE_H / 2.0)),
        material="plate_steel",
        name="plate_body",
    )
    side_plate.visual(
        Box((PLATE_W * 0.82, 0.08, 0.09)),
        origin=Origin(xyz=(0.0, -0.04, 0.045)),
        material="plate_steel",
        name="base_foot",
    )
    side_plate.visual(
        Box((PLATE_W * 0.30, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.025, PLATE_H - 0.04)),
        material="plate_steel",
        name="top_cap",
    )
    side_plate.visual(
        Box((RAIL_LEN, RAIL_T, RAIL_H)),
        origin=Origin(xyz=(0.0, RAIL_OUTER_Y - RAIL_T / 2.0, PLATE_H / 2.0 + RAIL_Z_OFFSET)),
        material="rail_steel",
        name="rail_upper",
    )
    side_plate.visual(
        Box((RAIL_LEN, RAIL_T, RAIL_H)),
        origin=Origin(xyz=(0.0, RAIL_OUTER_Y - RAIL_T / 2.0, PLATE_H / 2.0 - RAIL_Z_OFFSET)),
        material="rail_steel",
        name="rail_lower",
    )

    carriage = model.part("carriage")
    guide_center_local_y = (RAIL_OUTER_Y + CARRIAGE_PAD_D / 2.0) - SLIDE_Y
    strut_center_y = (guide_center_local_y - CARRIAGE_HEAD_D / 2.0) / 2.0 - 0.002
    carriage.visual(
        Box((CARRIAGE_W, CARRIAGE_HEAD_D, CARRIAGE_H)),
        origin=Origin(xyz=(0.0, -CARRIAGE_HEAD_D / 2.0, 0.0)),
        material="carriage_gray",
        name="carriage_head",
    )
    carriage.visual(
        Box((CARRIAGE_SPINE_W, CARRIAGE_SPINE_D, CARRIAGE_SPINE_H)),
        origin=Origin(xyz=(0.0, guide_center_local_y / 2.0 - 0.008, 0.0)),
        material="carriage_gray",
        name="carriage_spine",
    )
    carriage.visual(
        Box((GUIDE_L, CARRIAGE_PAD_D, GUIDE_H)),
        origin=Origin(xyz=(0.0, guide_center_local_y, RAIL_Z_OFFSET)),
        material="rail_steel",
        name="guide_upper",
    )
    carriage.visual(
        Box((GUIDE_L, CARRIAGE_PAD_D, GUIDE_H)),
        origin=Origin(xyz=(0.0, guide_center_local_y, -RAIL_Z_OFFSET)),
        material="rail_steel",
        name="guide_lower",
    )
    carriage.visual(
        Box((CARRIAGE_STRUT_W, CARRIAGE_STRUT_D, CARRIAGE_STRUT_H)),
        origin=Origin(xyz=(0.0, strut_center_y, 0.07)),
        material="carriage_gray",
        name="upper_strut",
    )
    carriage.visual(
        Box((CARRIAGE_STRUT_W, CARRIAGE_STRUT_D, CARRIAGE_STRUT_H)),
        origin=Origin(xyz=(0.0, strut_center_y, -0.07)),
        material="carriage_gray",
        name="lower_strut",
    )

    shoulder = model.part("shoulder_link")
    shoulder_web_len = SHOULDER_LEN - SHOULDER_ROOT_R - ELBOW_R
    shoulder.visual(
        Cylinder(radius=SHOULDER_ROOT_R, length=ARM_PLATE_T),
        origin=Origin(xyz=(0.0, 0.0, -ARM_PLATE_T / 2.0)),
        material="arm_blue",
        name="root_disk",
    )
    shoulder.visual(
        Box((SHOULDER_W, shoulder_web_len, ARM_PLATE_T)),
        origin=Origin(
            xyz=(0.0, (SHOULDER_ROOT_R + SHOULDER_LEN - ELBOW_R) / 2.0, -ARM_PLATE_T / 2.0)
        ),
        material="arm_blue",
        name="web",
    )
    shoulder.visual(
        Cylinder(radius=ELBOW_R, length=ARM_PLATE_T),
        origin=Origin(xyz=(0.0, SHOULDER_LEN, -ARM_PLATE_T / 2.0)),
        material="arm_blue",
        name="elbow_disk",
    )

    forearm = model.part("forearm_link")
    forearm_web_len = FOREARM_LEN - FOREARM_ROOT_R - TIP_R
    forearm.visual(
        Cylinder(radius=FOREARM_ROOT_R, length=ARM_PLATE_T),
        origin=Origin(xyz=(0.0, 0.0, ARM_PLATE_T / 2.0)),
        material="arm_silver",
        name="root_disk",
    )
    forearm.visual(
        Box((FOREARM_W, forearm_web_len, ARM_PLATE_T)),
        origin=Origin(
            xyz=(0.0, (FOREARM_ROOT_R + FOREARM_LEN - TIP_R) / 2.0, ARM_PLATE_T / 2.0)
        ),
        material="arm_silver",
        name="web",
    )
    forearm.visual(
        Cylinder(radius=TIP_R, length=ARM_PLATE_T),
        origin=Origin(xyz=(0.0, FOREARM_LEN, ARM_PLATE_T / 2.0)),
        material="arm_silver",
        name="tip_disk",
    )
    forearm.visual(
        Box((TOOL_PAD_D, TOOL_PAD_L, ARM_PLATE_T)),
        origin=Origin(xyz=(0.0, FOREARM_LEN + TOOL_PAD_L / 2.0 - 0.012, ARM_PLATE_T / 2.0)),
        material="arm_silver",
        name="tool_pad",
    )

    model.articulation(
        "slide_axis",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(SLIDE_X0, SLIDE_Y, PLATE_H / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_STROKE, effort=450.0, velocity=0.35),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.0, SHOULDER_ROOT_R, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.25, upper=1.25, effort=80.0, velocity=1.4),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.0, SHOULDER_LEN, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.2, upper=2.2, effort=55.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder_link")
    forearm = object_model.get_part("forearm_link")

    slide = object_model.get_articulation("slide_axis")
    shoulder_joint = object_model.get_articulation("carriage_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forearm")

    rail_upper = side_plate.get_visual("rail_upper")
    guide_upper = carriage.get_visual("guide_upper")
    carriage_body = carriage.get_visual("carriage_head")
    shoulder_root = shoulder.get_visual("root_disk")
    shoulder_elbow_disk = shoulder.get_visual("elbow_disk")
    forearm_root = forearm.get_visual("root_disk")

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

    ctx.check(
        "expected articulated topology",
        slide.articulation_type == ArticulationType.PRISMATIC
        and shoulder_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.articulation_type == ArticulationType.REVOLUTE,
        "The transfer slide must be prismatic with two carried revolute arm joints.",
    )
    ctx.check(
        "joint axes are mechanically credible",
        slide.axis == (1.0, 0.0, 0.0)
        and shoulder_joint.axis == (0.0, 0.0, 1.0)
        and elbow_joint.axis == (0.0, 0.0, 1.0),
        "Expected horizontal slide travel and vertical-axis shoulder/elbow pivots.",
    )

    ctx.expect_contact(
        carriage,
        side_plate,
        elem_a=guide_upper,
        elem_b=rail_upper,
        name="upper carriage guide rides on upper rail",
    )
    with ctx.pose({slide: SLIDE_STROKE}):
        ctx.expect_contact(
            carriage,
            side_plate,
            elem_a=guide_upper,
            elem_b=rail_upper,
            name="upper carriage guide stays on upper rail at full stroke",
        )
        ctx.expect_within(
            carriage,
            side_plate,
            axes="x",
            inner_elem=guide_upper,
            outer_elem=rail_upper,
            name="upper guide remains within rail travel length",
        )

    ctx.expect_contact(
        carriage,
        shoulder,
        elem_a=carriage_body,
        elem_b=shoulder_root,
        name="shoulder is carried by carriage front face",
    )
    ctx.expect_contact(
        shoulder,
        forearm,
        elem_a=shoulder_elbow_disk,
        elem_b=forearm_root,
        name="forearm is supported by elbow joint stack",
    )

    with ctx.pose({slide: 0.38, shoulder_joint: 0.9, elbow_joint: -1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no collisions in representative transfer pose")
        ctx.expect_gap(
            shoulder,
            side_plate,
            axis="y",
            min_gap=0.04,
            name="shoulder remains outboard of side plate",
        )
        ctx.expect_gap(
            forearm,
            side_plate,
            axis="y",
            min_gap=0.08,
            name="forearm remains clear of the side plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
