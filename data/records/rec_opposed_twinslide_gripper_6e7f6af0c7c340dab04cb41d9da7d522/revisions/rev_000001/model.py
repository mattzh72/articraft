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


BODY_W = 0.068
BODY_D = 0.074
BODY_H = 0.054
MOUNT_W = 0.038
MOUNT_D = 0.050
MOUNT_H = 0.012

SLIDE_SLOT_LEN = 0.0245
SLIDE_SLOT_D = 0.022
SLIDE_SLOT_H = 0.018
SLIDE_SLOT_Y = 0.012
SIDE_OPEN_MARGIN = 0.004

SLIDE_TONGUE_LEN = 0.023
SLIDE_TONGUE_D = SLIDE_SLOT_D
SLIDE_TONGUE_H = SLIDE_SLOT_H
SLIDE_TONGUE_X_OFFSET = 0.014
SLIDE_TONGUE_Y_OFFSET = -0.033

CARRIAGE_OPEN_X = 0.048
CARRIAGE_ORIGIN_Y = 0.045
JAW_TRAVEL = 0.012


def _body_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    mount_pad = cq.Workplane("XY").box(MOUNT_W, MOUNT_D, MOUNT_H).translate((0.0, -0.006, -BODY_H / 2))

    body = housing.union(mount_pad)

    top_relief = cq.Workplane("XY").box(0.030, 0.024, 0.008).translate((0.0, -0.006, BODY_H / 2 - 0.004))
    left_channel = cq.Workplane("XY").box(
        SLIDE_SLOT_LEN + SIDE_OPEN_MARGIN,
        SLIDE_SLOT_D,
        SLIDE_SLOT_H,
    ).translate(
        (-BODY_W / 2 + SLIDE_SLOT_LEN / 2 - SIDE_OPEN_MARGIN / 2, SLIDE_SLOT_Y, 0.0)
    )
    right_channel = cq.Workplane("XY").box(
        SLIDE_SLOT_LEN + SIDE_OPEN_MARGIN,
        SLIDE_SLOT_D,
        SLIDE_SLOT_H,
    ).translate(
        (BODY_W / 2 - SLIDE_SLOT_LEN / 2 + SIDE_OPEN_MARGIN / 2, SLIDE_SLOT_Y, 0.0)
    )
    left_carriage_relief = cq.Workplane("XY").box(0.020, 0.028, 0.040).translate(
        (-BODY_W / 2 + 0.010, 0.023, 0.007)
    )
    right_carriage_relief = cq.Workplane("XY").box(0.020, 0.028, 0.040).translate(
        (BODY_W / 2 - 0.010, 0.023, 0.007)
    )

    body = (
        body.cut(top_relief)
        .cut(left_channel)
        .cut(right_channel)
        .cut(left_carriage_relief)
        .cut(right_carriage_relief)
    )
    body = (
        body.faces("<Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.012, -0.010), (0.012, -0.010)])
        .hole(0.005)
    )
    return body


def _carriage_head_shape(side_sign: float) -> cq.Workplane:
    carriage = cq.Workplane("XY").box(0.022, 0.020, 0.026)
    bridge = cq.Workplane("XY").box(0.028, 0.026, 0.014).translate((side_sign * 0.008, -0.021, 0.0))
    finger_arm = cq.Workplane("XY").box(0.034, 0.012, 0.010).translate((side_sign * 0.017, 0.010, 0.006))
    finger_pad = cq.Workplane("XY").box(0.006, 0.010, 0.026).translate((side_sign * 0.031, 0.012, 0.013))
    return carriage.union(bridge).union(finger_arm).union(finger_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carriage_block_opposed_gripper")

    model.material("body_anodized", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("jaw_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        name="body_shell",
        material="body_anodized",
    )
    body.inertial = Inertial.from_geometry(Box((BODY_W, BODY_D, BODY_H + MOUNT_H)), mass=1.9)

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        mesh_from_cadquery(_carriage_head_shape(1.0), "left_carriage_head"),
        name="left_carriage_head",
        material="jaw_black",
    )
    left_carriage.visual(
        Box((SLIDE_TONGUE_LEN, SLIDE_TONGUE_D, SLIDE_TONGUE_H)),
        origin=Origin(xyz=(SLIDE_TONGUE_X_OFFSET, SLIDE_TONGUE_Y_OFFSET, 0.0)),
        name="left_slide_tongue",
        material="rail_steel",
    )
    left_carriage.inertial = Inertial.from_geometry(
        Box((0.050, 0.070, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(0.010, -0.012, 0.006)),
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        mesh_from_cadquery(_carriage_head_shape(-1.0), "right_carriage_head"),
        name="right_carriage_head",
        material="jaw_black",
    )
    right_carriage.visual(
        Box((SLIDE_TONGUE_LEN, SLIDE_TONGUE_D, SLIDE_TONGUE_H)),
        origin=Origin(xyz=(-SLIDE_TONGUE_X_OFFSET, SLIDE_TONGUE_Y_OFFSET, 0.0)),
        name="right_slide_tongue",
        material="rail_steel",
    )
    right_carriage.inertial = Inertial.from_geometry(
        Box((0.050, 0.070, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(-0.010, -0.012, 0.006)),
    )

    model.articulation(
        "body_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(-CARRIAGE_OPEN_X, CARRIAGE_ORIGIN_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=JAW_TRAVEL),
    )
    model.articulation(
        "body_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(CARRIAGE_OPEN_X, CARRIAGE_ORIGIN_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=JAW_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("body")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_joint = object_model.get_articulation("body_to_left_carriage")
    right_joint = object_model.get_articulation("body_to_right_carriage")

    left_head = left_carriage.get_visual("left_carriage_head")
    right_head = right_carriage.get_visual("right_carriage_head")
    left_tongue = left_carriage.get_visual("left_slide_tongue")
    right_tongue = right_carriage.get_visual("right_slide_tongue")

    ctx.check("body exists", body is not None)
    ctx.check("left carriage exists", left_carriage is not None)
    ctx.check("right carriage exists", right_carriage is not None)

    ctx.allow_overlap(
        body,
        left_carriage,
        elem_a="body_shell",
        elem_b="left_slide_tongue",
        reason="The left steel tongue is intentionally represented as sliding inside a simplified guide bore in the grounded center body proxy.",
    )
    ctx.allow_overlap(
        body,
        right_carriage,
        elem_a="body_shell",
        elem_b="right_slide_tongue",
        reason="The right steel tongue is intentionally represented as sliding inside a simplified guide bore in the grounded center body proxy.",
    )

    ctx.expect_within(
        left_carriage,
        body,
        axes="yz",
        inner_elem=left_tongue,
        name="left tongue stays inside the body guide window at rest",
    )
    ctx.expect_within(
        right_carriage,
        body,
        axes="yz",
        inner_elem=right_tongue,
        name="right tongue stays inside the body guide window at rest",
    )
    ctx.expect_overlap(
        left_carriage,
        body,
        axes="x",
        elem_a=left_tongue,
        min_overlap=0.010,
        name="left tongue remains engaged in the body at rest",
    )
    ctx.expect_overlap(
        right_carriage,
        body,
        axes="x",
        elem_a=right_tongue,
        min_overlap=0.010,
        name="right tongue remains engaged in the body at rest",
    )
    ctx.expect_gap(
        right_carriage,
        left_carriage,
        axis="x",
        positive_elem=right_head,
        negative_elem=left_head,
        min_gap=0.027,
        name="jaw fingers begin with an open pickup throat",
    )

    left_rest = ctx.part_world_position(left_carriage)
    right_rest = ctx.part_world_position(right_carriage)
    with ctx.pose({left_joint: JAW_TRAVEL, right_joint: JAW_TRAVEL}):
        ctx.expect_within(
            left_carriage,
            body,
            axes="yz",
            inner_elem=left_tongue,
            name="left tongue stays guided at full close",
        )
        ctx.expect_within(
            right_carriage,
            body,
            axes="yz",
            inner_elem=right_tongue,
            name="right tongue stays guided at full close",
        )
        ctx.expect_overlap(
            left_carriage,
            body,
            axes="x",
            elem_a=left_tongue,
            min_overlap=0.022,
            name="left tongue keeps retained insertion when closed",
        )
        ctx.expect_overlap(
            right_carriage,
            body,
            axes="x",
            elem_a=right_tongue,
            min_overlap=0.022,
            name="right tongue keeps retained insertion when closed",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem=right_head,
            negative_elem=left_head,
            min_gap=0.004,
            max_gap=0.008,
            name="jaw fingers leave a narrow pickup gap when closed",
        )
        left_closed = ctx.part_world_position(left_carriage)
        right_closed = ctx.part_world_position(right_carriage)

    ctx.check(
        "left carriage closes toward center",
        left_rest is not None and left_closed is not None and left_closed[0] > left_rest[0] + 0.010,
        details=f"rest={left_rest}, closed={left_closed}",
    )
    ctx.check(
        "right carriage closes toward center",
        right_rest is not None and right_closed is not None and right_closed[0] < right_rest[0] - 0.010,
        details=f"rest={right_rest}, closed={right_closed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
