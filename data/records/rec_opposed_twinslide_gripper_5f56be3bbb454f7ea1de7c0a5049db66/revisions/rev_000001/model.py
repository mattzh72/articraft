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


BODY_W = 0.032
BODY_D = 0.034
BODY_H = 0.076

MOUNT_W = 0.022
MOUNT_D = 0.010
MOUNT_H = 0.050

GUIDE_LEN = 0.028
GUIDE_D = 0.006
GUIDE_H = 0.006
GUIDE_X = 0.026
GUIDE_Y = BODY_D / 2.0 + GUIDE_D / 2.0 - 0.0002
GUIDE_Z = -0.010

TRAVEL = 0.009

SHOE_L = 0.018
SHOE_D = 0.016
SHOE_H = 0.028
SHOE_Y = BODY_D / 2.0 + SHOE_D / 2.0 + 0.0012
SHOE_Z = GUIDE_H / 2.0 + (SHOE_H * 0.58) / 2.0


def _housing_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    housing = housing.edges("|Z").fillet(0.0025)
    front_relief = cq.Workplane("XY").box(0.014, 0.012, 0.040).translate(
        (0.0, BODY_D / 2.0 - 0.006, -0.008)
    )
    return housing.cut(front_relief)


def _mount_pad_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(MOUNT_W, MOUNT_D, MOUNT_H)


def _guide_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(GUIDE_LEN, GUIDE_D, GUIDE_H)
    return rail.edges("|Z").fillet(0.001)


def _shoe_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(SHOE_L, SHOE_D, SHOE_H * 0.58)
    cap = cq.Workplane("XY").box(0.011, SHOE_D * 0.72, SHOE_H * 0.34).translate(
        (0.0, -0.001, SHOE_H * 0.23)
    )
    shoe = base.union(cap)
    return shoe.edges("|X").fillet(0.001)


def _finger_shape(side_sign: float) -> cq.Workplane:
    arm = cq.Workplane("XY").box(0.010, 0.010, 0.020).translate(
        (side_sign * 0.007, 0.010, -0.001)
    )
    pad = cq.Workplane("XY").box(0.004, 0.012, 0.026).translate(
        (side_sign * 0.011, 0.011, -0.004)
    )
    return arm.union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_body_opposed_slide_gripper")

    body_mat = model.material("body_anodized", rgba=(0.23, 0.25, 0.28, 1.0))
    steel_mat = model.material("guide_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    carriage_mat = model.material("carriage_black", rgba=(0.15, 0.16, 0.18, 1.0))
    finger_mat = model.material("finger_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_shape(), "body_housing"),
        name="housing_shell",
        material=body_mat,
    )
    body.visual(
        mesh_from_cadquery(_mount_pad_shape(), "body_mount_pad"),
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0 + MOUNT_D / 2.0 - 0.0003), 0.0)),
        name="mount_pad",
        material=body_mat,
    )
    body.visual(
        mesh_from_cadquery(_guide_shape(), "left_guide"),
        origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, GUIDE_Z)),
        name="left_guide",
        material=steel_mat,
    )
    body.visual(
        mesh_from_cadquery(_guide_shape(), "right_guide"),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, GUIDE_Z)),
        name="right_guide",
        material=steel_mat,
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        mesh_from_cadquery(_shoe_shape(), "left_carriage_shoe"),
        origin=Origin(xyz=(0.0, SHOE_Y - GUIDE_Y, SHOE_Z)),
        name="left_shoe",
        material=carriage_mat,
    )
    left_carriage.visual(
        mesh_from_cadquery(_finger_shape(1.0), "left_finger"),
        name="left_finger",
        material=finger_mat,
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        mesh_from_cadquery(_shoe_shape(), "right_carriage_shoe"),
        origin=Origin(xyz=(0.0, SHOE_Y - GUIDE_Y, SHOE_Z)),
        name="right_shoe",
        material=carriage_mat,
    )
    right_carriage.visual(
        mesh_from_cadquery(_finger_shape(-1.0), "right_finger"),
        name="right_finger",
        material=finger_mat,
    )

    model.articulation(
        "body_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, GUIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.08,
            lower=0.0,
            upper=TRAVEL,
        ),
    )
    model.articulation(
        "body_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, GUIDE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.08,
            lower=0.0,
            upper=TRAVEL,
        ),
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

    body.get_visual("housing_shell")
    left_guide = body.get_visual("left_guide")
    right_guide = body.get_visual("right_guide")
    left_shoe = left_carriage.get_visual("left_shoe")
    right_shoe = right_carriage.get_visual("right_shoe")
    left_finger = left_carriage.get_visual("left_finger")
    right_finger = right_carriage.get_visual("right_finger")

    ctx.expect_overlap(
        left_carriage,
        body,
        axes="x",
        elem_a=left_shoe,
        elem_b=left_guide,
        min_overlap=0.014,
        name="left carriage stays retained on its guide at rest",
    )
    ctx.expect_overlap(
        right_carriage,
        body,
        axes="x",
        elem_a=right_shoe,
        elem_b=right_guide,
        min_overlap=0.014,
        name="right carriage stays retained on its guide at rest",
    )
    ctx.expect_overlap(
        left_carriage,
        body,
        axes="y",
        elem_a=left_shoe,
        elem_b=left_guide,
        min_overlap=0.004,
        name="left shoe stays laterally engaged with the guide rail",
    )
    ctx.expect_overlap(
        right_carriage,
        body,
        axes="y",
        elem_a=right_shoe,
        elem_b=right_guide,
        min_overlap=0.004,
        name="right shoe stays laterally engaged with the guide rail",
    )
    ctx.expect_gap(
        left_carriage,
        body,
        axis="z",
        positive_elem=left_shoe,
        negative_elem=left_guide,
        max_penetration=1e-6,
        max_gap=0.0005,
        name="left shoe sits on top of the left guide rail",
    )
    ctx.expect_gap(
        right_carriage,
        body,
        axis="z",
        positive_elem=right_shoe,
        negative_elem=right_guide,
        max_penetration=1e-6,
        max_gap=0.0005,
        name="right shoe sits on top of the right guide rail",
    )
    ctx.expect_gap(
        right_carriage,
        left_carriage,
        axis="x",
        positive_elem=right_finger,
        negative_elem=left_finger,
        min_gap=0.020,
        max_gap=0.030,
        name="open fingers leave a usable pickup gap",
    )

    left_rest = ctx.part_world_position(left_carriage)
    right_rest = ctx.part_world_position(right_carriage)
    with ctx.pose({left_joint: TRAVEL, right_joint: TRAVEL}):
        ctx.expect_overlap(
            left_carriage,
            body,
            axes="x",
            elem_a=left_shoe,
            elem_b=left_guide,
            min_overlap=0.010,
            name="left carriage remains captured at full close",
        )
        ctx.expect_overlap(
            right_carriage,
            body,
            axes="x",
            elem_a=right_shoe,
            elem_b=right_guide,
            min_overlap=0.010,
            name="right carriage remains captured at full close",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem=right_finger,
            negative_elem=left_finger,
            min_gap=0.004,
            max_gap=0.010,
            name="closed fingers stop with a small central gripping gap",
        )
        left_closed = ctx.part_world_position(left_carriage)
        right_closed = ctx.part_world_position(right_carriage)

    ctx.check(
        "left carriage closes inward on +x",
        left_rest is not None
        and left_closed is not None
        and left_closed[0] > left_rest[0] + 0.008,
        details=f"rest={left_rest}, closed={left_closed}",
    )
    ctx.check(
        "right carriage closes inward on -x",
        right_rest is not None
        and right_closed is not None
        and right_closed[0] < right_rest[0] - 0.008,
        details=f"rest={right_rest}, closed={right_closed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
