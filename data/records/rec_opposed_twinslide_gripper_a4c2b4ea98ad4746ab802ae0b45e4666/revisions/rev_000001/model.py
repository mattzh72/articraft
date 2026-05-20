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


BODY_W = 0.024
BODY_D = 0.024
BODY_H = 0.032
BODY_Y = -0.036
BODY_REAR_FOOT_W = 0.014
BODY_REAR_FOOT_D = 0.034
BODY_REAR_FOOT_H = 0.009
BODY_REAR_FOOT_Y = -0.016
BODY_TOP_CAP_W = 0.030
BODY_TOP_CAP_D = 0.018
BODY_TOP_CAP_H = 0.008
BODY_TOP_CAP_Y = -0.037
BODY_FRONT_BRIDGE_W = 0.012
BODY_FRONT_BRIDGE_D = 0.012
BODY_FRONT_BRIDGE_H = 0.009
BODY_FRONT_BRIDGE_Y = -0.002

GUIDE_LEN = 0.058
GUIDE_W = 0.024
GUIDE_Y = -0.007
GUIDE_BASE_H = 0.008
GUIDE_COVER_W = 0.006
GUIDE_COVER_H = 0.002
GUIDE_RAIL_W = 0.008
GUIDE_RAIL_H = 0.006
GUIDE_CONTACT_LEN = 0.046
GUIDE_BODY_SEAT_X = 0.006
BODY_SIDE_MOUNT_D = 0.020
BODY_SIDE_MOUNT_H = 0.008

SLIDER_LEN = 0.024
SLIDER_W = 0.026
SLIDER_H = 0.014
SLIDER_SLOT_LEN = 0.028
SLIDER_SLOT_W = 0.011
SLIDER_SLOT_H = 0.007
SLIDER_TOP_PAD_LEN = 0.018
SLIDER_TOP_PAD_W = 0.014
SLIDER_TOP_PAD_H = 0.003

FINGER_BASE_Z = 0.006
FINGER_H = 0.018
JAW_PAD_W = 0.003
JAW_PAD_D = 0.009
JAW_PAD_H = 0.022
JAW_PAD_Z = 0.004

LEFT_GUIDE_X = -(BODY_W / 2.0 + GUIDE_LEN / 2.0 + GUIDE_BODY_SEAT_X)
RIGHT_GUIDE_X = -LEFT_GUIDE_X
LEFT_SLIDER_OPEN_X = 0.004
RIGHT_SLIDER_OPEN_X = -0.004
JAW_TRAVEL = 0.016


def _box(sx: float, sy: float, sz: float, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XY").box(sx, sy, sz, centered=(True, True, False)).translate((x, y, z))


def _cylinder(radius: float, height: float, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z))


def _body_shape():
    body = _box(BODY_W, BODY_D, BODY_H, y=BODY_Y)
    body = body.union(
        _box(
            BODY_REAR_FOOT_W,
            BODY_REAR_FOOT_D,
            BODY_REAR_FOOT_H,
            y=BODY_REAR_FOOT_Y,
        )
    )
    body = body.union(
        _box(
            BODY_TOP_CAP_W,
            BODY_TOP_CAP_D,
            BODY_TOP_CAP_H,
            y=BODY_TOP_CAP_Y,
            z=BODY_H,
        )
    )
    body = body.union(
        _box(
            BODY_FRONT_BRIDGE_W,
            BODY_FRONT_BRIDGE_D,
            BODY_FRONT_BRIDGE_H,
            y=BODY_FRONT_BRIDGE_Y,
        )
    )
    side_mount_x = BODY_W / 2.0 + GUIDE_BODY_SEAT_X / 2.0
    body = body.union(
        _box(
            GUIDE_BODY_SEAT_X,
            BODY_SIDE_MOUNT_D,
            BODY_SIDE_MOUNT_H,
            x=-side_mount_x,
            y=GUIDE_Y,
        )
    )
    body = body.union(
        _box(
            GUIDE_BODY_SEAT_X,
            BODY_SIDE_MOUNT_D,
            BODY_SIDE_MOUNT_H,
            x=side_mount_x,
            y=GUIDE_Y,
        )
    )
    body = body.cut(_box(0.005, 0.028, 0.014, x=-0.0095, y=-0.020, z=0.010))
    body = body.cut(_box(0.005, 0.028, 0.014, x=0.0095, y=-0.020, z=0.010))
    body = body.cut(_box(0.010, 0.014, 0.010, y=-0.004, z=0.010))
    return body


def _guide_shape():
    cover_y = GUIDE_W / 2.0 - GUIDE_COVER_W / 2.0
    guide = _box(GUIDE_LEN, GUIDE_W, GUIDE_BASE_H)
    guide = guide.union(
        _box(
            GUIDE_CONTACT_LEN,
            GUIDE_COVER_W,
            GUIDE_COVER_H,
            y=cover_y,
            z=GUIDE_BASE_H,
        )
    )
    guide = guide.union(
        _box(
            GUIDE_CONTACT_LEN,
            GUIDE_COVER_W,
            GUIDE_COVER_H,
            y=-cover_y,
            z=GUIDE_BASE_H,
        )
    )
    guide = guide.union(
        _box(
            GUIDE_CONTACT_LEN * 0.92,
            GUIDE_RAIL_W,
            GUIDE_RAIL_H,
            z=GUIDE_BASE_H,
        )
    )
    return guide


def _finger_points(side: int) -> list[tuple[float, float]]:
    points = [
        (-0.004, 0.012),
        (0.016, 0.012),
        (0.019, 0.026),
        (0.022, 0.050),
        (0.011, 0.050),
        (0.008, 0.028),
        (-0.001, 0.014),
    ]
    return [(side * x, y) for x, y in points]


def _slider_shape(side: int):
    slider = _box(SLIDER_LEN, SLIDER_W, SLIDER_H)
    slider = slider.cut(_box(SLIDER_SLOT_LEN, SLIDER_SLOT_W, SLIDER_SLOT_H))
    slider = slider.union(
        _box(
            SLIDER_TOP_PAD_LEN,
            SLIDER_TOP_PAD_W,
            SLIDER_TOP_PAD_H,
            x=side * 0.005,
            y=-0.001,
            z=SLIDER_H,
        )
    )

    finger = (
        cq.Workplane("XY")
        .polyline(_finger_points(side))
        .close()
        .extrude(FINGER_H)
        .translate((0.0, 0.0, FINGER_BASE_Z))
    )
    slider = slider.union(finger)
    slider = slider.union(
        _box(
            JAW_PAD_W,
            JAW_PAD_D,
            JAW_PAD_H,
            x=side * 0.0235,
            y=0.0455,
            z=JAW_PAD_Z,
        )
    )

    bolt_x = side * 0.008
    slider = slider.union(_cylinder(0.0020, 0.010, x=bolt_x, y=-0.002, z=0.004))
    slider = slider.union(_cylinder(0.0038, 0.003, x=bolt_x, y=-0.002, z=SLIDER_H))
    return slider


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_parallel_gripper")

    body_mat = model.material("body_anodized", rgba=(0.17, 0.18, 0.20, 1.0))
    guide_mat = model.material("guide_steel", rgba=(0.61, 0.64, 0.69, 1.0))
    slider_mat = model.material("slider_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    center_body = model.part("center_body")
    center_body.visual(
        mesh_from_cadquery(_body_shape(), "center_body"),
        material=body_mat,
        name="body_shell",
    )

    left_guide = model.part("left_guide")
    left_guide.visual(
        mesh_from_cadquery(_guide_shape(), "left_guide"),
        material=guide_mat,
        name="guide_shell",
    )

    right_guide = model.part("right_guide")
    right_guide.visual(
        mesh_from_cadquery(_guide_shape(), "right_guide"),
        material=guide_mat,
        name="guide_shell",
    )

    left_slider = model.part("left_slider")
    left_slider.visual(
        mesh_from_cadquery(_slider_shape(1), "left_slider"),
        material=slider_mat,
        name="slider_shell",
    )

    right_slider = model.part("right_slider")
    right_slider.visual(
        mesh_from_cadquery(_slider_shape(-1), "right_slider"),
        material=slider_mat,
        name="slider_shell",
    )

    model.articulation(
        "body_to_left_guide",
        ArticulationType.FIXED,
        parent=center_body,
        child=left_guide,
        origin=Origin(xyz=(LEFT_GUIDE_X, GUIDE_Y, 0.0)),
    )
    model.articulation(
        "body_to_right_guide",
        ArticulationType.FIXED,
        parent=center_body,
        child=right_guide,
        origin=Origin(xyz=(RIGHT_GUIDE_X, GUIDE_Y, 0.0)),
    )
    model.articulation(
        "left_guide_to_left_slider",
        ArticulationType.PRISMATIC,
        parent=left_guide,
        child=left_slider,
        origin=Origin(xyz=(LEFT_SLIDER_OPEN_X, 0.0, GUIDE_BASE_H + GUIDE_COVER_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "right_guide_to_right_slider",
        ArticulationType.PRISMATIC,
        parent=right_guide,
        child=right_slider,
        origin=Origin(xyz=(RIGHT_SLIDER_OPEN_X, 0.0, GUIDE_BASE_H + GUIDE_COVER_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_body = object_model.get_part("center_body")
    left_guide = object_model.get_part("left_guide")
    right_guide = object_model.get_part("right_guide")
    left_slider = object_model.get_part("left_slider")
    right_slider = object_model.get_part("right_slider")
    left_stage = object_model.get_articulation("left_guide_to_left_slider")
    right_stage = object_model.get_articulation("right_guide_to_right_slider")

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
        "mirrored_slider_axes",
        tuple(left_stage.axis) == (1.0, 0.0, 0.0) and tuple(right_stage.axis) == (-1.0, 0.0, 0.0),
        details=f"left axis={left_stage.axis}, right axis={right_stage.axis}",
    )
    ctx.expect_contact(center_body, left_guide, name="left guide is mounted to backbone")
    ctx.expect_contact(center_body, right_guide, name="right guide is mounted to backbone")
    ctx.expect_contact(left_slider, left_guide, name="left slider rides on its guide covers")
    ctx.expect_contact(right_slider, right_guide, name="right slider rides on its guide covers")
    ctx.expect_gap(
        right_slider,
        left_slider,
        axis="x",
        min_gap=0.034,
        max_gap=0.038,
        name="rest pose shows open work zone",
    )

    with ctx.pose({left_stage: JAW_TRAVEL, right_stage: JAW_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at full close")
        ctx.expect_contact(left_slider, left_guide, name="left slider stays grounded when closed")
        ctx.expect_contact(right_slider, right_guide, name="right slider stays grounded when closed")
        ctx.expect_gap(
            right_slider,
            left_slider,
            axis="x",
            min_gap=0.003,
            max_gap=0.006,
            name="jaw tips close to a small rectangular work zone",
        )
        ctx.expect_overlap(
            left_slider,
            right_slider,
            axes="yz",
            min_overlap=0.012,
            name="closed jaw tips stay aligned in height and depth",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
