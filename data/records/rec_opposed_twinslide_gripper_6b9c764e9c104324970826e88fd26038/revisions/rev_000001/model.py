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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_X = 0.120
BODY_Y = 0.096
BODY_Z = 0.148
BODY_WALL = 0.012
HOUSING_Z = 0.132
HOUSING_CENTER_Y = -0.012
HOUSING_CENTER_Z = -0.004

RAIL_INNER_X = 0.012
RAIL_OUTER_X = 0.168
RAIL_CENTER_Y = 0.046
RAIL_CORE_Y = 0.024
RAIL_CORE_Z = 0.022
RAIL_CENTER_Z = 0.036
WEAR_STRIP_Y = 0.020
WEAR_STRIP_Z = 0.003
RAIL_TOTAL_Z = RAIL_CORE_Z

JAW_OPEN_X = 0.117
JAW_TRAVEL = 0.024
JAW_JOINT_Y = RAIL_CENTER_Y
JAW_JOINT_Z = 0.092
CARRIAGE_X = 0.104
CARRIAGE_Y = 0.050
CARRIAGE_Z = 0.050
CARRIAGE_CENTER_Y = 0.014
RUNNER_X = 0.100
RUNNER_Y = 0.024
RUNNER_Z = 0.012
RUNNER_CENTER_Y = 0.000
RUNNER_CENTER_Z = RUNNER_Z / 2.0
CAP_X = 0.042
CAP_Y = 0.036
CAP_Z = 0.008
FINGER_ROOT_X = 0.022
FINGER_ROOT_Y = 0.048
FINGER_ROOT_Z = 0.062
FINGER_TIP_X = 0.014
FINGER_TIP_Y = 0.030
FINGER_TIP_Z = 0.036


def _box(dx: float, dy: float, dz: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate((x, y, z))


def _cyl_pocket(radius: float, depth: float, *, x: float, y: float, top_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(depth)
        .translate((x, y, top_z - (depth / 2.0)))
    )


def _body_shape() -> cq.Workplane:
    housing = _box(BODY_X, BODY_Y, HOUSING_Z, y=HOUSING_CENTER_Y, z=HOUSING_CENTER_Z)
    rear_cavity = _box(
        BODY_X - (2.0 * BODY_WALL),
        BODY_Y - (2.0 * BODY_WALL),
        HOUSING_Z - (2.0 * BODY_WALL),
        y=HOUSING_CENTER_Y - 0.006,
        z=HOUSING_CENTER_Z - 0.004,
    )
    cable_relief = _box(
        BODY_X - 0.032,
        0.036,
        HOUSING_Z - 0.040,
        y=HOUSING_CENTER_Y - (BODY_Y / 2.0) + 0.018,
        z=HOUSING_CENTER_Z,
    )
    front_window = _box(
        BODY_X - 0.020,
        BODY_Y - 0.024,
        0.026,
        y=HOUSING_CENTER_Y + 0.012,
        z=(HOUSING_CENTER_Z + (HOUSING_Z / 2.0)) - 0.014,
    )
    body = housing.cut(rear_cavity).cut(cable_relief).cut(front_window)

    top_cover = _box(
        BODY_X - 0.040,
        BODY_Y - 0.028,
        0.008,
        y=HOUSING_CENTER_Y + 0.002,
        z=(HOUSING_CENTER_Z + (HOUSING_Z / 2.0)) + 0.002,
    )
    rear_pad = _box(
        0.070,
        0.024,
        0.088,
        y=HOUSING_CENTER_Y - (BODY_Y / 2.0) - 0.012,
        z=-0.004,
    )
    body = body.union(top_cover).union(rear_pad)

    rail_length = RAIL_OUTER_X - RAIL_INNER_X
    rail_center_x = (RAIL_OUTER_X + RAIL_INNER_X) / 2.0

    for side in (-1.0, 1.0):
        x_center = side * rail_center_x
        rail_core = _box(
            rail_length,
            RAIL_CORE_Y,
            RAIL_CORE_Z,
            x=x_center,
            y=RAIL_CENTER_Y,
            z=RAIL_CENTER_Z,
        )
        top_strip = _box(
            rail_length,
            WEAR_STRIP_Y,
            WEAR_STRIP_Z,
            x=x_center,
            y=RAIL_CENTER_Y,
            z=RAIL_CENTER_Z + (RAIL_CORE_Z / 2.0) + (WEAR_STRIP_Z / 2.0),
        )
        inner_stop = _box(
            0.014,
            0.032,
            0.014,
            x=side * 0.028,
            y=RAIL_CENTER_Y,
            z=RAIL_CENTER_Z + 0.001,
        )
        outer_stop = _box(
            0.010,
            0.032,
            0.014,
            x=side * 0.174,
            y=RAIL_CENTER_Y,
            z=RAIL_CENTER_Z + 0.001,
        )
        body = body.union(rail_core).union(top_strip).union(inner_stop).union(outer_stop)

    body_top = (HOUSING_CENTER_Z + (HOUSING_Z / 2.0)) + 0.008
    for x_pt, y_pt in (
        (-0.036, HOUSING_CENTER_Y - 0.010),
        (0.036, HOUSING_CENTER_Y - 0.010),
        (-0.036, HOUSING_CENTER_Y + 0.018),
        (0.036, HOUSING_CENTER_Y + 0.018),
        (-0.012, HOUSING_CENTER_Y + 0.004),
        (0.012, HOUSING_CENTER_Y + 0.004),
    ):
        body = body.cut(_cyl_pocket(0.004, 0.004, x=x_pt, y=y_pt, top_z=body_top))

    for x_pt in (-0.020, 0.020):
        body = body.cut(
            _cyl_pocket(
                0.005,
                0.005,
                x=x_pt,
                y=HOUSING_CENTER_Y - (BODY_Y / 2.0) - 0.012,
                top_z=0.040,
            )
        )

    return body


def _jaw_shape(side: float) -> cq.Workplane:
    inner_dir = -side
    outer_dir = side

    runner = _box(RUNNER_X, RUNNER_Y, RUNNER_Z, y=0.0, z=0.0)
    carriage_center_z = (RUNNER_Z / 2.0) + (CARRIAGE_Z / 2.0)
    carriage = _box(CARRIAGE_X, CARRIAGE_Y, CARRIAGE_Z, y=CARRIAGE_CENTER_Y, z=carriage_center_z)
    top_pocket = _box(
        CARRIAGE_X - 0.028,
        0.034,
        0.008,
        y=CARRIAGE_CENTER_Y - 0.004,
        z=carriage_center_z + (CARRIAGE_Z / 2.0) - 0.007,
    )
    underside_relief = _box(
        CARRIAGE_X - 0.024,
        CARRIAGE_Y - 0.018,
        0.014,
        y=CARRIAGE_CENTER_Y + 0.004,
        z=(RUNNER_Z / 2.0) + 0.001,
    )
    jaw = runner.union(carriage).cut(top_pocket).cut(underside_relief)

    cap = _box(
        CAP_X,
        CAP_Y,
        CAP_Z,
        x=outer_dir * 0.020,
        y=CARRIAGE_CENTER_Y - 0.004,
        z=carriage_center_z + (CARRIAGE_Z / 2.0) + (CAP_Z / 2.0) - 0.001,
    )
    jaw = jaw.union(cap)

    finger_root = _box(
        FINGER_ROOT_X,
        FINGER_ROOT_Y,
        FINGER_ROOT_Z,
        x=inner_dir * ((CARRIAGE_X / 2.0) + (FINGER_ROOT_X / 2.0) - 0.001),
        y=CARRIAGE_CENTER_Y + (CARRIAGE_Y / 2.0) + (FINGER_ROOT_Y / 2.0) - 0.010,
        z=0.012,
    )
    finger_tip = _box(
        FINGER_TIP_X,
        FINGER_TIP_Y,
        FINGER_TIP_Z,
        x=inner_dir * ((CARRIAGE_X / 2.0) + FINGER_ROOT_X + (FINGER_TIP_X / 2.0) - 0.002),
        y=CARRIAGE_CENTER_Y + (CARRIAGE_Y / 2.0) + FINGER_ROOT_Y + (FINGER_TIP_Y / 2.0) - 0.014,
        z=-0.006,
    )
    finger_notch = _box(
        0.018,
        0.020,
        0.018,
        x=inner_dir * ((CARRIAGE_X / 2.0) + 0.018),
        y=CARRIAGE_CENTER_Y + (CARRIAGE_Y / 2.0) + FINGER_ROOT_Y + 0.002,
        z=0.028,
    )
    grip_flat = _box(
        0.010,
        0.010,
        0.022,
        x=inner_dir * ((CARRIAGE_X / 2.0) + FINGER_ROOT_X + 0.003),
        y=CARRIAGE_CENTER_Y + (CARRIAGE_Y / 2.0) + FINGER_ROOT_Y + 0.014,
        z=-0.008,
    )
    jaw = jaw.union(finger_root).union(finger_tip).cut(finger_notch).cut(grip_flat)

    cap_top = carriage_center_z + (CARRIAGE_Z / 2.0) + CAP_Z + 0.001
    for x_offset, y_pt in (
        (0.011, CARRIAGE_CENTER_Y - 0.014),
        (0.028, CARRIAGE_CENTER_Y - 0.014),
        (0.011, CARRIAGE_CENTER_Y + 0.004),
        (0.028, CARRIAGE_CENTER_Y + 0.004),
    ):
        jaw = jaw.cut(
            _cyl_pocket(
                0.003,
                0.004,
                x=outer_dir * x_offset,
                y=y_pt,
                top_z=cap_top,
            )
        )

    for x_offset in (0.010, 0.024):
        jaw = jaw.cut(
            _cyl_pocket(
                0.0025,
                0.003,
                x=inner_dir * ((CARRIAGE_X / 2.0) + x_offset),
                y=CARRIAGE_CENTER_Y + (CARRIAGE_Y / 2.0) + 0.014,
                top_z=0.040,
            )
        )

    return jaw


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_block_gripper")

    model.material("housing_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("jaw_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("wear_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("fastener_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    body = model.part("body")
    housing_front_y = 0.036
    rail_y = 0.049
    rail_front_y = rail_y + 0.013
    rail_length = RAIL_OUTER_X - RAIL_INNER_X
    rail_center_x = (RAIL_OUTER_X + RAIL_INNER_X) / 2.0

    body.visual(
        Box((BODY_X, BODY_Y, HOUSING_Z)),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, 0.0)),
        material="housing_gray",
        name="body_shell",
    )
    body.visual(
        Box((0.084, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y + 0.004, (HOUSING_Z / 2.0) + 0.005)),
        material="housing_gray",
        name="top_cover",
    )
    body.visual(
        Box((0.072, 0.024, 0.088)),
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y - (BODY_Y / 2.0) - 0.012, -0.004)),
        material="housing_gray",
        name="rear_pad",
    )
    body.visual(
        Box((0.090, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material="housing_gray",
        name="front_bridge",
    )

    for side in (-1.0, 1.0):
        x_center = side * rail_center_x
        body.visual(
            Box((rail_length, 0.026, 0.034)),
            origin=Origin(xyz=(x_center, rail_y, 0.0)),
            material="housing_gray",
            name=f"rail_{'left' if side < 0 else 'right'}",
        )
        body.visual(
            Box((rail_length, 0.020, 0.004)),
            origin=Origin(xyz=(x_center, rail_y, 0.019)),
            material="wear_steel",
            name=f"wear_strip_{'left' if side < 0 else 'right'}",
        )
        body.visual(
            Box((0.014, 0.032, 0.018)),
            origin=Origin(xyz=(side * 0.028, rail_y, 0.0)),
            material="housing_gray",
            name=f"inner_stop_{'left' if side < 0 else 'right'}",
        )
        body.visual(
            Box((0.010, 0.032, 0.018)),
            origin=Origin(xyz=(side * 0.173, rail_y, 0.0)),
            material="housing_gray",
            name=f"outer_stop_{'left' if side < 0 else 'right'}",
        )

    for x_pt, y_pt in (
        (-0.036, HOUSING_CENTER_Y - 0.012),
        (0.036, HOUSING_CENTER_Y - 0.012),
        (-0.036, HOUSING_CENTER_Y + 0.018),
        (0.036, HOUSING_CENTER_Y + 0.018),
        (-0.012, HOUSING_CENTER_Y + 0.004),
        (0.012, HOUSING_CENTER_Y + 0.004),
    ):
        body.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(x_pt, y_pt, (HOUSING_Z / 2.0) + 0.0095)),
            material="fastener_dark",
            name=f"cover_fastener_{x_pt:.3f}_{y_pt:.3f}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.360, BODY_Y, BODY_Z)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_jaw = model.part("left_jaw")
    for jaw, side, label in (
        (left_jaw, -1.0, "left"),
        (model.part("right_jaw"), 1.0, "right"),
    ):
        inner_dir = -side
        jaw.visual(
            Box((RUNNER_X, 0.014, 0.028)),
            origin=Origin(xyz=(0.0, rail_front_y + 0.007, 0.0)),
            material="wear_steel",
            name="runner",
        )
        jaw.visual(
            Box((CARRIAGE_X, CARRIAGE_Y, 0.052)),
            origin=Origin(xyz=(0.0, 0.101, 0.0)),
            material="jaw_black",
            name="carriage_block",
        )
        jaw.visual(
            Box((CAP_X, CAP_Y, CAP_Z)),
            origin=Origin(xyz=(side * 0.020, 0.097, 0.030)),
            material="wear_steel",
            name="carriage_cap",
        )
        jaw.visual(
            Box((FINGER_ROOT_X, FINGER_ROOT_Y, FINGER_ROOT_Z)),
            origin=Origin(
                xyz=(inner_dir * ((CARRIAGE_X / 2.0) + (FINGER_ROOT_X / 2.0) - 0.001), 0.150, 0.0)
            ),
            material="jaw_black",
            name="finger_root",
        )
        jaw.visual(
            Box((FINGER_TIP_X, FINGER_TIP_Y, FINGER_TIP_Z)),
            origin=Origin(
                xyz=(inner_dir * ((CARRIAGE_X / 2.0) + FINGER_ROOT_X + (FINGER_TIP_X / 2.0) - 0.002), 0.189, -0.010)
            ),
            material="jaw_black",
            name="finger_tip",
        )
        for x_offset, y_pt in ((0.011, 0.089), (0.028, 0.089), (0.011, 0.105), (0.028, 0.105)):
            jaw.visual(
                Cylinder(radius=0.003, length=0.003),
                origin=Origin(xyz=(side * x_offset, y_pt, 0.0345)),
                material="fastener_dark",
                name=f"{label}_cap_fastener_{x_offset:.3f}_{y_pt:.3f}",
            )
        jaw.inertial = Inertial.from_geometry(
            Box((0.145, 0.135, 0.070)),
            mass=2.0,
            origin=Origin(xyz=(0.0, 0.110, 0.0)),
        )
    right_jaw = model.get_part("right_jaw")

    model.articulation(
        "body_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-JAW_OPEN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.18, lower=0.0, upper=JAW_TRAVEL),
    )
    model.articulation(
        "body_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(JAW_OPEN_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.18, lower=0.0, upper=JAW_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("body_to_left_jaw")
    right_slide = object_model.get_articulation("body_to_right_jaw")

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
        "left jaw closes along +x",
        tuple(left_slide.axis) == (1.0, 0.0, 0.0),
        details=f"left axis was {left_slide.axis}",
    )
    ctx.check(
        "right jaw closes along -x",
        tuple(right_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"right axis was {right_slide.axis}",
    )
    ctx.expect_contact(left_jaw, body, name="left carriage bears on housing rail")
    ctx.expect_contact(right_jaw, body, name="right carriage bears on housing rail")
    ctx.expect_overlap(
        left_jaw,
        body,
        axes="xz",
        min_overlap=0.020,
        name="left carriage visibly overlaps guide body",
    )
    ctx.expect_overlap(
        right_jaw,
        body,
        axes="xz",
        min_overlap=0.020,
        name="right carriage visibly overlaps guide body",
    )
    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        min_gap=0.055,
        max_gap=0.075,
        name="open jaw gap",
    )

    with ctx.pose({left_slide: JAW_TRAVEL, right_slide: JAW_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at fully closed pose")
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.010,
            max_gap=0.020,
            name="closed jaw clearance",
        )
        ctx.expect_overlap(
            left_jaw,
            right_jaw,
            axes="yz",
            min_overlap=0.030,
            name="closed jaws share a common gripping window",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=9,
        ignore_adjacent=False,
        name="jaw travel sweep stays clear",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
