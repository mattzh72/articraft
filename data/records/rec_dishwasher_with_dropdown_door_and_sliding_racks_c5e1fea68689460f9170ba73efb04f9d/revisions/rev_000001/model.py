from __future__ import annotations

from math import pi

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


BODY_WIDTH = 0.598
BODY_DEPTH = 0.595
BODY_HEIGHT = 0.820
PLINTH_HEIGHT = 0.095
TOP_MARGIN = 0.012
SHELL_THICKNESS = 0.010
TUB_WIDTH = 0.556
TUB_HEIGHT = 0.700
DOOR_WIDTH = BODY_WIDTH - 0.006
DOOR_HEIGHT = BODY_HEIGHT - PLINTH_HEIGHT - TOP_MARGIN
DOOR_THICKNESS = 0.046
LOWER_RACK_LENGTH = 0.500
LOWER_RACK_WIDTH = 0.486
LOWER_RACK_HEIGHT = 0.148
UPPER_RACK_LENGTH = 0.492
UPPER_RACK_WIDTH = 0.492
UPPER_RACK_HEIGHT = 0.118


def _body_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        BODY_DEPTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        centered=(False, True, False),
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - SHELL_THICKNESS + 0.040,
            TUB_WIDTH,
            TUB_HEIGHT,
            centered=(False, True, False),
        )
        .translate((-0.020, 0.0, PLINTH_HEIGHT))
    )

    body = outer.cut(cavity)

    rail_length = BODY_DEPTH - 0.110
    rail_depth_start = 0.050
    upper_rail_z = 0.540
    lower_rail_z = 0.305
    for rail_z, rail_h in ((upper_rail_z, 0.015), (lower_rail_z, 0.014)):
        for rail_sign in (-1.0, 1.0):
            rail = (
                cq.Workplane("XY")
                .box(
                    rail_length,
                    0.020,
                    rail_h,
                    centered=(False, True, False),
                )
                .translate(
                    (
                        rail_depth_start,
                        rail_sign * (TUB_WIDTH * 0.5 - 0.006),
                        rail_z,
                    )
                )
            )
            body = body.union(rail)

    return body


def _door_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(
            DOOR_THICKNESS,
            DOOR_WIDTH,
            DOOR_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-0.007, 0.0, 0.0))
    )
    shell_cavity = (
        cq.Workplane("XY")
        .box(
            DOOR_THICKNESS - 0.016,
            DOOR_WIDTH - 0.060,
            DOOR_HEIGHT - 0.090,
            centered=(False, True, False),
        )
        .translate((0.002, 0.0, 0.050))
    )
    shell = outer.cut(shell_cavity)

    liner_outer = (
        cq.Workplane("XY")
        .box(
            0.017,
            DOOR_WIDTH - 0.028,
            DOOR_HEIGHT - 0.070,
            centered=(False, True, False),
        )
        .translate((0.001, 0.0, 0.036))
    )
    liner_inner = (
        cq.Workplane("XY")
        .box(
            0.013,
            DOOR_WIDTH - 0.094,
            DOOR_HEIGHT - 0.156,
            centered=(False, True, False),
        )
        .translate((0.005, 0.0, 0.075))
    )
    liner = liner_outer.cut(liner_inner)

    return shell.union(liner)


def _rack_shape(length: float, width: float, height: float, *, upper: bool) -> cq.Workplane:
    wall = 0.010

    def rack_box(
        sx: float,
        sy: float,
        sz: float,
        xyz: tuple[float, float, float],
        centered: tuple[bool, bool, bool] = (False, True, False),
    ) -> cq.Workplane:
        return cq.Workplane("XY").box(sx, sy, sz, centered=centered).translate(xyz)

    outer = rack_box(length, width, height, (0.0, 0.0, 0.0))
    inner = rack_box(
        length - 2.0 * wall,
        width - 2.0 * wall,
        height,
        (wall, 0.0, wall),
    )
    rack = outer.cut(inner)

    front_window = rack_box(0.018, width - 0.140, height - 0.046, (0.004, 0.0, 0.024))
    rear_window = rack_box(0.018, width - 0.160, height - 0.046, (length - 0.022, 0.0, 0.024))
    rack = rack.cut(front_window).cut(rear_window)

    for x_pos in (0.060, 0.175, 0.290, 0.405):
        for y_sign in (-1.0, 1.0):
            side_window = rack_box(
                0.070,
                0.018,
                height - 0.054,
                (x_pos, y_sign * (width * 0.5 - 0.009), 0.026),
            )
            rack = rack.cut(side_window)

    for x_pos in (0.090, 0.200, 0.310, 0.420):
        rack = rack.union(
            rack_box(0.008, width - 0.026, 0.012, (x_pos, 0.0, 0.010))
        )

    runner_z = 0.130 if upper else 0.134
    runner_y = 0.257
    runner_length = length - 0.040
    for y_sign in (-1.0, 1.0):
        rack = rack.union(
            rack_box(
                runner_length,
                0.010,
                0.014,
                (0.020, y_sign * runner_y, runner_z),
            )
        )
        for bracket_x in (0.070, 0.220, 0.370):
            rack = rack.union(
                rack_box(
                    0.012,
                    0.028,
                    0.012,
                    (bracket_x, y_sign * (width * 0.5 + 0.003), runner_z - 0.004),
                )
            )

    if upper:
        pivot_strip_z = 0.098
        pivot_x = 0.252
        for y_sign in (-1.0, 1.0):
            rack = rack.union(
                rack_box(
                    0.176,
                    0.010,
                    0.012,
                    (pivot_x - 0.088, y_sign * (width * 0.5 - 0.017), pivot_strip_z),
                )
            )

    return rack


def _mug_shelf_shape(sign: float) -> cq.Workplane:
    shelf_length = 0.170
    shelf_drop = 0.078
    thickness = 0.006
    rib = (
        cq.Workplane("XY")
        .box(shelf_length, 0.010, 0.012, centered=(True, True, False))
        .translate((0.0, sign * 0.005, -0.012))
    )
    panel = (
        cq.Workplane("XY")
        .box(shelf_length, thickness, shelf_drop, centered=(True, True, False))
        .translate((0.0, sign * thickness * 0.5, -shelf_drop))
    )
    return rib.union(panel)


def _spray_arm_shape(span: float) -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.018).extrude(0.012).translate((0.0, 0.0, -0.006))
    arm = (
        cq.Workplane("XY")
        .box(0.030, span, 0.008, centered=(True, True, True))
        .translate((0.0, 0.0, 0.0))
    )
    nozzle_0 = (
        cq.Workplane("XY")
        .box(0.020, 0.090, 0.008, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 24.0)
        .translate((0.0, span * 0.22, 0.0))
    )
    nozzle_1 = (
        cq.Workplane("XY")
        .box(0.020, 0.090, 0.008, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -24.0)
        .translate((0.0, -span * 0.22, 0.0))
    )
    return hub.union(arm).union(nozzle_0).union(nozzle_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_dishwasher")

    model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("rack_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("control_black", rgba=(0.09, 0.10, 0.11, 1.0))
    model.material("cap_blue", rgba=(0.22, 0.44, 0.78, 1.0))

    body = model.part("body")
    wall_height = BODY_HEIGHT - PLINTH_HEIGHT + 0.004
    wall_center_z = (PLINTH_HEIGHT - 0.002) + wall_height * 0.5
    body.visual(
        Box((BODY_DEPTH, SHELL_THICKNESS, wall_height)),
        origin=Origin(xyz=(BODY_DEPTH * 0.5, BODY_WIDTH * 0.5 - SHELL_THICKNESS * 0.5, wall_center_z)),
        material="stainless",
        name="left_wall",
    )
    body.visual(
        Box((BODY_DEPTH, SHELL_THICKNESS, wall_height)),
        origin=Origin(xyz=(BODY_DEPTH * 0.5, -BODY_WIDTH * 0.5 + SHELL_THICKNESS * 0.5, wall_center_z)),
        material="stainless",
        name="right_wall",
    )
    body.visual(
        Box((BODY_DEPTH, TUB_WIDTH, 0.010)),
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, PLINTH_HEIGHT)),
        material="stainless",
        name="tub_floor",
    )
    body.visual(
        Box((0.010, TUB_WIDTH, wall_height)),
        origin=Origin(xyz=(BODY_DEPTH - 0.005, 0.0, wall_center_z)),
        material="stainless",
        name="rear_wall",
    )
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, 0.010)),
        origin=Origin(xyz=(BODY_DEPTH * 0.5, 0.0, BODY_HEIGHT - 0.005)),
        material="stainless",
        name="roof",
    )
    body.visual(
        Box((0.014, BODY_WIDTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.007, 0.0, PLINTH_HEIGHT * 0.5)),
        material="graphite",
        name="kick_panel",
    )
    for prefix, rail_z, rail_h in (("upper", 0.540, 0.015), ("lower", 0.305, 0.014)):
        for rail_sign, suffix in ((1.0, "0"), (-1.0, "1")):
            body.visual(
                Box((BODY_DEPTH - 0.110, 0.020, rail_h)),
                origin=Origin(
                    xyz=(0.2925, rail_sign * (TUB_WIDTH * 0.5 - 0.006), rail_z),
                ),
                material="graphite",
                name=f"{prefix}_rail_{suffix}",
            )
            body.visual(
                Box((BODY_DEPTH - 0.140, 0.030, rail_h + 0.002)),
                origin=Origin(
                    xyz=(0.285, rail_sign * (BODY_WIDTH * 0.5 - 0.017), rail_z),
                ),
                material="graphite",
                name=f"{prefix}_mount_{suffix}",
            )
    body.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.310, 0.0, 0.105)),
        material="graphite",
        name="lower_hub_post",
    )
    body.visual(
        Box((0.300, 0.016, 0.014)),
        origin=Origin(xyz=(0.445, 0.0, 0.378)),
        material="graphite",
        name="upper_feed_tube",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.300, 0.0, 0.377)),
        material="graphite",
        name="upper_hub_post",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "dishwasher_door"),
        material="stainless",
        name="door_shell",
    )

    top_strip_depth = 0.028
    door.visual(
        Box((top_strip_depth, DOOR_WIDTH - 0.070, 0.016)),
        origin=Origin(
            xyz=(-0.004, 0.0, DOOR_HEIGHT - 0.008),
        ),
        material="graphite",
        name="top_strip",
    )
    door.visual(
        Box((0.010, DOOR_WIDTH - 0.120, 0.020)),
        origin=Origin(
            xyz=(0.018, 0.0, 0.010),
        ),
        material="graphite",
        name="hinge_leaf",
    )
    door.visual(
        Box((0.020, 0.082, 0.082)),
        origin=Origin(xyz=(0.014, 0.118, 0.276)),
        material="stainless",
        name="rinse_mount",
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        Box((LOWER_RACK_LENGTH, LOWER_RACK_WIDTH - 0.020, 0.010)),
        origin=Origin(xyz=(LOWER_RACK_LENGTH * 0.5, 0.0, 0.005)),
        material="rack_gray",
        name="floor",
    )
    lower_rack.visual(
        Box((LOWER_RACK_LENGTH, 0.010, LOWER_RACK_HEIGHT - 0.010)),
        origin=Origin(
            xyz=(
                LOWER_RACK_LENGTH * 0.5,
                LOWER_RACK_WIDTH * 0.5 - 0.005,
                (LOWER_RACK_HEIGHT - 0.010) * 0.5 + 0.005,
            )
        ),
        material="rack_gray",
        name="left_side",
    )
    lower_rack.visual(
        Box((LOWER_RACK_LENGTH, 0.010, LOWER_RACK_HEIGHT - 0.010)),
        origin=Origin(
            xyz=(
                LOWER_RACK_LENGTH * 0.5,
                -LOWER_RACK_WIDTH * 0.5 + 0.005,
                (LOWER_RACK_HEIGHT - 0.010) * 0.5 + 0.005,
            )
        ),
        material="rack_gray",
        name="right_side",
    )
    lower_rack.visual(
        Box((0.010, LOWER_RACK_WIDTH, LOWER_RACK_HEIGHT - 0.010)),
        origin=Origin(
            xyz=(
                LOWER_RACK_LENGTH - 0.005,
                0.0,
                (LOWER_RACK_HEIGHT - 0.010) * 0.5 + 0.005,
            )
        ),
        material="rack_gray",
        name="rear_frame",
    )
    lower_rack.visual(
        Box((0.010, LOWER_RACK_WIDTH - 0.090, 0.078)),
        origin=Origin(xyz=(0.005, 0.0, 0.039)),
        material="rack_gray",
        name="front_frame",
    )
    for y_sign, suffix in ((1.0, "0"), (-1.0, "1")):
        lower_rack.visual(
            Box((LOWER_RACK_LENGTH - 0.040, 0.010, 0.014)),
            origin=Origin(xyz=(LOWER_RACK_LENGTH * 0.5, y_sign * 0.257, 0.134)),
            material="graphite",
            name=f"runner_{suffix}",
        )
        lower_rack.visual(
            Box((LOWER_RACK_LENGTH - 0.130, 0.024, 0.020)),
            origin=Origin(xyz=(LOWER_RACK_LENGTH * 0.5, y_sign * 0.249, 0.124)),
            material="rack_gray",
            name=f"runner_mount_{suffix}",
        )
    lower_rack.visual(
        Box((LOWER_RACK_LENGTH - 0.120, 0.008, 0.010)),
        origin=Origin(xyz=(LOWER_RACK_LENGTH * 0.5, 0.0, 0.010)),
        material="rack_gray",
        name="crossbar",
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        Box((UPPER_RACK_LENGTH, UPPER_RACK_WIDTH - 0.022, 0.008)),
        origin=Origin(xyz=(UPPER_RACK_LENGTH * 0.5, 0.0, 0.004)),
        material="rack_gray",
        name="floor",
    )
    upper_rack.visual(
        Box((UPPER_RACK_LENGTH, 0.010, UPPER_RACK_HEIGHT - 0.008)),
        origin=Origin(
            xyz=(
                UPPER_RACK_LENGTH * 0.5,
                UPPER_RACK_WIDTH * 0.5 - 0.005,
                (UPPER_RACK_HEIGHT - 0.008) * 0.5 + 0.004,
            )
        ),
        material="rack_gray",
        name="left_side",
    )
    upper_rack.visual(
        Box((UPPER_RACK_LENGTH, 0.010, UPPER_RACK_HEIGHT - 0.008)),
        origin=Origin(
            xyz=(
                UPPER_RACK_LENGTH * 0.5,
                -UPPER_RACK_WIDTH * 0.5 + 0.005,
                (UPPER_RACK_HEIGHT - 0.008) * 0.5 + 0.004,
            )
        ),
        material="rack_gray",
        name="right_side",
    )
    upper_rack.visual(
        Box((0.010, UPPER_RACK_WIDTH, UPPER_RACK_HEIGHT - 0.008)),
        origin=Origin(
            xyz=(
                UPPER_RACK_LENGTH - 0.005,
                0.0,
                (UPPER_RACK_HEIGHT - 0.008) * 0.5 + 0.004,
            )
        ),
        material="rack_gray",
        name="rear_frame",
    )
    upper_rack.visual(
        Box((0.010, UPPER_RACK_WIDTH - 0.110, 0.060)),
        origin=Origin(xyz=(0.005, 0.0, 0.030)),
        material="rack_gray",
        name="front_frame",
    )
    for y_sign, suffix in ((1.0, "0"), (-1.0, "1")):
        upper_rack.visual(
            Box((UPPER_RACK_LENGTH - 0.040, 0.010, 0.014)),
            origin=Origin(xyz=(UPPER_RACK_LENGTH * 0.5, y_sign * 0.257, 0.1295)),
            material="graphite",
            name=f"runner_{suffix}",
        )
        upper_rack.visual(
            Box((UPPER_RACK_LENGTH - 0.130, 0.024, 0.020)),
            origin=Origin(xyz=(UPPER_RACK_LENGTH * 0.5, y_sign * 0.250, 0.116)),
            material="rack_gray",
            name=f"runner_mount_{suffix}",
        )
        upper_rack.visual(
            Box((0.176, 0.020, 0.010)),
            origin=Origin(
                xyz=(0.252, y_sign * (UPPER_RACK_WIDTH * 0.5 - 0.015), 0.108)
            ),
            material="rack_gray",
            name=f"shelf_mount_{suffix}",
        )
    upper_rack.visual(
        Box((UPPER_RACK_LENGTH - 0.120, 0.008, 0.010)),
        origin=Origin(xyz=(UPPER_RACK_LENGTH * 0.5, 0.0, 0.008)),
        material="rack_gray",
        name="crossbar",
    )

    lower_arm = model.part("lower_spray_arm")
    lower_arm.visual(
        mesh_from_cadquery(_spray_arm_shape(0.380), "lower_spray_arm"),
        material="graphite",
        name="arm",
    )
    lower_arm.visual(
        Box((0.020, 0.020, 0.012)),
        material="graphite",
        name="hub",
    )

    upper_arm = model.part("upper_spray_arm")
    upper_arm.visual(
        mesh_from_cadquery(_spray_arm_shape(0.320), "upper_spray_arm"),
        material="graphite",
        name="arm",
    )
    upper_arm.visual(
        Box((0.018, 0.018, 0.012)),
        material="graphite",
        name="hub",
    )

    for part_name, y_pos, width in (
        ("start_button", 0.180, 0.026),
        ("option_button_0", 0.125, 0.020),
        ("option_button_1", 0.090, 0.020),
        ("option_button_2", 0.055, 0.020),
    ):
        button = model.part(part_name)
        button.visual(
            Box((0.016, width, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material="control_black",
            name="cap",
        )
        model.articulation(
            f"{part_name}_press",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(-0.004, y_pos, DOOR_HEIGHT)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    rinse_cap = model.part("rinse_cap")
    rinse_cap.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="cap_blue",
        name="cap",
    )
    rinse_cap.visual(
        Box((0.006, 0.012, 0.012)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material="cap_blue",
        name="grip",
    )

    mug_shelf_0 = model.part("mug_shelf_0")
    mug_shelf_0.visual(
        Box((0.170, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, 0.003, -0.045)),
        material="rack_gray",
        name="panel",
    )
    mug_shelf_0.visual(
        Box((0.170, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.005, -0.005)),
        material="rack_gray",
        name="pivot",
    )
    mug_shelf_1 = model.part("mug_shelf_1")
    mug_shelf_1.visual(
        Box((0.170, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, -0.003, -0.045)),
        material="rack_gray",
        name="panel",
    )
    mug_shelf_1.visual(
        Box((0.170, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, -0.005)),
        material="rack_gray",
        name="pivot",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.023, 0.0, PLINTH_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=1.62,
        ),
    )
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.052, 0.0, 0.157)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=0.355,
        ),
    )
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.056, 0.0, 0.396)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.25,
            lower=0.0,
            upper=0.338,
        ),
    )
    model.articulation(
        "lower_spray_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lower_arm,
        origin=Origin(xyz=(0.310, 0.0, 0.121)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "upper_spray_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=upper_arm,
        origin=Origin(xyz=(0.300, 0.0, 0.390)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "rinse_cap_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=rinse_cap,
        origin=Origin(xyz=(0.026, 0.118, 0.276)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    model.articulation(
        "mug_shelf_0_fold",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=mug_shelf_0,
        origin=Origin(
            xyz=(0.252, UPPER_RACK_WIDTH * 0.5 - 0.020, 0.110),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "mug_shelf_1_fold",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=mug_shelf_1,
        origin=Origin(
            xyz=(0.252, -UPPER_RACK_WIDTH * 0.5 + 0.020, 0.110),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    lower_arm = object_model.get_part("lower_spray_arm")
    upper_arm = object_model.get_part("upper_spray_arm")
    rinse_cap = object_model.get_part("rinse_cap")
    mug_shelf_0 = object_model.get_part("mug_shelf_0")
    mug_shelf_1 = object_model.get_part("mug_shelf_1")

    door_hinge = object_model.get_articulation("door_hinge")
    lower_rack_slide = object_model.get_articulation("lower_rack_slide")
    upper_rack_slide = object_model.get_articulation("upper_rack_slide")
    mug_shelf_0_fold = object_model.get_articulation("mug_shelf_0_fold")
    mug_shelf_1_fold = object_model.get_articulation("mug_shelf_1_fold")
    start_button_press = object_model.get_articulation("start_button_press")
    option_button_0_press = object_model.get_articulation("option_button_0_press")
    option_button_1_press = object_model.get_articulation("option_button_1_press")
    option_button_2_press = object_model.get_articulation("option_button_2_press")

    ctx.allow_overlap(
        mug_shelf_0,
        upper_rack,
        elem_a="pivot",
        elem_b="shelf_mount_0",
        reason="The left mug shelf wraps its side pivot mount at the hinge line.",
    )
    ctx.allow_overlap(
        mug_shelf_1,
        upper_rack,
        elem_a="pivot",
        elem_b="shelf_mount_1",
        reason="The right mug shelf wraps its side pivot mount at the hinge line.",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(body, door, axis="x", max_gap=0.003, max_penetration=0.002, name="closed door sits flush to the tub front")
        ctx.expect_contact(rinse_cap, door, elem_a="cap", elem_b="rinse_mount", name="rinse cap seats on the door liner mount")
        ctx.expect_contact(lower_arm, body, elem_a="hub", elem_b="lower_hub_post", name="lower spray arm rides on its hub post")
        ctx.expect_contact(upper_arm, body, elem_a="hub", elem_b="upper_hub_post", name="upper spray arm rides on its hub post")
        ctx.expect_within(lower_rack, body, axes="y", margin=0.0, name="lower rack stays centered between side rails")
        ctx.expect_within(upper_rack, body, axes="y", margin=0.0, name="upper rack stays centered between side rails")

    door_closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.50}):
        door_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door drops forward when opened",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][0] < door_closed_aabb[0][0] - 0.35
        and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.20,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_rack_slide: 0.330}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(lower_rack, body, axes="x", min_overlap=0.14, name="lower rack retains insertion at extension")
        ctx.expect_within(lower_rack, body, axes="y", margin=0.0, name="lower rack remains aligned when extended")
    ctx.check(
        "lower rack extends out of the tub",
        lower_rest is not None and lower_extended is not None and lower_extended[0] < lower_rest[0] - 0.25,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({upper_rack_slide: 0.315}):
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(upper_rack, body, axes="x", min_overlap=0.12, name="upper rack retains insertion at extension")
        ctx.expect_within(upper_rack, body, axes="y", margin=0.0, name="upper rack remains aligned when extended")
    ctx.check(
        "upper rack extends out of the tub",
        upper_rest is not None and upper_extended is not None and upper_extended[0] < upper_rest[0] - 0.24,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    for joint_name, label, part_name in (
        (start_button_press, "start", "start_button"),
        (option_button_0_press, "option_0", "option_button_0"),
        (option_button_1_press, "option_1", "option_button_1"),
        (option_button_2_press, "option_2", "option_button_2"),
    ):
        button_part = object_model.get_part(part_name)
        rest_pos = ctx.part_world_position(button_part)
        with ctx.pose({joint_name: 0.004}):
            pressed_pos = ctx.part_world_position(button_part)
        ctx.check(
            f"{label} button presses downward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    shelf_0_closed = ctx.part_element_world_aabb(mug_shelf_0, elem="panel")
    with ctx.pose({mug_shelf_0_fold: 1.45}):
        shelf_0_open = ctx.part_element_world_aabb(mug_shelf_0, elem="panel")
    ctx.check(
        "left mug shelf folds inward",
        shelf_0_closed is not None
        and shelf_0_open is not None
        and ((shelf_0_closed[0][1] + shelf_0_closed[1][1]) * 0.5) > ((shelf_0_open[0][1] + shelf_0_open[1][1]) * 0.5) + 0.04,
        details=f"closed={shelf_0_closed}, open={shelf_0_open}",
    )

    shelf_1_closed = ctx.part_element_world_aabb(mug_shelf_1, elem="panel")
    with ctx.pose({mug_shelf_1_fold: 1.45}):
        shelf_1_open = ctx.part_element_world_aabb(mug_shelf_1, elem="panel")
    ctx.check(
        "right mug shelf folds inward",
        shelf_1_closed is not None
        and shelf_1_open is not None
        and ((shelf_1_closed[0][1] + shelf_1_closed[1][1]) * 0.5) < ((shelf_1_open[0][1] + shelf_1_open[1][1]) * 0.5) - 0.04,
        details=f"closed={shelf_1_closed}, open={shelf_1_open}",
    )

    return ctx.report()


object_model = build_object_model()
