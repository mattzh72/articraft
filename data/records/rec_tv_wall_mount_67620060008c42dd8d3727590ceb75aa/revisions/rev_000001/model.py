from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="empty_tv_wall_mount")

    satin_black = model.material("satin_black_powder_coat", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_pin = model.material("dark_pin_caps", rgba=(0.005, 0.005, 0.006, 1.0))
    screw_dark = model.material("black_screw_heads", rgba=(0.01, 0.01, 0.012, 1.0))
    edge_gray = model.material("worn_edge_highlights", rgba=(0.18, 0.18, 0.17, 1.0))

    # Root frame is the shoulder hinge axis on the front of the wall plate.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.018, 0.260, 0.420)),
        origin=Origin(xyz=(-0.060, 0.0, 0.0)),
        material=satin_black,
        name="plate_slab",
    )
    # Raised folded edges and ribs make the plate read as stamped steel rather than
    # as a plain block.
    wall_plate.visual(
        Box((0.012, 0.020, 0.420)),
        origin=Origin(xyz=(-0.047, 0.130, 0.0)),
        material=edge_gray,
        name="side_lip_0",
    )
    wall_plate.visual(
        Box((0.012, 0.020, 0.420)),
        origin=Origin(xyz=(-0.047, -0.130, 0.0)),
        material=edge_gray,
        name="side_lip_1",
    )
    wall_plate.visual(
        Box((0.012, 0.220, 0.018)),
        origin=Origin(xyz=(-0.047, 0.0, 0.195)),
        material=edge_gray,
        name="top_lip",
    )
    wall_plate.visual(
        Box((0.012, 0.220, 0.018)),
        origin=Origin(xyz=(-0.047, 0.0, -0.195)),
        material=edge_gray,
        name="bottom_lip",
    )
    for i, (y, z) in enumerate(((-0.085, 0.140), (0.085, 0.140), (-0.085, -0.140), (0.085, -0.140))):
        wall_plate.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(-0.049, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_dark,
            name=f"lag_screw_{i}",
        )

    # The shoulder bracket is a two-knuckle clevis with a central gap for the arm
    # barrel.  The gaps are deliberate physical clearance, not missing support.
    for suffix, z in (("upper", 0.0525), ("lower", -0.0525)):
        wall_plate.visual(
            Box((0.052, 0.094, 0.035)),
            origin=Origin(xyz=(-0.025, 0.0, z)),
            material=satin_black,
            name=f"shoulder_clevis_{suffix}",
        )
        wall_plate.visual(
            Cylinder(radius=0.034, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=satin_black,
            name="shoulder_knuckle_upper" if suffix == "upper" else "shoulder_knuckle_lower",
        )
    wall_plate.visual(
        Cylinder(radius=0.018, length=0.156),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_pin,
        name="shoulder_pin",
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="shoulder_barrel",
    )
    outer_arm.visual(
        Box((0.320, 0.054, 0.070)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=satin_black,
        name="outer_tube",
    )
    outer_arm.visual(
        Box((0.300, 0.018, 0.084)),
        origin=Origin(xyz=(0.190, 0.034, 0.0)),
        material=edge_gray,
        name="outer_side_flange_0",
    )
    outer_arm.visual(
        Box((0.300, 0.018, 0.084)),
        origin=Origin(xyz=(0.190, -0.034, 0.0)),
        material=edge_gray,
        name="outer_side_flange_1",
    )
    for suffix, z in (("upper", 0.0525), ("lower", -0.0525)):
        outer_arm.visual(
            Cylinder(radius=0.034, length=0.035),
            origin=Origin(xyz=(0.380, 0.0, z)),
            material=satin_black,
            name="elbow_knuckle_upper" if suffix == "upper" else "elbow_knuckle_lower",
        )
        outer_arm.visual(
            Box((0.044, 0.078, 0.035)),
            origin=Origin(xyz=(0.360, 0.0, z)),
            material=satin_black,
            name=f"elbow_clevis_{suffix}",
        )
    outer_arm.visual(
        Cylinder(radius=0.017, length=0.156),
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        material=dark_pin,
        name="elbow_pin",
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="elbow_barrel",
    )
    inner_arm.visual(
        Box((0.035, 0.048, 0.052)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=satin_black,
        name="elbow_web",
    )
    inner_arm.visual(
        Box((0.280, 0.052, 0.066)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=satin_black,
        name="inner_tube",
    )
    inner_arm.visual(
        Box((0.260, 0.016, 0.080)),
        origin=Origin(xyz=(0.170, 0.033, 0.0)),
        material=edge_gray,
        name="inner_side_flange_0",
    )
    inner_arm.visual(
        Box((0.260, 0.016, 0.080)),
        origin=Origin(xyz=(0.170, -0.033, 0.0)),
        material=edge_gray,
        name="inner_side_flange_1",
    )
    for suffix, z in (("upper", 0.0525), ("lower", -0.0525)):
        inner_arm.visual(
            Cylinder(radius=0.032, length=0.035),
            origin=Origin(xyz=(0.340, 0.0, z)),
            material=satin_black,
            name="head_knuckle_upper" if suffix == "upper" else "head_knuckle_lower",
        )
        inner_arm.visual(
            Box((0.040, 0.072, 0.035)),
            origin=Origin(xyz=(0.322, 0.0, z)),
            material=satin_black,
            name=f"head_clevis_{suffix}",
        )
    for suffix, y in (("side_0", 0.042), ("side_1", -0.042)):
        inner_arm.visual(
            Box((0.070, 0.018, 0.150)),
            origin=Origin(xyz=(0.305, y, 0.0)),
            material=satin_black,
            name=f"head_side_web_{suffix}",
        )
    inner_arm.visual(
        Cylinder(radius=0.016, length=0.150),
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        material=dark_pin,
        name="head_pin",
    )

    swivel_head = model.part("swivel_head")
    swivel_head.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="swivel_barrel",
    )
    swivel_head.visual(
        Box((0.042, 0.110, 0.050)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=satin_black,
        name="wrist_block",
    )
    for suffix, y in (("side_0", 0.055), ("side_1", -0.055)):
        swivel_head.visual(
            Box((0.050, 0.014, 0.026)),
            origin=Origin(xyz=(0.070, y, 0.0)),
            material=satin_black,
            name=f"wrist_side_bridge_{suffix}",
        )
    # Horizontal tilt yoke around the tilt axis at local x=0.09.
    for suffix, y in (("side_0", 0.0575), ("side_1", -0.0575)):
        swivel_head.visual(
            Box((0.044, 0.035, 0.082)),
            origin=Origin(xyz=(0.090, y, 0.0)),
            material=satin_black,
            name=f"tilt_yoke_{suffix}",
        )
        swivel_head.visual(
            Cylinder(radius=0.028, length=0.035),
            origin=Origin(xyz=(0.090, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="tilt_knuckle_side_0" if suffix == "side_0" else "tilt_knuckle_side_1",
        )
    swivel_head.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_pin,
        name="tilt_pin",
    )

    mounting_frame = model.part("mounting_frame")
    mounting_frame.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="tilt_barrel",
    )
    mounting_frame.visual(
        Box((0.014, 0.050, 0.040)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=satin_black,
        name="tilt_web",
    )
    mounting_frame.visual(
        Box((0.065, 0.110, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=satin_black,
        name="center_plate",
    )
    mounting_frame.visual(
        Box((0.032, 0.066, 0.264)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=satin_black,
        name="center_spine",
    )
    mounting_frame.visual(
        Box((0.032, 0.230, 0.030)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=satin_black,
        name="middle_crossbar",
    )
    # Empty rectangular VESA-style frame: four perimeter rails plus two inner
    # slotted rails.  The open center makes it clear no TV display is attached.
    mounting_frame.visual(
        Box((0.026, 0.420, 0.032)),
        origin=Origin(xyz=(0.100, 0.0, 0.145)),
        material=satin_black,
        name="top_bar",
    )
    mounting_frame.visual(
        Box((0.026, 0.420, 0.032)),
        origin=Origin(xyz=(0.100, 0.0, -0.145)),
        material=satin_black,
        name="bottom_bar",
    )
    for suffix, y in (("side_0", 0.195), ("side_1", -0.195)):
        mounting_frame.visual(
            Box((0.026, 0.032, 0.290)),
            origin=Origin(xyz=(0.100, y, 0.0)),
            material=satin_black,
            name=f"side_rail_{suffix}",
        )
    for suffix, y in (("inner_0", 0.095), ("inner_1", -0.095)):
        mounting_frame.visual(
            Box((0.024, 0.028, 0.290)),
            origin=Origin(xyz=(0.095, y, 0.0)),
            material=edge_gray,
            name=f"vesa_rail_{suffix}",
        )
        for idx, z in enumerate((-0.085, 0.085)):
            mounting_frame.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(0.108, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_dark,
                name=f"vesa_slot_{suffix}_{idx}",
            )

    ninety = math.pi / 2.0
    forty_five = math.pi / 4.0
    fifteen = math.radians(15.0)

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=outer_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=-ninety, upper=ninety),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=inner_arm,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=-ninety, upper=ninety),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=swivel_head,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=-forty_five, upper=forty_five),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel_head,
        child=mounting_frame,
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=-fifteen, upper=fifteen),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    outer_arm = object_model.get_part("outer_arm")
    inner_arm = object_model.get_part("inner_arm")
    swivel_head = object_model.get_part("swivel_head")
    mounting_frame = object_model.get_part("mounting_frame")

    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow_yaw")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    def _limits_close(joint, lower: float, upper: float, tol: float = 1e-6) -> bool:
        limits = joint.motion_limits
        return (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - lower) <= tol
            and abs(limits.upper - upper) <= tol
        )

    ctx.check("shoulder yaws 90 degrees each way", _limits_close(shoulder, -math.pi / 2.0, math.pi / 2.0))
    ctx.check("elbow yaws 90 degrees each way", _limits_close(elbow, -math.pi / 2.0, math.pi / 2.0))
    ctx.check("head swivels 45 degrees each way", _limits_close(head_swivel, -math.pi / 4.0, math.pi / 4.0))
    ctx.check(
        "head tilts 15 degrees each way",
        _limits_close(head_tilt, -math.radians(15.0), math.radians(15.0)),
    )

    ctx.allow_overlap(
        wall_plate,
        outer_arm,
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        reason="The shoulder pin is intentionally modeled as a captured shaft through the arm barrel.",
    )
    ctx.allow_overlap(
        outer_arm,
        inner_arm,
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        reason="The elbow pin is intentionally modeled as a captured shaft through the inner arm barrel.",
    )
    ctx.allow_overlap(
        inner_arm,
        swivel_head,
        elem_a="head_pin",
        elem_b="swivel_barrel",
        reason="The head swivel pin is intentionally modeled as a captured shaft through the swivel barrel.",
    )
    ctx.allow_overlap(
        swivel_head,
        mounting_frame,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tilt pin is intentionally modeled as a captured shaft through the horizontal frame barrel.",
    )

    ctx.expect_within(
        wall_plate,
        outer_arm,
        axes="xy",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_barrel",
        name="shoulder pin is centered in the barrel",
    )
    ctx.expect_overlap(
        wall_plate,
        outer_arm,
        axes="z",
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        min_overlap=0.060,
        name="shoulder pin spans the barrel",
    )
    ctx.expect_within(
        outer_arm,
        inner_arm,
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="elbow_barrel",
        name="elbow pin is centered in the barrel",
    )
    ctx.expect_overlap(
        outer_arm,
        inner_arm,
        axes="z",
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        min_overlap=0.060,
        name="elbow pin spans the barrel",
    )
    ctx.expect_within(
        inner_arm,
        swivel_head,
        axes="xy",
        inner_elem="head_pin",
        outer_elem="swivel_barrel",
        name="head pin is centered in the swivel barrel",
    )
    ctx.expect_overlap(
        inner_arm,
        swivel_head,
        axes="z",
        elem_a="head_pin",
        elem_b="swivel_barrel",
        min_overlap=0.060,
        name="head pin spans the swivel barrel",
    )
    ctx.expect_within(
        swivel_head,
        mounting_frame,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="tilt_barrel",
        name="tilt pin is centered in the frame barrel",
    )
    ctx.expect_overlap(
        swivel_head,
        mounting_frame,
        axes="y",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.075,
        name="tilt pin spans the frame barrel",
    )

    ctx.expect_contact(
        wall_plate,
        outer_arm,
        elem_a="shoulder_knuckle_upper",
        elem_b="shoulder_barrel",
        contact_tol=0.002,
        name="shoulder clevis captures arm barrel",
    )
    ctx.expect_contact(
        outer_arm,
        inner_arm,
        elem_a="elbow_knuckle_upper",
        elem_b="elbow_barrel",
        contact_tol=0.002,
        name="elbow clevis captures inner barrel",
    )
    ctx.expect_contact(
        inner_arm,
        swivel_head,
        elem_a="head_knuckle_upper",
        elem_b="swivel_barrel",
        contact_tol=0.002,
        name="head swivel barrel is captured",
    )
    ctx.expect_contact(
        swivel_head,
        mounting_frame,
        elem_a="tilt_knuckle_side_0",
        elem_b="tilt_barrel",
        contact_tol=0.002,
        name="tilt yoke captures frame barrel",
    )

    ctx.expect_origin_gap(
        mounting_frame,
        wall_plate,
        axis="x",
        min_gap=0.70,
        max_gap=0.90,
        name="folding arm holds bare frame out from wall",
    )

    frame_rest = ctx.part_world_position(mounting_frame)
    with ctx.pose({shoulder: math.pi / 2.0}):
        frame_swung = ctx.part_world_position(mounting_frame)
    ctx.check(
        "shoulder swing carries the arm sideways",
        frame_rest is not None and frame_swung is not None and frame_swung[1] > frame_rest[1] + 0.55,
        details=f"rest={frame_rest}, swung={frame_swung}",
    )

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    top_rest = _aabb_center_x(ctx.part_element_world_aabb(mounting_frame, elem="top_bar"))
    with ctx.pose({head_tilt: math.radians(15.0)}):
        top_tilted = _aabb_center_x(ctx.part_element_world_aabb(mounting_frame, elem="top_bar"))
    ctx.check(
        "tilt joint tips the rectangular frame",
        top_rest is not None and top_tilted is not None and top_tilted > top_rest + 0.02,
        details=f"top_rest_x={top_rest}, top_tilted_x={top_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
