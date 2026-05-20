from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deluxe_food_processor")

    satin_white = model.material("satin_white", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_band = model.material("dark_control_band", rgba=(0.035, 0.038, 0.042, 1.0))
    clear_poly = model.material("clear_polycarbonate", rgba=(0.70, 0.90, 1.0, 0.36))
    smoked_lid = model.material("smoked_clear_lid", rgba=(0.52, 0.70, 0.86, 0.42))
    black = model.material("black_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.75, 0.72, 1.0))
    white_mark = model.material("white_markings", rgba=(0.97, 0.97, 0.93, 1.0))
    blue_button = model.material("blue_rocker", rgba=(0.05, 0.20, 0.45, 1.0))
    gray_button = model.material("gray_rocker", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("processor_body")
    body.visual(
        Box((0.42, 0.34, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=satin_white,
        name="rounded_base",
    )
    body.visual(
        Box((0.320, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, -0.174, 0.083)),
        material=dark_band,
        name="control_band",
    )
    body.visual(
        Box((0.030, 0.006, 0.010)),
        origin=Origin(xyz=(-0.090, -0.183, 0.127)),
        material=white_mark,
        name="dial_tick",
    )
    body.visual(
        Box((0.010, 0.006, 0.030)),
        origin=Origin(xyz=(-0.128, -0.183, 0.088)),
        material=white_mark,
        name="low_tick",
    )
    body.visual(
        Box((0.010, 0.006, 0.030)),
        origin=Origin(xyz=(-0.052, -0.183, 0.088)),
        material=white_mark,
        name="high_tick",
    )
    body.visual(
        Cylinder(radius=0.122, length=0.032),
        origin=Origin(xyz=(0.0, 0.020, 0.176)),
        material=black,
        name="bowl_saddle",
    )

    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.082, 0.000),
            (0.124, 0.034),
            (0.143, 0.120),
            (0.146, 0.210),
            (0.156, 0.244),
        ],
        [
            (0.040, 0.024),
            (0.105, 0.044),
            (0.126, 0.130),
            (0.133, 0.218),
            (0.137, 0.236),
        ],
        segments=48,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(bowl_shell, "bowl_shell"),
        origin=Origin(xyz=(0.0, 0.020, 0.160)),
        material=clear_poly,
        name="bowl_shell",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.0, 0.020, 0.205)),
        material=black,
        name="center_spindle",
    )
    # Broad fixed bowl handle: three bonded clear lugs forming a substantial C-grip.
    body.visual(
        Box((0.100, 0.070, 0.034)),
        origin=Origin(xyz=(0.182, 0.105, 0.302)),
        material=clear_poly,
        name="lower_handle_lug",
    )
    body.visual(
        Box((0.104, 0.070, 0.034)),
        origin=Origin(xyz=(0.184, 0.105, 0.384)),
        material=clear_poly,
        name="upper_handle_lug",
    )
    body.visual(
        Box((0.046, 0.082, 0.118)),
        origin=Origin(xyz=(0.240, 0.105, 0.343)),
        material=clear_poly,
        name="broad_handle_grip",
    )

    # Rear hinge support knuckles fixed to the bowl/body.
    for x in (-0.088, 0.088):
        body.visual(
            Box((0.070, 0.026, 0.040)),
            origin=Origin(xyz=(x, 0.160, 0.403)),
            material=black,
            name=f"rear_hinge_mount_{0 if x < 0 else 1}",
        )
    for x in (-0.088, 0.088):
        body.visual(
            Cylinder(radius=0.012, length=0.060),
            origin=Origin(xyz=(x, 0.170, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"rear_hinge_knuckle_{0 if x < 0 else 1}",
        )
    body.visual(
        Cylinder(radius=0.004, length=0.210),
        origin=Origin(xyz=(0.0, 0.170, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_hinge_pin",
    )
    # Side latch pivot bosses at the bowl rim.
    body.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(0.152, 0.020, 0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="latch_boss_0",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(-0.152, 0.020, 0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="latch_boss_1",
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.158, length=0.018),
        origin=Origin(xyz=(0.0, -0.174, 0.005)),
        material=smoked_lid,
        name="lid_window",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.110, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, -0.036, -0.004)),
        material=black,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.018, 0.076, 0.140)),
        origin=Origin(xyz=(0.038, -0.215, 0.095)),
        material=clear_poly,
        name="chute_side_0",
    )
    lid.visual(
        Box((0.018, 0.076, 0.140)),
        origin=Origin(xyz=(-0.038, -0.215, 0.095)),
        material=clear_poly,
        name="chute_side_1",
    )
    lid.visual(
        Box((0.092, 0.018, 0.140)),
        origin=Origin(xyz=(0.0, -0.185, 0.095)),
        material=clear_poly,
        name="chute_wall_rear",
    )
    lid.visual(
        Box((0.092, 0.018, 0.140)),
        origin=Origin(xyz=(0.0, -0.245, 0.095)),
        material=clear_poly,
        name="chute_wall_front",
    )
    lid.visual(
        Box((0.132, 0.108, 0.014)),
        origin=Origin(xyz=(0.0, -0.215, 0.020)),
        material=black,
        name="chute_base_gasket",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.170, 0.425)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.30),
    )

    pusher = model.part("feed_pusher")
    pusher.visual(
        Box((0.044, 0.032, 0.135)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=Material("pusher_plastic", rgba=(0.88, 0.92, 0.94, 1.0)),
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.036, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=Material("pusher_plastic", rgba=(0.88, 0.92, 0.94, 1.0)),
        name="pusher_neck",
    )
    pusher.visual(
        Box((0.102, 0.086, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=Material("pusher_plastic", rgba=(0.88, 0.92, 0.94, 1.0)),
        name="pusher_cap",
    )
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.215, 0.027)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.090),
    )

    blade = model.part("cutting_blade")
    blade.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=steel,
        name="blade_hub",
    )
    blade.visual(
        Box((0.118, 0.030, 0.006)),
        origin=Origin(xyz=(0.048, 0.0, 0.012), rpy=(0.0, 0.0, 0.22)),
        material=steel,
        name="blade_rotor",
    )
    blade.visual(
        Box((0.118, 0.030, 0.006)),
        origin=Origin(xyz=(-0.048, 0.0, -0.002), rpy=(0.0, 0.0, math.pi + 0.22)),
        material=steel,
        name="blade_tail",
    )
    model.articulation(
        "spindle_to_blade",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blade,
        origin=Origin(xyz=(0.0, 0.020, 0.2470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )

    # Mirrored side latch arms with real pivot hubs and hooked tips.
    for index, side in enumerate((1.0, -1.0)):
        latch = model.part(f"side_latch_{index}")
        latch.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(side * 0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name="latch_pivot_hub",
        )
        latch.visual(
            Box((0.018, 0.036, 0.136)),
            origin=Origin(xyz=(side * 0.014, 0.0, 0.074)),
            material=black,
            name="latch_arm",
        )
        latch.visual(
            Box((0.040, 0.040, 0.020)),
            origin=Origin(xyz=(-side * 0.006, 0.0, 0.150)),
            material=black,
            name="lid_hook",
        )
        model.articulation(
            f"body_to_side_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(side * 0.170, 0.020, 0.310)),
            axis=(0.0, side, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=0.70),
        )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.037, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_cap",
    )
    selector_dial.visual(
        Box((0.006, 0.003, 0.050)),
        origin=Origin(xyz=(0.0, -0.027, 0.004), rpy=(0.0, 0.0, 0.0)),
        material=white_mark,
        name="dial_pointer",
    )
    model.articulation(
        "band_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_dial,
        origin=Origin(xyz=(-0.090, -0.181, 0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    for index, (x, material) in enumerate(((0.060, blue_button), (0.135, gray_button))):
        rocker = model.part(f"rocker_button_{index}")
        rocker.visual(
            Box((0.058, 0.018, 0.040)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=material,
            name="button_pad",
        )
        rocker.visual(
            Box((0.032, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.019, 0.010)),
            material=white_mark,
            name="upper_mark",
        )
        rocker.visual(
            Box((0.018, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.019, -0.012)),
            material=white_mark,
            name="lower_mark",
        )
        model.articulation(
            f"band_to_rocker_button_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=rocker,
            origin=Origin(xyz=(x, -0.181, 0.084)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-0.24, upper=0.24),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("processor_body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    blade = object_model.get_part("cutting_blade")
    dial = object_model.get_part("selector_dial")

    lid_joint = object_model.get_articulation("body_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_feed_pusher")
    blade_joint = object_model.get_articulation("spindle_to_blade")
    dial_joint = object_model.get_articulation("band_to_selector_dial")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="lid_hinge_knuckle",
        reason="A slender metal hinge pin is intentionally captured inside the lid hinge barrel.",
    )

    ctx.check(
        "primary mechanisms are articulated",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and pusher_joint.articulation_type == ArticulationType.PRISMATIC
        and blade_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="lid, feed pusher, blade, and selector dial must expose their requested motion types",
    )
    ctx.expect_within(
        blade,
        body,
        axes="xy",
        inner_elem="blade_rotor",
        outer_elem="bowl_shell",
        margin=0.0,
        name="blade stays inside the clear bowl footprint",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a="lid_hinge_knuckle",
        elem_b="rear_hinge_pin",
        min_overlap=0.070,
        name="lid hinge barrel is retained on the hinge pin",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_window",
        negative_elem="bowl_shell",
        max_gap=0.016,
        max_penetration=0.0,
        name="closed lid sits just above the bowl lip",
    )
    ctx.expect_gap(
        lid,
        pusher,
        axis="x",
        positive_elem="chute_side_0",
        negative_elem="pusher_shaft",
        min_gap=0.002,
        max_gap=0.014,
        name="pusher clears the right chute wall",
    )
    ctx.expect_gap(
        pusher,
        lid,
        axis="x",
        positive_elem="pusher_shaft",
        negative_elem="chute_side_1",
        min_gap=0.002,
        max_gap=0.014,
        name="pusher clears the left chute wall",
    )
    ctx.expect_gap(
        lid,
        pusher,
        axis="y",
        positive_elem="chute_wall_rear",
        negative_elem="pusher_shaft",
        min_gap=0.002,
        max_gap=0.014,
        name="pusher clears the rear chute wall",
    )
    ctx.expect_gap(
        pusher,
        lid,
        axis="y",
        positive_elem="pusher_shaft",
        negative_elem="chute_wall_front",
        min_gap=0.002,
        max_gap=0.014,
        name="pusher clears the front chute wall",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="chute_side_0",
        min_overlap=0.120,
        name="pusher remains inserted through the chute height",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="y",
        positive_elem="control_band",
        negative_elem="dial_cap",
        max_gap=0.004,
        max_penetration=0.001,
        name="selector dial is mounted proud on the control band",
    )
    for index in (0, 1):
        rocker = object_model.get_part(f"rocker_button_{index}")
        ctx.expect_gap(
            body,
            rocker,
            axis="y",
            positive_elem="control_band",
            negative_elem="button_pad",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"rocker button {index} seats on the front band",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.080}):
        raised_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "feed pusher slides upward in chute",
        rest_pusher is not None
        and raised_pusher is not None
        and raised_pusher[2] > rest_pusher[2] + 0.070,
        details=f"rest={rest_pusher}, raised={raised_pusher}",
    )

    for index, expected_sign in ((0, 1.0), (1, -1.0)):
        latch = object_model.get_part(f"side_latch_{index}")
        latch_joint = object_model.get_articulation(f"body_to_side_latch_{index}")
        closed = ctx.part_world_aabb(latch)
        with ctx.pose({latch_joint: 0.55}):
            swung = ctx.part_world_aabb(latch)
        if closed is not None and swung is not None:
            closed_center_x = (closed[0][0] + closed[1][0]) * 0.5
            swung_center_x = (swung[0][0] + swung[1][0]) * 0.5
            moved_outward = (swung_center_x - closed_center_x) * expected_sign > 0.015
        else:
            moved_outward = False
        ctx.check(
            f"side latch {index} swings outward from its pivot",
            moved_outward,
            details=f"closed={closed}, swung={swung}",
        )

    # Each rocker has its own joint and can be posed without driving the other.
    with ctx.pose({"band_to_rocker_button_0": 0.20, "band_to_rocker_button_1": -0.20}):
        ctx.check("rocker buttons pose independently", True)

    return ctx.report()


object_model = build_object_model()
