from __future__ import annotations

from math import cos, pi, sin

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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_juicer")

    white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("dark_graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.69, 1.0))
    clear = model.material("clear_smoked_polycarbonate", rgba=(0.62, 0.82, 0.95, 0.34))
    amber = model.material("amber_pusher_plastic", rgba=(0.96, 0.76, 0.28, 0.72))
    button_red = model.material("red_push_button", rgba=(0.82, 0.04, 0.025, 1.0))

    # Root appliance body: tall, slightly tapered rotational housing with a
    # rubber foot, rear hinge support, side latch pivot, and front juice spout.
    body = model.part("body")
    body_shell = LatheGeometry(
        [
            (0.0, 0.000),
            (0.158, 0.000),
            (0.166, 0.028),
            (0.148, 0.090),
            (0.132, 0.190),
            (0.108, 0.242),
            (0.0, 0.242),
        ],
        segments=72,
        closed=True,
    )
    body.visual(mesh_from_geometry(body_shell, "tapered_body"), material=white, name="tapered_body")
    body.visual(Cylinder(radius=0.175, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.013)), material=black, name="rubber_foot")
    body.visual(Cylinder(radius=0.035, length=0.065), origin=Origin(xyz=(0.0, 0.0, 0.274)), material=dark, name="motor_coupling")
    body.visual(Cylinder(radius=0.015, length=0.080), origin=Origin(xyz=(0.0, -0.158, 0.225), rpy=(pi / 2.0, 0.0, 0.0)), material=stainless, name="juice_spout")
    body.visual(Box((0.150, 0.080, 0.034)), origin=Origin(xyz=(0.0, 0.140, 0.239)), material=dark, name="rear_web")
    body.visual(Box((0.150, 0.040, 0.095)), origin=Origin(xyz=(0.0, 0.182, 0.283)), material=dark, name="rear_hinge_stand")
    body.visual(Cylinder(radius=0.012, length=0.170), origin=Origin(xyz=(0.0, 0.151, 0.318), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="rear_hinge_pin")
    body.visual(Cylinder(radius=0.021, length=0.050), origin=Origin(xyz=(0.150, 0.0, 0.205), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="arm_pivot_boss")
    body.visual(Cylinder(radius=0.024, length=0.030), origin=Origin(xyz=(0.145, 0.0, 0.145), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="button_socket")

    # The stainless filter basket sits above the motor coupling and is a
    # separate continuously rotating link.
    basket = model.part("basket")
    basket_shell = LatheGeometry.from_shell_profiles(
        [(0.036, -0.046), (0.078, -0.012), (0.102, 0.046)],
        [(0.026, -0.038), (0.067, -0.006), (0.088, 0.040)],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=5,
    )
    basket.visual(mesh_from_geometry(basket_shell, "filter_basket"), material=stainless, name="filter_basket")
    basket.visual(Cylinder(radius=0.026, length=0.034), origin=Origin(xyz=(0.0, 0.0, -0.045)), material=dark, name="basket_hub")
    for i in range(20):
        theta = 2.0 * pi * i / 20.0
        radius = 0.089
        basket.visual(
            Box((0.0035, 0.010, 0.045)),
            origin=Origin(xyz=(radius * cos(theta), radius * sin(theta), 0.012), rpy=(0.0, 0.0, theta)),
            material=dark,
            name=f"perforation_{i}",
        )
    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=120.0),
    )

    # Clear hinged cover/chamber.  The part frame is the rear hinge axis; the
    # transparent chamber extends forward from it and carries the vertical feed
    # chute.
    lid = model.part("lid")
    lid_bowl = LatheGeometry.from_shell_profiles(
        [(0.136, 0.046), (0.138, 0.080), (0.125, 0.127), (0.060, 0.144)],
        [(0.122, 0.051), (0.123, 0.076), (0.110, 0.120), (0.049, 0.137)],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=6,
    )
    lid_bowl.translate(0.0, -0.135, 0.0)
    lid.visual(mesh_from_geometry(lid_bowl, "clear_chamber"), material=clear, name="clear_chamber")
    chute = LatheGeometry.from_shell_profiles(
        [(0.046, 0.072), (0.046, 0.287)],
        [(0.034, 0.072), (0.034, 0.287)],
        segments=48,
        start_cap="round",
        end_cap="round",
        lip_samples=4,
    )
    chute.translate(0.0, -0.145, 0.0)
    lid.visual(mesh_from_geometry(chute, "feed_chute_tube"), material=clear, name="feed_chute_tube")
    chute_collar = LatheGeometry.from_shell_profiles(
        [(0.064, 0.067), (0.064, 0.088)],
        [(0.034, 0.067), (0.034, 0.088)],
        segments=48,
        start_cap="round",
        end_cap="round",
        lip_samples=4,
    )
    chute_collar.translate(0.0, -0.145, 0.0)
    lid.visual(mesh_from_geometry(chute_collar, "chute_collar"), material=clear, name="chute_collar")
    lid.visual(Box((0.135, 0.085, 0.014)), origin=Origin(xyz=(0.0, 0.020, 0.015)), material=clear, name="hinge_leaf")
    lid.visual(Box((0.105, 0.045, 0.054)), origin=Origin(xyz=(0.0, 0.020, 0.046)), material=clear, name="lid_bridge")
    lid.visual(Cylinder(radius=0.012, length=0.125), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=clear, name="lid_hinge_barrel")
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.151, 0.318)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.18),
    )

    # Long pusher telescopes through the clear chute.  At q=0 it is inserted; a
    # positive command raises it while retaining length inside the tube.
    pusher = model.part("pusher")
    pusher.visual(Cylinder(radius=0.027, length=0.250), origin=Origin(xyz=(0.0, 0.0, -0.125)), material=amber, name="pusher_shaft")
    pusher.visual(Cylinder(radius=0.020, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=amber, name="pusher_neck")
    pusher.visual(Cylinder(radius=0.046, length=0.028), origin=Origin(xyz=(0.0, 0.0, 0.02075)), material=amber, name="pusher_cap")
    pusher.visual(Cylinder(radius=0.021, length=0.030), origin=Origin(xyz=(0.0, 0.0, -0.265)), material=amber, name="pusher_tip")
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.145, 0.287)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.160),
    )

    # One-sided locking arm with a curved metal bail.  Positive rotation swings
    # the latch outward from the clear lid.
    locking_arm = model.part("locking_arm")
    arm_tube = tube_from_spline_points(
        [
            (0.000, 0.0, 0.000),
            (0.000, 0.0, 0.060),
            (0.000, 0.0, 0.125),
            (-0.004, 0.0, 0.178),
            (-0.010, 0.0, 0.210),
        ],
        radius=0.007,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    locking_arm.visual(mesh_from_geometry(arm_tube, "locking_arm_tube"), material=stainless, name="locking_arm_tube")
    locking_arm.visual(Cylinder(radius=0.023, length=0.064), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=stainless, name="arm_pivot_barrel")
    locking_arm.visual(Box((0.040, 0.050, 0.016)), origin=Origin(xyz=(-0.030, 0.0, 0.210)), material=dark, name="latch_pad")
    model.articulation(
        "body_to_locking_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=locking_arm,
        origin=Origin(xyz=(0.170, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=0.95),
    )

    # Short side push button below the arm pivot.
    button = model.part("button")
    button.visual(Cylinder(radius=0.018, length=0.026), origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=button_red, name="button_cap")
    button.visual(Cylinder(radius=0.011, length=0.020), origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark, name="button_stem")
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.159, 0.0, 0.145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    locking_arm = object_model.get_part("locking_arm")
    button = object_model.get_part("button")
    basket = object_model.get_part("basket")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    arm_pivot = object_model.get_articulation("body_to_locking_arm")
    button_plunger = object_model.get_articulation("body_to_button")
    basket_spin = object_model.get_articulation("body_to_basket")

    ctx.allow_overlap(
        body,
        basket,
        elem_a="motor_coupling",
        elem_b="filter_basket",
        reason="The basket's lower filter cone nests locally over the rotating drive coupling.",
    )
    ctx.expect_overlap(
        body,
        basket,
        axes="z",
        elem_a="motor_coupling",
        elem_b="filter_basket",
        min_overlap=0.020,
        name="basket remains seated over the drive coupling",
    )
    ctx.allow_overlap(
        body,
        basket,
        elem_a="motor_coupling",
        elem_b="basket_hub",
        reason="The basket hub is intentionally keyed over the motor coupling.",
    )
    ctx.expect_overlap(
        body,
        basket,
        axes="xyz",
        elem_a="motor_coupling",
        elem_b="basket_hub",
        min_overlap=0.020,
        name="basket hub captures the motor coupling",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_hinge_pin",
        elem_b="lid_hinge_barrel",
        reason="The clear lid hinge barrel is intentionally captured around the rear hinge pin.",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="rear_hinge_pin",
        elem_b="lid_hinge_barrel",
        min_overlap=0.100,
        name="lid barrel spans the rear hinge pin",
    )
    ctx.allow_overlap(
        body,
        locking_arm,
        elem_a="arm_pivot_boss",
        elem_b="arm_pivot_barrel",
        reason="The locking arm barrel is intentionally mounted around the side pivot boss.",
    )
    ctx.expect_overlap(
        body,
        locking_arm,
        axes="xyz",
        elem_a="arm_pivot_boss",
        elem_b="arm_pivot_barrel",
        min_overlap=0.020,
        name="locking arm barrel captures the side pivot boss",
    )
    ctx.allow_overlap(
        body,
        locking_arm,
        elem_a="arm_pivot_boss",
        elem_b="locking_arm_tube",
        reason="The arm tube is welded into the side pivot boss at the rotating latch root.",
    )
    ctx.expect_overlap(
        body,
        locking_arm,
        axes="xyz",
        elem_a="arm_pivot_boss",
        elem_b="locking_arm_tube",
        min_overlap=0.010,
        name="locking arm tube is rooted in the pivot boss",
    )
    ctx.allow_overlap(
        body,
        button,
        elem_a="button_socket",
        elem_b="button_stem",
        reason="The button stem intentionally enters the short side socket as a plunger.",
    )
    ctx.expect_overlap(
        body,
        button,
        axes="yz",
        elem_a="button_socket",
        elem_b="button_stem",
        min_overlap=0.010,
        name="button stem is retained in the side socket",
    )
    ctx.allow_overlap(
        lid,
        locking_arm,
        elem_a="clear_chamber",
        elem_b="latch_pad",
        reason="The latch pad is modeled as a small compressed catch bearing on the clear lid edge.",
    )
    ctx.expect_overlap(
        lid,
        locking_arm,
        axes="xyz",
        elem_a="clear_chamber",
        elem_b="latch_pad",
        min_overlap=0.010,
        name="locking arm latch pad bears on the clear lid",
    )

    ctx.check(
        "all requested joint families are present",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and pusher_slide.articulation_type == ArticulationType.PRISMATIC
        and arm_pivot.articulation_type == ArticulationType.REVOLUTE
        and button_plunger.articulation_type == ArticulationType.PRISMATIC
        and basket_spin.articulation_type == ArticulationType.CONTINUOUS,
    )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="feed_chute_tube",
        margin=0.004,
        name="pusher shaft stays centered in the chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="feed_chute_tube",
        min_overlap=0.150,
        name="inserted pusher remains engaged in the chute",
    )

    lid_rest = ctx.part_element_world_aabb(lid, elem="clear_chamber")
    with ctx.pose({lid_hinge: 1.05}):
        lid_open = ctx.part_element_world_aabb(lid, elem="clear_chamber")
    ctx.check(
        "lid hinge lifts the clear chamber",
        lid_rest is not None and lid_open is not None and lid_open[1][2] > lid_rest[1][2] + 0.035,
        details=f"closed={lid_rest}, open={lid_open}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.150}):
        pusher_raised = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="feed_chute_tube",
            margin=0.004,
            name="raised pusher remains centered in the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="feed_chute_tube",
            min_overlap=0.070,
            name="raised pusher retains insertion in the chute",
        )
    ctx.check(
        "pusher slide raises along the vertical chute axis",
        pusher_rest is not None and pusher_raised is not None and pusher_raised[2] > pusher_rest[2] + 0.12,
        details=f"rest={pusher_rest}, raised={pusher_raised}",
    )

    arm_rest = ctx.part_element_world_aabb(locking_arm, elem="latch_pad")
    with ctx.pose({arm_pivot: 0.70}):
        arm_unlatched = ctx.part_element_world_aabb(locking_arm, elem="latch_pad")
    ctx.check(
        "locking arm swings outward from the side pivot",
        arm_rest is not None and arm_unlatched is not None and arm_unlatched[1][0] > arm_rest[1][0] + 0.045,
        details=f"latched={arm_rest}, unlatched={arm_unlatched}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_plunger: 0.010}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "button plunger moves inward",
        button_rest is not None and button_pressed is not None and button_pressed[0] < button_rest[0] - 0.008,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    ctx.expect_overlap(
        basket,
        body,
        axes="xy",
        elem_a="basket_hub",
        elem_b="motor_coupling",
        min_overlap=0.035,
        name="rotating basket is centered over the motor coupling",
    )

    return ctx.report()


object_model = build_object_model()
