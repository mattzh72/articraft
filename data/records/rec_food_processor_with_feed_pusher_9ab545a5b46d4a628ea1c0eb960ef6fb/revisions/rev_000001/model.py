from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_food_processor")

    base_plastic = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    band_black = model.material("gloss_black_control_band", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    clear_plastic = model.material("clear_safety_plastic", rgba=(0.70, 0.93, 1.0, 0.36))
    smoke_clear = model.material("smoke_clear_plastic", rgba=(0.52, 0.67, 0.74, 0.45))
    stainless = model.material("brushed_stainless", rgba=(0.82, 0.82, 0.78, 1.0))
    button_mat = model.material("soft_grey_buttons", rgba=(0.72, 0.74, 0.72, 1.0))
    label_blue = model.material("preset_blue_mark", rgba=(0.12, 0.33, 0.78, 1.0))

    base = model.part("base_housing")

    base_geom = ExtrudeGeometry(
        rounded_rect_profile(0.54, 0.42, 0.065, corner_segments=10),
        0.105,
        center=False,
    )
    base.visual(
        mesh_from_geometry(base_geom, "rounded_low_base"),
        material=base_plastic,
        name="rounded_low_base",
    )

    base.visual(
        Box((0.405, 0.012, 0.066)),
        origin=Origin(xyz=(0.0, -0.216, 0.066)),
        material=band_black,
        name="front_control_band",
    )

    # Two rear hinge towers visibly carry the lid hinge above the bowl rim.
    for x in (-0.125, 0.125):
        base.visual(
            Box((0.050, 0.034, 0.205)),
            origin=Origin(xyz=(x, 0.207, 0.2075)),
            material=base_plastic,
            name=f"rear_hinge_tower_{0 if x < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.070),
            origin=Origin(xyz=(x, 0.201, 0.321), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_rubber,
            name=f"fixed_hinge_knuckle_{0 if x < 0 else 1}",
        )

    base.visual(
        Cylinder(radius=0.017, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=dark_rubber,
        name="central_spindle",
    )

    # Small rubber feet make the countertop appliance scale and support explicit.
    for i, (x, y) in enumerate(((-0.20, -0.14), (0.20, -0.14), (-0.20, 0.14), (0.20, 0.14))):
        base.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(x, y, -0.005)),
            material=dark_rubber,
            name=f"rubber_foot_{i}",
        )

    bowl = model.part("bowl")
    # Lathed inner and outer profiles form a transparent thin-wall bowl with a
    # real open prep cavity and a small thickened floor over the drive socket.
    bowl_outer = [
        (0.052, 0.000),
        (0.118, 0.008),
        (0.168, 0.045),
        (0.187, 0.145),
        (0.196, 0.205),
    ]
    bowl_inner = [
        (0.026, 0.012),
        (0.106, 0.020),
        (0.154, 0.055),
        (0.173, 0.145),
        (0.184, 0.195),
    ]
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                bowl_outer,
                bowl_inner,
                segments=72,
                start_cap="flat",
                end_cap="round",
                lip_samples=8,
            ),
            "clear_bowl_shell",
        ),
        material=clear_plastic,
        name="bowl_shell",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    lid = model.part("lid")
    # The lid part frame is the rear hinge line.  The shell's circular center is
    # forward of that line so positive hinge motion can raise the front edge.
    lid_outer = [
        (0.012, 0.017),
        (0.070, 0.026),
        (0.154, 0.025),
        (0.190, 0.014),
        (0.196, 0.000),
    ]
    lid_inner = [
        (0.006, 0.011),
        (0.066, 0.020),
        (0.150, 0.019),
        (0.181, 0.006),
    ]
    lid.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                lid_outer,
                lid_inner,
                segments=72,
                start_cap="flat",
                end_cap="round",
                lip_samples=8,
            ),
            "clear_lid_shell",
        ),
        origin=Origin(xyz=(0.0, -0.207, -0.003)),
        material=clear_plastic,
        name="lid_shell",
    )

    # Four separate clear walls make the feed chute genuinely hollow so the
    # matching pusher can slide through the open center without intersecting a
    # simplified solid proxy.
    chute_x = 0.055
    chute_y = -0.146
    wall = 0.011
    outer_w = 0.105
    outer_d = 0.086
    chute_h = 0.182
    lid.visual(
        Box((wall, outer_d, chute_h)),
        origin=Origin(xyz=(chute_x - outer_w / 2 + wall / 2, chute_y, 0.118)),
        material=clear_plastic,
        name="feed_chute_side_0",
    )
    lid.visual(
        Box((wall, outer_d, chute_h)),
        origin=Origin(xyz=(chute_x + outer_w / 2 - wall / 2, chute_y, 0.118)),
        material=clear_plastic,
        name="feed_chute_side_1",
    )
    lid.visual(
        Box((outer_w - 2 * wall + 0.002, wall, chute_h)),
        origin=Origin(xyz=(chute_x, chute_y - outer_d / 2 + wall / 2, 0.118)),
        material=clear_plastic,
        name="feed_chute_front_wall",
    )
    lid.visual(
        Box((outer_w - 2 * wall + 0.002, wall, chute_h)),
        origin=Origin(xyz=(chute_x, chute_y + outer_d / 2 - wall / 2, 0.118)),
        material=clear_plastic,
        name="feed_chute_rear_wall",
    )

    # A raised frame, not a solid plate, seats the chute to the lid while
    # leaving the feed opening and prep cavity visually open.
    lid.visual(
        Box((0.125, 0.014, 0.009)),
        origin=Origin(xyz=(chute_x, chute_y - outer_d / 2 - 0.002, 0.024)),
        material=smoke_clear,
        name="chute_base_front",
    )
    lid.visual(
        Box((0.125, 0.014, 0.009)),
        origin=Origin(xyz=(chute_x, chute_y + outer_d / 2 + 0.002, 0.024)),
        material=smoke_clear,
        name="chute_base_rear",
    )
    lid.visual(
        Box((0.014, 0.086, 0.009)),
        origin=Origin(xyz=(chute_x - outer_w / 2 - 0.002, chute_y, 0.024)),
        material=smoke_clear,
        name="chute_base_side_0",
    )
    lid.visual(
        Box((0.014, 0.086, 0.009)),
        origin=Origin(xyz=(chute_x + outer_w / 2 + 0.002, chute_y, 0.024)),
        material=smoke_clear,
        name="chute_base_side_1",
    )
    lid.visual(
        Cylinder(radius=0.0095, length=0.180),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="moving_hinge_knuckle",
    )
    lid.visual(
        Box((0.170, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, 0.010)),
        material=smoke_clear,
        name="hinge_leaf",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.207, 0.321)),
        # Closed lid geometry extends along local -Y from the rear hinge.
        # -X makes positive q lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.25),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.074, 0.056, 0.155)),
        origin=Origin(xyz=(0.0, 0.0, -0.0775)),
        material=smoke_clear,
        name="pusher_body",
    )
    pusher.visual(
        Box((0.124, 0.102, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=base_plastic,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.060, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.052, 0.025)),
        material=dark_rubber,
        name="pusher_grip_groove",
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.055, -0.146, 0.209)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.085),
    )

    cutter = model.part("cutter_disc")
    cutter.visual(
        Cylinder(radius=0.135, length=0.006),
        origin=Origin(),
        material=stainless,
        name="disc_plate",
    )
    cutter.visual(
        Cylinder(radius=0.029, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_rubber,
        name="disc_hub",
    )
    cutter.visual(
        Box((0.095, 0.017, 0.004)),
        origin=Origin(xyz=(0.055, 0.012, 0.006), rpy=(0.0, 0.0, math.radians(18))),
        material=stainless,
        name="raised_slicing_blade",
    )
    cutter.visual(
        Box((0.080, 0.010, 0.002)),
        origin=Origin(xyz=(-0.050, -0.022, 0.004), rpy=(0.0, 0.0, math.radians(18))),
        material=dark_rubber,
        name="blade_slot_shadow",
    )
    model.articulation(
        "base_to_cutter_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cutter,
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )

    dial = model.part("selector_dial")
    selector_knob = KnobGeometry(
        0.064,
        0.030,
        body_style="skirted",
        top_diameter=0.052,
        skirt=KnobSkirt(0.074, 0.006, flare=0.05, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=20, depth=0.0013),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0010, angle_deg=0.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(selector_knob, "selector_dial_cap"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="dial_cap",
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(-0.135, -0.222, 0.068)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    for i, x in enumerate((0.045, 0.110, 0.175)):
        button = model.part(f"preset_button_{i}")
        button.visual(
            Box((0.047, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.002, 0.004)),
            origin=Origin(xyz=(0.0, -0.0145, 0.007)),
            material=label_blue,
            name="preset_mark",
        )
        model.articulation(
            f"base_to_preset_button_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.222, 0.068)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.10, lower=0.0, upper=0.008),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    cutter = object_model.get_part("cutter_disc")
    dial = object_model.get_part("selector_dial")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    cutter_spin = object_model.get_articulation("base_to_cutter_disc")
    dial_spin = object_model.get_articulation("base_to_selector_dial")

    ctx.expect_overlap(lid, bowl, axes="xy", min_overlap=0.30, name="lid covers the wide bowl")
    ctx.expect_within(cutter, bowl, axes="xy", margin=0.012, name="cutter disc sits inside bowl footprint")
    ctx.expect_contact(
        cutter,
        base,
        elem_a="disc_hub",
        elem_b="central_spindle",
        contact_tol=0.002,
        name="cutter hub is carried by central spindle",
    )
    ctx.expect_gap(
        lid,
        pusher,
        axis="x",
        positive_elem="feed_chute_side_1",
        negative_elem="pusher_body",
        min_gap=0.002,
        max_gap=0.010,
        name="pusher clears chute side wall 1",
    )
    ctx.expect_gap(
        pusher,
        lid,
        axis="x",
        positive_elem="pusher_body",
        negative_elem="feed_chute_side_0",
        min_gap=0.002,
        max_gap=0.010,
        name="pusher clears chute side wall 0",
    )
    ctx.expect_gap(
        lid,
        pusher,
        axis="y",
        positive_elem="feed_chute_rear_wall",
        negative_elem="pusher_body",
        min_gap=0.002,
        max_gap=0.010,
        name="pusher clears rear chute wall",
    )
    ctx.expect_gap(
        pusher,
        lid,
        axis="y",
        positive_elem="pusher_body",
        negative_elem="feed_chute_front_wall",
        min_gap=0.002,
        max_gap=0.010,
        name="pusher clears front chute wall",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_body",
        elem_b="feed_chute_side_0",
        min_overlap=0.12,
        name="pusher is inserted into the chute at rest",
    )
    ctx.expect_contact(dial, base, elem_a="dial_cap", elem_b="front_control_band", contact_tol=0.003, name="dial mounts on front band")

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 0.95}):
        opened_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens lid upward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.09,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.075}):
        raised_pusher = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_body",
            elem_b="feed_chute_side_0",
            min_overlap=0.045,
            name="raised pusher remains guided by chute",
        )
    ctx.check(
        "pusher slides upward out of feed chute",
        rest_pusher is not None and raised_pusher is not None and raised_pusher[2] > rest_pusher[2] + 0.070,
        details=f"rest={rest_pusher}, raised={raised_pusher}",
    )

    ctx.check("cutter disc has continuous spin joint", cutter_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("selector dial has continuous spin joint", dial_spin.articulation_type == ArticulationType.CONTINUOUS)

    for i in range(3):
        joint = object_model.get_articulation(f"base_to_preset_button_{i}")
        button = object_model.get_part(f"preset_button_{i}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.008}):
            pushed = ctx.part_world_position(button)
        ctx.check(
            f"preset button {i} pushes inward",
            rest is not None and pushed is not None and pushed[1] > rest[1] + 0.006,
            details=f"rest={rest}, pushed={pushed}",
        )

    return ctx.report()


object_model = build_object_model()
