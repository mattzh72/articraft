from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_folding_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.76, 0.78, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    urethane = model.material("smoky_urethane", rgba=(0.06, 0.08, 0.10, 1.0))
    teal = model.material("teal_deck", rgba=(0.02, 0.42, 0.54, 1.0))
    grip = model.material("sandpaper_grip", rgba=(0.015, 0.015, 0.014, 1.0))
    red = model.material("red_clamp", rgba=(0.86, 0.08, 0.04, 1.0))

    front_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.042,
            inner_radius=0.054,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
            tread=TireTread(style="chevron", depth=0.0022, count=18, angle_deg=22.0, land_ratio=0.64),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0012),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "front_urethane_tire",
    )
    front_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.056,
            0.034,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.0025),
            hub=WheelHub(
                radius=0.017,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.023, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0023, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.017),
        ),
        "front_spoked_rim",
    )
    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.036,
            inner_radius=0.039,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.03),
            tread=TireTread(style="chevron", depth=0.0018, count=16, angle_deg=20.0, land_ratio=0.66),
            grooves=(TireGroove(center_offset=0.0, width=0.0035, depth=0.001),),
            sidewall=TireSidewall(style="rounded", bulge=0.025),
            shoulder=TireShoulder(width=0.0035, radius=0.0018),
        ),
        "rear_urethane_tire",
    )
    rear_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.040,
            0.029,
            rim=WheelRim(inner_radius=0.027, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(radius=0.013, width=0.025, cap_style="domed"),
            face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0018, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "rear_spoked_rim",
    )

    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.62, 0.165, 0.035, corner_segments=8),
            0.034,
            center=True,
        ),
        "rounded_deck",
    )

    front_frame = model.part("front_frame")
    # A wide two-wheel fork and axle crown: one rigid, connected front assembly.
    for y, name in ((-0.155, "fork_plate_0"), (0.155, "fork_plate_1")):
        front_frame.visual(
            Box((0.058, 0.018, 0.215)),
            origin=Origin(xyz=(0.0, y, 0.170)),
            material=aluminum,
            name=name,
        )
        front_frame.visual(
            Cylinder(radius=0.010, length=0.117),
            origin=Origin(xyz=(0.0, -0.1165 if y < 0 else 0.1165, 0.075), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"axle_stub_{0 if y < 0 else 1}",
        )
    front_frame.visual(
        Box((0.118, 0.350, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.267)),
        material=aluminum,
        name="fork_crown",
    )
    front_frame.visual(
        Cylinder(radius=0.012, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.075), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="deck_hinge_pin",
    )
    front_frame.visual(
        Box((0.030, 0.082, 0.044)),
        origin=Origin(xyz=(0.012, 0.0, 0.096)),
        material=aluminum,
        name="hinge_pin_boss",
    )
    front_frame.visual(
        Box((0.025, 0.070, 0.160)),
        origin=Origin(xyz=(0.015, 0.0, 0.170)),
        material=aluminum,
        name="hinge_boss_strut",
    )
    front_frame.visual(
        Cylinder(radius=0.025, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=aluminum,
        name="outer_sleeve",
    )
    front_frame.visual(
        Cylinder(radius=0.033, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.676)),
        material=red,
        name="clamp_collar",
    )
    front_frame.visual(
        Box((0.018, 0.095, 0.020)),
        origin=Origin(xyz=(0.034, 0.046, 0.676)),
        material=red,
        name="quick_release_lever",
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.016, length=0.780),
        # Hidden lower length remains inside the outer sleeve throughout travel.
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=aluminum,
        name="inner_mast",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.420), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="crossbar",
    )
    for y, name in ((-0.285, "grip_0"), (0.285, "grip_1")):
        handlebar.visual(
            Cylinder(radius=0.021, length=0.115),
            origin=Origin(xyz=(0.0, y, 0.420), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=name,
        )

    deck = model.part("deck")
    deck.visual(
        deck_mesh,
        origin=Origin(xyz=(-0.440, 0.0, 0.040)),
        material=teal,
        name="deck_shell",
    )
    deck.visual(
        Box((0.545, 0.125, 0.004)),
        origin=Origin(xyz=(-0.450, 0.0, 0.0585)),
        material=grip,
        name="grip_tape",
    )
    for x, name in ((-0.34, "grip_bar_0"), (-0.50, "grip_bar_1"), (-0.66, "grip_bar_2")):
        deck.visual(
            Box((0.055, 0.118, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.0615)),
            material=black_rubber,
            name=name,
        )
    deck.visual(
        Box((0.130, 0.075, 0.050)),
        origin=Origin(xyz=(-0.071, 0.0, 0.025)),
        material=teal,
        name="front_hinge_web",
    )
    deck.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_hinge_sleeve",
    )
    for y, name in ((-0.046, "rear_fork_arm_0"), (0.046, "rear_fork_arm_1")):
        deck.visual(
            Box((0.155, 0.012, 0.018)),
            origin=Origin(xyz=(-0.805, y, -0.020)),
            material=aluminum,
            name=name,
        )
        deck.visual(
            Box((0.032, 0.012, 0.060)),
            origin=Origin(xyz=(-0.735, y, 0.010)),
            material=aluminum,
            name=f"rear_dropout_{0 if y < 0 else 1}",
        )
    deck.visual(
        Cylinder(radius=0.0075, length=0.108),
        origin=Origin(xyz=(-0.840, 0.0, -0.020), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle_pin",
    )

    for y, name in ((-0.080, "front_wheel_0"), (0.080, "front_wheel_1")):
        wheel = model.part(name)
        wheel.visual(
            front_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=urethane,
            name="tire",
        )
        wheel.visual(
            front_rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=aluminum,
            name="rim",
        )
        model.articulation(
            f"front_frame_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=front_frame,
            child=wheel,
            origin=Origin(xyz=(0.0, y, 0.075)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        rear_tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=urethane,
        name="tire",
    )
    rear_wheel.visual(
        rear_rim_mesh,
        origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
        material=aluminum,
        name="rim",
    )

    model.articulation(
        "front_frame_to_handlebar",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.0, 0.685)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.220),
    )
    model.articulation(
        "front_frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.840, 0.0, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    handlebar = object_model.get_part("handlebar")
    deck = object_model.get_part("deck")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    rear_wheel = object_model.get_part("rear_wheel")
    slide = object_model.get_articulation("front_frame_to_handlebar")
    deck_hinge = object_model.get_articulation("front_frame_to_deck")

    ctx.allow_overlap(
        front,
        handlebar,
        elem_a="outer_sleeve",
        elem_b="inner_mast",
        reason="The simplified inner mast is intentionally retained inside the telescoping outer sleeve.",
    )
    ctx.allow_overlap(
        front,
        handlebar,
        elem_a="clamp_collar",
        elem_b="inner_mast",
        reason="The clamp collar encircles the moving mast and is simplified as a solid band around it.",
    )
    ctx.expect_within(
        handlebar,
        front,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="telescoping mast is centered in the sleeve",
    )
    ctx.expect_overlap(
        handlebar,
        front,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.15,
        name="collapsed mast remains inserted",
    )
    ctx.expect_within(
        handlebar,
        front,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="clamp_collar",
        margin=0.002,
        name="mast passes through the clamp collar center",
    )
    ctx.expect_overlap(
        handlebar,
        front,
        axes="z",
        elem_a="inner_mast",
        elem_b="clamp_collar",
        min_overlap=0.035,
        name="clamp collar surrounds mast at the sleeve top",
    )

    ctx.allow_overlap(
        front,
        deck,
        elem_a="deck_hinge_pin",
        elem_b="front_hinge_sleeve",
        reason="The deck hinge sleeve intentionally wraps the front-axle hinge pin.",
    )
    ctx.allow_overlap(
        front,
        deck,
        elem_a="deck_hinge_pin",
        elem_b="front_hinge_web",
        reason="The solid hinge web is locally simplified around the same captured front-axle pin.",
    )
    ctx.allow_overlap(
        front,
        deck,
        elem_a="hinge_pin_boss",
        elem_b="front_hinge_sleeve",
        reason="The fixed front hinge boss locally cups the rotating deck sleeve around the axle.",
    )
    ctx.expect_within(
        front,
        deck,
        axes="xz",
        inner_elem="deck_hinge_pin",
        outer_elem="front_hinge_sleeve",
        margin=0.006,
        name="hinge pin sits inside deck sleeve section",
    )
    ctx.expect_overlap(
        deck,
        front,
        axes="y",
        elem_a="front_hinge_sleeve",
        elem_b="deck_hinge_pin",
        min_overlap=0.09,
        name="deck hinge sleeve spans the pin",
    )
    ctx.expect_overlap(
        deck,
        front,
        axes="y",
        elem_a="front_hinge_web",
        elem_b="deck_hinge_pin",
        min_overlap=0.07,
        name="deck hinge web is local to the hinge pin",
    )
    ctx.expect_overlap(
        deck,
        front,
        axes="y",
        elem_a="front_hinge_sleeve",
        elem_b="hinge_pin_boss",
        min_overlap=0.07,
        name="hinge boss locally surrounds deck sleeve",
    )

    for wheel_part, stub_name, check_name in (
        (front_wheel_0, "axle_stub_0", "front wheel 0 captured by axle"),
        (front_wheel_1, "axle_stub_1", "front wheel 1 captured by axle"),
    ):
        ctx.allow_overlap(
            front,
            wheel_part,
            elem_a=stub_name,
            elem_b="rim",
            reason="The fork axle stub is intentionally captured through the wheel hub bore.",
        )
        ctx.expect_overlap(
            wheel_part,
            front,
            axes="y",
            elem_a="rim",
            elem_b=stub_name,
            min_overlap=0.035,
            name=check_name,
        )

    rest_handle_pos = ctx.part_world_position(handlebar)
    with ctx.pose({slide: 0.220}):
        extended_handle_pos = ctx.part_world_position(handlebar)
        ctx.expect_within(
            handlebar,
            front,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast stays centered",
        )
        ctx.expect_overlap(
            handlebar,
            front,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.09,
            name="extended mast retains insertion",
        )
    ctx.check(
        "handlebar column extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.20,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    rest_deck_aabb = ctx.part_world_aabb(deck)
    with ctx.pose({deck_hinge: 1.45}):
        folded_deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "deck folds upward around front axle",
        rest_deck_aabb is not None
        and folded_deck_aabb is not None
        and folded_deck_aabb[1][2] > rest_deck_aabb[1][2] + 0.35,
        details=f"rest={rest_deck_aabb}, folded={folded_deck_aabb}",
    )

    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="x",
        elem_a="rim",
        elem_b="rear_axle_pin",
        min_overlap=0.010,
        name="rear wheel is carried by the rear axle",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle_pin",
        elem_b="rim",
        reason="The rear axle pin is intentionally captured through the wheel hub bore.",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="y",
        elem_a="rim",
        elem_b="rear_axle_pin",
        min_overlap=0.025,
        name="rear axle crosses the wheel hub width",
    )

    return ctx.report()


object_model = build_object_model()
