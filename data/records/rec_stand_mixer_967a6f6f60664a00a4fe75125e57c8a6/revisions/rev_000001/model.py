from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glossy_enamel_stand_mixer")

    enamel = model.material("glossy_red_enamel", rgba=(0.72, 0.03, 0.025, 1.0))
    enamel_dark = model.material("dark_red_shadow", rgba=(0.38, 0.012, 0.012, 1.0))
    steel = model.material("bright_polished_steel", rgba=(0.86, 0.88, 0.86, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    cream = model.material("cream_index_marks", rgba=(0.94, 0.89, 0.76, 1.0))

    # Root base: a shallow rounded enamel plinth with rear pedestal, slide rails,
    # and the fixed housings for the user controls.
    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.42, 0.56, 0.060, corner_segments=12),
                0.075,
                cap=True,
                center=True,
            ),
            "rounded_base_slab",
        ),
        origin=Origin(xyz=(0.0, 0.055, 0.0375)),
        material=enamel,
        name="rounded_base_slab",
    )
    base.visual(
        mesh_from_geometry(
            superellipse_side_loft(
                [
                    (-0.235, 0.070, 0.330, 0.160),
                    (-0.185, 0.070, 0.382, 0.205),
                    (-0.120, 0.070, 0.360, 0.175),
                    (-0.075, 0.070, 0.285, 0.125),
                ],
                exponents=3.0,
                segments=64,
            ),
            "rear_pedestal",
        ),
        material=enamel,
        name="rear_pedestal",
    )
    base.visual(
        Box((0.026, 0.330, 0.012)),
        origin=Origin(xyz=(-0.092, 0.070, 0.081)),
        material=steel,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.026, 0.330, 0.012)),
        origin=Origin(xyz=(0.092, 0.070, 0.081)),
        material=steel,
        name="slide_rail_1",
    )
    base.visual(
        Box((0.065, 0.042, 0.012)),
        origin=Origin(xyz=(-0.190, -0.035, 0.081)),
        material=enamel_dark,
        name="lock_socket",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.094, -0.130, 0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel_dark,
        name="dial_boss",
    )
    # Hinge yoke cheeks flank the motor-head barrel without intersecting it.
    for x in (-0.150, 0.150):
        base.visual(
            Box((0.034, 0.052, 0.386)),
            origin=Origin(xyz=(x, -0.185, 0.263)),
            material=enamel,
            name=f"hinge_post_{0 if x < 0 else 1}",
        )
        base.visual(
            Box((0.034, 0.052, 0.098)),
            origin=Origin(xyz=(x, -0.185, 0.505)),
            material=enamel,
            name=f"hinge_cheek_{0 if x < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.017, length=0.038),
            origin=Origin(
                xyz=(x + (0.024 if x > 0 else -0.024), -0.185, 0.505),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"hinge_cap_{0 if x < 0 else 1}",
        )
    for x in (-0.165, 0.165):
        for y in (-0.150, 0.250):
            base.visual(
                Cylinder(radius=0.026, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=dark,
                name=f"foot_{'n' if y < 0 else 'f'}_{0 if x < 0 else 1}",
            )

    # Sliding bowl and shallow carriage.  The child frame sits on the rail top at
    # the centered/rest bowl position; positive travel pulls the bowl forward.
    bowl = model.part("bowl")
    bowl.visual(
        Box((0.270, 0.225, 0.020)),
        origin=Origin(xyz=(0.0, 0.000, 0.010)),
        material=enamel,
        name="carriage_plate",
    )
    bowl.visual(
        Cylinder(radius=0.070, length=0.036),
        origin=Origin(xyz=(0.0, 0.000, 0.038)),
        material=steel,
        name="bowl_foot",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.055, 0.052),
            (0.075, 0.062),
            (0.118, 0.125),
            (0.154, 0.215),
            (0.168, 0.266),
        ],
        inner_profile=[
            (0.035, 0.066),
            (0.060, 0.079),
            (0.102, 0.135),
            (0.139, 0.214),
            (0.153, 0.251),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "polished_bowl_shell"),
        material=steel,
        name="bowl_shell",
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.130, 0.087)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.080),
    )

    # Tilt head: the part frame is the rear hinge axis.  The lofted sections make
    # a bulbous teardrop motor shell that narrows into the front attachment nose.
    head = model.part("head")
    head_shell = superellipse_side_loft(
        [
            (-0.040, -0.034, 0.112, 0.165),
            (0.035, -0.085, 0.146, 0.250),
            (0.155, -0.115, 0.145, 0.280),
            (0.270, -0.100, 0.116, 0.215),
            (0.375, -0.068, 0.074, 0.128),
            (0.430, -0.040, 0.046, 0.070),
        ],
        exponents=2.35,
        segments=80,
    )
    head.visual(
        mesh_from_geometry(head_shell, "teardrop_motor_head"),
        material=enamel,
        name="motor_head_shell",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.390, -0.005), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nose_ring",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.092),
        origin=Origin(xyz=(0.0, 0.310, -0.084)),
        material=steel,
        name="attachment_hub",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.266),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.205, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.238, 0.126)),
        material=cream,
        name="top_highlight",
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.185, 0.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=0.72),
    )

    # Balloon whisk mounted under the nose.  Each wire is a separate curved tube
    # visibly captured by the top collar and bottom bead.
    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=steel,
        name="spindle",
    )
    whisk.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=steel,
        name="top_collar",
    )
    whisk.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.207)),
        material=steel,
        name="bottom_bead",
    )
    for i in range(8):
        a = 2.0 * math.pi * i / 8.0
        ca = math.cos(a)
        sa = math.sin(a)
        wire = tube_from_spline_points(
            [
                (0.019 * ca, 0.019 * sa, -0.052),
                (0.055 * ca, 0.055 * sa, -0.095),
                (0.082 * ca, 0.082 * sa, -0.150),
                (0.052 * ca, 0.052 * sa, -0.192),
                (0.011 * ca, 0.011 * sa, -0.209),
            ],
            radius=0.0028,
            samples_per_segment=16,
            radial_segments=12,
            cap_ends=True,
        )
        whisk.visual(
            mesh_from_geometry(wire, f"whisk_wire_{i}"),
            material=steel,
            name=f"wire_{i}",
        )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.0, 0.310, -0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=80.0),
    )

    # Base-mounted speed dial.  The joint frame is rotated so the knob's local
    # Z-axis protrudes from the side face of the base pedestal.
    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="dial_skirt",
    )
    speed_dial.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="dial_cap",
    )
    speed_dial.visual(
        Box((0.007, 0.034, 0.003)),
        origin=Origin(xyz=(0.0, 0.010, 0.0295), rpy=(0.0, 0.0, math.radians(-28.0))),
        material=cream,
        name="dial_pointer",
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(0.101, -0.130, 0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-2.35, upper=2.35),
    )

    # Base-mounted lock button with a short depressing prismatic stroke.
    lock_button = model.part("lock_button")
    lock_button.visual(
        Box((0.052, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark,
        name="button_cap",
    )
    model.articulation(
        "base_to_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_button,
        origin=Origin(xyz=(-0.190, -0.035, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=-0.008, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_dial = object_model.get_part("speed_dial")
    lock_button = object_model.get_part("lock_button")

    bowl_slide = object_model.get_articulation("base_to_bowl")
    head_tilt = object_model.get_articulation("base_to_head")
    whisk_spin = object_model.get_articulation("head_to_whisk")
    dial_turn = object_model.get_articulation("base_to_speed_dial")
    lock_press = object_model.get_articulation("base_to_lock_button")

    ctx.check(
        "canonical articulation set",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and whisk_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_turn.articulation_type == ArticulationType.REVOLUTE
        and lock_press.articulation_type == ArticulationType.PRISMATIC,
        details="Expected bowl slide, head hinge, whisk spin, speed dial turn, and lock button press.",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="slide_rail_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="bowl carriage rides on slide rail",
    )
    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="bottom_bead",
        outer_elem="bowl_shell",
        margin=0.020,
        name="whisk hangs inside bowl footprint",
    )

    bowl_rest = ctx.part_world_position(bowl)
    with ctx.pose({bowl_slide: 0.080}):
        bowl_forward = ctx.part_world_position(bowl)
        ctx.expect_overlap(
            bowl,
            base,
            axes="y",
            elem_a="carriage_plate",
            elem_b="slide_rail_0",
            min_overlap=0.100,
            name="extended bowl remains captured on rail",
        )
    ctx.check(
        "bowl slide moves forward",
        bowl_rest is not None and bowl_forward is not None and bowl_forward[1] > bowl_rest[1] + 0.070,
        details=f"rest={bowl_rest}, extended={bowl_forward}",
    )

    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({head_tilt: 0.72}):
        whisk_tilted = ctx.part_world_position(whisk)
    ctx.check(
        "head tilt lifts nose and whisk",
        whisk_rest is not None and whisk_tilted is not None and whisk_tilted[2] > whisk_rest[2] + 0.18,
        details=f"rest={whisk_rest}, tilted={whisk_tilted}",
    )

    button_rest = ctx.part_world_position(lock_button)
    with ctx.pose({lock_press: -0.008}):
        button_pressed = ctx.part_world_position(lock_button)
    ctx.check(
        "lock button depresses downward",
        button_rest is not None and button_pressed is not None and button_pressed[2] < button_rest[2] - 0.006,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    ctx.expect_contact(
        speed_dial,
        base,
        elem_a="dial_cap",
        elem_b="dial_boss",
        contact_tol=0.002,
        name="speed dial is seated on base boss",
    )

    return ctx.report()


object_model = build_object_model()
