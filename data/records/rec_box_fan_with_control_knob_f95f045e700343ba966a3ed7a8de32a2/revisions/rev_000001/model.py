from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_box_fan")

    charcoal = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.018, 1.0))
    satin_gray = model.material("satin_gray_plastic", rgba=(0.68, 0.70, 0.70, 1.0))
    pale_gray = model.material("pale_gray_housing", rgba=(0.82, 0.84, 0.83, 1.0))
    chrome = model.material("chrome_wire", rgba=(0.78, 0.80, 0.82, 1.0))
    blue_blade = model.material("translucent_blue_blade", rgba=(0.45, 0.68, 0.86, 0.82))
    black_mark = model.material("black_marking", rgba=(0.02, 0.02, 0.02, 1.0))

    sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.042, 0.060), (0.042, 0.580)],
            [(0.027, 0.060), (0.027, 0.580)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "lower_sleeve",
    )
    collar_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.042, tube=0.008, radial_segments=18, tubular_segments=64).translate(
            0.0,
            0.0,
            0.586,
        ),
        "sleeve_collar",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.245, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=charcoal,
        name="round_base",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_rubber,
        name="base_step",
    )
    base.visual(sleeve_mesh, material=satin_gray, name="lower_sleeve")
    base.visual(collar_mesh, material=satin_gray, name="sleeve_collar")
    base.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(xyz=(0.046, 0.0, 0.545), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="height_lock_screw",
    )

    upper_column = model.part("upper_column")
    # The upper column frame is located at the lower sleeve mouth.  The mast is
    # intentionally longer than the visible exposed column so that it remains
    # captured inside the lower sleeve at full height.
    upper_column.visual(
        Cylinder(radius=0.022, length=0.830),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=satin_gray,
        name="inner_mast",
    )
    upper_column.visual(
        Cylinder(radius=0.035, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
        material=pale_gray,
        name="neck_collar",
    )
    upper_column.visual(
        Box((0.100, 0.115, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=pale_gray,
        name="neck_block",
    )

    head_z = 0.680
    frame_outer_w = 0.720
    frame_outer_h = 0.680
    frame_depth = 0.145
    rail = 0.060
    upper_column.visual(
        Box((frame_outer_w, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, head_z + frame_outer_h / 2.0 - rail / 2.0)),
        material=pale_gray,
        name="top_rail",
    )
    upper_column.visual(
        Box((frame_outer_w, frame_depth, rail)),
        origin=Origin(xyz=(0.0, 0.0, head_z - frame_outer_h / 2.0 + rail / 2.0)),
        material=pale_gray,
        name="bottom_rail",
    )
    upper_column.visual(
        Box((rail, frame_depth, frame_outer_h)),
        origin=Origin(xyz=(-frame_outer_w / 2.0 + rail / 2.0, 0.0, head_z)),
        material=pale_gray,
        name="side_rail_0",
    )
    upper_column.visual(
        Box((rail, frame_depth, frame_outer_h)),
        origin=Origin(xyz=(frame_outer_w / 2.0 - rail / 2.0, 0.0, head_z)),
        material=pale_gray,
        name="side_rail_1",
    )
    upper_column.visual(
        Box((0.120, 0.120, 0.150)),
        origin=Origin(xyz=(0.0, 0.0, head_z - 0.305)),
        material=pale_gray,
        name="yoke_socket",
    )

    # Rear motor pod and struts.  This pod gives the propeller a believable axle
    # support and provides the rear face carrying the timer knob.
    upper_column.visual(
        Cylinder(radius=0.088, length=0.120),
        origin=Origin(xyz=(0.0, 0.035, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_gray,
        name="motor_pod",
    )
    upper_column.visual(
        Cylinder(radius=0.104, length=0.018),
        origin=Origin(xyz=(0.0, 0.100, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pale_gray,
        name="rear_face",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        mid_r = 0.190
        length = 0.305
        x = mid_r * math.cos(angle)
        z = head_z + mid_r * math.sin(angle)
        upper_column.visual(
            Box((length, 0.018, 0.018)),
            origin=Origin(xyz=(x, 0.045, z), rpy=(0.0, -angle, 0.0)),
            material=satin_gray,
            name=f"rear_strut_{index}",
        )

    # The wire grille sits proud of the square housing.  Concentric torus rings,
    # radial wires, and four small tabs tie the guard back into the housing rails.
    grille_y = -0.105
    upper_column.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.070, tube=0.0036, radial_segments=12, tubular_segments=72),
            "front_ring_0",
        ),
        origin=Origin(xyz=(0.0, grille_y, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="front_ring_0",
    )
    upper_column.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.145, tube=0.0036, radial_segments=12, tubular_segments=72),
            "front_ring_1",
        ),
        origin=Origin(xyz=(0.0, grille_y, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="front_ring_1",
    )
    upper_column.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.215, tube=0.0036, radial_segments=12, tubular_segments=72),
            "front_ring_2",
        ),
        origin=Origin(xyz=(0.0, grille_y, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="front_ring_2",
    )
    upper_column.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.285, tube=0.0036, radial_segments=12, tubular_segments=72),
            "front_ring_3",
        ),
        origin=Origin(xyz=(0.0, grille_y, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="front_ring_3",
    )
    upper_column.visual(
        Cylinder(radius=0.039, length=0.012),
        origin=Origin(xyz=(0.0, grille_y, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="guard_hub",
    )
    upper_column.visual(
        Cylinder(radius=0.005, length=0.118),
        origin=Origin(xyz=(0.0, -0.048, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="center_axle",
    )
    for index, angle in enumerate(i * math.tau / 16.0 for i in range(16)):
        inner = 0.036
        outer = 0.300
        mid = 0.5 * (inner + outer)
        length = outer - inner
        x = mid * math.cos(angle)
        z = head_z + mid * math.sin(angle)
        upper_column.visual(
            Box((length, 0.006, 0.006)),
            origin=Origin(xyz=(x, grille_y, z), rpy=(0.0, -angle, 0.0)),
            material=chrome,
            name=f"spoke_{index}",
        )
    upper_column.visual(
        Box((0.032, 0.042, 0.012)),
        origin=Origin(xyz=(-0.292, -0.086, head_z)),
        material=chrome,
        name="grille_tab_0",
    )
    upper_column.visual(
        Box((0.032, 0.042, 0.012)),
        origin=Origin(xyz=(0.292, -0.086, head_z)),
        material=chrome,
        name="grille_tab_1",
    )
    upper_column.visual(
        Box((0.012, 0.042, 0.032)),
        origin=Origin(xyz=(0.0, -0.086, head_z + 0.292)),
        material=chrome,
        name="grille_tab_2",
    )
    upper_column.visual(
        Box((0.012, 0.042, 0.032)),
        origin=Origin(xyz=(0.0, -0.086, head_z - 0.292)),
        material=chrome,
        name="grille_tab_3",
    )

    propeller = model.part("propeller")
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.235,
            0.056,
            5,
            thickness=0.030,
            blade_pitch_deg=32.0,
            blade_sweep_deg=28.0,
            blade=FanRotorBlade(shape="broad", tip_pitch_deg=15.0, camber=0.18, tip_clearance=0.006),
            hub=FanRotorHub(style="spinner", bore_diameter=0.014),
        ),
        "five_blade_rotor",
    )
    propeller.visual(rotor_mesh, material=blue_blade, name="five_blade_rotor")

    timer_knob = model.part("timer_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.033,
            body_style="skirted",
            top_diameter=0.052,
            skirt=KnobSkirt(0.082, 0.008, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "timer_knob",
    )
    timer_knob.visual(knob_mesh, material=charcoal, name="timer_knob")
    timer_knob.visual(
        Box((0.003, 0.0015, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.034), rpy=(0.0, 0.0, 0.0)),
        material=black_mark,
        name="timer_mark",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.250),
    )
    model.articulation(
        "column_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=upper_column,
        child=propeller,
        origin=Origin(xyz=(0.0, -0.056, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )
    model.articulation(
        "column_to_timer",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=timer_knob,
        origin=Origin(xyz=(0.0, 0.109, head_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.5, lower=0.0, upper=math.radians(330.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("upper_column")
    propeller = object_model.get_part("propeller")
    timer_knob = object_model.get_part("timer_knob")
    height_slide = object_model.get_articulation("base_to_column")
    prop_joint = object_model.get_articulation("column_to_propeller")
    timer_joint = object_model.get_articulation("column_to_timer")

    ctx.check(
        "three primary mechanisms are articulated",
        height_slide.articulation_type == ArticulationType.PRISMATIC
        and prop_joint.articulation_type == ArticulationType.CONTINUOUS
        and timer_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"types={(height_slide.articulation_type, prop_joint.articulation_type, timer_joint.articulation_type)}"
        ),
    )

    ctx.expect_within(
        column,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="lower_sleeve",
        margin=0.001,
        name="mast is centered in lower sleeve",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="lower_sleeve",
        min_overlap=0.25,
        name="collapsed mast remains captured",
    )

    rest_position = ctx.part_world_position(column)
    with ctx.pose({height_slide: 0.250}):
        ctx.expect_within(
            column,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="lower_sleeve",
            margin=0.001,
            name="raised mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="lower_sleeve",
            min_overlap=0.075,
            name="raised mast retains insertion",
        )
        raised_position = ctx.part_world_position(column)
    ctx.check(
        "height joint raises fan head",
        rest_position is not None and raised_position is not None and raised_position[2] > rest_position[2] + 0.20,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    ctx.expect_gap(
        propeller,
        column,
        axis="y",
        positive_elem="five_blade_rotor",
        negative_elem="guard_hub",
        min_gap=0.020,
        max_gap=0.090,
        name="propeller sits behind front grille",
    )
    ctx.expect_gap(
        column,
        propeller,
        axis="y",
        positive_elem="motor_pod",
        negative_elem="five_blade_rotor",
        min_gap=0.004,
        max_gap=0.060,
        name="propeller clears rear motor pod",
    )
    ctx.expect_overlap(
        propeller,
        column,
        axes="xz",
        elem_a="five_blade_rotor",
        elem_b="front_ring_3",
        min_overlap=0.20,
        name="rotor is contained by guard footprint",
    )
    ctx.expect_contact(
        timer_knob,
        column,
        elem_a="timer_knob",
        elem_b="rear_face",
        contact_tol=0.003,
        name="timer knob seats on rear face",
    )

    return ctx.report()


object_model = build_object_model()
