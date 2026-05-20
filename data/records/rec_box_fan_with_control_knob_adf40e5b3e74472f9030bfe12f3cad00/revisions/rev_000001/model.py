from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barn_ventilation_fan")

    galvanized = Material("dull_galvanized_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_metal = Material("dark_motor_metal", rgba=(0.08, 0.09, 0.09, 1.0))
    black = Material("matte_black", rgba=(0.01, 0.012, 0.012, 1.0))
    blue = Material("barn_motor_blue", rgba=(0.12, 0.20, 0.28, 1.0))
    label_white = Material("stenciled_white", rgba=(0.88, 0.88, 0.82, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.94, 0.94),
                (1.20, 1.20),
                0.18,
                opening_shape="circle",
                outer_shape="rect",
                outer_corner_radius=0.018,
            ),
            "square_fan_housing",
        ),
        material=galvanized,
        name="square_housing",
    )

    # Rear motor and four cross braces keep the fan shaft visibly supported inside
    # the square housing while leaving the round air path open.
    housing.visual(
        Cylinder(radius=0.115, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=blue,
        name="rear_motor",
    )
    housing.visual(
        Box((1.02, 0.035, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=dark_metal,
        name="horizontal_motor_brace",
    )
    housing.visual(
        Box((0.035, 1.02, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=dark_metal,
        name="vertical_motor_brace",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=dark_metal,
        name="shaft_stub",
    )

    # Shutter hinge headers protrude from the outside face and provide a real
    # support line for each louver flap.
    shutter_width = 0.88
    shutter_height = 0.195
    shutter_gap = 0.018
    hinge_z = 0.126
    top_hinge_y = 0.430
    hinge_ys = [top_hinge_y - i * (shutter_height + shutter_gap) for i in range(4)]
    for i, hinge_y in enumerate(hinge_ys):
        housing.visual(
            Box((0.94, 0.018, 0.050)),
            origin=Origin(xyz=(0.0, hinge_y + 0.009, 0.114)),
            material=dark_metal,
            name=f"shutter_header_{i}",
        )
        housing.visual(
            Cylinder(radius=0.006, length=0.92),
            origin=Origin(xyz=(0.0, hinge_y + 0.014, 0.136), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"hinge_rod_{i}",
        )

    # A raised control plate on the lower corner carries the continuous speed
    # dial without interfering with the shutter travel.
    housing.visual(
        Box((0.225, 0.155, 0.050)),
        origin=Origin(xyz=(0.425, -0.495, 0.115)),
        material=dark_metal,
        name="speed_plate",
    )
    housing.visual(
        Box((0.090, 0.010, 0.004)),
        origin=Origin(xyz=(0.425, -0.565, 0.142)),
        material=label_white,
        name="speed_label",
    )
    for j, x_offset in enumerate((-0.055, -0.028, 0.0, 0.028, 0.055)):
        housing.visual(
            Box((0.006, 0.020 + 0.006 * (j % 2), 0.004)),
            origin=Origin(xyz=(0.425 + x_offset, -0.515, 0.142)),
            material=label_white,
            name=f"speed_tick_{j}",
        )

    rotor = model.part("fan_blade")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.365,
                0.075,
                5,
                thickness=0.048,
                blade_pitch_deg=34.0,
                blade_sweep_deg=27.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=17.0, camber=0.12, tip_clearance=0.010),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.018, rear_collar_radius=0.052),
            ),
            "barn_fan_blade",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=black,
        name="five_blade_rotor",
    )
    model.articulation(
        "housing_to_fan_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )

    for i, hinge_y in enumerate(hinge_ys):
        shutter = model.part(f"shutter_{i}")
        shutter.visual(
            Box((shutter_width, shutter_height, 0.018)),
            origin=Origin(xyz=(0.0, -shutter_height / 2.0, 0.0)),
            material=galvanized,
            name="flap_panel",
        )
        shutter.visual(
            Box((shutter_width, 0.020, 0.026)),
            origin=Origin(xyz=(0.0, -0.010, 0.004)),
            material=galvanized,
            name="hinge_leaf",
        )
        shutter.visual(
            Cylinder(radius=0.007, length=shutter_width),
            origin=Origin(xyz=(0.0, -0.018, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name="rolled_edge",
        )
        shutter.visual(
            Box((0.80, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.072, 0.011)),
            material=galvanized,
            name="upper_stiffener",
        )
        shutter.visual(
            Box((0.80, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.145, 0.011)),
            material=galvanized,
            name="lower_stiffener",
        )
        model.articulation(
            f"housing_to_shutter_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=shutter,
            origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.05),
        )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.075,
                0.040,
                body_style="faceted",
                base_diameter=0.080,
                top_diameter=0.060,
                edge_radius=0.001,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0012, width=0.003),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "speed_dial_knob",
        ),
        material=black,
        name="dial_cap",
    )
    model.articulation(
        "housing_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_dial,
        origin=Origin(xyz=(0.425, -0.495, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    fan_blade = object_model.get_part("fan_blade")
    speed_dial = object_model.get_part("speed_dial")

    fan_joint = object_model.get_articulation("housing_to_fan_blade")
    dial_joint = object_model.get_articulation("housing_to_speed_dial")
    ctx.allow_overlap(
        housing,
        fan_blade,
        elem_a="shaft_stub",
        elem_b="five_blade_rotor",
        reason="The fixed drive shaft is intentionally captured inside the rotor hub so the continuously spinning blade is physically supported.",
    )
    ctx.check(
        "fan blade spins continuously",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={fan_joint.articulation_type}",
    )
    ctx.check(
        "speed dial rotates continuously",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={dial_joint.articulation_type}",
    )
    ctx.expect_within(
        fan_blade,
        housing,
        axes="xy",
        inner_elem="five_blade_rotor",
        outer_elem="square_housing",
        margin=0.002,
        name="rotor fits inside square housing opening",
    )
    ctx.expect_within(
        housing,
        fan_blade,
        axes="xy",
        inner_elem="shaft_stub",
        outer_elem="five_blade_rotor",
        margin=0.0,
        name="drive shaft is centered in rotor hub",
    )
    ctx.expect_overlap(
        housing,
        fan_blade,
        axes="z",
        elem_a="shaft_stub",
        elem_b="five_blade_rotor",
        min_overlap=0.015,
        name="drive shaft remains inserted in hub",
    )
    ctx.expect_contact(
        speed_dial,
        housing,
        elem_a="dial_cap",
        elem_b="speed_plate",
        contact_tol=0.002,
        name="speed dial is seated on control plate",
    )

    for i in range(4):
        shutter = object_model.get_part(f"shutter_{i}")
        hinge = object_model.get_articulation(f"housing_to_shutter_{i}")
        ctx.check(
            f"shutter {i} has revolute hinge",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and hinge.motion_limits is not None
            and hinge.motion_limits.lower == 0.0
            and hinge.motion_limits.upper is not None
            and hinge.motion_limits.upper > 0.8,
            details=f"type={hinge.articulation_type}, limits={hinge.motion_limits}",
        )
        ctx.expect_contact(
            shutter,
            housing,
            elem_a="hinge_leaf",
            elem_b=f"shutter_header_{i}",
            contact_tol=0.002,
            name=f"shutter {i} hinge edge is mounted",
        )
        closed_aabb = ctx.part_world_aabb(shutter)
        with ctx.pose({hinge: 0.80}):
            opened_aabb = ctx.part_world_aabb(shutter)
        ctx.check(
            f"shutter {i} opens outward",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.10,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
