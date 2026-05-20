from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_tower_fan")

    graphite = model.material("graphite", rgba=(0.15, 0.17, 0.19, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_silver = model.material("dark_silver", rgba=(0.47, 0.49, 0.52, 1.0))
    smoked = model.material("smoked", rgba=(0.20, 0.23, 0.25, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")

    base_radius = 0.165
    base_height = 0.030
    trim_radius = 0.120
    trim_height = 0.014
    stem_radius = 0.034
    stem_height = 0.080

    housing_width = 0.156
    housing_depth = 0.122
    housing_height = 0.745
    housing_bottom_z = base_height + trim_height + stem_height
    housing_center_z = housing_bottom_z + housing_height * 0.5
    top_cap_height = 0.055
    top_cap_center_z = housing_bottom_z + housing_height + top_cap_height * 0.5

    side_wall_thickness = 0.012
    back_wall_thickness = 0.008
    front_bridge_depth = 0.014
    front_bridge_height = 0.100
    opening_width = housing_width - 2.0 * side_wall_thickness
    opening_bottom_z = housing_bottom_z + front_bridge_height
    opening_top_z = housing_bottom_z + housing_height - front_bridge_height
    opening_height = opening_top_z - opening_bottom_z

    body.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=graphite,
        name="pedestal_base",
    )
    body.visual(
        Cylinder(radius=trim_radius, length=trim_height),
        origin=Origin(xyz=(0.0, 0.0, base_height + trim_height * 0.5)),
        material=dark_silver,
        name="pedestal_trim",
    )
    body.visual(
        Cylinder(radius=stem_radius, length=stem_height),
        origin=Origin(xyz=(0.0, 0.0, base_height + trim_height + stem_height * 0.5)),
        material=graphite,
        name="pedestal_stem",
    )
    body.visual(
        Box((housing_width, housing_depth, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, housing_bottom_z + 0.015)),
        material=graphite,
        name="lower_housing_collar",
    )

    side_wall_y = 0.0
    side_wall_z = housing_center_z
    side_wall_x = housing_width * 0.5 - side_wall_thickness * 0.5
    body.visual(
        Box((side_wall_thickness, housing_depth, housing_height)),
        origin=Origin(xyz=(-side_wall_x, side_wall_y, side_wall_z)),
        material=graphite,
        name="left_wall",
    )
    body.visual(
        Box((side_wall_thickness, housing_depth, housing_height)),
        origin=Origin(xyz=(side_wall_x, side_wall_y, side_wall_z)),
        material=graphite,
        name="right_wall",
    )
    body.visual(
        Box((opening_width, back_wall_thickness, housing_height)),
        origin=Origin(
            xyz=(0.0, -housing_depth * 0.5 + back_wall_thickness * 0.5, housing_center_z)
        ),
        material=graphite,
        name="rear_wall",
    )
    body.visual(
        Box((opening_width, front_bridge_depth, front_bridge_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - front_bridge_depth * 0.5,
                housing_bottom_z + front_bridge_height * 0.5,
            )
        ),
        material=graphite,
        name="lower_front_bridge",
    )
    body.visual(
        Box((opening_width, front_bridge_depth, front_bridge_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - front_bridge_depth * 0.5,
                housing_bottom_z + housing_height - front_bridge_height * 0.5,
            )
        ),
        material=graphite,
        name="upper_front_bridge",
    )
    body.visual(
        Box((housing_width, housing_depth, top_cap_height)),
        origin=Origin(xyz=(0.0, 0.0, top_cap_center_z)),
        material=graphite,
        name="top_cap",
    )
    body.visual(
        Box((0.110, 0.074, 0.006)),
        origin=Origin(
            xyz=(0.0, 0.0, housing_bottom_z + housing_height + top_cap_height + 0.003)
        ),
        material=dark_silver,
        name="control_deck",
    )
    body.visual(
        Box((0.030, 0.010, 0.003)),
        origin=Origin(
            xyz=(0.0, -0.024, housing_bottom_z + housing_height + top_cap_height + 0.0075)
        ),
        material=smoked,
        name="status_window",
    )

    grille_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (opening_width, opening_height),
            0.004,
            slot_size=(opening_height * 0.78, 0.006),
            pitch=(0.014, opening_height + 0.05),
            frame=0.009,
            corner_radius=0.006,
            slot_angle_deg=89.0,
            stagger=False,
            center=True,
        ),
        "tower_fan_front_grille",
    )
    body.visual(
        grille_mesh,
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - 0.002,
                opening_bottom_z + opening_height * 0.5,
            ),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=satin_black,
        name="front_grille",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.061),
        origin=Origin(xyz=(0.0, 0.0, housing_bottom_z + 0.0605)),
        material=satin_black,
        name="lower_motor_housing",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, housing_bottom_z + housing_height - 0.017)),
        material=satin_black,
        name="upper_bearing_housing",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.330, 0.330, housing_bottom_z + housing_height + top_cap_height + 0.040)),
        mass=5.8,
        origin=Origin(
            xyz=(0.0, 0.0, (housing_bottom_z + housing_height + top_cap_height + 0.040) * 0.5)
        ),
    )

    blower_height = 0.620
    blower_outer_radius = 0.044
    blower_mesh = mesh_from_geometry(
        BlowerWheelGeometry(
            blower_outer_radius,
            0.022,
            blower_height,
            30,
            blade_thickness=0.003,
            blade_sweep_deg=28.0,
            backplate=True,
            shroud=True,
            center=True,
        ),
        "tower_fan_blower_wheel",
    )
    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        blower_mesh,
        material=wheel_black,
        name="wheel_shell",
    )
    blower_wheel.visual(
        Cylinder(radius=0.006, length=blower_height - 0.020),
        origin=Origin(),
        material=dark_silver,
        name="axle",
    )
    blower_wheel.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -blower_height * 0.5 + 0.010)),
        material=dark_silver,
        name="lower_end_cap",
    )
    blower_wheel.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, blower_height * 0.5 - 0.010)),
        material=dark_silver,
        name="upper_end_cap",
    )
    blower_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=blower_outer_radius, length=blower_height + 0.040),
        mass=0.75,
        origin=Origin(),
    )

    dial_positions = {
        "left_dial": (-0.030, 0.010, housing_bottom_z + housing_height + top_cap_height + 0.006),
        "right_dial": (0.030, 0.010, housing_bottom_z + housing_height + top_cap_height + 0.006),
    }
    for dial_name in dial_positions:
        dial = model.part(dial_name)
        dial.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=knob_black,
            name="knob_body",
        )
        dial.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=dark_silver,
            name="knob_cap",
        )
        dial.visual(
            Box((0.004, 0.011, 0.004)),
            origin=Origin(xyz=(0.0, 0.012, 0.021)),
            material=smoked,
            name="indicator",
        )
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=0.018, length=0.025),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        )

    model.articulation(
        "body_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, 0.0, housing_bottom_z + 0.401)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=22.0),
    )
    for dial_name, dial_xyz in dial_positions.items():
        model.articulation(
            f"body_to_{dial_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=model.get_part(dial_name),
            origin=Origin(xyz=dial_xyz),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=5.0,
                lower=-2.5,
                upper=2.5,
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    blower_wheel = object_model.get_part("blower_wheel")
    blower_joint = object_model.get_articulation("body_to_blower_wheel")
    left_dial = object_model.get_part("left_dial")
    right_dial = object_model.get_part("right_dial")
    left_joint = object_model.get_articulation("body_to_left_dial")
    right_joint = object_model.get_articulation("body_to_right_dial")

    for dial_part, label in ((left_dial, "left"), (right_dial, "right")):
        ctx.expect_gap(
            dial_part,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            negative_elem="control_deck",
            name=f"{label} dial sits on control deck",
        )
        ctx.expect_within(
            dial_part,
            body,
            axes="xy",
            margin=0.0,
            outer_elem="control_deck",
            name=f"{label} dial stays within control deck footprint",
        )

    blower_rest = ctx.part_world_position(blower_wheel)
    with ctx.pose({blower_joint: 1.3}):
        blower_spun = ctx.part_world_position(blower_wheel)

    ctx.check(
        "blower wheel uses continuous vertical spin",
        blower_joint.motion_limits is not None
        and blower_joint.motion_limits.lower is None
        and blower_joint.motion_limits.upper is None
        and tuple(round(value, 6) for value in blower_joint.axis) == (0.0, 0.0, 1.0)
        and blower_rest is not None
        and blower_spun is not None
        and max(abs(a - b) for a, b in zip(blower_rest, blower_spun)) < 1e-6,
        details=f"axis={blower_joint.axis}, rest={blower_rest}, spun={blower_spun}",
    )

    for joint_name, joint in (("left", left_joint), ("right", right_joint)):
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} dial has bounded rotary control travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}, limits={limits}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
