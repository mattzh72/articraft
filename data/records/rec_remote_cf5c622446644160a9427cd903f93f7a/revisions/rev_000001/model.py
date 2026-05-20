from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broadcast_fader_controller")

    housing_color = model.material("housing_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    deck_color = model.material("deck_black", rgba=(0.09, 0.10, 0.11, 1.0))
    slot_color = model.material("slot_dark", rgba=(0.05, 0.05, 0.06, 1.0))
    trim_color = model.material("trim_grey", rgba=(0.34, 0.36, 0.38, 1.0))
    cap_color = model.material("fader_cap_light", rgba=(0.84, 0.86, 0.88, 1.0))
    knob_color = model.material("encoder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    marker_color = model.material("marker_white", rgba=(0.92, 0.92, 0.90, 1.0))

    housing_width = 0.285
    housing_depth = 0.185
    housing_height = 0.034
    wall_thickness = 0.004
    floor_thickness = 0.003
    top_thickness = 0.0025

    front_strip_depth = 0.024
    rear_strip_depth = 0.048
    slot_zone_depth = housing_depth - (2.0 * wall_thickness) - front_strip_depth - rear_strip_depth
    slot_opening_width = 0.012
    slot_opening_length = 0.092
    channel_floor_width = 0.010
    channel_floor_length = 0.118
    channel_floor_height = 0.003
    channel_floor_center_z = 0.017
    slot_support_length = 0.010
    slot_support_height = 0.015
    slot_support_center_z = 0.0255
    slot_center_y = (
        -housing_depth * 0.5
        + wall_thickness
        + front_strip_depth
        + (slot_zone_depth * 0.5)
    )
    fader_x_positions = (-0.090, -0.030, 0.030, 0.090)
    encoder_y = housing_depth * 0.5 - wall_thickness - 0.022

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=housing_color,
        name="bottom_plate",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, housing_height)),
        origin=Origin(xyz=(-housing_width * 0.5 + wall_thickness * 0.5, 0.0, housing_height * 0.5)),
        material=housing_color,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, housing_height)),
        origin=Origin(xyz=(housing_width * 0.5 - wall_thickness * 0.5, 0.0, housing_height * 0.5)),
        material=housing_color,
        name="right_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, wall_thickness, housing_height)),
        origin=Origin(
            xyz=(0.0, -housing_depth * 0.5 + wall_thickness * 0.5, housing_height * 0.5)
        ),
        material=housing_color,
        name="front_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, wall_thickness, housing_height)),
        origin=Origin(
            xyz=(0.0, housing_depth * 0.5 - wall_thickness * 0.5, housing_height * 0.5)
        ),
        material=housing_color,
        name="rear_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, front_strip_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -housing_depth * 0.5
                + wall_thickness
                + front_strip_depth * 0.5,
                housing_height - top_thickness * 0.5,
            )
        ),
        material=deck_color,
        name="front_deck",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, rear_strip_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5
                - wall_thickness
                - rear_strip_depth * 0.5,
                housing_height - top_thickness * 0.5,
            )
        ),
        material=deck_color,
        name="rear_deck",
    )

    slot_strip_edges = (
        -housing_width * 0.5 + wall_thickness,
        (fader_x_positions[0] + fader_x_positions[1]) * 0.5,
        (fader_x_positions[1] + fader_x_positions[2]) * 0.5,
        (fader_x_positions[2] + fader_x_positions[3]) * 0.5,
        housing_width * 0.5 - wall_thickness,
    )
    strip_centers_and_widths = []
    previous_right = slot_strip_edges[0]
    for index, channel_x in enumerate(fader_x_positions):
        slot_left = channel_x - slot_opening_width * 0.5
        strip_centers_and_widths.append(((previous_right + slot_left) * 0.5, slot_left - previous_right))
        previous_right = channel_x + slot_opening_width * 0.5
        if index == len(fader_x_positions) - 1:
            final_left = previous_right
            final_right = slot_strip_edges[-1]
            strip_centers_and_widths.append(
                ((final_left + final_right) * 0.5, final_right - final_left)
            )

    for strip_index, (strip_center_x, strip_width) in enumerate(strip_centers_and_widths):
        housing.visual(
            Box((strip_width, slot_zone_depth, top_thickness)),
            origin=Origin(
                xyz=(
                    strip_center_x,
                    slot_center_y,
                    housing_height - top_thickness * 0.5,
                )
            ),
            material=deck_color,
            name=f"slot_bridge_{strip_index + 1}",
        )

    for index, channel_x in enumerate(fader_x_positions, start=1):
        housing.visual(
            Box((channel_floor_width, channel_floor_length, channel_floor_height)),
            origin=Origin(xyz=(channel_x, slot_center_y, channel_floor_center_z)),
            material=slot_color,
            name=f"slot_floor_{index}",
        )
        housing.visual(
            Box((channel_floor_width, slot_support_length, slot_support_height)),
            origin=Origin(
                xyz=(
                    channel_x,
                    slot_center_y - 0.051,
                    slot_support_center_z,
                )
            ),
            material=slot_color,
            name=f"slot_front_web_{index}",
        )
        housing.visual(
            Box((channel_floor_width, slot_support_length, slot_support_height)),
            origin=Origin(
                xyz=(
                    channel_x,
                    slot_center_y + 0.051,
                    slot_support_center_z,
                )
            ),
            material=slot_color,
            name=f"slot_rear_web_{index}",
        )
        housing.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(channel_x, encoder_y, housing_height + 0.0015)),
            material=trim_color,
            name=f"encoder_plinth_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    fader_travel = 0.040
    fader_cap_size = (0.016, 0.024, 0.011)
    fader_stem_size = (0.0055, 0.011, 0.012)

    for index, channel_x in enumerate(fader_x_positions, start=1):
        fader = model.part(f"fader_{index}")
        fader.visual(
            Box(fader_cap_size),
            origin=Origin(xyz=(0.0, 0.0, fader_cap_size[2] * 0.5)),
            material=cap_color,
            name="cap",
        )
        fader.visual(
            Box((0.010, 0.018, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=marker_color,
            name="grip_ridge",
        )
        fader.visual(
            Box(fader_stem_size),
            origin=Origin(xyz=(0.0, 0.0, -fader_stem_size[2] * 0.5)),
            material=trim_color,
            name="stem",
        )
        fader.inertial = Inertial.from_geometry(
            Box((0.018, 0.026, 0.023)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        )
        model.articulation(
            f"housing_to_fader_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(channel_x, slot_center_y, housing_height)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.20,
                lower=-fader_travel,
                upper=fader_travel,
            ),
        )

    for index, channel_x in enumerate(fader_x_positions, start=1):
        knob = model.part(f"encoder_{index}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=trim_color,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=knob_color,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.0095, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
            material=knob_color,
            name="top_crown",
        )
        knob.visual(
            Box((0.002, 0.007, 0.001)),
            origin=Origin(xyz=(0.006, 0.0, 0.0245)),
            material=marker_color,
            name="index_marker",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.012, length=0.024),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
        )
        model.articulation(
            f"housing_to_encoder_{index}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(channel_x, encoder_y, housing_height + 0.003)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=15.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    faders = [object_model.get_part(f"fader_{index}") for index in range(1, 5)]
    encoders = [object_model.get_part(f"encoder_{index}") for index in range(1, 5)]
    fader_joints = [
        object_model.get_articulation(f"housing_to_fader_{index}") for index in range(1, 5)
    ]
    encoder_joints = [
        object_model.get_articulation(f"housing_to_encoder_{index}") for index in range(1, 5)
    ]

    ctx.check("housing part exists", housing is not None)
    for index, (fader, joint) in enumerate(zip(faders, fader_joints, strict=False), start=1):
        limits = joint.motion_limits
        ctx.check(
            f"fader {index} uses prismatic slot travel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"joint={joint.name}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_within(
            fader,
            housing,
            axes="xy",
            inner_elem="stem",
            outer_elem=f"slot_floor_{index}",
            margin=0.001,
            name=f"fader {index} stem stays within its slot floor at rest",
        )
        lower_limit = limits.lower if limits is not None and limits.lower is not None else -0.04
        upper_limit = limits.upper if limits is not None and limits.upper is not None else 0.04
        with ctx.pose({joint: lower_limit}):
            ctx.expect_within(
                fader,
                housing,
                axes="xy",
                inner_elem="stem",
                outer_elem=f"slot_floor_{index}",
                margin=0.001,
                name=f"fader {index} stem stays within slot at lower travel",
            )
            lower_pos = ctx.part_world_position(fader)
        with ctx.pose({joint: upper_limit}):
            ctx.expect_within(
                fader,
                housing,
                axes="xy",
                inner_elem="stem",
                outer_elem=f"slot_floor_{index}",
                margin=0.001,
                name=f"fader {index} stem stays within slot at upper travel",
            )
            upper_pos = ctx.part_world_position(fader)
        ctx.check(
            f"fader {index} translates along controller depth",
            lower_pos is not None
            and upper_pos is not None
            and upper_pos[1] > lower_pos[1] + 0.06
            and abs(upper_pos[0] - lower_pos[0]) < 1e-6
            and abs(upper_pos[2] - lower_pos[2]) < 1e-6,
            details=f"lower={lower_pos}, upper={upper_pos}",
        )

    for index, (encoder, joint) in enumerate(
        zip(encoders, encoder_joints, strict=False), start=1
    ):
        limits = joint.motion_limits
        ctx.check(
            f"encoder {index} is endless",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"joint={joint.name}, axis={joint.axis}, limits={limits}",
        )
        rest_pos = ctx.part_world_position(encoder)
        with ctx.pose({joint: 2.4}):
            spun_pos = ctx.part_world_position(encoder)
        ctx.check(
            f"encoder {index} spins in place",
            rest_pos is not None
            and spun_pos is not None
            and abs(rest_pos[0] - spun_pos[0]) < 1e-6
            and abs(rest_pos[1] - spun_pos[1]) < 1e-6
            and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
