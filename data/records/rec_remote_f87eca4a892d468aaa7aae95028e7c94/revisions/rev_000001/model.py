from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="dj_midi_pad_controller")

    model.material("housing_shell", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("panel_surface", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("pad_body", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("pad_coral", rgba=(0.82, 0.38, 0.33, 1.0))
    model.material("pad_amber", rgba=(0.86, 0.63, 0.22, 1.0))
    model.material("pad_lime", rgba=(0.57, 0.77, 0.27, 1.0))
    model.material("pad_teal", rgba=(0.24, 0.70, 0.72, 1.0))
    model.material("knob_body", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("knob_cap", rgba=(0.70, 0.72, 0.74, 1.0))

    housing_width = 0.310
    housing_depth = 0.270
    housing_height = 0.036
    wall_thickness = 0.004
    bottom_thickness = 0.002
    top_thickness = 0.003

    opening_size = 0.040
    rib_width = 0.006
    grid_border = 0.010
    pad_zone_span = 2.0 * grid_border + 4.0 * opening_size + 3.0 * rib_width
    grid_center_y = 0.018
    grid_left = -pad_zone_span / 2.0
    grid_front = grid_center_y - pad_zone_span / 2.0

    pad_pitch = opening_size + rib_width
    pad_cap_size = 0.038
    pad_cap_thickness = 0.004
    pad_stem_size = 0.028
    pad_stem_thickness = 0.010
    pad_retainer_size = 0.044
    pad_retainer_thickness = 0.002
    pad_plunger_size = 0.018
    pad_plunger_thickness = 0.004
    pad_proud = 0.0012
    pad_travel = 0.0018

    housing = model.part("housing")
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, housing_depth - 2.0 * wall_thickness, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material="housing_shell",
        name="housing_bottom",
    )
    housing.visual(
        Box((housing_width, wall_thickness, housing_height)),
        origin=Origin(xyz=(0.0, -(housing_depth - wall_thickness) / 2.0, housing_height / 2.0)),
        material="housing_shell",
        name="front_wall",
    )
    housing.visual(
        Box((housing_width, wall_thickness, housing_height)),
        origin=Origin(xyz=(0.0, (housing_depth - wall_thickness) / 2.0, housing_height / 2.0)),
        material="housing_shell",
        name="rear_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth - 2.0 * wall_thickness, housing_height)),
        origin=Origin(xyz=(-(housing_width - wall_thickness) / 2.0, 0.0, housing_height / 2.0)),
        material="housing_shell",
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth - 2.0 * wall_thickness, housing_height)),
        origin=Origin(xyz=((housing_width - wall_thickness) / 2.0, 0.0, housing_height / 2.0)),
        material="housing_shell",
        name="right_wall",
    )

    front_deck_depth = grid_front + housing_depth / 2.0
    rear_deck_depth = housing_depth / 2.0 - (grid_front + pad_zone_span)
    side_deck_width = housing_width / 2.0 + grid_left

    housing.visual(
        Box((housing_width, front_deck_depth, top_thickness)),
        origin=Origin(
            xyz=(0.0, -housing_depth / 2.0 + front_deck_depth / 2.0, housing_height - top_thickness / 2.0)
        ),
        material="panel_surface",
        name="front_deck",
    )
    housing.visual(
        Box((housing_width, rear_deck_depth, top_thickness)),
        origin=Origin(
            xyz=(0.0, housing_depth / 2.0 - rear_deck_depth / 2.0, housing_height - top_thickness / 2.0)
        ),
        material="panel_surface",
        name="rear_deck",
    )
    housing.visual(
        Box((side_deck_width, pad_zone_span, top_thickness)),
        origin=Origin(
            xyz=(-housing_width / 2.0 + side_deck_width / 2.0, grid_center_y, housing_height - top_thickness / 2.0)
        ),
        material="panel_surface",
        name="left_deck",
    )
    housing.visual(
        Box((side_deck_width, pad_zone_span, top_thickness)),
        origin=Origin(
            xyz=(housing_width / 2.0 - side_deck_width / 2.0, grid_center_y, housing_height - top_thickness / 2.0)
        ),
        material="panel_surface",
        name="right_deck",
    )

    x_strip_starts = [
        grid_left,
        grid_left + grid_border + opening_size,
        grid_left + grid_border + opening_size + rib_width + opening_size,
        grid_left + grid_border + 2.0 * opening_size + 2.0 * rib_width + opening_size,
        grid_left + grid_border + 3.0 * opening_size + 3.0 * rib_width,
    ]
    x_strip_widths = [grid_border, rib_width, rib_width, rib_width, grid_border]
    for idx, (start_x, width) in enumerate(zip(x_strip_starts, x_strip_widths), start=1):
        housing.visual(
            Box((width, pad_zone_span, top_thickness)),
            origin=Origin(
                xyz=(start_x + width / 2.0, grid_center_y, housing_height - top_thickness / 2.0)
            ),
            material="panel_surface",
            name=f"grid_x_rib_{idx}",
        )

    y_strip_starts = [
        grid_front,
        grid_front + grid_border + opening_size,
        grid_front + grid_border + opening_size + rib_width + opening_size,
        grid_front + grid_border + 2.0 * opening_size + 2.0 * rib_width + opening_size,
        grid_front + grid_border + 3.0 * opening_size + 3.0 * rib_width,
    ]
    y_strip_widths = [grid_border, rib_width, rib_width, rib_width, grid_border]
    for idx, (start_y, width) in enumerate(zip(y_strip_starts, y_strip_widths), start=1):
        housing.visual(
            Box((pad_zone_span, width, top_thickness)),
            origin=Origin(
                xyz=(0.0, start_y + width / 2.0, housing_height - top_thickness / 2.0)
            ),
            material="panel_surface",
            name=f"grid_y_rib_{idx}",
        )

    pad_materials = ["pad_coral", "pad_amber", "pad_lime", "pad_teal"]
    pad_x_positions = [
        grid_left + grid_border + opening_size / 2.0 + col * pad_pitch for col in range(4)
    ]
    pad_y_positions = [
        grid_front + grid_border + opening_size / 2.0 + row * pad_pitch for row in range(4)
    ]

    for row_idx, pad_y in enumerate(pad_y_positions, start=1):
        for col_idx, pad_x in enumerate(pad_x_positions, start=1):
            pad_name = f"pad_r{row_idx}_c{col_idx}"
            pad_part = model.part(pad_name)
            pad_part.visual(
                Box((pad_cap_size, pad_cap_size, pad_cap_thickness)),
                origin=Origin(),
                material=pad_materials[row_idx - 1],
                name="pad_cap",
            )
            pad_part.visual(
                Box((pad_stem_size, pad_stem_size, pad_stem_thickness)),
                origin=Origin(xyz=(0.0, 0.0, -(pad_cap_thickness / 2.0 + pad_stem_thickness / 2.0))),
                material="pad_body",
                name="pad_stem",
            )
            pad_part.visual(
                Box((pad_retainer_size, pad_retainer_size, pad_retainer_thickness)),
                origin=Origin(
                    xyz=(
                        0.0,
                        0.0,
                        -(pad_cap_thickness / 2.0 + pad_retainer_thickness / 2.0 + 0.0002),
                    )
                ),
                material="pad_body",
                name="pad_retainer",
            )
            pad_part.visual(
                Box((pad_plunger_size, pad_plunger_size, pad_plunger_thickness)),
                origin=Origin(
                    xyz=(
                        0.0,
                        0.0,
                        -(pad_cap_thickness / 2.0 + pad_stem_thickness + pad_plunger_thickness / 2.0),
                    )
                ),
                material="pad_body",
                name="pad_plunger",
            )

            model.articulation(
                f"{pad_name}_press",
                ArticulationType.PRISMATIC,
                parent=housing,
                child=pad_part,
                origin=Origin(
                    xyz=(
                        pad_x,
                        pad_y,
                        housing_height - pad_cap_thickness / 2.0 + pad_proud,
                    )
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=pad_travel,
                ),
            )

    knob_x_positions = (-0.102, -0.034, 0.034, 0.102)
    knob_y = -0.100
    knob_flange_radius = 0.0115
    knob_flange_height = 0.004
    knob_body_radius = 0.0102
    knob_body_height = 0.010
    knob_top_radius = 0.0086
    knob_top_height = 0.002
    indicator_width = 0.0022
    indicator_depth = 0.007
    indicator_height = 0.0012

    for knob_idx, knob_x in enumerate(knob_x_positions, start=1):
        knob_name = f"encoder_{knob_idx}"
        knob_part = model.part(knob_name)
        knob_part.visual(
            Cylinder(radius=knob_flange_radius, length=knob_flange_height),
            origin=Origin(xyz=(0.0, 0.0, knob_flange_height / 2.0)),
            material="knob_body",
            name="knob_skirt",
        )
        knob_part.visual(
            Cylinder(radius=knob_body_radius, length=knob_body_height),
            origin=Origin(xyz=(0.0, 0.0, knob_flange_height + knob_body_height / 2.0)),
            material="knob_body",
            name="knob_body",
        )
        knob_part.visual(
            Cylinder(radius=knob_top_radius, length=knob_top_height),
            origin=Origin(
                xyz=(0.0, 0.0, knob_flange_height + knob_body_height + knob_top_height / 2.0)
            ),
            material="knob_cap",
            name="knob_top",
        )
        knob_part.visual(
            Box((indicator_width, indicator_depth, indicator_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    knob_top_radius * 0.45,
                    knob_flange_height + knob_body_height + knob_top_height + indicator_height / 2.0,
                )
            ),
            material="knob_cap",
            name="knob_indicator",
        )

        model.articulation(
            f"{knob_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob_part,
            origin=Origin(xyz=(knob_x, knob_y, housing_height)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    pad_names = [f"pad_r{row}_c{col}" for row in range(1, 5) for col in range(1, 5)]
    knob_names = [f"encoder_{idx}" for idx in range(1, 5)]

    pad_parts = [object_model.get_part(name) for name in pad_names]
    knob_parts = [object_model.get_part(name) for name in knob_names]
    pad_joints = [object_model.get_articulation(f"{name}_press") for name in pad_names]
    knob_joints = [object_model.get_articulation(f"{name}_spin") for name in knob_names]

    ctx.check(
        "controller includes 16 pads and 4 encoders",
        len(pad_parts) == 16 and len(knob_parts) == 4,
        details=f"pads={len(pad_parts)}, encoders={len(knob_parts)}",
    )

    ctx.check(
        "all pad joints are short downward plungers",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (0.0, 0.0, -1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and 0.001 <= joint.motion_limits.upper <= 0.003
            for joint in pad_joints
        ),
        details=", ".join(
            f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}, "
            f"limits={None if joint.motion_limits is None else (joint.motion_limits.lower, joint.motion_limits.upper)}"
            for joint in pad_joints
        ),
    )

    ctx.check(
        "all encoder joints spin continuously about vertical axes",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in knob_joints
        ),
        details=", ".join(
            f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}, "
            f"limits={None if joint.motion_limits is None else (joint.motion_limits.lower, joint.motion_limits.upper)}"
            for joint in knob_joints
        ),
    )

    for pad_part in pad_parts:
        ctx.expect_within(
            pad_part,
            housing,
            axes="xy",
            margin=0.0,
            name=f"{pad_part.name} stays inside the controller footprint",
        )

    for knob_part in knob_parts:
        ctx.expect_contact(
            knob_part,
            housing,
            elem_a="knob_skirt",
            name=f"{knob_part.name} is seated on the housing top",
        )

    pad_motion_ok = True
    pad_motion_details: list[str] = []
    for pad_part, pad_joint in zip(pad_parts, pad_joints):
        rest_position = ctx.part_world_position(pad_part)
        upper = 0.0 if pad_joint.motion_limits is None or pad_joint.motion_limits.upper is None else pad_joint.motion_limits.upper
        with ctx.pose({pad_joint: upper}):
            pressed_position = ctx.part_world_position(pad_part)

        moved_down = (
            rest_position is not None
            and pressed_position is not None
            and pressed_position[2] < rest_position[2] - 0.001
            and abs(pressed_position[0] - rest_position[0]) < 1e-6
            and abs(pressed_position[1] - rest_position[1]) < 1e-6
        )
        if not moved_down:
            pad_motion_ok = False
            pad_motion_details.append(
                f"{pad_part.name}: rest={rest_position}, pressed={pressed_position}, upper={upper}"
            )

    ctx.check(
        "all pads press straight down",
        pad_motion_ok,
        details="; ".join(pad_motion_details),
    )

    first_knob = knob_parts[0]
    first_knob_joint = knob_joints[0]
    rest_knob_position = ctx.part_world_position(first_knob)
    with ctx.pose({first_knob_joint: 1.7}):
        spun_knob_position = ctx.part_world_position(first_knob)
    ctx.check(
        "encoder rotation stays on its axis",
        rest_knob_position is not None
        and spun_knob_position is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_knob_position, spun_knob_position)),
        details=f"rest={rest_knob_position}, spun={spun_knob_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
