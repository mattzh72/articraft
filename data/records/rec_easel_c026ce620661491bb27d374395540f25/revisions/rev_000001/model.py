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
    model = ArticulatedObject(name="h_frame_studio_easel")

    oak = model.material("oak", rgba=(0.64, 0.48, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.22, 0.23, 0.25, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    front_leg_x = 0.30
    front_leg_size = (0.06, 0.035, 1.78)
    rear_leg_size = (0.05, 0.03, 1.55)

    frame = model.part("frame")
    frame.visual(
        Box(front_leg_size),
        origin=Origin(xyz=(-front_leg_x, 0.0, front_leg_size[2] / 2.0)),
        material=oak,
        name="frame_front_left_leg",
    )
    frame.visual(
        Box(front_leg_size),
        origin=Origin(xyz=(front_leg_x, 0.0, front_leg_size[2] / 2.0)),
        material=oak,
        name="frame_front_right_leg",
    )
    frame.visual(
        Box((0.72, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.73)),
        material=oak,
        name="frame_top_header",
    )
    frame.visual(
        Box((0.72, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=oak,
        name="frame_front_floor_stretcher",
    )
    frame.visual(
        Box((0.66, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        material=oak,
        name="frame_mid_crossbar",
    )

    frame.visual(
        Box((0.055, 0.42, 0.05)),
        origin=Origin(xyz=(-0.2725, 0.21, 0.025)),
        material=oak,
        name="frame_left_base_rail",
    )
    frame.visual(
        Box((0.055, 0.42, 0.05)),
        origin=Origin(xyz=(0.2725, 0.21, 0.025)),
        material=oak,
        name="frame_right_base_rail",
    )

    frame.visual(
        Box(rear_leg_size),
        origin=Origin(xyz=(-0.24, 0.20, 0.79), rpy=(0.24, 0.0, 0.0)),
        material=oak,
        name="frame_rear_left_leg",
    )
    frame.visual(
        Box(rear_leg_size),
        origin=Origin(xyz=(0.24, 0.20, 0.79), rpy=(0.24, 0.0, 0.0)),
        material=oak,
        name="frame_rear_right_leg",
    )
    frame.visual(
        Box((0.52, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.31, 0.31)),
        material=oak,
        name="frame_rear_lower_brace",
    )
    frame.visual(
        Box((0.46, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.10, 1.23)),
        material=oak,
        name="frame_rear_upper_brace",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.46, 1.82)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.16, 0.91)),
    )

    def add_slide_bracket_set(
        part,
        *,
        bracket_height: float,
        bracket_z: float,
        bar_size: tuple[float, float, float],
        bar_y: float,
        bar_name: str,
        left_front_name: str,
        right_front_name: str,
        left_outer_cheek_name: str,
        left_inner_cheek_name: str,
        right_inner_cheek_name: str,
        right_outer_cheek_name: str,
        left_knob_name: str,
        right_knob_name: str,
        lip_size: tuple[float, float, float] | None = None,
        lip_origin: tuple[float, float, float] | None = None,
        lip_name: str | None = None,
        knob_radius: float = 0.011,
        knob_length: float = 0.022,
        knob_z: float = 0.02,
    ) -> None:
        cheek_thickness = 0.014
        cheek_depth = 0.05
        plate_thickness = 0.022
        side_clearance = 0.003
        leg_half_width = front_leg_size[0] / 2.0
        cheek_offset = leg_half_width + side_clearance + cheek_thickness / 2.0
        plate_width = front_leg_size[0] + 2.0 * (side_clearance + cheek_thickness)
        plate_y = -0.0285
        cheek_y = -0.0085
        bracket_positions = {
            "left": -front_leg_x,
            "right": front_leg_x,
        }

        part.visual(
            Box(bar_size),
            origin=Origin(xyz=(0.0, bar_y, bracket_z)),
            material=oak,
            name=bar_name,
        )

        if lip_size is not None and lip_origin is not None and lip_name is not None:
            part.visual(
                Box(lip_size),
                origin=Origin(xyz=lip_origin),
                material=oak,
                name=lip_name,
            )

        front_names = {"left": left_front_name, "right": right_front_name}
        outer_cheek_names = {"left": left_outer_cheek_name, "right": right_outer_cheek_name}
        inner_cheek_names = {"left": left_inner_cheek_name, "right": right_inner_cheek_name}
        knob_names = {"left": left_knob_name, "right": right_knob_name}

        for side_name, x_center in bracket_positions.items():
            part.visual(
                Box((plate_width, plate_thickness, bracket_height)),
                origin=Origin(xyz=(x_center, plate_y, bracket_z)),
                material=steel,
                name=front_names[side_name],
            )
            part.visual(
                Box((cheek_thickness, cheek_depth, bracket_height)),
                origin=Origin(xyz=(x_center - cheek_offset, cheek_y, bracket_z)),
                material=steel,
                name=outer_cheek_names[side_name] if side_name == "left" else inner_cheek_names[side_name],
            )
            part.visual(
                Box((cheek_thickness, cheek_depth, bracket_height)),
                origin=Origin(xyz=(x_center + cheek_offset, cheek_y, bracket_z)),
                material=steel,
                name=inner_cheek_names[side_name] if side_name == "left" else outer_cheek_names[side_name],
            )

            knob_sign = -1.0 if side_name == "left" else 1.0
            part.visual(
                Cylinder(radius=knob_radius, length=knob_length),
                origin=Origin(
                    xyz=(
                        x_center + knob_sign * (cheek_offset + cheek_thickness / 2.0 + knob_length / 2.0),
                        -0.004,
                        knob_z,
                    ),
                    rpy=(0.0, 1.5707963267948966, 0.0),
                ),
                material=knob_black,
                name=knob_names[side_name],
            )

    canvas_support = model.part("canvas_support")
    add_slide_bracket_set(
        canvas_support,
        bracket_height=0.18,
        bracket_z=0.0,
        bar_size=(0.66, 0.04, 0.06),
        bar_y=-0.056,
        bar_name="canvas_support_beam",
        left_front_name="canvas_left_bracket_front",
        right_front_name="canvas_right_bracket_front",
        left_outer_cheek_name="canvas_left_bracket_outer_cheek",
        left_inner_cheek_name="canvas_left_bracket_inner_cheek",
        right_inner_cheek_name="canvas_right_bracket_inner_cheek",
        right_outer_cheek_name="canvas_right_bracket_outer_cheek",
        left_knob_name="canvas_left_locking_knob",
        right_knob_name="canvas_right_locking_knob",
        lip_size=(0.66, 0.018, 0.045),
        lip_origin=(0.0, -0.064, 0.016),
        lip_name="canvas_support_lip",
        knob_radius=0.012,
        knob_length=0.024,
        knob_z=0.028,
    )
    canvas_support.inertial = Inertial.from_geometry(
        Box((0.74, 0.12, 0.18)),
        mass=1.9,
        origin=Origin(xyz=(0.0, -0.05, 0.01)),
    )

    top_clamp = model.part("top_clamp")
    add_slide_bracket_set(
        top_clamp,
        bracket_height=0.15,
        bracket_z=0.0,
        bar_size=(0.56, 0.034, 0.05),
        bar_y=-0.050,
        bar_name="top_clamp_beam",
        left_front_name="clamp_left_bracket_front",
        right_front_name="clamp_right_bracket_front",
        left_outer_cheek_name="clamp_left_bracket_outer_cheek",
        left_inner_cheek_name="clamp_left_bracket_inner_cheek",
        right_inner_cheek_name="clamp_right_bracket_inner_cheek",
        right_outer_cheek_name="clamp_right_bracket_outer_cheek",
        left_knob_name="clamp_left_locking_knob",
        right_knob_name="clamp_right_locking_knob",
        lip_size=(0.28, 0.016, 0.045),
        lip_origin=(0.0, -0.062, -0.025),
        lip_name="top_clamp_jaw",
        knob_radius=0.010,
        knob_length=0.022,
        knob_z=0.020,
    )
    top_clamp.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.0, -0.075, 0.015), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=knob_black,
        name="top_clamp_center_grip",
    )
    top_clamp.inertial = Inertial.from_geometry(
        Box((0.64, 0.10, 0.16)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.05, 0.0)),
    )

    model.articulation(
        "frame_to_canvas_support",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=canvas_support,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.86,
        ),
    )

    model.articulation(
        "frame_to_top_clamp",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=top_clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.15,
            lower=0.0,
            upper=0.42,
        ),
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

    frame = object_model.get_part("frame")
    canvas_support = object_model.get_part("canvas_support")
    top_clamp = object_model.get_part("top_clamp")
    canvas_slide = object_model.get_articulation("frame_to_canvas_support")
    clamp_slide = object_model.get_articulation("frame_to_top_clamp")

    ctx.check(
        "canvas support uses vertical prismatic motion",
        canvas_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(canvas_slide.axis) == (0.0, 0.0, 1.0)
        and canvas_slide.motion_limits is not None
        and canvas_slide.motion_limits.lower == 0.0
        and canvas_slide.motion_limits.upper is not None
        and canvas_slide.motion_limits.upper > 0.5,
        details=f"type={canvas_slide.articulation_type}, axis={canvas_slide.axis}, limits={canvas_slide.motion_limits}",
    )
    ctx.check(
        "top clamp uses vertical prismatic motion",
        clamp_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(clamp_slide.axis) == (0.0, 0.0, 1.0)
        and clamp_slide.motion_limits is not None
        and clamp_slide.motion_limits.lower == 0.0
        and clamp_slide.motion_limits.upper is not None
        and clamp_slide.motion_limits.upper > 0.25,
        details=f"type={clamp_slide.articulation_type}, axis={clamp_slide.axis}, limits={clamp_slide.motion_limits}",
    )

    with ctx.pose({canvas_slide: 0.0, clamp_slide: 0.0}):
        ctx.expect_contact(
            frame,
            canvas_support,
            elem_a="frame_front_left_leg",
            elem_b="canvas_left_bracket_front",
            name="canvas support front bracket bears on the left leg",
        )
        ctx.expect_overlap(
            canvas_support,
            frame,
            axes="z",
            elem_a="canvas_left_bracket_front",
            elem_b="frame_front_left_leg",
            min_overlap=0.16,
            name="canvas support left bracket engages the front leg",
        )
        ctx.expect_contact(
            frame,
            top_clamp,
            elem_a="frame_front_right_leg",
            elem_b="clamp_right_bracket_front",
            name="top clamp front bracket bears on the right leg",
        )
        ctx.expect_overlap(
            top_clamp,
            frame,
            axes="z",
            elem_a="clamp_right_bracket_front",
            elem_b="frame_front_right_leg",
            min_overlap=0.13,
            name="top clamp right bracket engages the front leg",
        )

    canvas_rest = ctx.part_world_position(canvas_support)
    with ctx.pose({canvas_slide: canvas_slide.motion_limits.upper}):
        canvas_raised = ctx.part_world_position(canvas_support)
        ctx.expect_overlap(
            canvas_support,
            frame,
            axes="z",
            elem_a="canvas_right_bracket_front",
            elem_b="frame_front_right_leg",
            min_overlap=0.16,
            name="raised canvas support still rides on the right front leg",
        )
    ctx.check(
        "canvas support slides upward",
        canvas_rest is not None
        and canvas_raised is not None
        and canvas_raised[2] > canvas_rest[2] + 0.20,
        details=f"rest={canvas_rest}, raised={canvas_raised}",
    )

    clamp_rest = ctx.part_world_position(top_clamp)
    with ctx.pose({clamp_slide: clamp_slide.motion_limits.upper}):
        clamp_raised = ctx.part_world_position(top_clamp)
        ctx.expect_overlap(
            top_clamp,
            frame,
            axes="z",
            elem_a="clamp_left_bracket_front",
            elem_b="frame_front_left_leg",
            min_overlap=0.12,
            name="raised top clamp still rides on the left front leg",
        )
    ctx.check(
        "top clamp slides upward",
        clamp_rest is not None
        and clamp_raised is not None
        and clamp_raised[2] > clamp_rest[2] + 0.10,
        details=f"rest={clamp_rest}, raised={clamp_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
