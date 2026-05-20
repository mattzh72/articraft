from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ValidationError,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roland_style_drum_machine")

    body_dark = model.material("body_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    deck_dark = model.material("deck_dark", rgba=(0.07, 0.08, 0.09, 1.0))
    rubber_pad = model.material("rubber_pad", rgba=(0.20, 0.21, 0.23, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))
    bezel_grey = model.material("bezel_grey", rgba=(0.30, 0.32, 0.34, 1.0))
    accent_red = model.material("accent_red", rgba=(0.72, 0.12, 0.10, 1.0))
    display_tint = model.material("display_tint", rgba=(0.16, 0.34, 0.40, 0.50))

    body_width = 0.460
    body_depth = 0.300
    body_top = 0.056
    front_lip_top = 0.015
    wall_thickness = 0.008
    shell_bottom = 0.006
    inner_width = body_width - 2.0 * wall_thickness
    inner_depth = body_depth - 2.0 * wall_thickness
    front_y = -body_depth * 0.5
    back_y = body_depth * 0.5
    deck_front_y = -0.090

    def side_wall_loop(x_value: float) -> list[tuple[float, float, float]]:
        return [
            (x_value, front_y, 0.0),
            (x_value, back_y, 0.0),
            (x_value, back_y, body_top),
            (x_value, deck_front_y, body_top),
            (x_value, front_y, front_lip_top),
        ]

    left_side_mesh = mesh_from_geometry(
        section_loft(
            [
                side_wall_loop(-body_width * 0.5),
                side_wall_loop(-body_width * 0.5 + wall_thickness),
            ]
        ),
        "drum_machine_left_side",
    )
    right_side_mesh = mesh_from_geometry(
        section_loft(
            [
                side_wall_loop(body_width * 0.5 - wall_thickness),
                side_wall_loop(body_width * 0.5),
            ]
        ),
        "drum_machine_right_side",
    )

    pad_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.041, 0.041, 0.006),
            0.008,
            cap=True,
            closed=True,
        ),
        "drum_pad_cap",
    )

    housing = model.part("housing")
    housing.visual(
        Box((body_width, body_depth, shell_bottom)),
        origin=Origin(xyz=(0.0, 0.0, shell_bottom * 0.5)),
        material=body_dark,
        name="bottom_shell",
    )
    housing.visual(left_side_mesh, material=body_dark, name="left_cheek")
    housing.visual(right_side_mesh, material=body_dark, name="right_cheek")
    housing.visual(
        Box((inner_width, wall_thickness, front_lip_top)),
        origin=Origin(xyz=(0.0, front_y + wall_thickness * 0.5, front_lip_top * 0.5)),
        material=body_dark,
        name="front_lip",
    )
    housing.visual(
        Box((inner_width, wall_thickness, body_top)),
        origin=Origin(xyz=(0.0, back_y - wall_thickness * 0.5, body_top * 0.5)),
        material=body_dark,
        name="rear_wall",
    )

    slope_rise = body_top - front_lip_top
    slope_run = deck_front_y - front_y
    slope_angle = math.atan2(slope_rise, slope_run)
    slope_length = math.hypot(slope_run, slope_rise) + 0.004
    housing.visual(
        Box((inner_width, slope_length, 0.004)),
        origin=Origin(
            xyz=(0.0, (front_y + deck_front_y) * 0.5, (front_lip_top + body_top) * 0.5),
            rpy=(slope_angle, 0.0, 0.0),
        ),
        material=deck_dark,
        name="front_slope",
    )
    housing.visual(
        Box((inner_width, back_y - deck_front_y, 0.004)),
        origin=Origin(
            xyz=(0.0, (deck_front_y + back_y) * 0.5, body_top - 0.002),
        ),
        material=deck_dark,
        name="top_deck",
    )

    pad_bank_center = (0.100, 0.030)
    pad_pitch = 0.047
    pad_bank_outer = 0.194
    pad_bank_frame = 0.006
    frame_height = 0.010
    frame_z = body_top + frame_height * 0.5
    housing.visual(
        Box((pad_bank_outer, pad_bank_frame, frame_height)),
        origin=Origin(
            xyz=(
                pad_bank_center[0],
                pad_bank_center[1] - pad_bank_outer * 0.5 + pad_bank_frame * 0.5,
                frame_z,
            )
        ),
        material=bezel_grey,
        name="pad_frame_front",
    )
    housing.visual(
        Box((pad_bank_outer, pad_bank_frame, frame_height)),
        origin=Origin(
            xyz=(
                pad_bank_center[0],
                pad_bank_center[1] + pad_bank_outer * 0.5 - pad_bank_frame * 0.5,
                frame_z,
            )
        ),
        material=bezel_grey,
        name="pad_frame_back",
    )
    housing.visual(
        Box((pad_bank_frame, pad_bank_outer - 2.0 * pad_bank_frame, frame_height)),
        origin=Origin(
            xyz=(
                pad_bank_center[0] - pad_bank_outer * 0.5 + pad_bank_frame * 0.5,
                pad_bank_center[1],
                frame_z,
            )
        ),
        material=bezel_grey,
        name="pad_frame_left",
    )
    housing.visual(
        Box((pad_bank_frame, pad_bank_outer - 2.0 * pad_bank_frame, frame_height)),
        origin=Origin(
            xyz=(
                pad_bank_center[0] + pad_bank_outer * 0.5 - pad_bank_frame * 0.5,
                pad_bank_center[1],
                frame_z,
            )
        ),
        material=bezel_grey,
        name="pad_frame_right",
    )
    for rib_number, rib_index in enumerate((-1.0, 0.0, 1.0)):
        housing.visual(
            Box((pad_bank_frame, pad_bank_outer - 2.0 * pad_bank_frame, frame_height)),
            origin=Origin(
                xyz=(
                    pad_bank_center[0] + rib_index * pad_pitch,
                    pad_bank_center[1],
                    frame_z,
                )
            ),
            material=bezel_grey,
            name=f"pad_vertical_rib_{rib_number}",
        )
    for rib_number, rib_index in enumerate((-1.0, 0.0, 1.0)):
        housing.visual(
            Box((pad_bank_outer - 2.0 * pad_bank_frame, pad_bank_frame, frame_height)),
            origin=Origin(
                xyz=(
                    pad_bank_center[0],
                    pad_bank_center[1] + rib_index * pad_pitch,
                    frame_z,
                )
            ),
            material=bezel_grey,
            name=f"pad_horizontal_rib_{rib_number}",
        )

    encoder_mount = (-0.145, 0.068)
    housing.visual(
        Cylinder(radius=0.022, length=0.003),
        origin=Origin(xyz=(encoder_mount[0], encoder_mount[1], body_top + 0.0015)),
        material=bezel_grey,
        name="encoder_bezel",
    )
    housing.visual(
        Box((0.064, 0.026, 0.0025)),
        origin=Origin(xyz=(-0.030, 0.060, body_top + 0.00125)),
        material=display_tint,
        name="display_window",
    )
    housing.visual(
        Box((0.110, 0.028, 0.0015)),
        origin=Origin(xyz=(-0.045, -0.010, body_top + 0.00075)),
        material=accent_red,
        name="accent_bar",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_top)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, body_top * 0.5)),
    )

    encoder = model.part("data_encoder")
    encoder.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=knob_black,
        name="encoder_skirt",
    )
    encoder.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=knob_black,
        name="encoder_grip",
    )
    encoder.visual(
        Box((0.010, 0.0025, 0.0025)),
        origin=Origin(xyz=(0.010, 0.0, 0.022)),
        material=accent_red,
        name="encoder_pointer",
    )
    encoder.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.026),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )
    model.articulation(
        "housing_to_data_encoder",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=encoder,
        origin=Origin(xyz=(encoder_mount[0], encoder_mount[1], body_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    pad_bottom = body_top + 0.0032
    for row in range(4):
        for col in range(4):
            pad = model.part(f"pad_r{row}_c{col}")
            pad.visual(pad_mesh, material=rubber_pad, name="pad_shell")
            pad.inertial = Inertial.from_geometry(
                Box((0.041, 0.041, 0.008)),
                mass=0.05,
                origin=Origin(xyz=(0.0, 0.0, 0.004)),
            )
            pad_x = pad_bank_center[0] + (col - 1.5) * pad_pitch
            pad_y = pad_bank_center[1] + (1.5 - row) * pad_pitch
            model.articulation(
                f"housing_to_pad_r{row}_c{col}",
                ArticulationType.PRISMATIC,
                parent=housing,
                child=pad,
                origin=Origin(xyz=(pad_x, pad_y, pad_bottom)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=12.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=0.0030,
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
    housing = object_model.get_part("housing")
    encoder = object_model.get_part("data_encoder")
    encoder_joint = object_model.get_articulation("housing_to_data_encoder")
    sample_pad = object_model.get_part("pad_r0_c0")
    next_pad = object_model.get_part("pad_r0_c1")
    lower_pad = object_model.get_part("pad_r1_c0")
    sample_pad_joint = object_model.get_articulation("housing_to_pad_r0_c0")

    expected_pad_parts = [f"pad_r{row}_c{col}" for row in range(4) for col in range(4)]
    missing_pad_parts: list[str] = []
    for part_name in expected_pad_parts:
        try:
            object_model.get_part(part_name)
        except ValidationError:
            missing_pad_parts.append(part_name)
    ctx.check(
        "sixteen articulated pads are present",
        not missing_pad_parts,
        details=f"missing={missing_pad_parts}",
    )

    expected_pad_joints = [
        f"housing_to_pad_r{row}_c{col}" for row in range(4) for col in range(4)
    ]
    missing_pad_joints: list[str] = []
    for joint_name in expected_pad_joints:
        try:
            object_model.get_articulation(joint_name)
        except ValidationError:
            missing_pad_joints.append(joint_name)
    ctx.check(
        "each pad has its own prismatic articulation",
        not missing_pad_joints,
        details=f"missing={missing_pad_joints}",
    )

    encoder_limits = encoder_joint.motion_limits
    ctx.check(
        "data encoder uses a continuous rotary articulation",
        encoder_joint.articulation_type == ArticulationType.CONTINUOUS
        and encoder_limits is not None
        and encoder_limits.lower is None
        and encoder_limits.upper is None
        and tuple(round(value, 6) for value in encoder_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={encoder_joint.articulation_type}, axis={encoder_joint.axis}, "
            f"limits={encoder_limits}"
        ),
    )

    ctx.expect_gap(
        encoder,
        housing,
        axis="z",
        positive_elem="encoder_skirt",
        negative_elem="top_deck",
        min_gap=0.0,
        max_gap=0.0005,
        name="encoder knob sits on the top deck",
    )

    ctx.expect_origin_distance(
        next_pad,
        sample_pad,
        axes="x",
        min_dist=0.046,
        max_dist=0.048,
        name="adjacent pads keep a Roland-style horizontal pitch",
    )
    ctx.expect_origin_distance(
        sample_pad,
        lower_pad,
        axes="y",
        min_dist=0.046,
        max_dist=0.048,
        name="adjacent pads keep a Roland-style vertical pitch",
    )
    ctx.expect_gap(
        sample_pad,
        housing,
        axis="z",
        positive_elem="pad_shell",
        negative_elem="top_deck",
        min_gap=0.0030,
        max_gap=0.0036,
        name="sample pad stands proud above the deck at rest",
    )

    encoder_pointer_rest = ctx.part_element_world_aabb(encoder, elem="encoder_pointer")
    rest_pad_position = ctx.part_world_position(sample_pad)
    with ctx.pose({encoder_joint: math.pi * 0.5, sample_pad_joint: 0.0030}):
        encoder_pointer_turned = ctx.part_element_world_aabb(encoder, elem="encoder_pointer")
        pressed_pad_position = ctx.part_world_position(sample_pad)
        ctx.expect_gap(
            sample_pad,
            housing,
            axis="z",
            positive_elem="pad_shell",
            negative_elem="top_deck",
            min_gap=0.0001,
            max_gap=0.0008,
            name="sample pad compresses nearly flush with the deck",
        )

    def aabb_center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    def aabb_center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    rest_pointer_x = aabb_center_x(encoder_pointer_rest)
    rest_pointer_y = aabb_center_y(encoder_pointer_rest)
    turned_pointer_x = aabb_center_x(encoder_pointer_turned)
    turned_pointer_y = aabb_center_y(encoder_pointer_turned)
    ctx.check(
        "encoder pointer moves around the knob when rotated",
        rest_pointer_x is not None
        and rest_pointer_y is not None
        and turned_pointer_x is not None
        and turned_pointer_y is not None
        and turned_pointer_y > rest_pointer_y + 0.007
        and turned_pointer_x < rest_pointer_x - 0.007,
        details=(
            f"rest_center=({rest_pointer_x}, {rest_pointer_y}), "
            f"turned_center=({turned_pointer_x}, {turned_pointer_y})"
        ),
    )
    ctx.check(
        "sample pad travels downward when pressed",
        rest_pad_position is not None
        and pressed_pad_position is not None
        and pressed_pad_position[2] < rest_pad_position[2] - 0.0025,
        details=f"rest={rest_pad_position}, pressed={pressed_pad_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
