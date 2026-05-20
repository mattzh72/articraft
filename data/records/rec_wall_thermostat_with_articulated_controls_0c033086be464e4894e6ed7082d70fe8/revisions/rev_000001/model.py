from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate_mesh(
    width: float,
    height: float,
    depth: float,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, height, radius),
            depth,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    wall_white = model.material("wall_white", rgba=(0.94, 0.94, 0.92, 1.0))
    face_ivory = model.material("face_ivory", rgba=(0.97, 0.96, 0.93, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.78, 0.78, 0.76, 1.0))
    dial_charcoal = model.material("dial_charcoal", rgba=(0.20, 0.21, 0.22, 1.0))
    button_gray = model.material("button_gray", rgba=(0.84, 0.84, 0.82, 1.0))
    slider_sage = model.material("slider_sage", rgba=(0.64, 0.69, 0.63, 1.0))

    body_depth = 0.0245
    face_z = body_depth

    housing_mesh = _rounded_plate_mesh(
        width=0.148,
        height=0.108,
        depth=0.021,
        radius=0.014,
        name="thermostat_housing",
    )
    backplate_mesh = _rounded_plate_mesh(
        width=0.156,
        height=0.116,
        depth=0.002,
        radius=0.017,
        name="thermostat_backplate",
    )
    fascia_mesh = _rounded_plate_mesh(
        width=0.142,
        height=0.102,
        depth=0.0015,
        radius=0.012,
        name="thermostat_fascia",
    )
    button_mesh = _rounded_plate_mesh(
        width=0.020,
        height=0.012,
        depth=0.004,
        radius=0.003,
        name="thermostat_button",
    )
    slider_mesh = _rounded_plate_mesh(
        width=0.014,
        height=0.0114,
        depth=0.004,
        radius=0.0025,
        name="thermostat_slider_thumb",
    )
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.071,
            0.014,
            body_style="domed",
            top_diameter=0.064,
            edge_radius=0.0012,
            side_draft_deg=4.0,
            center=False,
        ),
        "thermostat_dial",
    )

    body = model.part("body")
    body.visual(backplate_mesh, material=wall_white, name="backplate")
    body.visual(
        housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_gray,
        name="housing",
    )
    body.visual(
        fascia_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=face_ivory,
        name="face_plate",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.0012),
        origin=Origin(xyz=(0.0, 0.010, face_z + 0.0006)),
        material=trim_gray,
        name="dial_seat",
    )

    button_frame_depth = 0.005
    button_outer_width = 0.0232
    button_outer_height = 0.0152
    button_wall = 0.0016
    for button_side, button_x in (("left", -0.049), ("right", 0.049)):
        body.visual(
            Box((button_outer_width, button_wall, button_frame_depth)),
            origin=Origin(xyz=(button_x, 0.0168, face_z + button_frame_depth * 0.5)),
            material=trim_gray,
            name=f"{button_side}_button_bezel_top",
        )
        body.visual(
            Box((button_outer_width, button_wall, button_frame_depth)),
            origin=Origin(xyz=(button_x, 0.0032, face_z + button_frame_depth * 0.5)),
            material=trim_gray,
            name=f"{button_side}_button_bezel_bottom",
        )
        body.visual(
            Box((button_wall, button_outer_height, button_frame_depth)),
            origin=Origin(xyz=(button_x - 0.0108, 0.010, face_z + button_frame_depth * 0.5)),
            material=trim_gray,
            name=f"{button_side}_button_bezel_inner",
        )
        body.visual(
            Box((button_wall, button_outer_height, button_frame_depth)),
            origin=Origin(xyz=(button_x + 0.0108, 0.010, face_z + button_frame_depth * 0.5)),
            material=trim_gray,
            name=f"{button_side}_button_bezel_outer",
        )

    slider_center_y = -0.034
    track_length = 0.066
    track_height = 0.018
    body.visual(
        Box((track_length, track_height, 0.0016)),
        origin=Origin(xyz=(0.0, slider_center_y, face_z + 0.0008)),
        material=trim_gray,
        name="track_backer",
    )
    body.visual(
        Box((track_length, 0.0022, 0.0050)),
        origin=Origin(xyz=(0.0, slider_center_y + 0.0068, face_z + 0.0025)),
        material=trim_gray,
        name="track_rail_top",
    )
    body.visual(
        Box((track_length, 0.0022, 0.0050)),
        origin=Origin(xyz=(0.0, slider_center_y - 0.0068, face_z + 0.0025)),
        material=trim_gray,
        name="track_rail_bottom",
    )
    body.visual(
        Box((0.0022, track_height, 0.0034)),
        origin=Origin(xyz=(-0.0319, slider_center_y, face_z + 0.0017)),
        material=trim_gray,
        name="track_stop_left",
    )
    body.visual(
        Box((0.0022, track_height, 0.0034)),
        origin=Origin(xyz=(0.0319, slider_center_y, face_z + 0.0017)),
        material=trim_gray,
        name="track_stop_right",
    )

    dial = model.part("dial")
    dial.visual(dial_mesh, material=dial_charcoal, name="dial_cap")
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.010, face_z + 0.0012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    left_button = model.part("left_button")
    left_button.visual(
        button_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0032)),
        material=button_gray,
        name="button_cap",
    )
    model.articulation(
        "body_to_left_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_button,
        origin=Origin(xyz=(-0.049, 0.010, face_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0022,
        ),
    )

    right_button = model.part("right_button")
    right_button.visual(
        button_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0032)),
        material=button_gray,
        name="button_cap",
    )
    model.articulation(
        "body_to_right_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_button,
        origin=Origin(xyz=(0.049, 0.010, face_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0022,
        ),
    )

    slider = model.part("slider")
    slider.visual(
        slider_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0034)),
        material=slider_sage,
        name="slider_thumb",
    )
    slider.visual(
        Box((0.0022, 0.0076, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0080)),
        material=button_gray,
        name="slider_rib",
    )
    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(0.0, slider_center_y, face_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.04,
            lower=-0.008,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    slider = object_model.get_part("slider")

    dial_joint = object_model.get_articulation("body_to_dial")
    left_button_joint = object_model.get_articulation("body_to_left_button")
    right_button_joint = object_model.get_articulation("body_to_right_button")
    slider_joint = object_model.get_articulation("body_to_slider")

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=(
            f"type={dial_joint.articulation_type}, "
            f"limits=({None if dial_limits is None else dial_limits.lower}, "
            f"{None if dial_limits is None else dial_limits.upper})"
        ),
    )

    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="dial_seat",
        min_gap=0.0,
        max_gap=0.001,
        name="dial seats just above the dial boss",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="dial_cap",
        elem_b="dial_seat",
        min_overlap=0.060,
        name="dial stays centered over the dial boss",
    )

    for button_part, button_name in (
        (left_button, "left"),
        (right_button, "right"),
    ):
        ctx.expect_gap(
            button_part,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="face_plate",
            min_gap=0.0025,
            max_gap=0.0045,
            name=f"{button_name} button stands proud of the thermostat face",
        )
        ctx.expect_overlap(
            button_part,
            body,
            axes="xy",
            elem_a="button_cap",
            elem_b="face_plate",
            min_overlap=0.010,
            name=f"{button_name} button stays mounted over the front fascia",
        )

    ctx.expect_gap(
        slider,
        body,
        axis="z",
        positive_elem="slider_thumb",
        negative_elem="track_backer",
        min_gap=0.0015,
        max_gap=0.0045,
        name="slider thumb rides proud of the guide backer",
    )
    ctx.expect_within(
        slider,
        body,
        axes="y",
        inner_elem="slider_thumb",
        outer_elem="track_backer",
        margin=0.0020,
        name="slider thumb remains within the guide height",
    )
    ctx.expect_overlap(
        slider,
        body,
        axes="x",
        elem_a="slider_thumb",
        elem_b="track_backer",
        min_overlap=0.012,
        name="slider thumb remains captured by the guide length",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: 1.7}):
        dial_rotated = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place about the center axis",
        dial_rest is not None
        and dial_rotated is not None
        and abs(dial_rotated[0] - dial_rest[0]) < 1e-6
        and abs(dial_rotated[1] - dial_rest[1]) < 1e-6
        and abs(dial_rotated[2] - dial_rest[2]) < 1e-6,
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    left_upper = left_button_joint.motion_limits.upper if left_button_joint.motion_limits is not None else 0.0
    with ctx.pose({left_button_joint: left_upper}):
        left_pressed = ctx.part_world_position(left_button)
        right_during_left_press = ctx.part_world_position(right_button)
        ctx.expect_gap(
            left_button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="face_plate",
            max_penetration=0.0,
            max_gap=0.0025,
            name="left button still clears the face when fully pressed",
        )
    ctx.check(
        "left button depresses independently",
        left_rest is not None
        and right_rest is not None
        and left_pressed is not None
        and right_during_left_press is not None
        and left_pressed[2] < left_rest[2] - 0.0015
        and abs(right_during_left_press[2] - right_rest[2]) < 1e-6,
        details=(
            f"left_rest={left_rest}, left_pressed={left_pressed}, "
            f"right_rest={right_rest}, right_during_left_press={right_during_left_press}"
        ),
    )

    right_upper = right_button_joint.motion_limits.upper if right_button_joint.motion_limits is not None else 0.0
    with ctx.pose({right_button_joint: right_upper}):
        right_pressed = ctx.part_world_position(right_button)
        left_during_right_press = ctx.part_world_position(left_button)
        ctx.expect_gap(
            right_button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="face_plate",
            max_penetration=0.0,
            max_gap=0.0025,
            name="right button still clears the face when fully pressed",
        )
    ctx.check(
        "right button depresses independently",
        right_rest is not None
        and left_rest is not None
        and right_pressed is not None
        and left_during_right_press is not None
        and right_pressed[2] < right_rest[2] - 0.0015
        and abs(left_during_right_press[2] - left_rest[2]) < 1e-6,
        details=(
            f"right_rest={right_rest}, right_pressed={right_pressed}, "
            f"left_rest={left_rest}, left_during_right_press={left_during_right_press}"
        ),
    )

    slider_rest = ctx.part_world_position(slider)
    slider_upper = slider_joint.motion_limits.upper if slider_joint.motion_limits is not None else 0.0
    with ctx.pose({slider_joint: slider_upper}):
        slider_shifted = ctx.part_world_position(slider)
        ctx.expect_within(
            slider,
            body,
            axes="y",
            inner_elem="slider_thumb",
            outer_elem="track_backer",
            margin=0.0020,
            name="slider stays in the guide at full travel",
        )
        ctx.expect_overlap(
            slider,
            body,
            axes="x",
            elem_a="slider_thumb",
            elem_b="track_backer",
            min_overlap=0.012,
            name="slider remains retained in the guide at full travel",
        )
    ctx.check(
        "slider travels laterally along the lower guide",
        slider_rest is not None
        and slider_shifted is not None
        and slider_shifted[0] > slider_rest[0] + 0.006
        and abs(slider_shifted[1] - slider_rest[1]) < 1e-6
        and abs(slider_shifted[2] - slider_rest[2]) < 1e-6,
        details=f"rest={slider_rest}, shifted={slider_shifted}",
    )

    return ctx.report()


object_model = build_object_model()
