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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(
    width: float,
    height: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (center_x - half_w, center_y - half_h),
        (center_x + half_w, center_y - half_h),
        (center_x + half_w, center_y + half_h),
        (center_x - half_w, center_y + half_h),
    ]


def _build_upright_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    slot_width: float,
    slot_height: float,
    slot_center_from_bottom: float,
):
    outer = _rect_profile(width, height)
    slot = _rect_profile(
        slot_width,
        slot_height,
        center_y=slot_center_from_bottom - (height * 0.5),
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [slot],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_whiteboard_easel")

    wood = model.material("oak_frame", rgba=(0.77, 0.66, 0.49, 1.0))
    wood_dark = model.material("oak_dark", rgba=(0.61, 0.49, 0.33, 1.0))
    board_white = model.material("board_white", rgba=(0.97, 0.98, 0.98, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    upright_width = 0.10
    upright_height = 1.50
    upright_thickness = 0.028
    upright_center_x = 0.32
    slot_width = 0.028
    slot_height = 0.46
    slot_center_from_bottom = 0.78

    upright_mesh = mesh_from_geometry(
        _build_upright_mesh(
            width=upright_width,
            height=upright_height,
            thickness=upright_thickness,
            slot_width=slot_width,
            slot_height=slot_height,
            slot_center_from_bottom=slot_center_from_bottom,
        ),
        "whiteboard_easel_upright",
    )

    front_frame = model.part("front_frame")
    front_frame.visual(
        upright_mesh,
        origin=Origin(xyz=(-upright_center_x, 0.0, upright_height * 0.5)),
        material=wood,
        name="left_upright",
    )
    front_frame.visual(
        upright_mesh,
        origin=Origin(xyz=(upright_center_x, 0.0, upright_height * 0.5)),
        material=wood,
        name="right_upright",
    )

    board_center_z = 0.83
    front_frame.visual(
        Box((0.520, 0.008, 0.760)),
        origin=Origin(xyz=(0.0, 0.004, board_center_z)),
        material=board_white,
        name="whiteboard_surface",
    )
    front_frame.visual(
        Box((0.560, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.001, board_center_z + 0.391)),
        material=aluminum,
        name="board_top_rail",
    )
    front_frame.visual(
        Box((0.560, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.001, board_center_z - 0.391)),
        material=aluminum,
        name="board_bottom_rail",
    )
    front_frame.visual(
        Box((0.022, 0.018, 0.804)),
        origin=Origin(xyz=(-0.271, 0.001, board_center_z)),
        material=aluminum,
        name="board_left_rail",
    )
    front_frame.visual(
        Box((0.022, 0.018, 0.804)),
        origin=Origin(xyz=(0.271, 0.001, board_center_z)),
        material=aluminum,
        name="board_right_rail",
    )
    front_frame.visual(
        Box((0.620, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, -0.010, 0.160)),
        material=wood_dark,
        name="base_crossbar",
    )
    front_frame.visual(
        Box((0.700, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.010, 1.490)),
        material=wood_dark,
        name="crown_base",
    )
    front_frame.visual(
        Cylinder(radius=0.010, length=0.220),
        origin=Origin(
            xyz=(0.0, -0.004, 1.510),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="crown_center_barrel",
    )
    front_frame.visual(
        Box((0.120, 0.060, 0.020)),
        origin=Origin(xyz=(-upright_center_x, 0.0, 0.010)),
        material=rubber,
        name="left_foot",
    )
    front_frame.visual(
        Box((0.120, 0.060, 0.020)),
        origin=Origin(xyz=(upright_center_x, 0.0, 0.010)),
        material=rubber,
        name="right_foot",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.760, 0.140, upright_height)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, upright_height * 0.5)),
    )

    crown_cap = model.part("crown_cap")
    crown_cap.visual(
        Box((0.690, 0.070, 0.016)),
        origin=Origin(xyz=(0.0, 0.050, 0.026)),
        material=wood_dark,
        name="crown_leaf",
    )
    crown_cap.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(
            xyz=(-0.150, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="left_knuckle",
    )
    crown_cap.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(
            xyz=(0.150, 0.000, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="right_knuckle",
    )
    crown_cap.visual(
        Box((0.080, 0.016, 0.026)),
        origin=Origin(xyz=(-0.150, 0.010, 0.013)),
        material=graphite,
        name="left_hinge_cheek",
    )
    crown_cap.visual(
        Box((0.080, 0.016, 0.026)),
        origin=Origin(xyz=(0.150, 0.010, 0.013)),
        material=graphite,
        name="right_hinge_cheek",
    )
    crown_cap.visual(
        Box((0.300, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.076, 0.041)),
        material=graphite,
        name="crown_grip",
    )
    crown_cap.inertial = Inertial.from_geometry(
        Box((0.690, 0.085, 0.040)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.042, 0.026)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.520, 0.110, 0.018)),
        origin=Origin(xyz=(0.0, 0.075, -0.070)),
        material=wood_dark,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.520, 0.014, 0.028)),
        origin=Origin(xyz=(0.0, 0.123, -0.049)),
        material=wood_dark,
        name="tray_front_lip",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x_pos = side_sign * 0.290
        pin_x = side_sign * upright_center_x
        tray.visual(
            Box((0.060, 0.034, 0.018)),
            origin=Origin(xyz=(x_pos, 0.034, -0.070)),
            material=wood_dark,
            name=f"{side_name}_tray_arm",
        )
        tray.visual(
            Box((0.034, 0.020, 0.072)),
            origin=Origin(xyz=(pin_x, 0.028, -0.034)),
            material=wood_dark,
            name=f"{side_name}_front_bracket",
        )
        tray.visual(
            Cylinder(radius=0.025, length=0.004),
            origin=Origin(
                xyz=(pin_x, 0.016, 0.000),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"{side_name}_front_knob",
        )
        tray.visual(
            Cylinder(radius=0.025, length=0.004),
            origin=Origin(
                xyz=(pin_x, -0.016, 0.000),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"{side_name}_rear_knob",
        )
        tray.visual(
            Cylinder(radius=0.010, length=0.032),
            origin=Origin(
                xyz=(pin_x, 0.000, 0.000),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"{side_name}_slot_pin",
        )
    tray.inertial = Inertial.from_geometry(
        Box((0.690, 0.130, 0.110)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.055, -0.020)),
    )

    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Box((0.240, 0.016, 0.080)),
        origin=Origin(xyz=(0.0, -0.018, 0.000)),
        material=graphite,
        name="brace_tongue",
    )
    rear_brace.visual(
        Cylinder(radius=0.011, length=0.360),
        origin=Origin(
            xyz=(0.0, -0.012, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="brace_barrel",
    )
    rear_brace.visual(
        Box((0.150, 0.024, 1.020)),
        origin=Origin(xyz=(0.0, -0.235, 0.495), rpy=(0.40, 0.0, 0.0)),
        material=wood,
        name="brace_board",
    )
    rear_brace.inertial = Inertial.from_geometry(
        Box((0.360, 0.440, 1.020)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.200, 0.500)),
    )

    model.articulation(
        "front_frame_to_crown_cap",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=crown_cap,
        origin=Origin(xyz=(0.0, -0.004, 1.510)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "front_frame_to_tray",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, slot_center_from_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=-0.160,
            upper=0.160,
        ),
    )
    model.articulation(
        "front_frame_to_rear_brace",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_brace,
        origin=Origin(xyz=(0.0, -0.020, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.450,
            upper=0.250,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    crown_cap = object_model.get_part("crown_cap")
    tray = object_model.get_part("tray")
    rear_brace = object_model.get_part("rear_brace")
    crown_left_knuckle = crown_cap.get_visual("left_knuckle")
    crown_center_barrel = front_frame.get_visual("crown_center_barrel")
    brace_tongue = rear_brace.get_visual("brace_tongue")
    base_crossbar = front_frame.get_visual("base_crossbar")

    crown_hinge = object_model.get_articulation("front_frame_to_crown_cap")
    tray_slide = object_model.get_articulation("front_frame_to_tray")
    rear_brace_hinge = object_model.get_articulation("front_frame_to_rear_brace")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mechanism_joint_axes",
        crown_hinge.axis == (1.0, 0.0, 0.0)
        and tray_slide.axis == (0.0, 0.0, 1.0)
        and rear_brace_hinge.axis == (1.0, 0.0, 0.0),
        details=(
            f"crown={crown_hinge.axis}, tray={tray_slide.axis}, "
            f"rear={rear_brace_hinge.axis}"
        ),
    )
    ctx.check(
        "mechanism_joint_types",
        crown_hinge.articulation_type == ArticulationType.REVOLUTE
        and tray_slide.articulation_type == ArticulationType.PRISMATIC
        and rear_brace_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"crown={crown_hinge.articulation_type}, "
            f"tray={tray_slide.articulation_type}, "
            f"rear={rear_brace_hinge.articulation_type}"
        ),
    )
    ctx.check(
        "tray_travel_brackets_slot_range",
        tray_slide.motion_limits is not None
        and tray_slide.motion_limits.lower is not None
        and tray_slide.motion_limits.upper is not None
        and tray_slide.motion_limits.lower < 0.0 < tray_slide.motion_limits.upper,
        details=f"tray limits={tray_slide.motion_limits}",
    )

    ctx.expect_contact(
        crown_cap,
        front_frame,
        elem_a=crown_left_knuckle,
        elem_b=crown_center_barrel,
        name="crown_cap_hinge_knuckle_contacts_frame_barrel",
    )
    ctx.expect_contact(
        rear_brace,
        front_frame,
        elem_a=brace_tongue,
        elem_b=base_crossbar,
        name="rear_brace_mounted_to_base_crossbar",
    )

    with ctx.pose({tray_slide: tray_slide.motion_limits.lower}):
        ctx.expect_contact(tray, front_frame, name="tray_contacts_frame_at_low_position")
        ctx.expect_overlap(tray, front_frame, axes="x", min_overlap=0.62, name="tray_spans_uprights_low")

    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_contact(tray, front_frame, name="tray_contacts_frame_at_high_position")
        ctx.expect_overlap(tray, front_frame, axes="x", min_overlap=0.62, name="tray_spans_uprights_high")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
