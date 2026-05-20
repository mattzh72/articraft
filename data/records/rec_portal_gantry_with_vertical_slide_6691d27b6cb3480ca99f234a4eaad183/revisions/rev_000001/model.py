from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


FOOT_WIDTH = 0.10
FOOT_DEPTH = 0.30
FOOT_HEIGHT = 0.04

UPRIGHT_WIDTH = 0.09
UPRIGHT_DEPTH = 0.12
UPRIGHT_HEIGHT = 0.46
SUPPORT_CENTER_X = 0.315

BEAM_LENGTH = 0.72
BEAM_DEPTH = 0.12
BEAM_HEIGHT = 0.10
BEAM_CENTER_Z = FOOT_HEIGHT + UPRIGHT_HEIGHT + (BEAM_HEIGHT / 2.0)

BEAM_RAIL_LENGTH = 0.56
BEAM_RAIL_DEPTH = 0.008
BEAM_RAIL_HEIGHT = 0.014
BEAM_RAIL_OFFSET_Z = 0.025
BEAM_RAIL_CENTER_Y = -(BEAM_DEPTH / 2.0) - (BEAM_RAIL_DEPTH / 2.0)
BEAM_RAIL_FRONT_Y = -(BEAM_DEPTH / 2.0) - BEAM_RAIL_DEPTH

CARRIAGE_WIDTH = 0.18
CARRIAGE_HEIGHT = 0.16
CARRIAGE_PLATE_DEPTH = 0.026
CARRIAGE_PLATE_CENTER_Y = -0.021
CARRIAGE_BEARING_WIDTH = 0.16
CARRIAGE_BEARING_DEPTH = 0.008
CARRIAGE_BEARING_HEIGHT = 0.032
CARRIAGE_BEARING_OFFSET_Z = BEAM_RAIL_OFFSET_Z
CARRIAGE_MOUNT_WIDTH = 0.10
CARRIAGE_MOUNT_DEPTH = 0.040
CARRIAGE_MOUNT_HEIGHT = 0.10
CARRIAGE_MOUNT_CENTER_Y = -0.054
CARRIAGE_MOUNT_CENTER_Z = -0.005
CARRIAGE_CAP_DEPTH = 0.016
CARRIAGE_CAP_HEIGHT = 0.04
CARRIAGE_CAP_CENTER_Y = -0.023
CARRIAGE_CAP_CENTER_Z = 0.055

Z_SLIDE_WIDTH = 0.14
Z_SLIDE_PLATE_DEPTH = 0.022
Z_SLIDE_HEIGHT = 0.36
Z_SLIDE_BODY_CENTER_Y = -0.011
Z_SLIDE_BODY_CENTER_Z = -0.18
Z_TOP_CAP_DEPTH = 0.030
Z_TOP_CAP_HEIGHT = 0.05
Z_TOP_CAP_CENTER_Y = -0.015
Z_TOP_CAP_CENTER_Z = -0.025
Z_SIDE_CHEEK_WIDTH = 0.014
Z_SIDE_CHEEK_DEPTH = 0.024
Z_SIDE_CHEEK_HEIGHT = 0.24
Z_SIDE_CHEEK_CENTER_Y = -0.012
Z_SIDE_CHEEK_CENTER_Z = -0.18
Z_SIDE_CHEEK_OFFSET_X = 0.053
Z_RAIL_WIDTH = 0.018
Z_RAIL_DEPTH = 0.008
Z_RAIL_HEIGHT = 0.30
Z_RAIL_OFFSET_X = 0.035
Z_RAIL_CENTER_Y = -0.026
Z_RAIL_FRONT_Y = -0.030
Z_RAIL_CENTER_Z = -0.18

RAM_SLIDER_WIDTH = 0.08
RAM_SLIDER_DEPTH = 0.024
RAM_SLIDER_HEIGHT = 0.14
RAM_SLIDER_CENTER_Y = -0.020
RAM_PAD_WIDTH = 0.018
RAM_PAD_DEPTH = 0.008
RAM_PAD_HEIGHT = 0.10
RAM_PAD_CENTER_Y = -0.004
RAM_EXTENSION_WIDTH = 0.040
RAM_EXTENSION_DEPTH = 0.040
RAM_EXTENSION_HEIGHT = 0.10
RAM_EXTENSION_CENTER_Y = -0.042
RAM_EXTENSION_CENTER_Z = -0.12
TOOL_PLATE_WIDTH = 0.09
TOOL_PLATE_DEPTH = 0.06
TOOL_PLATE_HEIGHT = 0.012
TOOL_PLATE_CENTER_Y = -0.042
TOOL_PLATE_CENTER_Z = -0.176
SPINDLE_NOSE_RADIUS = 0.012
SPINDLE_NOSE_LENGTH = 0.05
SPINDLE_NOSE_CENTER_Y = -0.042
SPINDLE_NOSE_CENTER_Z = -0.207

X_TRAVEL = 0.19
Z_TRAVEL = 0.08

FRAME_TO_CARRIAGE_ORIGIN = (0.0, BEAM_RAIL_FRONT_Y, BEAM_CENTER_Z)
CARRIAGE_TO_Z_SLIDE_ORIGIN = (
    0.0,
    CARRIAGE_MOUNT_CENTER_Y - (CARRIAGE_MOUNT_DEPTH / 2.0),
    -0.02,
)
Z_SLIDE_TO_RAM_ORIGIN = (0.0, Z_RAIL_FRONT_Y, Z_RAIL_CENTER_Z)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _windowed_upright(center_x: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT)
    window = (
        cq.Workplane("XY")
        .box(UPRIGHT_WIDTH - 0.030, UPRIGHT_DEPTH + 0.006, UPRIGHT_HEIGHT - 0.12)
        .translate((0.0, 0.0, 0.015))
    )
    return (
        outer.cut(window)
        .edges("|Z")
        .fillet(0.005)
        .translate((center_x, 0.0, FOOT_HEIGHT + (UPRIGHT_HEIGHT / 2.0)))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_portal_gantry")

    frame_mat = model.material("frame_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    rail_mat = model.material("rail_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    carriage_mat = model.material("carriage_orange", rgba=(0.90, 0.48, 0.12, 1.0))
    slide_mat = model.material("slide_charcoal", rgba=(0.28, 0.31, 0.34, 1.0))
    ram_mat = model.material("ram_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    tool_mat = model.material("tool_black", rgba=(0.10, 0.10, 0.12, 1.0))

    frame = model.part("frame")

    left_foot = (
        cq.Workplane("XY")
        .box(FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)
        .edges("|Z")
        .fillet(0.008)
        .translate((-SUPPORT_CENTER_X, 0.0, FOOT_HEIGHT / 2.0))
    )
    right_foot = (
        cq.Workplane("XY")
        .box(FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)
        .edges("|Z")
        .fillet(0.008)
        .translate((SUPPORT_CENTER_X, 0.0, FOOT_HEIGHT / 2.0))
    )
    left_upright = _windowed_upright(-SUPPORT_CENTER_X)
    right_upright = _windowed_upright(SUPPORT_CENTER_X)
    top_beam = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)
        .edges("|X")
        .fillet(0.008)
        .translate((0.0, 0.0, BEAM_CENTER_Z))
    )
    upper_beam_rail = (
        cq.Workplane("XY")
        .box(BEAM_RAIL_LENGTH, BEAM_RAIL_DEPTH, BEAM_RAIL_HEIGHT)
        .edges("|X")
        .fillet(0.002)
        .translate((0.0, BEAM_RAIL_CENTER_Y, BEAM_CENTER_Z + BEAM_RAIL_OFFSET_Z))
    )
    lower_beam_rail = (
        cq.Workplane("XY")
        .box(BEAM_RAIL_LENGTH, BEAM_RAIL_DEPTH, BEAM_RAIL_HEIGHT)
        .edges("|X")
        .fillet(0.002)
        .translate((0.0, BEAM_RAIL_CENTER_Y, BEAM_CENTER_Z - BEAM_RAIL_OFFSET_Z))
    )

    frame.visual(
        mesh_from_cadquery(left_foot, "left_foot"),
        material=frame_mat,
        name="left_foot",
    )
    frame.visual(
        mesh_from_cadquery(right_foot, "right_foot"),
        material=frame_mat,
        name="right_foot",
    )
    frame.visual(
        mesh_from_cadquery(left_upright, "left_upright"),
        material=frame_mat,
        name="left_upright",
    )
    frame.visual(
        mesh_from_cadquery(right_upright, "right_upright"),
        material=frame_mat,
        name="right_upright",
    )
    frame.visual(
        mesh_from_cadquery(top_beam, "top_beam"),
        material=frame_mat,
        name="top_beam",
    )
    frame.visual(
        mesh_from_cadquery(upper_beam_rail, "upper_beam_rail"),
        material=rail_mat,
        name="upper_beam_rail",
    )
    frame.visual(
        mesh_from_cadquery(lower_beam_rail, "lower_beam_rail"),
        material=rail_mat,
        name="lower_beam_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, FOOT_DEPTH, BEAM_CENTER_Z + (BEAM_HEIGHT / 2.0))),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, (BEAM_CENTER_Z + (BEAM_HEIGHT / 2.0)) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage_plate = (
        cq.Workplane("XY")
        .box(CARRIAGE_WIDTH, CARRIAGE_PLATE_DEPTH, CARRIAGE_HEIGHT)
        .edges("|Y")
        .fillet(0.006)
        .translate((0.0, CARRIAGE_PLATE_CENTER_Y, 0.0))
    )
    upper_bearing = _box(
        (CARRIAGE_BEARING_WIDTH, CARRIAGE_BEARING_DEPTH, CARRIAGE_BEARING_HEIGHT),
        (0.0, -0.004, CARRIAGE_BEARING_OFFSET_Z),
    )
    lower_bearing = _box(
        (CARRIAGE_BEARING_WIDTH, CARRIAGE_BEARING_DEPTH, CARRIAGE_BEARING_HEIGHT),
        (0.0, -0.004, -CARRIAGE_BEARING_OFFSET_Z),
    )
    carriage_mount = (
        cq.Workplane("XY")
        .box(CARRIAGE_MOUNT_WIDTH, CARRIAGE_MOUNT_DEPTH, CARRIAGE_MOUNT_HEIGHT)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, CARRIAGE_MOUNT_CENTER_Y, CARRIAGE_MOUNT_CENTER_Z))
    )
    carriage_cap = (
        cq.Workplane("XY")
        .box(CARRIAGE_WIDTH, CARRIAGE_CAP_DEPTH, CARRIAGE_CAP_HEIGHT)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.0, CARRIAGE_CAP_CENTER_Y, CARRIAGE_CAP_CENTER_Z))
    )

    carriage.visual(
        mesh_from_cadquery(carriage_plate, "carriage_plate"),
        material=carriage_mat,
        name="carriage_plate",
    )
    carriage.visual(
        mesh_from_cadquery(upper_bearing, "upper_bearing"),
        material=rail_mat,
        name="upper_bearing",
    )
    carriage.visual(
        mesh_from_cadquery(lower_bearing, "lower_bearing"),
        material=rail_mat,
        name="lower_bearing",
    )
    carriage.visual(
        mesh_from_cadquery(carriage_mount, "carriage_mount"),
        material=carriage_mat,
        name="carriage_mount",
    )
    carriage.visual(
        mesh_from_cadquery(carriage_cap, "carriage_cap"),
        material=carriage_mat,
        name="carriage_cap",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, 0.08, CARRIAGE_HEIGHT)),
        mass=5.0,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
    )

    z_slide = model.part("z_slide")
    z_mount_plate = _box(
        (Z_SLIDE_WIDTH, Z_SLIDE_PLATE_DEPTH, Z_SLIDE_HEIGHT),
        (0.0, Z_SLIDE_BODY_CENTER_Y, Z_SLIDE_BODY_CENTER_Z),
    )
    z_top_cap = (
        cq.Workplane("XY")
        .box(Z_SLIDE_WIDTH, Z_TOP_CAP_DEPTH, Z_TOP_CAP_HEIGHT)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, Z_TOP_CAP_CENTER_Y, Z_TOP_CAP_CENTER_Z))
    )
    left_z_cheek = _box(
        (Z_SIDE_CHEEK_WIDTH, Z_SIDE_CHEEK_DEPTH, Z_SIDE_CHEEK_HEIGHT),
        (-Z_SIDE_CHEEK_OFFSET_X, Z_SIDE_CHEEK_CENTER_Y, Z_SIDE_CHEEK_CENTER_Z),
    )
    right_z_cheek = _box(
        (Z_SIDE_CHEEK_WIDTH, Z_SIDE_CHEEK_DEPTH, Z_SIDE_CHEEK_HEIGHT),
        (Z_SIDE_CHEEK_OFFSET_X, Z_SIDE_CHEEK_CENTER_Y, Z_SIDE_CHEEK_CENTER_Z),
    )
    left_z_rail = _box(
        (Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT),
        (-Z_RAIL_OFFSET_X, Z_RAIL_CENTER_Y, Z_RAIL_CENTER_Z),
    )
    right_z_rail = _box(
        (Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT),
        (Z_RAIL_OFFSET_X, Z_RAIL_CENTER_Y, Z_RAIL_CENTER_Z),
    )

    z_slide.visual(
        mesh_from_cadquery(z_mount_plate, "z_mount_plate"),
        material=slide_mat,
        name="z_mount_plate",
    )
    z_slide.visual(
        mesh_from_cadquery(z_top_cap, "z_top_cap"),
        material=slide_mat,
        name="z_top_cap",
    )
    z_slide.visual(
        mesh_from_cadquery(left_z_cheek, "left_z_cheek"),
        material=slide_mat,
        name="left_z_cheek",
    )
    z_slide.visual(
        mesh_from_cadquery(right_z_cheek, "right_z_cheek"),
        material=slide_mat,
        name="right_z_cheek",
    )
    z_slide.visual(
        mesh_from_cadquery(left_z_rail, "left_z_rail"),
        material=rail_mat,
        name="left_z_rail",
    )
    z_slide.visual(
        mesh_from_cadquery(right_z_rail, "right_z_rail"),
        material=rail_mat,
        name="right_z_rail",
    )
    z_slide.inertial = Inertial.from_geometry(
        Box((Z_SLIDE_WIDTH, 0.03, Z_SLIDE_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, -0.015, -0.18)),
    )

    ram = model.part("ram")
    ram_slider = (
        cq.Workplane("XY")
        .box(RAM_SLIDER_WIDTH, RAM_SLIDER_DEPTH, RAM_SLIDER_HEIGHT)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, RAM_SLIDER_CENTER_Y, 0.0))
    )
    left_rail_pad = _box(
        (RAM_PAD_WIDTH, RAM_PAD_DEPTH, RAM_PAD_HEIGHT),
        (-Z_RAIL_OFFSET_X, RAM_PAD_CENTER_Y, 0.0),
    )
    right_rail_pad = _box(
        (RAM_PAD_WIDTH, RAM_PAD_DEPTH, RAM_PAD_HEIGHT),
        (Z_RAIL_OFFSET_X, RAM_PAD_CENTER_Y, 0.0),
    )
    ram_extension = (
        cq.Workplane("XY")
        .box(RAM_EXTENSION_WIDTH, RAM_EXTENSION_DEPTH, RAM_EXTENSION_HEIGHT)
        .edges("|Y")
        .fillet(0.003)
        .translate((0.0, RAM_EXTENSION_CENTER_Y, RAM_EXTENSION_CENTER_Z))
    )
    tool_plate = (
        cq.Workplane("XY")
        .box(TOOL_PLATE_WIDTH, TOOL_PLATE_DEPTH, TOOL_PLATE_HEIGHT)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, TOOL_PLATE_CENTER_Y, TOOL_PLATE_CENTER_Z))
    )

    ram.visual(
        mesh_from_cadquery(ram_slider, "ram_slider"),
        material=ram_mat,
        name="ram_slider",
    )
    ram.visual(
        mesh_from_cadquery(left_rail_pad, "left_rail_pad"),
        material=rail_mat,
        name="left_rail_pad",
    )
    ram.visual(
        mesh_from_cadquery(right_rail_pad, "right_rail_pad"),
        material=rail_mat,
        name="right_rail_pad",
    )
    ram.visual(
        mesh_from_cadquery(ram_extension, "ram_extension"),
        material=ram_mat,
        name="ram_extension",
    )
    ram.visual(
        mesh_from_cadquery(tool_plate, "tool_plate"),
        material=tool_mat,
        name="tool_plate",
    )
    ram.visual(
        Cylinder(radius=SPINDLE_NOSE_RADIUS, length=SPINDLE_NOSE_LENGTH),
        origin=Origin(xyz=(0.0, SPINDLE_NOSE_CENTER_Y, SPINDLE_NOSE_CENTER_Z)),
        material=tool_mat,
        name="spindle_nose",
    )
    ram.inertial = Inertial.from_geometry(
        Box((0.09, 0.07, 0.28)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -0.036, -0.10)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=FRAME_TO_CARRIAGE_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.8,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_z_slide",
        ArticulationType.FIXED,
        parent=carriage,
        child=z_slide,
        origin=Origin(xyz=CARRIAGE_TO_Z_SLIDE_ORIGIN),
    )
    model.articulation(
        "z_slide_to_ram",
        ArticulationType.PRISMATIC,
        parent=z_slide,
        child=ram,
        origin=Origin(xyz=Z_SLIDE_TO_RAM_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.5,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    z_slide = object_model.get_part("z_slide")
    ram = object_model.get_part("ram")

    x_axis = object_model.get_articulation("frame_to_carriage")
    z_mount = object_model.get_articulation("carriage_to_z_slide")
    z_axis = object_model.get_articulation("z_slide_to_ram")

    upper_beam_rail = frame.get_visual("upper_beam_rail")
    lower_beam_rail = frame.get_visual("lower_beam_rail")
    upper_bearing = carriage.get_visual("upper_bearing")
    lower_bearing = carriage.get_visual("lower_bearing")
    carriage_mount = carriage.get_visual("carriage_mount")
    z_mount_plate = z_slide.get_visual("z_mount_plate")
    left_z_rail = z_slide.get_visual("left_z_rail")
    right_z_rail = z_slide.get_visual("right_z_rail")
    left_rail_pad = ram.get_visual("left_rail_pad")
    right_rail_pad = ram.get_visual("right_rail_pad")

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
        "x_axis_is_horizontal_prismatic",
        x_axis.joint_type == ArticulationType.PRISMATIC
        and tuple(x_axis.axis) == (1.0, 0.0, 0.0)
        and x_axis.motion_limits is not None
        and x_axis.motion_limits.lower == -X_TRAVEL
        and x_axis.motion_limits.upper == X_TRAVEL,
        details="Carriage should be the horizontal prismatic beam axis.",
    )
    ctx.check(
        "z_slide_mount_is_fixed",
        z_mount.joint_type == ArticulationType.FIXED,
        details="The outer Z slide should hang rigidly from the carriage.",
    )
    ctx.check(
        "z_axis_is_vertical_prismatic",
        z_axis.joint_type == ArticulationType.PRISMATIC
        and tuple(z_axis.axis) == (0.0, 0.0, -1.0)
        and z_axis.motion_limits is not None
        and z_axis.motion_limits.lower == 0.0
        and z_axis.motion_limits.upper == Z_TRAVEL,
        details="The ram should move vertically on the hanging Z slide.",
    )

    ctx.expect_contact(
        carriage,
        frame,
        elem_a=upper_bearing,
        elem_b=upper_beam_rail,
        name="upper_carriage_bearing_contacts_upper_beam_rail",
    )
    ctx.expect_contact(
        carriage,
        frame,
        elem_a=lower_bearing,
        elem_b=lower_beam_rail,
        name="lower_carriage_bearing_contacts_lower_beam_rail",
    )
    ctx.expect_contact(
        z_slide,
        carriage,
        elem_a=z_mount_plate,
        elem_b=carriage_mount,
        name="z_slide_mount_plate_contacts_carriage_mount",
    )
    ctx.expect_contact(
        ram,
        z_slide,
        elem_a=left_rail_pad,
        elem_b=left_z_rail,
        name="left_ram_pad_contacts_left_z_rail",
    )
    ctx.expect_contact(
        ram,
        z_slide,
        elem_a=right_rail_pad,
        elem_b=right_z_rail,
        name="right_ram_pad_contacts_right_z_rail",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        elem_a=upper_bearing,
        elem_b=upper_beam_rail,
        axes="x",
        min_overlap=0.14,
        name="carriage_has_horizontal_guide_engagement",
    )
    ctx.expect_overlap(
        ram,
        z_slide,
        elem_a=left_rail_pad,
        elem_b=left_z_rail,
        axes="z",
        min_overlap=0.099,
        name="ram_has_vertical_guide_engagement",
    )
    ctx.expect_origin_gap(
        carriage,
        z_slide,
        axis="z",
        min_gap=0.015,
        max_gap=0.03,
        name="z_slide_hangs_below_carriage_origin",
    )

    with ctx.pose({x_axis: -X_TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=upper_bearing,
            elem_b=upper_beam_rail,
            name="left_travel_upper_bearing_stays_on_upper_rail",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=lower_bearing,
            elem_b=lower_beam_rail,
            name="left_travel_lower_bearing_stays_on_lower_rail",
        )

    with ctx.pose({x_axis: X_TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=upper_bearing,
            elem_b=upper_beam_rail,
            name="right_travel_upper_bearing_stays_on_upper_rail",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=lower_bearing,
            elem_b=lower_beam_rail,
            name="right_travel_lower_bearing_stays_on_lower_rail",
        )

    with ctx.pose({z_axis: Z_TRAVEL}):
        ctx.expect_contact(
            ram,
            z_slide,
            elem_a=left_rail_pad,
            elem_b=left_z_rail,
            name="full_drop_left_ram_pad_stays_on_left_z_rail",
        )
        ctx.expect_contact(
            ram,
            z_slide,
            elem_a=right_rail_pad,
            elem_b=right_z_rail,
            name="full_drop_right_ram_pad_stays_on_right_z_rail",
        )
        ctx.expect_overlap(
            ram,
            z_slide,
            elem_a=left_rail_pad,
            elem_b=left_z_rail,
            axes="z",
            min_overlap=0.099,
            name="full_drop_retains_vertical_rail_overlap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
