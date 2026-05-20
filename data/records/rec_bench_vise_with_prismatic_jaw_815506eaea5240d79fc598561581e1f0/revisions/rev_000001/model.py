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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _dovetail_way_mesh(*, length: float, bottom_width: float, top_width: float, height: float, name: str):
    half_length = length * 0.5
    section_a = [
        (-half_length, -bottom_width * 0.5, 0.0),
        (-half_length, bottom_width * 0.5, 0.0),
        (-half_length, top_width * 0.5, height),
        (-half_length, -top_width * 0.5, height),
    ]
    section_b = [
        (half_length, -bottom_width * 0.5, 0.0),
        (half_length, bottom_width * 0.5, 0.0),
        (half_length, top_width * 0.5, height),
        (half_length, -top_width * 0.5, height),
    ]
    return mesh_from_geometry(section_loft([section_a, section_b]), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="welding_positioner_vise")

    base_paint = model.material("base_paint", rgba=(0.19, 0.20, 0.22, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.40, 0.43, 0.46, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))

    x_cyl = (0.0, math.pi / 2.0, 0.0)
    y_cyl = (math.pi / 2.0, 0.0, 0.0)

    left_way = _dovetail_way_mesh(
        length=0.180,
        bottom_width=0.024,
        top_width=0.012,
        height=0.010,
        name="left_dovetail_way",
    )
    right_way = _dovetail_way_mesh(
        length=0.180,
        bottom_width=0.024,
        top_width=0.012,
        height=0.010,
        name="right_dovetail_way",
    )

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.340, 0.260, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=base_paint,
        name="plate",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((0.340, 0.260, 0.016)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.072, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=machine_gray,
        name="bearing_drum",
    )
    turntable.visual(
        Cylinder(radius=0.110, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=jaw_steel,
        name="rotary_disc",
    )
    turntable.visual(
        Box((0.180, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=machine_gray,
        name="vise_mount",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.068),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
    )

    model.articulation(
        "table_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0),
    )

    vise_bed = model.part("vise_bed")
    vise_bed.visual(
        Box((0.280, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=machine_gray,
        name="bed_body",
    )
    vise_bed.visual(
        left_way,
        origin=Origin(xyz=(0.020, 0.030, 0.020)),
        material=jaw_steel,
        name="left_way",
    )
    vise_bed.visual(
        right_way,
        origin=Origin(xyz=(0.020, -0.030, 0.020)),
        material=jaw_steel,
        name="right_way",
    )
    vise_bed.visual(
        Box((0.018, 0.066, 0.020)),
        origin=Origin(xyz=(0.128, 0.0, 0.010)),
        material=machine_gray,
        name="front_end_block",
    )
    vise_bed.inertial = Inertial.from_geometry(
        Box((0.280, 0.110, 0.032)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    model.articulation(
        "turntable_to_bed",
        ArticulationType.FIXED,
        parent=turntable,
        child=vise_bed,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
    )

    fixed_jaw = model.part("fixed_jaw")
    fixed_jaw.visual(
        Box((0.040, 0.100, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=machine_gray,
        name="pedestal",
    )
    fixed_jaw.visual(
        Box((0.040, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.035, 0.022)),
        material=machine_gray,
        name="left_web",
    )
    fixed_jaw.visual(
        Box((0.040, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.035, 0.022)),
        material=machine_gray,
        name="right_web",
    )
    fixed_jaw.visual(
        Box((0.040, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=machine_gray,
        name="jaw_body",
    )
    fixed_jaw.visual(
        Box((0.006, 0.094, 0.026)),
        origin=Origin(xyz=(0.023, 0.0, 0.047)),
        material=jaw_steel,
        name="jaw_face",
    )
    fixed_jaw.inertial = Inertial.from_geometry(
        Box((0.046, 0.100, 0.060)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "bed_to_fixed_jaw",
        ArticulationType.FIXED,
        parent=vise_bed,
        child=fixed_jaw,
        origin=Origin(xyz=(-0.094, 0.0, 0.020)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.060, 0.020, 0.010)),
        origin=Origin(xyz=(0.006, 0.029, 0.005)),
        material=jaw_steel,
        name="left_shoe",
    )
    moving_jaw.visual(
        Box((0.060, 0.020, 0.010)),
        origin=Origin(xyz=(0.006, -0.029, 0.005)),
        material=jaw_steel,
        name="right_shoe",
    )
    moving_jaw.visual(
        Box((0.044, 0.018, 0.014)),
        origin=Origin(xyz=(0.014, 0.028, 0.017)),
        material=machine_gray,
        name="left_carriage_wall",
    )
    moving_jaw.visual(
        Box((0.044, 0.018, 0.014)),
        origin=Origin(xyz=(0.014, -0.028, 0.017)),
        material=machine_gray,
        name="right_carriage_wall",
    )
    moving_jaw.visual(
        Box((0.018, 0.028, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, 0.008)),
        material=machine_gray,
        name="nut_lower_bridge",
    )
    moving_jaw.visual(
        Box((0.018, 0.006, 0.014)),
        origin=Origin(xyz=(0.020, 0.011, 0.017)),
        material=machine_gray,
        name="nut_left_plate",
    )
    moving_jaw.visual(
        Box((0.018, 0.006, 0.014)),
        origin=Origin(xyz=(0.020, -0.011, 0.017)),
        material=machine_gray,
        name="nut_right_plate",
    )
    moving_jaw.visual(
        Box((0.018, 0.028, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.029)),
        material=machine_gray,
        name="nut_upper_bridge",
    )
    moving_jaw.visual(
        Box((0.034, 0.094, 0.042)),
        origin=Origin(xyz=(-0.004, 0.0, 0.045)),
        material=machine_gray,
        name="jaw_body",
    )
    moving_jaw.visual(
        Box((0.006, 0.090, 0.032)),
        origin=Origin(xyz=(-0.023, 0.0, 0.043)),
        material=jaw_steel,
        name="jaw_face",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.066, 0.094, 0.068)),
        mass=1.6,
        origin=Origin(xyz=(0.004, 0.0, 0.034)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=vise_bed,
        child=moving_jaw,
        origin=Origin(xyz=(-0.042, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.08,
            lower=0.0,
            upper=0.090,
        ),
    )

    leadscrew = model.part("leadscrew")
    leadscrew.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=x_cyl),
        material=screw_steel,
        name="thrust_collar",
    )
    leadscrew.visual(
        Cylinder(radius=0.0055, length=0.204),
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=x_cyl),
        material=screw_steel,
        name="screw_shaft",
    )
    leadscrew.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=x_cyl),
        material=screw_steel,
        name="handle_hub",
    )
    leadscrew.visual(
        Cylinder(radius=0.0035, length=0.120),
        origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=y_cyl),
        material=handle_black,
        name="bar_handle",
    )
    leadscrew.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.210, 0.060, 0.0), rpy=y_cyl),
        material=handle_black,
        name="handle_knob_pos",
    )
    leadscrew.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.210, -0.060, 0.0), rpy=y_cyl),
        material=handle_black,
        name="handle_knob_neg",
    )
    leadscrew.inertial = Inertial.from_geometry(
        Box((0.230, 0.135, 0.030)),
        mass=0.7,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
    )

    model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=fixed_jaw,
        child=leadscrew,
        origin=Origin(xyz=(0.020, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_plate = object_model.get_part("base_plate")
    turntable = object_model.get_part("turntable")
    vise_bed = object_model.get_part("vise_bed")
    fixed_jaw = object_model.get_part("fixed_jaw")
    moving_jaw = object_model.get_part("moving_jaw")
    leadscrew = object_model.get_part("leadscrew")

    table_swivel = object_model.get_articulation("table_swivel")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("screw_spin")

    fixed_face = fixed_jaw.get_visual("jaw_face")
    moving_face = moving_jaw.get_visual("jaw_face")

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

    ctx.expect_contact(turntable, base_plate)
    ctx.expect_contact(vise_bed, turntable)
    ctx.expect_contact(fixed_jaw, vise_bed)
    ctx.expect_contact(moving_jaw, vise_bed)
    ctx.expect_contact(leadscrew, fixed_jaw)

    ctx.expect_within(turntable, base_plate, axes="xy", margin=0.0)
    ctx.expect_overlap(vise_bed, turntable, axes="xy", min_overlap=0.110)
    ctx.expect_overlap(
        moving_jaw,
        fixed_jaw,
        axes="y",
        min_overlap=0.085,
        elem_a=moving_face,
        elem_b=fixed_face,
        name="jaw_faces_align_across_width",
    )
    ctx.expect_overlap(
        moving_jaw,
        fixed_jaw,
        axes="z",
        min_overlap=0.020,
        elem_a=moving_face,
        elem_b=fixed_face,
        name="jaw_faces_overlap_in_height",
    )
    ctx.expect_gap(
        moving_jaw,
        fixed_jaw,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=moving_face,
        negative_elem=fixed_face,
        name="closed_jaws_meet_without_overlap",
    )

    ctx.check(
        "swivel_axis_is_vertical",
        tuple(table_swivel.axis) == (0.0, 0.0, 1.0),
        f"table_swivel axis was {table_swivel.axis}",
    )
    ctx.check(
        "jaw_slide_axis_is_longitudinal",
        abs(abs(jaw_slide.axis[0]) - 1.0) < 1e-9 and abs(jaw_slide.axis[1]) < 1e-9 and abs(jaw_slide.axis[2]) < 1e-9,
        f"jaw_slide axis was {jaw_slide.axis}",
    )
    ctx.check(
        "screw_spin_axis_is_longitudinal",
        abs(abs(screw_spin.axis[0]) - 1.0) < 1e-9 and abs(screw_spin.axis[1]) < 1e-9 and abs(screw_spin.axis[2]) < 1e-9,
        f"screw_spin axis was {screw_spin.axis}",
    )

    jaw_closed = ctx.part_world_position(moving_jaw)
    with ctx.pose({jaw_slide: 0.090}):
        jaw_open = ctx.part_world_position(moving_jaw)
        ctx.expect_contact(moving_jaw, vise_bed)
        ctx.expect_gap(
            moving_jaw,
            fixed_jaw,
            axis="x",
            min_gap=0.089,
            max_gap=0.091,
            positive_elem=moving_face,
            negative_elem=fixed_face,
            name="open_jaw_gap_matches_travel",
        )
    if jaw_closed is not None and jaw_open is not None:
        ctx.check(
            "moving_jaw_translates_along_x",
            jaw_open[0] > jaw_closed[0] + 0.085
            and abs(jaw_open[1] - jaw_closed[1]) < 1e-6
            and abs(jaw_open[2] - jaw_closed[2]) < 1e-6,
            f"closed={jaw_closed}, open={jaw_open}",
        )

    fixed_rest = ctx.part_world_position(fixed_jaw)
    with ctx.pose({table_swivel: math.pi / 2.0}):
        fixed_swiveled = ctx.part_world_position(fixed_jaw)
        ctx.expect_contact(turntable, base_plate)
        ctx.expect_contact(vise_bed, turntable)
    if fixed_rest is not None and fixed_swiveled is not None:
        ctx.check(
            "turntable_rotates_upper_vise_about_center",
            abs(fixed_swiveled[0]) < 0.006
            and abs(abs(fixed_swiveled[1]) - abs(fixed_rest[0])) < 0.006
            and abs(fixed_swiveled[2] - fixed_rest[2]) < 1e-6,
            f"rest={fixed_rest}, swiveled={fixed_swiveled}",
        )

    screw_rest_aabb = ctx.part_world_aabb(leadscrew)
    with ctx.pose({screw_spin: math.pi / 2.0}):
        screw_turned_aabb = ctx.part_world_aabb(leadscrew)
        ctx.expect_contact(leadscrew, fixed_jaw)
    if screw_rest_aabb is not None and screw_turned_aabb is not None:
        rest_y = screw_rest_aabb[1][1] - screw_rest_aabb[0][1]
        rest_z = screw_rest_aabb[1][2] - screw_rest_aabb[0][2]
        turned_y = screw_turned_aabb[1][1] - screw_turned_aabb[0][1]
        turned_z = screw_turned_aabb[1][2] - screw_turned_aabb[0][2]
        ctx.check(
            "bar_handle_rotates_with_leadscrew",
            rest_y > rest_z + 0.08 and turned_z > turned_y + 0.08,
            f"rest_yz=({rest_y:.4f}, {rest_z:.4f}), turned_yz=({turned_y:.4f}, {turned_z:.4f})",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
