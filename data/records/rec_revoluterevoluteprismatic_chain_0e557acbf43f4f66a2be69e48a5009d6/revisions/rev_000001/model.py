from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.006

BRACKET_CHEEK_Y = 0.020
LINK1_PLATE_Y = 0.026
LINK2_PLATE_Y = 0.014

BRACKET_WALL_T = 0.010
BRACKET_W = 0.120
BRACKET_H = 0.220

LINK1_LEN = 0.240
LINK1_H = 0.056

LINK2_LEN = 0.290
LINK2_H = 0.050

SLIDE_ORIGIN_X = 0.145
SLIDE_STROKE = 0.030

ROOT_LOWER = -0.70
ROOT_UPPER = 0.60
ELBOW_LOWER = -1.95
ELBOW_UPPER = 0.10


def box_at(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - length / 2.0, center[2]))
    )


def union_all(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def plate_box(
    length: float,
    height: float,
    thickness: float,
    center: tuple[float, float, float],
    *,
    window_length: float | None = None,
    window_height: float | None = None,
) -> cq.Workplane:
    plate = box_at(length, thickness, height, center)
    if window_length is not None and window_height is not None:
        plate = plate.cut(
            box_at(
                window_length,
                thickness + 0.002,
                window_height,
                center,
            )
        )
    return plate


def make_wall_bracket_shape() -> cq.Workplane:
    wall_plate = box_at(BRACKET_WALL_T, BRACKET_W, BRACKET_H, (-0.035, 0.0, 0.0))
    center_spine = box_at(0.018, 0.040, 0.122, (-0.020, 0.0, 0.0))
    upper_web = box_at(0.022, 0.070, 0.014, (-0.018, 0.0, 0.044))
    lower_web = box_at(0.022, 0.070, 0.014, (-0.018, 0.0, -0.044))

    cheek_right = box_at(0.020, PLATE_T, 0.066, (0.004, BRACKET_CHEEK_Y, 0.0))
    cheek_left = box_at(0.020, PLATE_T, 0.066, (0.004, -BRACKET_CHEEK_Y, 0.0))
    cheek_root_right = box_at(0.008, 0.010, 0.020, (-0.004, BRACKET_CHEEK_Y - 0.002, 0.0))
    cheek_root_left = box_at(0.008, 0.010, 0.020, (-0.004, -BRACKET_CHEEK_Y + 0.002, 0.0))

    upper_stop_right = box_at(0.006, 0.006, 0.012, (0.011, BRACKET_CHEEK_Y + 0.006, 0.024))
    upper_stop_left = box_at(0.006, 0.006, 0.012, (0.011, -BRACKET_CHEEK_Y - 0.006, 0.024))
    lower_stop_right = box_at(0.006, 0.006, 0.012, (0.011, BRACKET_CHEEK_Y + 0.006, -0.024))
    lower_stop_left = box_at(0.006, 0.006, 0.012, (0.011, -BRACKET_CHEEK_Y - 0.006, -0.024))

    bracket = union_all(
        wall_plate,
        center_spine,
        upper_web,
        lower_web,
        cheek_right,
        cheek_left,
        cheek_root_right,
        cheek_root_left,
        upper_stop_right,
        upper_stop_left,
        lower_stop_right,
        lower_stop_left,
    )

    for y_pos in (-0.034, 0.034):
        for z_pos in (-0.064, 0.064):
            bracket = bracket.cut(x_cylinder(0.005, BRACKET_WALL_T + 0.004, (-0.035, y_pos, z_pos)))

    return bracket


def make_link1_shape() -> cq.Workplane:
    right_plate = plate_box(
        0.188,
        0.050,
        PLATE_T,
        (0.124, LINK1_PLATE_Y, 0.0),
        window_length=0.106,
        window_height=0.022,
    )
    left_plate = plate_box(
        0.188,
        0.050,
        PLATE_T,
        (0.124, -LINK1_PLATE_Y, 0.0),
        window_length=0.106,
        window_height=0.022,
    )

    root_knuckle = y_cylinder(0.012, 0.034, (0.0, 0.0, 0.0))
    root_bridge = box_at(0.014, 0.018, 0.014, (0.015, 0.0, 0.0))
    shoulder_web = box_at(0.024, 0.036, 0.016, (0.036, 0.0, 0.0))

    mid_top_bridge = box_at(0.026, 0.058, 0.006, (0.090, 0.0, 0.022))
    mid_bottom_bridge = box_at(0.026, 0.058, 0.006, (0.090, 0.0, -0.022))
    elbow_cheek_right = box_at(0.034, PLATE_T, 0.040, (LINK1_LEN - 0.014, LINK1_PLATE_Y, 0.0))
    elbow_cheek_left = box_at(0.034, PLATE_T, 0.040, (LINK1_LEN - 0.014, -LINK1_PLATE_Y, 0.0))
    elbow_top_tie = box_at(0.012, 0.058, 0.006, (LINK1_LEN - 0.026, 0.0, 0.019))
    elbow_bottom_tie = box_at(0.012, 0.058, 0.006, (LINK1_LEN - 0.026, 0.0, -0.019))

    top_flange_right = box_at(0.122, 0.003, 0.006, (0.146, LINK1_PLATE_Y + 0.0045, 0.022))
    bottom_flange_right = box_at(0.122, 0.003, 0.006, (0.146, LINK1_PLATE_Y + 0.0045, -0.022))
    top_flange_left = box_at(0.122, 0.003, 0.006, (0.146, -LINK1_PLATE_Y - 0.0045, 0.022))
    bottom_flange_left = box_at(0.122, 0.003, 0.006, (0.146, -LINK1_PLATE_Y - 0.0045, -0.022))

    return union_all(
        right_plate,
        left_plate,
        root_knuckle,
        root_bridge,
        shoulder_web,
        mid_top_bridge,
        mid_bottom_bridge,
        elbow_cheek_right,
        elbow_cheek_left,
        elbow_top_tie,
        elbow_bottom_tie,
        top_flange_right,
        bottom_flange_right,
        top_flange_left,
        bottom_flange_left,
    )


def make_link2_shape() -> cq.Workplane:
    elbow_knuckle = y_cylinder(0.011, 0.034, (0.0, 0.0, 0.0))
    elbow_bridge = box_at(0.014, 0.018, 0.014, (0.015, 0.0, 0.0))
    shoulder_web = box_at(0.022, 0.030, 0.016, (0.036, 0.0, 0.0))

    right_rail = plate_box(0.210, 0.040, 0.005, (0.160, 0.018, 0.0), window_length=0.130, window_height=0.018)
    left_rail = plate_box(0.210, 0.040, 0.005, (0.160, -0.018, 0.0), window_length=0.130, window_height=0.018)

    upper_front_bridge = box_at(0.018, 0.028, 0.006, (0.064, 0.0, 0.018))
    lower_front_bridge = box_at(0.018, 0.028, 0.006, (0.064, 0.0, -0.018))
    top_flange_right = box_at(0.145, 0.003, 0.005, (0.182, 0.0145, 0.0185))
    bottom_flange_right = box_at(0.145, 0.003, 0.005, (0.182, 0.0145, -0.0185))
    top_flange_left = box_at(0.145, 0.003, 0.005, (0.182, -0.0145, 0.0185))
    bottom_flange_left = box_at(0.145, 0.003, 0.005, (0.182, -0.0145, -0.0185))

    stop_tab_front_right = box_at(0.008, 0.004, 0.012, (0.262, 0.023, 0.0))
    stop_tab_front_left = box_at(0.008, 0.004, 0.012, (0.262, -0.023, 0.0))
    stop_tab_rear_right = box_at(0.008, 0.004, 0.012, (0.100, 0.023, 0.0))
    stop_tab_rear_left = box_at(0.008, 0.004, 0.012, (0.100, -0.023, 0.0))

    return union_all(
        elbow_knuckle,
        elbow_bridge,
        shoulder_web,
        right_rail,
        left_rail,
        upper_front_bridge,
        lower_front_bridge,
        top_flange_right,
        bottom_flange_right,
        top_flange_left,
        bottom_flange_left,
        stop_tab_front_right,
        stop_tab_front_left,
        stop_tab_rear_right,
        stop_tab_rear_left,
    )


def make_carriage_shape() -> cq.Workplane:
    rear_shoe = box_at(0.050, 0.010, 0.018, (-0.020, 0.0, 0.0))
    center_beam = box_at(0.050, 0.008, 0.016, (0.015, 0.0, 0.0))
    nose_block = box_at(0.070, 0.012, 0.028, (0.060, 0.0, 0.0))
    front_cap = box_at(0.010, 0.016, 0.032, (0.100, 0.0, 0.0))
    upper_slide_pad = box_at(0.060, 0.010, 0.004, (0.010, 0.0, 0.013))
    lower_slide_pad = box_at(0.060, 0.010, 0.004, (0.010, 0.0, -0.013))

    pads = [
        box_at(0.020, 0.004, 0.012, (-0.022, 0.0135, 0.0)),
        box_at(0.020, 0.004, 0.012, (-0.022, -0.0135, 0.0)),
        box_at(0.020, 0.004, 0.012, (0.012, 0.0135, 0.0)),
        box_at(0.020, 0.004, 0.012, (0.012, -0.0135, 0.0)),
    ]

    return union_all(rear_shoe, center_beam, nose_block, front_cap, upper_slide_pad, lower_slide_pad, *pads)


def vec_close(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_folding_arm")

    bracket_mat = model.material("bracket_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    link_mat = model.material("link_finish", rgba=(0.58, 0.61, 0.64, 1.0))
    carriage_mat = model.material("carriage_finish", rgba=(0.23, 0.25, 0.28, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(make_wall_bracket_shape(), "wall_bracket"),
        material=bracket_mat,
        name="bracket_shell",
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(make_link1_shape(), "proximal_link"),
        material=link_mat,
        name="proximal_link_shell",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(make_link2_shape(), "distal_link"),
        material=link_mat,
        name="distal_link_shell",
    )

    nose_carriage = model.part("nose_carriage")
    nose_carriage.visual(
        mesh_from_cadquery(make_carriage_shape(), "nose_carriage"),
        material=carriage_mat,
        name="nose_carriage_shell",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=proximal_link,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.6,
            lower=ROOT_LOWER,
            upper=ROOT_UPPER,
        ),
    )

    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.8,
            lower=ELBOW_LOWER,
            upper=ELBOW_UPPER,
        ),
    )

    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=distal_link,
        child=nose_carriage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    nose_carriage = object_model.get_part("nose_carriage")

    root_hinge = object_model.get_articulation("root_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    nose_slide = object_model.get_articulation("nose_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0035)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        wall_bracket,
        proximal_link,
        reason="root hinge tongue nests between the wall bracket clevis cheeks around the pivot envelope",
    )
    ctx.allow_overlap(
        proximal_link,
        distal_link,
        reason="elbow hinge tongue nests within the proximal clevis around the pivot envelope",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "root hinge axis",
        vec_close(root_hinge.axis, (0.0, 1.0, 0.0)),
        f"expected root hinge axis (0, 1, 0), got {root_hinge.axis}",
    )
    ctx.check(
        "elbow hinge axis",
        vec_close(elbow_hinge.axis, (0.0, 1.0, 0.0)),
        f"expected elbow hinge axis (0, 1, 0), got {elbow_hinge.axis}",
    )
    ctx.check(
        "nose slide axis",
        vec_close(nose_slide.axis, (1.0, 0.0, 0.0)),
        f"expected nose slide axis (1, 0, 0), got {nose_slide.axis}",
    )

    ctx.expect_contact(proximal_link, wall_bracket, name="root pivot stack is supported")
    ctx.expect_contact(distal_link, proximal_link, name="elbow pivot stack is supported")
    ctx.expect_contact(
        nose_carriage,
        distal_link,
        contact_tol=0.0035,
        name="nose carriage rides on distal link",
    )

    ctx.expect_within(
        nose_carriage,
        distal_link,
        axes="yz",
        margin=0.001,
        name="carriage stays inside rail envelope retracted",
    )
    ctx.expect_overlap(
        nose_carriage,
        distal_link,
        axes="x",
        min_overlap=0.10,
        name="carriage retains engagement retracted",
    )

    with ctx.pose({nose_slide: SLIDE_STROKE}):
        ctx.expect_within(
            nose_carriage,
            distal_link,
            axes="yz",
            margin=0.001,
            name="carriage stays inside rail envelope extended",
        )
        ctx.expect_overlap(
            nose_carriage,
            distal_link,
            axes="x",
            min_overlap=0.11,
            name="carriage retains engagement extended",
        )

    with ctx.pose({root_hinge: 0.55, elbow_hinge: -1.55, nose_slide: SLIDE_STROKE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no interpenetration in compact folded pose")

    with ctx.pose({root_hinge: -0.55, elbow_hinge: -0.55, nose_slide: SLIDE_STROKE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no interpenetration in lowered reach pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
