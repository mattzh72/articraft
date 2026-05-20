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


BACK_W = 0.13
BACK_H = 0.42
PLATE_T = 0.008
RAIL_W = 0.040
RAIL_H = 0.30
RAIL_D = 0.026
STOP_W = 0.060
STOP_D = 0.018
STOP_H = 0.016

SLIDE_TRAVEL = 0.18

CAR_W = 0.11
CAR_H = 0.11
CAR_D = 0.046
CAR_FRONT_OFFSET = (CAR_D - RAIL_D) / 2.0
HINGE_R = 0.010
EAR_W = 0.012
LUG_W = 0.024
EAR_X = (LUG_W + EAR_W) / 2.0
RIB_W = 0.010
HINGE_Y = 0.056
HINGE_Z = 0.032

TAB_W = 0.11
TAB_H = 0.10
TAB_T = 0.008


def _x_axis_cylinder(
    *, x_center: float, y_center: float, z_center: float, radius: float, length: float
):
    return (
        cq.Workplane("YZ")
        .center(y_center, z_center)
        .circle(radius)
        .extrude(length)
        .translate((x_center - (length / 2.0), 0.0, 0.0))
    )


def _make_backplate_shape():
    plate = (
        cq.Workplane("XY")
        .box(BACK_W, PLATE_T, BACK_H)
        .edges("|Y")
        .fillet(0.010)
    )

    slot_cutter = (
        cq.Workplane("XZ")
        .pushPoints([(0.0, -0.14), (0.0, 0.14)])
        .slot2D(0.055, 0.012, 90)
        .extrude(PLATE_T * 3.0)
        .translate((0.0, -1.5 * PLATE_T, 0.0))
    )
    plate = plate.cut(slot_cutter)

    rail = cq.Workplane("XY").box(RAIL_W, RAIL_D, RAIL_H).translate(
        (0.0, (PLATE_T / 2.0) + (RAIL_D / 2.0), 0.0)
    )
    top_stop = cq.Workplane("XY").box(STOP_W, STOP_D, STOP_H).translate(
        (0.0, (PLATE_T / 2.0) + (STOP_D / 2.0), (RAIL_H / 2.0) + (STOP_H / 2.0))
    )
    bottom_stop = cq.Workplane("XY").box(STOP_W, STOP_D, STOP_H).translate(
        (0.0, (PLATE_T / 2.0) + (STOP_D / 2.0), -((RAIL_H / 2.0) + (STOP_H / 2.0)))
    )

    return plate.union(rail).union(top_stop).union(bottom_stop)


def _make_carriage_shape():
    lower_block = (
        cq.Workplane("XY")
        .box(CAR_W, 0.018, 0.070)
        .translate((0.0, 0.040, -0.015))
        .edges("|Y")
        .fillet(0.007)
    )

    shoe_w = 0.016
    shoe_center_x = (RAIL_W / 2.0) + (shoe_w / 2.0)
    left_shoe = cq.Workplane("XY").box(shoe_w, 0.024, 0.092).translate(
        (-shoe_center_x, 0.020, -0.006)
    )
    right_shoe = cq.Workplane("XY").box(shoe_w, 0.024, 0.092).translate(
        (shoe_center_x, 0.020, -0.006)
    )

    cheek_depth = 0.020
    cheek_height = 0.040
    left_cheek = cq.Workplane("XY").box(EAR_W, cheek_depth, cheek_height).translate(
        (-EAR_X, HINGE_Y, HINGE_Z - 0.002)
    )
    right_cheek = cq.Workplane("XY").box(EAR_W, cheek_depth, cheek_height).translate(
        (EAR_X, HINGE_Y, HINGE_Z - 0.002)
    )

    return lower_block.union(left_shoe).union(right_shoe).union(left_cheek).union(right_cheek)


def _make_output_tab_shape():
    lug = _x_axis_cylinder(
        x_center=0.0,
        y_center=0.0,
        z_center=0.0,
        radius=HINGE_R,
        length=LUG_W,
    )

    neck = cq.Workplane("XY").box(0.018, 0.018, 0.024).translate((0.0, 0.014, -0.004))
    stem = cq.Workplane("XY").box(0.022, 0.022, 0.050).translate((0.0, 0.028, -0.024))
    plate = (
        cq.Workplane("XY")
        .box(TAB_W, TAB_T, TAB_H)
        .translate((0.0, 0.042, -0.040))
        .edges("|Y")
        .fillet(0.008)
    )

    plate_holes = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-0.030, -0.048),
                (0.030, -0.048),
                (-0.030, 0.008),
                (0.030, 0.008),
            ]
        )
        .circle(0.004)
        .extrude(TAB_T * 3.0)
        .translate((0.0, 0.042 - (1.5 * TAB_T), -0.040))
    )
    plate = plate.cut(plate_holes)

    return lug.union(neck).union(stem).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_vertical_tilt_axis")

    steel = model.material("steel_gray", rgba=(0.62, 0.65, 0.69, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_make_backplate_shape(), "backplate"),
        material=steel,
        name="backplate_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage"),
        material=graphite,
        name="carriage_body",
    )

    output_tab = model.part("output_tab")
    output_tab.visual(
        mesh_from_cadquery(_make_output_tab_shape(), "output_tab"),
        material=matte_black,
        name="output_tab_body",
    )

    model.articulation(
        "backplate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(0.0, (PLATE_T / 2.0) + (RAIL_D / 2.0), -(SLIDE_TRAVEL / 2.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_output_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_tab,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    carriage = object_model.get_part("carriage")
    output_tab = object_model.get_part("output_tab")
    slide = object_model.get_articulation("backplate_to_carriage")
    tilt = object_model.get_articulation("carriage_to_output_tab")

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
    ctx.allow_overlap(
        carriage,
        output_tab,
        reason=(
            "The revolute output tab is modeled as a tight clevis-and-lug hinge. "
            "Because the pin and running clearance are omitted, the nested hinge "
            "knuckles intentionally share a small amount of volume at the mount."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "primary_parts_present",
        all(part is not None for part in (backplate, carriage, output_tab)),
        "Expected backplate, carriage, and output tab parts.",
    )
    ctx.check(
        "joint_layout_matches_prompt",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt.axis) == (1.0, 0.0, 0.0),
        "Expected a vertical prismatic carriage and a horizontal tilt hinge.",
    )

    ctx.expect_contact(carriage, backplate, name="carriage_contacts_guide")
    ctx.expect_contact(output_tab, carriage, name="output_tab_contacts_hinge_mount")

    with ctx.pose({slide: slide.motion_limits.lower}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        high_pos = ctx.part_world_position(carriage)

    slide_delta = None
    if low_pos is not None and high_pos is not None:
        slide_delta = high_pos[2] - low_pos[2]
    ctx.check(
        "carriage_moves_upward_on_slide",
        slide_delta is not None and slide_delta > 0.17,
        f"Expected upward prismatic travel of about 0.18 m, got {slide_delta!r}.",
    )

    with ctx.pose({tilt: 0.0}):
        closed_aabb = ctx.part_world_aabb(output_tab)
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        open_aabb = ctx.part_world_aabb(output_tab)

    closed_max_y = None if closed_aabb is None else closed_aabb[1][1]
    open_max_y = None if open_aabb is None else open_aabb[1][1]
    ctx.check(
        "output_tab_tilts_forward",
        closed_max_y is not None
        and open_max_y is not None
        and open_max_y > closed_max_y + 0.03,
        f"Expected forward tilt to increase output tab reach, got closed/open max y "
        f"{closed_max_y!r}/{open_max_y!r}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
