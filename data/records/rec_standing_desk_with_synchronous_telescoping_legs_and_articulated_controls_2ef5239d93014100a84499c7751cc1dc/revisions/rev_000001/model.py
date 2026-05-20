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


TOP_WIDTH = 1.10
TOP_DEPTH = 0.55
TOP_THICKNESS = 0.028
TOP_CORNER_RADIUS = 0.03

LEG_CENTER_X = 0.38

UNDERFRAME_PLATE_X = 0.20
UNDERFRAME_PLATE_Y = 0.11
UNDERFRAME_PLATE_T = 0.018
UNDERFRAME_BEAM_X = 0.60
UNDERFRAME_BEAM_Y = 0.07
UNDERFRAME_BEAM_Z = 0.052

OUTER_PLATE_X = 0.11
OUTER_PLATE_Y = 0.085
OUTER_PLATE_T = 0.012
OUTER_SLEEVE_X = 0.090
OUTER_SLEEVE_Y = 0.060
OUTER_SLEEVE_Z = 0.520
OUTER_WALL = 0.004

INNER_STAGE_X = 0.078
INNER_STAGE_Y = 0.048
INNER_STAGE_UP = 0.400
INNER_STAGE_DOWN = 0.180
INNER_STAGE_Z = INNER_STAGE_UP + INNER_STAGE_DOWN
INNER_STAGE_CENTER_Z = (INNER_STAGE_UP - INNER_STAGE_DOWN) / 2.0
GUIDE_PAD_PROUD = 0.002
GUIDE_PAD_OVERLAP = 0.0005
GUIDE_PAD_THICK = GUIDE_PAD_PROUD + GUIDE_PAD_OVERLAP
GUIDE_PAD_SIDE_SPAN = 0.016
GUIDE_PAD_FRONT_SPAN = 0.022
GUIDE_PAD_HEIGHT = 0.160
GUIDE_PAD_CENTER_Z = 0.250

FOOT_MOUNT_X = 0.120
FOOT_MOUNT_Y = 0.090
FOOT_MOUNT_Z = 0.018
FOOT_BEAM_X = 0.090
FOOT_BEAM_Y = 0.620
FOOT_BEAM_Z = 0.028
GLIDE_RADIUS = 0.015
GLIDE_Z = 0.008
GLIDE_OFFSET_Y = 0.230

LEG_TRAVEL = 0.280
OUTER_TO_INNER_ORIGIN_Z = -OUTER_PLATE_T - OUTER_SLEEVE_Z

PANEL_FLANGE_X = 0.160
PANEL_FLANGE_Y = 0.024
PANEL_FLANGE_Z = 0.004
PANEL_NECK_X = 0.136
PANEL_NECK_Y = 0.012
PANEL_NECK_Z = 0.014
PANEL_BODY_X = 0.150
PANEL_BODY_Y = 0.052
PANEL_BODY_Z = 0.022
PANEL_BODY_CENTER_Y = -0.037
PANEL_BODY_CENTER_Z = -0.024
PANEL_FRONT_Y = PANEL_BODY_CENTER_Y - (PANEL_BODY_Y / 2.0)
PANEL_DISPLAY_X = 0.058
PANEL_DISPLAY_Z = 0.012
PANEL_DISPLAY_RECESS = 0.003
PANEL_DISPLAY_CENTER_Z = -0.020
PANEL_BUTTON_XS = (-0.048, -0.016, 0.016, 0.048)
PANEL_BUTTON_Z = -0.028
BUTTON_HOLE_RADIUS = 0.0053
BUTTON_HOLE_DEPTH = 0.012

BUTTON_STEM_RADIUS = 0.0043
BUTTON_CAP_RADIUS = 0.0065
BUTTON_STEM_Y = 0.007
BUTTON_CAP_Y = 0.004
BUTTON_TRAVEL = 0.003


def _filleted_box(x: float, y: float, z: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z).edges("|Z").fillet(radius)


def _build_top_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)
        .edges("|Z")
        .fillet(TOP_CORNER_RADIUS)
        .translate((0.0, 0.0, TOP_THICKNESS / 2.0))
    )


def _build_underframe_shape() -> cq.Workplane:
    left_plate = _filleted_box(
        UNDERFRAME_PLATE_X,
        UNDERFRAME_PLATE_Y,
        UNDERFRAME_PLATE_T,
        0.004,
    ).translate((LEG_CENTER_X, 0.0, -UNDERFRAME_PLATE_T / 2.0))
    right_plate = _filleted_box(
        UNDERFRAME_PLATE_X,
        UNDERFRAME_PLATE_Y,
        UNDERFRAME_PLATE_T,
        0.004,
    ).translate((-LEG_CENTER_X, 0.0, -UNDERFRAME_PLATE_T / 2.0))
    center_beam = _filleted_box(
        UNDERFRAME_BEAM_X,
        UNDERFRAME_BEAM_Y,
        UNDERFRAME_BEAM_Z,
        0.004,
    ).translate(
        (
            0.0,
            0.0,
            -UNDERFRAME_PLATE_T - (UNDERFRAME_BEAM_Z / 2.0),
        )
    )
    return left_plate.union(right_plate).union(center_beam)


def _build_outer_top_plate() -> cq.Workplane:
    return _filleted_box(OUTER_PLATE_X, OUTER_PLATE_Y, OUTER_PLATE_T, 0.003).translate(
        (0.0, 0.0, -OUTER_PLATE_T / 2.0)
    )


def _build_outer_sleeve() -> cq.Workplane:
    sleeve = _filleted_box(
        OUTER_SLEEVE_X,
        OUTER_SLEEVE_Y,
        OUTER_SLEEVE_Z,
        0.004,
    ).translate((0.0, 0.0, -OUTER_PLATE_T - (OUTER_SLEEVE_Z / 2.0)))
    cavity = cq.Workplane("XY").box(
        OUTER_SLEEVE_X - (2.0 * OUTER_WALL),
        OUTER_SLEEVE_Y - (2.0 * OUTER_WALL),
        OUTER_SLEEVE_Z + 0.004,
    ).translate((0.0, 0.0, -OUTER_PLATE_T - (OUTER_SLEEVE_Z / 2.0) - 0.002))
    return sleeve.cut(cavity)


def _build_inner_stage() -> cq.Workplane:
    stage = _filleted_box(INNER_STAGE_X, INNER_STAGE_Y, INNER_STAGE_Z, 0.003).translate(
        (0.0, 0.0, INNER_STAGE_CENTER_Z)
    )

    side_pad_x = (INNER_STAGE_X / 2.0) + GUIDE_PAD_PROUD - (GUIDE_PAD_THICK / 2.0)
    front_pad_y = (INNER_STAGE_Y / 2.0) + GUIDE_PAD_PROUD - (GUIDE_PAD_THICK / 2.0)

    right_pad = cq.Workplane("XY").box(
        GUIDE_PAD_THICK,
        GUIDE_PAD_SIDE_SPAN,
        GUIDE_PAD_HEIGHT,
    ).translate((side_pad_x, 0.0, GUIDE_PAD_CENTER_Z))
    left_pad = cq.Workplane("XY").box(
        GUIDE_PAD_THICK,
        GUIDE_PAD_SIDE_SPAN,
        GUIDE_PAD_HEIGHT,
    ).translate((-side_pad_x, 0.0, GUIDE_PAD_CENTER_Z))
    front_pad = cq.Workplane("XY").box(
        GUIDE_PAD_FRONT_SPAN,
        GUIDE_PAD_THICK,
        GUIDE_PAD_HEIGHT,
    ).translate((0.0, front_pad_y, GUIDE_PAD_CENTER_Z))
    back_pad = cq.Workplane("XY").box(
        GUIDE_PAD_FRONT_SPAN,
        GUIDE_PAD_THICK,
        GUIDE_PAD_HEIGHT,
    ).translate((0.0, -front_pad_y, GUIDE_PAD_CENTER_Z))

    return (
        stage.union(right_pad)
        .union(left_pad)
        .union(front_pad)
        .union(back_pad)
    )


def _build_foot_assembly() -> cq.Workplane:
    mount_center_z = -INNER_STAGE_DOWN - (FOOT_MOUNT_Z / 2.0)
    foot_center_z = -INNER_STAGE_DOWN - FOOT_MOUNT_Z - (FOOT_BEAM_Z / 2.0)
    glide_bottom_z = -INNER_STAGE_DOWN - FOOT_MOUNT_Z - FOOT_BEAM_Z - GLIDE_Z

    mount = _filleted_box(FOOT_MOUNT_X, FOOT_MOUNT_Y, FOOT_MOUNT_Z, 0.004).translate(
        (0.0, 0.0, mount_center_z)
    )
    beam = _filleted_box(FOOT_BEAM_X, FOOT_BEAM_Y, FOOT_BEAM_Z, 0.006).translate(
        (0.0, 0.0, foot_center_z)
    )
    left_glide = (
        cq.Workplane("XY")
        .circle(GLIDE_RADIUS)
        .extrude(GLIDE_Z)
        .translate((0.0, GLIDE_OFFSET_Y, glide_bottom_z))
    )
    right_glide = (
        cq.Workplane("XY")
        .circle(GLIDE_RADIUS)
        .extrude(GLIDE_Z)
        .translate((0.0, -GLIDE_OFFSET_Y, glide_bottom_z))
    )
    return mount.union(beam).union(left_glide).union(right_glide)


def _build_panel_housing() -> cq.Workplane:
    flange = _filleted_box(PANEL_FLANGE_X, PANEL_FLANGE_Y, PANEL_FLANGE_Z, 0.002).translate(
        (0.0, -0.004, -PANEL_FLANGE_Z / 2.0)
    )
    neck = _filleted_box(PANEL_NECK_X, PANEL_NECK_Y, PANEL_NECK_Z, 0.002).translate(
        (0.0, -0.017, -0.011)
    )
    body = _filleted_box(PANEL_BODY_X, PANEL_BODY_Y, PANEL_BODY_Z, 0.003).translate(
        (0.0, PANEL_BODY_CENTER_Y, PANEL_BODY_CENTER_Z)
    )
    housing = flange.union(neck).union(body)

    button_cutters = (
        cq.Workplane("XZ")
        .pushPoints([(x, PANEL_BUTTON_Z) for x in PANEL_BUTTON_XS])
        .circle(BUTTON_HOLE_RADIUS)
        .extrude(BUTTON_HOLE_DEPTH)
        .translate((0.0, PANEL_FRONT_Y, 0.0))
    )
    display_cut = cq.Workplane("XY").box(
        PANEL_DISPLAY_X,
        PANEL_DISPLAY_RECESS,
        PANEL_DISPLAY_Z,
    ).translate(
        (
            0.0,
            PANEL_FRONT_Y + (PANEL_DISPLAY_RECESS / 2.0),
            PANEL_DISPLAY_CENTER_Z,
        )
    )
    return housing.cut(button_cutters).cut(display_cut)


def _build_button_shape() -> cq.Workplane:
    stem = cq.Workplane("XZ").circle(BUTTON_STEM_RADIUS).extrude(BUTTON_STEM_Y)
    cap = (
        cq.Workplane("XZ")
        .circle(BUTTON_CAP_RADIUS)
        .extrude(BUTTON_CAP_Y)
        .translate((0.0, -BUTTON_CAP_Y, 0.0))
    )
    return stem.union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_standing_desk")

    model.material("walnut_top", rgba=(0.60, 0.44, 0.30, 1.0))
    model.material("powder_coat", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("dark_panel", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("button_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("display_glass", rgba=(0.08, 0.12, 0.16, 1.0))

    desk_top = model.part("desk_top")
    desk_top.visual(
        mesh_from_cadquery(_build_top_shape(), "desk_top"),
        material="walnut_top",
        name="desktop_slab",
    )
    desk_top.inertial = Inertial.from_geometry(
        Box((TOP_WIDTH, TOP_DEPTH, TOP_THICKNESS)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_THICKNESS / 2.0)),
    )

    underframe = model.part("underframe")
    underframe.visual(
        Box((UNDERFRAME_PLATE_X, UNDERFRAME_PLATE_Y, UNDERFRAME_PLATE_T)),
        origin=Origin(xyz=(LEG_CENTER_X, 0.0, -UNDERFRAME_PLATE_T / 2.0)),
        material="powder_coat",
        name="left_mount_plate",
    )
    underframe.visual(
        Box((UNDERFRAME_PLATE_X, UNDERFRAME_PLATE_Y, UNDERFRAME_PLATE_T)),
        origin=Origin(xyz=(-LEG_CENTER_X, 0.0, -UNDERFRAME_PLATE_T / 2.0)),
        material="powder_coat",
        name="right_mount_plate",
    )
    underframe.visual(
        Box((UNDERFRAME_BEAM_X, UNDERFRAME_BEAM_Y, UNDERFRAME_BEAM_Z)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -UNDERFRAME_PLATE_T - (UNDERFRAME_BEAM_Z / 2.0),
            )
        ),
        material="powder_coat",
        name="center_beam",
    )
    underframe.inertial = Inertial.from_geometry(
        Box((2.0 * LEG_CENTER_X + UNDERFRAME_PLATE_X, UNDERFRAME_PLATE_Y, 0.07)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )
    model.articulation(
        "desk_top_to_underframe",
        ArticulationType.FIXED,
        parent=desk_top,
        child=underframe,
        origin=Origin(),
    )

    for side_name, x_sign in (("left", 1.0), ("right", -1.0)):
        outer = model.part(f"{side_name}_outer_leg")
        outer.visual(
            Box((OUTER_PLATE_X, OUTER_PLATE_Y, OUTER_PLATE_T)),
            origin=Origin(xyz=(0.0, 0.0, -OUTER_PLATE_T / 2.0)),
            material="powder_coat",
            name="outer_top_plate",
        )
        outer.visual(
            Box((OUTER_WALL, OUTER_SLEEVE_Y, OUTER_SLEEVE_Z)),
            origin=Origin(
                xyz=(
                    (OUTER_SLEEVE_X / 2.0) - (OUTER_WALL / 2.0),
                    0.0,
                    -OUTER_PLATE_T - (OUTER_SLEEVE_Z / 2.0),
                )
            ),
            material="powder_coat",
            name="outer_right_wall",
        )
        outer.visual(
            Box((OUTER_WALL, OUTER_SLEEVE_Y, OUTER_SLEEVE_Z)),
            origin=Origin(
                xyz=(
                    -(OUTER_SLEEVE_X / 2.0) + (OUTER_WALL / 2.0),
                    0.0,
                    -OUTER_PLATE_T - (OUTER_SLEEVE_Z / 2.0),
                )
            ),
            material="powder_coat",
            name="outer_left_wall",
        )
        outer.visual(
            Box((OUTER_SLEEVE_X - (2.0 * OUTER_WALL), OUTER_WALL, OUTER_SLEEVE_Z)),
            origin=Origin(
                xyz=(
                    0.0,
                    (OUTER_SLEEVE_Y / 2.0) - (OUTER_WALL / 2.0),
                    -OUTER_PLATE_T - (OUTER_SLEEVE_Z / 2.0),
                )
            ),
            material="powder_coat",
            name="outer_front_wall",
        )
        outer.visual(
            Box((OUTER_SLEEVE_X - (2.0 * OUTER_WALL), OUTER_WALL, OUTER_SLEEVE_Z)),
            origin=Origin(
                xyz=(
                    0.0,
                    -(OUTER_SLEEVE_Y / 2.0) + (OUTER_WALL / 2.0),
                    -OUTER_PLATE_T - (OUTER_SLEEVE_Z / 2.0),
                )
            ),
            material="powder_coat",
            name="outer_back_wall",
        )
        outer.inertial = Inertial.from_geometry(
            Box((OUTER_PLATE_X, OUTER_PLATE_Y, OUTER_PLATE_T + OUTER_SLEEVE_Z)),
            mass=5.8,
            origin=Origin(xyz=(0.0, 0.0, -(OUTER_PLATE_T + OUTER_SLEEVE_Z) / 2.0)),
        )

        inner = model.part(f"{side_name}_inner_leg")
        inner.visual(
            mesh_from_cadquery(_build_inner_stage(), f"{side_name}_inner_stage"),
            material="powder_coat",
            name="inner_stage",
        )
        inner.visual(
            mesh_from_cadquery(_build_foot_assembly(), f"{side_name}_foot_assembly"),
            material="powder_coat",
            name="foot_assembly",
        )
        inner.inertial = Inertial.from_geometry(
            Box(
                (
                    FOOT_BEAM_X,
                    FOOT_BEAM_Y,
                    INNER_STAGE_UP + INNER_STAGE_DOWN + FOOT_MOUNT_Z + FOOT_BEAM_Z + GLIDE_Z,
                )
            ),
            mass=7.5,
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    (INNER_STAGE_UP - (INNER_STAGE_DOWN + FOOT_MOUNT_Z + FOOT_BEAM_Z + GLIDE_Z))
                    / 2.0,
                )
            ),
        )

        model.articulation(
            f"underframe_to_{side_name}_outer",
            ArticulationType.FIXED,
            parent=underframe,
            child=outer,
            origin=Origin(xyz=(x_sign * LEG_CENTER_X, 0.0, -UNDERFRAME_PLATE_T)),
        )
        model.articulation(
            f"{side_name}_outer_to_inner",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=inner,
            origin=Origin(xyz=(0.0, 0.0, OUTER_TO_INNER_ORIGIN_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=LEG_TRAVEL,
                effort=1800.0,
                velocity=0.050,
            ),
        )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_build_panel_housing(), "control_panel_housing"),
        material="dark_panel",
        name="panel_housing",
    )
    control_panel.visual(
        Box((PANEL_DISPLAY_X, 0.0012, PANEL_DISPLAY_Z - 0.001)),
        origin=Origin(
            xyz=(
                0.0,
                PANEL_FRONT_Y + 0.0015,
                PANEL_DISPLAY_CENTER_Z,
            )
        ),
        material="display_glass",
        name="display_window",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((PANEL_FLANGE_X, 0.070, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.030, -0.020)),
    )
    model.articulation(
        "desk_top_to_control_panel",
        ArticulationType.FIXED,
        parent=desk_top,
        child=control_panel,
        origin=Origin(
            xyz=(
                0.310,
                -(TOP_DEPTH / 2.0) + 0.030,
                0.0,
            )
        ),
    )

    button_shape = _build_button_shape()
    button_specs = (
        ("preset_1_button", PANEL_BUTTON_XS[0]),
        ("preset_2_button", PANEL_BUTTON_XS[1]),
        ("down_button", PANEL_BUTTON_XS[2]),
        ("up_button", PANEL_BUTTON_XS[3]),
    )
    for button_name, x_pos in button_specs:
        button = model.part(button_name)
        button.visual(
            mesh_from_cadquery(button_shape, button_name),
            material="button_black",
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.014, BUTTON_STEM_Y + BUTTON_CAP_Y, 0.014)),
            mass=0.02,
            origin=Origin(xyz=(0.0, (BUTTON_STEM_Y - BUTTON_CAP_Y) / 2.0, 0.0)),
        )
        model.articulation(
            f"control_panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(x_pos, PANEL_FRONT_Y, PANEL_BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0,
                upper=BUTTON_TRAVEL,
                effort=12.0,
                velocity=0.030,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    desk_top = object_model.get_part("desk_top")
    control_panel = object_model.get_part("control_panel")
    left_outer = object_model.get_part("left_outer_leg")
    right_outer = object_model.get_part("right_outer_leg")
    left_inner = object_model.get_part("left_inner_leg")
    right_inner = object_model.get_part("right_inner_leg")
    up_button = object_model.get_part("up_button")
    preset_button = object_model.get_part("preset_1_button")

    left_slide = object_model.get_articulation("left_outer_to_inner")
    right_slide = object_model.get_articulation("right_outer_to_inner")
    up_button_joint = object_model.get_articulation("control_panel_to_up_button")
    preset_button_joint = object_model.get_articulation("control_panel_to_preset_1_button")

    ctx.expect_gap(
        desk_top,
        control_panel,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        name="control panel hangs directly beneath the top",
    )

    for outer_part, inner_part, label in (
        (left_outer, left_inner, "left"),
        (right_outer, right_inner, "right"),
    ):
        ctx.expect_within(
            inner_part,
            outer_part,
            axes="xy",
            inner_elem="inner_stage",
            margin=0.002,
            name=f"{label} inner stage stays centered in the outer sleeve",
        )
        ctx.expect_overlap(
            inner_part,
            outer_part,
            axes="z",
            elem_a="inner_stage",
            min_overlap=0.32,
            name=f"{label} inner stage has deep retained insertion at rest",
        )

    left_rest = ctx.part_world_position(left_inner)
    right_rest = ctx.part_world_position(right_inner)
    with ctx.pose({left_slide: LEG_TRAVEL, right_slide: LEG_TRAVEL}):
        ctx.expect_within(
            left_inner,
            left_outer,
            axes="xy",
            inner_elem="inner_stage",
            margin=0.002,
            name="left inner stage stays centered when extended",
        )
        ctx.expect_within(
            right_inner,
            right_outer,
            axes="xy",
            inner_elem="inner_stage",
            margin=0.002,
            name="right inner stage stays centered when extended",
        )
        ctx.expect_overlap(
            left_inner,
            left_outer,
            axes="z",
            elem_a="inner_stage",
            min_overlap=0.10,
            name="left leg retains insertion at maximum height",
        )
        ctx.expect_overlap(
            right_inner,
            right_outer,
            axes="z",
            elem_a="inner_stage",
            min_overlap=0.10,
            name="right leg retains insertion at maximum height",
        )
        left_extended = ctx.part_world_position(left_inner)
        right_extended = ctx.part_world_position(right_inner)

    ctx.check(
        "left lifting leg extends downward",
        left_rest is not None
        and left_extended is not None
        and left_extended[2] < left_rest[2] - 0.20,
        details=f"rest={left_rest}, extended={left_extended}",
    )
    ctx.check(
        "right lifting leg extends downward",
        right_rest is not None
        and right_extended is not None
        and right_extended[2] < right_rest[2] - 0.20,
        details=f"rest={right_rest}, extended={right_extended}",
    )

    up_rest = ctx.part_world_position(up_button)
    with ctx.pose({up_button_joint: BUTTON_TRAVEL}):
        up_pressed = ctx.part_world_position(up_button)
    ctx.check(
        "up button presses inward",
        up_rest is not None
        and up_pressed is not None
        and up_pressed[1] > up_rest[1] + 0.002,
        details=f"rest={up_rest}, pressed={up_pressed}",
    )

    preset_rest = ctx.part_world_position(preset_button)
    with ctx.pose({preset_button_joint: BUTTON_TRAVEL}):
        preset_pressed = ctx.part_world_position(preset_button)
    ctx.check(
        "preset button presses inward",
        preset_rest is not None
        and preset_pressed is not None
        and preset_pressed[1] > preset_rest[1] + 0.002,
        details=f"rest={preset_rest}, pressed={preset_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
