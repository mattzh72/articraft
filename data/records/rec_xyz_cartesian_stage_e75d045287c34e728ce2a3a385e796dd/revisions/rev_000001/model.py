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


BASE_L = 0.42
BASE_W = 0.26
BASE_T = 0.018
SIDE_BEAM_W = 0.038
SIDE_BEAM_H = 0.040
END_BRACE_L = 0.060
END_BRACE_H = 0.028
X_RAIL_L = 0.360
X_RAIL_W = 0.026
X_RAIL_H = 0.012
X_RAIL_Y = 0.072

X_PAD_H = X_RAIL_H
X_CARR_L = 0.300
X_CARR_W = 0.200
X_TOP_T = 0.020
X_TOP_POCKET_D = 0.008
Y_RAIL_X = 0.045
Y_RAIL_L = 0.180
Y_RAIL_W = 0.022
Y_RAIL_H = 0.010

Y_PAD_H = Y_RAIL_H
CROSS_W = 0.160
CROSS_L = 0.120
CROSS_TOP_T = 0.018
CROSS_POCKET_D = 0.006
GUIDE_SHELF_W = 0.102
GUIDE_SHELF_L = 0.044
GUIDE_SHELF_T = 0.012
GUIDE_POST_W = 0.074
GUIDE_POST_L = 0.030
GUIDE_POST_H = 0.010
GUIDE_CHEEK_T = 0.018
GUIDE_INNER_W = 0.060
GUIDE_CHEEK_H = 0.150
GUIDE_BACK_T = 0.010
GUIDE_BASE_OVERLAP = 0.004

RAM_W = 0.050
RAM_D = 0.026
RAM_H = 0.118
RAM_CAP_W = 0.058
RAM_CAP_D = 0.032
RAM_CAP_T = 0.016
RAM_NECK_W = 0.028
RAM_NECK_D = 0.018
RAM_NECK_H = 0.024
RAM_NOSE_W = 0.040
RAM_NOSE_D = 0.018
RAM_NOSE_H = 0.040
RAM_Y_OFFSET = 0.008

FRAME_TO_X_Z = BASE_T + SIDE_BEAM_H + X_RAIL_H
X_TO_Y_Z = X_PAD_H + X_TOP_T + Y_RAIL_H
Y_TO_Z_Z = Y_PAD_H + CROSS_TOP_T - GUIDE_BASE_OVERLAP + GUIDE_SHELF_T


def make_lower_frame() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BASE_L - 0.120, BASE_W - 0.100)
        .cutBlind(-BASE_T * 0.55)
    )

    side_beam = cq.Workplane("XY").box(
        BASE_L - 0.040, SIDE_BEAM_W, SIDE_BEAM_H, centered=(True, True, False)
    )
    end_brace = cq.Workplane("XY").box(
        END_BRACE_L, BASE_W - 0.090, END_BRACE_H, centered=(True, True, False)
    )
    rail = cq.Workplane("XY").box(
        X_RAIL_L, X_RAIL_W, X_RAIL_H, centered=(True, True, False)
    )

    frame = base
    frame = frame.union(side_beam.translate((0.0, X_RAIL_Y, BASE_T)))
    frame = frame.union(side_beam.translate((0.0, -X_RAIL_Y, BASE_T)))
    frame = frame.union(
        end_brace.translate((BASE_L * 0.5 - END_BRACE_L * 0.5 - 0.030, 0.0, BASE_T))
    )
    frame = frame.union(
        end_brace.translate((-BASE_L * 0.5 + END_BRACE_L * 0.5 + 0.030, 0.0, BASE_T))
    )
    frame = frame.union(rail.translate((0.0, X_RAIL_Y, BASE_T + SIDE_BEAM_H)))
    frame = frame.union(rail.translate((0.0, -X_RAIL_Y, BASE_T + SIDE_BEAM_H)))
    return frame


def make_x_carriage() -> cq.Workplane:
    top = cq.Workplane("XY").box(
        X_CARR_L, X_CARR_W, X_TOP_T, centered=(True, True, False)
    )
    top = top.translate((0.0, 0.0, X_PAD_H))
    top = (
        top.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(X_CARR_L - 0.090, X_CARR_W - 0.080)
        .cutBlind(-X_TOP_POCKET_D)
    )

    pad = cq.Workplane("XY").box(
        X_CARR_L - 0.030, X_RAIL_W, X_PAD_H, centered=(True, True, False)
    )
    y_rail = cq.Workplane("XY").box(
        Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, centered=(True, True, False)
    )

    carriage = top
    carriage = carriage.union(pad.translate((0.0, X_RAIL_Y, 0.0)))
    carriage = carriage.union(pad.translate((0.0, -X_RAIL_Y, 0.0)))
    carriage = carriage.union(
        y_rail.translate((Y_RAIL_X, 0.0, X_PAD_H + X_TOP_T))
    )
    carriage = carriage.union(
        y_rail.translate((-Y_RAIL_X, 0.0, X_PAD_H + X_TOP_T))
    )
    return carriage


def make_cross_slide() -> cq.Workplane:
    top = cq.Workplane("XY").box(
        CROSS_W, CROSS_L, CROSS_TOP_T, centered=(True, True, False)
    )
    top = top.translate((0.0, 0.0, Y_PAD_H))
    top = (
        top.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(CROSS_W - 0.050, CROSS_L - 0.040)
        .cutBlind(-CROSS_POCKET_D)
    )

    pad = cq.Workplane("XY").box(
        Y_RAIL_W, CROSS_L - 0.010, Y_PAD_H, centered=(True, True, False)
    )
    shelf = cq.Workplane("XY").box(
        GUIDE_SHELF_W, GUIDE_SHELF_L, GUIDE_SHELF_T, centered=(True, True, False)
    )
    post = cq.Workplane("XY").box(
        GUIDE_POST_W, GUIDE_POST_L, GUIDE_POST_H, centered=(True, True, False)
    )
    cheek = cq.Workplane("XY").box(
        GUIDE_CHEEK_T, GUIDE_SHELF_L, GUIDE_CHEEK_H, centered=(True, True, False)
    )
    back = cq.Workplane("XY").box(
        GUIDE_SHELF_W, GUIDE_BACK_T, GUIDE_CHEEK_H, centered=(True, True, False)
    )

    cross_slide = top
    cross_slide = cross_slide.union(pad.translate((Y_RAIL_X, 0.0, 0.0)))
    cross_slide = cross_slide.union(pad.translate((-Y_RAIL_X, 0.0, 0.0)))
    guide_base_z = Y_PAD_H + CROSS_TOP_T - GUIDE_BASE_OVERLAP

    cross_slide = cross_slide.union(shelf.translate((0.0, 0.0, guide_base_z)))
    cross_slide = cross_slide.union(
        post.translate((0.0, 0.0, guide_base_z - 0.006))
    )
    cross_slide = cross_slide.union(
        cheek.translate(
            (
                0.5 * (GUIDE_INNER_W + GUIDE_CHEEK_T),
                0.0,
                guide_base_z,
            )
        )
    )
    cross_slide = cross_slide.union(
        cheek.translate(
            (
                -0.5 * (GUIDE_INNER_W + GUIDE_CHEEK_T),
                0.0,
                guide_base_z,
            )
        )
    )
    cross_slide = cross_slide.union(
        back.translate(
            (
                0.0,
                -0.5 * GUIDE_SHELF_L + 0.5 * GUIDE_BACK_T,
                guide_base_z,
            )
        )
    )
    return cross_slide.combine()


def make_vertical_ram() -> cq.Workplane:
    ram = cq.Workplane("XY").box(RAM_W, RAM_D, RAM_H, centered=(True, True, False))
    cap = cq.Workplane("XY").box(
        RAM_CAP_W, RAM_CAP_D, RAM_CAP_T, centered=(True, True, False)
    )
    neck = cq.Workplane("XY").box(
        RAM_NECK_W, RAM_NECK_D, RAM_NECK_H, centered=(True, True, False)
    )
    nose = cq.Workplane("XY").box(
        RAM_NOSE_W, RAM_NOSE_D, RAM_NOSE_H, centered=(True, True, False)
    )

    ram = ram.union(cap.translate((0.0, RAM_Y_OFFSET, RAM_H - 0.010)))
    ram = ram.union(neck.translate((0.0, RAM_Y_OFFSET + 0.013, 0.016)))
    ram = ram.union(nose.translate((0.0, RAM_Y_OFFSET + 0.022, 0.018)))
    ram = ram.translate((0.0, RAM_Y_OFFSET, 0.0))
    return ram.combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_positioning_stack")

    model.material("frame_paint", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("machined_alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("slide_alloy", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("ram_finish", rgba=(0.60, 0.64, 0.70, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        mesh_from_cadquery(make_lower_frame(), "lower_frame"),
        material="frame_paint",
        name="frame_body",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(make_x_carriage(), "x_carriage"),
        material="machined_alloy",
        name="x_carriage_body",
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        mesh_from_cadquery(make_cross_slide(), "cross_slide"),
        material="slide_alloy",
        name="cross_slide_body",
    )

    vertical_ram = model.part("vertical_ram")
    vertical_ram.visual(
        mesh_from_cadquery(make_vertical_ram(), "vertical_ram"),
        material="ram_finish",
        name="vertical_ram_body",
    )

    model.articulation(
        "frame_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, FRAME_TO_X_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=-0.060,
            upper=0.060,
        ),
    )
    model.articulation(
        "x_carriage_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, X_TO_Y_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=-0.030,
            upper=0.030,
        ),
    )
    model.articulation(
        "cross_slide_to_ram",
        ArticulationType.PRISMATIC,
        parent=cross_slide,
        child=vertical_ram,
        origin=Origin(xyz=(0.0, 0.0, Y_TO_Z_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.000,
            upper=0.080,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    lower_frame = object_model.get_part("lower_frame")
    x_carriage = object_model.get_part("x_carriage")
    cross_slide = object_model.get_part("cross_slide")
    vertical_ram = object_model.get_part("vertical_ram")

    x_axis = object_model.get_articulation("frame_to_x_carriage")
    y_axis = object_model.get_articulation("x_carriage_to_cross_slide")
    z_axis = object_model.get_articulation("cross_slide_to_ram")

    ctx.expect_contact(
        x_carriage,
        lower_frame,
        name="x_carriage_is_supported_by_lower_frame",
    )
    ctx.expect_contact(
        cross_slide,
        x_carriage,
        name="cross_slide_is_supported_by_x_carriage",
    )
    ctx.expect_contact(
        vertical_ram,
        cross_slide,
        name="vertical_ram_is_supported_by_cross_slide",
    )

    ctx.check(
        "prismatic_axes_match_xyz_stack",
        x_axis.axis == (1.0, 0.0, 0.0)
        and y_axis.axis == (0.0, 1.0, 0.0)
        and z_axis.axis == (0.0, 0.0, 1.0),
        details=(
            f"axes were x={x_axis.axis}, y={y_axis.axis}, z={z_axis.axis}"
        ),
    )

    with ctx.pose({x_axis: x_axis.motion_limits.lower}):
        x_low = ctx.part_world_position(x_carriage)
    with ctx.pose({x_axis: x_axis.motion_limits.upper}):
        x_high = ctx.part_world_position(x_carriage)
        ctx.expect_overlap(
            x_carriage,
            lower_frame,
            axes="xy",
            min_overlap=0.18,
            name="x_carriage_remains_over_frame_at_max_x",
        )

    with ctx.pose({y_axis: y_axis.motion_limits.lower}):
        y_low = ctx.part_world_position(cross_slide)
    with ctx.pose({y_axis: y_axis.motion_limits.upper}):
        y_high = ctx.part_world_position(cross_slide)
        ctx.expect_overlap(
            cross_slide,
            x_carriage,
            axes="xy",
            min_overlap=0.10,
            name="cross_slide_remains_over_x_carriage_at_max_y",
        )

    with ctx.pose({z_axis: z_axis.motion_limits.lower}):
        z_low = ctx.part_world_position(vertical_ram)
    with ctx.pose({z_axis: z_axis.motion_limits.upper}):
        z_high = ctx.part_world_position(vertical_ram)
        ctx.expect_overlap(
            vertical_ram,
            cross_slide,
            axes="xy",
            min_overlap=0.04,
            name="vertical_ram_remains_laterally_captured_at_max_z",
        )

    x_ok = (
        x_low is not None
        and x_high is not None
        and x_high[0] - x_low[0] > 0.100
        and abs(x_high[1] - x_low[1]) < 1e-6
        and abs(x_high[2] - x_low[2]) < 1e-6
    )
    y_ok = (
        y_low is not None
        and y_high is not None
        and y_high[1] - y_low[1] > 0.050
        and abs(y_high[0] - y_low[0]) < 1e-6
        and abs(y_high[2] - y_low[2]) < 1e-6
    )
    z_ok = (
        z_low is not None
        and z_high is not None
        and z_high[2] - z_low[2] > 0.070
        and abs(z_high[0] - z_low[0]) < 1e-6
        and abs(z_high[1] - z_low[1]) < 1e-6
    )

    ctx.check(
        "x_axis_moves_only_along_x",
        x_ok,
        details=f"lower={x_low}, upper={x_high}",
    )
    ctx.check(
        "y_axis_moves_only_along_y",
        y_ok,
        details=f"lower={y_low}, upper={y_high}",
    )
    ctx.check(
        "z_axis_moves_only_along_z",
        z_ok,
        details=f"lower={z_low}, upper={z_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
