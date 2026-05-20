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


BASE_L = 0.460
BASE_W = 0.240
BASE_H = 0.024

RAIL_Y = 0.084
PEDESTAL_L = 0.390
PEDESTAL_W = 0.028
PEDESTAL_H = 0.012
RAIL_L = 0.360
RAIL_W = 0.018
RAIL_H = 0.012
RAIL_TOP_Z = BASE_H + PEDESTAL_H + RAIL_H

RAIL_CAP_L = 0.012
RAIL_CAP_H = 0.020

BRIDGE_SHOE_L = 0.084
BRIDGE_SHOE_W = 0.044
BRIDGE_SHOE_H = 0.030
BRIDGE_BEAM_L = 0.120
BRIDGE_BEAM_W = 0.188
BRIDGE_BEAM_H = 0.022
BRIDGE_DECK_L = 0.122
BRIDGE_DECK_W = 0.196
BRIDGE_DECK_H = 0.018
BRIDGE_WINDOW_L = 0.082
BRIDGE_WINDOW_W = 0.136
BRIDGE_GUIDE_X = 0.041
BRIDGE_GUIDE_W = 0.016
BRIDGE_GUIDE_L = 0.184
BRIDGE_GUIDE_H = 0.010
BRIDGE_GUIDE_TOP_Z = RAIL_TOP_Z + BRIDGE_SHOE_H + BRIDGE_BEAM_H + BRIDGE_DECK_H + BRIDGE_GUIDE_H

Y_SLIDE_L = 0.114
Y_SLIDE_W = 0.074
Y_SLIDE_H = 0.018
Y_SLIDE_BOTTOM_Z = BRIDGE_GUIDE_TOP_Z
Y_SLIDE_WINDOW_L = 0.074
Y_SLIDE_WINDOW_W = 0.046
Y_HOUSING_L = 0.072
Y_HOUSING_W = 0.058
Y_HOUSING_H = 0.090
Y_HOUSING_BOTTOM_Z = Y_SLIDE_BOTTOM_Z + Y_SLIDE_H
Z_GUIDE_SLOT_L = 0.074
Z_GUIDE_SLOT_W = 0.046
COLUMN_T = 0.018
COLUMN_D = 0.052
COLUMN_H = 0.078
REAR_WEB_T = 0.008
REAR_WEB_H = 0.070
TOP_CAP_D = 0.016
TOP_CAP_H = 0.010
Z_JOINT_Z = Y_HOUSING_BOTTOM_Z + 0.078

Y_COVER_T = 0.004
Y_COVER_BOTTOM_Z = Y_HOUSING_BOTTOM_Z + 0.006
Y_COVER_H = 0.066

Z_GUIDE_TOP_Z = Y_HOUSING_BOTTOM_Z + Y_HOUSING_H
RAM_GUIDE_L = 0.036
RAM_GUIDE_W = 0.030
RAM_GUIDE_H = 0.060
RAM_BODY_L = 0.044
RAM_BODY_W = 0.034
RAM_BODY_H = 0.036
RAM_BLOCK_L = 0.054
RAM_BLOCK_W = 0.040
RAM_BLOCK_H = 0.010
RAM_FLANGE_L = 0.064
RAM_FLANGE_W = 0.046
RAM_FLANGE_H = 0.005

X_TRAVEL = 0.110
Y_TRAVEL = 0.030
Z_TRAVEL = 0.055


def _box(size: tuple[float, float, float], center_xy: tuple[float, float], bottom_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2], centered=(True, True, False))
        .translate((center_xy[0], center_xy[1], bottom_z))
    )


def _base_body_shape() -> cq.Workplane:
    body = _box((BASE_L, BASE_W, BASE_H), (0.0, 0.0), 0.0)
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BASE_L - 0.110, 0.086)
        .cutBlind(-0.006)
    )
    return body


def _base_rail_shape() -> cq.Workplane:
    pedestals = _box((PEDESTAL_L, PEDESTAL_W, PEDESTAL_H), (0.0, -RAIL_Y), BASE_H)
    pedestals = pedestals.union(_box((PEDESTAL_L, PEDESTAL_W, PEDESTAL_H), (0.0, RAIL_Y), BASE_H))

    rails = _box((RAIL_L, RAIL_W, RAIL_H), (0.0, -RAIL_Y), BASE_H + PEDESTAL_H)
    rails = rails.union(_box((RAIL_L, RAIL_W, RAIL_H), (0.0, RAIL_Y), BASE_H + PEDESTAL_H))

    cap_x = RAIL_L / 2.0 + RAIL_CAP_L / 2.0
    caps = _box((RAIL_CAP_L, PEDESTAL_W, RAIL_CAP_H), (-cap_x, -RAIL_Y), BASE_H)
    caps = caps.union(_box((RAIL_CAP_L, PEDESTAL_W, RAIL_CAP_H), (cap_x, -RAIL_Y), BASE_H))
    caps = caps.union(_box((RAIL_CAP_L, PEDESTAL_W, RAIL_CAP_H), (-cap_x, RAIL_Y), BASE_H))
    caps = caps.union(_box((RAIL_CAP_L, PEDESTAL_W, RAIL_CAP_H), (cap_x, RAIL_Y), BASE_H))

    return pedestals.union(rails).union(caps)


def _bridge_body_shape() -> cq.Workplane:
    shoes = _box((BRIDGE_SHOE_L, BRIDGE_SHOE_W, BRIDGE_SHOE_H), (0.0, -RAIL_Y), RAIL_TOP_Z)
    shoes = shoes.union(_box((BRIDGE_SHOE_L, BRIDGE_SHOE_W, BRIDGE_SHOE_H), (0.0, RAIL_Y), RAIL_TOP_Z))

    beam_bottom = RAIL_TOP_Z + BRIDGE_SHOE_H
    deck_bottom = beam_bottom + BRIDGE_BEAM_H

    beam = _box((BRIDGE_BEAM_L, BRIDGE_BEAM_W, BRIDGE_BEAM_H), (0.0, 0.0), beam_bottom)
    deck = _box((BRIDGE_DECK_L, BRIDGE_DECK_W, BRIDGE_DECK_H), (0.0, 0.0), deck_bottom)

    end_housings = _box((0.056, 0.010, BRIDGE_DECK_H), (0.0, -0.084), deck_bottom)
    end_housings = end_housings.union(_box((0.056, 0.010, BRIDGE_DECK_H), (0.0, 0.084), deck_bottom))

    body = shoes.union(beam).union(deck).union(end_housings)
    window = _box((BRIDGE_WINDOW_L, BRIDGE_WINDOW_W, 0.060), (0.0, 0.0), beam_bottom)
    return body.cut(window)


def _bridge_guides_shape() -> cq.Workplane:
    guide_bottom = RAIL_TOP_Z + BRIDGE_SHOE_H + BRIDGE_BEAM_H + BRIDGE_DECK_H
    guides = _box((BRIDGE_GUIDE_W, BRIDGE_GUIDE_L, BRIDGE_GUIDE_H), (-BRIDGE_GUIDE_X, 0.0), guide_bottom)
    guides = guides.union(
        _box((BRIDGE_GUIDE_W, BRIDGE_GUIDE_L, BRIDGE_GUIDE_H), (BRIDGE_GUIDE_X, 0.0), guide_bottom)
    )
    return guides


def _y_carriage_shape() -> cq.Workplane:
    slide = _box((Y_SLIDE_L, Y_SLIDE_W, Y_SLIDE_H), (0.0, 0.0), Y_SLIDE_BOTTOM_Z)
    slide = slide.cut(
        _box(
            (Y_SLIDE_WINDOW_L, Y_SLIDE_WINDOW_W, Y_SLIDE_H + 0.004),
            (0.0, 0.0),
            Y_SLIDE_BOTTOM_Z - 0.001,
        )
    )

    outer_width = Z_GUIDE_SLOT_L + 2.0 * COLUMN_T
    column_x = Z_GUIDE_SLOT_L / 2.0 + COLUMN_T / 2.0
    rear_web_y = -(COLUMN_D - REAR_WEB_T) / 2.0

    left_column = _box((COLUMN_T, COLUMN_D, COLUMN_H), (-column_x, 0.0), Y_HOUSING_BOTTOM_Z)
    right_column = _box((COLUMN_T, COLUMN_D, COLUMN_H), (column_x, 0.0), Y_HOUSING_BOTTOM_Z)
    top_cap = _box((outer_width, COLUMN_D, TOP_CAP_H), (0.0, 0.0), Y_HOUSING_BOTTOM_Z + COLUMN_H)

    side_covers = _box(
        (Y_COVER_T, COLUMN_D, Y_COVER_H),
        (-(outer_width / 2.0 + Y_COVER_T / 2.0), 0.0),
        Y_COVER_BOTTOM_Z,
    )
    side_covers = side_covers.union(
        _box(
            (Y_COVER_T, COLUMN_D, Y_COVER_H),
            ((outer_width / 2.0 + Y_COVER_T / 2.0), 0.0),
            Y_COVER_BOTTOM_Z,
        )
    )

    carriage = slide.union(left_column).union(right_column).union(top_cap).union(side_covers)
    carriage = (
        carriage.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.034, 0.046)
        .cutBlind(-0.003)
    )
    carriage = (
        carriage.faces("<X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.034, 0.046)
        .cutBlind(-0.003)
    )
    return carriage


def _z_ram_shape() -> cq.Workplane:
    guide = _box((RAM_GUIDE_L, RAM_GUIDE_W, RAM_GUIDE_H), (0.0, 0.0), -RAM_GUIDE_H)
    body = _box((RAM_BODY_L, RAM_BODY_W, RAM_BODY_H), (0.0, 0.0), -RAM_GUIDE_H - RAM_BODY_H)
    clamp_block = _box(
        (RAM_BLOCK_L, RAM_BLOCK_W, RAM_BLOCK_H),
        (0.0, 0.0),
        -RAM_GUIDE_H - RAM_BODY_H - RAM_BLOCK_H,
    )
    flange = _box(
        (RAM_FLANGE_L, RAM_FLANGE_W, RAM_FLANGE_H),
        (0.0, 0.0),
        -RAM_GUIDE_H - RAM_BODY_H - RAM_BLOCK_H - RAM_FLANGE_H,
    )
    hole_positions = (
        (-0.020, -0.012),
        (-0.020, 0.012),
        (0.020, -0.012),
        (0.020, 0.012),
    )
    for x_pos, y_pos in hole_positions:
        flange = flange.cut(
            cq.Workplane("XY")
            .cylinder(RAM_FLANGE_H + 0.004, 0.00225, centered=(True, True, False))
            .translate(
                (
                    x_pos,
                    y_pos,
                    -RAM_GUIDE_H - RAM_BODY_H - RAM_BLOCK_H - RAM_FLANGE_H - 0.002,
                )
            )
        )

    ram = guide.union(body).union(clamp_block).union(flange)
    ram = (
        ram.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(0.018, RAM_GUIDE_H + RAM_BODY_H - 0.006)
        .cutBlind(-0.0015)
    )
    ram = (
        ram.faces("<Y")
        .workplane(centerOption="CenterOfMass")
        .rect(0.018, RAM_GUIDE_H + RAM_BODY_H - 0.006)
        .cutBlind(-0.0015)
    )
    return ram


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_automation_stage")

    model.material("base_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("rail_steel", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("bridge_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("guide_steel", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("carriage_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("ram_gray", rgba=(0.48, 0.50, 0.54, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body"),
        material="base_gray",
        name="body",
    )
    base.visual(
        mesh_from_cadquery(_base_rail_shape(), "base_rails"),
        material="rail_steel",
        name="rails",
    )

    bridge = model.part("bridge_saddle")
    bridge.visual(
        mesh_from_cadquery(_bridge_body_shape(), "bridge_body"),
        material="bridge_aluminum",
        name="body",
    )
    bridge.visual(
        mesh_from_cadquery(_bridge_guides_shape(), "bridge_guides"),
        material="guide_steel",
        name="guides",
    )

    y_carriage = model.part("crosswise_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_y_carriage_shape(), "y_carriage_body"),
        material="carriage_aluminum",
        name="body",
    )

    z_ram = model.part("z_ram")
    z_ram.visual(
        mesh_from_cadquery(_z_ram_shape(), "z_ram_body"),
        material="ram_gray",
        name="body",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-X_TRAVEL, upper=X_TRAVEL, effort=450.0, velocity=0.30),
    )
    model.articulation(
        "bridge_to_crosswise",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-Y_TRAVEL, upper=Y_TRAVEL, effort=220.0, velocity=0.25),
    )
    model.articulation(
        "crosswise_to_z",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_ram,
        origin=Origin(xyz=(0.0, 0.0, Z_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=Z_TRAVEL, effort=180.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge_saddle")
    y_carriage = object_model.get_part("crosswise_carriage")
    z_ram = object_model.get_part("z_ram")

    x_axis = object_model.get_articulation("base_to_bridge")
    y_axis = object_model.get_articulation("bridge_to_crosswise")
    z_axis = object_model.get_articulation("crosswise_to_z")

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
        "all_primary_axes_are_prismatic",
        all(
            axis.articulation_type == ArticulationType.PRISMATIC
            for axis in (x_axis, y_axis, z_axis)
        ),
        "Expected X, Y, and Z motions to all be prismatic.",
    )
    ctx.check(
        "axis_directions_match_stage_stack",
        x_axis.axis == (1.0, 0.0, 0.0)
        and y_axis.axis == (0.0, 1.0, 0.0)
        and z_axis.axis == (0.0, 0.0, -1.0),
        f"Unexpected axes: X={x_axis.axis}, Y={y_axis.axis}, Z={z_axis.axis}",
    )

    ctx.expect_contact(
        base,
        bridge,
        elem_a="rails",
        contact_tol=0.001,
        name="bridge_saddle_is_supported_by_base_rails",
    )
    ctx.expect_contact(
        bridge,
        y_carriage,
        elem_a="guides",
        contact_tol=0.001,
        name="crosswise_carriage_is_supported_by_bridge_guides",
    )
    ctx.expect_contact(
        y_carriage,
        z_ram,
        contact_tol=0.001,
        name="z_ram_is_guided_inside_carriage_housing",
    )

    with ctx.pose({z_axis: z_axis.motion_limits.upper}):
        ctx.expect_gap(
            z_ram,
            base,
            axis="z",
            min_gap=0.008,
            name="extended_ram_keeps_clearance_over_base",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        name="sampled_articulation_motion_stays_clear",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
