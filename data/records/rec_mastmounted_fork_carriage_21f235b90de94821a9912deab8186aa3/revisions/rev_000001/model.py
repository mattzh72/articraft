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


MAST_HEIGHT = 1.23
RAIL_CENTER_X = 0.24
CHANNEL_W = 0.07
CHANNEL_D = 0.09
CHANNEL_WALL = 0.008

TOP_CROSSHEAD_T = 0.08
TOP_CROSSHEAD_D = 0.075
LOWER_BRACE_T = 0.055
LOWER_BRACE_D = 0.028

CARRIAGE_W = 0.366
CARRIAGE_BODY_D = 0.020
CARRIAGE_BODY_H = 0.30
CARRIAGE_TOP_BEAM_D = 0.050
CARRIAGE_TOP_BEAM_H = 0.050
CARRIAGE_BOTTOM_BEAM_D = 0.060
CARRIAGE_BOTTOM_BEAM_H = 0.055
GUIDE_SHOE_W = 0.024
GUIDE_SHOE_D = 0.046
GUIDE_SHOE_H = 0.24

BACKREST_BAR_W = 0.022
BACKREST_BAR_D = 0.018
BACKREST_BAR_H = 0.17
BACKREST_TOP_W = 0.25
BACKREST_TOP_H = 0.020

FORK_X_OFFSET = 0.125
FORK_WIDTH = 0.078
FORK_LENGTH = 0.335
FORK_THICK = 0.028
FORK_TIP_THICK = 0.010
FORK_TAPER_LEN = 0.115
FORK_SHANK_D = 0.034
FORK_SHANK_H = 0.19
FORK_Z = -(CARRIAGE_BODY_H / 2.0) - 0.014

LIFT_HOME_Z = 0.26
LIFT_TRAVEL = 0.50


def _channel_shape(open_direction: int) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        CHANNEL_W,
        CHANNEL_D,
        MAST_HEIGHT,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CHANNEL_W - CHANNEL_WALL,
            CHANNEL_D - (2.0 * CHANNEL_WALL),
            MAST_HEIGHT + 0.004,
            centered=(True, True, False),
        )
        .translate((open_direction * (CHANNEL_WALL / 2.0), 0.0, -0.002))
    )
    return outer.cut(inner)


def _mast_upright_shape(side: int) -> cq.Workplane:
    return _channel_shape(-side).translate((side * RAIL_CENTER_X, 0.0, 0.0))


def _top_crosshead_shape() -> cq.Workplane:
    overall_width = (2.0 * RAIL_CENTER_X) + CHANNEL_W
    return cq.Workplane("XY").box(
        overall_width,
        TOP_CROSSHEAD_D,
        TOP_CROSSHEAD_T,
        centered=(True, True, False),
    ).translate((0.0, -0.004, MAST_HEIGHT - TOP_CROSSHEAD_T))


def _lower_brace_shape() -> cq.Workplane:
    overall_width = (2.0 * RAIL_CENTER_X) + CHANNEL_W
    return cq.Workplane("XY").box(
        overall_width,
        LOWER_BRACE_D,
        LOWER_BRACE_T,
        centered=(True, True, False),
    ).translate((0.0, -(CHANNEL_D / 2.0) + (LOWER_BRACE_D / 2.0), 0.055))


def _carriage_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(CARRIAGE_W, CARRIAGE_BODY_D, CARRIAGE_BODY_H)

    top_beam = cq.Workplane("XY").box(
        CARRIAGE_W * 0.95,
        CARRIAGE_TOP_BEAM_D,
        CARRIAGE_TOP_BEAM_H,
    ).translate((0.0, 0.0, (CARRIAGE_BODY_H / 2.0) - (CARRIAGE_TOP_BEAM_H / 2.0) - 0.004))

    bottom_beam = cq.Workplane("XY").box(
        CARRIAGE_W * 0.95,
        CARRIAGE_BOTTOM_BEAM_D,
        CARRIAGE_BOTTOM_BEAM_H,
    ).translate(
        (
            0.0,
            0.004,
            -(CARRIAGE_BODY_H / 2.0) + (CARRIAGE_BOTTOM_BEAM_H / 2.0) + 0.008,
        )
    )

    guide_x = (RAIL_CENTER_X - (CHANNEL_W / 2.0)) - (GUIDE_SHOE_W / 2.0)
    guide_y = (CHANNEL_D / 2.0) - (GUIDE_SHOE_D / 2.0)

    result = frame.union(top_beam).union(bottom_beam)
    for side in (-1.0, 1.0):
        for front_back in (-1.0, 1.0):
            shoe = cq.Workplane("XY").box(
                GUIDE_SHOE_W,
                GUIDE_SHOE_D,
                GUIDE_SHOE_H,
            ).translate((side * guide_x, front_back * guide_y, -0.010))
            result = result.union(shoe)

    bar_x = 0.098
    backrest_z = (CARRIAGE_BODY_H / 2.0) + (BACKREST_BAR_H / 2.0) - 0.010
    left_bar = cq.Workplane("XY").box(
        BACKREST_BAR_W,
        BACKREST_BAR_D,
        BACKREST_BAR_H,
    ).translate((-bar_x, 0.0, backrest_z))
    right_bar = cq.Workplane("XY").box(
        BACKREST_BAR_W,
        BACKREST_BAR_D,
        BACKREST_BAR_H,
    ).translate((bar_x, 0.0, backrest_z))
    top_bar = cq.Workplane("XY").box(
        BACKREST_TOP_W,
        BACKREST_BAR_D,
        BACKREST_TOP_H,
    ).translate((0.0, 0.0, (CARRIAGE_BODY_H / 2.0) + BACKREST_BAR_H - 0.020))

    return result.union(left_bar).union(right_bar).union(top_bar)


def _forks_shape() -> cq.Workplane:
    forks: cq.Workplane | None = None
    tine_y = (CARRIAGE_BODY_D / 2.0) + (FORK_LENGTH / 2.0) - 0.004
    shank_y = (CARRIAGE_BODY_D / 2.0) + (FORK_SHANK_D / 2.0) - 0.003
    shank_z = FORK_Z + (FORK_SHANK_H / 2.0) + (FORK_THICK / 2.0) - 0.004
    rear_y = -(FORK_LENGTH / 2.0)
    front_y = FORK_LENGTH / 2.0
    z_bottom = -(FORK_THICK / 2.0)
    z_top = FORK_THICK / 2.0
    z_tip_top = z_bottom + FORK_TIP_THICK

    for side in (-1.0, 1.0):
        tine = (
            cq.Workplane("YZ")
            .polyline(
                [
                    (rear_y, z_bottom),
                    (front_y, z_bottom),
                    (front_y, z_tip_top),
                    (front_y - FORK_TAPER_LEN, z_top),
                    (rear_y, z_top),
                ]
            )
            .close()
            .extrude(FORK_WIDTH)
            .translate((-FORK_WIDTH / 2.0, 0.0, 0.0))
            .translate((side * FORK_X_OFFSET, tine_y, FORK_Z))
        )
        shank = cq.Workplane("XY").box(
            FORK_WIDTH,
            FORK_SHANK_D,
            FORK_SHANK_H,
        ).translate((side * FORK_X_OFFSET, shank_y, shank_z))
        fork = tine.union(shank)
        forks = fork if forks is None else forks.union(fork)

    return forks


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="forklift_mast_module")

    model.material("mast_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("carriage_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("fork_steel", rgba=(0.09, 0.10, 0.11, 1.0))

    mast = model.part("mast_frame")
    mast.visual(
        mesh_from_cadquery(_mast_upright_shape(-1), "mast_left_upright"),
        material="mast_steel",
        name="left_upright",
    )
    mast.visual(
        mesh_from_cadquery(_mast_upright_shape(1), "mast_right_upright"),
        material="mast_steel",
        name="right_upright",
    )
    mast.visual(
        mesh_from_cadquery(_top_crosshead_shape(), "mast_top_crosshead"),
        material="mast_steel",
        name="top_crosshead",
    )
    mast.visual(
        mesh_from_cadquery(_lower_brace_shape(), "mast_lower_brace"),
        material="mast_steel",
        name="lower_brace",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_frame_shape(), "carriage_frame"),
        material="carriage_steel",
        name="carriage_frame",
    )
    carriage.visual(
        mesh_from_cadquery(_forks_shape(), "carriage_forks"),
        material="fork_steel",
        name="forks",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.35,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast_frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")
    top_crosshead = mast.get_visual("top_crosshead")
    left_upright = mast.get_visual("left_upright")
    forks = carriage.get_visual("forks")
    carriage_frame = carriage.get_visual("carriage_frame")

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
        "parts_present",
        mast is not None and carriage is not None,
        "Expected mast_frame and carriage parts to exist.",
    )
    ctx.check(
        "lift_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic joint; got type={lift.articulation_type} axis={lift.axis}.",
    )
    ctx.check(
        "lift_joint_limits",
        lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper == LIFT_TRAVEL,
        f"Expected lift travel 0.0..{LIFT_TRAVEL}, got {lift.motion_limits}.",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_contact(
            mast,
            carriage,
            name="carriage_guides_contact_mast",
        )
        ctx.expect_overlap(
            mast,
            carriage,
            axes="x",
            min_overlap=0.34,
            name="carriage_stays_between_uprights",
        )
        mast_left_aabb = ctx.part_element_world_aabb(mast, elem=left_upright)
        fork_aabb = ctx.part_element_world_aabb(carriage, elem=forks)
        if mast_left_aabb is not None and fork_aabb is not None:
            forward_projection = fork_aabb[1][1] - mast_left_aabb[1][1]
            ctx.check(
                "forks_project_forward_of_mast",
                forward_projection >= 0.24,
                f"Expected forks to project at least 0.24 m forward; got {forward_projection:.3f} m.",
            )
        else:
            ctx.fail("forks_project_forward_of_mast", "Could not resolve fork or mast visual AABB.")

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=top_crosshead,
            negative_elem=carriage_frame,
            min_gap=0.04,
            max_gap=0.12,
            name="raised_carriage_clears_crosshead",
        )

    with ctx.pose({lift: 0.0}):
        lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        upper_pos = ctx.part_world_position(carriage)

    if lower_pos is not None and upper_pos is not None:
        dz = upper_pos[2] - lower_pos[2]
        dx = abs(upper_pos[0] - lower_pos[0])
        dy = abs(upper_pos[1] - lower_pos[1])
        ctx.check(
            "carriage_lifts_along_mast_axis",
            abs(dz - LIFT_TRAVEL) <= 0.002 and dx <= 1e-6 and dy <= 1e-6,
            f"Expected pure {LIFT_TRAVEL:.3f} m vertical travel; got dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}.",
        )
    else:
        ctx.fail("carriage_lifts_along_mast_axis", "Could not resolve carriage world positions in both poses.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
