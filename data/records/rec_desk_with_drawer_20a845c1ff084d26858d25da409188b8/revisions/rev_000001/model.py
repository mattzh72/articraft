from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_WIDTH = 1.84
DESK_DEPTH = 0.84
DESK_HEIGHT = 0.765
TOP_THICK = 0.04

PEDESTAL_WIDTH = 0.42
PEDESTAL_DEPTH = 0.76
PEDESTAL_HEIGHT = DESK_HEIGHT - TOP_THICK
PEDESTAL_CENTER_X = 0.67
CENTER_KNEE_WIDTH = 2.0 * (PEDESTAL_CENTER_X - PEDESTAL_WIDTH / 2.0)

SIDE_THICK = 0.02
BACK_THICK = 0.012
TOP_PANEL_THICK = 0.02
BOTTOM_PANEL_THICK = 0.02
TOE_KICK_HEIGHT = 0.08
TOE_KICK_RECESS = 0.06
KICK_PLATE_THICK = 0.02

DRAWER_FACE_GAP = 0.008
DRAWER_FRONT_THICK = 0.022
DRAWER_FRONT_PROUD = 0.008
DRAWER_BODY_SIDE_THICK = 0.008
DRAWER_BODY_BACK_THICK = 0.008
DRAWER_BODY_BOTTOM_THICK = 0.006

OPENING_BOTTOM_Z = TOE_KICK_HEIGHT + BOTTOM_PANEL_THICK
OPENING_TOP_Z = PEDESTAL_HEIGHT - TOP_PANEL_THICK
PEDESTAL_DRAWER_FRONT_HEIGHT = (
    OPENING_TOP_Z - OPENING_BOTTOM_Z - 4.0 * DRAWER_FACE_GAP
) / 3.0
PEDESTAL_DRAWER_FRONT_WIDTH = PEDESTAL_WIDTH - 2.0 * SIDE_THICK - 0.014
PEDESTAL_DRAWER_BODY_WIDTH = 0.332
PEDESTAL_DRAWER_BODY_DEPTH = 0.56
PEDESTAL_DRAWER_BODY_HEIGHT = 0.156
PEDESTAL_RUNNER_THICK = 0.012
PEDESTAL_RUNNER_HEIGHT = 0.012
PEDESTAL_RAIL_LENGTH = 0.56
PEDESTAL_DRAWER_TRAVEL = 0.32
PEDESTAL_DRAWER_CLOSED_Y = (
    -PEDESTAL_DEPTH / 2.0
    - DRAWER_FRONT_PROUD
    + DRAWER_FRONT_THICK
    + PEDESTAL_DRAWER_BODY_DEPTH / 2.0
)
PEDESTAL_RAIL_CENTER_X = PEDESTAL_WIDTH / 2.0 - SIDE_THICK - PEDESTAL_RUNNER_THICK / 2.0
PEDESTAL_RUNNER_CENTER_X = (
    PEDESTAL_DRAWER_BODY_WIDTH / 2.0 + PEDESTAL_RUNNER_THICK / 2.0
)

APRON_HEIGHT = 0.12
APRON_BOTTOM_Z = PEDESTAL_HEIGHT - APRON_HEIGHT
BRIDGE_OUTER_WIDTH = CENTER_KNEE_WIDTH
BRIDGE_DEPTH = 0.56
BRIDGE_SIDE_THICK = 0.02
BRIDGE_REAR_THICK = 0.02
BRIDGE_CENTER_Y = -PEDESTAL_DEPTH / 2.0 + BRIDGE_DEPTH / 2.0
BRIDGE_CENTER_Z = APRON_BOTTOM_Z + APRON_HEIGHT / 2.0

FRIEZE_DRAWER_FRONT_WIDTH = 0.85
FRIEZE_DRAWER_FRONT_HEIGHT = 0.095
FRIEZE_DRAWER_BODY_WIDTH = 0.832
FRIEZE_DRAWER_BODY_DEPTH = 0.46
FRIEZE_DRAWER_BODY_HEIGHT = 0.07
FRIEZE_RUNNER_THICK = 0.012
FRIEZE_RUNNER_HEIGHT = 0.012
FRIEZE_RAIL_LENGTH = 0.48
FRIEZE_DRAWER_TRAVEL = 0.28
FRIEZE_DRAWER_CLOSED_Y = (
    -PEDESTAL_DEPTH / 2.0
    - 0.007
    + DRAWER_FRONT_THICK
    + FRIEZE_DRAWER_BODY_DEPTH / 2.0
)
FRIEZE_RAIL_CENTER_X = (
    BRIDGE_OUTER_WIDTH / 2.0 - BRIDGE_SIDE_THICK - FRIEZE_RUNNER_THICK / 2.0
)
FRIEZE_RUNNER_CENTER_X = FRIEZE_DRAWER_BODY_WIDTH / 2.0 + FRIEZE_RUNNER_THICK / 2.0

HANDLE_DEPTH = 0.018
PEDESTAL_HANDLE_WIDTH = 0.14
FRIEZE_HANDLE_WIDTH = 0.18
HANDLE_HEIGHT = 0.012

DRAWER_LEVELS = ("bottom", "middle", "top")


def _add_box(part, *, name, size, xyz, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _pedestal_drawer_center_z(level_index: int) -> float:
    return (
        OPENING_BOTTOM_Z
        + DRAWER_FACE_GAP
        + PEDESTAL_DRAWER_FRONT_HEIGHT / 2.0
        + level_index * (PEDESTAL_DRAWER_FRONT_HEIGHT + DRAWER_FACE_GAP)
    )


def _build_pedestal_carcass(part, *, wood, rail_metal):
    side_x = PEDESTAL_WIDTH / 2.0 - SIDE_THICK / 2.0
    _add_box(
        part,
        name=f"{part.name}_left_side",
        size=(SIDE_THICK, PEDESTAL_DEPTH, PEDESTAL_HEIGHT),
        xyz=(-side_x, 0.0, PEDESTAL_HEIGHT / 2.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_right_side",
        size=(SIDE_THICK, PEDESTAL_DEPTH, PEDESTAL_HEIGHT),
        xyz=(side_x, 0.0, PEDESTAL_HEIGHT / 2.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_back",
        size=(PEDESTAL_WIDTH - 2.0 * SIDE_THICK, BACK_THICK, PEDESTAL_HEIGHT),
        xyz=(0.0, PEDESTAL_DEPTH / 2.0 - BACK_THICK / 2.0, PEDESTAL_HEIGHT / 2.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_top_panel",
        size=(PEDESTAL_WIDTH - 2.0 * SIDE_THICK, PEDESTAL_DEPTH - BACK_THICK, TOP_PANEL_THICK),
        xyz=(0.0, 0.0, PEDESTAL_HEIGHT - TOP_PANEL_THICK / 2.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_bottom_panel",
        size=(PEDESTAL_WIDTH - 2.0 * SIDE_THICK, PEDESTAL_DEPTH - BACK_THICK, BOTTOM_PANEL_THICK),
        xyz=(0.0, 0.0, TOE_KICK_HEIGHT + BOTTOM_PANEL_THICK / 2.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_kick_plate",
        size=(PEDESTAL_WIDTH - 2.0 * SIDE_THICK, KICK_PLATE_THICK, TOE_KICK_HEIGHT),
        xyz=(
            0.0,
            -PEDESTAL_DEPTH / 2.0 + TOE_KICK_RECESS + KICK_PLATE_THICK / 2.0,
            TOE_KICK_HEIGHT / 2.0,
        ),
        material=wood,
    )

    for index, level in enumerate(DRAWER_LEVELS):
        drawer_z = _pedestal_drawer_center_z(index)
        for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
            _add_box(
                part,
                name=f"{part.name}_{level}_{side_name}_rail",
                size=(PEDESTAL_RUNNER_THICK, PEDESTAL_RAIL_LENGTH, PEDESTAL_RUNNER_HEIGHT),
                xyz=(
                    x_sign * PEDESTAL_RAIL_CENTER_X,
                    PEDESTAL_DRAWER_CLOSED_Y,
                    drawer_z,
                ),
                material=rail_metal,
            )


def _build_box_drawer(
    part,
    *,
    front_width,
    front_height,
    body_width,
    body_depth,
    body_height,
    runner_center_x,
    runner_thick,
    runner_height,
    handle_width,
    wood,
    pull_metal,
    rail_metal,
):
    front_center_y = -body_depth / 2.0 - DRAWER_FRONT_THICK / 2.0
    handle_center_y = -body_depth / 2.0 - DRAWER_FRONT_THICK - HANDLE_DEPTH / 2.0

    _add_box(
        part,
        name=f"{part.name}_front",
        size=(front_width, DRAWER_FRONT_THICK, front_height),
        xyz=(0.0, front_center_y, 0.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_left_side",
        size=(DRAWER_BODY_SIDE_THICK, body_depth, body_height),
        xyz=(-body_width / 2.0 + DRAWER_BODY_SIDE_THICK / 2.0, 0.0, 0.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_right_side",
        size=(DRAWER_BODY_SIDE_THICK, body_depth, body_height),
        xyz=(body_width / 2.0 - DRAWER_BODY_SIDE_THICK / 2.0, 0.0, 0.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_back",
        size=(body_width - 2.0 * DRAWER_BODY_SIDE_THICK, DRAWER_BODY_BACK_THICK, body_height),
        xyz=(0.0, body_depth / 2.0 - DRAWER_BODY_BACK_THICK / 2.0, 0.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_bottom",
        size=(
            body_width - 2.0 * DRAWER_BODY_SIDE_THICK,
            body_depth,
            DRAWER_BODY_BOTTOM_THICK,
        ),
        xyz=(0.0, 0.0, -body_height / 2.0 + DRAWER_BODY_BOTTOM_THICK / 2.0),
        material=wood,
    )
    _add_box(
        part,
        name=f"{part.name}_left_runner",
        size=(runner_thick, body_depth, runner_height),
        xyz=(-runner_center_x, 0.0, 0.0),
        material=rail_metal,
    )
    _add_box(
        part,
        name=f"{part.name}_right_runner",
        size=(runner_thick, body_depth, runner_height),
        xyz=(runner_center_x, 0.0, 0.0),
        material=rail_metal,
    )
    _add_box(
        part,
        name=f"{part.name}_pull",
        size=(handle_width, HANDLE_DEPTH, HANDLE_HEIGHT),
        xyz=(0.0, handle_center_y, 0.0),
        material=pull_metal,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="partner_desk")

    wood = model.material("walnut", rgba=(0.36, 0.24, 0.16, 1.0))
    top_wood = model.material("walnut_top", rgba=(0.42, 0.28, 0.18, 1.0))
    rail_metal = model.material("drawer_runner", rgba=(0.66, 0.67, 0.69, 1.0))
    pull_metal = model.material("antique_brass", rgba=(0.63, 0.54, 0.34, 1.0))

    worktop = model.part("worktop")
    _add_box(
        worktop,
        name="top_slab",
        size=(DESK_WIDTH, DESK_DEPTH, TOP_THICK),
        xyz=(0.0, 0.0, DESK_HEIGHT - TOP_THICK / 2.0),
        material=top_wood,
    )

    left_pedestal = model.part("left_pedestal")
    _build_pedestal_carcass(left_pedestal, wood=wood, rail_metal=rail_metal)

    right_pedestal = model.part("right_pedestal")
    _build_pedestal_carcass(right_pedestal, wood=wood, rail_metal=rail_metal)

    bridge = model.part("bridge_frame")
    _add_box(
        bridge,
        name="left_web",
        size=(BRIDGE_SIDE_THICK, BRIDGE_DEPTH, APRON_HEIGHT),
        xyz=(
            -BRIDGE_OUTER_WIDTH / 2.0 + BRIDGE_SIDE_THICK / 2.0,
            BRIDGE_CENTER_Y,
            BRIDGE_CENTER_Z,
        ),
        material=wood,
    )
    _add_box(
        bridge,
        name="right_web",
        size=(BRIDGE_SIDE_THICK, BRIDGE_DEPTH, APRON_HEIGHT),
        xyz=(
            BRIDGE_OUTER_WIDTH / 2.0 - BRIDGE_SIDE_THICK / 2.0,
            BRIDGE_CENTER_Y,
            BRIDGE_CENTER_Z,
        ),
        material=wood,
    )
    _add_box(
        bridge,
        name="rear_stretcher",
        size=(BRIDGE_OUTER_WIDTH - 2.0 * BRIDGE_SIDE_THICK, BRIDGE_REAR_THICK, APRON_HEIGHT),
        xyz=(
            0.0,
            BRIDGE_CENTER_Y + BRIDGE_DEPTH / 2.0 - BRIDGE_REAR_THICK / 2.0,
            BRIDGE_CENTER_Z,
        ),
        material=wood,
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        _add_box(
            bridge,
            name=f"{side_name}_frieze_rail",
            size=(FRIEZE_RUNNER_THICK, FRIEZE_RAIL_LENGTH, FRIEZE_RUNNER_HEIGHT),
            xyz=(
                x_sign * FRIEZE_RAIL_CENTER_X,
                FRIEZE_DRAWER_CLOSED_Y,
                BRIDGE_CENTER_Z,
            ),
            material=rail_metal,
        )

    model.articulation(
        "worktop_to_left_pedestal",
        ArticulationType.FIXED,
        parent=worktop,
        child=left_pedestal,
        origin=Origin(xyz=(-PEDESTAL_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "worktop_to_right_pedestal",
        ArticulationType.FIXED,
        parent=worktop,
        child=right_pedestal,
        origin=Origin(xyz=(PEDESTAL_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "worktop_to_bridge_frame",
        ArticulationType.FIXED,
        parent=worktop,
        child=bridge,
        origin=Origin(),
    )

    pedestal_centers = {
        "bottom": _pedestal_drawer_center_z(0),
        "middle": _pedestal_drawer_center_z(1),
        "top": _pedestal_drawer_center_z(2),
    }
    for side_name, pedestal in (
        ("left", left_pedestal),
        ("right", right_pedestal),
    ):
        for level in ("top", "middle", "bottom"):
            drawer = model.part(f"{side_name}_{level}_drawer")
            _build_box_drawer(
                drawer,
                front_width=PEDESTAL_DRAWER_FRONT_WIDTH,
                front_height=PEDESTAL_DRAWER_FRONT_HEIGHT,
                body_width=PEDESTAL_DRAWER_BODY_WIDTH,
                body_depth=PEDESTAL_DRAWER_BODY_DEPTH,
                body_height=PEDESTAL_DRAWER_BODY_HEIGHT,
                runner_center_x=PEDESTAL_RUNNER_CENTER_X,
                runner_thick=PEDESTAL_RUNNER_THICK,
                runner_height=PEDESTAL_RUNNER_HEIGHT,
                handle_width=PEDESTAL_HANDLE_WIDTH,
                wood=wood,
                pull_metal=pull_metal,
                rail_metal=rail_metal,
            )
            model.articulation(
                f"{side_name}_{level}_drawer_slide",
                ArticulationType.PRISMATIC,
                parent=pedestal,
                child=drawer,
                origin=Origin(xyz=(0.0, PEDESTAL_DRAWER_CLOSED_Y, pedestal_centers[level])),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=80.0,
                    velocity=0.45,
                    lower=0.0,
                    upper=PEDESTAL_DRAWER_TRAVEL,
                ),
            )

    frieze_drawer = model.part("frieze_drawer")
    _build_box_drawer(
        frieze_drawer,
        front_width=FRIEZE_DRAWER_FRONT_WIDTH,
        front_height=FRIEZE_DRAWER_FRONT_HEIGHT,
        body_width=FRIEZE_DRAWER_BODY_WIDTH,
        body_depth=FRIEZE_DRAWER_BODY_DEPTH,
        body_height=FRIEZE_DRAWER_BODY_HEIGHT,
        runner_center_x=FRIEZE_RUNNER_CENTER_X,
        runner_thick=FRIEZE_RUNNER_THICK,
        runner_height=FRIEZE_RUNNER_HEIGHT,
        handle_width=FRIEZE_HANDLE_WIDTH,
        wood=wood,
        pull_metal=pull_metal,
        rail_metal=rail_metal,
    )
    model.articulation(
        "frieze_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=frieze_drawer,
        origin=Origin(xyz=(0.0, FRIEZE_DRAWER_CLOSED_Y, BRIDGE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.45,
            lower=0.0,
            upper=FRIEZE_DRAWER_TRAVEL,
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

    worktop = object_model.get_part("worktop")
    bridge = object_model.get_part("bridge_frame")
    left_pedestal = object_model.get_part("left_pedestal")
    right_pedestal = object_model.get_part("right_pedestal")

    pedestal_drawers = []
    drawer_parent_pairs = []
    for side_name, pedestal in (
        ("left", left_pedestal),
        ("right", right_pedestal),
    ):
        for level in ("top", "middle", "bottom"):
            drawer = object_model.get_part(f"{side_name}_{level}_drawer")
            pedestal_drawers.append(drawer)
            drawer_parent_pairs.append((drawer, pedestal))

    frieze_drawer = object_model.get_part("frieze_drawer")
    drawer_parent_pairs.append((frieze_drawer, bridge))

    for part in [worktop, bridge, left_pedestal, right_pedestal, *pedestal_drawers, frieze_drawer]:
        ctx.check(f"{part.name}_present", part is not None, f"missing part {part.name}")

    ctx.expect_contact(worktop, left_pedestal, name="left_pedestal_meets_worktop")
    ctx.expect_contact(worktop, right_pedestal, name="right_pedestal_meets_worktop")
    ctx.expect_contact(worktop, bridge, name="bridge_meets_worktop")
    ctx.expect_contact(bridge, left_pedestal, name="bridge_meets_left_pedestal")
    ctx.expect_contact(bridge, right_pedestal, name="bridge_meets_right_pedestal")

    for drawer, parent in drawer_parent_pairs:
        ctx.expect_contact(drawer, parent, name=f"{drawer.name}_supported_by_parent")
        ctx.expect_overlap(drawer, parent, axes="xz", min_overlap=0.07, name=f"{drawer.name}_aligned_to_parent")
        ctx.expect_within(drawer, parent, axes="xz", margin=0.04, name=f"{drawer.name}_within_parent_span")

    for joint_name in (
        "left_top_drawer_slide",
        "left_middle_drawer_slide",
        "left_bottom_drawer_slide",
        "right_top_drawer_slide",
        "right_middle_drawer_slide",
        "right_bottom_drawer_slide",
        "frieze_drawer_slide",
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_is_prismatic",
            joint.joint_type == ArticulationType.PRISMATIC,
            f"{joint_name} should be prismatic, got {joint.joint_type}",
        )
        ctx.check(
            f"{joint_name}_axis_faces_forward",
            tuple(round(value, 6) for value in joint.axis) == (0.0, -1.0, 0.0),
            f"{joint_name} axis should be (0, -1, 0), got {joint.axis}",
        )
        ctx.check(
            f"{joint_name}_travel_limits",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.28,
            f"{joint_name} should have realistic forward travel, got {limits}",
        )

    with ctx.pose(
        {
            "left_top_drawer_slide": PEDESTAL_DRAWER_TRAVEL,
            "right_top_drawer_slide": PEDESTAL_DRAWER_TRAVEL,
            "frieze_drawer_slide": FRIEZE_DRAWER_TRAVEL,
        }
    ):
        left_top = object_model.get_part("left_top_drawer")
        right_top = object_model.get_part("right_top_drawer")
        ctx.fail_if_parts_overlap_in_current_pose(name="open_drawers_clearance")
        ctx.expect_origin_gap(
            left_pedestal,
            left_top,
            axis="y",
            min_gap=0.35,
            name="left_top_drawer_opens_forward",
        )
        ctx.expect_origin_gap(
            right_pedestal,
            right_top,
            axis="y",
            min_gap=0.35,
            name="right_top_drawer_opens_forward",
        )
        ctx.expect_origin_gap(
            bridge,
            frieze_drawer,
            axis="y",
            min_gap=0.38,
            name="frieze_drawer_opens_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
