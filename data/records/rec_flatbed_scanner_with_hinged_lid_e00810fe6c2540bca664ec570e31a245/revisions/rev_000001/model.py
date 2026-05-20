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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_W = 0.34
BODY_D = 0.14
BODY_H = 0.074
BODY_FLOOR_T = 0.005
BODY_WALL_T = 0.006

LID_W = 0.348
LID_D = 0.148
LID_T = 0.006
LID_FRONT_SKIRT_H = 0.008
LID_FRONT_SKIRT_D = 0.008
LID_CORNER_R = 0.014
LID_WINDOW_W = 0.166
LID_WINDOW_D = 0.028
LID_WINDOW_T = 0.0015

HINGE_RADIUS = 0.0045
HINGE_BARREL_L = 0.320
HINGE_COLLAR_L = 0.010

ASSIST_PIN_RADIUS = 0.0035
ASSIST_PIN_L = 0.006

SLOT_W = 0.302
SLOT_Z = 0.030
SLOT_H = 0.010
SLOT_BEZEL_W = 0.318
SLOT_BEZEL_H = 0.022
SLOT_BEZEL_T = 0.003

GUIDE_W = 0.034
GUIDE_D = 0.108
GUIDE_T = 0.004
GUIDE_Y = 0.020

TRAY_W = 0.294
TRAY_D = 0.170
TRAY_T = 0.004
TRAY_RUNNER_W = 0.020
TRAY_BRIDGE_D = 0.020
TRAY_INNER_RAIL_W = 0.008
TRAY_INNER_RAIL_D = 0.113
TRAY_HANDLE_W = 0.118
TRAY_HANDLE_D = 0.028
TRAY_HANDLE_T = 0.008
TRAY_HANDLE_R = 0.006
TRAY_TRAVEL = 0.090

LID_OPEN_LIMIT = 1.20


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="negative_film_scanner")

    body_color = model.material("body_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    lid_color = model.material("lid_graphite", rgba=(0.22, 0.23, 0.24, 1.0))
    tray_color = model.material("tray_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_color = model.material("hinge_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    glass_color = model.material("window_smoke", rgba=(0.18, 0.24, 0.28, 0.45))
    bezel_color = model.material("slot_bezel", rgba=(0.05, 0.05, 0.06, 1.0))

    lid_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(LID_W, LID_D, LID_CORNER_R),
            LID_T,
            center=True,
        ),
        "scanner_lid_panel",
    )
    tray_handle_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(TRAY_HANDLE_W, TRAY_HANDLE_D, TRAY_HANDLE_R),
            TRAY_HANDLE_T,
            center=True,
        ),
        "scanner_tray_handle",
    )

    body = model.part(
        "body",
        inertial=Inertial.from_geometry(
            Box((BODY_W, BODY_D, BODY_H)),
            mass=1.8,
            origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        ),
    )

    wall_h = BODY_H - BODY_FLOOR_T
    side_wall_z = BODY_FLOOR_T + wall_h / 2.0
    side_wall_x = BODY_W / 2.0 - BODY_WALL_T / 2.0
    rear_wall_y = BODY_D / 2.0 - BODY_WALL_T / 2.0
    front_face_y = -BODY_D / 2.0 + BODY_WALL_T / 2.0

    body.visual(
        Box((BODY_W, BODY_D, BODY_FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_FLOOR_T / 2.0)),
        material=body_color,
        name="floor",
    )
    body.visual(
        Box((BODY_WALL_T, BODY_D, wall_h)),
        origin=Origin(xyz=(-side_wall_x, 0.0, side_wall_z)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL_T, BODY_D, wall_h)),
        origin=Origin(xyz=(side_wall_x, 0.0, side_wall_z)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL_T, BODY_WALL_T, wall_h)),
        origin=Origin(xyz=(0.0, rear_wall_y, side_wall_z)),
        material=body_color,
        name="rear_wall",
    )

    cheek_w = (BODY_W - SLOT_W) / 2.0
    cheek_x = SLOT_W / 2.0 + cheek_w / 2.0
    slot_bottom = SLOT_Z - SLOT_H / 2.0
    slot_top = SLOT_Z + SLOT_H / 2.0
    lower_lip_h = slot_bottom - BODY_FLOOR_T
    upper_bridge_h = BODY_H - slot_top

    body.visual(
        Box((cheek_w, BODY_WALL_T, wall_h)),
        origin=Origin(xyz=(-cheek_x, front_face_y, side_wall_z)),
        material=body_color,
        name="front_left_cheek",
    )
    body.visual(
        Box((cheek_w, BODY_WALL_T, wall_h)),
        origin=Origin(xyz=(cheek_x, front_face_y, side_wall_z)),
        material=body_color,
        name="front_right_cheek",
    )
    body.visual(
        Box((SLOT_W, BODY_WALL_T, lower_lip_h)),
        origin=Origin(
            xyz=(0.0, front_face_y, BODY_FLOOR_T + lower_lip_h / 2.0),
        ),
        material=body_color,
        name="front_lower_lip",
    )
    body.visual(
        Box((SLOT_W, BODY_WALL_T, upper_bridge_h)),
        origin=Origin(xyz=(0.0, front_face_y, slot_top + upper_bridge_h / 2.0)),
        material=body_color,
        name="front_upper_bridge",
    )

    guide_x = BODY_W / 2.0 - BODY_WALL_T - GUIDE_W / 2.0
    guide_z = SLOT_Z - TRAY_T / 2.0 - GUIDE_T / 2.0
    body.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_T)),
        origin=Origin(xyz=(-guide_x, GUIDE_Y, guide_z)),
        material=body_color,
        name="left_guide_rail",
    )
    body.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_T)),
        origin=Origin(xyz=(guide_x, GUIDE_Y, guide_z)),
        material=body_color,
        name="right_guide_rail",
    )

    bezel_strip_w = (SLOT_BEZEL_W - SLOT_W) / 2.0
    bezel_strip_h = (SLOT_BEZEL_H - SLOT_H) / 2.0
    bezel_y = -BODY_D / 2.0 - SLOT_BEZEL_T / 2.0
    bezel_side_x = SLOT_W / 2.0 + bezel_strip_w / 2.0
    body.visual(
        Box((bezel_strip_w, SLOT_BEZEL_T, SLOT_BEZEL_H)),
        origin=Origin(xyz=(-bezel_side_x, bezel_y, SLOT_Z)),
        material=bezel_color,
        name="left_slot_bezel",
    )
    body.visual(
        Box((bezel_strip_w, SLOT_BEZEL_T, SLOT_BEZEL_H)),
        origin=Origin(xyz=(bezel_side_x, bezel_y, SLOT_Z)),
        material=bezel_color,
        name="right_slot_bezel",
    )
    body.visual(
        Box((SLOT_W, SLOT_BEZEL_T, bezel_strip_h)),
        origin=Origin(xyz=(0.0, bezel_y, SLOT_Z + SLOT_H / 2.0 + bezel_strip_h / 2.0)),
        material=bezel_color,
        name="upper_slot_bezel",
    )
    body.visual(
        Box((SLOT_W, SLOT_BEZEL_T, bezel_strip_h)),
        origin=Origin(xyz=(0.0, bezel_y, SLOT_Z - SLOT_H / 2.0 - bezel_strip_h / 2.0)),
        material=bezel_color,
        name="lower_slot_bezel",
    )

    collar_x = BODY_W / 2.0 - HINGE_COLLAR_L / 2.0
    hinge_y = BODY_D / 2.0
    hinge_z = BODY_H + HINGE_RADIUS
    cylinder_x_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_COLLAR_L),
        origin=Origin(
            xyz=(-collar_x, hinge_y, hinge_z),
            rpy=cylinder_x_axis.rpy,
        ),
        material=metal_color,
        name="left_hinge_collar",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_COLLAR_L),
        origin=Origin(
            xyz=(collar_x, hinge_y, hinge_z),
            rpy=cylinder_x_axis.rpy,
        ),
        material=metal_color,
        name="right_hinge_collar",
    )

    pin_x = BODY_W / 2.0 - 0.018
    pin_y = BODY_D / 2.0 - 0.006
    pin_z = BODY_H - ASSIST_PIN_L / 2.0
    body.visual(
        Cylinder(radius=ASSIST_PIN_RADIUS, length=ASSIST_PIN_L),
        origin=Origin(xyz=(-pin_x, pin_y, pin_z)),
        material=metal_color,
        name="left_assist_pin",
    )
    body.visual(
        Cylinder(radius=ASSIST_PIN_RADIUS, length=ASSIST_PIN_L),
        origin=Origin(xyz=(pin_x, pin_y, pin_z)),
        material=metal_color,
        name="right_assist_pin",
    )

    lid_panel_center_y = -(LID_D / 2.0 + HINGE_RADIUS)
    lid_panel_center_z = BODY_H + LID_T / 2.0 - hinge_z
    lid = model.part(
        "lid",
        inertial=Inertial.from_geometry(
            Box((LID_W, LID_D, LID_T)),
            mass=0.55,
            origin=Origin(xyz=(0.0, lid_panel_center_y, lid_panel_center_z)),
        ),
    )
    lid.visual(
        lid_panel_mesh,
        origin=Origin(xyz=(0.0, lid_panel_center_y, lid_panel_center_z)),
        material=lid_color,
        name="lid_panel",
    )
    lid.visual(
        Box((HINGE_BARREL_L, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.0075, -0.0015)),
        material=metal_color,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_BARREL_L),
        origin=Origin(rpy=cylinder_x_axis.rpy),
        material=metal_color,
        name="hinge_barrel",
    )
    lid.visual(
        Box((TRAY_HANDLE_W, LID_FRONT_SKIRT_D, LID_FRONT_SKIRT_H)),
        origin=Origin(
            xyz=(
                0.0,
                -LID_D - HINGE_RADIUS + LID_FRONT_SKIRT_D / 2.0,
                lid_panel_center_z + LID_T / 2.0 - LID_FRONT_SKIRT_H / 2.0,
            ),
        ),
        material=lid_color,
        name="front_skirt",
    )
    lid.visual(
        Box((LID_WINDOW_W, LID_WINDOW_D, LID_WINDOW_T)),
        origin=Origin(
            xyz=(
                0.0,
                lid_panel_center_y + 0.018,
                lid_panel_center_z + LID_T / 2.0 + LID_WINDOW_T / 2.0,
            ),
        ),
        material=glass_color,
        name="scan_window",
    )

    tray = model.part(
        "tray",
        inertial=Inertial.from_geometry(
            Box((TRAY_W, TRAY_D, TRAY_HANDLE_T)),
            mass=0.22,
            origin=Origin(xyz=(0.0, TRAY_D / 2.0 - 0.020, TRAY_HANDLE_T / 2.0)),
        ),
    )
    tray_runner_x = TRAY_W / 2.0 - TRAY_RUNNER_W / 2.0
    tray_frame_y = TRAY_D / 2.0 - 0.020
    tray.visual(
        Box((TRAY_RUNNER_W, TRAY_D, TRAY_T)),
        origin=Origin(xyz=(-tray_runner_x, tray_frame_y, 0.0)),
        material=tray_color,
        name="left_runner",
    )
    tray.visual(
        Box((TRAY_RUNNER_W, TRAY_D, TRAY_T)),
        origin=Origin(xyz=(tray_runner_x, tray_frame_y, 0.0)),
        material=tray_color,
        name="right_runner",
    )

    bridge_w = TRAY_W - 2.0 * TRAY_RUNNER_W
    tray.visual(
        Box((bridge_w, TRAY_BRIDGE_D, TRAY_T)),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=tray_color,
        name="front_bridge",
    )
    tray.visual(
        Box((bridge_w, TRAY_BRIDGE_D, TRAY_T)),
        origin=Origin(xyz=(0.0, 0.140, 0.0)),
        material=tray_color,
        name="rear_bridge",
    )

    inner_rail_x = 0.033
    inner_rail_y = 0.0735
    tray.visual(
        Box((TRAY_INNER_RAIL_W, TRAY_INNER_RAIL_D, TRAY_T)),
        origin=Origin(xyz=(-inner_rail_x, inner_rail_y, 0.0)),
        material=tray_color,
        name="left_film_rail",
    )
    tray.visual(
        Box((TRAY_INNER_RAIL_W, TRAY_INNER_RAIL_D, TRAY_T)),
        origin=Origin(xyz=(inner_rail_x, inner_rail_y, 0.0)),
        material=tray_color,
        name="right_film_rail",
    )
    tray.visual(
        tray_handle_mesh,
        origin=Origin(xyz=(0.0, -0.011, 0.0005)),
        material=tray_color,
        name="handle",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=LID_OPEN_LIMIT,
        ),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, SLOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.35,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_tray")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "parts_present",
        all(part is not None for part in (body, lid, tray)),
        "Expected body, lid, and tray parts to exist.",
    )
    ctx.check(
        "lid_hinge_axis",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        f"Expected lid hinge axis (-1, 0, 0), got {lid_hinge.axis!r}.",
    )
    ctx.check(
        "tray_slide_axis",
        tuple(tray_slide.axis) == (0.0, -1.0, 0.0),
        f"Expected tray slide axis (0, -1, 0), got {tray_slide.axis!r}.",
    )

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="hinge_barrel",
            elem_b="left_hinge_collar",
            name="lid_left_hinge_contact_closed",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="hinge_barrel",
            elem_b="right_hinge_collar",
            name="lid_right_hinge_contact_closed",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="left_wall",
            min_gap=0.0,
            max_gap=0.001,
            name="lid_closed_seated_on_body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.12,
            name="lid_closed_covers_scanner_body",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="left_runner",
            elem_b="left_guide_rail",
            name="tray_left_runner_supported_inserted",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="right_runner",
            elem_b="right_guide_rail",
            name="tray_right_runner_supported_inserted",
        )
        ctx.expect_within(
            tray,
            body,
            axes="x",
            margin=0.0,
            name="tray_inserted_within_body_width",
        )
        ctx.expect_origin_gap(
            body,
            tray,
            axis="y",
            min_gap=0.069,
            max_gap=0.071,
            name="tray_inserted_origin_at_front_slot",
        )

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper, tray_slide: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="hinge_barrel",
            elem_b="left_hinge_collar",
            name="lid_left_hinge_contact_open",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="hinge_barrel",
            elem_b="right_hinge_collar",
            name="lid_right_hinge_contact_open",
        )

    with ctx.pose({lid_hinge: 0.0, tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_contact(
            tray,
            body,
            elem_a="left_runner",
            elem_b="left_guide_rail",
            name="tray_left_runner_supported_extended",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="right_runner",
            elem_b="right_guide_rail",
            name="tray_right_runner_supported_extended",
        )
        ctx.expect_within(
            tray,
            body,
            axes="x",
            margin=0.0,
            name="tray_extended_within_body_width",
        )
        ctx.expect_origin_gap(
            body,
            tray,
            axis="y",
            min_gap=0.159,
            max_gap=0.161,
            name="tray_pulled_forward",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for joint in (lid_hinge, tray_slide):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_lower_no_overlap",
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{joint.name}_upper_no_overlap",
                )
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
