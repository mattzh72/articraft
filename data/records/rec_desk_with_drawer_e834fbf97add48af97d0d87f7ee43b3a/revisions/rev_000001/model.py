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
)


DESK_TOP_Z = 0.75
TOP_THICKNESS = 0.04
TOP_BOTTOM_Z = DESK_TOP_Z - TOP_THICKNESS

MAIN_SPAN_X = 1.60
MAIN_DEPTH_Y = 0.75
RETURN_WIDTH_X = 0.70
RETURN_SPAN_Y = 1.45

PEDESTAL_W = 0.46
PEDESTAL_D = 0.58
PEDESTAL_H = TOP_BOTTOM_Z
PEDESTAL_PANEL_T = 0.018
PEDESTAL_BACK_T = 0.012
DRAWER_OPENING_H = (
    PEDESTAL_H - 2.0 * PEDESTAL_PANEL_T - PEDESTAL_PANEL_T
) / 2.0
LOWER_DRAWER_CENTER_Z = PEDESTAL_PANEL_T + DRAWER_OPENING_H * 0.5
UPPER_DRAWER_CENTER_Z = PEDESTAL_H - PEDESTAL_PANEL_T - DRAWER_OPENING_H * 0.5
DRAWER_TRAVEL = 0.30


def _build_box_frame_leg(
    part,
    *,
    span_axis: str,
    outer_span: float,
    tube: float,
    height: float,
    material,
) -> None:
    offset = outer_span * 0.5 - tube * 0.5
    if span_axis == "y":
        part.visual(
            Box((tube, tube, height)),
            origin=Origin(xyz=(0.0, -offset, height * 0.5)),
            material=material,
            name="left_post",
        )
        part.visual(
            Box((tube, tube, height)),
            origin=Origin(xyz=(0.0, offset, height * 0.5)),
            material=material,
            name="right_post",
        )
        part.visual(
            Box((tube, outer_span, tube)),
            origin=Origin(xyz=(0.0, 0.0, height - tube * 0.5)),
            material=material,
            name="top_beam",
        )
    else:
        part.visual(
            Box((tube, tube, height)),
            origin=Origin(xyz=(-offset, 0.0, height * 0.5)),
            material=material,
            name="left_post",
        )
        part.visual(
            Box((tube, tube, height)),
            origin=Origin(xyz=(offset, 0.0, height * 0.5)),
            material=material,
            name="right_post",
        )
        part.visual(
            Box((outer_span, tube, tube)),
            origin=Origin(xyz=(0.0, 0.0, height - tube * 0.5)),
            material=material,
            name="top_beam",
        )


def _build_drawer(
    part,
    *,
    wood,
    metal,
    front_width: float = 0.392,
    front_height: float = 0.296,
    body_width: float = 0.376,
    body_depth: float = 0.46,
    body_height: float = 0.18,
) -> None:
    front_thickness = 0.018
    panel_half_h = front_height * 0.5
    side_thickness = 0.012
    bottom_thickness = 0.012
    rail_thickness = 0.006
    rail_height = 0.03
    rail_length = 0.42
    rail_bridge_thickness = 0.007
    rail_bridge_length = 0.10

    bottom_center_z = -panel_half_h + bottom_thickness * 0.5
    side_center_z = bottom_center_z + body_height * 0.5
    back_center_z = bottom_center_z + (body_height - bottom_thickness) * 0.5
    side_offset_x = body_width * 0.5 - side_thickness * 0.5

    body_center_y = -(front_thickness + body_depth * 0.5)
    back_center_y = -(front_thickness + body_depth - side_thickness * 0.5)
    rail_center_y = -(0.02 + rail_length * 0.5)

    part.visual(
        Box((front_width, front_thickness, front_height)),
        origin=Origin(xyz=(0.0, -front_thickness * 0.5, 0.0)),
        material=wood,
        name="front_panel",
    )
    part.visual(
        Box((body_width, body_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, body_center_y, bottom_center_z)),
        material=wood,
        name="bottom_panel",
    )
    part.visual(
        Box((side_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-side_offset_x, body_center_y, side_center_z)),
        material=wood,
        name="left_side",
    )
    part.visual(
        Box((side_thickness, body_depth, body_height)),
        origin=Origin(xyz=(side_offset_x, body_center_y, side_center_z)),
        material=wood,
        name="right_side",
    )
    part.visual(
        Box((body_width - 2.0 * side_thickness, side_thickness, body_height - bottom_thickness)),
        origin=Origin(xyz=(0.0, back_center_y, back_center_z)),
        material=wood,
        name="back_panel",
    )

    rail_offset_x = front_width * 0.5 + rail_thickness * 0.5 - 0.001
    part.visual(
        Box((rail_thickness, rail_length, rail_height)),
        origin=Origin(xyz=(-rail_offset_x, rail_center_y, 0.0)),
        material=metal,
        name="left_slide",
    )
    part.visual(
        Box((rail_thickness, rail_length, rail_height)),
        origin=Origin(xyz=(rail_offset_x, rail_center_y, 0.0)),
        material=metal,
        name="right_slide",
    )
    rail_bridge_offset_x = (
        side_offset_x + side_thickness * 0.5 + rail_bridge_thickness * 0.5
    )
    part.visual(
        Box((rail_bridge_thickness, rail_bridge_length, rail_height)),
        origin=Origin(xyz=(-rail_bridge_offset_x, rail_center_y, 0.0)),
        material=metal,
        name="left_slide_bridge",
    )
    part.visual(
        Box((rail_bridge_thickness, rail_bridge_length, rail_height)),
        origin=Origin(xyz=(rail_bridge_offset_x, rail_center_y, 0.0)),
        material=metal,
        name="right_slide_bridge",
    )

    handle_z = 0.0
    handle_standoff = 0.024
    handle_bar_y = handle_standoff + 0.004
    handle_half_span_x = 0.062
    part.visual(
        Cylinder(radius=0.004, length=handle_standoff),
        origin=Origin(
            xyz=(-handle_half_span_x, handle_standoff * 0.5, handle_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=metal,
        name="left_handle_post",
    )
    part.visual(
        Cylinder(radius=0.004, length=handle_standoff),
        origin=Origin(
            xyz=(handle_half_span_x, handle_standoff * 0.5, handle_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=metal,
        name="right_handle_post",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.18),
        origin=Origin(
            xyz=(0.0, handle_bar_y, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=metal,
        name="handle_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_l_desk")

    oak = model.material("oak", rgba=(0.60, 0.44, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    graphite = model.material("graphite", rgba=(0.30, 0.32, 0.35, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))

    worktop_profile = [
        (0.0, 0.0),
        (MAIN_SPAN_X, 0.0),
        (MAIN_SPAN_X, MAIN_DEPTH_Y),
        (RETURN_WIDTH_X, MAIN_DEPTH_Y),
        (RETURN_WIDTH_X, RETURN_SPAN_Y),
        (0.0, RETURN_SPAN_Y),
    ]
    worktop_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(worktop_profile, TOP_THICKNESS),
        "corner_worktop",
    )

    worktop = model.part("worktop")
    worktop.visual(
        worktop_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOP_BOTTOM_Z)),
        material=oak,
        name="desktop",
    )
    worktop.inertial = Inertial.from_geometry(
        Box((MAIN_SPAN_X, RETURN_SPAN_Y, TOP_THICKNESS)),
        mass=24.0,
        origin=Origin(xyz=(MAIN_SPAN_X * 0.5, RETURN_SPAN_Y * 0.5, TOP_BOTTOM_Z + TOP_THICKNESS * 0.5)),
    )

    main_leg = model.part("main_leg")
    _build_box_frame_leg(
        main_leg,
        span_axis="y",
        outer_span=0.62,
        tube=0.05,
        height=TOP_BOTTOM_Z,
        material=dark_steel,
    )
    main_leg.inertial = Inertial.from_geometry(
        Box((0.05, 0.62, TOP_BOTTOM_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_BOTTOM_Z * 0.5)),
    )

    return_leg = model.part("return_leg")
    _build_box_frame_leg(
        return_leg,
        span_axis="x",
        outer_span=0.62,
        tube=0.05,
        height=TOP_BOTTOM_Z,
        material=dark_steel,
    )
    return_leg.inertial = Inertial.from_geometry(
        Box((0.62, 0.05, TOP_BOTTOM_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_BOTTOM_Z * 0.5)),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((PEDESTAL_PANEL_T, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(xyz=(PEDESTAL_PANEL_T * 0.5, PEDESTAL_D * 0.5, PEDESTAL_H * 0.5)),
        material=graphite,
        name="left_side_panel",
    )
    pedestal.visual(
        Box((PEDESTAL_PANEL_T, PEDESTAL_D, PEDESTAL_H)),
        origin=Origin(
            xyz=(PEDESTAL_W - PEDESTAL_PANEL_T * 0.5, PEDESTAL_D * 0.5, PEDESTAL_H * 0.5)
        ),
        material=graphite,
        name="right_side_panel",
    )
    pedestal.visual(
        Box((PEDESTAL_W - 2.0 * PEDESTAL_PANEL_T, PEDESTAL_D, PEDESTAL_PANEL_T)),
        origin=Origin(
            xyz=(PEDESTAL_W * 0.5, PEDESTAL_D * 0.5, PEDESTAL_PANEL_T * 0.5)
        ),
        material=graphite,
        name="bottom_panel",
    )
    pedestal.visual(
        Box((PEDESTAL_W - 2.0 * PEDESTAL_PANEL_T, PEDESTAL_D, PEDESTAL_PANEL_T)),
        origin=Origin(
            xyz=(PEDESTAL_W * 0.5, PEDESTAL_D * 0.5, PEDESTAL_H - PEDESTAL_PANEL_T * 0.5)
        ),
        material=graphite,
        name="top_panel",
    )
    pedestal.visual(
        Box((PEDESTAL_W - 2.0 * PEDESTAL_PANEL_T, PEDESTAL_D, PEDESTAL_PANEL_T)),
        origin=Origin(xyz=(PEDESTAL_W * 0.5, PEDESTAL_D * 0.5, PEDESTAL_H * 0.5)),
        material=graphite,
        name="drawer_divider",
    )
    pedestal.visual(
        Box((PEDESTAL_W - 2.0 * PEDESTAL_PANEL_T, PEDESTAL_BACK_T, PEDESTAL_H - 2.0 * PEDESTAL_PANEL_T)),
        origin=Origin(
            xyz=(
                PEDESTAL_W * 0.5,
                PEDESTAL_BACK_T * 0.5,
                PEDESTAL_H * 0.5,
            )
        ),
        material=graphite,
        name="back_panel",
    )

    fixed_rail_x = 0.027
    fixed_rail_length = 0.43
    fixed_rail_y = 0.115 + fixed_rail_length * 0.5
    fixed_rail_h = 0.032
    fixed_rail_bridge_t = 0.006
    fixed_rail_bridge_y = 0.13 + 0.05
    for prefix, rail_z in (
        ("lower", LOWER_DRAWER_CENTER_Z),
        ("upper", UPPER_DRAWER_CENTER_Z),
    ):
        pedestal.visual(
            Box((0.006, fixed_rail_length, fixed_rail_h)),
            origin=Origin(xyz=(fixed_rail_x, fixed_rail_y, rail_z)),
            material=aluminum,
            name=f"{prefix}_left_rail",
        )
        pedestal.visual(
            Box((0.006, fixed_rail_length, fixed_rail_h)),
            origin=Origin(xyz=(PEDESTAL_W - fixed_rail_x, fixed_rail_y, rail_z)),
            material=aluminum,
            name=f"{prefix}_right_rail",
        )
        pedestal.visual(
            Box((fixed_rail_bridge_t, 0.10, fixed_rail_h)),
            origin=Origin(
                xyz=(
                    PEDESTAL_PANEL_T + fixed_rail_bridge_t * 0.5,
                    fixed_rail_bridge_y,
                    rail_z,
                )
            ),
            material=aluminum,
            name=f"{prefix}_left_rail_bridge",
        )
        pedestal.visual(
            Box((fixed_rail_bridge_t, 0.10, fixed_rail_h)),
            origin=Origin(
                xyz=(
                    PEDESTAL_W - PEDESTAL_PANEL_T - fixed_rail_bridge_t * 0.5,
                    fixed_rail_bridge_y,
                    rail_z,
                )
            ),
            material=aluminum,
            name=f"{prefix}_right_rail_bridge",
        )

    pedestal.inertial = Inertial.from_geometry(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        mass=28.0,
        origin=Origin(xyz=(PEDESTAL_W * 0.5, PEDESTAL_D * 0.5, PEDESTAL_H * 0.5)),
    )

    lower_drawer = model.part("lower_drawer")
    _build_drawer(lower_drawer, wood=oak, metal=aluminum)
    lower_drawer.inertial = Inertial.from_geometry(
        Box((0.41, 0.52, 0.30)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.26, -0.03)),
    )

    upper_drawer = model.part("upper_drawer")
    _build_drawer(upper_drawer, wood=oak, metal=aluminum)
    upper_drawer.inertial = Inertial.from_geometry(
        Box((0.41, 0.52, 0.30)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.26, -0.03)),
    )

    riser_mount = model.part("riser_mount")
    riser_mount.visual(
        Box((0.72, 0.07, 0.04)),
        origin=Origin(xyz=(0.0, 0.035, -0.10)),
        material=dark_steel,
        name="base_beam",
    )
    for name, x in (
        ("left_outer_plate", -0.34),
        ("left_inner_plate", -0.28),
        ("right_inner_plate", 0.28),
        ("right_outer_plate", 0.34),
    ):
        riser_mount.visual(
            Box((0.01, 0.06, 0.12)),
            origin=Origin(xyz=(x, 0.015, -0.06)),
            material=dark_steel,
            name=name,
        )
    riser_mount.visual(
        Box((0.68, 0.018, 0.02)),
        origin=Origin(xyz=(0.0, 0.015, -0.095)),
        material=dark_steel,
        name="rear_brace",
    )
    riser_mount.inertial = Inertial.from_geometry(
        Box((0.72, 0.07, 0.12)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.035, -0.06)),
    )

    monitor_shelf = model.part("monitor_shelf")
    monitor_shelf.visual(
        Box((0.78, 0.24, 0.024)),
        origin=Origin(xyz=(0.0, 0.12, 0.012)),
        material=oak,
        name="platform",
    )
    monitor_shelf.visual(
        Box((0.78, 0.02, 0.018)),
        origin=Origin(xyz=(0.0, 0.23, 0.021)),
        material=oak,
        name="front_lip",
    )
    monitor_shelf.visual(
        Box((0.70, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.035, 0.03)),
        material=graphite,
        name="rear_stiffener",
    )
    for name, x in (("left_hinge_barrel", -0.31), ("right_hinge_barrel", 0.31)):
        monitor_shelf.visual(
            Cylinder(radius=0.015, length=0.05),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=graphite,
            name=name,
        )
        monitor_shelf.visual(
            Box((0.02, 0.09, 0.07)),
            origin=Origin(xyz=(x, 0.04, -0.02)),
            material=graphite,
            name=f"{name}_gusset",
        )
    monitor_shelf.inertial = Inertial.from_geometry(
        Box((0.78, 0.24, 0.10)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.12, 0.01)),
    )

    model.articulation(
        "worktop_to_main_leg",
        ArticulationType.FIXED,
        parent=worktop,
        child=main_leg,
        origin=Origin(xyz=(1.48, 0.375, 0.0)),
    )
    model.articulation(
        "worktop_to_return_leg",
        ArticulationType.FIXED,
        parent=worktop,
        child=return_leg,
        origin=Origin(xyz=(0.35, 1.35, 0.0)),
    )
    model.articulation(
        "worktop_to_pedestal",
        ArticulationType.FIXED,
        parent=worktop,
        child=pedestal,
        origin=Origin(xyz=(0.12, 0.12, 0.0)),
    )
    model.articulation(
        "pedestal_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=lower_drawer,
        origin=Origin(xyz=(PEDESTAL_W * 0.5, PEDESTAL_D, LOWER_DRAWER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "pedestal_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=upper_drawer,
        origin=Origin(xyz=(PEDESTAL_W * 0.5, PEDESTAL_D, UPPER_DRAWER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "worktop_to_riser_mount",
        ArticulationType.FIXED,
        parent=worktop,
        child=riser_mount,
        origin=Origin(xyz=(0.93, 0.0, 0.87)),
    )
    model.articulation(
        "riser_mount_to_monitor_shelf",
        ArticulationType.REVOLUTE,
        parent=riser_mount,
        child=monitor_shelf,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    worktop = object_model.get_part("worktop")
    main_leg = object_model.get_part("main_leg")
    return_leg = object_model.get_part("return_leg")
    pedestal = object_model.get_part("pedestal")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_drawer = object_model.get_part("upper_drawer")
    riser_mount = object_model.get_part("riser_mount")
    monitor_shelf = object_model.get_part("monitor_shelf")
    lower_slide = object_model.get_articulation("pedestal_to_lower_drawer")
    upper_slide = object_model.get_articulation("pedestal_to_upper_drawer")
    shelf_tilt = object_model.get_articulation("riser_mount_to_monitor_shelf")

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

    ctx.expect_contact(main_leg, worktop, name="main leg frame meets worktop")
    ctx.expect_contact(return_leg, worktop, name="return leg frame meets worktop")
    ctx.expect_contact(pedestal, worktop, name="pedestal supports the worktop")
    ctx.expect_contact(riser_mount, worktop, name="monitor riser bracket mounts to the worktop")

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        ctx.expect_contact(
            lower_drawer,
            pedestal,
            elem_a="left_slide",
            elem_b="lower_left_rail",
            name="lower drawer left slide contacts its rail",
        )
        ctx.expect_overlap(
            lower_drawer,
            pedestal,
            axes="y",
            elem_a="left_slide",
            elem_b="lower_left_rail",
            min_overlap=0.35,
            name="lower drawer stays deeply engaged on its rail when closed",
        )
        ctx.expect_contact(
            upper_drawer,
            pedestal,
            elem_a="right_slide",
            elem_b="upper_right_rail",
            name="upper drawer right slide contacts its rail",
        )
        ctx.expect_overlap(
            upper_drawer,
            pedestal,
            axes="y",
            elem_a="right_slide",
            elem_b="upper_right_rail",
            min_overlap=0.35,
            name="upper drawer stays deeply engaged on its rail when closed",
        )
        ctx.expect_contact(
            monitor_shelf,
            riser_mount,
            elem_a="left_hinge_barrel",
            elem_b="left_inner_plate",
            name="monitor shelf hinge barrel sits in the riser clevis",
        )
        ctx.expect_gap(
            monitor_shelf,
            worktop,
            axis="z",
            positive_elem="platform",
            min_gap=0.11,
            name="monitor shelf platform clears the desk surface",
        )

    lower_closed = ctx.part_world_position(lower_drawer)
    upper_closed = ctx.part_world_position(upper_drawer)
    lower_open = None
    upper_open = None
    with ctx.pose({lower_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            lower_drawer,
            pedestal,
            axes="y",
            elem_a="left_slide",
            elem_b="lower_left_rail",
            min_overlap=0.08,
            name="lower drawer retains insertion at full extension",
        )
        lower_open = ctx.part_world_position(lower_drawer)
    with ctx.pose({upper_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            upper_drawer,
            pedestal,
            axes="y",
            elem_a="right_slide",
            elem_b="upper_right_rail",
            min_overlap=0.08,
            name="upper drawer retains insertion at full extension",
        )
        upper_open = ctx.part_world_position(upper_drawer)

    ctx.check(
        "lower drawer opens forward",
        lower_closed is not None
        and lower_open is not None
        and lower_open[1] > lower_closed[1] + 0.25,
        details=f"closed={lower_closed}, open={lower_open}",
    )
    ctx.check(
        "upper drawer opens forward",
        upper_closed is not None
        and upper_open is not None
        and upper_open[1] > upper_closed[1] + 0.25,
        details=f"closed={upper_closed}, open={upper_open}",
    )

    front_lip_closed = None
    front_lip_open = None
    with ctx.pose({shelf_tilt: 0.0}):
        front_lip_closed = ctx.part_element_world_aabb(monitor_shelf, elem="front_lip")
    with ctx.pose({shelf_tilt: 0.30}):
        ctx.expect_gap(
            monitor_shelf,
            worktop,
            axis="z",
            positive_elem="front_lip",
            min_gap=0.17,
            name="tilted monitor shelf front lip remains above the desktop",
        )
        front_lip_open = ctx.part_element_world_aabb(monitor_shelf, elem="front_lip")

    ctx.check(
        "monitor shelf tilts upward at the front edge",
        front_lip_closed is not None
        and front_lip_open is not None
        and front_lip_open[1][2] > front_lip_closed[1][2] + 0.04,
        details=f"closed={front_lip_closed}, open={front_lip_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
