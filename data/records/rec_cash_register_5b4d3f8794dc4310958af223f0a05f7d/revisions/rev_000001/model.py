from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="restaurant_counter_register")

    housing = model.material("housing", rgba=(0.18, 0.19, 0.21, 1.0))
    trim = model.material("trim", rgba=(0.27, 0.28, 0.30, 1.0))
    keycap = model.material("keycap", rgba=(0.82, 0.83, 0.81, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.14, 0.15, 0.17, 1.0))
    accent = model.material("accent", rgba=(0.62, 0.64, 0.67, 1.0))
    glass = model.material("glass", rgba=(0.05, 0.06, 0.08, 1.0))

    body = model.part("body")

    # Low cash-drawer base.
    body.visual(
        Box((0.34, 0.36, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=housing,
        name="base_floor",
    )
    body.visual(
        Box((0.32, 0.012, 0.10)),
        origin=Origin(xyz=(-0.01, -0.174, 0.05)),
        material=housing,
        name="base_side_0",
    )
    body.visual(
        Box((0.32, 0.012, 0.10)),
        origin=Origin(xyz=(-0.01, 0.174, 0.05)),
        material=housing,
        name="base_side_1",
    )
    body.visual(
        Box((0.012, 0.348, 0.10)),
        origin=Origin(xyz=(-0.164, 0.0, 0.05)),
        material=housing,
        name="base_rear",
    )
    body.visual(
        Box((0.32, 0.34, 0.010)),
        origin=Origin(xyz=(-0.01, 0.0, 0.095)),
        material=housing,
        name="base_ceiling",
    )
    body.visual(
        Box((0.24, 0.013, 0.040)),
        origin=Origin(xyz=(-0.02, -0.1615, 0.045)),
        material=trim,
        name="drawer_guide_0",
    )
    body.visual(
        Box((0.24, 0.013, 0.040)),
        origin=Origin(xyz=(-0.02, 0.1615, 0.045)),
        material=trim,
        name="drawer_guide_1",
    )

    # Keypad deck and printer bay surround. The printer opening is real, not a seam.
    body.visual(
        Box((0.17, 0.34, 0.025)),
        origin=Origin(xyz=(0.085, 0.0, 0.1125)),
        material=housing,
        name="keyboard_slab",
    )
    body.visual(
        Box((0.10, 0.22, 0.008)),
        origin=Origin(xyz=(-0.05, 0.0, 0.104)),
        material=trim,
        name="printer_floor",
    )
    body.visual(
        Box((0.108, 0.035, 0.042)),
        origin=Origin(xyz=(-0.046, -0.1225, 0.129)),
        material=housing,
        name="printer_rail_0",
    )
    body.visual(
        Box((0.108, 0.035, 0.042)),
        origin=Origin(xyz=(-0.046, 0.1225, 0.129)),
        material=housing,
        name="printer_rail_1",
    )
    body.visual(
        Box((0.020, 0.28, 0.042)),
        origin=Origin(xyz=(-0.105, 0.0, 0.129)),
        material=housing,
        name="printer_rear_beam",
    )

    # Short center post for the operator display.
    body.visual(
        Box((0.040, 0.080, 0.025)),
        origin=Origin(xyz=(-0.085, 0.0, 0.1625)),
        material=trim,
        name="post_brace",
    )
    body.visual(
        Box((0.035, 0.070, 0.090)),
        origin=Origin(xyz=(-0.105, 0.0, 0.195)),
        material=trim,
        name="display_post",
    )
    body.visual(
        Box((0.020, 0.120, 0.006)),
        origin=Origin(xyz=(-0.105, 0.0, 0.243)),
        material=accent,
        name="display_post_cap",
    )

    # Keys mounted directly on the keyboard deck so the deck reads clearly.
    key_x = (0.032, 0.069, 0.106, 0.143)
    key_y = (-0.114, -0.038, 0.038, 0.114)
    for row, y in enumerate(key_y):
        for col, x in enumerate(key_x):
            body.visual(
                Box((0.028, 0.032, 0.007)),
                origin=Origin(xyz=(x, y, 0.1285)),
                material=keycap,
                name=f"key_{row}_{col}",
            )

    drawer = model.part("cash_drawer")
    drawer.visual(
        Box((0.26, 0.31, 0.055)),
        origin=Origin(xyz=(-0.13, 0.0, 0.0425)),
        material=trim,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.018, 0.332, 0.084)),
        origin=Origin(xyz=(0.009, 0.0, 0.052)),
        material=drawer_face,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.100, 0.014)),
        origin=Origin(xyz=(0.024, 0.0, 0.052)),
        material=accent,
        name="drawer_handle",
    )

    lid = model.part("printer_lid")
    lid.visual(
        Box((0.095, 0.208, 0.010)),
        origin=Origin(xyz=(0.0475, 0.0, -0.005)),
        material=housing,
        name="lid_panel",
    )
    lid.visual(
        Box((0.014, 0.080, 0.014)),
        origin=Origin(xyz=(0.088, 0.0, -0.012)),
        material=accent,
        name="lid_pull",
    )

    display = model.part("display")
    display.visual(
        Box((0.014, 0.120, 0.018)),
        origin=Origin(xyz=(0.007, 0.0, 0.009)),
        material=accent,
        name="display_mount",
    )
    display.visual(
        Box((0.046, 0.220, 0.140)),
        origin=Origin(xyz=(0.030, 0.0, 0.070)),
        material=drawer_face,
        name="display_body",
    )
    display.visual(
        Box((0.002, 0.190, 0.110)),
        origin=Origin(xyz=(0.054, 0.0, 0.074)),
        material=glass,
        name="screen_glass",
    )

    model.articulation(
        "body_to_cash_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.152, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.14),
    )
    model.articulation(
        "body_to_printer_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.095, 0.0, 0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_display",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display,
        origin=Origin(xyz=(-0.105, 0.0, 0.246)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.45, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("cash_drawer")
    lid = object_model.get_part("printer_lid")
    display = object_model.get_part("display")

    drawer_joint = object_model.get_articulation("body_to_cash_drawer")
    lid_joint = object_model.get_articulation("body_to_printer_lid")
    display_joint = object_model.get_articulation("body_to_display")

    drawer_limits = drawer_joint.motion_limits
    lid_limits = lid_joint.motion_limits
    display_limits = display_joint.motion_limits

    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_drawer = ctx.part_element_world_aabb(drawer, elem="drawer_front")
        rest_origin = ctx.part_world_position(drawer)
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                min_overlap=0.08,
                name="cash drawer stays aligned with the base opening",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                min_overlap=0.08,
                name="cash drawer keeps retained insertion at full extension",
            )
            extended_drawer = ctx.part_element_world_aabb(drawer, elem="drawer_front")
            extended_origin = ctx.part_world_position(drawer)
        rest_center = _aabb_center(rest_drawer)
        extended_center = _aabb_center(extended_drawer)
        ctx.check(
            "cash drawer extends forward",
            rest_center is not None
            and extended_center is not None
            and rest_origin is not None
            and extended_origin is not None
            and extended_center[0] > rest_center[0] + 0.10
            and extended_origin[0] > rest_origin[0] + 0.10,
            details=(
                f"rest_center={rest_center}, extended_center={extended_center}, "
                f"rest_origin={rest_origin}, extended_origin={extended_origin}"
            ),
        )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
        with ctx.pose({lid_joint: lid_limits.upper}):
            open_lid = ctx.part_element_world_aabb(lid, elem="lid_panel")
        closed_center = _aabb_center(closed_lid)
        open_center = _aabb_center(open_lid)
        ctx.check(
            "printer lid lifts upward from the rear hinge",
            closed_center is not None
            and open_center is not None
            and open_center[2] > closed_center[2] + 0.04,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    if display_limits is not None and display_limits.lower is not None and display_limits.upper is not None:
        with ctx.pose({display_joint: display_limits.lower}):
            forward_display = ctx.part_element_world_aabb(display, elem="display_body")
        with ctx.pose({display_joint: display_limits.upper}):
            back_display = ctx.part_element_world_aabb(display, elem="display_body")
        forward_center = _aabb_center(forward_display)
        back_center = _aabb_center(back_display)
        ctx.check(
            "operator display tilts about the post-top hinge",
            forward_center is not None
            and back_center is not None
            and forward_center[0] > back_center[0] + 0.015,
            details=f"forward_center={forward_center}, back_center={back_center}",
        )

    return ctx.report()


object_model = build_object_model()
