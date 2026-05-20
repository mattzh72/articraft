from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_AXIS_Z = 0.03
SLEEVE_TOP_Z = 0.194
HINGE_Z_ON_MAST = 0.18
DISPLAY_WIDTH = 0.546
DISPLAY_HEIGHT = 0.346
DISPLAY_DEPTH = 0.042
SCREEN_WIDTH = 0.49
SCREEN_HEIGHT = 0.274
SCREEN_CENTER_Z = 0.042
CONTROL_Y = -0.038
CONTROL_Z = -0.133
ROCKER_X = 0.15
BUTTON_XS = (0.066, 0.094, 0.122)


def _pedestal_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(0.24, 0.18, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.03)
        .faces(">Z")
        .edges()
        .fillet(0.008)
    )
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .box(0.18, 0.125, 0.01, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )
    swivel_disk = (
        cq.Workplane("XY")
        .workplane(offset=0.024)
        .circle(0.04)
        .extrude(0.006)
    )
    return foot.union(shoulder).union(swivel_disk)


def _sleeve_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .box(0.078, 0.05, 0.028, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.01)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )
    sleeve = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .box(0.058, 0.034, 0.182, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.007)
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.025)
        .box(0.042, 0.022, 0.169, centered=(True, True, False))
    )
    return collar.union(sleeve).cut(cavity)


def _mast_post_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.042, 0.02, 0.294, centered=(True, True, False))
        .translate((0.0, 0.012, -0.142))
        .edges("|Z")
        .fillet(0.004)
    )


def _mast_head_shape() -> cq.Workplane:
    spine = (
        cq.Workplane("XY")
        .box(0.05, 0.032, 0.072, centered=(True, True, False))
        .translate((0.0, 0.024, 0.118))
        .edges("|Z")
        .fillet(0.004)
    )
    crosshead = (
        cq.Workplane("XY")
        .box(0.082, 0.014, 0.024, centered=(True, True, True))
        .translate((0.0, 0.034, 0.172))
    )
    ear_0 = (
        cq.Workplane("XY")
        .box(0.012, 0.014, 0.042, centered=(True, True, True))
        .translate((-0.032, 0.02, HINGE_Z_ON_MAST))
    )
    ear_1 = (
        cq.Workplane("XY")
        .box(0.012, 0.014, 0.042, centered=(True, True, True))
        .translate((0.032, 0.02, HINGE_Z_ON_MAST))
    )
    return spine.union(crosshead).union(ear_0).union(ear_1)


def _display_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(DISPLAY_WIDTH, DISPLAY_DEPTH, DISPLAY_HEIGHT, centered=(True, True, True))
        .translate((0.0, -0.03, 0.04))
        .edges("|Z")
        .fillet(0.014)
    )
    back_bulge = (
        cq.Workplane("XY")
        .box(0.18, 0.014, 0.08, centered=(True, True, True))
        .translate((0.0, -0.002, 0.03))
        .edges("|Z")
        .fillet(0.004)
    )
    trunnion = (
        cq.Workplane("XY")
        .box(0.05, 0.012, 0.012, centered=(True, True, True))
        .translate((0.0, 0.007, 0.0))
    )
    rocker_well = (
        cq.Workplane("XY")
        .box(0.022, 0.01, 0.008, centered=(True, True, False))
        .translate((ROCKER_X, CONTROL_Y, CONTROL_Z))
    )
    button_well = None
    for idx, x_pos in enumerate(BUTTON_XS):
        this_well = (
            cq.Workplane("XY")
            .circle(0.006)
            .extrude(0.008)
            .translate((x_pos, CONTROL_Y, CONTROL_Z))
        )
        button_well = this_well if idx == 0 else button_well.union(this_well)
    return shell.union(back_bulge).union(trunnion).cut(rocker_well).cut(button_well)


def _power_rocker_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(0.03, 0.016, 0.006, centered=(True, True, True))
        .translate((0.0, 0.0, -0.003))
    )
    tongue = (
        cq.Workplane("XY")
        .box(0.018, 0.01, 0.006, centered=(True, True, True))
        .translate((0.0, 0.0, 0.001))
    )
    return cap.union(tongue)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    shell_black = model.material("shell_black", rgba=(0.12, 0.13, 0.14, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.40, 0.42, 0.44, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.05, 0.06, 0.07, 1.0))
    control_black = model.material("control_black", rgba=(0.09, 0.09, 0.10, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_base_shape(), "pedestal"),
        material=stand_gray,
        name="pedestal_body",
    )

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shape(), "sleeve"),
        material=stand_gray,
        name="sleeve_body",
    )

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_mast_post_shape(), "mast_post"),
        material=stand_gray,
        name="mast_post",
    )
    mast.visual(
        mesh_from_cadquery(_mast_head_shape(), "mast_head"),
        material=stand_gray,
        name="mast_head",
    )

    display_shell = model.part("display_shell")
    display_shell.visual(
        mesh_from_cadquery(_display_shell_shape(), "display_shell"),
        material=shell_black,
        name="shell_body",
    )

    screen_panel = model.part("screen_panel")
    screen_panel.visual(
        Box((SCREEN_WIDTH, 0.002, SCREEN_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.05, SCREEN_CENTER_Z)),
        material=dark_glass,
        name="panel",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        mesh_from_cadquery(_power_rocker_shape(), "power_rocker"),
        material=control_black,
        name="rocker_cap",
    )

    for idx in range(3):
        button = model.part(f"menu_button_{idx}")
        button.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=control_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0022, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=control_black,
            name="button_stem",
        )

    model.articulation(
        "pedestal_to_sleeve",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=sleeve,
        origin=Origin(xyz=(0.0, 0.0, BASE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "sleeve_to_mast",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.07, effort=45.0, velocity=0.12),
    )
    model.articulation(
        "mast_to_display_shell",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=display_shell,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z_ON_MAST)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.18, upper=0.45, effort=6.0, velocity=1.5),
    )
    model.articulation(
        "display_shell_to_screen_panel",
        ArticulationType.FIXED,
        parent=display_shell,
        child=screen_panel,
        origin=Origin(),
    )
    model.articulation(
        "display_shell_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=display_shell,
        child=power_rocker,
        origin=Origin(xyz=(ROCKER_X, CONTROL_Y, CONTROL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.2, upper=0.2, effort=1.0, velocity=2.0),
    )
    for idx, x_pos in enumerate(BUTTON_XS):
        model.articulation(
            f"display_shell_to_menu_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=display_shell,
            child=model.get_part(f"menu_button_{idx}"),
            origin=Origin(xyz=(x_pos, CONTROL_Y, CONTROL_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.0035, effort=1.0, velocity=0.02),
        )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        0.5 * (lo[0] + hi[0]),
        0.5 * (lo[1] + hi[1]),
        0.5 * (lo[2] + hi[2]),
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    mast = object_model.get_part("mast")
    screen_panel = object_model.get_part("screen_panel")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    menu_button_2 = object_model.get_part("menu_button_2")

    ctx.allow_overlap(
        sleeve,
        mast,
        elem_a="sleeve_body",
        elem_b="mast_post",
        reason="The telescoping neck is represented as a tight sliding sleeve proxy around the inner mast.",
    )

    swivel = object_model.get_articulation("pedestal_to_sleeve")
    slide = object_model.get_articulation("sleeve_to_mast")
    tilt = object_model.get_articulation("mast_to_display_shell")
    rocker_joint = object_model.get_articulation("display_shell_to_power_rocker")
    button_joint_0 = object_model.get_articulation("display_shell_to_menu_button_0")
    button_joint_1 = object_model.get_articulation("display_shell_to_menu_button_1")

    slide_limits = slide.motion_limits
    tilt_limits = tilt.motion_limits
    rocker_limits = rocker_joint.motion_limits
    button_0_limits = button_joint_0.motion_limits

    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({slide: 0.0}):
            ctx.expect_within(
                mast,
                sleeve,
                axes="xy",
                inner_elem="mast_post",
                outer_elem="sleeve_body",
                margin=0.003,
                name="mast stays centered in sleeve at rest",
            )
            ctx.expect_overlap(
                mast,
                sleeve,
                axes="z",
                elem_a="mast_post",
                elem_b="sleeve_body",
                min_overlap=0.13,
                name="mast remains deeply inserted at rest",
            )

        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_within(
                mast,
                sleeve,
                axes="xy",
                inner_elem="mast_post",
                outer_elem="sleeve_body",
                margin=0.003,
                name="mast stays centered when raised",
            )
            ctx.expect_overlap(
                mast,
                sleeve,
                axes="z",
                elem_a="mast_post",
                elem_b="sleeve_body",
                min_overlap=0.06,
                name="mast retains insertion when raised",
            )

        rest_center = None
        raised_center = None
        with ctx.pose({slide: 0.0}):
            rest_center = _aabb_center(ctx.part_world_aabb(screen_panel))
        with ctx.pose({slide: slide_limits.upper}):
            raised_center = _aabb_center(ctx.part_world_aabb(screen_panel))
        ctx.check(
            "screen rises with height adjustment",
            rest_center is not None and raised_center is not None and raised_center[2] > rest_center[2] + 0.05,
            details=f"rest={rest_center}, raised={raised_center}",
        )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        forward_aabb = None
        rearward_aabb = None
        with ctx.pose({tilt: tilt_limits.lower}):
            forward_aabb = ctx.part_world_aabb(screen_panel)
        with ctx.pose({tilt: tilt_limits.upper}):
            rearward_aabb = ctx.part_world_aabb(screen_panel)
        ctx.check(
            "screen tilts backward at upper limit",
            forward_aabb is not None
            and rearward_aabb is not None
            and rearward_aabb[1][1] > forward_aabb[1][1] + 0.06,
            details=f"forward={forward_aabb}, rearward={rearward_aabb}",
        )

    rest_center = None
    swiveled_center = None
    with ctx.pose({swivel: 0.0}):
        rest_center = _aabb_center(ctx.part_world_aabb(screen_panel))
    with ctx.pose({swivel: 1.0}):
        swiveled_center = _aabb_center(ctx.part_world_aabb(screen_panel))
    ctx.check(
        "screen assembly swivels about the pedestal",
        rest_center is not None
        and swiveled_center is not None
        and abs(swiveled_center[0] - rest_center[0]) > 0.02
        and abs(swiveled_center[1] - rest_center[1]) > 0.015,
        details=f"rest={rest_center}, swiveled={swiveled_center}",
    )

    ctx.expect_origin_distance(
        power_rocker,
        menu_button_2,
        axes="x",
        min_dist=0.022,
        name="power rocker stays distinct from the menu button cluster",
    )

    if button_0_limits is not None and button_0_limits.upper is not None:
        rest_button_0 = None
        rest_button_1 = None
        pressed_button_0 = None
        pressed_button_1 = None
        with ctx.pose({button_joint_0: 0.0, button_joint_1: 0.0}):
            rest_button_0 = ctx.part_world_position(menu_button_0)
            rest_button_1 = ctx.part_world_position(menu_button_1)
        with ctx.pose({button_joint_0: button_0_limits.upper, button_joint_1: 0.0}):
            pressed_button_0 = ctx.part_world_position(menu_button_0)
            pressed_button_1 = ctx.part_world_position(menu_button_1)
        ctx.check(
            "menu buttons depress independently",
            rest_button_0 is not None
            and pressed_button_0 is not None
            and rest_button_1 is not None
            and pressed_button_1 is not None
            and pressed_button_0[2] > rest_button_0[2] + 0.0025
            and abs(pressed_button_1[2] - rest_button_1[2]) < 0.0005,
            details=(
                f"button_0_rest={rest_button_0}, button_0_pressed={pressed_button_0}, "
                f"button_1_rest={rest_button_1}, button_1_pressed={pressed_button_1}"
            ),
        )

    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        rocker_low = None
        rocker_high = None
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            rocker_low = _aabb_center(ctx.part_world_aabb(power_rocker))
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_high = _aabb_center(ctx.part_world_aabb(power_rocker))
        ctx.check(
            "power rocker tips on its local pivot",
            rocker_low is not None
            and rocker_high is not None
            and abs(rocker_high[1] - rocker_low[1]) > 0.0003,
            details=f"low={rocker_low}, high={rocker_high}",
        )

    return ctx.report()


object_model = build_object_model()
