from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

BODY_W = 0.285
BODY_D = 0.122
BODY_H = 0.102


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_bluetooth_speaker")

    housing = model.material("housing", rgba=(0.17, 0.18, 0.20, 1.0))
    trim = model.material("trim", rgba=(0.11, 0.12, 0.13, 1.0))
    grille = model.material("grille", rgba=(0.07, 0.07, 0.08, 1.0))
    grille_shadow = model.material("grille_shadow", rgba=(0.03, 0.03, 0.04, 1.0))
    control_bank = model.material("control_bank", rgba=(0.20, 0.21, 0.23, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    button_finish = model.material("button_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    rocker_finish = model.material("rocker_finish", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        Box((0.245, 0.094, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - 0.007)),
        material=trim,
        name="top_cap",
    )
    body.visual(
        Box((0.016, BODY_D * 0.88, 0.088)),
        origin=Origin(xyz=(-BODY_W * 0.5 + 0.008, 0.0, 0.047)),
        material=trim,
        name="left_endcap",
    )
    body.visual(
        Box((0.016, BODY_D * 0.88, 0.088)),
        origin=Origin(xyz=(BODY_W * 0.5 - 0.008, 0.0, 0.047)),
        material=trim,
        name="right_endcap",
    )
    body.visual(
        Box((0.167, 0.012, 0.078)),
        origin=Origin(xyz=(-0.044, -0.056, 0.053)),
        material=grille_shadow,
        name="speaker_cavity",
    )
    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.176, 0.090),
                0.0035,
                hole_diameter=0.0045,
                pitch=(0.010, 0.010),
                frame=0.009,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "speaker_front_grille",
        ),
        origin=Origin(xyz=(-0.044, -0.0605, 0.055,), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille,
        name="front_grille",
    )
    body.visual(
        Box((0.116, 0.010, 0.046)),
        origin=Origin(xyz=(0.071, -0.062, 0.070)),
        material=control_bank,
        name="control_bank",
    )
    body.visual(
        Box((0.102, 0.003, 0.033)),
        origin=Origin(xyz=(0.071, -0.067, 0.070)),
        material=trim,
        name="control_face",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(
            xyz=(-BODY_W * 0.5 - 0.0005, 0.0, 0.108),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim,
        name="left_handle_pivot",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(
            xyz=(BODY_W * 0.5 + 0.0005, 0.0, 0.108),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim,
        name="right_handle_pivot",
    )
    for index, (x, y) in enumerate(
        (
            (-0.104, -0.040),
            (0.104, -0.040),
            (-0.104, 0.040),
            (0.104, 0.040),
        )
    ):
        body.visual(
            Box((0.026, 0.018, 0.008)),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"foot_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(-0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_finish,
        name="left_barrel",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_finish,
        name="right_barrel",
    )
    handle.visual(
        Box((0.050, 0.032, 0.022)),
        origin=Origin(xyz=(-0.108, 0.022, 0.011)),
        material=handle_finish,
        name="left_arm",
    )
    handle.visual(
        Box((0.050, 0.032, 0.022)),
        origin=Origin(xyz=(0.108, 0.022, 0.011)),
        material=handle_finish,
        name="right_arm",
    )
    handle.visual(
        Box((0.196, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.040, 0.021)),
        material=handle_finish,
        name="grip_bar",
    )
    handle.visual(
        Box((0.154, 0.012, 0.007)),
        origin=Origin(xyz=(0.0, 0.040, 0.031)),
        material=rubber,
        name="grip_pad",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.4,
            lower=0.0,
            upper=1.18,
        ),
    )

    volume_wheel = model.part("volume_wheel")
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.025,
            0.014,
            body_style="cylindrical",
            grip=KnobGrip(style="fluted", count=22, depth=0.0009),
            center=False,
        ),
        "speaker_volume_wheel",
    )
    volume_wheel.visual(
        wheel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_finish,
        name="wheel",
    )
    model.articulation(
        "body_to_volume_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=volume_wheel,
        origin=Origin(xyz=(0.032, -0.0685, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.020, 0.010, 0.016)),
        origin=Origin(),
        material=rocker_finish,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.006, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, -0.005, 0.004)),
        material=button_finish,
        name="rocker_indicator",
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(0.069, -0.072, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    for index, x in enumerate((0.096, 0.117)):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.005, length=0.005),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0033, length=0.004),
            origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim,
            name="button_stem",
        )
        model.articulation(
            f"body_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.0715, 0.070)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=0.05,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def _element_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return float(mins[1] + maxs[1]) * 0.5


def _element_top_z(aabb) -> float | None:
    if aabb is None:
        return None
    _, maxs = aabb
    return float(maxs[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    body = object_model.get_part("body")
    wheel = object_model.get_part("volume_wheel")
    rocker = object_model.get_part("power_rocker")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")

    handle_joint = object_model.get_articulation("body_to_handle")
    wheel_joint = object_model.get_articulation("body_to_volume_wheel")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    menu_joint_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_joint_1 = object_model.get_articulation("body_to_menu_button_1")

    ctx.check(
        "wheel_joint_is_continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_joint.articulation_type!r}",
    )
    ctx.check(
        "rocker_joint_is_revolute",
        rocker_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={rocker_joint.articulation_type!r}",
    )
    ctx.check(
        "menu_buttons_are_prismatic",
        menu_joint_0.articulation_type == ArticulationType.PRISMATIC
        and menu_joint_1.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"menu0={menu_joint_0.articulation_type!r}, "
            f"menu1={menu_joint_1.articulation_type!r}"
        ),
    )

    ctx.expect_gap(
        rocker,
        wheel,
        axis="x",
        min_gap=0.010,
        max_gap=0.050,
        name="wheel stays visually separated from rocker",
    )
    ctx.expect_gap(
        menu_0,
        rocker,
        axis="x",
        min_gap=0.007,
        max_gap=0.040,
        name="rocker stays distinct from first menu button",
    )
    ctx.expect_gap(
        menu_1,
        menu_0,
        axis="x",
        min_gap=0.006,
        max_gap=0.040,
        name="menu buttons remain individually readable",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="grip_bar",
        negative_elem="top_cap",
        min_gap=0.001,
        name="handle grip clears the speaker top at rest",
    )

    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.upper is not None:
        rest_top = _element_top_z(ctx.part_element_world_aabb(handle, elem="grip_bar"))
        with ctx.pose({handle_joint: handle_limits.upper}):
            raised_top = _element_top_z(ctx.part_element_world_aabb(handle, elem="grip_bar"))
        ctx.check(
            "handle_lifts_upward",
            rest_top is not None and raised_top is not None and raised_top > rest_top + 0.016,
            details=f"rest_top={rest_top}, raised_top={raised_top}",
        )

    rocker_limits = rocker_joint.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            rocker_low = _element_center_y(
                ctx.part_element_world_aabb(rocker, elem="rocker_indicator")
            )
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_high = _element_center_y(
                ctx.part_element_world_aabb(rocker, elem="rocker_indicator")
            )
        ctx.check(
            "power_rocker_tilts_on_its_pivot",
            rocker_low is not None and rocker_high is not None and abs(rocker_high - rocker_low) > 0.002,
            details=f"rocker_low={rocker_low}, rocker_high={rocker_high}",
        )

    for joint, button, name in (
        (menu_joint_0, menu_0, "first menu button depresses inward"),
        (menu_joint_1, menu_1, "second menu button depresses inward"),
    ):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            name,
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
