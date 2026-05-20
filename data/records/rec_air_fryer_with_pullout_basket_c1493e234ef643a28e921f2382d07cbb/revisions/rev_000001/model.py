from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_D = 0.335
BODY_W = 0.320
BODY_H = 0.400
BODY_WALL = 0.004

DRAWER_Y = 0.035
DRAWER_Z = 0.052
DRAWER_TRAVEL = 0.175
DRAWER_OPEN_W = 0.235
DRAWER_OPEN_H = 0.168
DRAWER_BODY_D = 0.232
DRAWER_W = 0.224
DRAWER_H = 0.133
DRAWER_FACE_W = 0.240
DRAWER_FACE_H = 0.176
DRAWER_WALL = 0.003

CAVITY_D = 0.228
CAVITY_W = 0.230
CAVITY_H = 0.215
CAVITY_WALL = 0.0025
CAVITY_Z = 0.072

BASKET_D = 0.200
BASKET_W = 0.210
BASKET_H = 0.098
BASKET_WALL = 0.0025

HANDLE_D = 0.054
HANDLE_W = 0.132
HANDLE_H = 0.040

POD_D = 0.038
POD_W = 0.068
POD_H = 0.235
POD_Y = -0.118
POD_Z = 0.118
POD_FACE_X = 0.030

EPS = 0.0008


def _add_open_shell(
    part,
    *,
    depth: float,
    width: float,
    height: float,
    wall: float,
    material,
    prefix: str,
    front_height: float | None = None,
) -> None:
    t = wall + EPS
    part.visual(
        Box((depth, width, t)),
        origin=Origin(xyz=(-depth * 0.5, 0.0, t * 0.5)),
        material=material,
        name=f"{prefix}_bottom",
    )
    part.visual(
        Box((depth + t, t, height)),
        origin=Origin(xyz=(-(depth - t) * 0.5, width * 0.5 - t * 0.5, height * 0.5)),
        material=material,
        name=f"{prefix}_left_wall",
    )
    part.visual(
        Box((depth + t, t, height)),
        origin=Origin(xyz=(-(depth - t) * 0.5, -width * 0.5 + t * 0.5, height * 0.5)),
        material=material,
        name=f"{prefix}_right_wall",
    )
    part.visual(
        Box((t, width, height)),
        origin=Origin(xyz=(-depth + t * 0.5, 0.0, height * 0.5)),
        material=material,
        name=f"{prefix}_rear_wall",
    )
    part.visual(
        Box((depth, width, t)),
        origin=Origin(xyz=(-depth * 0.5, 0.0, height - t * 0.5)),
        material=material,
        name=f"{prefix}_top",
    )
    if front_height is not None and front_height > 0.0:
        part.visual(
            Box((t, width, front_height)),
            origin=Origin(xyz=(t * 0.5, 0.0, front_height * 0.5)),
            material=material,
            name=f"{prefix}_front_wall",
        )


def _dial_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.040,
            0.024,
            body_style="skirted",
            top_diameter=0.032,
            skirt=KnobSkirt(0.048, 0.005, flare=0.05),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windowed_air_fryer")

    body_black = model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    basket_dark = model.material("basket_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.40, 0.52, 0.58, 0.35))
    button_black = model.material("button_black", rgba=(0.06, 0.07, 0.08, 1.0))

    body = model.part("body")
    t_body = BODY_WALL + EPS
    body.visual(
        Box((BODY_D - t_body, t_body, BODY_H - 0.002)),
        origin=Origin(xyz=(0.0, BODY_W * 0.5 - t_body * 0.5, (BODY_H - 0.002) * 0.5)),
        material=body_black,
        name="left_wall",
    )
    body.visual(
        Box((BODY_D - t_body, t_body, BODY_H - 0.002)),
        origin=Origin(xyz=(0.0, -BODY_W * 0.5 + t_body * 0.5, (BODY_H - 0.002) * 0.5)),
        material=body_black,
        name="right_wall",
    )
    body.visual(
        Box((t_body, BODY_W - 2.0 * t_body, BODY_H - 0.002)),
        origin=Origin(xyz=(-BODY_D * 0.5 + t_body * 0.5, 0.0, (BODY_H - 0.002) * 0.5)),
        material=body_black,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_D - t_body, BODY_W - 2.0 * t_body, t_body)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - t_body * 0.5)),
        material=body_black,
        name="top_shell",
    )
    body.visual(
        Box((BODY_D - 0.018, BODY_W - 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=body_black,
        name="base_plinth",
    )
    body.visual(
        Box((0.010, 0.230, 0.182)),
        origin=Origin(xyz=(BODY_D * 0.5 - 0.005, 0.040, 0.309)),
        material=body_black,
        name="front_upper",
    )
    body.visual(
        Box((0.010, 0.230, 0.056)),
        origin=Origin(xyz=(BODY_D * 0.5 - 0.005, 0.040, 0.028)),
        material=body_black,
        name="front_lower",
    )
    body.visual(
        Box((0.010, 0.014, 0.172)),
        origin=Origin(
            xyz=(
                BODY_D * 0.5 - 0.005,
                DRAWER_Y + DRAWER_OPEN_W * 0.5 + 0.007,
                DRAWER_Z + DRAWER_OPEN_H * 0.5,
            )
        ),
        material=body_black,
        name="front_jamb",
    )

    cavity = model.part("cavity")
    _add_open_shell(
        cavity,
        depth=CAVITY_D,
        width=CAVITY_W,
        height=CAVITY_H,
        wall=CAVITY_WALL,
        material=cavity_dark,
        prefix="cavity",
    )
    model.articulation(
        "body_to_cavity",
        ArticulationType.FIXED,
        parent=body,
        child=cavity,
        origin=Origin(xyz=(BODY_D * 0.5 - 0.012, DRAWER_Y, CAVITY_Z)),
    )

    control_pod = model.part("control_pod")
    pod_t = 0.006
    ring_t = 0.008
    control_pod.visual(
        Box((pod_t, POD_W, POD_H)),
        origin=Origin(xyz=(pod_t * 0.5, 0.0, POD_H * 0.5)),
        material=body_black,
        name="back_plate",
    )
    control_pod.visual(
        Box((POD_D, pod_t, POD_H)),
        origin=Origin(xyz=(POD_D * 0.5, POD_W * 0.5 - pod_t * 0.5, POD_H * 0.5)),
        material=body_black,
        name="side_wall_0",
    )
    control_pod.visual(
        Box((POD_D, pod_t, POD_H)),
        origin=Origin(xyz=(POD_D * 0.5, -POD_W * 0.5 + pod_t * 0.5, POD_H * 0.5)),
        material=body_black,
        name="side_wall_1",
    )
    control_pod.visual(
        Box((POD_D, POD_W - 2.0 * pod_t, pod_t)),
        origin=Origin(xyz=(POD_D * 0.5, 0.0, pod_t * 0.5)),
        material=body_black,
        name="bottom_shell",
    )
    control_pod.visual(
        Box((POD_D, POD_W - 2.0 * pod_t, pod_t)),
        origin=Origin(xyz=(POD_D * 0.5, 0.0, POD_H - pod_t * 0.5)),
        material=body_black,
        name="top_shell",
    )
    control_pod.visual(
        Box((ring_t, POD_W, 0.016)),
        origin=Origin(xyz=(POD_D - ring_t * 0.5, 0.0, 0.008)),
        material=trim_black,
        name="front_ring_bottom",
    )
    control_pod.visual(
        Box((ring_t, POD_W, 0.020)),
        origin=Origin(xyz=(POD_D - ring_t * 0.5, 0.0, POD_H - 0.010)),
        material=trim_black,
        name="front_ring_top",
    )
    control_pod.visual(
        Box((ring_t, 0.009, POD_H - 0.036)),
        origin=Origin(xyz=(POD_D - ring_t * 0.5, POD_W * 0.5 - 0.0045, POD_H * 0.5)),
        material=trim_black,
        name="front_ring_left",
    )
    control_pod.visual(
        Box((ring_t, 0.009, POD_H - 0.036)),
        origin=Origin(xyz=(POD_D - ring_t * 0.5, -POD_W * 0.5 + 0.0045, POD_H * 0.5)),
        material=trim_black,
        name="front_ring_right",
    )
    for index, y_pos in enumerate((-0.020, 0.0, 0.020)):
        control_pod.visual(
            Box((0.025, 0.004, 0.016)),
            origin=Origin(xyz=(0.0185, y_pos + 0.007, 0.040)),
            material=trim_black,
            name=f"button_guide_{index}_0",
        )
        control_pod.visual(
            Box((0.025, 0.004, 0.016)),
            origin=Origin(xyz=(0.0185, y_pos - 0.007, 0.040)),
            material=trim_black,
            name=f"button_guide_{index}_1",
        )
    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(BODY_D * 0.5, POD_Y, POD_Z)),
    )

    drawer = model.part("drawer")
    t_drawer = DRAWER_WALL + EPS
    drawer.visual(
        Box((DRAWER_BODY_D, DRAWER_W, t_drawer)),
        origin=Origin(xyz=(-DRAWER_BODY_D * 0.5, 0.0, t_drawer * 0.5)),
        material=trim_black,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((DRAWER_BODY_D + t_drawer, t_drawer, DRAWER_H)),
        origin=Origin(
            xyz=(-(DRAWER_BODY_D - t_drawer) * 0.5, DRAWER_W * 0.5 - t_drawer * 0.5, DRAWER_H * 0.5)
        ),
        material=trim_black,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((DRAWER_BODY_D + t_drawer, t_drawer, DRAWER_H)),
        origin=Origin(
            xyz=(-(DRAWER_BODY_D - t_drawer) * 0.5, -DRAWER_W * 0.5 + t_drawer * 0.5, DRAWER_H * 0.5)
        ),
        material=trim_black,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((t_drawer, DRAWER_W, DRAWER_H)),
        origin=Origin(xyz=(-DRAWER_BODY_D + t_drawer * 0.5, 0.0, DRAWER_H * 0.5)),
        material=trim_black,
        name="drawer_rear_wall",
    )
    drawer.visual(
        Box((0.014, DRAWER_FACE_W, 0.066)),
        origin=Origin(xyz=(0.007, 0.0, 0.033)),
        material=trim_black,
        name="front_bottom",
    )
    drawer.visual(
        Box((0.014, DRAWER_FACE_W, 0.070)),
        origin=Origin(xyz=(0.007, 0.0, 0.141)),
        material=trim_black,
        name="front_top",
    )
    drawer.visual(
        Box((0.014, 0.082, 0.048)),
        origin=Origin(xyz=(0.007, 0.079, 0.086)),
        material=trim_black,
        name="front_side_0",
    )
    drawer.visual(
        Box((0.014, 0.082, 0.048)),
        origin=Origin(xyz=(0.007, -0.079, 0.086)),
        material=trim_black,
        name="front_side_1",
    )
    drawer_slide = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_D * 0.5, DRAWER_Y, DRAWER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Box((HANDLE_D, 0.012, HANDLE_H)),
        origin=Origin(xyz=(HANDLE_D * 0.5, 0.060, 0.018 + HANDLE_H * 0.5)),
        material=trim_black,
        name="handle_side_0",
    )
    handle.visual(
        Box((HANDLE_D, 0.012, HANDLE_H)),
        origin=Origin(xyz=(HANDLE_D * 0.5, -0.060, 0.018 + HANDLE_H * 0.5)),
        material=trim_black,
        name="handle_side_1",
    )
    handle.visual(
        Box((0.010, 0.108, 0.022)),
        origin=Origin(xyz=(0.005, 0.0, 0.029)),
        material=trim_black,
        name="handle_rear_bar",
    )
    handle.visual(
        Box((0.010, 0.108, 0.022)),
        origin=Origin(xyz=(HANDLE_D - 0.005, 0.0, 0.029)),
        material=trim_black,
        name="handle_front_bar",
    )
    handle.visual(
        Box((0.022, 0.042, 0.008)),
        origin=Origin(xyz=(0.026, 0.044, 0.054)),
        material=trim_black,
        name="handle_top_0",
    )
    handle.visual(
        Box((0.022, 0.042, 0.008)),
        origin=Origin(xyz=(0.026, -0.044, 0.054)),
        material=trim_black,
        name="handle_top_1",
    )
    handle.visual(
        Box((0.050, 0.004, 0.024)),
        origin=Origin(xyz=(0.027, 0.011, 0.042)),
        material=trim_black,
        name="button_guide_0",
    )
    handle.visual(
        Box((0.050, 0.004, 0.024)),
        origin=Origin(xyz=(0.027, -0.011, 0.042)),
        material=trim_black,
        name="button_guide_1",
    )
    model.articulation(
        "drawer_to_handle",
        ArticulationType.FIXED,
        parent=drawer,
        child=handle,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )

    basket = model.part("basket")
    _add_open_shell(
        basket,
        depth=BASKET_D,
        width=BASKET_W,
        height=BASKET_H,
        wall=BASKET_WALL,
        material=basket_dark,
        prefix="basket",
        front_height=0.045,
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(-0.018, 0.0, 0.018)),
    )

    window_panel = model.part("window_panel")
    window_panel.visual(
        Box((0.0025, 0.076, 0.044)),
        origin=Origin(xyz=(0.00125, 0.0, 0.086)),
        material=glass_tint,
        name="window_glass",
    )
    model.articulation(
        "drawer_to_window_panel",
        ArticulationType.FIXED,
        parent=drawer,
        child=window_panel,
        origin=Origin(),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.026, 0.018, 0.006)),
        origin=Origin(xyz=(0.026, 0.0, 0.056)),
        material=button_black,
        name="button_cap",
    )
    release_button.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.026, 0.0, 0.048)),
        material=button_black,
        name="button_stem",
    )
    model.articulation(
        "handle_to_release_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=release_button,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.005,
        ),
    )

    dial_mesh = _dial_mesh("air_fryer_dial")
    for index, z_pos in enumerate((0.158, 0.086)):
        dial = model.part(f"dial_{index}")
        dial.visual(
            dial_mesh,
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=trim_black,
            name="dial_knob",
        )
        model.articulation(
            f"pod_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=control_pod,
            child=dial,
            origin=Origin(xyz=(POD_FACE_X, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=10.0),
        )

    for index, y_pos in enumerate((-0.020, 0.0, 0.020)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.008, 0.016, 0.010)),
            origin=Origin(xyz=(0.004, 0.0, 0.005)),
            material=button_black,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.008)),
            origin=Origin(xyz=(-0.003, 0.0, 0.005)),
            material=button_black,
            name="button_stem",
        )
        model.articulation(
            f"pod_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(POD_FACE_X, y_pos, 0.040)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cavity = object_model.get_part("cavity")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    window_panel = object_model.get_part("window_panel")
    release_button = object_model.get_part("release_button")
    control_pod = object_model.get_part("control_pod")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    release_joint = object_model.get_articulation("handle_to_release_button")

    with ctx.pose({drawer_slide: 0.0}):
        ctx.expect_gap(
            window_panel,
            body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name="drawer window sits flush with body front",
        )
        ctx.expect_within(
            basket,
            drawer,
            axes="yz",
            margin=0.005,
            name="basket stays nested within drawer side walls",
        )

    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            cavity,
            axes="x",
            min_overlap=0.035,
            name="drawer retains insertion at full extension",
        )
        extended_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer extends outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    release_rest = ctx.part_world_position(release_button)
    with ctx.pose({release_joint: 0.005}):
        release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release button presses downward",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[2] < release_rest[2] - 0.003,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"pod_to_button_{index}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.003,
            details=f"rest={rest}, pressed={pressed}",
        )

    for index in range(2):
        joint = object_model.get_articulation(f"pod_to_dial_{index}")
        limits = joint.motion_limits
        ctx.check(
            f"dial_{index} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    ctx.expect_gap(
        control_pod,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        name="control pod mounts flush to body front",
    )

    return ctx.report()


object_model = build_object_model()
