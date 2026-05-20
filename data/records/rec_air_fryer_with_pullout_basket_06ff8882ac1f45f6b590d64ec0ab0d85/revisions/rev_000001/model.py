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


BODY_W = 0.318
BODY_D = 0.376
BODY_H = 0.410

CAVITY_W = 0.270
CAVITY_D = 0.314
CAVITY_H = 0.188
CAVITY_Y0 = -0.176
CAVITY_Z0 = 0.066

DRAWER_W = 0.252
DRAWER_D = 0.304
DRAWER_H = 0.170
DRAWER_Y0 = -0.176
DRAWER_Z0 = 0.070

BUTTON_TRAVEL = 0.0035
LATCH_TRAVEL = 0.0040
DRAWER_TRAVEL = 0.180

CLUSTER_W = 0.098
CLUSTER_D = 0.010
CLUSTER_H = 0.152
CLUSTER_Z = 0.332
CLUSTER_Y = -BODY_D / 2.0 + 0.003
CLUSTER_FRONT_Y = CLUSTER_Y - CLUSTER_D / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    button_finish = model.material("button_finish", rgba=(0.80, 0.82, 0.84, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.84, 0.11, 0.10, 1.0))

    body = model.part("body")
    front_y = -BODY_D / 2.0 + 0.010
    opening_top = CAVITY_Z0 + CAVITY_H
    upper_side_w = 0.094

    body.visual(
        Box((0.018, BODY_D, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + 0.009, 0.0, BODY_H / 2.0)),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((0.018, BODY_D, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - 0.009, 0.0, BODY_H / 2.0)),
        material=body_finish,
        name="right_shell",
    )
    body.visual(
        Box((BODY_W - 0.036, 0.018, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.009, BODY_H / 2.0)),
        material=body_finish,
        name="rear_shell",
    )
    body.visual(
        Box((BODY_W - 0.018, BODY_D - 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - 0.012)),
        material=body_finish,
        name="roof_shell",
    )
    body.visual(
        Box((BODY_W - 0.020, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, front_y, 0.030)),
        material=body_finish,
        name="front_sill",
    )
    body.visual(
        Box((0.018, 0.020, CAVITY_H)),
        origin=Origin(xyz=(-0.141, front_y, CAVITY_Z0 + CAVITY_H / 2.0)),
        material=body_finish,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.018, 0.020, CAVITY_H)),
        origin=Origin(xyz=(0.141, front_y, CAVITY_Z0 + CAVITY_H / 2.0)),
        material=body_finish,
        name="front_jamb_1",
    )
    body.visual(
        Box((upper_side_w, 0.020, BODY_H - opening_top)),
        origin=Origin(
            xyz=(-0.102, front_y, opening_top + (BODY_H - opening_top) / 2.0),
        ),
        material=body_finish,
        name="upper_face_0",
    )
    body.visual(
        Box((upper_side_w, 0.020, BODY_H - opening_top)),
        origin=Origin(
            xyz=(0.102, front_y, opening_top + (BODY_H - opening_top) / 2.0),
        ),
        material=body_finish,
        name="upper_face_1",
    )
    body.visual(
        Box((0.110, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, front_y, opening_top + 0.007)),
        material=body_finish,
        name="cluster_bridge",
    )
    body.visual(
        Box((0.110, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, front_y, BODY_H - 0.008)),
        material=body_finish,
        name="cluster_cap",
    )
    body.visual(
        Box((0.078, CLUSTER_D, 0.066)),
        origin=Origin(xyz=(0.0, CLUSTER_Y, CLUSTER_Z + 0.030)),
        material=trim_finish,
        name="control_cluster",
    )
    body.visual(
        Box((CLUSTER_W, CLUSTER_D, 0.024)),
        origin=Origin(xyz=(0.0, CLUSTER_Y, CLUSTER_Z - 0.014)),
        material=trim_finish,
        name="button_bezel_top",
    )
    body.visual(
        Box((CLUSTER_W, CLUSTER_D, 0.016)),
        origin=Origin(xyz=(0.0, CLUSTER_Y, CLUSTER_Z - 0.064)),
        material=trim_finish,
        name="button_bezel_bottom",
    )
    for index, cluster_x in enumerate((-0.042, -0.014, 0.014, 0.042)):
        body.visual(
            Box((0.008 if abs(cluster_x) == 0.014 else 0.014, CLUSTER_D, 0.036)),
            origin=Origin(xyz=(cluster_x, CLUSTER_Y, CLUSTER_Z - 0.043)),
            material=trim_finish,
            name=f"button_bezel_{index}",
        )
    for side, x_sign in enumerate((-1.0, 1.0)):
        for slot_index, slot_z in enumerate((0.300, 0.318, 0.336, 0.354, 0.372)):
            body.visual(
                Box((0.004, 0.120, 0.006)),
                origin=Origin(
                    xyz=(x_sign * (BODY_W / 2.0 - 0.002), 0.036, slot_z),
                ),
                material=trim_finish,
                name=f"side_vent_{side}_{slot_index}",
            )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_W, DRAWER_D, 0.010)),
        origin=Origin(xyz=(0.0, DRAWER_D / 2.0, 0.005)),
        material=drawer_finish,
        name="drawer_shell",
    )
    drawer.visual(
        Box((0.004, DRAWER_D, DRAWER_H - 0.010)),
        origin=Origin(xyz=(-DRAWER_W / 2.0 + 0.002, DRAWER_D / 2.0, 0.090)),
        material=drawer_finish,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.004, DRAWER_D, DRAWER_H - 0.010)),
        origin=Origin(xyz=(DRAWER_W / 2.0 - 0.002, DRAWER_D / 2.0, 0.090)),
        material=drawer_finish,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((DRAWER_W, 0.004, DRAWER_H - 0.010)),
        origin=Origin(xyz=(0.0, DRAWER_D - 0.002, 0.090)),
        material=drawer_finish,
        name="drawer_rear",
    )
    drawer.visual(
        Box((DRAWER_W, 0.004, DRAWER_H - 0.020)),
        origin=Origin(xyz=(0.0, 0.002, 0.085)),
        material=drawer_finish,
        name="drawer_front_wall",
    )
    drawer.visual(
        Box((0.264, 0.012, 0.178)),
        origin=Origin(xyz=(0.0, -0.006, 0.089)),
        material=drawer_finish,
        name="drawer_fascia",
    )
    drawer.visual(
        Box((0.132, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, -0.019, 0.080)),
        material=drawer_finish,
        name="handle_rear",
    )
    drawer.visual(
        Box((0.044, 0.042, 0.028)),
        origin=Origin(xyz=(-0.044, -0.033, 0.080)),
        material=drawer_finish,
        name="handle_grip_0",
    )
    drawer.visual(
        Box((0.044, 0.042, 0.028)),
        origin=Origin(xyz=(0.044, -0.033, 0.080)),
        material=drawer_finish,
        name="handle_grip_1",
    )
    drawer.visual(
        Box((0.006, 0.022, 0.012)),
        origin=Origin(xyz=(-0.020, -0.024, 0.103)),
        material=drawer_finish,
        name="latch_guide_0",
    )
    drawer.visual(
        Box((0.006, 0.022, 0.012)),
        origin=Origin(xyz=(0.020, -0.024, 0.103)),
        material=drawer_finish,
        name="latch_guide_1",
    )
    drawer.visual(
        Box((0.044, 0.042, 0.010)),
        origin=Origin(xyz=(-0.044, -0.033, 0.105)),
        material=drawer_finish,
        name="handle_top_0",
    )
    drawer.visual(
        Box((0.044, 0.042, 0.010)),
        origin=Origin(xyz=(0.044, -0.033, 0.105)),
        material=drawer_finish,
        name="handle_top_1",
    )
    drawer.visual(
        Box((0.064, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, 0.105)),
        material=drawer_finish,
        name="handle_top_rear",
    )

    basket = model.part("basket")
    basket_w = 0.228
    basket_d = 0.258
    basket_h = 0.145
    basket_y0 = 0.020
    basket_z0 = 0.018
    wall = 0.0035
    basket.visual(
        Box((wall, basket_d, basket_h)),
        origin=Origin(xyz=(-basket_w / 2.0 + wall / 2.0, basket_y0 + basket_d / 2.0, basket_z0 + basket_h / 2.0)),
        material=basket_finish,
        name="basket_shell",
    )
    basket.visual(
        Box((wall, basket_d, basket_h)),
        origin=Origin(xyz=(basket_w / 2.0 - wall / 2.0, basket_y0 + basket_d / 2.0, basket_z0 + basket_h / 2.0)),
        material=basket_finish,
        name="basket_side_1",
    )
    basket.visual(
        Box((basket_w, wall, basket_h)),
        origin=Origin(xyz=(0.0, basket_y0 + wall / 2.0, basket_z0 + basket_h / 2.0)),
        material=basket_finish,
        name="basket_front",
    )
    basket.visual(
        Box((basket_w, wall, basket_h)),
        origin=Origin(xyz=(0.0, basket_y0 + basket_d - wall / 2.0, basket_z0 + basket_h / 2.0)),
        material=basket_finish,
        name="basket_rear",
    )
    basket.visual(
        Box((basket_w, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, basket_y0 + 0.020, 0.020)),
        material=basket_finish,
        name="basket_floor_0",
    )
    basket.visual(
        Box((basket_w, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, basket_y0 + basket_d - 0.020, 0.020)),
        material=basket_finish,
        name="basket_floor_1",
    )
    basket.visual(
        Box((0.012, basket_d - 0.024, 0.004)),
        origin=Origin(xyz=(-basket_w / 2.0 + 0.006, basket_y0 + basket_d / 2.0, 0.020)),
        material=basket_finish,
        name="basket_floor_2",
    )
    basket.visual(
        Box((0.012, basket_d - 0.024, 0.004)),
        origin=Origin(xyz=(basket_w / 2.0 - 0.006, basket_y0 + basket_d / 2.0, 0.020)),
        material=basket_finish,
        name="basket_floor_3",
    )
    for index, bar_y in enumerate((0.074, 0.118, 0.162, 0.206)):
        basket.visual(
            Box((basket_w - 0.004, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, bar_y, 0.020)),
            material=basket_finish,
            name=f"basket_grate_{index}",
        )
    for foot_x in (-0.080, 0.080):
        for foot_y in (basket_y0 + 0.020, basket_y0 + basket_d - 0.020):
            basket.visual(
                Box((0.016, 0.016, 0.008)),
                origin=Origin(xyz=(foot_x, foot_y, 0.014)),
                material=basket_finish,
                name=f"basket_foot_{int((foot_x > 0) * 2 + (foot_y > 0.18))}",
            )

    latch = model.part("latch")
    latch.visual(
        Box((0.034, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, -0.0045)),
        material=latch_finish,
        name="latch_button",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.026,
                body_style="skirted",
                top_diameter=0.044,
                skirt=KnobSkirt(0.060, 0.006, flare=0.10),
                grip=KnobGrip(style="fluted", count=20, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "air_fryer_selector_knob_v2",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="selector_knob",
    )

    button_positions = (-0.028, 0.0, 0.028)
    button_parts = []
    for index, button_x in enumerate(button_positions):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box((0.022, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=button_finish,
            name="preset_button",
        )
        button_parts.append((button, button_x))

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_Y0, DRAWER_Z0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(),
    )
    model.articulation(
        "drawer_to_latch",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=latch,
        origin=Origin(xyz=(0.0, -0.024, 0.109)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.12, lower=0.0, upper=LATCH_TRAVEL),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, CLUSTER_FRONT_Y, CLUSTER_Z + 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    for index, (button, button_x) in enumerate(button_parts):
        model.articulation(
            f"body_to_preset_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, CLUSTER_FRONT_Y, CLUSTER_Z - 0.042)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.10, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    latch = object_model.get_part("latch")
    knob = object_model.get_part("selector_knob")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    latch_joint = object_model.get_articulation("drawer_to_latch")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    button_joints = [object_model.get_articulation(f"body_to_preset_button_{index}") for index in range(3)]

    ctx.check(
        "selector knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )
    ctx.check(
        "drawer joint is prismatic",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_joint.articulation_type!r}",
    )
    ctx.expect_within(
        basket,
        drawer,
        axes="xy",
        margin=0.0,
        name="basket footprint stays inside drawer shell",
    )
    ctx.expect_contact(
        basket,
        drawer,
        name="basket is supported by the drawer floor",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        margin=0.0,
        name="drawer stays centered in the lower body opening",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        min_overlap=0.120,
        name="closed drawer remains deeply inserted in the body cavity",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_b="control_cluster",
        name="selector knob mounts onto the control cluster",
    )

    button_rest_positions = []
    for index in range(3):
        button_part = object_model.get_part(f"preset_button_{index}")
        ctx.expect_within(
            button_part,
            body,
            axes="xz",
            margin=0.0,
            name=f"preset button {index} stays within the control cluster footprint",
        )
        button_rest_positions.append(ctx.part_world_position(button_part))

    latch_rest = ctx.part_world_position(latch)
    drawer_rest = ctx.part_world_position(drawer)

    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            margin=0.0,
            name="extended drawer stays aligned to the body opening",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.090,
            name="extended drawer keeps retained insertion",
        )
        drawer_open = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends forward",
            drawer_rest is not None and drawer_open is not None and drawer_open[1] < drawer_rest[1] - 0.15,
            details=f"rest={drawer_rest}, open={drawer_open}",
        )

    with ctx.pose({latch_joint: LATCH_TRAVEL}):
        latch_pressed = ctx.part_world_position(latch)
        ctx.check(
            "latch presses downward",
            latch_rest is not None and latch_pressed is not None and latch_pressed[2] < latch_rest[2] - 0.003,
            details=f"rest={latch_rest}, pressed={latch_pressed}",
        )
        ctx.expect_within(
            latch,
            drawer,
            axes="xy",
            margin=0.0,
            name="latch stays within the handle housing footprint",
        )

    for index, joint in enumerate(button_joints):
        button_part = object_model.get_part(f"preset_button_{index}")
        with ctx.pose({joint: BUTTON_TRAVEL}):
            button_pressed = ctx.part_world_position(button_part)
            button_rest = button_rest_positions[index]
            ctx.check(
                f"preset button {index} presses inward",
                button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.002,
                details=f"rest={button_rest}, pressed={button_pressed}",
            )

    return ctx.report()


object_model = build_object_model()
