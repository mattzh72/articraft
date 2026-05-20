from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trolley_event_speaker")

    cabinet_mat = model.material("charcoal_molded_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    grille_mat = model.material("black_perforated_steel", rgba=(0.006, 0.006, 0.007, 1.0))
    rubber_mat = model.material("matte_black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_trim_mat = model.material("dark_graphite_trim", rgba=(0.10, 0.105, 0.11, 1.0))
    metal_mat = model.material("brushed_dark_metal", rgba=(0.33, 0.34, 0.34, 1.0))
    deck_mat = model.material("satin_control_deck", rgba=(0.018, 0.020, 0.024, 1.0))
    rocker_mat = model.material("red_power_rocker", rgba=(0.75, 0.045, 0.030, 1.0))
    button_mat = model.material("cool_gray_buttons", rgba=(0.55, 0.58, 0.60, 1.0))
    label_mat = model.material("white_printed_symbols", rgba=(0.92, 0.94, 0.92, 1.0))

    # The cabinet frame uses a practical event-speaker scale: about 1.08 m tall
    # before the pull handle is extended, with a deep rear for battery/amplifier mass.
    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.44, 0.34, 0.92)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=cabinet_mat,
        name="main_shell",
    )
    cabinet.visual(
        Box((0.48, 0.34, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        material=rubber_mat,
        name="top_bumper",
    )
    cabinet.visual(
        Box((0.47, 0.36, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=rubber_mat,
        name="bottom_bumper",
    )
    for x in (-0.225, 0.225):
        for y in (-0.172, 0.172):
            cabinet.visual(
                Cylinder(radius=0.026, length=0.91),
                origin=Origin(xyz=(x, y, 0.565)),
                material=rubber_mat,
                name=f"corner_post_{'front' if y < 0 else 'rear'}_{0 if x < 0 else 1}",
            )

    cabinet.visual(
        Box((0.372, 0.012, 0.765)),
        origin=Origin(xyz=(0.0, -0.174, 0.585)),
        material=grille_mat,
        name="front_grille",
    )
    for z in [0.230, 0.300, 0.370, 0.440, 0.510, 0.580, 0.650, 0.720, 0.790, 0.860, 0.930]:
        cabinet.visual(
            Box((0.335, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.183, z)),
            material=dark_trim_mat,
            name=f"grille_louver_{int(z * 1000)}",
        )
    for z, radius, name in ((0.745, 0.125, "upper_driver"), (0.415, 0.145, "lower_driver")):
        cabinet.visual(
            Cylinder(radius=radius + 0.022, length=0.014),
            origin=Origin(xyz=(0.0, -0.184, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_mat,
            name=f"{name}_surround",
        )
        cabinet.visual(
            Cylinder(radius=radius, length=0.016),
            origin=Origin(xyz=(0.0, -0.193, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim_mat,
            name=f"{name}_cone",
        )
        cabinet.visual(
            Cylinder(radius=radius * 0.42, length=0.018),
            origin=Origin(xyz=(0.0, -0.205, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grille_mat,
            name=f"{name}_dust_cap",
        )

    # A small, separate top-front control deck is fixed to the cabinet. The
    # individual controls on it are articulated child links below.
    cabinet.visual(
        Box((0.335, 0.140, 0.016)),
        origin=Origin(xyz=(-0.015, -0.070, 1.088)),
        material=deck_mat,
        name="control_deck",
    )
    cabinet.visual(
        Box((0.155, 0.026, 0.006)),
        origin=Origin(xyz=(0.105, -0.127, 1.099)),
        material=label_mat,
        name="menu_label_strip",
    )

    # Rear trolley sleeves: the moving handle rods intentionally slide inside
    # these external rails.
    for x, idx in ((-0.160, 0), (0.160, 1)):
        cabinet.visual(
            Cylinder(radius=0.019, length=0.420),
            origin=Origin(xyz=(x, 0.188, 0.910)),
            material=metal_mat,
            name=f"outer_rail_{idx}",
        )
    for z, name in ((1.105, "upper_rail_bridge"), (0.705, "lower_rail_bridge")):
        cabinet.visual(
            Box((0.380, 0.040, 0.036)),
            origin=Origin(xyz=(0.0, 0.150, z)),
            material=rubber_mat,
            name=name,
        )

    # Axle bosses and short exposed axle stubs visually carry the transport wheels.
    for x, idx in ((-0.235, 0), (0.235, 1)):
        cabinet.visual(
            Box((0.030, 0.052, 0.060)),
            origin=Origin(xyz=(x, 0.176, 0.105)),
            material=rubber_mat,
            name=f"axle_boss_{idx}",
        )
        cabinet.visual(
            Cylinder(radius=0.009, length=0.028),
            origin=Origin(xyz=(x, 0.190, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"axle_stub_{idx}",
        )

    # Telescoping pull handle, with two inner rods and a connected hand grip.
    pull_handle = model.part("pull_handle")
    for x, idx in ((-0.160, 0), (0.160, 1)):
        pull_handle.visual(
            Cylinder(radius=0.011, length=0.480),
            origin=Origin(xyz=(x, 0.0, -0.120)),
            material=metal_mat,
            name=f"inner_rail_{idx}",
        )
    pull_handle.visual(
        Cylinder(radius=0.020, length=0.385),
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_mat,
        name="hand_grip",
    )
    pull_handle.visual(
        Box((0.355, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=rubber_mat,
        name="grip_saddle",
    )
    model.articulation(
        "cabinet_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=pull_handle,
        origin=Origin(xyz=(0.0, 0.188, 1.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    # Transport wheels use primitive tire/rim layers to keep the spinning
    # collision simple while still reading as rolling hardware.
    for x, idx in ((-0.280, 0), (0.280, 1)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            Cylinder(radius=0.085, length=0.055),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.052, length=0.060),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.021, length=0.066),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim_mat,
            name="hub",
        )
        model.articulation(
            f"cabinet_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=wheel,
            origin=Origin(xyz=(x, 0.190, 0.105)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    # The power rocker is a wider red tilting cap on a short transverse pivot.
    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.078, 0.052, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=rocker_mat,
        name="rocker_cap",
    )
    power_rocker.visual(
        Box((0.060, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=label_mat,
        name="rocker_mark",
    )
    power_rocker.visual(
        Cylinder(radius=0.006, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="rocker_pivot",
    )
    model.articulation(
        "cabinet_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=power_rocker,
        origin=Origin(xyz=(-0.105, -0.070, 1.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.26, upper=0.26),
    )

    # Two smaller menu buttons depress independently beside the rocker.
    for x, idx in ((0.025, 0), (0.083, 1)):
        button = model.part(f"menu_button_{idx}")
        button.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0135)),
            material=label_mat,
            name="button_symbol",
        )
        model.articulation(
            f"cabinet_to_menu_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, -0.070, 1.096)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    handle = object_model.get_part("pull_handle")
    handle_slide = object_model.get_articulation("cabinet_to_pull_handle")
    rocker = object_model.get_part("power_rocker")
    rocker_joint = object_model.get_articulation("cabinet_to_power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    menu_joint_0 = object_model.get_articulation("cabinet_to_menu_button_0")
    menu_joint_1 = object_model.get_articulation("cabinet_to_menu_button_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    wheel_joint_0 = object_model.get_articulation("cabinet_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("cabinet_to_wheel_1")

    for idx in (0, 1):
        ctx.allow_overlap(
            cabinet,
            handle,
            elem_a=f"outer_rail_{idx}",
            elem_b=f"inner_rail_{idx}",
            reason="The telescoping handle rod is intentionally captured inside the rear sleeve.",
        )
        ctx.expect_within(
            handle,
            cabinet,
            axes="xy",
            inner_elem=f"inner_rail_{idx}",
            outer_elem=f"outer_rail_{idx}",
            margin=0.001,
            name=f"handle rail {idx} is centered in sleeve",
        )
        ctx.expect_overlap(
            handle,
            cabinet,
            axes="z",
            elem_a=f"inner_rail_{idx}",
            elem_b=f"outer_rail_{idx}",
            min_overlap=0.30,
            name=f"collapsed handle rail {idx} remains inserted",
        )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.32}):
        extended_handle_pos = ctx.part_world_position(handle)
        for idx in (0, 1):
            ctx.expect_within(
                handle,
                cabinet,
                axes="xy",
                inner_elem=f"inner_rail_{idx}",
                outer_elem=f"outer_rail_{idx}",
                margin=0.001,
                name=f"extended handle rail {idx} stays in sleeve",
            )
            ctx.expect_overlap(
                handle,
                cabinet,
                axes="z",
                elem_a=f"inner_rail_{idx}",
                elem_b=f"outer_rail_{idx}",
                min_overlap=0.09,
                name=f"extended handle rail {idx} remains retained",
            )
    ctx.check(
        "pull handle slides upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.30,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    ctx.allow_overlap(
        cabinet,
        rocker,
        elem_a="control_deck",
        elem_b="rocker_pivot",
        reason="The short rocker pivot is seated through the deck opening.",
    )
    ctx.expect_contact(
        rocker,
        cabinet,
        elem_a="rocker_cap",
        elem_b="control_deck",
        contact_tol=0.002,
        name="power rocker cap sits on deck",
    )
    ctx.check(
        "power rocker has short pivot rotation",
        rocker_joint.articulation_type == ArticulationType.REVOLUTE
        and rocker_joint.motion_limits is not None
        and rocker_joint.motion_limits.lower <= -0.24
        and rocker_joint.motion_limits.upper >= 0.24,
        details=f"type={rocker_joint.articulation_type}, limits={rocker_joint.motion_limits}",
    )

    for idx, button, joint in ((0, menu_button_0, menu_joint_0), (1, menu_button_1, menu_joint_1)):
        ctx.expect_contact(
            button,
            cabinet,
            elem_a="button_cap",
            elem_b="control_deck",
            contact_tol=0.002,
            name=f"menu button {idx} cap sits on deck",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"menu button {idx} depresses independently",
            joint.articulation_type == ArticulationType.PRISMATIC
            and rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.005,
            details=f"joint={joint.articulation_type}, rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.check(
        "transport wheels use continuous spin joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"wheel0={wheel_joint_0.articulation_type}, wheel1={wheel_joint_1.articulation_type}",
    )
    rest_wheel_pos = (ctx.part_world_position(wheel_0), ctx.part_world_position(wheel_1))
    with ctx.pose({wheel_joint_0: math.pi, wheel_joint_1: -math.pi}):
        spun_wheel_pos = (ctx.part_world_position(wheel_0), ctx.part_world_position(wheel_1))
    ctx.check(
        "wheel spin keeps axle centers fixed",
        rest_wheel_pos == spun_wheel_pos,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
