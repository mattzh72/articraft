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


def _control_button(part, material) -> None:
    """A proud round button cap, with its rear face at the part frame."""

    part.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="button_cap",
    )


def _top_button(part, material) -> None:
    part.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=material,
        name="button_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trolley_portable_speaker")

    matte_body = model.material("matte_body", rgba=(0.055, 0.060, 0.065, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.015, 0.017, 0.020, 1.0))
    grille_black = model.material("grille_black", rgba=(0.020, 0.022, 0.025, 1.0))
    rubber = model.material("rubber", rgba=(0.030, 0.030, 0.030, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    button_grey = model.material("button_grey", rgba=(0.22, 0.23, 0.25, 1.0))
    blue_button = model.material("blue_button", rgba=(0.05, 0.25, 0.78, 1.0))
    red_button = model.material("red_button", rgba=(0.75, 0.06, 0.04, 1.0))
    label_white = model.material("label_white", rgba=(0.82, 0.84, 0.85, 1.0))
    display_glass = model.material("display_glass", rgba=(0.08, 0.18, 0.24, 0.72))

    body = model.part("body")
    body.visual(
        Box((0.300, 0.360, 0.760)),
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        material=matte_body,
        name="body_shell",
    )
    for x in (-0.150, 0.150):
        for y in (-0.180, 0.180):
            body.visual(
                Cylinder(radius=0.024, length=0.760),
                origin=Origin(xyz=(x, y, 0.460)),
                material=matte_body,
                name=f"corner_bumper_{'rear' if x > 0 else 'front'}_{'pos' if y > 0 else 'neg'}",
            )

    body.visual(
        Box((0.008, 0.288, 0.470)),
        origin=Origin(xyz=(-0.154, 0.0, 0.405)),
        material=grille_black,
        name="speaker_grille",
    )
    for index, z in enumerate([0.190 + 0.035 * i for i in range(13)]):
        body.visual(
            Box((0.010, 0.270, 0.006)),
            origin=Origin(xyz=(-0.160, 0.0, z)),
            material=dark_panel,
            name=f"grille_rib_{index}",
        )
    body.visual(
        Cylinder(radius=0.098, length=0.010),
        origin=Origin(xyz=(-0.163, 0.0, 0.465), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_panel,
        name="woofer_shadow",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.011),
        origin=Origin(xyz=(-0.164, 0.0, 0.645), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_panel,
        name="tweeter_shadow",
    )

    body.visual(
        Box((0.012, 0.260, 0.096)),
        origin=Origin(xyz=(-0.156, 0.0, 0.755)),
        material=dark_panel,
        name="front_control_face",
    )
    body.visual(
        Box((0.132, 0.270, 0.030)),
        origin=Origin(xyz=(-0.050, 0.0, 0.855)),
        material=dark_panel,
        name="control_deck",
    )
    body.visual(
        Box((0.038, 0.100, 0.004)),
        origin=Origin(xyz=(-0.108, 0.0, 0.868)),
        material=display_glass,
        name="display_window",
    )
    for y in (-0.080, 0.0, 0.080):
        body.visual(
            Box((0.004, 0.026, 0.002)),
            origin=Origin(xyz=(-0.164, y, 0.792)),
            material=label_white,
            name=f"front_label_{int((y + 0.08) * 1000)}",
        )

    # Twin rear guide rails and side clamp brackets for the sliding pull handle.
    for index, y in enumerate((-0.120, 0.120)):
        body.visual(
            Cylinder(radius=0.012, length=0.580),
            origin=Origin(xyz=(0.185, y, 0.620)),
            material=satin_metal,
            name=f"rear_rail_{index}",
        )
        outside_y = y + (0.018 if y > 0 else -0.018)
        for level, z in enumerate((0.455, 0.790)):
            body.visual(
                Box((0.070, 0.012, 0.034)),
                origin=Origin(xyz=(0.163, outside_y, z)),
                material=matte_body,
                name=f"rail_bracket_{index}_{level}",
            )
        body.visual(
            Cylinder(radius=0.015, length=0.028),
            origin=Origin(xyz=(0.185, y, 0.910)),
            material=matte_body,
            name=f"rail_collar_{index}",
        )

    # Wheel axle stubs and lower corner protection that keep the wheels mounted.
    for index, y in enumerate((-0.235, 0.235)):
        body.visual(
            Cylinder(radius=0.014, length=0.056),
            origin=Origin(xyz=(0.126, y, 0.067), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=f"axle_{index}",
        )
        body.visual(
            Box((0.052, 0.026, 0.056)),
            origin=Origin(xyz=(0.126, y * 0.84, 0.090)),
            material=matte_body,
            name=f"wheel_yoke_{index}",
        )
    body.visual(
        Box((0.060, 0.340, 0.040)),
        origin=Origin(xyz=(0.118, 0.0, 0.070)),
        material=matte_body,
        name="rear_bottom_rail",
    )

    # Telescoping pull handle: two sliding tubes joined by a rubberized grip.
    handle = model.part("pull_handle")
    for index, y in enumerate((-0.120, 0.120)):
        handle.visual(
            Cylinder(radius=0.007, length=0.560),
            origin=Origin(xyz=(0.0, y, -0.090)),
            material=satin_metal,
            name=f"handle_rod_{index}",
        )
    handle.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.197), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="handle_crossbar",
    )
    handle.visual(
        Cylinder(radius=0.020, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.197), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    model.articulation(
        "body_to_pull_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.185, 0.0, 0.880)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    # Three separate front push-buttons.
    for index, (y, material) in enumerate(((-0.080, red_button), (0.0, button_grey), (0.080, blue_button))):
        button = model.part(f"front_button_{index}")
        _control_button(button, material)
        model.articulation(
            f"body_to_front_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.162, y, 0.742)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.06, lower=0.0, upper=0.010),
        )

    # Small independent buttons on the top control deck.
    for index, (x, y, material) in enumerate(
        (
            (-0.055, -0.090, button_grey),
            (-0.030, -0.090, button_grey),
            (-0.005, -0.090, blue_button),
            (0.020, -0.090, button_grey),
        )
    ):
        deck_button = model.part(f"top_button_{index}")
        _top_button(deck_button, material)
        model.articulation(
            f"body_to_top_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=deck_button,
            origin=Origin(xyz=(x, y, 0.870)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    wheel_positions = [(-0.235, "wheel_0"), (0.235, "wheel_1")]
    for index, (y, wheel_name) in enumerate(wheel_positions):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.064, length=0.044),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.042, length=0.049),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.056),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_panel,
            name="hub_cap",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(0.126, y, 0.067)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("pull_handle")
    handle_joint = object_model.get_articulation("body_to_pull_handle")

    for index in range(2):
        ctx.allow_overlap(
            body,
            handle,
            elem_a=f"rear_rail_{index}",
            elem_b=f"handle_rod_{index}",
            reason="The telescoping handle rods are intentionally represented as sliding inside solid rear rail sleeve proxies.",
        )
        ctx.allow_overlap(
            body,
            handle,
            elem_a=f"rail_collar_{index}",
            elem_b=f"handle_rod_{index}",
            reason="The collar is a simplified solid lip around the telescoping handle rod.",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem=f"handle_rod_{index}",
            outer_elem=f"rear_rail_{index}",
            margin=0.002,
            name=f"handle rod {index} stays centered in rear rail",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a=f"handle_rod_{index}",
            elem_b=f"rear_rail_{index}",
            min_overlap=0.28,
            name=f"handle rod {index} remains inserted at rest",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem=f"handle_rod_{index}",
            outer_elem=f"rail_collar_{index}",
            margin=0.002,
            name=f"handle rod {index} passes through collar",
        )
        with ctx.pose({handle_joint: 0.280}):
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a=f"handle_rod_{index}",
                elem_b=f"rear_rail_{index}",
                min_overlap=0.10,
                name=f"handle rod {index} remains captured when extended",
            )

    rest_handle = ctx.part_world_position(handle)
    with ctx.pose({handle_joint: 0.280}):
        extended_handle = ctx.part_world_position(handle)
    ctx.check(
        "pull handle slides upward",
        rest_handle is not None
        and extended_handle is not None
        and extended_handle[2] > rest_handle[2] + 0.25,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    for index in range(2):
        wheel = object_model.get_part(f"wheel_{index}")
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=f"axle_{index}",
            elem_b="rim",
            reason="The metal axle stub is intentionally captured through the wheel hub/rim for continuous wheel spin.",
        )
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=f"axle_{index}",
            elem_b="hub_cap",
            reason="The axle stub is intentionally nested in the hub cap bore proxy.",
        )
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=f"axle_{index}",
            elem_b="tire",
            reason="The wheel tire is a solid rolling proxy; the axle path represents the hidden central bore.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=f"axle_{index}",
            elem_b="rim",
            min_overlap=0.020,
            name=f"wheel {index} is retained on axle",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=f"axle_{index}",
            elem_b="hub_cap",
            min_overlap=0.020,
            name=f"wheel {index} hub cap captures axle",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=f"axle_{index}",
            elem_b="tire",
            min_overlap=0.020,
            name=f"wheel {index} tire proxy is centered on axle",
        )
        spin_joint = object_model.get_articulation(f"body_to_wheel_{index}")
        ctx.check(
            f"wheel {index} uses continuous spin joint",
            str(spin_joint.articulation_type).lower().endswith("continuous"),
            details=str(spin_joint.articulation_type),
        )

    for index in range(3):
        button = object_model.get_part(f"front_button_{index}")
        joint = object_model.get_articulation(f"body_to_front_button_{index}")
        ctx.expect_gap(
            body,
            button,
            axis="x",
            positive_elem="front_control_face",
            negative_elem="button_cap",
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"front button {index} sits in control face",
        )
        before = ctx.part_world_position(button)
        with ctx.pose({joint: 0.010}):
            after = ctx.part_world_position(button)
        ctx.check(
            f"front button {index} depresses inward",
            before is not None and after is not None and after[0] > before[0] + 0.008,
            details=f"before={before}, after={after}",
        )

    for index in range(4):
        button = object_model.get_part(f"top_button_{index}")
        joint = object_model.get_articulation(f"body_to_top_button_{index}")
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_deck",
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"top button {index} sits proud of control deck",
        )
        before = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            after = ctx.part_world_position(button)
        ctx.check(
            f"top button {index} depresses downward",
            before is not None and after is not None and after[2] < before[2] - 0.004,
            details=f"before={before}, after={after}",
        )

    return ctx.report()


object_model = build_object_model()
