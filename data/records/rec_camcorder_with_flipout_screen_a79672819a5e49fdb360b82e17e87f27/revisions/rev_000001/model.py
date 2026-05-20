from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius=radius)]


def _body_shell_mesh():
    # Short home-video camcorder proportions: a compact rounded rectangular
    # housing rather than a long broadcast-style camera body.
    sections = [
        _rounded_yz_section(-0.076, 0.074, 0.068, 0.014),
        _rounded_yz_section(-0.030, 0.082, 0.076, 0.017),
        _rounded_yz_section(0.047, 0.082, 0.076, 0.017),
        _rounded_yz_section(0.076, 0.074, 0.070, 0.014),
    ]
    return section_loft(sections)


def _lens_ring_mesh():
    # A real focusing/zoom ring is a hollow sleeve around the barrel.  The
    # stepped outer profile gives visible rubberized grip bands while keeping a
    # clear inner bore for the fixed lens barrel.
    outer_profile = [
        (0.028, -0.012),
        (0.030, -0.010),
        (0.030, -0.007),
        (0.0275, -0.0058),
        (0.030, -0.0042),
        (0.030, -0.0015),
        (0.0275, 0.0000),
        (0.030, 0.0015),
        (0.030, 0.0042),
        (0.0275, 0.0058),
        (0.030, 0.0070),
        (0.030, 0.010),
        (0.028, 0.012),
    ]
    inner_profile = [
        (0.0218, -0.012),
        (0.0218, 0.012),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_camcorder")

    charcoal = model.material("charcoal_plastic", rgba=(0.055, 0.058, 0.064, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.017, 1.0))
    screen_black = model.material("screen_black", rgba=(0.002, 0.003, 0.006, 1.0))
    glass_blue = model.material("coated_glass", rgba=(0.10, 0.20, 0.30, 0.72))
    button_silver = model.material("button_silver", rgba=(0.63, 0.66, 0.70, 1.0))
    label_white = model.material("label_white", rgba=(0.92, 0.93, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell_mesh(), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=charcoal,
        name="body_shell",
    )
    body.visual(
        Box((0.112, 0.0022, 0.048)),
        origin=Origin(xyz=(-0.008, 0.0410, 0.048)),
        material=screen_black,
        name="screen_opening",
    )
    body.visual(
        Box((0.086, 0.0032, 0.008)),
        origin=Origin(xyz=(0.000, 0.0414, 0.018)),
        material=satin_gray,
        name="button_well",
    )
    body.visual(
        Cylinder(radius=0.0215, length=0.044),
        origin=Origin(xyz=(0.097, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_gray,
        name="lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.0180, length=0.004),
        origin=Origin(xyz=(0.121, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="front_glass",
    )
    body.visual(
        Cylinder(radius=0.0300, length=0.002),
        origin=Origin(xyz=(0.093, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_gray,
        name="barrel_shoulder",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.1235, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screen_black,
        name="dark_lens_center",
    )
    body.visual(
        Cylinder(radius=0.0030, length=0.060),
        origin=Origin(xyz=(-0.066, 0.0430, 0.046)),
        material=satin_gray,
        name="side_hinge_post",
    )
    body.visual(
        Box((0.006, 0.006, 0.064)),
        origin=Origin(xyz=(-0.066, 0.0405, 0.046)),
        material=satin_gray,
        name="hinge_leaf",
    )
    body.visual(
        Box((0.020, 0.054, 0.006)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0785)),
        material=satin_gray,
        name="top_dial_plinth",
    )

    side_screen = model.part("side_screen")
    side_screen.visual(
        Box((0.110, 0.008, 0.054)),
        origin=Origin(xyz=(0.055, 0.006, 0.000)),
        material=satin_gray,
        name="screen_frame",
    )
    side_screen.visual(
        Box((0.086, 0.0016, 0.038)),
        origin=Origin(xyz=(0.057, 0.0108, 0.000)),
        material=screen_black,
        name="lcd_glass",
    )
    side_screen.visual(
        Cylinder(radius=0.0035, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_gray,
        name="screen_hinge_barrel",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_geometry(_lens_ring_mesh(), "focus_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="focus_ring",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.0070, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_gray,
        name="dial_stem",
    )
    mode_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.012,
                body_style="faceted",
                edge_radius=0.0007,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0010, width=0.0014),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "mode_dial_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0058)),
        material=button_silver,
        name="dial_cap",
    )
    mode_dial.visual(
        Box((0.0020, 0.011, 0.0012)),
        origin=Origin(xyz=(0.0, -0.010, 0.0183)),
        material=label_white,
        name="dial_mark",
    )

    button_positions = (-0.030, 0.000, 0.030)
    buttons = []
    for index, x_pos in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0050, length=0.006),
            origin=Origin(xyz=(0.0, 0.0030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=button_silver,
            name="button_cap",
        )
        buttons.append((button, x_pos))

    model.articulation(
        "screen_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_screen,
        origin=Origin(xyz=(-0.066, 0.0495, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.45, velocity=2.0, lower=0.0, upper=math.radians(105)),
    )
    model.articulation(
        "lens_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.106, 0.0, 0.039)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=8.0),
    )
    model.articulation(
        "mode_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.025, 0.0, 0.0815)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.06, velocity=6.0),
    )
    for index, (button, x_pos) in enumerate(buttons):
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0430, 0.0178)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=0.04, lower=0.0, upper=0.0030),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    side_screen = object_model.get_part("side_screen")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")

    screen_hinge = object_model.get_articulation("screen_hinge")
    lens_spin = object_model.get_articulation("lens_ring_spin")
    dial_spin = object_model.get_articulation("mode_dial_spin")

    ctx.check(
        "lens ring is continuous",
        lens_spin.articulation_type == ArticulationType.CONTINUOUS and lens_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={lens_spin.articulation_type}, axis={lens_spin.axis}",
    )
    ctx.check(
        "mode dial is continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS and dial_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={dial_spin.articulation_type}, axis={dial_spin.axis}",
    )

    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="top_dial_plinth",
        min_gap=0.004,
        max_gap=0.008,
        name="dial cap is visibly separated above shell",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_stem",
        negative_elem="top_dial_plinth",
        min_gap=-0.0005,
        max_gap=0.001,
        name="dial stem mounts to upper shell",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="focus_ring",
        elem_b="lens_barrel",
        min_overlap=0.018,
        name="focus ring encircles the barrel length",
    )
    ctx.expect_within(
        body,
        lens_ring,
        axes="yz",
        inner_elem="lens_barrel",
        outer_elem="focus_ring",
        margin=0.001,
        name="lens barrel stays inside focus ring bore footprint",
    )

    closed_aabb = ctx.part_world_aabb(side_screen)
    with ctx.pose({screen_hinge: math.radians(95)}):
        open_aabb = ctx.part_world_aabb(side_screen)
    ctx.check(
        "side screen rotates outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.030,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"button_{index}_press")
        limits = joint.motion_limits
        ctx.check(
            f"button {index} is independent prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, -1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper == 0.0030,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="button_cap",
            negative_elem="button_well",
            min_gap=-0.0005,
            max_gap=0.001,
            name=f"button {index} seats in body wall",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.0030}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index} depresses inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.0025,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
