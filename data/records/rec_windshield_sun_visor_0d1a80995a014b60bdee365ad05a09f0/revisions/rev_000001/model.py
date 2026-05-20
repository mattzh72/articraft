from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_windshield_sun_visor")

    roof_liner = model.material("warm_gray_roof_liner", rgba=(0.66, 0.62, 0.54, 1.0))
    padded_fabric = model.material("padded_light_gray_fabric", rgba=(0.55, 0.56, 0.54, 1.0))
    seam_fabric = model.material("dark_gray_seam_binding", rgba=(0.23, 0.24, 0.23, 1.0))
    black_plastic = model.material("satin_black_plastic", rgba=(0.02, 0.022, 0.02, 1.0))
    brushed_pin = model.material("dark_brushed_metal", rgba=(0.34, 0.34, 0.32, 1.0))
    warning_yellow = model.material("muted_warning_label", rgba=(0.82, 0.70, 0.28, 1.0))

    # A slim slice of roof/headliner keeps the visor bracket visibly mounted.
    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.56, 0.13, 0.018)),
        origin=Origin(xyz=(0.22, 0.0, 0.070)),
        material=roof_liner,
        name="roof_header",
    )
    mount_plate_shape = (
        cq.Workplane("XY")
        .box(0.13, 0.075, 0.018)
        .edges("|Z")
        .fillet(0.012)
    )
    roof_bracket.visual(
        mesh_from_cadquery(mount_plate_shape, "rounded_mount_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=black_plastic,
        name="mount_plate",
    )
    roof_bracket.visual(
        Cylinder(radius=0.028, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=black_plastic,
        name="upper_pivot_boss",
    )
    for x in (-0.040, 0.040):
        roof_bracket.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0415)),
            material=brushed_pin,
            name=f"screw_head_{0 if x < 0 else 1}",
        )
    # The opposite roof clip is static, but helps the object read as a real car visor.
    roof_bracket.visual(
        Box((0.055, 0.018, 0.012)),
        origin=Origin(xyz=(0.425, -0.047, 0.055)),
        material=black_plastic,
        name="retainer_bridge",
    )
    roof_bracket.visual(
        Box((0.010, 0.012, 0.034)),
        origin=Origin(xyz=(0.404, -0.047, 0.032)),
        material=black_plastic,
        name="retainer_prong_0",
    )
    roof_bracket.visual(
        Box((0.010, 0.012, 0.034)),
        origin=Origin(xyz=(0.446, -0.047, 0.032)),
        material=black_plastic,
        name="retainer_prong_1",
    )

    # The side-swinging arm sits below the bracket and carries the horizontal hinge
    # line out toward the cabin so its flip axis is deliberately easy to see.
    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Cylinder(radius=0.019, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=brushed_pin,
        name="vertical_pivot_pin",
    )
    pivot_arm.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=black_plastic,
        name="drop_post",
    )
    pivot_arm.visual(
        Box((0.024, 0.062, 0.016)),
        origin=Origin(xyz=(0.0, -0.031, -0.050)),
        material=black_plastic,
        name="offset_arm",
    )
    pivot_arm.visual(
        Cylinder(radius=0.008, length=0.472),
        origin=Origin(xyz=(0.215, -0.055, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_pin,
        name="hinge_pin",
    )
    pivot_arm.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, -0.055, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="hinge_end_knuckle",
    )

    visor_panel = model.part("visor_panel")
    visor_body_shape = (
        cq.Workplane("XY")
        .box(0.430, 0.028, 0.180)
        .edges()
        .fillet(0.008)
    )
    visor_panel.visual(
        mesh_from_cadquery(visor_body_shape, "rounded_padded_visor"),
        origin=Origin(xyz=(0.215, 0.045, -0.105)),
        material=padded_fabric,
        name="padded_panel",
    )
    visor_panel.visual(
        Box((0.360, 0.003, 0.012)),
        origin=Origin(xyz=(0.235, 0.031, -0.164)),
        material=warning_yellow,
        name="warning_label",
    )
    # Soft binding strips are slightly proud on the cabin-facing surface.
    visor_panel.visual(
        Box((0.395, 0.004, 0.008)),
        origin=Origin(xyz=(0.215, 0.031, -0.022)),
        material=seam_fabric,
        name="top_binding",
    )
    visor_panel.visual(
        Box((0.395, 0.004, 0.008)),
        origin=Origin(xyz=(0.215, 0.031, -0.188)),
        material=seam_fabric,
        name="bottom_binding",
    )
    visor_panel.visual(
        Box((0.008, 0.004, 0.152)),
        origin=Origin(xyz=(0.018, 0.031, -0.105)),
        material=seam_fabric,
        name="end_binding_0",
    )
    visor_panel.visual(
        Box((0.008, 0.004, 0.152)),
        origin=Origin(xyz=(0.412, 0.031, -0.105)),
        material=seam_fabric,
        name="end_binding_1",
    )
    for i, x in enumerate((0.058, 0.215, 0.372)):
        visor_panel.visual(
            Cylinder(radius=0.014, length=0.070),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name=f"hinge_sleeve_{i}",
        )
        visor_panel.visual(
            Box((0.044, 0.034, 0.019)),
            origin=Origin(xyz=(x, 0.027, -0.0165)),
            material=black_plastic,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "side_pivot",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, -0.055, -0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_bracket")
    arm = object_model.get_part("pivot_arm")
    panel = object_model.get_part("visor_panel")
    side = object_model.get_articulation("side_pivot")
    flip = object_model.get_articulation("flip_hinge")

    for i in range(3):
        sleeve = f"hinge_sleeve_{i}"
        ctx.allow_overlap(
            arm,
            panel,
            elem_a="hinge_pin",
            elem_b=sleeve,
            reason="The exposed metal hinge pin is intentionally captured inside each visor hinge sleeve.",
        )
        ctx.expect_within(
            arm,
            panel,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=sleeve,
            margin=0.002,
            name=f"hinge pin is centered in sleeve {i}",
        )
        ctx.expect_overlap(
            arm,
            panel,
            axes="x",
            elem_a="hinge_pin",
            elem_b=sleeve,
            min_overlap=0.050,
            name=f"hinge pin passes through sleeve {i}",
        )

    ctx.expect_contact(
        roof,
        arm,
        elem_a="upper_pivot_boss",
        elem_b="vertical_pivot_pin",
        contact_tol=0.001,
        name="side pivot pin seats under bracket boss",
    )
    ctx.expect_gap(
        roof,
        panel,
        axis="z",
        min_gap=0.035,
        name="visor hangs below roof hardware",
    )

    rest_panel = ctx.part_element_world_aabb(panel, elem="padded_panel")
    with ctx.pose({flip: 1.25}):
        raised_panel = ctx.part_element_world_aabb(panel, elem="padded_panel")
    ctx.check(
        "horizontal hinge flips visor upward",
        rest_panel is not None
        and raised_panel is not None
        and raised_panel[1][2] > rest_panel[1][2] + 0.055,
        details=f"rest={rest_panel}, raised={raised_panel}",
    )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({side: 1.20}):
        swung_pos = ctx.part_world_position(panel)
    ctx.check(
        "vertical pivot swings visor sideways",
        rest_pos is not None and swung_pos is not None and swung_pos[0] > rest_pos[0] + 0.040,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
