from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PANEL_LENGTH = 0.420
PANEL_HEIGHT = 0.160
PANEL_THICKNESS = 0.025
HINGE_X = 0.078
HINGE_Z = -0.050


def _padded_panel_mesh():
    """A long, thin rounded-rectangle pad extruded through its thickness."""
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(PANEL_LENGTH, PANEL_HEIGHT, 0.026, corner_segments=10),
            PANEL_THICKNESS,
        ),
        "rounded_padded_visor_panel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_windshield_sun_visor")

    headliner = model.material("warm_grey_headliner", rgba=(0.70, 0.68, 0.62, 1.0))
    bracket_plastic = model.material("matte_grey_plastic", rgba=(0.43, 0.43, 0.40, 1.0))
    padded_vinyl = model.material("light_taupe_padded_vinyl", rgba=(0.78, 0.74, 0.66, 1.0))
    seam_dark = model.material("recessed_seam_shadow", rgba=(0.35, 0.32, 0.28, 1.0))
    mirror_glass = model.material("slightly_blue_mirror", rgba=(0.62, 0.74, 0.82, 0.55))
    label_paper = model.material("airbag_warning_label", rgba=(0.92, 0.78, 0.34, 1.0))

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.620, 0.140, 0.025)),
        origin=Origin(xyz=(0.220, 0.000, 0.0125)),
        material=headliner,
        name="headliner_patch",
    )
    roof_bracket.visual(
        Box((0.070, 0.055, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.0045)),
        material=bracket_plastic,
        name="mounting_plate",
    )
    roof_bracket.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.016)),
        material=bracket_plastic,
        name="pivot_socket",
    )
    roof_bracket.visual(
        Box((0.052, 0.030, 0.014)),
        origin=Origin(xyz=(0.418, 0.000, -0.006)),
        material=bracket_plastic,
        name="retainer_clip",
    )
    roof_bracket.visual(
        Box((0.030, 0.020, 0.008)),
        origin=Origin(xyz=(0.418, 0.000, -0.017)),
        material=bracket_plastic,
        name="clip_lip",
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.000, 0.000, -0.025)),
        material=bracket_plastic,
        name="vertical_post",
    )
    pivot_arm.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_plastic,
        name="elbow_boss",
    )
    pivot_arm.visual(
        Cylinder(radius=0.0085, length=HINGE_X),
        origin=Origin(xyz=(HINGE_X / 2.0, 0.000, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_plastic,
        name="horizontal_arm",
    )
    pivot_arm.visual(
        Cylinder(radius=0.0070, length=0.090),
        origin=Origin(xyz=(HINGE_X + 0.045, 0.000, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bracket_plastic,
        name="hinge_pin",
    )
    pivot_arm.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(HINGE_X - 0.020, 0.000, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bracket_plastic,
        name="hinge_cheek",
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        _padded_panel_mesh(),
        # Mesh starts as XY rounded rectangle extruded along Z; rotate it so the
        # long rounded rectangle hangs in the local XZ plane with thickness in Y.
        origin=Origin(
            xyz=(PANEL_LENGTH / 2.0, 0.000, -PANEL_HEIGHT / 2.0 - 0.009),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=padded_vinyl,
        name="padded_panel",
    )
    visor_panel.visual(
        Cylinder(radius=0.013, length=0.120),
        origin=Origin(xyz=(0.055, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=padded_vinyl,
        name="hinge_sleeve",
    )
    visor_panel.visual(
        Box((0.365, 0.0050, 0.010)),
        origin=Origin(xyz=(0.230, -0.0125, -0.018)),
        material=seam_dark,
        name="top_stitched_seam",
    )
    visor_panel.visual(
        Box((0.145, 0.0040, 0.058)),
        origin=Origin(xyz=(0.265, -0.0128, -0.078)),
        material=mirror_glass,
        name="vanity_mirror",
    )
    visor_panel.visual(
        Box((0.112, 0.0040, 0.036)),
        origin=Origin(xyz=(0.140, -0.0128, -0.118)),
        material=label_paper,
        name="warning_label",
    )

    model.articulation(
        "bracket_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_arm,
        origin=Origin(xyz=(0.000, 0.000, -0.023)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.57),
    )
    model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_panel,
        origin=Origin(xyz=(HINGE_X, 0.000, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2, lower=-1.35, upper=0.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_bracket")
    arm = object_model.get_part("pivot_arm")
    panel = object_model.get_part("visor_panel")
    side_joint = object_model.get_articulation("bracket_to_arm")
    flip_joint = object_model.get_articulation("arm_to_panel")

    ctx.allow_overlap(
        arm,
        panel,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The visor's hinge pin is intentionally captured inside the padded panel hinge sleeve.",
    )
    ctx.expect_within(
        arm,
        panel,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="hinge pin is centered in sleeve",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.050,
        name="hinge pin remains inserted in sleeve",
    )

    ctx.expect_gap(
        roof,
        panel,
        axis="z",
        min_gap=0.025,
        positive_elem="pivot_socket",
        negative_elem="hinge_sleeve",
        name="deployed visor clears the roof socket",
    )

    panel_aabb = ctx.part_element_world_aabb(panel, elem="padded_panel")
    if panel_aabb is None:
        ctx.fail("padded panel dimensions available", "Expected padded_panel world AABB.")
    else:
        mins, maxs = panel_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "visor panel is long and thin",
            size[0] > 0.38 and size[2] > 0.13 and size[1] < 0.035,
            details=f"padded_panel size={size!r}",
        )

    deployed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({flip_joint: -1.20}):
        stowed_aabb = ctx.part_world_aabb(panel)
    if deployed_aabb is not None and stowed_aabb is not None:
        deployed_center_z = (float(deployed_aabb[0][2]) + float(deployed_aabb[1][2])) / 2.0
        stowed_center_z = (float(stowed_aabb[0][2]) + float(stowed_aabb[1][2])) / 2.0
        ctx.check(
            "horizontal hinge flips visor upward to stowed pose",
            stowed_center_z > deployed_center_z + 0.035,
            details=f"deployed_z={deployed_center_z:.3f}, stowed_z={stowed_center_z:.3f}",
        )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({side_joint: 1.20}):
        swung_pos = ctx.part_world_position(panel)
    ctx.check(
        "vertical pivot swings visor sideways",
        rest_pos is not None and swung_pos is not None and swung_pos[1] > rest_pos[1] + 0.055,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
