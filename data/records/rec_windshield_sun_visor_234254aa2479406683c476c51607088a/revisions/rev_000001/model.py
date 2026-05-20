from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_windshield_sun_visor")

    headliner = model.material("warm_grey_headliner", rgba=(0.55, 0.53, 0.49, 1.0))
    black_plastic = model.material("black_acetal_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    dark_metal = model.material("black_phosphate_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    visor_vinyl = model.material("molded_light_grey_vinyl", rgba=(0.63, 0.61, 0.56, 1.0))
    seam = model.material("heat_staked_edge_seam", rgba=(0.42, 0.41, 0.38, 1.0))
    screw = model.material("zinc_screw_heads", rgba=(0.62, 0.61, 0.57, 1.0))

    # A single molded roof base carries the visible screw cover, snap socket,
    # and far-end retainer clip.  The thin header strip makes the attachment
    # path to the clip explicit without adding a separate loose part.
    roof_base = model.part("roof_base")
    roof_base.visual(
        Box((0.120, 0.070, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
        material=headliner,
        name="mount_plate",
    )
    roof_base.visual(
        Cylinder(radius=0.027, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
        material=black_plastic,
        name="pivot_socket",
    )
    roof_base.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(-0.035, 0.000, 0.018)),
        material=screw,
        name="screw_head_0",
    )
    roof_base.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.035, 0.000, 0.018)),
        material=screw,
        name="screw_head_1",
    )
    roof_base.visual(
        Box((0.555, 0.025, 0.008)),
        origin=Origin(xyz=(0.280, 0.040, 0.004)),
        material=headliner,
        name="roof_header_strip",
    )
    roof_base.visual(
        Box((0.020, 0.020, 0.070)),
        origin=Origin(xyz=(0.555, 0.040, -0.030)),
        material=black_plastic,
        name="retainer_drop",
    )
    roof_base.visual(
        Box((0.050, 0.012, 0.016)),
        origin=Origin(xyz=(0.535, 0.030, -0.070)),
        material=black_plastic,
        name="retainer_hook",
    )

    # The rotating arm is one low-cost metal rod with an overmolded collar.
    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.012, length=0.056),
        origin=Origin(xyz=(0.000, 0.000, -0.028)),
        material=dark_metal,
        name="swing_post",
    )
    hinge_arm.visual(
        Cylinder(radius=0.021, length=0.014),
        origin=Origin(xyz=(0.000, 0.000, -0.046)),
        material=black_plastic,
        name="detent_collar",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0065, length=0.575),
        origin=Origin(xyz=(0.2875, 0.000, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_shaft",
    )
    hinge_arm.visual(
        Box((0.035, 0.028, 0.024)),
        origin=Origin(xyz=(0.022, 0.000, -0.055)),
        material=black_plastic,
        name="overmold_elbow",
    )
    hinge_arm.visual(
        Box((0.020, 0.012, 0.012)),
        origin=Origin(xyz=(0.036, -0.011, -0.046)),
        material=black_plastic,
        name="snap_detent",
    )

    # The visor panel is a single rounded extrusion-like slab with a molded
    # hinge sleeve and heat-staked perimeter detail.  No vanity mirror is added:
    # the cost target favors the fewest molded and assembled pieces.
    visor_panel = model.part("visor_panel")
    panel_shell = ExtrudeGeometry(
        rounded_rect_profile(0.440, 0.180, 0.026, corner_segments=10),
        0.022,
        cap=True,
        center=True,
    )
    visor_panel.visual(
        mesh_from_geometry(panel_shell, "rounded_visor_panel"),
        origin=Origin(xyz=(0.220, 0.000, -0.100), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=visor_vinyl,
        name="padded_panel",
    )
    visor_panel.visual(
        Cylinder(radius=0.0135, length=0.440),
        origin=Origin(xyz=(0.220, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=visor_vinyl,
        name="hinge_sleeve",
    )
    visor_panel.visual(
        Box((0.410, 0.004, 0.006)),
        origin=Origin(xyz=(0.220, -0.012, -0.016)),
        material=seam,
        name="top_seam",
    )
    visor_panel.visual(
        Box((0.410, 0.004, 0.006)),
        origin=Origin(xyz=(0.220, -0.012, -0.184)),
        material=seam,
        name="bottom_seam",
    )
    visor_panel.visual(
        Box((0.006, 0.004, 0.150)),
        origin=Origin(xyz=(0.012, -0.012, -0.100)),
        material=seam,
        name="hinge_side_seam",
    )
    visor_panel.visual(
        Box((0.006, 0.004, 0.150)),
        origin=Origin(xyz=(0.428, -0.012, -0.100)),
        material=seam,
        name="clip_side_seam",
    )
    visor_panel.visual(
        Box((0.070, 0.006, 0.010)),
        origin=Origin(xyz=(0.300, -0.016, -0.178)),
        material=seam,
        name="pull_lip",
    )
    visor_panel.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(0.430, 0.000, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clip_pin",
    )

    model.articulation(
        "secondary_pivot",
        ArticulationType.REVOLUTE,
        parent=roof_base,
        child=hinge_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.57),
    )
    model.articulation(
        "primary_hinge",
        ArticulationType.REVOLUTE,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.130, 0.000, -0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof_base = object_model.get_part("roof_base")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    secondary = object_model.get_articulation("secondary_pivot")
    primary = object_model.get_articulation("primary_hinge")

    ctx.allow_overlap(
        roof_base,
        hinge_arm,
        elem_a="pivot_socket",
        elem_b="swing_post",
        reason="The steel swing post is intentionally captured inside the molded roof socket.",
    )
    ctx.expect_within(
        hinge_arm,
        roof_base,
        axes="xy",
        inner_elem="swing_post",
        outer_elem="pivot_socket",
        margin=0.002,
        name="swing post centered in roof socket",
    )
    ctx.expect_overlap(
        hinge_arm,
        roof_base,
        axes="z",
        elem_a="swing_post",
        elem_b="pivot_socket",
        min_overlap=0.015,
        name="swing post retained axially in socket",
    )

    ctx.allow_overlap(
        hinge_arm,
        visor_panel,
        elem_a="hinge_shaft",
        elem_b="hinge_sleeve",
        reason="The hinge shaft is intentionally modeled as a captured solid proxy inside the molded visor sleeve.",
    )
    ctx.expect_within(
        hinge_arm,
        visor_panel,
        axes="yz",
        inner_elem="hinge_shaft",
        outer_elem="hinge_sleeve",
        margin=0.001,
        name="hinge shaft centered in sleeve bore",
    )
    ctx.expect_overlap(
        hinge_arm,
        visor_panel,
        axes="x",
        elem_a="hinge_shaft",
        elem_b="hinge_sleeve",
        min_overlap=0.400,
        name="hinge shaft spans molded sleeve",
    )

    deployed_aabb = ctx.part_world_aabb(visor_panel)
    with ctx.pose({primary: 1.20}):
        folded_aabb = ctx.part_world_aabb(visor_panel)
    ctx.check(
        "primary hinge folds visor upward",
        deployed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > deployed_aabb[0][2] + 0.045,
        details=f"deployed={deployed_aabb}, folded={folded_aabb}",
    )

    front_position = ctx.part_world_position(visor_panel)
    with ctx.pose({secondary: 1.20}):
        side_position = ctx.part_world_position(visor_panel)
    ctx.check(
        "secondary pivot swings visor sideways",
        front_position is not None
        and side_position is not None
        and side_position[1] > front_position[1] + 0.09,
        details=f"front={front_position}, side={side_position}",
    )

    return ctx.report()


object_model = build_object_model()
