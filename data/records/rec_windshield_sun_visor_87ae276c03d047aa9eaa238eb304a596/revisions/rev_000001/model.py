from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_windshield_sun_visor")

    metal = model.material("powder_coated_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    bare_steel = model.material("bare_wear_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    bronze = model.material("replaceable_bronze_bushing", rgba=(0.72, 0.48, 0.22, 1.0))
    rubber = model.material("black_service_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    panel_fabric = model.material("charcoal_visor_face", rgba=(0.11, 0.12, 0.13, 1.0))
    access_gray = model.material("maintenance_cover_gray", rgba=(0.33, 0.35, 0.36, 1.0))
    warning_yellow = model.material("inspection_label_yellow", rgba=(0.95, 0.76, 0.18, 1.0))

    roof = model.part("roof_bracket")
    roof.visual(
        Box((0.70, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.030, 0.075)),
        material=metal,
        name="roof_backing_plate",
    )
    roof.visual(
        Box((0.58, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, -0.012, 0.070)),
        material=metal,
        name="welded_roof_rib",
    )
    for idx, x in enumerate((-0.055, 0.055)):
        roof.visual(
            Box((0.030, 0.075, 0.095)),
            origin=Origin(xyz=(x, 0.000, 0.025)),
            material=metal,
            name=f"primary_cheek_{idx}",
        )
    roof.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="primary_shaft",
    )
    for idx, x in enumerate((-0.084, 0.084)):
        roof.visual(
            Cylinder(radius=0.022, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name=f"outer_bushing_{idx}",
        )
    for idx, (x, y) in enumerate(
        ((-0.285, 0.050), (-0.155, 0.050), (0.155, 0.050), (0.285, 0.050))
    ):
        roof.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(x, y, 0.0875)),
            material=bare_steel,
            name=f"roof_bolt_{idx}",
        )
    roof.visual(
        Box((0.125, 0.054, 0.006)),
        origin=Origin(xyz=(0.0, 0.036, 0.087)),
        material=access_gray,
        name="roof_access_cover",
    )
    roof.visual(
        Box((0.060, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, 0.091)),
        material=warning_yellow,
        name="torque_label",
    )

    arm = model.part("hinge_arm")
    arm.visual(
        Cylinder(radius=0.026, length=0.068),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="primary_hub",
    )
    arm.visual(
        Box((0.054, 0.120, 0.128)),
        origin=Origin(xyz=(0.0, -0.065, -0.086)),
        material=metal,
        name="drop_arm_web",
    )
    for idx, x in enumerate((-0.033, 0.033)):
        arm.visual(
            Box((0.011, 0.105, 0.150)),
            origin=Origin(xyz=(x, -0.071, -0.097)),
            material=bare_steel,
            name=f"side_gusset_{idx}",
        )
    arm.visual(
        Box((0.044, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, -0.055, -0.025)),
        material=access_gray,
        name="primary_grease_plate",
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(xyz=(0.0, -0.120, -0.180)),
        material=bare_steel,
        name="secondary_shaft",
    )
    arm.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, -0.120, -0.142)),
        material=metal,
        name="secondary_top_collar",
    )
    arm.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, -0.120, -0.254)),
        material=metal,
        name="secondary_retainer_collar",
    )
    arm.visual(
        Box((0.070, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.120, -0.267)),
        material=bare_steel,
        name="retainer_clip",
    )
    arm.visual(
        Box((0.060, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, -0.128, -0.098)),
        material=warning_yellow,
        name="service_index_mark",
    )

    panel = model.part("visor_panel")
    panel_profile = rounded_rect_profile(0.580, 0.250, 0.035, corner_segments=8)
    panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(panel_profile, 0.026, center=True),
        "rounded_visor_panel",
    )
    panel.visual(
        panel_mesh,
        origin=Origin(xyz=(0.0, -0.085, -0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_fabric,
        name="panel_core",
    )
    panel.visual(
        Cylinder(radius=0.027, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=bronze,
        name="pivot_sleeve",
    )
    panel.visual(
        Box((0.108, 0.046, 0.035)),
        origin=Origin(xyz=(0.0, -0.050, -0.052)),
        material=metal,
        name="sleeve_bridge",
    )
    panel.visual(
        Box((0.505, 0.040, 0.036)),
        origin=Origin(xyz=(0.0, -0.084, -0.060)),
        material=metal,
        name="top_service_rail",
    )
    panel.visual(
        Box((0.550, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, -0.085, -0.318)),
        material=rubber,
        name="bottom_edge_pad",
    )
    for idx, x in enumerate((-0.302, 0.302)):
        panel.visual(
            Box((0.024, 0.032, 0.225)),
            origin=Origin(xyz=(x * 0.976, -0.085, -0.195)),
            material=rubber,
            name=f"side_edge_pad_{idx}",
        )
    panel.visual(
        Box((0.165, 0.006, 0.080)),
        origin=Origin(xyz=(0.0, -0.101, -0.205)),
        material=access_gray,
        name="service_cover",
    )
    for idx, (x, z) in enumerate(
        ((-0.065, -0.175), (0.065, -0.175), (-0.065, -0.235), (0.065, -0.235))
    ):
        panel.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(x, -0.106, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bare_steel,
            name=f"cover_screw_{idx}",
        )
    for idx, x in enumerate((-0.205, 0.205)):
        panel.visual(
            Box((0.082, 0.016, 0.030)),
            origin=Origin(xyz=(x, -0.107, -0.300)),
            material=rubber,
            name=f"replaceable_bumper_{idx}",
        )
    panel.visual(
        Box((0.120, 0.006, 0.035)),
        origin=Origin(xyz=(-0.185, -0.1005, -0.128)),
        material=warning_yellow,
        name="service_label",
    )

    model.articulation(
        "primary_pivot",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=arm,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.45, friction=0.18),
    )
    model.articulation(
        "secondary_pivot",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=panel,
        origin=Origin(xyz=(0.0, -0.120, -0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-1.35, upper=1.35),
        motion_properties=MotionProperties(damping=0.38, friction=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    roof = object_model.get_part("roof_bracket")
    arm = object_model.get_part("hinge_arm")
    panel = object_model.get_part("visor_panel")
    primary = object_model.get_articulation("primary_pivot")
    secondary = object_model.get_articulation("secondary_pivot")

    ctx.allow_overlap(
        roof,
        arm,
        elem_a="primary_shaft",
        elem_b="primary_hub",
        reason=(
            "The primary pivot shaft is intentionally captured inside the "
            "replaceable hinge hub/bushing proxy."
        ),
    )
    ctx.allow_overlap(
        arm,
        panel,
        elem_a="secondary_shaft",
        elem_b="pivot_sleeve",
        reason=(
            "The secondary swing shaft is intentionally nested in the bronze "
            "service sleeve to show a field-replaceable wear interface."
        ),
    )

    ctx.expect_within(
        roof,
        arm,
        axes="yz",
        inner_elem="primary_shaft",
        outer_elem="primary_hub",
        margin=0.002,
        name="primary shaft is centered in hinge hub",
    )
    ctx.expect_overlap(
        roof,
        arm,
        axes="x",
        elem_a="primary_shaft",
        elem_b="primary_hub",
        min_overlap=0.055,
        name="primary hub retains shaft length",
    )
    ctx.expect_within(
        arm,
        panel,
        axes="xy",
        inner_elem="secondary_shaft",
        outer_elem="pivot_sleeve",
        margin=0.002,
        name="secondary shaft is centered in sleeve",
    )
    ctx.expect_overlap(
        arm,
        panel,
        axes="z",
        elem_a="secondary_shaft",
        elem_b="pivot_sleeve",
        min_overlap=0.060,
        name="secondary sleeve retains shaft height",
    )

    rest_panel_origin = ctx.part_world_position(panel)
    with ctx.pose({primary: 1.35}):
        folded_panel_origin = ctx.part_world_position(panel)
    ctx.check(
        "primary pivot folds visor upward toward roof",
        rest_panel_origin is not None
        and folded_panel_origin is not None
        and folded_panel_origin[2] > rest_panel_origin[2] + 0.20,
        details=f"rest={rest_panel_origin}, folded={folded_panel_origin}",
    )

    rest_panel_box = ctx.part_element_world_aabb(panel, elem="panel_core")
    with ctx.pose({secondary: 1.10}):
        swung_panel_box = ctx.part_element_world_aabb(panel, elem="panel_core")
    if rest_panel_box is None or swung_panel_box is None:
        ctx.fail("secondary pivot moves panel sideways", "panel_core AABB unavailable")
    else:
        rest_center_x = (rest_panel_box[0][0] + rest_panel_box[1][0]) * 0.5
        swung_center_x = (swung_panel_box[0][0] + swung_panel_box[1][0]) * 0.5
        ctx.check(
            "secondary pivot moves panel sideways",
            swung_center_x > rest_center_x + 0.055,
            details=f"rest_center_x={rest_center_x}, swung_center_x={swung_center_x}",
        )

    return ctx.report()


object_model = build_object_model()
