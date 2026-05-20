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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_searchlight_tower")

    anodized = model.material("matte_anodized", rgba=(0.18, 0.20, 0.22, 1.0))
    dark = model.material("black_hardcoat", rgba=(0.02, 0.025, 0.03, 1.0))
    machined = model.material("machined_datum", rgba=(0.72, 0.72, 0.68, 1.0))
    bronze = model.material("bronze_bearing", rgba=(0.72, 0.46, 0.19, 1.0))
    white = model.material("etched_white", rgba=(0.92, 0.92, 0.84, 1.0))
    red = model.material("red_pointer", rgba=(0.95, 0.06, 0.03, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.25, 0.45, 0.72, 0.55))

    tower = model.part("tower")
    tower.visual(
        Box((0.60, 0.48, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark,
        name="base_plate",
    )
    for i, (x, y) in enumerate(
        ((0.245, 0.185), (0.245, -0.185), (-0.245, 0.185), (-0.245, -0.185))
    ):
        tower.visual(
            Cylinder(radius=0.030, length=0.014),
            origin=Origin(xyz=(x, y, -0.007)),
            material=machined,
            name=f"leveling_foot_{i}",
        )
    tower.visual(
        Box((0.135, 0.135, 1.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=anodized,
        name="mast_column",
    )
    tower.visual(
        Box((0.155, 0.018, 0.840)),
        origin=Origin(xyz=(0.077, 0.0, 0.500)),
        material=machined,
        name="front_datum_flat",
    )
    tower.visual(
        Box((0.250, 0.230, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 1.095)),
        material=anodized,
        name="mast_cap",
    )
    tower.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        material=anodized,
        name="bearing_neck",
    )
    tower.visual(
        Cylinder(radius=0.240, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 1.174)),
        material=machined,
        name="azimuth_scale",
    )
    for i in range(24):
        angle = 2.0 * math.pi * i / 24.0
        major = i % 6 == 0
        radius = 0.202
        length = 0.044 if major else 0.027
        width = 0.005 if major else 0.003
        tower.visual(
            Box((length, width, 0.003)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 1.1815),
                rpy=(0.0, 0.0, angle),
            ),
            material=white,
            name=f"azimuth_tick_{i}",
        )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.145, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=anodized,
        name="turntable_upper",
    )
    pan_stage.visual(
        Box((0.300, 0.420, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=anodized,
        name="yoke_foot",
    )
    pan_stage.visual(
        Box((0.050, 0.080, 0.060)),
        origin=Origin(xyz=(-0.175, 0.0, 0.070)),
        material=anodized,
        name="pan_knob_boss",
    )
    for side, y, bearing_name in (
        (0, 0.170, "bearing_block_0"),
        (1, -0.170, "bearing_block_1"),
    ):
        pan_stage.visual(
            Box((0.075, 0.050, 0.216)),
            origin=Origin(xyz=(0.0, y, 0.172)),
            material=anodized,
            name=f"yoke_post_{side}_lower",
        )
        pan_stage.visual(
            Box((0.075, 0.050, 0.104)),
            origin=Origin(xyz=(0.0, y, 0.452)),
            material=anodized,
            name=f"yoke_post_{side}_upper",
        )
        pan_stage.visual(
            Cylinder(radius=0.060, length=0.070),
            origin=Origin(xyz=(0.0, y, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bronze,
            name=bearing_name,
        )
    pan_stage.visual(
        Box((0.190, 0.004, 0.170)),
        origin=Origin(xyz=(0.0, -0.207, 0.340)),
        material=machined,
        name="tilt_scale_plate",
    )
    for i, angle_deg in enumerate((-45, -30, -15, 0, 15, 30, 45)):
        angle = math.radians(angle_deg)
        radius = 0.074
        pan_stage.visual(
            Box((0.022 if angle_deg % 30 == 0 else 0.014, 0.002, 0.003)),
            origin=Origin(
                xyz=(radius * math.sin(angle), -0.210, 0.340 + radius * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=white,
            name=f"tilt_tick_{i}",
        )
    pan_stage.visual(
        Box((0.100, 0.015, 0.006)),
        origin=Origin(xyz=(0.178, 0.0, 0.027)),
        material=red,
        name="azimuth_pointer",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.026, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="trunnion_shaft",
    )
    lamp_head.visual(
        Cylinder(radius=0.105, length=0.350),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lamp_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.112, length=0.025),
        origin=Origin(xyz=(0.252, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.095, length=0.018),
        origin=Origin(xyz=(0.266, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.096, length=0.035),
        origin=Origin(xyz=(-0.127, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="rear_cap",
    )
    for i, x in enumerate((-0.085, -0.065, -0.045)):
        lamp_head.visual(
            Cylinder(radius=0.112, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined,
            name=f"cooling_fin_{i}",
        )
    lamp_head.visual(
        Box((0.200, 0.025, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, 0.120)),
        material=machined,
        name="sight_rail",
    )
    for i, x in enumerate((-0.040, 0.120)):
        lamp_head.visual(
            Box((0.020, 0.020, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.115)),
            material=machined,
            name=f"sight_post_{i}",
        )

    pan_knob = model.part("pan_knob")
    pan_knob.visual(
        Cylinder(radius=0.008, length=0.035),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="knob_stem",
    )
    pan_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.055,
                0.030,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=18, depth=0.0010),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            ),
            "pan_knob_cap",
        ),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="knob_cap",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="knob_stem",
    )
    tilt_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.065,
                0.035,
                body_style="lobed",
                grip=KnobGrip(style="fluted", count=12, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            ),
            "tilt_knob_cap",
        ),
        origin=Origin(xyz=(0.0, -0.0475, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_cap",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.6, lower=-0.40, upper=0.75),
    )
    model.articulation(
        "pan_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=pan_stage,
        child=pan_knob,
        origin=Origin(xyz=(-0.200, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "tilt_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=pan_stage,
        child=tilt_knob,
        origin=Origin(xyz=(0.0, -0.209, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan_knob = object_model.get_part("pan_knob")
    tilt_knob = object_model.get_part("tilt_knob")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        pan_stage,
        lamp_head,
        elem_a="bearing_block_0",
        elem_b="trunnion_shaft",
        reason="The tilt trunnion shaft is intentionally captured inside the machined bearing block proxy.",
    )
    ctx.allow_overlap(
        pan_stage,
        lamp_head,
        elem_a="bearing_block_1",
        elem_b="trunnion_shaft",
        reason="The opposite tilt bearing intentionally contains the same through-shaft.",
    )

    ctx.expect_contact(
        pan_stage,
        tower,
        elem_a="turntable_upper",
        elem_b="azimuth_scale",
        contact_tol=1e-5,
        name="pan turntable seated on azimuth scale",
    )
    ctx.expect_contact(
        pan_knob,
        pan_stage,
        elem_a="knob_stem",
        elem_b="pan_knob_boss",
        contact_tol=1e-5,
        name="pan adjustment knob stem touches boss",
    )
    ctx.expect_contact(
        tilt_knob,
        pan_stage,
        elem_a="knob_stem",
        elem_b="tilt_scale_plate",
        contact_tol=1e-5,
        name="tilt lock knob stem touches scale plate",
    )

    ctx.expect_within(
        lamp_head,
        pan_stage,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_block_0",
        margin=0.0,
        name="trunnion centered in first bearing bore proxy",
    )
    ctx.expect_within(
        lamp_head,
        pan_stage,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_block_1",
        margin=0.0,
        name="trunnion centered in second bearing bore proxy",
    )
    ctx.expect_overlap(
        pan_stage,
        lamp_head,
        axes="y",
        elem_a="bearing_block_0",
        elem_b="trunnion_shaft",
        min_overlap=0.060,
        name="first bearing retains shaft length",
    )
    ctx.expect_overlap(
        pan_stage,
        lamp_head,
        axes="y",
        elem_a="bearing_block_1",
        elem_b="trunnion_shaft",
        min_overlap=0.060,
        name="second bearing retains shaft length",
    )
    ctx.expect_gap(
        pan_stage,
        lamp_head,
        axis="y",
        positive_elem="bearing_block_0",
        negative_elem="lamp_barrel",
        min_gap=0.020,
        max_gap=0.060,
        name="positive yoke cheek has controlled lamp clearance",
    )
    ctx.expect_gap(
        lamp_head,
        pan_stage,
        axis="y",
        positive_elem="lamp_barrel",
        negative_elem="bearing_block_1",
        min_gap=0.020,
        max_gap=0.060,
        name="negative yoke cheek has controlled lamp clearance",
    )

    ctx.check(
        "pan and tilt axes are separated",
        abs(tilt_axis.origin.xyz[2]) > 0.25
        and pan_axis.axis == (0.0, 0.0, 1.0)
        and tilt_axis.axis == (0.0, -1.0, 0.0),
        details=f"pan_axis={pan_axis.axis}, tilt_axis={tilt_axis.axis}, tilt_origin={tilt_axis.origin.xyz}",
    )

    def _center_y(bounds):
        return None if bounds is None else 0.5 * (bounds[0][1] + bounds[1][1])

    def _max_z(bounds):
        return None if bounds is None else bounds[1][2]

    pointer_rest = ctx.part_element_world_aabb(pan_stage, elem="azimuth_pointer")
    with ctx.pose({pan_axis: math.pi / 2.0}):
        pointer_turned = ctx.part_element_world_aabb(pan_stage, elem="azimuth_pointer")
    ctx.check(
        "pan joint sweeps azimuth pointer",
        pointer_rest is not None
        and pointer_turned is not None
        and abs(_center_y(pointer_turned) - _center_y(pointer_rest)) > 0.12,
        details=f"rest={pointer_rest}, turned={pointer_turned}",
    )

    lens_rest = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    with ctx.pose({tilt_axis: 0.50}):
        lens_raised = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    ctx.check(
        "positive tilt raises searchlight nose",
        lens_rest is not None
        and lens_raised is not None
        and _max_z(lens_raised) > _max_z(lens_rest) + 0.05,
        details=f"rest={lens_rest}, raised={lens_raised}",
    )

    return ctx.report()


object_model = build_object_model()
