from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    red = model.material("molded_red", rgba=(0.74, 0.08, 0.045, 1.0))
    dark_red = model.material("dark_recess_red", rgba=(0.36, 0.035, 0.030, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.055, 0.060, 0.064, 1.0))
    metal = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    latch_yellow = model.material("yellow_latch", rgba=(0.95, 0.70, 0.08, 1.0))

    body = model.part("body")

    # Open molded bin: bottom, four walls, and an upper rim.  The separate wall
    # panels overlap at their seams so the body is one continuous supported shell.
    body.visual(
        Box((0.58, 0.34, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=red,
        name="bottom_pan",
    )
    body.visual(
        Box((0.58, 0.026, 0.340)),
        origin=Origin(xyz=(0.0, 0.157, 0.275)),
        material=red,
        name="side_wall_0",
    )
    body.visual(
        Box((0.58, 0.026, 0.340)),
        origin=Origin(xyz=(0.0, -0.157, 0.275)),
        material=red,
        name="side_wall_1",
    )
    body.visual(
        Box((0.030, 0.34, 0.340)),
        origin=Origin(xyz=(0.275, 0.0, 0.275)),
        material=red,
        name="front_wall",
    )
    body.visual(
        Box((0.030, 0.34, 0.340)),
        origin=Origin(xyz=(-0.275, 0.0, 0.275)),
        material=red,
        name="rear_wall",
    )
    body.visual(
        Box((0.59, 0.035, 0.030)),
        origin=Origin(xyz=(0.0, 0.162, 0.445)),
        material=dark_red,
        name="rim_side_0",
    )
    body.visual(
        Box((0.59, 0.035, 0.030)),
        origin=Origin(xyz=(0.0, -0.162, 0.445)),
        material=dark_red,
        name="rim_side_1",
    )
    body.visual(
        Box((0.038, 0.35, 0.030)),
        origin=Origin(xyz=(0.285, 0.0, 0.445)),
        material=dark_red,
        name="rim_front",
    )
    body.visual(
        Box((0.038, 0.35, 0.030)),
        origin=Origin(xyz=(-0.285, 0.0, 0.445)),
        material=dark_red,
        name="rim_rear",
    )

    # Recessed molded side treatment and fixed front latch receiver.
    for y, suffix in ((0.174, "0"), (-0.174, "1")):
        body.visual(
            Box((0.34, 0.012, 0.120)),
            origin=Origin(xyz=(0.035, y, 0.300)),
            material=dark_red,
            name=f"side_recess_{suffix}",
        )
        body.visual(
            Box((0.135, 0.014, 0.030)),
            origin=Origin(xyz=(-0.220, y, 0.164)),
            material=charcoal,
            name=f"wheel_arch_{suffix}",
        )
    body.visual(
        Box((0.016, 0.120, 0.052)),
        origin=Origin(xyz=(0.298, 0.0, 0.365)),
        material=charcoal,
        name="front_strike_plate",
    )

    # Front feet keep the toolbox level while the rear wheels carry the axle.
    for y, suffix in ((0.115, "0"), (-0.115, "1")):
        body.visual(
            Box((0.080, 0.070, 0.060)),
            origin=Origin(xyz=(0.220, y, 0.070)),
            material=black,
            name=f"front_foot_{suffix}",
        )

    # Rear steel axle and two recessed guide tubes for the telescoping handle.
    body.visual(
        Cylinder(radius=0.009, length=0.470),
        origin=Origin(xyz=(-0.220, 0.0, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="rear_axle_bar",
    )
    for y, suffix in ((0.150, "0"), (-0.150, "1")):
        body.visual(
            Box((0.046, 0.040, 0.050)),
            origin=Origin(xyz=(-0.220, y, 0.100)),
            material=charcoal,
            name=f"axle_mount_{suffix}",
        )
    body.visual(
        Cylinder(radius=0.030, length=0.029),
        origin=Origin(xyz=(-0.220, 0.1625, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_spacer_0",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.029),
        origin=Origin(xyz=(-0.220, -0.1625, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_spacer_1",
    )
    body.visual(
        Box((0.012, 0.260, 0.255)),
        origin=Origin(xyz=(-0.302, 0.0, 0.325)),
        material=dark_red,
        name="handle_recess_back",
    )
    body.visual(
        Box((0.012, 0.052, 0.300)),
        origin=Origin(xyz=(-0.304, 0.105, 0.310)),
        material=charcoal,
        name="guide_0_back",
    )
    body.visual(
        Box((0.055, 0.007, 0.300)),
        origin=Origin(xyz=(-0.337, 0.1245, 0.310)),
        material=charcoal,
        name="guide_0_side_0",
    )
    body.visual(
        Box((0.055, 0.007, 0.300)),
        origin=Origin(xyz=(-0.337, 0.0855, 0.310)),
        material=charcoal,
        name="guide_0_side_1",
    )
    body.visual(
        Box((0.012, 0.052, 0.300)),
        origin=Origin(xyz=(-0.304, -0.105, 0.310)),
        material=charcoal,
        name="guide_1_back",
    )
    body.visual(
        Box((0.055, 0.007, 0.300)),
        origin=Origin(xyz=(-0.337, -0.0855, 0.310)),
        material=charcoal,
        name="guide_1_side_0",
    )
    body.visual(
        Box((0.055, 0.007, 0.300)),
        origin=Origin(xyz=(-0.337, -0.1245, 0.310)),
        material=charcoal,
        name="guide_1_side_1",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.595, 0.360, 0.045)),
        origin=Origin(xyz=(0.2975, 0.0, 0.0225)),
        material=red,
        name="lid_panel",
    )
    lid.visual(
        Box((0.515, 0.280, 0.012)),
        origin=Origin(xyz=(0.305, 0.0, 0.051)),
        material=dark_red,
        name="lid_recess_panel",
    )
    lid.visual(
        Box((0.015, 0.112, 0.018)),
        origin=Origin(xyz=(0.6025, 0.0, 0.025)),
        material=latch_yellow,
        name="front_latch_root",
    )
    lid.visual(
        Box((0.020, 0.110, 0.064)),
        origin=Origin(xyz=(0.620, 0.0, 0.002)),
        material=latch_yellow,
        name="front_latch_tab",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.260),
        origin=Origin(xyz=(-0.006, 0.0, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_sleeve",
    )

    stage_0 = model.part("handle_stage_0")
    stage_0.visual(
        Box((0.007, 0.030, 0.420)),
        origin=Origin(xyz=(0.010, 0.105, -0.145)),
        material=metal,
        name="stage0_0_web",
    )
    stage_0.visual(
        Box((0.035, 0.006, 0.420)),
        origin=Origin(xyz=(-0.007, 0.118, -0.145)),
        material=metal,
        name="stage0_0_side_0",
    )
    stage_0.visual(
        Box((0.035, 0.006, 0.420)),
        origin=Origin(xyz=(-0.007, 0.092, -0.145)),
        material=metal,
        name="stage0_0_side_1",
    )
    stage_0.visual(
        Box((0.007, 0.030, 0.420)),
        origin=Origin(xyz=(0.010, -0.105, -0.145)),
        material=metal,
        name="stage0_1_web",
    )
    stage_0.visual(
        Box((0.035, 0.006, 0.420)),
        origin=Origin(xyz=(-0.007, -0.092, -0.145)),
        material=metal,
        name="stage0_1_side_0",
    )
    stage_0.visual(
        Box((0.035, 0.006, 0.420)),
        origin=Origin(xyz=(-0.007, -0.118, -0.145)),
        material=metal,
        name="stage0_1_side_1",
    )
    stage_0.visual(
        Box((0.026, 0.255, 0.032)),
        origin=Origin(xyz=(0.006, 0.0, 0.068)),
        material=metal,
        name="stage0_bridge",
    )

    stage_1 = model.part("handle_stage_1")
    stage_1.visual(
        Box((0.013, 0.020, 0.380)),
        origin=Origin(xyz=(-0.018, 0.105, -0.150)),
        material=metal,
        name="stage1_rod_0",
    )
    stage_1.visual(
        Box((0.013, 0.020, 0.380)),
        origin=Origin(xyz=(-0.018, -0.105, -0.150)),
        material=metal,
        name="stage1_rod_1",
    )
    stage_1.visual(
        Box((0.050, 0.275, 0.034)),
        origin=Origin(xyz=(-0.018, 0.0, 0.057)),
        material=black,
        name="pull_grip",
    )

    wheel_geom = WheelGeometry(
        0.052,
        0.046,
        rim=WheelRim(inner_radius=0.035, flange_height=0.006, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.017,
            width=0.030,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.024, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.020),
    )
    tire_geom = TireGeometry(
        0.075,
        0.056,
        inner_radius=0.052,
        tread=TireTread(style="block", depth=0.005, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.045),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    for y, suffix in ((0.205, "0"), (-0.205, "1")):
        wheel = model.part(f"rear_wheel_{suffix}")
        wheel.visual(
            mesh_from_geometry(wheel_geom, f"rear_wheel_{suffix}_rim"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=metal,
            name="rim",
        )
        wheel.visual(
            mesh_from_geometry(tire_geom, f"rear_wheel_{suffix}_tire"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.040),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="axle_bushing",
        )
        model.articulation(
            f"body_to_rear_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.220, y, 0.078)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.305, 0.0, 0.460)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_handle_stage_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage_0,
        origin=Origin(xyz=(-0.337, 0.0, 0.465)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.230),
    )
    model.articulation(
        "handle_stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.230),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    stage_0 = object_model.get_part("handle_stage_0")
    stage_1 = object_model.get_part("handle_stage_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    stage_0_slide = object_model.get_articulation("body_to_handle_stage_0")
    stage_1_slide = object_model.get_articulation("handle_stage_0_to_stage_1")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")

    for wheel in (rear_wheel_0, rear_wheel_1):
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle_bar",
            elem_b="axle_bushing",
            reason="The wheel bushing is intentionally captured around the fixed steel axle.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            min_overlap=0.035,
            elem_a="rear_axle_bar",
            elem_b="axle_bushing",
            name=f"{wheel.name} bushing remains captured on axle",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xz",
            min_overlap=0.015,
            elem_a="rear_axle_bar",
            elem_b="axle_bushing",
            name=f"{wheel.name} bushing is coaxial with axle",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lid_panel",
        negative_elem="rim_front",
        name="closed lid sits on the body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.25,
        elem_a="lid_panel",
        elem_b="bottom_pan",
        name="lid covers the toolbox body footprint",
    )
    ctx.expect_overlap(
        stage_0,
        body,
        axes="z",
        min_overlap=0.25,
        elem_a="stage0_0_web",
        elem_b="guide_0_back",
        name="lower handle stage is retained in rear guide tube",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="z",
        min_overlap=0.28,
        elem_a="stage1_rod_0",
        elem_b="stage0_0_web",
        name="upper handle stage is retained in lower channel",
    )

    rest_stage_0 = ctx.part_world_position(stage_0)
    rest_stage_1 = ctx.part_world_position(stage_1)
    rest_lid_box = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({stage_0_slide: 0.230, stage_1_slide: 0.230, lid_hinge: 1.25}):
        ctx.expect_overlap(
            stage_0,
            body,
            axes="z",
            min_overlap=0.115,
            elem_a="stage0_0_web",
            elem_b="guide_0_back",
            name="extended lower stage remains inserted in guide tube",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="z",
            min_overlap=0.115,
            elem_a="stage1_rod_0",
            elem_b="stage0_0_web",
            name="extended upper stage remains inserted in lower channel",
        )
        raised_stage_0 = ctx.part_world_position(stage_0)
        raised_stage_1 = ctx.part_world_position(stage_1)
        raised_lid_box = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "handle stages slide upward",
        rest_stage_0 is not None
        and rest_stage_1 is not None
        and raised_stage_0 is not None
        and raised_stage_1 is not None
        and raised_stage_0[2] > rest_stage_0[2] + 0.20
        and raised_stage_1[2] > rest_stage_1[2] + 0.40,
        details=f"rest=({rest_stage_0}, {rest_stage_1}), raised=({raised_stage_0}, {raised_stage_1})",
    )
    ctx.check(
        "lid hinge lifts the front edge upward",
        rest_lid_box is not None
        and raised_lid_box is not None
        and raised_lid_box[1][2] > rest_lid_box[1][2] + 0.35,
        details=f"rest_lid_aabb={rest_lid_box}, raised_lid_aabb={raised_lid_box}",
    )

    for suffix in ("0", "1"):
        wheel_joint = object_model.get_articulation(f"body_to_rear_wheel_{suffix}")
        ctx.check(
            f"rear wheel {suffix} uses continuous axle rotation",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
