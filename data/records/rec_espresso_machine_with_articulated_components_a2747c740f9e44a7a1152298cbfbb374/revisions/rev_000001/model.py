from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_group_espresso_machine")

    body_paint = model.material("body_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    warming_deck_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.33, 0.21),
            0.006,
            hole_diameter=0.010,
            pitch=(0.018, 0.018),
            frame=0.012,
            corner_radius=0.010,
            stagger=True,
        ),
        "warming_deck",
    )
    tray_grate_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.332, 0.202),
            0.004,
            slot_size=(0.034, 0.006),
            pitch=(0.044, 0.016),
            frame=0.012,
            corner_radius=0.010,
            slot_angle_deg=0.0,
            stagger=True,
        ),
        "tray_grate",
    )
    portafilter_handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -0.014, -0.006),
                (0.0, -0.040, -0.014),
                (0.0, -0.095, -0.020),
                (0.0, -0.152, -0.018),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        "portafilter_handle",
    )
    wand_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.0, -0.012, -0.012),
                (0.0, -0.020, -0.050),
                (0.0, -0.032, -0.120),
                (0.0, -0.045, -0.190),
            ],
            radius=0.0055,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "steam_wand",
    )

    body = model.part("body")
    body.visual(
        Box((0.46, 0.48, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=body_paint,
        name="roof",
    )
    body.visual(
        Box((0.04, 0.48, 0.18)),
        origin=Origin(xyz=(-0.210, 0.0, 0.180)),
        material=body_paint,
        name="left_wall",
    )
    body.visual(
        Box((0.04, 0.48, 0.18)),
        origin=Origin(xyz=(0.210, 0.0, 0.180)),
        material=body_paint,
        name="right_wall",
    )
    body.visual(
        Box((0.38, 0.05, 0.18)),
        origin=Origin(xyz=(0.0, 0.215, 0.180)),
        material=body_paint,
        name="rear_wall",
    )
    body.visual(
        Box((0.42, 0.06, 0.07)),
        origin=Origin(xyz=(0.0, -0.24, 0.24)),
        material=stainless,
        name="fascia",
    )
    body.visual(
        Box((0.036, 0.46, 0.09)),
        origin=Origin(xyz=(-0.212, 0.0, 0.045)),
        material=stainless,
        name="left_skirt",
    )
    body.visual(
        Box((0.036, 0.46, 0.09)),
        origin=Origin(xyz=(0.212, 0.0, 0.045)),
        material=stainless,
        name="right_skirt",
    )
    body.visual(
        Box((0.39, 0.09, 0.09)),
        origin=Origin(xyz=(0.0, 0.195, 0.045)),
        material=stainless,
        name="rear_base",
    )
    body.visual(
        Box((0.39, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -0.235, 0.075)),
        material=stainless,
        name="front_apron",
    )
    body.visual(
        Box((0.12, 0.08, 0.055)),
        origin=Origin(xyz=(0.0, -0.202, 0.183)),
        material=stainless,
        name="group_block",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.032),
        origin=Origin(xyz=(0.0, -0.183, 0.164)),
        material=brushed_steel,
        name="group_head",
    )
    body.visual(
        Box((0.028, 0.078, 0.050)),
        origin=Origin(xyz=(0.184, -0.239, 0.215)),
        material=brushed_steel,
        name="wand_mount",
    )
    body.visual(
        Box((0.05, 0.18, 0.085)),
        origin=Origin(xyz=(-0.180, 0.125, 0.3725)),
        material=stainless,
        name="left_riser",
    )
    body.visual(
        Box((0.05, 0.18, 0.085)),
        origin=Origin(xyz=(0.180, 0.125, 0.3725)),
        material=stainless,
        name="right_riser",
    )
    body.visual(
        Box((0.31, 0.03, 0.085)),
        origin=Origin(xyz=(0.0, 0.225, 0.3725)),
        material=stainless,
        name="rear_riser",
    )
    body.visual(
        Box((0.03, 0.24, 0.035)),
        origin=Origin(xyz=(-0.190, -0.085, 0.0475)),
        material=stainless,
        name="left_guide",
    )
    body.visual(
        Box((0.03, 0.24, 0.035)),
        origin=Origin(xyz=(0.190, -0.085, 0.0475)),
        material=stainless,
        name="right_guide",
    )
    body.visual(
        warming_deck_mesh,
        origin=Origin(xyz=(0.0, 0.125, 0.418)),
        material=brushed_steel,
        name="warming_deck",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.46, 0.52, 0.42)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=brushed_steel,
        name="flange",
    )
    portafilter.visual(
        Cylinder(radius=0.026, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=stainless,
        name="basket",
    )
    portafilter.visual(
        Box((0.028, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, -0.062)),
        material=stainless,
        name="spout_bridge",
    )
    for index, x_pos in enumerate((-0.010, 0.010)):
        portafilter.visual(
            Cylinder(radius=0.0048, length=0.018),
            origin=Origin(xyz=(x_pos, -0.008, -0.076)),
            material=stainless,
            name=f"spout_{index}",
        )
    portafilter.visual(
        portafilter_handle_mesh,
        material=black_plastic,
        name="handle",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.07, 0.18, 0.09)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.07, -0.030)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=brushed_steel,
        name="collar",
    )
    wand.visual(
        wand_mesh,
        material=stainless,
        name="wand",
    )
    wand.visual(
        Cylinder(radius=0.0042, length=0.016),
        origin=Origin(xyz=(0.0, -0.045, -0.198)),
        material=brushed_steel,
        name="tip",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.03, 0.07, 0.21)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.02, -0.10)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.26, 0.17, 0.006)),
        origin=Origin(xyz=(0.0, -0.085, 0.003)),
        material=stainless,
        name="service_panel",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.16),
        origin=Origin(
            xyz=(0.0, -0.150, 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black_plastic,
        name="pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.26, 0.17, 0.016)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.085, 0.008)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.35, 0.22, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=stainless,
        name="tray_bottom",
    )
    tray.visual(
        Box((0.008, 0.22, 0.030)),
        origin=Origin(xyz=(-0.171, 0.0, 0.015)),
        material=stainless,
        name="left_wall",
    )
    tray.visual(
        Box((0.008, 0.22, 0.030)),
        origin=Origin(xyz=(0.171, 0.0, 0.015)),
        material=stainless,
        name="right_wall",
    )
    tray.visual(
        Box((0.35, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, -0.106, 0.015)),
        material=stainless,
        name="front_wall",
    )
    tray.visual(
        Box((0.35, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.106, 0.015)),
        material=stainless,
        name="back_wall",
    )
    tray.visual(
        tray_grate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=brushed_steel,
        name="grate",
    )
    tray.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(
            xyz=(0.0, -0.105, 0.032),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="tray_pull",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.35, 0.22, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.0, -0.183, 0.147)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.85,
            upper=0.35,
        ),
    )
    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.184, -0.285, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=-0.40,
            upper=1.00,
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.205, 0.330)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=0.0,
            upper=0.48,
        ),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.105, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=0.11,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    wand = object_model.get_part("wand")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")

    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_wand")
    lid_joint = object_model.get_articulation("body_to_lid")
    tray_joint = object_model.get_articulation("body_to_tray")

    def _center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_head",
        negative_elem="flange",
        max_gap=0.003,
        max_penetration=0.0,
        name="portafilter seats just below group head",
    )
    ctx.expect_overlap(
        portafilter,
        body,
        axes="xy",
        elem_a="flange",
        elem_b="group_head",
        min_overlap=0.055,
        name="portafilter stays aligned with brew axis",
    )
    ctx.expect_within(
        tray,
        body,
        axes="x",
        margin=0.0,
        name="tray stays centered between side skirts",
    )

    tray_upper = tray_joint.motion_limits.upper or 0.0
    lid_upper = lid_joint.motion_limits.upper or 0.0
    wand_upper = wand_joint.motion_limits.upper or 0.0
    portafilter_lower = portafilter_joint.motion_limits.lower or 0.0
    portafilter_upper = portafilter_joint.motion_limits.upper or 0.0

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: tray_upper}):
        ext_tray_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            body,
            axes="x",
            margin=0.0,
            name="extended tray remains guided laterally",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.10,
            name="extended tray retains insertion depth",
        )
    ctx.check(
        "tray slides forward",
        rest_tray_pos is not None
        and ext_tray_pos is not None
        and ext_tray_pos[1] < rest_tray_pos[1] - 0.08,
        details=f"rest={rest_tray_pos}, extended={ext_tray_pos}",
    )

    rest_pull = _center(ctx.part_element_world_aabb(lid, elem="pull"))
    with ctx.pose({lid_joint: lid_upper}):
        open_pull = _center(ctx.part_element_world_aabb(lid, elem="pull"))
    ctx.check(
        "service lid lifts upward",
        rest_pull is not None
        and open_pull is not None
        and open_pull[2] > rest_pull[2] + 0.05,
        details=f"closed={rest_pull}, open={open_pull}",
    )

    rest_tip = _center(ctx.part_element_world_aabb(wand, elem="tip"))
    with ctx.pose({wand_joint: wand_upper}):
        swung_tip = _center(ctx.part_element_world_aabb(wand, elem="tip"))
    ctx.check(
        "steam wand swings outward",
        rest_tip is not None
        and swung_tip is not None
        and swung_tip[0] > rest_tip[0] + 0.03,
        details=f"rest={rest_tip}, swung={swung_tip}",
    )

    with ctx.pose({portafilter_joint: portafilter_lower}):
        left_handle = _center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    with ctx.pose({portafilter_joint: portafilter_upper}):
        right_handle = _center(ctx.part_element_world_aabb(portafilter, elem="handle"))
    ctx.check(
        "portafilter rotates around vertical brew axis",
        left_handle is not None
        and right_handle is not None
        and left_handle[0] < -0.05
        and right_handle[0] > 0.02,
        details=f"lower={left_handle}, upper={right_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
