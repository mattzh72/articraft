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
    model = ArticulatedObject(name="orchestra_music_stand")

    black = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    satin = model.material("satin_black_metal", rgba=(0.035, 0.034, 0.032, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    hinge_metal = model.material("worn_hinge_black", rgba=(0.02, 0.021, 0.020, 1.0))

    base = model.part("base")

    # Tripod hub and floor contact pads.
    base.visual(
        Cylinder(radius=0.050, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=satin,
        name="tripod_hub",
    )
    base.visual(
        Box((0.080, 0.080, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=satin,
        name="hub_collar",
    )

    leg_inner_radius = 0.040
    leg_outer_radius = 0.440
    leg_start_z = 0.102
    leg_end_z = 0.026
    for index, yaw in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        start = (
            leg_inner_radius * math.cos(yaw),
            leg_inner_radius * math.sin(yaw),
            leg_start_z,
        )
        end = (
            leg_outer_radius * math.cos(yaw),
            leg_outer_radius * math.sin(yaw),
            leg_end_z,
        )
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        pitch = -math.asin(dz / length)
        base.visual(
            Box((length, 0.025, 0.020)),
            origin=Origin(
                xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
                rpy=(0.0, pitch, yaw),
            ),
            material=satin,
            name=f"tripod_leg_{index}",
        )
        base.visual(
            Box((0.130, 0.045, 0.020)),
            origin=Origin(xyz=end, rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name=f"foot_pad_{index}",
        )

    # Four-wall square lower sleeve: a real clear central opening for the sliding stem.
    sleeve_height = 0.720
    sleeve_center_z = 0.465
    sleeve_outer = 0.054
    sleeve_inner = 0.040
    wall = (sleeve_outer - sleeve_inner) * 0.5
    for side, x in (("x_pos", sleeve_outer * 0.5 - wall * 0.5), ("x_neg", -sleeve_outer * 0.5 + wall * 0.5)):
        base.visual(
            Box((wall, sleeve_outer, sleeve_height)),
            origin=Origin(xyz=(x, 0.0, sleeve_center_z)),
            material=satin,
            name=f"sleeve_wall_{side}",
        )
    for side, y in (("y_pos", sleeve_outer * 0.5 - wall * 0.5), ("y_neg", -sleeve_outer * 0.5 + wall * 0.5)):
        base.visual(
            Box((sleeve_inner + 2.0 * wall, wall, sleeve_height)),
            origin=Origin(xyz=(0.0, y, sleeve_center_z)),
            material=satin,
            name=f"sleeve_wall_{side}",
        )
    base.visual(
        Box((0.074, 0.074, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=satin,
        name="sleeve_lower_collar",
    )
    collar_outer = 0.074
    collar_inner = 0.040
    collar_wall = (collar_outer - collar_inner) * 0.5
    for side, x in (("x_pos", collar_outer * 0.5 - collar_wall * 0.5), ("x_neg", -collar_outer * 0.5 + collar_wall * 0.5)):
        base.visual(
            Box((collar_wall, collar_outer, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.830)),
            material=satin,
            name=f"top_collar_wall_{side}",
        )
    for side, y in (("y_pos", collar_outer * 0.5 - collar_wall * 0.5), ("y_neg", -collar_outer * 0.5 + collar_wall * 0.5)):
        base.visual(
            Box((collar_inner + 2.0 * collar_wall, collar_wall, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.830)),
            material=satin,
            name=f"top_collar_wall_{side}",
        )

    stem = model.part("stem")
    stem.visual(
        Box((0.032, 0.032, 0.860)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=black,
        name="stem_tube",
    )
    for z in (-0.300, -0.060):
        for side, x in (("pos", 0.018), ("neg", -0.018)):
            stem.visual(
                Box((0.004, 0.020, 0.060)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=rubber,
                name=f"guide_pad_x_{side}_{int((z + 0.5) * 1000)}",
            )
        for side, y in (("pos", 0.018), ("neg", -0.018)):
            stem.visual(
                Box((0.020, 0.004, 0.060)),
                origin=Origin(xyz=(0.0, y, z)),
                material=rubber,
                name=f"guide_pad_y_{side}_{int((z + 0.5) * 1000)}",
            )
    stem.visual(
        Box((0.058, 0.058, 0.052)),
        origin=Origin(xyz=(0.0, 0.048, 0.446)),
        material=satin,
        name="head_block",
    )
    stem.visual(
        Box((0.034, 0.012, 0.046)),
        origin=Origin(xyz=(0.0, 0.014, 0.446)),
        material=satin,
        name="head_neck",
    )
    stem.visual(
        Box((0.155, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, 0.078, 0.460)),
        material=satin,
        name="tilt_crossbar",
    )
    for side, x in (("pos", 0.075), ("neg", -0.075)):
        stem.visual(
            Box((0.018, 0.078, 0.088)),
            origin=Origin(xyz=(x, 0.034, 0.460)),
            material=satin,
            name=f"tilt_cheek_{side}",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.590, 0.012, 0.380)),
        origin=Origin(xyz=(0.0, -0.030, 0.030)),
        material=black,
        name="tray_panel",
    )
    tray.visual(
        Box((0.610, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, -0.044, 0.226)),
        material=satin,
        name="top_fold",
    )
    for side, x in (("pos", 0.304), ("neg", -0.304)):
        tray.visual(
            Box((0.014, 0.034, 0.390)),
            origin=Origin(xyz=(x, -0.044, 0.030)),
            material=satin,
            name=f"side_fold_{side}",
        )
    for index, z in enumerate((-0.070, 0.040, 0.150)):
        tray.visual(
            Box((0.520, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, -0.038, z)),
            material=satin,
            name=f"pressed_rib_{index}",
        )
    for side, x in (("pos", 0.049), ("neg", -0.049)):
        tray.visual(
            Box((0.036, 0.022, 0.088)),
            origin=Origin(xyz=(x, -0.014, 0.000)),
            material=satin,
            name=f"pivot_ear_{side}",
        )

    lip = model.part("lip")
    lip.visual(
        Cylinder(radius=0.0060, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="lip_hinge_barrel",
    )
    lip.visual(
        Box((0.560, 0.078, 0.010)),
        origin=Origin(xyz=(0.0, -0.041, -0.006)),
        material=black,
        name="lip_shelf",
    )
    lip.visual(
        Cylinder(radius=0.0075, length=0.560),
        origin=Origin(xyz=(0.0, -0.086, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="front_rolled_edge",
    )

    model.articulation(
        "sleeve_to_stem",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.280),
    )
    model.articulation(
        "stem_to_tray",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.460), rpy=(-0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "tray_to_lip",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lip,
        origin=Origin(xyz=(0.0, -0.0210, -0.1645)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stem = object_model.get_part("stem")
    tray = object_model.get_part("tray")
    lip = object_model.get_part("lip")
    sleeve_to_stem = object_model.get_articulation("sleeve_to_stem")
    stem_to_tray = object_model.get_articulation("stem_to_tray")
    tray_to_lip = object_model.get_articulation("tray_to_lip")

    # The visible pivot trunnions and the front lip hinge are intentionally seated
    # with a tiny local bite so the joint reads as captured rather than floating.
    for side in ("pos", "neg"):
        ctx.allow_overlap(
            stem,
            tray,
            elem_a=f"tilt_cheek_{side}",
            elem_b=f"pivot_ear_{side}",
            reason="The tray pivot ear is seated in the stem-head cheek as a captured trunnion bearing.",
        )
        ctx.expect_contact(
            stem,
            tray,
            elem_a=f"tilt_cheek_{side}",
            elem_b=f"pivot_ear_{side}",
            name=f"tray pivot ear {side} is captured in the stem head",
        )
    ctx.allow_overlap(
        lip,
        tray,
        elem_a="lip_hinge_barrel",
        elem_b="tray_panel",
        reason="The long lower lip hinge barrel is seated into the folded front edge of the tray.",
    )
    ctx.expect_contact(
        lip,
        tray,
        elem_a="lip_hinge_barrel",
        elem_b="tray_panel",
        name="retaining lip hinge is seated on tray front edge",
    )

    # Square telescoping stem remains centered in the lower square sleeve and
    # retains insertion even at maximum height.
    ctx.expect_origin_distance(
        stem,
        base,
        axes="xy",
        max_dist=0.002,
        name="square stem stays centered in sleeve",
    )
    ctx.expect_gap(
        base,
        stem,
        axis="x",
        positive_elem="sleeve_wall_x_pos",
        negative_elem="stem_tube",
        min_gap=0.003,
        max_gap=0.006,
        name="square sleeve has real side clearance",
    )
    ctx.expect_overlap(
        stem,
        base,
        axes="z",
        elem_a="stem_tube",
        elem_b="sleeve_wall_x_pos",
        min_overlap=0.35,
        name="collapsed stem remains deeply inserted in sleeve",
    )

    def _center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_stem_center = _center(stem, "stem_tube")
    with ctx.pose({sleeve_to_stem: 0.280}):
        ctx.expect_origin_distance(
            stem,
            base,
            axes="xy",
            max_dist=0.002,
            name="extended stem remains centered in sleeve",
        )
        ctx.expect_overlap(
            stem,
            base,
            axes="z",
            elem_a="stem_tube",
            elem_b="sleeve_wall_x_pos",
            min_overlap=0.12,
            name="extended stem retains insertion in lower sleeve",
        )
        extended_stem_center = _center(stem, "stem_tube")
    ctx.check(
        "prismatic stem raises the tray head",
        rest_stem_center is not None
        and extended_stem_center is not None
        and extended_stem_center[2] > rest_stem_center[2] + 0.25,
        details=f"rest={rest_stem_center}, extended={extended_stem_center}",
    )

    # Horizontal tilt axis at the stem head: rotating the tray changes the desk
    # panel attitude while the captured side ears remain mounted.
    rest_panel_center = _center(tray, "tray_panel")
    with ctx.pose({stem_to_tray: 0.35}):
        tilted_panel_center = _center(tray, "tray_panel")
    ctx.check(
        "tray rotates about horizontal stem-head axis",
        rest_panel_center is not None
        and tilted_panel_center is not None
        and tilted_panel_center[1] < rest_panel_center[1] - 0.005
        and tilted_panel_center[2] < rest_panel_center[2] - 0.005,
        details=f"rest={rest_panel_center}, tilted={tilted_panel_center}",
    )

    # Long lower lip spans almost the full tray width and folds upward on its
    # own hinge rather than being fused to the tray.
    ctx.expect_overlap(
        lip,
        tray,
        axes="x",
        elem_a="lip_shelf",
        elem_b="tray_panel",
        min_overlap=0.54,
        name="retaining lip spans the rectangular tray width",
    )
    rest_lip_edge = _center(lip, "front_rolled_edge")
    with ctx.pose({tray_to_lip: 1.0}):
        folded_lip_edge = _center(lip, "front_rolled_edge")
    ctx.check(
        "retaining lip folds upward on long front hinge",
        rest_lip_edge is not None
        and folded_lip_edge is not None
        and folded_lip_edge[2] > rest_lip_edge[2] + 0.05,
        details=f"rest={rest_lip_edge}, folded={folded_lip_edge}",
    )

    return ctx.report()


object_model = build_object_model()
