from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    hw = width / 2.0
    hd = depth / 2.0
    return [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _inclined_cylinder_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Origin and length for a cylinder whose local +Z follows a YZ-plane segment."""
    sx, sy, sz = start
    ex, ey, ez = end
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dy * dy + dz * dz)
    roll = -math.atan2(dy, dz)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(roll, 0.0, 0.0)), length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_monocular_microscope")

    enamel = model.material("warm_gray_enamel", rgba=(0.78, 0.79, 0.74, 1.0))
    dark_enamel = model.material("black_enamel", rgba=(0.035, 0.038, 0.040, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.74, 0.73, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.58, 0.78, 0.92, 0.45))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    lens_black = model.material("lens_black", rgba=(0.0, 0.0, 0.0, 1.0))

    # A single concave extrusion gives the stand a real U/horseshoe footprint:
    # two forward feet joined by a rear bridge, with the open notch toward -Y.
    horseshoe_profile = [
        (-0.160, -0.180),
        (-0.078, -0.180),
        (-0.078, 0.040),
        (0.078, 0.040),
        (0.078, -0.180),
        (0.160, -0.180),
        (0.160, 0.180),
        (-0.160, 0.180),
    ]
    horseshoe_base = mesh_from_geometry(
        ExtrudeGeometry.from_z0(horseshoe_profile, 0.036),
        "horseshoe_base",
    )

    curved_arm = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.0, 0.118, 0.026),
                (0.0, 0.150, 0.120),
                (0.0, 0.130, 0.240),
                (0.0, 0.075, 0.340),
                (0.0, 0.035, 0.382),
            ],
            profile=rounded_rect_profile(0.052, 0.034, radius=0.007, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
        ),
        "curved_arm",
    )

    stage_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.140, 0.140),
            [_circle_profile(0.018, 60)],
            0.012,
            center=True,
        ),
        "stage_plate_with_aperture",
    )

    stand = model.part("stand")
    stand.visual(horseshoe_base, material=enamel, name="horseshoe_base")
    stand.visual(curved_arm, material=enamel, name="curved_arm")
    stand.visual(
        Box((0.105, 0.100, 0.045)),
        origin=Origin(xyz=(0.0, 0.125, 0.058)),
        material=enamel,
        name="rear_pedestal",
    )
    stand.visual(
        Box((0.085, 0.055, 0.125)),
        origin=Origin(xyz=(0.0, 0.082, 0.102)),
        material=enamel,
        name="stage_support_neck",
    )
    stand.visual(
        Box((0.190, 0.158, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=enamel,
        name="carrier_support_plate",
    )
    stand.visual(
        Box((0.026, 0.145, 0.012)),
        origin=Origin(xyz=(-0.054, 0.0, 0.164)),
        material=dark_metal,
        name="fore_rail_0",
    )
    stand.visual(
        Box((0.026, 0.145, 0.012)),
        origin=Origin(xyz=(0.054, 0.0, 0.164)),
        material=dark_metal,
        name="fore_rail_1",
    )
    stand.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material=chrome,
        name="substage_condenser",
    )
    stand.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=glass,
        name="condenser_lens",
    )
    stand.visual(
        Box((0.096, 0.100, 0.075)),
        origin=Origin(xyz=(0.0, 0.030, 0.360)),
        material=enamel,
        name="inclined_head",
    )
    stand.visual(
        Box((0.070, 0.064, 0.028)),
        origin=Origin(xyz=(0.0, 0.014, 0.322)),
        material=enamel,
        name="nosepiece_bearing_boss",
    )

    tube_origin, tube_length = _inclined_cylinder_origin(
        (0.0, 0.046, 0.382),
        (0.0, 0.170, 0.505),
    )
    stand.visual(
        Cylinder(radius=0.019, length=tube_length),
        origin=tube_origin,
        material=dark_enamel,
        name="inclined_monocular_tube",
    )
    ocular_origin, ocular_length = _inclined_cylinder_origin(
        (0.0, 0.158, 0.493),
        (0.0, 0.215, 0.550),
    )
    stand.visual(
        Cylinder(radius=0.021, length=ocular_length),
        origin=ocular_origin,
        material=dark_enamel,
        name="eyepiece_barrel",
    )
    stand.visual(
        Cylinder(radius=0.023, length=0.010),
        origin=Origin(xyz=(0.0, 0.2185, 0.5535), rpy=ocular_origin.rpy),
        material=rubber,
        name="eyecup_rim",
    )
    for index, (x, y) in enumerate(((-0.112, -0.128), (0.112, -0.128), (0.0, 0.142))):
        stand.visual(
            Cylinder(radius=0.025, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )

    stage_assembly = model.part("stage_assembly")
    stage_assembly.visual(
        Box((0.170, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_enamel,
        name="fore_saddle",
    )
    stage_assembly.visual(
        Box((0.150, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, -0.067, 0.010)),
        material=dark_metal,
        name="fore_slide_lip",
    )
    stage_assembly.visual(
        Box((0.145, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.048, 0.019)),
        material=chrome,
        name="cross_rail_0",
    )
    stage_assembly.visual(
        Box((0.145, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.048, 0.019)),
        material=chrome,
        name="cross_rail_1",
    )

    stage_carrier = model.part("stage_carrier")
    stage_carrier.visual(
        Box((0.145, 0.126, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_metal,
        name="cross_saddle",
    )
    stage_carrier.visual(
        stage_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_enamel,
        name="stage_plate",
    )
    stage_carrier.visual(
        Box((0.080, 0.028, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.02275)),
        material=glass,
        name="glass_slide",
    )
    for index, y in enumerate((-0.047, 0.047)):
        stage_carrier.visual(
            Box((0.092, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.024)),
            material=chrome,
            name=f"slide_clip_{index}",
        )
        stage_carrier.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(-0.049, y, 0.024)),
            material=chrome,
            name=f"clip_pivot_{index}",
        )

    nosepiece = model.part("nosepiece")
    nosepiece.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=chrome,
        name="turret_disk",
    )
    nosepiece.visual(
        Cylinder(radius=0.014, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=chrome,
        name="turret_spindle",
    )
    objective_specs = (
        (0.0, 0.0085, 0.052),
        (2.0 * math.pi / 3.0, 0.0075, 0.046),
        (4.0 * math.pi / 3.0, 0.0065, 0.040),
    )
    angle, radius, length = objective_specs[0]
    x = 0.023 * math.cos(angle)
    y = 0.023 * math.sin(angle)
    nosepiece.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, -0.007 - length / 2.0)),
        material=chrome,
        name="objective_barrel_0",
    )
    nosepiece.visual(
        Cylinder(radius=radius * 0.82, length=0.009),
        origin=Origin(xyz=(x, y, -0.0115 - length)),
        material=lens_black,
        name="objective_tip_0",
    )
    angle, radius, length = objective_specs[1]
    x = 0.023 * math.cos(angle)
    y = 0.023 * math.sin(angle)
    nosepiece.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, -0.007 - length / 2.0)),
        material=chrome,
        name="objective_barrel_1",
    )
    nosepiece.visual(
        Cylinder(radius=radius * 0.82, length=0.009),
        origin=Origin(xyz=(x, y, -0.0115 - length)),
        material=lens_black,
        name="objective_tip_1",
    )
    angle, radius, length = objective_specs[2]
    x = 0.023 * math.cos(angle)
    y = 0.023 * math.sin(angle)
    nosepiece.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, -0.007 - length / 2.0)),
        material=chrome,
        name="objective_barrel_2",
    )
    nosepiece.visual(
        Cylinder(radius=radius * 0.82, length=0.009),
        origin=Origin(xyz=(x, y, -0.0115 - length)),
        material=lens_black,
        name="objective_tip_2",
    )

    model.articulation(
        "fore_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=stage_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.055, lower=-0.030, upper=0.030),
    )
    model.articulation(
        "cross_slide",
        ArticulationType.PRISMATIC,
        parent=stage_assembly,
        child=stage_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.060, lower=-0.035, upper=0.035),
    )
    model.articulation(
        "nosepiece_turret",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=nosepiece,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=0.0, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    stage_assembly = object_model.get_part("stage_assembly")
    stage_carrier = object_model.get_part("stage_carrier")
    nosepiece = object_model.get_part("nosepiece")
    fore_slide = object_model.get_articulation("fore_slide")
    cross_slide = object_model.get_articulation("cross_slide")
    turret = object_model.get_articulation("nosepiece_turret")

    ctx.allow_overlap(
        nosepiece,
        stand,
        elem_a="turret_spindle",
        elem_b="nosepiece_bearing_boss",
        reason="The turret spindle is intentionally captured inside the head bearing boss.",
    )
    ctx.allow_overlap(
        nosepiece,
        stand,
        elem_a="turret_spindle",
        elem_b="inclined_head",
        reason="The upper turret spindle is seated in the simplified solid microscope head casting.",
    )
    ctx.expect_within(
        nosepiece,
        stand,
        axes="xy",
        inner_elem="turret_spindle",
        outer_elem="nosepiece_bearing_boss",
        margin=0.001,
        name="turret spindle is centered in bearing boss",
    )
    ctx.expect_overlap(
        nosepiece,
        stand,
        axes="z",
        elem_a="turret_spindle",
        elem_b="nosepiece_bearing_boss",
        min_overlap=0.010,
        name="turret spindle remains captured vertically",
    )
    ctx.expect_within(
        nosepiece,
        stand,
        axes="xy",
        inner_elem="turret_spindle",
        outer_elem="inclined_head",
        margin=0.001,
        name="upper spindle sits inside the head casting",
    )
    ctx.expect_overlap(
        nosepiece,
        stand,
        axes="z",
        elem_a="turret_spindle",
        elem_b="inclined_head",
        min_overlap=0.010,
        name="upper spindle is seated into the head casting",
    )

    ctx.check(
        "nosepiece uses a vertical revolute turret",
        turret.articulation_type == ArticulationType.REVOLUTE and tuple(turret.axis) == (0.0, 0.0, 1.0),
        details=f"type={turret.articulation_type}, axis={turret.axis}",
    )
    ctx.check(
        "stage has two orthogonal prismatic slides",
        fore_slide.articulation_type == ArticulationType.PRISMATIC
        and cross_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(fore_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(cross_slide.axis) == (1.0, 0.0, 0.0),
        details=f"fore={fore_slide.axis}, cross={cross_slide.axis}",
    )

    ctx.expect_gap(
        stage_assembly,
        stand,
        axis="z",
        positive_elem="fore_saddle",
        negative_elem="fore_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="fore saddle rides on fixed support rail",
    )
    ctx.expect_overlap(
        stage_assembly,
        stand,
        axes="xy",
        elem_a="fore_saddle",
        elem_b="fore_rail_0",
        min_overlap=0.020,
        name="fore saddle retained on support rail",
    )
    ctx.expect_gap(
        stage_carrier,
        stage_assembly,
        axis="z",
        positive_elem="cross_saddle",
        negative_elem="cross_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="cross saddle rides on lateral rail",
    )
    ctx.expect_overlap(
        stage_carrier,
        stage_assembly,
        axes="xy",
        elem_a="cross_saddle",
        elem_b="cross_rail_0",
        min_overlap=0.010,
        name="cross saddle retained on lateral rail",
    )
    ctx.expect_gap(
        nosepiece,
        stage_carrier,
        axis="z",
        positive_elem="objective_tip_0",
        negative_elem="stage_plate",
        min_gap=0.020,
        max_gap=0.060,
        name="objective clears the mechanical stage",
    )

    rest_fore = ctx.part_world_position(stage_assembly)
    with ctx.pose({fore_slide: 0.030}):
        forward_fore = ctx.part_world_position(stage_assembly)
        ctx.expect_overlap(
            stage_assembly,
            stand,
            axes="y",
            elem_a="fore_saddle",
            elem_b="fore_rail_0",
            min_overlap=0.080,
            name="fore slide remains engaged at travel end",
        )
    ctx.check(
        "full stage assembly moves fore and aft",
        rest_fore is not None and forward_fore is not None and forward_fore[1] > rest_fore[1] + 0.025,
        details=f"rest={rest_fore}, forward={forward_fore}",
    )

    rest_cross = ctx.part_world_position(stage_carrier)
    with ctx.pose({cross_slide: 0.035}):
        right_cross = ctx.part_world_position(stage_carrier)
        ctx.expect_overlap(
            stage_carrier,
            stage_assembly,
            axes="x",
            elem_a="cross_saddle",
            elem_b="cross_rail_0",
            min_overlap=0.090,
            name="left-right cross slide remains engaged",
        )
    ctx.check(
        "stage carrier moves left to right",
        rest_cross is not None and right_cross is not None and right_cross[0] > rest_cross[0] + 0.030,
        details=f"rest={rest_cross}, right={right_cross}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    barrel_rest = _aabb_center(ctx.part_element_world_aabb(nosepiece, elem="objective_barrel_0"))
    with ctx.pose({turret: math.pi / 2.0}):
        barrel_quarter = _aabb_center(ctx.part_element_world_aabb(nosepiece, elem="objective_barrel_0"))
    ctx.check(
        "turret rotation carries objective around optical axis",
        barrel_rest is not None
        and barrel_quarter is not None
        and abs(barrel_quarter[1] - barrel_rest[1]) > 0.018
        and abs(barrel_quarter[2] - barrel_rest[2]) < 0.002,
        details=f"rest={barrel_rest}, quarter_turn={barrel_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
