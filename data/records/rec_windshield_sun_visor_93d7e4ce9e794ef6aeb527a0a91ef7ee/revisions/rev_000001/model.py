from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_sun_visor")

    headliner_fabric = model.material("headliner_fabric", rgba=(0.72, 0.66, 0.56, 1.0))
    visor_fabric = model.material("visor_padded_fabric", rgba=(0.68, 0.62, 0.52, 1.0))
    seam_fabric = model.material("raised_stitching", rgba=(0.55, 0.50, 0.43, 1.0))
    tan_plastic = model.material("molded_tan_plastic", rgba=(0.58, 0.52, 0.44, 1.0))
    dark_shadow = model.material("recess_shadow", rgba=(0.10, 0.095, 0.085, 1.0))
    label_yellow = model.material("airbag_label", rgba=(0.94, 0.80, 0.34, 1.0))
    label_ink = model.material("label_ink", rgba=(0.05, 0.05, 0.04, 1.0))
    mirror_glass = model.material("muted_mirror", rgba=(0.58, 0.66, 0.70, 0.72))
    windshield_tint = model.material("windshield_tint", rgba=(0.30, 0.42, 0.50, 0.35))
    satin_metal = model.material("satin_metal", rgba=(0.50, 0.48, 0.44, 1.0))

    roof = model.part("roof_header")
    roof.visual(
        Box((0.90, 0.34, 0.030)),
        origin=Origin(xyz=(0.02, 0.035, 0.015)),
        material=headliner_fabric,
        name="headliner_panel",
    )
    roof.visual(
        Box((0.90, 0.018, 0.018)),
        origin=Origin(xyz=(0.02, -0.142, -0.004)),
        material=dark_shadow,
        name="windshield_seal",
    )
    roof.visual(
        Box((0.88, 0.010, 0.155)),
        origin=Origin(xyz=(0.02, -0.142, -0.055)),
        material=windshield_tint,
        name="windshield_strip",
    )
    roof.visual(
        Box((0.075, 0.055, 0.018)),
        origin=Origin(xyz=(-0.34, -0.080, -0.009)),
        material=tan_plastic,
        name="pivot_cover",
    )
    roof.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(-0.34, -0.080, -0.015)),
        material=tan_plastic,
        name="pivot_socket",
    )
    roof.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(-0.362, -0.080, -0.019)),
        material=satin_metal,
        name="pivot_screw_0",
    )
    roof.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(-0.318, -0.080, -0.019)),
        material=satin_metal,
        name="pivot_screw_1",
    )
    roof.visual(
        Box((0.070, 0.038, 0.012)),
        origin=Origin(xyz=(0.310, -0.080, -0.006)),
        material=tan_plastic,
        name="clip_base",
    )
    roof.visual(
        Box((0.010, 0.032, 0.038)),
        origin=Origin(xyz=(0.270, -0.104, -0.030)),
        material=tan_plastic,
        name="clip_jaw_0",
    )
    roof.visual(
        Box((0.010, 0.032, 0.038)),
        origin=Origin(xyz=(0.343, -0.104, -0.030)),
        material=tan_plastic,
        name="clip_jaw_1",
    )
    roof.visual(
        Box((0.058, 0.012, 0.006)),
        origin=Origin(xyz=(0.310, -0.080, -0.014)),
        material=tan_plastic,
        name="clip_lip",
    )
    roof.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.288, -0.080, -0.012)),
        material=satin_metal,
        name="clip_screw_0",
    )
    roof.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.332, -0.080, -0.012)),
        material=satin_metal,
        name="clip_screw_1",
    )
    roof.visual(
        Box((0.62, 0.006, 0.004)),
        origin=Origin(xyz=(-0.010, 0.155, -0.002)),
        material=seam_fabric,
        name="header_seam",
    )

    swivel = model.part("swivel_arm")
    swivel.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=tan_plastic,
        name="pivot_collar",
    )
    swivel.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=satin_metal,
        name="vertical_pin",
    )
    swivel.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=tan_plastic,
        name="elbow_boss",
    )
    swivel.visual(
        Cylinder(radius=0.007, length=0.590),
        origin=Origin(xyz=(0.295, 0.0, -0.030), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_rod",
    )
    swivel.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, -0.030), rpy=(0.0, pi / 2.0, 0.0)),
        material=tan_plastic,
        name="rod_knuckle",
    )

    visor = model.part("visor_panel")
    cushion = ExtrudeGeometry(
        rounded_rect_profile(0.675, 0.210, 0.045, corner_segments=10),
        0.026,
        center=True,
    ).translate(0.340, 0.130, -0.009)
    visor.visual(_mesh(cushion, "visor_cushion"), material=visor_fabric, name="visor_cushion")
    visor.visual(
        Cylinder(radius=0.018, length=0.610),
        origin=Origin(xyz=(0.340, 0.008, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=visor_fabric,
        name="hinge_sleeve",
    )
    visor.visual(
        Box((0.038, 0.065, 0.017)),
        origin=Origin(xyz=(0.640, 0.005, -0.004)),
        material=tan_plastic,
        name="retainer_boss",
    )
    visor.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(0.652, -0.022, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="retainer_pin",
    )
    piping = tube_from_spline_points(
        [
            (0.075, 0.050, -0.022),
            (0.185, 0.032, -0.022),
            (0.485, 0.032, -0.022),
            (0.610, 0.052, -0.022),
            (0.630, 0.130, -0.022),
            (0.600, 0.220, -0.022),
            (0.330, 0.238, -0.022),
            (0.090, 0.215, -0.022),
            (0.060, 0.130, -0.022),
        ],
        radius=0.0022,
        closed_spline=True,
        samples_per_segment=10,
        radial_segments=10,
    )
    visor.visual(_mesh(piping, "stitched_piping"), material=seam_fabric, name="stitched_piping")
    visor.visual(
        Box((0.145, 0.060, 0.002)),
        origin=Origin(xyz=(0.230, 0.126, -0.0218)),
        material=tan_plastic,
        name="mirror_cover",
    )
    visor.visual(
        Box((0.115, 0.040, 0.0015)),
        origin=Origin(xyz=(0.230, 0.126, -0.0230)),
        material=mirror_glass,
        name="mirror_glass",
    )
    visor.visual(
        Box((0.128, 0.046, 0.002)),
        origin=Origin(xyz=(0.505, 0.122, -0.0218)),
        material=label_yellow,
        name="warning_label",
    )
    visor.visual(
        Box((0.090, 0.004, 0.001)),
        origin=Origin(xyz=(0.504, 0.110, -0.0231)),
        material=label_ink,
        name="label_line_0",
    )
    visor.visual(
        Box((0.095, 0.004, 0.001)),
        origin=Origin(xyz=(0.506, 0.124, -0.0231)),
        material=label_ink,
        name="label_line_1",
    )
    visor.visual(
        Box((0.070, 0.004, 0.001)),
        origin=Origin(xyz=(0.493, 0.138, -0.0231)),
        material=label_ink,
        name="label_line_2",
    )
    visor.visual(
        Box((0.360, 0.010, 0.010)),
        origin=Origin(xyz=(0.360, 0.232, -0.014)),
        material=seam_fabric,
        name="pull_lip",
    )

    model.articulation(
        "side_rotation",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=swivel,
        origin=Origin(xyz=(-0.340, -0.080, -0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "visor_swing",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=visor,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.2, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_header")
    swivel = object_model.get_part("swivel_arm")
    visor = object_model.get_part("visor_panel")
    swing = object_model.get_articulation("visor_swing")
    side = object_model.get_articulation("side_rotation")

    ctx.allow_overlap(
        swivel,
        visor,
        elem_a="hinge_rod",
        elem_b="hinge_sleeve",
        reason="The metal hinge rod is intentionally captured inside the padded visor sleeve proxy.",
    )
    ctx.expect_within(
        swivel,
        visor,
        axes="yz",
        inner_elem="hinge_rod",
        outer_elem="hinge_sleeve",
        margin=0.004,
        name="hinge rod is retained inside sleeve",
    )
    ctx.expect_overlap(
        swivel,
        visor,
        axes="x",
        elem_a="hinge_rod",
        elem_b="hinge_sleeve",
        min_overlap=0.55,
        name="hinge rod spans visor sleeve",
    )

    ctx.expect_gap(
        roof,
        visor,
        axis="z",
        positive_elem="headliner_panel",
        negative_elem="visor_cushion",
        min_gap=0.030,
        max_gap=0.070,
        name="stowed visor sits below headliner",
    )
    ctx.expect_overlap(
        roof,
        visor,
        axes="xy",
        elem_a="headliner_panel",
        elem_b="visor_cushion",
        min_overlap=0.15,
        name="visor parks under roof header",
    )

    rest_lip = ctx.part_element_world_aabb(visor, elem="pull_lip")
    with ctx.pose({swing: 1.10}):
        lowered_lip = ctx.part_element_world_aabb(visor, elem="pull_lip")
    ctx.check(
        "visor swing lowers free edge",
        rest_lip is not None
        and lowered_lip is not None
        and lowered_lip[0][2] < rest_lip[0][2] - 0.12,
        details=f"rest={rest_lip}, lowered={lowered_lip}",
    )

    rest_cushion = ctx.part_element_world_aabb(visor, elem="visor_cushion")
    with ctx.pose({side: 1.15}):
        side_cushion = ctx.part_element_world_aabb(visor, elem="visor_cushion")
    if rest_cushion is not None and side_cushion is not None:
        rest_center_y = 0.5 * (rest_cushion[0][1] + rest_cushion[1][1])
        side_center_y = 0.5 * (side_cushion[0][1] + side_cushion[1][1])
        side_ok = side_center_y > rest_center_y + 0.16
    else:
        side_ok = False
        rest_center_y = None
        side_center_y = None
    ctx.check(
        "side rotation sweeps visor sideways",
        side_ok,
        details=f"rest_center_y={rest_center_y}, side_center_y={side_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
