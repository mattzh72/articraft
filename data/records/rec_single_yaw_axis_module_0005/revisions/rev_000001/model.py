from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_THICKNESS = 0.016
PEDESTAL_TOP_Z = 0.074
YAW_PLANE_Z = 0.092


def _circle_points(radius: float, count: int, start_deg: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.radians(start_deg + 360.0 * i / count)),
            radius * math.sin(math.radians(start_deg + 360.0 * i / count)),
        )
        for i in range(count)
    ]


def _base_structure() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.40, 0.30, BASE_THICKNESS).translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    for sx in (-0.175, 0.175):
        for sy in (-0.135, 0.135):
            plate = plate.union(
                cq.Workplane("XY")
                .box(0.09, 0.07, BASE_THICKNESS)
                .translate((sx, sy, BASE_THICKNESS / 2.0))
            )

    pedestal_wall = (
        cq.Workplane("XY")
        .circle(0.105)
        .circle(0.052)
        .extrude(PEDESTAL_TOP_Z - BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    outer_bearing_ring = (
        cq.Workplane("XY")
        .circle(0.158)
        .circle(0.118)
        .extrude(YAW_PLANE_Z - PEDESTAL_TOP_Z)
        .translate((0.0, 0.0, PEDESTAL_TOP_Z))
    )

    service_outer = cq.Workplane("XY").box(0.094, 0.084, 0.056).translate((0.120, 0.0, 0.044))
    service_inner = cq.Workplane("XY").box(0.090, 0.064, 0.046).translate((0.126, 0.0, 0.046))
    service_box = service_outer.cut(service_inner)

    cable_tube = (
        cq.Workplane("YZ")
        .circle(0.014)
        .circle(0.010)
        .extrude(0.070)
        .translate((0.040, 0.0, 0.044))
    )

    stop_bridge = cq.Workplane("XY").box(0.110, 0.024, 0.030).translate((0.0, 0.168, 0.031))
    stop_posts = (
        cq.Workplane("XY")
        .pushPoints([(-0.030, 0.168), (0.030, 0.168)])
        .circle(0.0065)
        .extrude(0.032)
        .translate((0.0, 0.0, 0.046))
    )
    stop_pad = cq.Workplane("XY").box(0.070, 0.040, 0.010).translate((0.0, 0.144, 0.021))

    gusset_ns = (
        cq.Workplane("XZ")
        .polyline([(0.060, BASE_THICKNESS), (0.135, BASE_THICKNESS), (0.106, PEDESTAL_TOP_Z), (0.074, PEDESTAL_TOP_Z)])
        .close()
        .extrude(0.014)
        .translate((0.0, -0.007, 0.0))
    )
    gusset_north = gusset_ns.translate((0.0, 0.058, 0.0))
    gusset_south = gusset_ns.translate((0.0, -0.072, 0.0))
    gusset_ew = gusset_ns.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0)
    gusset_east = gusset_ew.translate((0.058, 0.0, 0.0))
    gusset_west = gusset_ew.translate((-0.072, 0.0, 0.0))

    structure = plate
    for elem in (
        pedestal_wall,
        outer_bearing_ring,
        service_box,
        cable_tube,
        stop_bridge,
        stop_posts,
        stop_pad,
        gusset_north,
        gusset_south,
        gusset_east,
        gusset_west,
    ):
        structure = structure.union(elem)

    center_bore = cq.Workplane("XY").circle(0.048).extrude(0.140).translate((0.0, 0.0, -0.010))
    structure = structure.cut(center_bore)

    flange_bolts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.185, -0.140),
                (-0.185, 0.140),
                (0.185, -0.140),
                (0.185, 0.140),
                (-0.220, -0.140),
                (-0.220, 0.140),
                (0.220, -0.140),
                (0.220, 0.140),
            ]
        )
        .circle(0.006)
        .extrude(0.006)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    ring_bolts = (
        cq.Workplane("XY")
        .pushPoints(_circle_points(0.138, 12, start_deg=15.0))
        .circle(0.0055)
        .extrude(0.005)
        .translate((0.0, 0.0, YAW_PLANE_Z))
    )

    return structure.union(flange_bolts).union(ring_bolts)


def _support_flange() -> cq.Workplane:
    base_ring = (
        cq.Workplane("XY")
        .circle(0.116)
        .circle(0.068)
        .extrude(YAW_PLANE_Z - PEDESTAL_TOP_Z)
        .translate((0.0, 0.0, PEDESTAL_TOP_Z))
    )
    pilot_ring = (
        cq.Workplane("XY")
        .circle(0.082)
        .circle(0.060)
        .extrude(0.006)
        .translate((0.0, 0.0, YAW_PLANE_Z - 0.006))
    )
    clamping_bolts = (
        cq.Workplane("XY")
        .pushPoints(_circle_points(0.099, 10, start_deg=18.0))
        .circle(0.0045)
        .extrude(0.004)
        .translate((0.0, 0.0, YAW_PLANE_Z))
    )
    return base_ring.union(pilot_ring).union(clamping_bolts)


def _upper_structure() -> cq.Workplane:
    deck = cq.Workplane("XY").box(0.340, 0.260, 0.018).translate((0.0, 0.0, 0.049))
    center_pad = cq.Workplane("XY").box(0.126, 0.094, 0.012).translate((0.0, 0.0, 0.064))

    x_ribs = (
        cq.Workplane("XY")
        .pushPoints([(-0.075, 0.0), (0.075, 0.0)])
        .rect(0.090, 0.014)
        .extrude(0.024)
        .translate((0.0, 0.0, 0.016))
    )
    y_ribs = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -0.060), (0.0, 0.060)])
        .rect(0.014, 0.074)
        .extrude(0.024)
        .translate((0.0, 0.0, 0.016))
    )

    side_cheek_outer = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -0.106), (0.0, 0.106)])
        .rect(0.140, 0.016)
        .extrude(0.050)
        .translate((0.0, 0.0, 0.058))
    )
    side_cheek_holes = (
        cq.Workplane("YZ")
        .pushPoints([(-0.036, 0.082), (0.036, 0.082), (-0.036, -0.082), (0.036, -0.082)])
        .circle(0.010)
        .extrude(0.180)
        .translate((-0.090, 0.0, 0.0))
    )
    side_cheeks = side_cheek_outer.cut(side_cheek_holes)

    striker = cq.Workplane("XY").box(0.020, 0.034, 0.028).translate((0.0, 0.145, 0.026))
    striker_backer = cq.Workplane("XY").box(0.040, 0.050, 0.014).translate((0.0, 0.118, 0.023))

    hub_sleeve = (
        cq.Workplane("XY")
        .circle(0.045)
        .circle(0.028)
        .extrude(0.028)
        .translate((0.0, 0.0, -0.028))
    )

    upper = deck
    for elem in (center_pad, x_ribs, y_ribs, side_cheeks, striker, striker_backer, hub_sleeve):
        upper = upper.union(elem)
    return upper


def _rotary_support() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.116).circle(0.070).extrude(0.012)
    upper_shoulder = (
        cq.Workplane("XY")
        .circle(0.096)
        .circle(0.080)
        .extrude(0.009)
        .translate((0.0, 0.0, 0.012))
    )
    inner_retainer = (
        cq.Workplane("XY")
        .circle(0.056)
        .circle(0.042)
        .extrude(0.011)
        .translate((0.0, 0.0, 0.010))
    )
    bolt_heads = (
        cq.Workplane("XY")
        .pushPoints(_circle_points(0.093, 10, start_deg=18.0))
        .circle(0.0048)
        .extrude(0.005)
        .translate((0.0, 0.0, 0.021))
    )
    return ring.union(upper_shoulder).union(inner_retainer).union(bolt_heads)


def _access_cover() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.006, 0.074, 0.052)
    pocket = cq.Workplane("XY").box(0.0025, 0.050, 0.030).translate((0.0015, 0.0, 0.0))
    stiffener = cq.Workplane("XY").box(0.003, 0.024, 0.040).translate((-0.0005, 0.0, 0.0))
    plate = plate.cut(pocket).union(stiffener)
    screws = (
        cq.Workplane("YZ")
        .pushPoints([(-0.022, -0.016), (-0.022, 0.016), (0.022, -0.016), (0.022, 0.016)])
        .circle(0.004)
        .extrude(0.004)
        .translate((0.003, 0.0, 0.0))
    )
    return plate.union(screws)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_axis_module", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.42, 0.44, 0.47, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_base_structure(), "base_structure.obj", assets=ASSETS),
        name="base_structure",
        material=dark_steel,
    )
    base.visual(
        mesh_from_cadquery(_support_flange(), "support_flange.obj", assets=ASSETS),
        name="support_flange",
        material=machined_steel,
    )
    base.visual(
        Box((0.002, 0.074, 0.052)),
        origin=Origin(xyz=(0.166, 0.0, 0.045)),
        name="cover_seat",
        material=machined_steel,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 0.12)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    upper = model.part("upper_platform")
    upper.visual(
        mesh_from_cadquery(_upper_structure(), "upper_structure.obj", assets=ASSETS),
        name="upper_structure",
        material=dark_steel,
    )
    upper.visual(
        mesh_from_cadquery(_rotary_support(), "rotary_support.obj", assets=ASSETS),
        name="rotary_support",
        material=machined_steel,
    )
    upper.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 0.10)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    cover = model.part("access_cover")
    cover.visual(
        mesh_from_cadquery(_access_cover(), "access_cover.obj", assets=ASSETS),
        name="cover_plate",
        material=cover_gray,
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.01, 0.08, 0.06)),
        mass=0.8,
        origin=Origin(),
    )

    model.articulation(
        "base_to_upper_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, YAW_PLANE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_access_cover",
        ArticulationType.FIXED,
        parent=base,
        child=cover,
        origin=Origin(xyz=(0.170, 0.0, 0.045)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    upper = object_model.get_part("upper_platform")
    cover = object_model.get_part("access_cover")
    yaw = object_model.get_articulation("base_to_upper_yaw")

    base_structure = base.get_visual("base_structure")
    base_support = base.get_visual("support_flange")
    cover_seat = base.get_visual("cover_seat")
    upper_structure = upper.get_visual("upper_structure")
    upper_support = upper.get_visual("rotary_support")
    cover_plate = cover.get_visual("cover_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_distance(upper, base, axes="xy", max_dist=1e-6)
    ctx.expect_contact(upper, base, elem_a=upper_support, elem_b=base_support, contact_tol=1e-6)
    ctx.expect_gap(
        upper,
        base,
        axis="z",
        positive_elem=upper_support,
        negative_elem=base_support,
        min_gap=0.0,
        max_gap=0.0005,
    )
    ctx.expect_overlap(upper, base, axes="xy", min_overlap=0.18)
    ctx.expect_overlap(upper, base, axes="xy", elem_a=upper_structure, elem_b=base_structure, min_overlap=0.12)

    ctx.expect_contact(cover, base, elem_a=cover_plate, elem_b=cover_seat, contact_tol=1e-6)
    ctx.expect_gap(
        cover,
        base,
        axis="x",
        positive_elem=cover_plate,
        negative_elem=cover_seat,
        min_gap=0.0,
        max_gap=0.0005,
    )
    ctx.expect_overlap(cover, base, axes="yz", elem_a=cover_plate, elem_b=cover_seat, min_overlap=0.04)

    with ctx.pose({yaw: -1.0}):
        ctx.expect_origin_distance(upper, base, axes="xy", max_dist=1e-6)
        ctx.expect_contact(upper, base, elem_a=upper_support, elem_b=base_support, contact_tol=1e-6)
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem=upper_support,
            negative_elem=base_support,
            min_gap=0.0,
            max_gap=0.0005,
        )
        ctx.expect_overlap(upper, base, axes="xy", min_overlap=0.12)

    with ctx.pose({yaw: 1.0}):
        ctx.expect_origin_distance(upper, base, axes="xy", max_dist=1e-6)
        ctx.expect_contact(upper, base, elem_a=upper_support, elem_b=base_support, contact_tol=1e-6)
        ctx.expect_gap(
            upper,
            base,
            axis="z",
            positive_elem=upper_support,
            negative_elem=base_support,
            min_gap=0.0,
            max_gap=0.0005,
        )
        ctx.expect_overlap(upper, base, axes="xy", min_overlap=0.12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
