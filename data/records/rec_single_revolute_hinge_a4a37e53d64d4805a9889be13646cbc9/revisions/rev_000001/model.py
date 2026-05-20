from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_LENGTH = 0.088
BARREL_RADIUS = 0.0063
PIN_RADIUS = 0.00235
MOVING_BORE_RADIUS = PIN_RADIUS + 0.00035
GROUND_LEAF_WIDTH = 0.034
MOVING_LEAF_WIDTH = 0.032
GROUND_LEAF_THICKNESS = 0.0046
MOVING_LEAF_THICKNESS = 0.0040
GROUND_PLATE_INNER_X = -0.0105
MOVING_PLATE_INNER_X = 0.0105
GROUND_WEB_INNER_X = -0.0034
MOVING_WEB_INNER_X = 0.0034
PIN_HEAD_RADIUS = 0.0043
WASHER_OUTER_RADIUS = 0.00545
WASHER_INNER_RADIUS = PIN_RADIUS + 0.00020

GROUND_KNUCKLES = [(-0.0440, -0.0270), (-0.0090, 0.0090), (0.0270, 0.0440)]
MOVING_KNUCKLES = [(-0.02575, -0.01025), (0.01025, 0.02575)]
WASHER_BANDS = [(-0.02655, -0.02575), (0.02575, 0.02655)]


def _tube_segment(z0: float, z1: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(z1 - z0)
    )


def _solid_segment(z0: float, z1: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(z1 - z0)


def _xz_rect(x0: float, x1: float, z0: float, z1: float, thickness: float, *, negative_y: bool) -> cq.Workplane:
    rect = (
        cq.Workplane("XZ")
        .center(0.5 * (x0 + x1), 0.5 * (z0 + z1))
        .rect(abs(x1 - x0), abs(z1 - z0))
        .extrude(thickness)
    )
    if negative_y:
        rect = rect.translate((0.0, -thickness, 0.0))
    return rect


def _ground_leaf_plate() -> cq.Workplane:
    x_outer = -(BARREL_RADIUS + GROUND_LEAF_WIDTH)
    plate = (
        cq.Workplane("XZ")
        .center(0.5 * (x_outer + GROUND_PLATE_INNER_X), 0.0)
        .sketch()
        .rect(GROUND_PLATE_INNER_X - x_outer, HINGE_LENGTH)
        .vertices()
        .chamfer(0.0030)
        .finalize()
        .extrude(GROUND_LEAF_THICKNESS)
        .translate((0.0, -GROUND_LEAF_THICKNESS, 0.0))
    )

    hole_points = [(-0.0230, -0.028), (-0.0200, 0.0), (-0.0230, 0.028)]
    through_cutter = (
        cq.Workplane("XZ")
        .pushPoints(hole_points)
        .circle(0.00255)
        .extrude(0.020, both=True)
    )
    counterbore = (
        cq.Workplane("XZ")
        .pushPoints(hole_points)
        .circle(0.00475)
        .extrude(0.00125)
        .translate((0.0, -GROUND_LEAF_THICKNESS, 0.0))
    )
    return plate.cut(through_cutter).cut(counterbore)


def _ground_leaf_hinge_cluster() -> cq.Workplane:
    body = cq.Workplane("XY")

    for z0, z1 in GROUND_KNUCKLES:
        body = body.union(
            _xz_rect(
                GROUND_PLATE_INNER_X,
                GROUND_WEB_INNER_X,
                z0,
                z1,
                GROUND_LEAF_THICKNESS,
                negative_y=True,
            )
        )
        body = body.union(_solid_segment(z0, z1, BARREL_RADIUS))

    body = body.union(_solid_segment(-0.0460, 0.0460, PIN_RADIUS))
    body = body.union(_solid_segment(-0.0478, -0.0440, PIN_HEAD_RADIUS))
    body = body.union(_solid_segment(0.0440, 0.0478, PIN_HEAD_RADIUS))
    body = body.union(_tube_segment(WASHER_BANDS[0][0], WASHER_BANDS[0][1], WASHER_OUTER_RADIUS, WASHER_INNER_RADIUS))
    body = body.union(_tube_segment(WASHER_BANDS[1][0], WASHER_BANDS[1][1], WASHER_OUTER_RADIUS, WASHER_INNER_RADIUS))
    return body


def _moving_leaf_plate() -> cq.Workplane:
    x_outer = BARREL_RADIUS + MOVING_LEAF_WIDTH
    plate = (
        cq.Workplane("XZ")
        .center(0.5 * (MOVING_PLATE_INNER_X + x_outer), 0.0)
        .sketch()
        .rect(x_outer - MOVING_PLATE_INNER_X, HINGE_LENGTH)
        .vertices()
        .fillet(0.0038)
        .finalize()
        .extrude(MOVING_LEAF_THICKNESS)
    )

    slot_cutter = (
        cq.Workplane("XZ")
        .pushPoints([(0.0205, -0.0205), (0.0205, 0.0205)])
        .slot2D(0.0115, 0.0050, 90)
        .extrude(0.020, both=True)
    )
    center_hole = (
        cq.Workplane("XZ")
        .pushPoints([(0.0175, 0.0)])
        .circle(0.0022)
        .extrude(0.020, both=True)
    )
    return plate.cut(slot_cutter).cut(center_hole)


def _moving_leaf_hinge_cluster() -> cq.Workplane:
    body = cq.Workplane("XY")

    for z0, z1 in MOVING_KNUCKLES:
        body = body.union(
            _xz_rect(
                MOVING_WEB_INNER_X,
                MOVING_PLATE_INNER_X,
                z0,
                z1,
                MOVING_LEAF_THICKNESS,
                negative_y=False,
            )
        )
        body = body.union(_tube_segment(z0, z1, BARREL_RADIUS, MOVING_BORE_RADIUS))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_compact_hinge")

    zinc = model.material("zinc_leaf", rgba=(0.73, 0.75, 0.78, 1.0))
    black_oxide = model.material("black_oxide_leaf", rgba=(0.34, 0.36, 0.38, 1.0))
    dark_pin = model.material("dark_pin", rgba=(0.22, 0.24, 0.26, 1.0))

    grounded_leaf = model.part("grounded_leaf")
    grounded_leaf.visual(
        mesh_from_cadquery(_ground_leaf_plate(), "grounded_leaf_plate"),
        origin=Origin(),
        material=zinc,
        name="grounded_leaf_plate",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(_ground_leaf_hinge_cluster(), "grounded_leaf_hinge_cluster"),
        origin=Origin(),
        material=dark_pin,
        name="grounded_leaf_hinge_cluster",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_moving_leaf_plate(), "moving_leaf_plate"),
        origin=Origin(),
        material=black_oxide,
        name="moving_leaf_plate",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_moving_leaf_hinge_cluster(), "moving_leaf_hinge_cluster"),
        origin=Origin(),
        material=black_oxide,
        name="moving_leaf_hinge_cluster",
    )

    hinge = model.articulation(
        "grounded_to_moving",
        ArticulationType.REVOLUTE,
        parent=grounded_leaf,
        child=moving_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.60,
        ),
    )
    grounded_leaf.meta["role"] = "grounded_leaf"
    moving_leaf.meta["role"] = "carried_leaf"
    hinge.meta["mechanism"] = "primary_hinge"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded_leaf = object_model.get_part("grounded_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("grounded_to_moving")
    grounded_plate = grounded_leaf.get_visual("grounded_leaf_plate")
    moving_plate = moving_leaf.get_visual("moving_leaf_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.allow_overlap(
        grounded_leaf,
        moving_leaf,
        reason="Coaxial interleaved hinge knuckles share a swept barrel envelope; exact plate clearance is asserted separately.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.check(
        "hinge parts present",
        all(part is not None for part in (grounded_leaf, moving_leaf)),
        "expected grounded and moving leaves",
    )
    ctx.check(
        "hinge axis is supported pin axis",
        hinge.axis == (0.0, 0.0, 1.0),
        f"unexpected hinge axis: {hinge.axis}",
    )
    ctx.expect_contact(
        grounded_leaf,
        moving_leaf,
        name="moving leaf is physically supported by grounded leaf hardware",
    )
    ctx.expect_gap(
        moving_leaf,
        grounded_leaf,
        axis="y",
        min_gap=0.00045,
        max_gap=0.00075,
        positive_elem=moving_plate,
        negative_elem=grounded_plate,
        name="leaf plates keep a small running clearance across the hinge line",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            grounded_leaf,
            moving_leaf,
            axes="z",
            min_overlap=0.070,
            name="interleaved leaves share barrel length",
        )
        ctx.expect_gap(
            moving_leaf,
            grounded_leaf,
            axis="x",
            min_gap=0.020,
            positive_elem=moving_plate,
            negative_elem=grounded_plate,
            name="rest pose keeps mounting leaves separated across hinge line",
        )

    with ctx.pose({hinge: 1.45}):
        ctx.expect_gap(
            moving_leaf,
            grounded_leaf,
            axis="x",
            min_gap=0.0005,
            positive_elem=moving_plate,
            negative_elem=grounded_plate,
            name="opened hinge plate clears grounded plate in swing path",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
