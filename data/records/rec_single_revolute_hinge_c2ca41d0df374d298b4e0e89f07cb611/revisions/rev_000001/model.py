from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_HEIGHT = 0.240
LEAF_WIDTH = 0.100
PLATE_THICKNESS = 0.008
LEAF_EDGE_OFFSET = 0.006
PLATE_Y_CENTER = -0.018
MOUNT_HOLE_DIAMETER = 0.010

KNUCKLE_OUTER_RADIUS = 0.015
KNUCKLE_BORE_RADIUS = 0.0105
PIN_RADIUS = 0.0092
SPACER_RADIUS = 0.013
HEAD_RADIUS = 0.015
HEAD_THICKNESS = 0.005

KNUCKLE_LENGTH = 0.038
KNUCKLE_GAP = 0.006
KNUCKLE_START_Z = -(5 * KNUCKLE_LENGTH + 4 * KNUCKLE_GAP) / 2.0
ROOT_BLOCK_WIDTH = 0.015
ROOT_BLOCK_THICKNESS = 0.020
ROOT_BLOCK_Y_CENTER = -0.010


def _cylinder(radius: float, length: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, z_center - length / 2.0))
    )


def _tube(outer_radius: float, inner_radius: float, length: float, z_center: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner).translate((0.0, 0.0, z_center - length / 2.0))


def _segment_center(index: int) -> float:
    return KNUCKLE_START_Z + index * (KNUCKLE_LENGTH + KNUCKLE_GAP) + KNUCKLE_LENGTH / 2.0


def _gap_center(index: int) -> float:
    gap_start = KNUCKLE_START_Z + index * (KNUCKLE_LENGTH + KNUCKLE_GAP) + KNUCKLE_LENGTH
    return gap_start + KNUCKLE_GAP / 2.0


def _plate_center_x(side: int) -> float:
    return side * (KNUCKLE_OUTER_RADIUS + LEAF_EDGE_OFFSET + LEAF_WIDTH / 2.0)


def _leaf_plate(side: int) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(LEAF_WIDTH, PLATE_THICKNESS, HINGE_HEIGHT)
        .translate((_plate_center_x(side), PLATE_Y_CENTER, 0.0))
    )
    return (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.020, -0.065),
                (-0.020, 0.065),
                (0.020, -0.065),
                (0.020, 0.065),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
    )


def _leaf_hinge_body(side: int, segment_indices: tuple[int, ...]) -> cq.Workplane:
    hinge_body = None
    for idx in segment_indices:
        segment = _cylinder(
            KNUCKLE_OUTER_RADIUS,
            KNUCKLE_LENGTH,
            _segment_center(idx),
        ).union(
            cq.Workplane("XY")
            .box(ROOT_BLOCK_WIDTH, ROOT_BLOCK_THICKNESS, KNUCKLE_LENGTH)
            .translate(
                (
                    side * (KNUCKLE_OUTER_RADIUS + LEAF_EDGE_OFFSET / 2.0),
                    ROOT_BLOCK_Y_CENTER,
                    _segment_center(idx),
                )
            )
        )
        hinge_body = segment if hinge_body is None else hinge_body.union(segment)
    bore = _cylinder(KNUCKLE_BORE_RADIUS, HINGE_HEIGHT + 0.012, 0.0)
    return hinge_body.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_door_hinge")

    leaf_finish = model.material("leaf_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    barrel_finish = model.material("barrel_finish", rgba=(0.38, 0.40, 0.43, 1.0))

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        mesh_from_cadquery(_leaf_plate(-1), "fixed_leaf_plate"),
        material=leaf_finish,
        name="fixed_plate",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_leaf_hinge_body(-1, (0, 2, 4)), "fixed_leaf_knuckles"),
        material=leaf_finish,
        name="fixed_knuckles",
    )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=PIN_RADIUS, length=5 * KNUCKLE_LENGTH + 4 * KNUCKLE_GAP),
        material=barrel_finish,
        name="barrel_pin",
    )
    for gap_index in range(4):
        barrel.visual(
            Cylinder(radius=SPACER_RADIUS, length=KNUCKLE_GAP),
            origin=Origin(xyz=(0.0, 0.0, _gap_center(gap_index))),
            material=barrel_finish,
            name=f"barrel_collar_{gap_index}",
        )
    barrel.visual(
        Cylinder(radius=HEAD_RADIUS, length=HEAD_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                KNUCKLE_START_Z - HEAD_THICKNESS / 2.0,
            )
        ),
        material=barrel_finish,
        name="barrel_head_bottom",
    )
    barrel.visual(
        Cylinder(radius=HEAD_RADIUS, length=HEAD_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -KNUCKLE_START_Z + HEAD_THICKNESS / 2.0,
            )
        ),
        material=barrel_finish,
        name="barrel_head_top",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_leaf_plate(1), "moving_leaf_plate"),
        material=leaf_finish,
        name="moving_plate",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_leaf_hinge_body(1, (1, 3)), "moving_leaf_knuckles"),
        material=leaf_finish,
        name="moving_knuckles",
    )

    model.articulation(
        "fixed_leaf_to_barrel",
        ArticulationType.FIXED,
        parent=fixed_leaf,
        child=barrel,
        origin=Origin(),
    )
    model.articulation(
        "barrel_hinge",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=30.0, velocity=1.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("fixed_leaf")
    barrel = object_model.get_part("barrel")
    moving_leaf = object_model.get_part("moving_leaf")

    fixed_plate = fixed_leaf.get_visual("fixed_plate")
    fixed_knuckles = fixed_leaf.get_visual("fixed_knuckles")
    barrel_pin = barrel.get_visual("barrel_pin")
    barrel_collar_1 = barrel.get_visual("barrel_collar_1")
    barrel_head_bottom = barrel.get_visual("barrel_head_bottom")
    moving_plate = moving_leaf.get_visual("moving_plate")
    moving_knuckles = moving_leaf.get_visual("moving_knuckles")
    hinge = object_model.get_articulation("barrel_hinge")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    axis = hinge.axis
    limits = hinge.motion_limits
    ctx.check(
        "barrel_axis_is_vertical",
        abs(axis[0]) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2] - 1.0) < 1e-9,
        details=f"axis={axis}",
    )
    ctx.check(
        "moving_leaf_has_one_sided_opening_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and limits.upper >= 1.5,
        details=f"limits={limits}",
    )

    ctx.expect_within(
        barrel,
        fixed_leaf,
        axes="xy",
        inner_elem=barrel_pin,
        outer_elem=fixed_knuckles,
        name="fixed_leaf_bore_contains_pin",
    )
    ctx.expect_contact(
        moving_leaf,
        barrel,
        elem_a=moving_knuckles,
        elem_b=barrel_collar_1,
        name="moving_leaf_is_supported_by_barrel",
    )
    ctx.expect_contact(
        fixed_leaf,
        barrel,
        elem_a=fixed_knuckles,
        elem_b=barrel_head_bottom,
        name="fixed_leaf_is_retained_on_barrel",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="x",
            positive_elem=moving_plate,
            negative_elem=fixed_plate,
            min_gap=0.040,
            max_gap=0.046,
            name="closed_leaf_seam_reads_cleanly",
        )

    with ctx.pose({hinge: 1.35}):
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="y",
            positive_elem=moving_plate,
            negative_elem=fixed_plate,
            min_gap=0.020,
            name="opened_leaf_swings_outward",
        )
        ctx.expect_within(
            barrel,
            moving_leaf,
            axes="xy",
            inner_elem=barrel_pin,
            outer_elem=moving_knuckles,
            name="opened_leaf_bore_stays_around_pin",
        )
        ctx.expect_contact(
            moving_leaf,
            barrel,
            elem_a=moving_knuckles,
            elem_b=barrel_collar_1,
            name="opened_leaf_stays_on_barrel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
