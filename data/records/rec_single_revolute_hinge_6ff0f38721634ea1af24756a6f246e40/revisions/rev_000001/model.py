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


THICKNESS = 0.0016
BARREL_RADIUS = 0.0040
BARREL_SPAN = 0.0380

BRACKET_BACKSET = 0.0130
BRACKET_HEIGHT = 0.0240

COVER_FLANGE_DROP = 0.0090
COVER_LEAF_LENGTH = 0.0300
COVER_LEAF_WIDTH = 0.0340

HOLE_RADIUS = 0.0018
SHEET_OVERLAP = 0.0010
EAR_LENGTH = 0.0060
EAR_HEIGHT = 0.0036

BRACKET_SEGMENTS = ((-0.0190, -0.0110), (-0.0040, 0.0040), (0.0110, 0.0190))
COVER_SEGMENTS = ((-0.0110, -0.0040), (0.0040, 0.0110))


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Shape:
    return cq.Workplane("XY").box(*size).translate(center).val()


def _barrel_segment(y0: float, y1: float) -> cq.Shape:
    return cq.Solid.makeCylinder(
        BARREL_RADIUS,
        y1 - y0,
        pnt=cq.Vector(0.0, y0, 0.0),
        dir=cq.Vector(0.0, 1.0, 0.0),
    )


def _x_hole(x_center: float, y_center: float, z_center: float, radius: float, length: float) -> cq.Shape:
    return (
        cq.Workplane("YZ")
        .center(y_center, z_center)
        .circle(radius)
        .extrude(length)
        .translate((x_center - length / 2.0, 0.0, 0.0))
        .val()
    )


def _z_hole(x_center: float, y_center: float, z_center: float, radius: float, length: float) -> cq.Shape:
    return (
        cq.Workplane("XY")
        .center(x_center, y_center)
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, z_center - length / 2.0))
        .val()
    )


def _fuse_all(*shapes: cq.Shape) -> cq.Shape:
    fused = cq.Workplane(obj=shapes[0])
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused.val()


def _make_bracket_body() -> cq.Shape:
    lip_x_min = -BRACKET_BACKSET - SHEET_OVERLAP
    lip_x_max = -BARREL_RADIUS
    vertical_plate = _box(
        (THICKNESS, BARREL_SPAN, BRACKET_HEIGHT),
        (-BRACKET_BACKSET - THICKNESS / 2.0, 0.0, -BRACKET_HEIGHT / 2.0),
    )
    support_lip = _box(
        (lip_x_max - lip_x_min, BARREL_SPAN, THICKNESS),
        ((lip_x_min + lip_x_max) / 2.0, 0.0, -THICKNESS / 2.0),
    )
    body = _fuse_all(vertical_plate, support_lip)

    bracket_wp = cq.Workplane(obj=body)
    for y_center, z_center in ((-0.0110, -0.0080), (0.0110, -0.0170)):
        bracket_wp = bracket_wp.cut(
            _x_hole(
                x_center=-BRACKET_BACKSET - THICKNESS / 2.0,
                y_center=y_center,
                z_center=z_center,
                radius=HOLE_RADIUS,
                length=0.0100,
            )
        )
    return bracket_wp.val()


def _make_bracket_lug(y0: float, y1: float) -> cq.Shape:
    ear = _box(
        (EAR_LENGTH, y1 - y0, EAR_HEIGHT),
        (-BARREL_RADIUS - 0.0010, (y0 + y1) / 2.0, -EAR_HEIGHT / 2.0),
    )
    return _fuse_all(ear, _barrel_segment(y0, y1))


def _make_cover_body() -> cq.Shape:
    flange_x_center = BARREL_RADIUS + THICKNESS / 2.0
    cover_x_min = BARREL_RADIUS - SHEET_OVERLAP
    hinge_flange = _box(
        (THICKNESS, BARREL_SPAN, COVER_FLANGE_DROP),
        (flange_x_center, 0.0, -COVER_FLANGE_DROP / 2.0),
    )
    cover_plate = _box(
        (COVER_LEAF_LENGTH, COVER_LEAF_WIDTH, THICKNESS),
        (
            cover_x_min + COVER_LEAF_LENGTH / 2.0,
            0.0,
            -COVER_FLANGE_DROP - THICKNESS / 2.0,
        ),
    )
    body = _fuse_all(hinge_flange, cover_plate)

    cover_wp = cq.Workplane(obj=body)
    for x_center, y_center in ((0.0150, -0.0100), (0.0245, 0.0100)):
        cover_wp = cover_wp.cut(
            _z_hole(
                x_center=x_center,
                y_center=y_center,
                z_center=-COVER_FLANGE_DROP - THICKNESS / 2.0,
                radius=HOLE_RADIUS,
                length=0.0080,
            )
        )
    return cover_wp.val()


def _make_cover_lug(y0: float, y1: float) -> cq.Shape:
    ear = _box(
        (EAR_LENGTH, y1 - y0, EAR_HEIGHT),
        (BARREL_RADIUS + 0.0010, (y0 + y1) / 2.0, -EAR_HEIGHT / 2.0),
    )
    return _fuse_all(ear, _barrel_segment(y0, y1))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_cover_hinge")

    bracket_finish = model.material("bracket_finish", rgba=(0.60, 0.63, 0.67, 1.0))
    cover_finish = model.material("cover_finish", rgba=(0.70, 0.73, 0.76, 1.0))

    bracket_leaf = model.part("bracket_leaf")
    bracket_leaf.visual(
        mesh_from_cadquery(_make_bracket_body(), "bracket_body"),
        material=bracket_finish,
        name="bracket_body",
    )
    for index, (y0, y1) in enumerate(BRACKET_SEGMENTS, start=1):
        bracket_leaf.visual(
            mesh_from_cadquery(_make_bracket_lug(y0, y1), f"bracket_lug_{index}"),
            material=bracket_finish,
            name=f"bracket_lug_{index}",
        )

    cover_leaf = model.part("cover_leaf")
    cover_leaf.visual(
        mesh_from_cadquery(_make_cover_body(), "cover_body"),
        material=cover_finish,
        name="cover_body",
    )
    for index, (y0, y1) in enumerate(COVER_SEGMENTS, start=1):
        cover_leaf.visual(
            mesh_from_cadquery(_make_cover_lug(y0, y1), f"cover_lug_{index}"),
            material=cover_finish,
            name=f"cover_lug_{index}",
        )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=bracket_leaf,
        child=cover_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket_leaf = object_model.get_part("bracket_leaf")
    cover_leaf = object_model.get_part("cover_leaf")
    cover_hinge = object_model.get_articulation("cover_hinge")

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

    limits = cover_hinge.motion_limits
    open_angle = limits.upper if limits is not None and limits.upper is not None else 1.70

    ctx.check(
        "hinge uses the supported barrel axis",
        tuple(cover_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={cover_hinge.axis}",
    )
    ctx.check(
        "hinge opens through a realistic cover range",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= 1.5,
        details=f"limits={limits}",
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_contact(
            cover_leaf,
            bracket_leaf,
            contact_tol=2e-4,
            name="closed knuckles stay supported by the bracket leaf",
        )
        ctx.expect_overlap(
            cover_leaf,
            bracket_leaf,
            axes="y",
            min_overlap=0.034,
            name="closed leaves share the full barrel span",
        )
        closed_cover_aabb = ctx.part_element_world_aabb(cover_leaf, elem="cover_body")

    with ctx.pose({cover_hinge: open_angle}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at full-open pose")
        ctx.expect_contact(
            cover_leaf,
            bracket_leaf,
            contact_tol=2e-4,
            name="opened knuckles remain supported on the barrel axis",
        )
        ctx.expect_overlap(
            cover_leaf,
            bracket_leaf,
            axes="y",
            min_overlap=0.034,
            name="opened hinge still retains the cover across the barrel span",
        )
        open_cover_aabb = ctx.part_element_world_aabb(cover_leaf, elem="cover_body")

    ctx.check(
        "cover leaf swings upward when opened",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.018,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
