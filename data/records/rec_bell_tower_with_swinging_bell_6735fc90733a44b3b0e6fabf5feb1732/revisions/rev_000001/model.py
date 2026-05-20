from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_X = 0.52
HINGE_Z = 2.36


def _cone_frustum(base_radius: float, top_radius: float, height: float, z0: float) -> cq.Workplane:
    """CadQuery cone with its base on z0 and top at z0 + height."""
    solid = cq.Solid.makeCone(
        base_radius,
        top_radius,
        height,
        pnt=cq.Vector(0.0, 0.0, z0),
        dir=cq.Vector(0.0, 0.0, 1.0),
    )
    return cq.Workplane("XY").add(solid)


def _frustum_shell(
    base_outer: float,
    top_outer: float,
    base_inner: float,
    top_inner: float,
    height: float,
    z0: float,
) -> cq.Workplane:
    outer = _cone_frustum(base_outer, top_outer, height, z0)
    inner = _cone_frustum(base_inner, top_inner, height + 0.020, z0 - 0.010)
    return outer.cut(inner)


def _cap_with_shingles() -> cq.Workplane:
    cap = _cone_frustum(0.42, 0.035, 0.48, 2.645)
    for z in (2.690, 2.775, 2.860, 2.945):
        # Raised courses protrude just outside the cone face and are unioned
        # into the cap so the roof remains a single connected timber assembly.
        t = (z - 2.645) / 0.48
        r0 = 0.42 + (0.035 - 0.42) * t + 0.020
        r1 = 0.42 + (0.035 - 0.42) * ((z + 0.032 - 2.645) / 0.48) + 0.014
        cap = cap.union(_cone_frustum(r0, max(r1, 0.030), 0.032, z))
    return cap


def _bell_shell() -> cq.Workplane:
    """A hollow flared iron bell, authored around the local Z axis."""
    shell = _frustum_shell(0.315, 0.070, 0.245, 0.035, 0.680, -0.780)
    flared_lip = _frustum_shell(0.335, 0.302, 0.252, 0.238, 0.070, -0.800)
    neck = _frustum_shell(0.088, 0.058, 0.040, 0.026, 0.100, -0.165)
    # A cast crown boss ties the hollow shell to the headstock block and gives
    # the top of the bell the heavier iron mass seen on harbor warning bells.
    crown = cq.Workplane("XY").cylinder(0.075, 0.072).translate((0.0, 0.0, -0.090))
    return shell.union(flared_lip).union(neck).union(crown)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seaside_harbor_bell_tower")

    timber = model.material("weathered_timber", rgba=(0.46, 0.29, 0.16, 1.0))
    end_grain = model.material("dark_end_grain", rgba=(0.24, 0.14, 0.08, 1.0))
    shingle = model.material("salt_grey_shingle", rgba=(0.34, 0.33, 0.29, 1.0))
    iron = model.material("blackened_iron", rgba=(0.035, 0.037, 0.040, 1.0))
    worn_iron = model.material("worn_iron_edges", rgba=(0.12, 0.12, 0.11, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=0.13, length=2.64),
        origin=Origin(xyz=(0.0, 0.0, 1.32)),
        material=timber,
        name="post",
    )
    # Slightly proud horizontal checking bands break up the plain cylinder and
    # read as weathered timber grain without making the post a separate stack.
    for idx, z in enumerate((0.58, 1.18, 1.82, 2.36)):
        tower.visual(
            Cylinder(radius=0.133, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=end_grain,
            name=f"post_grain_{idx}",
        )

    tower.visual(
        Box((1.40, 0.17, 0.15)),
        origin=Origin(xyz=(0.25, 0.0, 2.585)),
        material=timber,
        name="crossbeam",
    )
    tower.visual(
        Box((0.035, 0.175, 0.155)),
        origin=Origin(xyz=(-0.45, 0.0, 2.585)),
        material=end_grain,
        name="beam_end_0",
    )
    tower.visual(
        Box((0.035, 0.175, 0.155)),
        origin=Origin(xyz=(0.95, 0.0, 2.585)),
        material=end_grain,
        name="beam_end_1",
    )

    # The cap is a low seaside cone centered on the post top, with overlapping
    # shingle courses so it reads as shingled rather than a smooth cone.
    tower.visual(
        mesh_from_cadquery(_cap_with_shingles(), "conical_cap"),
        material=shingle,
        name="conical_cap",
    )

    # Fixed iron cleat/hanger under the timber beam.  The two cheek plates flank
    # the moving hinge pin with a small visible clearance rather than colliding.
    tower.visual(
        Box((0.58, 0.13, 0.055)),
        origin=Origin(xyz=(HINGE_X, 0.0, 2.495)),
        material=iron,
        name="headstock_cleat",
    )
    tower.visual(
        Box((0.050, 0.125, 0.230)),
        origin=Origin(xyz=(HINGE_X - 0.245, 0.0, 2.390)),
        material=iron,
        name="cleat_plate_0",
    )
    tower.visual(
        Box((0.050, 0.125, 0.230)),
        origin=Origin(xyz=(HINGE_X + 0.245, 0.0, 2.390)),
        material=iron,
        name="cleat_plate_1",
    )

    bell = model.part("bell")
    bell.visual(
        mesh_from_cadquery(_bell_shell(), "bell_shell", tolerance=0.0008, angular_tolerance=0.06),
        material=iron,
        name="bell_shell",
    )
    bell.visual(
        Box((0.260, 0.092, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=iron,
        name="headstock_block",
    )
    bell.visual(
        Cylinder(radius=0.029, length=0.520),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_iron,
        name="hinge_pin",
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("tower_to_bell")

    ctx.allow_overlap(
        bell,
        tower,
        elem_a="hinge_pin",
        elem_b="cleat_plate_0",
        reason="The iron hinge pin is intentionally captured through the fixed cleat plate socket.",
    )
    ctx.allow_overlap(
        bell,
        tower,
        elem_a="hinge_pin",
        elem_b="cleat_plate_1",
        reason="The iron hinge pin is intentionally captured through the fixed cleat plate socket.",
    )

    ctx.expect_gap(
        bell,
        tower,
        axis="x",
        positive_elem="bell_shell",
        negative_elem="post",
        min_gap=0.040,
        name="bell hangs clear of the timber post",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="x",
        positive_elem="hinge_pin",
        negative_elem="cleat_plate_0",
        max_penetration=0.050,
        name="hinge pin remains captured in the inner cleat plate",
    )
    ctx.expect_gap(
        tower,
        bell,
        axis="x",
        positive_elem="cleat_plate_1",
        negative_elem="hinge_pin",
        max_penetration=0.050,
        name="hinge pin remains captured in the outer cleat plate",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="cleat_plate_0",
        min_overlap=0.040,
        name="pin axis aligns with the inner cleat plate",
    )
    ctx.expect_overlap(
        bell,
        tower,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="cleat_plate_1",
        min_overlap=0.040,
        name="pin axis aligns with the outer cleat plate",
    )

    def _aabb_center(aabb):
        lo, hi = aabb
        return tuple((float(lo[i]) + float(hi[i])) * 0.5 for i in range(3))

    rest_center = _aabb_center(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    with ctx.pose({hinge: 0.65}):
        swung_center = _aabb_center(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    ctx.check(
        "positive hinge angle swings bell about the horizontal beam axis",
        swung_center[1] > rest_center[1] + 0.10 and swung_center[2] > rest_center[2] + 0.04,
        details=f"rest_center={rest_center}, swung_center={swung_center}",
    )

    return ctx.report()


object_model = build_object_model()
