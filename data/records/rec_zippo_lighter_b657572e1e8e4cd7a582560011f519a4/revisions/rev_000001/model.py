from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_open_case(width: float, depth: float, height: float, wall: float) -> cq.Workplane:
    """Lower lighter case: a rounded rectangular metal cup open at the top."""
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0022)
    )
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - wall, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    return outer.cut(inner)


def _rounded_lid_cap(
    width: float,
    depth: float,
    height: float,
    wall: float,
    hinge_offset: float,
    hinge_z: float,
) -> cq.Workplane:
    """Hollow cap in the lid frame: hinge axis is just outside the right wall."""
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0022)
    )
    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - wall,
        centered=(True, True, False),
    )
    cap = outer.cut(inner)
    return cap.translate((-width / 2.0 - hinge_offset, 0.0, -hinge_z))


def _knurled_striker_wheel(radius: float, thickness: float, tooth_count: int = 24) -> cq.Workplane:
    """Small steel striker wheel; local cylinder axis is Z before visual rotation."""
    wheel = cq.Workplane("XY").circle(radius).extrude(thickness / 2.0, both=True)
    tooth_radial = 0.00085
    tooth_tangent = 0.00115
    for i in range(tooth_count):
        angle = 360.0 * i / tooth_count
        tooth = (
            cq.Workplane("XY")
            .box(tooth_radial, tooth_tangent, thickness * 1.04)
            .translate((radius + tooth_radial / 2.0, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        wheel = wheel.union(tooth)
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_flip_top_lighter")

    # Pocket Zippo-like proportions, in meters.
    width = 0.038
    depth = 0.0125
    body_h = 0.039
    lid_h = 0.018
    wall = 0.0012
    hinge_offset = 0.0030
    hinge_z = 0.0022
    hinge_radius = 0.0016
    hinge_x = width / 2.0 + hinge_offset
    hinge_world_z = body_h + hinge_z

    chrome = model.material("brushed_chrome", rgba=(0.70, 0.70, 0.66, 1.0))
    dark_steel = model.material("dark_perforated_steel", rgba=(0.42, 0.43, 0.40, 1.0))
    bright_steel = model.material("bright_striker_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    wick_mat = model.material("charred_cotton_wick", rgba=(0.86, 0.73, 0.50, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.03, 0.03, 0.025, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_rounded_open_case(width, depth, body_h, wall), "lower_case_shell"),
        material=chrome,
        name="case_shell",
    )
    # Dark inner fuel insert makes the open-top case read hollow while supporting the chimney.
    case.visual(
        Box((0.031, depth - 2.0 * wall, body_h - 0.002)),
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
        material=shadow,
        name="fuel_insert",
    )
    case.visual(
        Box((0.027, depth - 0.0024, 0.0012)),
        origin=Origin(xyz=(-0.002, 0.0, body_h + 0.0006)),
        material=dark_steel,
        name="insert_deck",
    )

    chimney_panel = PerforatedPanelGeometry(
        (0.023, 0.0155),
        0.00065,
        hole_diameter=0.0017,
        pitch=(0.0042, 0.0038),
        frame=0.0020,
        corner_radius=0.0010,
        stagger=True,
    )
    chimney_mesh = mesh_from_geometry(chimney_panel, "chimney_perforated_panel")
    case.visual(
        chimney_mesh,
        origin=Origin(
            xyz=(-0.0030, 0.00435, body_h + 0.0083),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="chimney_front",
    )
    case.visual(
        chimney_mesh,
        origin=Origin(
            xyz=(-0.0030, -0.00435, body_h + 0.0083),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="chimney_back",
    )
    case.visual(
        Box((0.0010, 0.0087, 0.0145)),
        origin=Origin(xyz=(-0.0150, 0.0, body_h + 0.0077)),
        material=dark_steel,
        name="chimney_end_wall",
    )
    case.visual(
        Cylinder(radius=0.00115, length=0.0090),
        origin=Origin(xyz=(-0.0068, 0.0, body_h + 0.0054)),
        material=wick_mat,
        name="wick",
    )

    axle_x = 0.0068
    axle_z = body_h + 0.0104
    case.visual(
        Cylinder(radius=0.00055, length=0.0107),
        origin=Origin(xyz=(axle_x, 0.0, axle_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="wheel_axle",
    )

    # Side-mounted hinge knuckles attached to the lower case, leaving the center gap for the lid knuckle.
    for y, name in ((0.00425, "hinge_knuckle_0"), (-0.00425, "hinge_knuckle_1")):
        case.visual(
            Cylinder(radius=hinge_radius, length=0.0031),
            origin=Origin(xyz=(hinge_x, y, hinge_world_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=name,
        )
        case.visual(
            Box((hinge_offset, 0.0031, 0.0060)),
            origin=Origin(xyz=(width / 2.0 + hinge_offset / 2.0, y, hinge_world_z)),
            material=chrome,
            name=f"hinge_leaf_{0 if y > 0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(
            _rounded_lid_cap(width, depth, lid_h, wall, hinge_offset, hinge_z),
            "lid_shell",
        ),
        material=chrome,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.0042),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_knuckle",
    )
    lid.visual(
        Box((hinge_offset, 0.0042, 0.0060)),
        origin=Origin(xyz=(-hinge_offset / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_leaf",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        mesh_from_cadquery(_knurled_striker_wheel(0.00425, 0.0044), "striker_wheel"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="wheel_disk",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_world_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=5.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=striker_wheel,
        origin=Origin(xyz=(axle_x, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.allow_overlap(
        case,
        wheel,
        elem_a="wheel_axle",
        elem_b="wheel_disk",
        reason="The striker wheel is intentionally captured on the fixed transverse axle.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            case,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="case_shell",
            max_gap=0.0012,
            max_penetration=0.0,
            name="closed lid sits on the case rim",
        )
        ctx.expect_within(
            case,
            lid,
            axes="xy",
            inner_elem="chimney_front",
            outer_elem="lid_shell",
            margin=0.0007,
            name="front chimney guard fits inside the closed lid",
        )
        ctx.expect_within(
            case,
            lid,
            axes="xy",
            inner_elem="chimney_back",
            outer_elem="lid_shell",
            margin=0.0007,
            name="rear chimney guard fits inside the closed lid",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.55}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(
            lid,
            case,
            axis="x",
            positive_elem="lid_shell",
            negative_elem="chimney_front",
            min_gap=0.004,
            name="opened lid swings clear of the chimney",
        )

    ctx.check(
        "lid rotates upward on side hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.010,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.expect_overlap(
        wheel,
        case,
        axes="y",
        elem_a="wheel_disk",
        elem_b="wheel_axle",
        min_overlap=0.004,
        name="striker wheel is retained on its axle",
    )
    ctx.expect_gap(
        case,
        wheel,
        axis="y",
        positive_elem="chimney_front",
        negative_elem="wheel_disk",
        min_gap=0.00045,
        name="front chimney guard clears striker wheel",
    )
    ctx.expect_gap(
        wheel,
        case,
        axis="y",
        positive_elem="wheel_disk",
        negative_elem="chimney_back",
        min_gap=0.00045,
        name="rear chimney guard clears striker wheel",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        spun_wheel_pos = ctx.part_world_position(wheel)
    ctx.check(
        "striker wheel spins about a fixed transverse axle",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and all(abs(a - b) < 1.0e-7 for a, b in zip(rest_wheel_pos, spun_wheel_pos)),
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
