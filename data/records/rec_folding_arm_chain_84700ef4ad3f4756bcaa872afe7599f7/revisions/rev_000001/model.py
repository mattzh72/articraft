from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PIVOT_Z = 0.070
BAR_0_LEN = 0.260
BAR_1_LEN = 0.180
BAR_2_LEN = 0.220
BAR_RADIUS = 0.020
BAR_THICKNESS = 0.012
PIVOT_HOLE_RADIUS = 0.0085


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _link_profile(length: float, radius: float, segments: int = 18) -> list[tuple[float, float]]:
    """Capsule outline in local X/Z, returned as a 2-D profile for extrusion."""
    pts: list[tuple[float, float]] = []
    pts.append((0.0, -radius))
    pts.append((length, -radius))
    for i in range(segments + 1):
        a = -math.pi / 2.0 + math.pi * i / segments
        pts.append((length + radius * math.cos(a), radius * math.sin(a)))
    pts.append((0.0, radius))
    for i in range(segments + 1):
        a = math.pi / 2.0 + math.pi * i / segments
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _link_mesh(length: float, name: str):
    geom = ExtrudeWithHolesGeometry(
        _link_profile(length, BAR_RADIUS),
        [
            _circle_profile(0.0, 0.0, PIVOT_HOLE_RADIUS),
            _circle_profile(length, 0.0, PIVOT_HOLE_RADIUS),
        ],
        BAR_THICKNESS,
        center=True,
    )
    # ExtrudeWithHolesGeometry extrudes along its local Z.  Rotate so the
    # through-holes and machined-pin axes are along local Y, while the capsule
    # profile remains in the folding X/Z plane.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _add_y_cylinder(
    part,
    radius: float,
    length: float,
    *,
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_three_bar_folding_linkage")

    painted_steel = model.material("powder_coated_anchor", rgba=(0.16, 0.19, 0.21, 1.0))
    zinc = model.material("brushed_zinc", rgba=(0.62, 0.64, 0.61, 1.0))
    satin_zinc = model.material("satin_zinc", rgba=(0.53, 0.55, 0.53, 1.0))
    black_oxide = model.material("black_oxide_pins", rgba=(0.035, 0.038, 0.040, 1.0))
    brass_tab = model.material("warm_catch_tab", rgba=(0.82, 0.60, 0.23, 1.0))
    rubber = model.material("black_rubber_pad", rgba=(0.015, 0.014, 0.013, 1.0))

    anchor = model.part("anchor_foot")
    anchor.visual(
        Box((0.220, 0.130, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, 0.007)),
        material=painted_steel,
        name="foot_plate",
    )
    anchor.visual(
        Box((0.070, 0.070, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.019)),
        material=painted_steel,
        name="clevis_base_pad",
    )
    anchor.visual(
        Box((0.052, 0.011, 0.076)),
        origin=Origin(xyz=(0.0, -0.030, 0.052)),
        material=painted_steel,
        name="clevis_cheek_neg",
    )
    anchor.visual(
        Box((0.052, 0.011, 0.076)),
        origin=Origin(xyz=(0.0, 0.030, 0.052)),
        material=painted_steel,
        name="clevis_cheek_pos",
    )
    for x in (-0.035, 0.105):
        for y in (-0.043, 0.043):
            anchor.visual(
                Cylinder(radius=0.0075, length=0.0030),
                origin=Origin(xyz=(x, y, 0.015)),
                material=black_oxide,
                name=f"screw_head_{x:+.3f}_{y:+.3f}",
            )
            anchor.visual(
                Box((0.012, 0.0020, 0.0012)),
                origin=Origin(xyz=(x, y, 0.0170)),
                material=painted_steel,
                name=f"screw_slot_{x:+.3f}_{y:+.3f}",
            )
    # The visible pin is split into cheek stubs so the central link has real
    # swing clearance in the clevis gap instead of a solid collision proxy.
    for y, suffix in ((-0.032, "neg"), (0.032, "pos")):
        _add_y_cylinder(
            anchor,
            0.0046,
            0.014,
            xyz=(0.0, y, PIVOT_Z),
            material=black_oxide,
            name=f"root_pin_{suffix}",
        )
    for y, suffix in ((-0.039, "neg"), (0.039, "pos")):
        _add_y_cylinder(
            anchor,
            0.014,
            0.005,
            xyz=(0.0, y, PIVOT_Z),
            material=black_oxide,
            name=f"root_pin_head_{suffix}",
        )

    bar_0 = model.part("bar_0")
    bar_0.visual(
        _link_mesh(BAR_0_LEN, "bar_0_plate_mesh"),
        origin=Origin(),
        material=zinc,
        name="bar_plate",
    )
    bar_0.visual(
        Cylinder(radius=0.012, length=0.0185),
        origin=Origin(xyz=(0.0, -0.01525, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_zinc,
        name="root_washer_neg",
    )
    bar_0.visual(
        Cylinder(radius=0.012, length=0.0185),
        origin=Origin(xyz=(0.0, 0.01525, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_zinc,
        name="root_washer_pos",
    )
    bar_0.visual(
        Cylinder(radius=0.0046, length=0.024),
        origin=Origin(xyz=(BAR_0_LEN, 0.000, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="distal_pin",
    )
    _add_y_cylinder(
        bar_0,
        0.013,
        0.004,
        xyz=(BAR_0_LEN, -0.0075, 0.0),
        material=satin_zinc,
        name="distal_collar",
    )
    _add_y_cylinder(
        bar_0,
        0.010,
        0.006,
        xyz=(BAR_0_LEN, -0.015, 0.0),
        material=black_oxide,
        name="distal_pin_head",
    )

    bar_1 = model.part("bar_1")
    bar_1.visual(
        _link_mesh(BAR_1_LEN, "bar_1_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=satin_zinc,
        name="bar_plate",
    )
    bar_1.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_zinc,
        name="proximal_washer",
    )
    bar_1.visual(
        Cylinder(radius=0.0046, length=0.024),
        origin=Origin(xyz=(BAR_1_LEN, 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="distal_pin",
    )
    _add_y_cylinder(
        bar_1,
        0.013,
        0.004,
        xyz=(BAR_1_LEN, 0.0315, 0.0),
        material=satin_zinc,
        name="distal_collar",
    )
    _add_y_cylinder(
        bar_1,
        0.010,
        0.006,
        xyz=(BAR_1_LEN, 0.039, 0.0),
        material=black_oxide,
        name="distal_pin_head",
    )

    bar_2 = model.part("bar_2")
    bar_2.visual(
        _link_mesh(BAR_2_LEN, "bar_2_plate_mesh"),
        origin=Origin(),
        material=zinc,
        name="bar_plate",
    )
    bar_2.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_zinc,
        name="proximal_washer",
    )
    bar_2.visual(
        Box((0.078, 0.014, 0.016)),
        origin=Origin(xyz=(BAR_2_LEN + 0.036, 0.0, -0.006)),
        material=brass_tab,
        name="hook_tongue",
    )
    bar_2.visual(
        Box((0.014, 0.014, 0.052)),
        origin=Origin(xyz=(BAR_2_LEN + 0.076, 0.0, 0.014)),
        material=brass_tab,
        name="hook_lip",
    )
    bar_2.visual(
        Box((0.004, 0.016, 0.034)),
        origin=Origin(xyz=(BAR_2_LEN + 0.067, 0.0, 0.013)),
        material=rubber,
        name="hook_pad",
    )

    model.articulation(
        "anchor_to_bar_0",
        ArticulationType.REVOLUTE,
        parent=anchor,
        child=bar_0,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(0.0, -0.35, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.60, upper=0.35),
    )
    model.articulation(
        "bar_0_to_bar_1",
        ArticulationType.REVOLUTE,
        parent=bar_0,
        child=bar_1,
        origin=Origin(xyz=(BAR_0_LEN, 0.0, 0.0), rpy=(0.0, -2.30, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=0.0, upper=2.30),
    )
    model.articulation(
        "bar_1_to_bar_2",
        ArticulationType.REVOLUTE,
        parent=bar_1,
        child=bar_2,
        origin=Origin(xyz=(BAR_1_LEN, 0.0, 0.0), rpy=(0.0, 2.80, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.4, lower=-2.80, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    anchor = object_model.get_part("anchor_foot")
    bar_0 = object_model.get_part("bar_0")
    bar_1 = object_model.get_part("bar_1")
    bar_2 = object_model.get_part("bar_2")
    j0 = object_model.get_articulation("anchor_to_bar_0")
    j1 = object_model.get_articulation("bar_0_to_bar_1")
    j2 = object_model.get_articulation("bar_1_to_bar_2")

    ctx.check(
        "three revolute folding joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "bar lengths are intentionally nonuniform",
        BAR_0_LEN > BAR_2_LEN > BAR_1_LEN and (BAR_0_LEN - BAR_1_LEN) > 0.060,
        details=f"lengths={(BAR_0_LEN, BAR_1_LEN, BAR_2_LEN)}",
    )
    ctx.check(
        "all pivots fold about the common y axis",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in (j0, j1, j2)),
        details=f"axes={[j.axis for j in (j0, j1, j2)]}",
    )

    ctx.expect_contact(
        bar_0,
        anchor,
        elem_a="root_washer_pos",
        elem_b="clevis_cheek_pos",
        name="root bushing bears on the clevis cheek",
    )
    ctx.expect_contact(
        bar_1,
        bar_0,
        elem_a="proximal_washer",
        elem_b="distal_pin",
        name="second bar is mechanically captured at first pin",
    )
    ctx.expect_contact(
        bar_2,
        bar_1,
        elem_a="proximal_washer",
        elem_b="distal_pin",
        name="third bar is mechanically captured at second pin",
    )
    ctx.expect_gap(
        anchor,
        bar_0,
        axis="y",
        positive_elem="clevis_cheek_pos",
        negative_elem="bar_plate",
        min_gap=0.016,
        name="root clevis has positive-side swing clearance",
    )
    ctx.expect_gap(
        bar_0,
        anchor,
        axis="y",
        positive_elem="bar_plate",
        negative_elem="clevis_cheek_neg",
        min_gap=0.016,
        name="root clevis has negative-side swing clearance",
    )
    ctx.expect_gap(
        bar_1,
        bar_0,
        axis="y",
        positive_elem="bar_plate",
        negative_elem="bar_plate",
        min_gap=0.010,
        name="first and second bar plates are offset for clearance",
    )
    ctx.expect_gap(
        bar_1,
        bar_2,
        axis="y",
        positive_elem="bar_plate",
        negative_elem="bar_plate",
        min_gap=0.010,
        name="second and third bar plates are offset for clearance",
    )
    ctx.expect_gap(
        bar_2,
        bar_0,
        axis="z",
        positive_elem="bar_plate",
        negative_elem="bar_plate",
        min_gap=0.004,
        name="folded package keeps nonadjacent bars separated",
    )

    with ctx.pose({j0: 0.35, j1: 2.30, j2: -2.80}):
        ctx.expect_gap(
            bar_0,
            anchor,
            axis="z",
            positive_elem="bar_plate",
            negative_elem="foot_plate",
            min_gap=0.030,
            name="extended first bar clears the grounded foot",
        )
        ctx.expect_gap(
            bar_1,
            bar_0,
            axis="y",
            positive_elem="bar_plate",
            negative_elem="bar_plate",
            min_gap=0.010,
            name="extended first-second plate offset remains clear",
        )
        ctx.expect_gap(
            bar_1,
            bar_2,
            axis="y",
            positive_elem="bar_plate",
            negative_elem="bar_plate",
            min_gap=0.010,
            name="extended second-third plate offset remains clear",
        )
        hook_lip_aabb = ctx.part_element_world_aabb(bar_2, elem="hook_lip")
        hook_tongue_aabb = ctx.part_element_world_aabb(bar_2, elem="hook_tongue")
        bar_2_aabb = ctx.part_world_aabb(bar_2)
        ctx.check(
            "extended hook reaches beyond the anchor",
            bar_2_aabb is not None and bar_2_aabb[1][0] > 0.70,
            details=f"bar_2_aabb={bar_2_aabb}",
        )
        ctx.check(
            "end tab has an upturned hook lip",
            hook_lip_aabb is not None
            and hook_tongue_aabb is not None
            and hook_lip_aabb[1][2] > hook_tongue_aabb[1][2] + 0.030,
            details=f"hook_lip={hook_lip_aabb}, hook_tongue={hook_tongue_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
