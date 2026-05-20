from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _rounded_rect_profile(
    width: float,
    depth: float,
    radius: float,
    *,
    center_y: float,
    corner_segments: int = 8,
) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_d = depth / 2.0
    centers = (
        (half_w - radius, center_y + half_d - radius, 0.0),
        (-half_w + radius, center_y + half_d - radius, math.pi / 2.0),
        (-half_w + radius, center_y - half_d + radius, math.pi),
        (half_w - radius, center_y - half_d + radius, 3.0 * math.pi / 2.0),
    )
    pts: list[tuple[float, float]] = []
    for cx, cy, start in centers:
        for i in range(corner_segments + 1):
            a = start + (math.pi / 2.0) * i / corner_segments
            pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return pts


def _strap_profile(
    *,
    root_y: float,
    nose_y: float,
    root_width: float,
    neck_width: float,
    corner_radius: float,
    arc_segments: int = 16,
) -> list[tuple[float, float]]:
    neck_y = nose_y - neck_width / 2.0
    root_half = root_width / 2.0
    neck_half = neck_width / 2.0
    pts: list[tuple[float, float]] = [
        (-root_half + corner_radius, root_y),
        (-root_half, root_y + corner_radius),
        (-neck_half, neck_y),
    ]
    # Rounded nose, centered on the strap centerline.
    for i in range(arc_segments + 1):
        a = math.pi - math.pi * i / arc_segments
        pts.append((neck_half * math.cos(a), neck_y + neck_half * math.sin(a)))
    pts.extend(
        [
            (root_half, root_y + corner_radius),
            (root_half - corner_radius, root_y),
        ]
    )
    return pts


def _hollow_cylinder_x(
    x0: float,
    x1: float,
    outer_radius: float,
    inner_radius: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        outer0.append(geom.add_vertex(x0, outer_radius * ca, outer_radius * sa))
        outer1.append(geom.add_vertex(x1, outer_radius * ca, outer_radius * sa))
        inner0.append(geom.add_vertex(x0, inner_radius * ca, inner_radius * sa))
        inner1.append(geom.add_vertex(x1, inner_radius * ca, inner_radius * sa))

    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        geom.add_face(inner0[j], inner0[i], inner1[j])
        geom.add_face(inner1[j], inner0[i], inner1[i])
        geom.add_face(outer0[j], outer0[i], inner0[i])
        geom.add_face(outer0[j], inner0[i], inner0[j])
        geom.add_face(outer1[i], outer1[j], inner1[j])
        geom.add_face(outer1[i], inner1[j], inner1[i])
    return geom


def _merge_tubes(spans: list[tuple[float, float]], outer_radius: float, inner_radius: float) -> MeshGeometry:
    geom = MeshGeometry()
    for x0, x1 in spans:
        geom.merge(_hollow_cylinder_x(x0, x1, outer_radius, inner_radius))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="strap_and_leaf_hatch_hinge")

    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.55, 1.0))
    worn_edges = model.material("worn_bright_edges", rgba=(0.72, 0.74, 0.70, 1.0))
    dark_bores = model.material("dark_recesses", rgba=(0.03, 0.032, 0.035, 1.0))

    hinge_length = 0.180
    leaf_thickness = 0.004
    leaf_z = -0.014
    barrel_outer = 0.012
    barrel_inner = 0.0058
    # The straight pin is a close bearing fit in the hollow knuckles so the
    # moving strap is supported by the grounded leaf rather than floating free.
    pin_radius = barrel_inner
    clearance = 0.003
    knuckle_len = (hinge_length - 4.0 * clearance) / 5.0

    spans: list[tuple[float, float]] = []
    x = -hinge_length / 2.0
    for _ in range(5):
        spans.append((x, x + knuckle_len))
        x += knuckle_len + clearance
    back_spans = [spans[i] for i in (0, 2, 4)]
    strap_spans = [spans[i] for i in (1, 3)]

    back_leaf = model.part("back_leaf")
    strap_leaf = model.part("strap_leaf")

    back_plate = ExtrudeWithHolesGeometry(
        _rounded_rect_profile(0.180, 0.082, 0.008, center_y=-0.047),
        [
            _circle_profile(-0.052, -0.026, 0.0062),
            _circle_profile(0.052, -0.026, 0.0062),
            _circle_profile(-0.052, -0.067, 0.0062),
            _circle_profile(0.052, -0.067, 0.0062),
        ],
        leaf_thickness,
        center=True,
    ).translate(0.0, 0.0, leaf_z)
    back_leaf.visual(
        mesh_from_geometry(back_plate, "back_leaf_plate"),
        material=galvanized,
        name="back_plate",
    )
    back_leaf.visual(
        mesh_from_geometry(_merge_tubes(back_spans, barrel_outer, barrel_inner), "back_leaf_knuckles"),
        material=galvanized,
        name="fixed_knuckles",
    )

    for idx, (x0, x1) in enumerate(back_spans):
        back_leaf.visual(
            Box((x1 - x0, 0.018, 0.005)),
            origin=Origin(xyz=((x0 + x1) / 2.0, -0.004, -0.0117)),
            material=galvanized,
            name=f"fixed_curl_tab_{idx}",
        )

    pin_length = hinge_length + 0.010
    back_leaf.visual(
        Cylinder(radius=pin_radius, length=pin_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edges,
        name="straight_pin",
    )
    cap_len = 0.005
    for side, x_cap in (("end_0", -hinge_length / 2.0 - cap_len / 2.0 + 0.0002), ("end_1", hinge_length / 2.0 + cap_len / 2.0 - 0.0002)):
        back_leaf.visual(
            Cylinder(radius=barrel_outer * 1.08, length=cap_len),
            origin=Origin(xyz=(x_cap, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_edges,
            name=f"pin_head_{side}",
        )

    for idx, (sx, sy) in enumerate(((-0.052, -0.026), (0.052, -0.026), (-0.052, -0.067), (0.052, -0.067))):
        back_leaf.visual(
            Cylinder(radius=0.010, length=0.0022),
            origin=Origin(xyz=(sx, sy, -0.0113)),
            material=worn_edges,
            name=f"back_screw_{idx}",
        )
        back_leaf.visual(
            Box((0.014, 0.0022, 0.0009)),
            origin=Origin(xyz=(sx, sy, -0.0099)),
            material=dark_bores,
            name=f"back_drive_slot_{idx}",
        )

    strap_plate = ExtrudeWithHolesGeometry(
        _strap_profile(
            root_y=0.006,
            nose_y=0.340,
            root_width=0.150,
            neck_width=0.054,
            corner_radius=0.010,
        ),
        [
            _circle_profile(0.0, 0.070, 0.0065),
            _circle_profile(0.0, 0.168, 0.0065),
            _circle_profile(0.0, 0.278, 0.0065),
        ],
        leaf_thickness,
        center=True,
    ).translate(0.0, 0.0, leaf_z)
    strap_leaf.visual(
        mesh_from_geometry(strap_plate, "strap_leaf_plate"),
        material=galvanized,
        name="strap_plate",
    )
    for idx, (x0, x1) in enumerate(strap_spans):
        strap_leaf.visual(
            mesh_from_geometry(_hollow_cylinder_x(x0, x1, barrel_outer, barrel_inner), f"strap_leaf_knuckle_{idx}"),
            material=galvanized,
            name=f"moving_knuckle_{idx}",
        )
        strap_leaf.visual(
            Box((x1 - x0, 0.018, 0.005)),
            origin=Origin(xyz=((x0 + x1) / 2.0, 0.004, -0.0117)),
            material=galvanized,
            name=f"moving_curl_tab_{idx}",
        )

    for idx, sy in enumerate((0.070, 0.168, 0.278)):
        strap_leaf.visual(
            Cylinder(radius=0.0105, length=0.0024),
            origin=Origin(xyz=(0.0, sy, -0.0112)),
            material=worn_edges,
            name=f"strap_screw_{idx}",
        )
        strap_leaf.visual(
            Box((0.015, 0.0022, 0.0008)),
            origin=Origin(xyz=(0.0, sy, -0.0097)),
            material=dark_bores,
            name=f"strap_drive_slot_{idx}",
        )

    model.articulation(
        "pin_joint",
        ArticulationType.REVOLUTE,
        parent=back_leaf,
        child=strap_leaf,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_leaf = object_model.get_part("back_leaf")
    strap_leaf = object_model.get_part("strap_leaf")
    pin_joint = object_model.get_articulation("pin_joint")

    for idx in range(2):
        moving_knuckle = f"moving_knuckle_{idx}"
        ctx.allow_overlap(
            back_leaf,
            strap_leaf,
            elem_a="straight_pin",
            elem_b=moving_knuckle,
            reason=(
                "The straight pin is intentionally a close captured bearing fit "
                "through the moving knuckle bore."
            ),
        )
        ctx.expect_within(
            back_leaf,
            strap_leaf,
            axes="yz",
            inner_elem="straight_pin",
            outer_elem=moving_knuckle,
            margin=0.001,
            name=f"pin centered in moving knuckle {idx}",
        )
        ctx.expect_overlap(
            back_leaf,
            strap_leaf,
            axes="x",
            elem_a="straight_pin",
            elem_b=moving_knuckle,
            min_overlap=0.030,
            name=f"pin spans moving knuckle {idx}",
        )

    ctx.expect_gap(
        strap_leaf,
        back_leaf,
        axis="y",
        positive_elem="strap_plate",
        negative_elem="back_plate",
        min_gap=0.006,
        max_gap=0.020,
        name="flat leaves meet on opposite sides of barrel",
    )

    rest_aabb = ctx.part_element_world_aabb(strap_leaf, elem="strap_plate")
    with ctx.pose({pin_joint: 1.2}):
        raised_aabb = ctx.part_element_world_aabb(strap_leaf, elem="strap_plate")

    ctx.check(
        "strap leaf swings upward about straight pin",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.20,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
