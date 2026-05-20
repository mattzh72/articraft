from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CASE_W = 0.026
CASE_D = 0.011
CASE_H = 0.055
LID_H = 0.022
WALL = 0.00115
HINGE_R = 0.00155
HINGE_X = -CASE_W / 2.0 - 0.0010


def _rounded_open_case(width: float, depth: float, height: float) -> cq.Workplane:
    """Thin-walled open-top lighter body, local base on z=0."""
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.0022)
    )
    cut_h = height - 0.0018 + 0.004
    cutter = (
        cq.Workplane("XY")
        .box(width - 2 * WALL, depth - 2 * WALL, cut_h)
        .translate((0.0, 0.0, 0.0018 + cut_h / 2.0))
    )
    return outer.cut(cutter)


def _rounded_lid_cap(width: float, depth: float, height: float) -> cq.Workplane:
    """Open-bottom cap; local hinge axis is just outside the x-min wall."""
    hinge_clearance = 0.0010
    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((hinge_clearance + width / 2.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.0020)
    )
    cut_h = height - 0.0012 + 0.004
    cutter = (
        cq.Workplane("XY")
        .box(width - 2 * WALL, depth - 2 * WALL, cut_h)
        .translate((hinge_clearance + width / 2.0, 0.0, -0.002 + cut_h / 2.0))
    )
    return outer.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_side_hinged_lighter")

    brushed = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.64, 1.0))
    dark = model.material("charcoal_shadow", rgba=(0.025, 0.023, 0.020, 1.0))
    hinge_dark = model.material("hinge_shadow", rgba=(0.18, 0.17, 0.15, 1.0))
    brass = model.material("warm_brass", rgba=(0.92, 0.68, 0.30, 1.0))
    wick_mat = model.material("singed_wick", rgba=(0.45, 0.36, 0.23, 1.0))
    flint_steel = model.material("flint_steel", rgba=(0.42, 0.42, 0.40, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_rounded_open_case(CASE_W, CASE_D, CASE_H), "open_case_shell", tolerance=0.00035),
        material=brushed,
        name="case_shell",
    )
    # Shallow engraved-looking front inset and base cap keep the narrow metal body from
    # reading as a plain block.
    case.visual(
        Box((CASE_W * 0.72, 0.00035, CASE_H * 0.68)),
        origin=Origin(xyz=(0.0010, -CASE_D / 2 - 0.00008, CASE_H * 0.43)),
        material=Material("soft_reflection", rgba=(0.58, 0.57, 0.52, 1.0)),
        name="front_inset",
    )
    case.visual(
        Box((CASE_W * 0.86, 0.00065, 0.0020)),
        origin=Origin(xyz=(0.0, -CASE_D / 2 - 0.00012, 0.0040)),
        material=hinge_dark,
        name="base_seam",
    )

    # Compact chimney insert fixed in the case mouth.
    case.visual(
        Box((0.020, 0.0070, 0.0020)),
        origin=Origin(xyz=(0.0015, 0.0, CASE_H + 0.0010)),
        material=flint_steel,
        name="insert_deck",
    )
    for x, name in ((-0.0112, "insert_lip_0"), (0.0122, "insert_lip_1")):
        case.visual(
            Box((0.0038, 0.0070, 0.0020)),
            origin=Origin(xyz=(x, 0.0, CASE_H + 0.0006)),
            material=flint_steel,
            name=name,
        )
    case.visual(
        Box((0.0010, 0.0068, 0.0140)),
        origin=Origin(xyz=(-0.0065, 0.0, CASE_H + 0.0088)),
        material=flint_steel,
        name="chimney_side_0",
    )
    case.visual(
        Box((0.0010, 0.0068, 0.0140)),
        origin=Origin(xyz=(0.0055, 0.0, CASE_H + 0.0088)),
        material=flint_steel,
        name="chimney_side_1",
    )
    case.visual(
        Box((0.0120, 0.0010, 0.0120)),
        origin=Origin(xyz=(-0.0005, -0.0034, CASE_H + 0.0080)),
        material=flint_steel,
        name="chimney_front",
    )
    for i, x in enumerate((-0.0043, 0.0000, 0.0043)):
        for j, z in enumerate((CASE_H + 0.0058, CASE_H + 0.0100)):
            case.visual(
                Cylinder(radius=0.00072, length=0.00034),
                origin=Origin(xyz=(x, -0.00395, z), rpy=(pi / 2, 0.0, 0.0)),
                material=dark,
                name=f"chimney_hole_{i}_{j}",
            )
    case.visual(
        Cylinder(radius=0.00105, length=0.0110),
        origin=Origin(xyz=(-0.0008, 0.0003, CASE_H + 0.0074)),
        material=wick_mat,
        name="wick",
    )

    # Visible hinge hardware on the fixed body.
    case.visual(
        Box((0.0012, CASE_D * 0.54, 0.014)),
        origin=Origin(xyz=((HINGE_X + -CASE_W / 2.0) / 2.0, 0.0, CASE_H - 0.0080)),
        material=flint_steel,
        name="case_hinge_leaf",
    )
    case.visual(
        Cylinder(radius=HINGE_R, length=0.014),
        origin=Origin(xyz=(HINGE_X, 0.0, CASE_H - 0.0080)),
        material=flint_steel,
        name="case_hinge_barrel",
    )

    # Forks beside the chimney support the freely spinning striker wheel.
    for y, name in ((-0.00305, "striker_fork_0"), (0.00305, "striker_fork_1")):
        case.visual(
            Box((0.0015, 0.00085, 0.0170)),
            origin=Origin(xyz=(0.0067, y, CASE_H + 0.0085)),
            material=flint_steel,
            name=name,
        )
    case.visual(
        Box((0.0040, 0.0074, 0.0013)),
        origin=Origin(xyz=(0.0054, 0.0, CASE_H + 0.0054)),
        material=flint_steel,
        name="striker_fork_bridge",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_lid_cap(CASE_W, CASE_D, LID_H), "hollow_lid_cap", tolerance=0.00035),
        material=brushed,
        name="lid_shell",
    )
    lid.visual(
        Box((0.0017, CASE_D * 0.54, 0.0170)),
        origin=Origin(xyz=(0.00055, 0.0, LID_H / 2.0)),
        material=flint_steel,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.0170),
        origin=Origin(xyz=(0.0, 0.0, LID_H / 2.0)),
        material=flint_steel,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.00072, length=0.0042),
        origin=Origin(xyz=(0.0040, 0.0, 0.0100), rpy=(pi / 2, 0.0, 0.0)),
        material=hinge_dark,
        name="cam_pivot_pin",
    )
    for y, name in ((-0.0019, "cam_pivot_bracket_0"), (0.0019, "cam_pivot_bracket_1")):
        lid.visual(
            Box((0.0042, 0.00075, 0.0024)),
            origin=Origin(xyz=(0.0026, y, 0.0100)),
            material=flint_steel,
            name=name,
        )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.0070,
                0.0030,
                body_style="cylindrical",
                edge_radius=0.00025,
                grip=KnobGrip(style="knurled", count=28, depth=0.00045, helix_angle_deg=24.0),
            ),
            "knurled_striker_wheel",
        ),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=hinge_dark,
        name="wheel",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.00135, length=0.0023),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=brass,
        name="cam_hub",
    )
    cam_lever.visual(
        Box((0.0021, 0.0012, 0.0092)),
        origin=Origin(xyz=(0.0012, 0.0, -0.0045), rpy=(0.0, 0.26, 0.0)),
        material=brass,
        name="cam_arm",
    )
    cam_lever.visual(
        Box((0.0030, 0.00125, 0.0017)),
        origin=Origin(xyz=(0.0024, 0.0, -0.0090), rpy=(0.0, -0.35, 0.0)),
        material=brass,
        name="cam_toe",
    )

    lid_hinge = model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, CASE_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "case_to_striker_wheel",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=striker_wheel,
        origin=Origin(xyz=(0.0067, 0.0, CASE_H + 0.0162)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.06, velocity=60.0),
    )
    model.articulation(
        "lid_to_cam_lever",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam_lever,
        origin=Origin(xyz=(0.0040, 0.0, 0.0100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0, lower=0.0, upper=0.85),
        mimic=Mimic(joint=lid_hinge.name, multiplier=0.46, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("striker_wheel")
    cam = object_model.get_part("cam_lever")
    lid_hinge = object_model.get_articulation("case_to_lid")
    wheel_spin = object_model.get_articulation("case_to_striker_wheel")
    cam_pivot = object_model.get_articulation("lid_to_cam_lever")

    ctx.allow_overlap(
        lid,
        cam,
        elem_a="cam_pivot_pin",
        elem_b="cam_hub",
        reason="The small cam hub is intentionally captured around the lid-mounted pivot pin.",
    )
    ctx.expect_within(
        lid,
        cam,
        axes="xz",
        inner_elem="cam_pivot_pin",
        outer_elem="cam_hub",
        margin=0.00035,
        name="cam hub surrounds pivot pin in cross-section",
    )
    ctx.expect_overlap(
        lid,
        cam,
        axes="y",
        elem_a="cam_pivot_pin",
        elem_b="cam_hub",
        min_overlap=0.0015,
        name="cam hub stays on the short pivot",
    )

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="case_shell",
        max_gap=0.0007,
        max_penetration=0.0,
        name="closed lid seats on case mouth",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        elem_a="lid_shell",
        elem_b="case_shell",
        min_overlap=0.008,
        name="lid footprint matches slim case",
    )
    ctx.expect_gap(
        wheel,
        case,
        axis="y",
        positive_elem="wheel",
        negative_elem="striker_fork_0",
        min_gap=0.0002,
        max_gap=0.0020,
        name="striker wheel clears one fork cheek",
    )
    ctx.expect_gap(
        case,
        wheel,
        axis="y",
        positive_elem="striker_fork_1",
        negative_elem="wheel",
        min_gap=0.0002,
        max_gap=0.0020,
        name="striker wheel clears other fork cheek",
    )

    case_box = ctx.part_world_aabb(case)
    lid_box = ctx.part_world_aabb(lid)
    if case_box is not None and lid_box is not None:
        lo = tuple(min(case_box[0][i], lid_box[0][i]) for i in range(3))
        hi = tuple(max(case_box[1][i], lid_box[1][i]) for i in range(3))
        dims = tuple(hi[i] - lo[i] for i in range(3))
    else:
        dims = None
    ctx.check(
        "slim pocket lighter proportions",
        dims is not None
        and 0.072 <= dims[2] <= 0.080
        and 0.026 <= dims[0] <= 0.032
        and 0.010 <= dims[1] <= 0.014
        and dims[2] / dims[0] > 2.45,
        details=f"closed overall dims={dims}",
    )

    ctx.check(
        "striker wheel uses continuous spin",
        getattr(wheel_spin, "articulation_type", None) == ArticulationType.CONTINUOUS
        or getattr(wheel_spin, "type", None) == ArticulationType.CONTINUOUS,
        details=f"wheel joint={wheel_spin}",
    )
    ctx.check(
        "cam lever follows lid opening",
        getattr(cam_pivot, "mimic", None) is not None,
        details="cam lever joint should mimic the side lid hinge",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_cam_aabb = ctx.part_world_aabb(cam)
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        open_cam_aabb = ctx.part_world_aabb(cam)
        ctx.expect_gap(
            lid,
            case,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="case_shell",
            max_penetration=0.0,
            name="opened lid stays above case rim",
        )
    if closed_lid_aabb is not None and open_lid_aabb is not None:
        closed_y = (closed_lid_aabb[0][1] + closed_lid_aabb[1][1]) / 2.0
        open_y = (open_lid_aabb[0][1] + open_lid_aabb[1][1]) / 2.0
    else:
        closed_y = open_y = None
    ctx.check(
        "side hinge swings lid outward",
        closed_y is not None and open_y is not None and open_y > closed_y + 0.010,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )
    if closed_cam_aabb is not None and open_cam_aabb is not None:
        closed_cam_y = (closed_cam_aabb[0][1] + closed_cam_aabb[1][1]) / 2.0
        open_cam_y = (open_cam_aabb[0][1] + open_cam_aabb[1][1]) / 2.0
    else:
        closed_cam_y = open_cam_y = None
    ctx.check(
        "cam lever changes pose with lid",
        closed_cam_y is not None and open_cam_y is not None and abs(open_cam_y - closed_cam_y) > 0.0015,
        details=f"closed_cam_y={closed_cam_y}, open_cam_y={open_cam_y}",
    )

    return ctx.report()


object_model = build_object_model()
