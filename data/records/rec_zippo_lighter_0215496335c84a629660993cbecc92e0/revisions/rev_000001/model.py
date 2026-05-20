from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_W = 0.0360
BODY_D = 0.0122
BODY_H = 0.0425
LID_H = 0.0150
WALL_T = 0.00065
CORNER_R = 0.00165
LID_CORNER_R = 0.00150
HINGE_R = 0.00110
HINGE_AXIS_X = BODY_W * 0.5 + 0.00105
HINGE_AXIS_Z = BODY_H + 0.00310
LID_W = BODY_W + 0.00010
LID_D = BODY_D + 0.00010

BARREL_SPAN = 0.01040
BARREL_GAP = 0.00035
BARREL_LEN = (BARREL_SPAN - 4.0 * BARREL_GAP) / 5.0
BARREL_START = -BARREL_SPAN * 0.5 + BARREL_LEN * 0.5
BARREL_CENTERS = tuple(BARREL_START + i * (BARREL_LEN + BARREL_GAP) for i in range(5))

INSERT_W = 0.0280
INSERT_D = 0.0091
INSERT_H = 0.0240
INSERT_BOTTOM_Z = -0.0185
CHIMNEY_W = 0.0154
CHIMNEY_D = 0.0088
CHIMNEY_H = 0.0118
CHIMNEY_BOTTOM_Z = -0.0036
WHEEL_R = 0.00325
WHEEL_W = 0.00440
AXLE_R = 0.00075
AXLE_LEN = CHIMNEY_W - 0.00050
WICK_R = 0.00105
WICK_Y = 0.00205
CHIMNEY_ORIGIN = Origin(xyz=(0.0, -0.00110, BODY_H + 0.00475))


def _box(dx: float, dy: float, dz: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate(center)


def _cyl_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length * 0.5))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _cyl_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length * 0.5))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _cyl_z(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length * 0.5))
    )


def _shell_box(
    width: float,
    depth: float,
    height: float,
    wall: float,
    radius: float,
    *,
    open_top: bool = False,
    open_bottom: bool = False,
) -> cq.Workplane:
    outer = _box(width, depth, height, (0.0, 0.0, height * 0.5))
    outer = outer.edges("|Z").fillet(radius)
    inner_height = height - wall
    if open_top:
        inner_center_z = wall + inner_height * 0.5
    elif open_bottom:
        inner_center_z = inner_height * 0.5
    else:
        inner_height = height - 2.0 * wall
        inner_center_z = height * 0.5
    inner = _box(width - 2.0 * wall, depth - 2.0 * wall, inner_height, (0.0, 0.0, inner_center_z))
    return outer.cut(inner)


def _build_case_shape() -> cq.Workplane:
    case = _shell_box(BODY_W, BODY_D, BODY_H, WALL_T, CORNER_R, open_top=True)
    case = case.edges("<Z").fillet(CORNER_R * 0.55)

    lug_dx = HINGE_AXIS_X - BODY_W * 0.5 + HINGE_R * 0.65
    lug_dz = 0.0050
    for barrel_index in (0, 2, 4):
        y_center = BARREL_CENTERS[barrel_index]
        lug = _box(
            lug_dx,
            BARREL_LEN + 0.00020,
            lug_dz,
            (BODY_W * 0.5 + lug_dx * 0.5 - HINGE_R * 0.15, y_center, HINGE_AXIS_Z - 0.00100),
        )
        barrel = _cyl_y(HINGE_R, BARREL_LEN, (HINGE_AXIS_X, y_center, HINGE_AXIS_Z))
        case = case.union(lug).union(barrel)

    return case


def _build_lid_shape() -> cq.Workplane:
    lid = _shell_box(LID_W, LID_D, LID_H, WALL_T, LID_CORNER_R, open_bottom=True)
    lid = lid.edges(">Z").fillet(LID_CORNER_R * 0.58)
    lid = lid.translate((-HINGE_AXIS_X, 0.0, -(HINGE_AXIS_Z - BODY_H)))

    lid_right_face = -HINGE_AXIS_X + LID_W * 0.5
    lug_dx = abs(lid_right_face) + HINGE_R * 0.65
    lug_dz = 0.0050
    for barrel_index in (1, 3):
        y_center = BARREL_CENTERS[barrel_index]
        lug = _box(
            lug_dx,
            BARREL_LEN + 0.00020,
            lug_dz,
            (lid_right_face + lug_dx * 0.5 - HINGE_R * 0.08, y_center, -0.00055),
        )
        barrel = _cyl_y(HINGE_R, BARREL_LEN, (0.0, y_center, 0.0))
        lid = lid.union(lug).union(barrel)

    return lid


def _build_chimney_shape() -> cq.Workplane:
    insert = _shell_box(INSERT_W, INSERT_D, INSERT_H, WALL_T, 0.00085, open_top=True).translate((0.0, 0.0, INSERT_BOTTOM_Z))
    upper = _shell_box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H, WALL_T, 0.00070, open_top=True).translate((0.0, 0.0, CHIMNEY_BOTTOM_Z))
    chimney = insert.union(upper)

    front_back_window = _box(
        CHIMNEY_W - 0.00240,
        CHIMNEY_D * 1.5,
        CHIMNEY_H - 0.00200,
        (0.0, 0.0, CHIMNEY_BOTTOM_Z + CHIMNEY_H * 0.5 + 0.00020),
    )
    chimney = chimney.cut(front_back_window)

    axle_bore = _cyl_x(AXLE_R + 0.00015, CHIMNEY_W + 0.00300, (0.0, 0.0, 0.0))
    chimney = chimney.cut(axle_bore)

    for hole_y in (-0.00175, 0.00175):
        for hole_z in (-0.00160, 0.00050, 0.00270, 0.00490):
            chimney = chimney.cut(_cyl_x(0.00072, CHIMNEY_W + 0.00200, (0.0, hole_y, hole_z)))

    wick = _cyl_z(WICK_R, 0.0140, (0.0, WICK_Y, -0.00100))
    chimney = chimney.union(wick)

    return chimney


def _build_wheel_shape() -> cq.Workplane:
    axle = _cyl_x(AXLE_R, AXLE_LEN, (0.0, 0.0, 0.0))
    disc = cq.Workplane("YZ").polygon(20, WHEEL_R * 2.0).extrude(WHEEL_W).translate((-WHEEL_W * 0.5, 0.0, 0.0))
    hub = _cyl_x(WHEEL_R * 0.78, WHEEL_W * 0.72, (0.0, 0.0, 0.0))
    return axle.union(disc).union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_lighter")

    brushed_steel = model.material("brushed_steel", rgba=(0.75, 0.76, 0.79, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.44, 0.45, 0.47, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_build_case_shape(), "case_shell"),
        material=brushed_steel,
        name="case_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "lid_shell"),
        material=brushed_steel,
        name="lid_shell",
    )

    chimney = model.part("chimney")
    chimney.visual(
        mesh_from_cadquery(_build_chimney_shape(), "chimney_shell"),
        material=satin_steel,
        name="chimney_shell",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_build_wheel_shape(), "wheel_shell"),
        material=wheel_steel,
        name="wheel_shell",
    )

    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=5.0,
            lower=0.0,
            upper=math.radians(118.0),
        ),
    )
    model.articulation(
        "case_to_chimney",
        ArticulationType.FIXED,
        parent=case,
        child=chimney,
        origin=CHIMNEY_ORIGIN,
    )
    model.articulation(
        "chimney_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=chimney,
        child=wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    chimney = object_model.get_part("chimney")
    wheel = object_model.get_part("wheel")
    lid_hinge = object_model.get_articulation("case_to_lid")

    ctx.allow_overlap(
        case,
        chimney,
        reason="The chimney insert is intentionally nested inside the open lighter body cavity, and the shell-style case visual is being classified as penetrating its contained insert.",
    )
    ctx.allow_overlap(
        lid,
        chimney,
        reason="In the closed pose the lid intentionally encloses the chimney insert, and the thin hollow lid shell is being classified as overlapping the contained insert volume.",
    )
    ctx.allow_overlap(
        chimney,
        wheel,
        reason="The striker wheel axle intentionally passes through the chimney guard holes, and the shell-based chimney visual is being classified as overlapping that retained axle fit.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            case,
            axes="xy",
            elem_a="lid_shell",
            elem_b="case_shell",
            min_overlap=0.010,
            name="closed lid covers the body footprint",
        )
        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "closed lid aligns to the case seam",
            lid_aabb is not None and abs(lid_aabb[0][2] - BODY_H) <= 0.00035,
            details=f"lid_aabb={lid_aabb}, body_top={BODY_H}",
        )
        ctx.expect_within(
            chimney,
            case,
            axes="xy",
            inner_elem="chimney_shell",
            outer_elem="case_shell",
            margin=0.0015,
            name="chimney insert stays inside the case plan",
        )
        ctx.expect_within(
            wheel,
            chimney,
            axes="xz",
            inner_elem="wheel_shell",
            outer_elem="chimney_shell",
            margin=0.0012,
            name="striker wheel stays between the chimney guards",
        )
        chimney_aabb = ctx.part_element_world_aabb(chimney, elem="chimney_shell")
        ctx.check(
            "closed lid remains taller than the chimney",
            lid_aabb is not None and chimney_aabb is not None and lid_aabb[1][2] > chimney_aabb[1][2] + 0.0015,
            details=f"lid_aabb={lid_aabb}, chimney_aabb={chimney_aabb}",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({lid_hinge: upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward from the side hinge",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.010
            and open_lid_aabb[1][0] > closed_lid_aabb[1][0] + 0.020,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
