from __future__ import annotations

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


CASE_W = 0.0380
CASE_D = 0.0135
CASE_H = 0.0435
LID_H = 0.0180

CASE_CORNER_R = 0.0015
CASE_FLOOR_T = 0.0011
TOP_BAND_H = 0.0042
TOP_BAND_INSET = 0.0009
CASE_INNER_W = CASE_W - 2.0 * (TOP_BAND_INSET + 0.00075)
CASE_INNER_D = CASE_D - 2.0 * (TOP_BAND_INSET + 0.00075)

LID_WALL = 0.0006
LID_ROOF_T = 0.0008

HINGE_OFFSET = 0.00095
HINGE_R = 0.00105
HINGE_SEG_L = 0.0022
HINGE_CENTERS = (-0.0051, -0.00255, 0.0, 0.00255, 0.0051)

INSERT_W = 0.0336
INSERT_D = 0.0094
INSERT_BODY_H = 0.0347
INSERT_CORNER_R = 0.0010
INSERT_TOP_PLATE_T = 0.0016

CHIMNEY_W = 0.0108
CHIMNEY_D = 0.0076
CHIMNEY_H = 0.0132
CHIMNEY_WALL = 0.0007
CHIMNEY_X = -0.0025
CHIMNEY_BASE_Z = INSERT_BODY_H - 0.0003

WHEEL_X = 0.0134
WHEEL_Z = INSERT_BODY_H + 0.0053
WHEEL_R = 0.0032
WHEEL_T = 0.0024
WHEEL_BORE_R = 0.00080
AXLE_R = 0.00055
AXLE_L = 0.0048

INSERT_BOTTOM_Z = CASE_FLOOR_T
INSERT_TRAVEL = 0.0180


def _ground_box(width: float, depth: float, height: float, fillet: float = 0.0) -> cq.Workplane:
    shape = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    if fillet > 0.0:
        shape = shape.edges("|Z").fillet(fillet)
    return shape


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center_xyz)
    )


def _mesh(shape: cq.Workplane, name: str, tolerance: float = 0.00018):
    return mesh_from_cadquery(shape, name, tolerance=tolerance, angular_tolerance=0.05)


def _build_case_shell() -> cq.Workplane:
    lower_wall_t = (CASE_W - CASE_INNER_W) * 0.5
    lower_face_t = (CASE_D - CASE_INNER_D) * 0.5
    band_outer_w = CASE_W - 2.0 * TOP_BAND_INSET
    band_outer_d = CASE_D - 2.0 * TOP_BAND_INSET
    band_wall_t = (band_outer_w - CASE_INNER_W) * 0.5
    lower_wall_h = CASE_H - TOP_BAND_H - CASE_FLOOR_T

    shell = _ground_box(CASE_W, CASE_D, CASE_FLOOR_T, 0.0008)
    shell = shell.union(
        _ground_box(lower_wall_t, CASE_D, lower_wall_h, 0.00035).translate(
            (-CASE_W * 0.5 + lower_wall_t * 0.5, 0.0, CASE_FLOOR_T)
        )
    )
    shell = shell.union(
        _ground_box(lower_wall_t, CASE_D, lower_wall_h, 0.00035).translate(
            (CASE_W * 0.5 - lower_wall_t * 0.5, 0.0, CASE_FLOOR_T)
        )
    )
    shell = shell.union(
        _ground_box(CASE_INNER_W, lower_face_t, lower_wall_h, 0.00035).translate(
            (0.0, CASE_D * 0.5 - lower_face_t * 0.5, CASE_FLOOR_T)
        )
    )
    shell = shell.union(
        _ground_box(CASE_INNER_W, lower_face_t, lower_wall_h, 0.00035).translate(
            (0.0, -CASE_D * 0.5 + lower_face_t * 0.5, CASE_FLOOR_T)
        )
    )

    band_z = CASE_H - TOP_BAND_H
    shell = shell.union(
        _ground_box(band_wall_t, band_outer_d, TOP_BAND_H, 0.00018).translate(
            (-band_outer_w * 0.5 + band_wall_t * 0.5, 0.0, band_z)
        )
    )
    shell = shell.union(
        _ground_box(band_wall_t, band_outer_d, TOP_BAND_H, 0.00018).translate(
            (band_outer_w * 0.5 - band_wall_t * 0.5, 0.0, band_z)
        )
    )
    shell = shell.union(
        _ground_box(CASE_INNER_W, band_wall_t, TOP_BAND_H, 0.00018).translate(
            (0.0, band_outer_d * 0.5 - band_wall_t * 0.5, band_z)
        )
    )
    shell = shell.union(
        _ground_box(CASE_INNER_W, band_wall_t, TOP_BAND_H, 0.00018).translate(
            (0.0, -band_outer_d * 0.5 + band_wall_t * 0.5, band_z)
        )
    )

    for index in (0, 2, 4):
        barrel = _y_cylinder(
            HINGE_R,
            HINGE_SEG_L,
            (-CASE_W * 0.5 - HINGE_OFFSET, HINGE_CENTERS[index], CASE_H),
        )
        bridge = _ground_box(0.0020, HINGE_SEG_L, 0.0040, 0.00008).translate(
            (-CASE_W * 0.5 + 0.00015, HINGE_CENTERS[index], CASE_H - HINGE_R - 0.0010)
        )
        shell = shell.union(barrel).union(bridge)

    return shell


def _build_lid_shell() -> cq.Workplane:
    x_center = HINGE_OFFSET + CASE_W * 0.5
    lid = _ground_box(CASE_W, CASE_D, LID_ROOF_T, 0.0008).translate((x_center, 0.0, LID_H - LID_ROOF_T))
    lid = lid.union(
        _ground_box(LID_WALL, CASE_D, LID_H, 0.00018).translate((HINGE_OFFSET + LID_WALL * 0.5, 0.0, 0.0))
    )
    lid = lid.union(
        _ground_box(LID_WALL, CASE_D, LID_H, 0.00018).translate((HINGE_OFFSET + CASE_W - LID_WALL * 0.5, 0.0, 0.0))
    )
    lid = lid.union(
        _ground_box(CASE_W - 2.0 * LID_WALL, LID_WALL, LID_H, 0.00018).translate(
            (x_center, CASE_D * 0.5 - LID_WALL * 0.5, 0.0)
        )
    )
    lid = lid.union(
        _ground_box(CASE_W - 2.0 * LID_WALL, LID_WALL, LID_H, 0.00018).translate(
            (x_center, -CASE_D * 0.5 + LID_WALL * 0.5, 0.0)
        )
    )

    for index in (1, 3):
        barrel = _y_cylinder(HINGE_R, HINGE_SEG_L, (0.0, HINGE_CENTERS[index], 0.0))
        bridge = _ground_box(0.0018, HINGE_SEG_L, 0.0038, 0.00008).translate((0.00125, HINGE_CENTERS[index], -HINGE_R - 0.0008))
        lid = lid.union(barrel).union(bridge)

    return lid


def _build_insert_shape() -> cq.Workplane:
    insert = _ground_box(INSERT_W, INSERT_D, INSERT_BODY_H, INSERT_CORNER_R)

    top_plate = _ground_box(0.0288, 0.0089, INSERT_TOP_PLATE_T, 0.0005).translate(
        (0.0, 0.0, INSERT_BODY_H - 0.00015)
    )
    insert = insert.union(top_plate)

    chimney_outer = _ground_box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H, 0.00045).translate(
        (CHIMNEY_X, 0.0, CHIMNEY_BASE_Z)
    )
    chimney_inner = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_W - 2.0 * CHIMNEY_WALL,
            CHIMNEY_D - 2.0 * CHIMNEY_WALL,
            CHIMNEY_H + 0.002,
            centered=(True, True, False),
        )
        .translate((CHIMNEY_X, 0.0, CHIMNEY_BASE_Z - 0.001))
    )
    chimney = chimney_outer.cut(chimney_inner)

    side_cutout = (
        cq.Workplane("XY")
        .box(CHIMNEY_W * 0.72, CHIMNEY_D * 0.62, 0.0068, centered=(True, True, False))
        .translate((CHIMNEY_X + CHIMNEY_W * 0.28, 0.0, CHIMNEY_BASE_Z + CHIMNEY_H - 0.0078))
    )
    chimney = chimney.cut(side_cutout)

    hole_cutters = None
    for hole_z in (CHIMNEY_BASE_Z + 0.0039, CHIMNEY_BASE_Z + 0.0072, CHIMNEY_BASE_Z + 0.0102):
        cutter = _y_cylinder(0.0006, CHIMNEY_D + 0.003, (CHIMNEY_X - 0.0022, 0.0, hole_z))
        hole_cutters = cutter if hole_cutters is None else hole_cutters.union(cutter)
    chimney = chimney.cut(hole_cutters)
    insert = insert.union(chimney)

    bracket_front = _ground_box(0.0015, 0.0010, 0.0072, 0.00018).translate(
        (WHEEL_X, WHEEL_T * 0.5 + 0.0011, INSERT_BODY_H - 0.0010)
    )
    bracket_rear = _ground_box(0.0015, 0.0010, 0.0072, 0.00018).translate(
        (WHEEL_X, -(WHEEL_T * 0.5 + 0.0011), INSERT_BODY_H - 0.0010)
    )
    axle_stub_front = _y_cylinder(AXLE_R, 0.0010, (WHEEL_X, WHEEL_T * 0.5 + 0.0011, WHEEL_Z))
    axle_stub_rear = _y_cylinder(AXLE_R, 0.0010, (WHEEL_X, -(WHEEL_T * 0.5 + 0.0011), WHEEL_Z))

    insert = insert.union(bracket_front).union(bracket_rear).union(axle_stub_front).union(axle_stub_rear)
    return insert


def _build_wheel_shape() -> cq.Workplane:
    wheel = cq.Workplane("XY").circle(WHEEL_R).circle(WHEEL_BORE_R).extrude(WHEEL_T, both=True)
    wheel = wheel.union(
        cq.Workplane("XY")
        .circle(WHEEL_R * 0.72)
        .circle(WHEEL_BORE_R)
        .extrude(WHEEL_T * 0.55, both=True)
    )

    grooves = None
    for angle_deg in range(0, 180, 12):
        groove = (
            cq.Workplane("XY")
            .box(0.00038, WHEEL_R * 2.35, WHEEL_T * 1.3)
            .translate((WHEEL_R - 0.00004, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), float(angle_deg))
        )
        grooves = groove if grooves is None else grooves.union(groove)
    wheel = wheel.cut(grooves)
    return wheel.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)


def _joint_kind(joint) -> str:
    raw = joint.articulation_type
    return getattr(raw, "name", str(raw)).lower()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_lighter")

    chrome_case = model.material("chrome_case", rgba=(0.77, 0.79, 0.82, 1.0))
    insert_steel = model.material("insert_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))

    case = model.part("case")
    case.visual(_mesh(_build_case_shell(), "lighter_case_shell", tolerance=0.00022), material=chrome_case, name="case_shell")

    lid = model.part("lid")
    lid.visual(_mesh(_build_lid_shell(), "lighter_lid_shell", tolerance=0.00022), material=chrome_case, name="lid_shell")

    insert = model.part("insert")
    insert.visual(_mesh(_build_insert_shape(), "lighter_insert", tolerance=0.00016), material=insert_steel, name="insert_body")

    wheel = model.part("wheel")
    wheel.visual(_mesh(_build_wheel_shape(), "lighter_wheel", tolerance=0.00012), material=dark_steel, name="wheel")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(-CASE_W * 0.5 - HINGE_OFFSET, 0.0, CASE_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0, lower=0.0, upper=2.0),
    )
    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.20, lower=0.0, upper=INSERT_TRAVEL),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    insert_slide = object_model.get_articulation("insert_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.check("lid articulation is revolute", _joint_kind(lid_hinge).endswith("revolute"), details=str(lid_hinge.articulation_type))
    ctx.check("insert articulation is prismatic", _joint_kind(insert_slide).endswith("prismatic"), details=str(insert_slide.articulation_type))
    ctx.check("wheel articulation is continuous", _joint_kind(wheel_spin).endswith("continuous"), details=str(wheel_spin.articulation_type))
    ctx.allow_overlap(
        case,
        insert,
        elem_a="case_shell",
        elem_b="insert_body",
        reason="The removable insert is intentionally modeled as a close sliding fit seated inside the thin lower shell at the rest pose.",
    )

    lid_limits = lid_hinge.motion_limits
    insert_limits = insert_slide.motion_limits

    if lid_limits is not None and lid_limits.lower is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.lower}):
            ctx.expect_overlap(
                lid,
                case,
                axes="xy",
                elem_a="lid_shell",
                elem_b="case_shell",
                min_overlap=0.010,
                name="closed lid covers the case opening",
            )
            lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

        with ctx.pose({lid_hinge: lid_limits.upper}):
            lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

        ctx.check(
            "lid swings upward from the side hinge",
            lid_closed_aabb is not None
            and lid_open_aabb is not None
            and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.010,
            details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
        )

    if insert_limits is not None and insert_limits.lower is not None and insert_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper if lid_limits is not None and lid_limits.upper is not None else 1.8}):
            with ctx.pose({insert_slide: insert_limits.lower}):
                ctx.expect_within(
                    insert,
                    case,
                    axes="xy",
                    inner_elem="insert_body",
                    outer_elem="case_shell",
                    margin=0.0012,
                    name="insert stays centered in the case at rest",
                )
                ctx.expect_overlap(
                    insert,
                    case,
                    axes="z",
                    elem_a="insert_body",
                    elem_b="case_shell",
                    min_overlap=0.028,
                    name="insert remains deeply seated at rest",
                )
                insert_rest = ctx.part_world_position(insert)

            with ctx.pose({insert_slide: insert_limits.upper}):
                ctx.expect_within(
                    insert,
                    case,
                    axes="xy",
                    inner_elem="insert_body",
                    outer_elem="case_shell",
                    margin=0.0012,
                    name="raised insert stays aligned with the case",
                )
                ctx.expect_overlap(
                    insert,
                    case,
                    axes="z",
                    elem_a="insert_body",
                    elem_b="case_shell",
                    min_overlap=0.014,
                    name="raised insert still remains retained in the shell",
                )
                insert_raised = ctx.part_world_position(insert)

        ctx.check(
            "insert lifts upward out of the lower shell",
            insert_rest is not None and insert_raised is not None and insert_raised[2] > insert_rest[2] + 0.012,
            details=f"rest={insert_rest}, raised={insert_raised}",
        )

    wheel_rest = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: 1.7}):
        wheel_spun = ctx.part_world_position(wheel)
    ctx.check(
        "wheel stays on its short axle while spinning",
        wheel_rest is not None
        and wheel_spun is not None
        and max(abs(a - b) for a, b in zip(wheel_rest, wheel_spun)) < 1e-6,
        details=f"rest={wheel_rest}, spun={wheel_spun}",
    )

    return ctx.report()


object_model = build_object_model()
