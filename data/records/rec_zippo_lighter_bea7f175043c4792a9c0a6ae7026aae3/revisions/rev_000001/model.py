from __future__ import annotations

from math import pi

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


BODY_W = 0.0372
BODY_D = 0.0127
BODY_H = 0.0408
CAP_H = 0.0168
WALL = 0.00055
SEAM_GAP = 0.00025
CORNER_R = 0.00135

HINGE_R = 0.00105
HINGE_AXIS_X = -(BODY_W / 2.0) - HINGE_R
HINGE_AXIS_Z = BODY_H
HINGE_BODY_LEN = 0.0042
HINGE_CAP_LEN = 0.0032
HINGE_END_GAP = 0.00105

INSERT_STEM_W = 0.0156
INSERT_STEM_D = 0.0076
INSERT_STEM_H = 0.0310
CHIMNEY_W = 0.0196
CHIMNEY_D = 0.0090
CHIMNEY_H = 0.0165
CHIMNEY_Z0 = 0.0313
CHIMNEY_WALL = 0.00045

WHEEL_X = 0.0037
WHEEL_Z = 0.0419
WHEEL_R = 0.0038
WHEEL_LEN = 0.0058
AXLE_R = 0.0011
AXLE_LEN = 0.0074

WICK_X = -0.0028
WICK_R = 0.0008
WICK_LEN = 0.0180
WICK_Z = 0.0407


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _box_centered(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _box_from_bottom(size: tuple[float, float, float], base_center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").transformed(offset=base_center).box(*size, centered=(True, True, False))


def _lower_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    outer = outer.edges("|Z").fillet(CORNER_R)
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WALL))
        .box(BODY_W - (2.0 * WALL), BODY_D - (2.0 * WALL), BODY_H - WALL + 0.002, centered=(True, True, False))
    )
    return outer.cut(inner)


def _body_hinge_shape() -> cq.Workplane:
    body_barrel = _cylinder_y(HINGE_R, HINGE_BODY_LEN, (HINGE_AXIS_X, 0.0, HINGE_AXIS_Z))
    body_tab = _box_centered(
        (HINGE_R * 2.0, HINGE_BODY_LEN, 0.0032),
        (-(BODY_W / 2.0) - (HINGE_R / 2.0), 0.0, BODY_H - 0.0012),
    )
    return body_barrel.union(body_tab)


def _insert_shape() -> cq.Workplane:
    stem = _box_from_bottom((INSERT_STEM_W, INSERT_STEM_D, INSERT_STEM_H), (0.0, 0.0, WALL))

    chimney_outer = _box_from_bottom((CHIMNEY_W, CHIMNEY_D, CHIMNEY_H), (0.0, 0.0, CHIMNEY_Z0))
    chimney_inner = _box_from_bottom(
        (
            CHIMNEY_W - (2.0 * CHIMNEY_WALL),
            CHIMNEY_D - (2.0 * CHIMNEY_WALL),
            CHIMNEY_H + 0.003,
        ),
        (0.0, 0.0, CHIMNEY_Z0 + CHIMNEY_WALL),
    )
    chimney = chimney_outer.cut(chimney_inner)

    for x_pos in (-0.0044, 0.0, 0.0044):
        for z_pos in (CHIMNEY_Z0 + 0.0048, CHIMNEY_Z0 + 0.0104):
            chimney = chimney.cut(_cylinder_y(0.0010, CHIMNEY_D + 0.004, (x_pos, 0.0, z_pos)))

    wick_cutter = (
        cq.Workplane("XY")
        .center(WICK_X, 0.0)
        .circle(0.00135)
        .extrude(0.010)
        .translate((0.0, 0.0, CHIMNEY_Z0 + CHIMNEY_H - 0.010))
    )
    chimney = chimney.cut(wick_cutter)

    front_bridge = _box_centered((0.0048, CHIMNEY_D, 0.0035), (0.0063, 0.0, 0.0365))
    return stem.union(chimney).union(front_bridge)


def _cap_shape() -> cq.Workplane:
    outer = _box_from_bottom((BODY_W, BODY_D, CAP_H), (HINGE_R + (BODY_W / 2.0), 0.0, SEAM_GAP))
    outer = outer.edges("|Z").fillet(CORNER_R)
    inner = _box_from_bottom(
        (BODY_W - (2.0 * WALL), BODY_D - (2.0 * WALL), CAP_H - WALL + 0.002),
        (HINGE_R + (BODY_W / 2.0), 0.0, -0.001),
    )
    return outer.cut(inner)


def _cap_hinge_shape() -> cq.Workplane:
    y_offset = (BODY_D / 2.0) - HINGE_END_GAP - (HINGE_CAP_LEN / 2.0)
    cap_barrel_a = _cylinder_y(HINGE_R, HINGE_CAP_LEN, (0.0, y_offset, 0.0))
    cap_barrel_b = _cylinder_y(HINGE_R, HINGE_CAP_LEN, (0.0, -y_offset, 0.0))
    cap_tab_a = _box_centered((HINGE_R * 2.0, HINGE_CAP_LEN, 0.0030), (HINGE_R / 2.0, y_offset, 0.0015))
    cap_tab_b = _box_centered((HINGE_R * 2.0, HINGE_CAP_LEN, 0.0030), (HINGE_R / 2.0, -y_offset, 0.0015))
    return cap_barrel_a.union(cap_barrel_b).union(cap_tab_a).union(cap_tab_b)


def _wheel_shape() -> cq.Workplane:
    wheel = _cylinder_y(WHEEL_R, WHEEL_LEN, (0.0, 0.0, 0.0))
    wheel = wheel.union(_cylinder_y(AXLE_R, AXLE_LEN, (0.0, 0.0, 0.0)))

    groove = _box_centered((0.0010, AXLE_LEN + 0.004, (WHEEL_R * 2.4)), (WHEEL_R - 0.00035, 0.0, 0.0))
    for angle_deg in range(0, 180, 12):
        wheel = wheel.cut(groove.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), float(angle_deg)))

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pocket_lighter")

    chrome = model.material("chrome", rgba=(0.80, 0.81, 0.84, 1.0))
    insert_metal = model.material("insert_metal", rgba=(0.66, 0.68, 0.71, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    wick_fiber = model.material("wick_fiber", rgba=(0.83, 0.78, 0.67, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_lower_shell_shape(), "body_shell"), material=chrome, name="body_shell")
    body.visual(mesh_from_cadquery(_body_hinge_shape(), "body_hinge"), material=chrome, name="body_hinge")
    body.visual(mesh_from_cadquery(_insert_shape(), "insert"), material=insert_metal, name="insert")
    body.visual(
        Cylinder(radius=WICK_R, length=WICK_LEN),
        origin=Origin(xyz=(WICK_X, 0.0, WICK_Z)),
        material=wick_fiber,
        name="wick",
    )

    cap = model.part("cap")
    cap.visual(mesh_from_cadquery(_cap_shape(), "cap_shell"), material=chrome, name="cap_shell")
    cap.visual(mesh_from_cadquery(_cap_hinge_shape(), "cap_hinge"), material=chrome, name="cap_hinge")

    wheel = model.part("wheel")
    wheel.visual(mesh_from_cadquery(_wheel_shape(), "thumb_wheel"), material=wheel_steel, name="wheel")
    wheel.visual(
        Cylinder(radius=AXLE_R, length=0.00035),
        origin=Origin(xyz=(0.0, 0.003875, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="axle_tip_0",
    )
    wheel.visual(
        Cylinder(radius=AXLE_R, length=0.00035),
        origin=Origin(xyz=(0.0, -0.003875, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="axle_tip_1",
    )

    cap_hinge = model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0, lower=0.0, upper=2.15),
    )
    cap_hinge.meta["qc_samples"] = [0.0, 0.7, 1.4, 2.15]

    model.articulation(
        "thumb_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    wheel = object_model.get_part("wheel")
    cap_hinge = object_model.get_articulation("cap_hinge")

    def elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if bounds is None:
            return None
        mins, maxs = bounds
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    closed_cap_pos = elem_center(cap, "cap_shell")

    ctx.expect_gap(
        cap,
        body,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="body_shell",
        min_gap=0.0001,
        max_gap=0.0008,
        name="cap seam stays tight when closed",
    )
    ctx.expect_overlap(
        cap,
        body,
        axes="xy",
        elem_a="cap_shell",
        elem_b="body_shell",
        min_overlap=0.010,
        name="closed cap covers the body footprint",
    )
    ctx.expect_within(
        wheel,
        body,
        axes="y",
        inner_elem="wheel",
        outer_elem="insert",
        margin=0.0007,
        name="thumb wheel stays between the chimney walls",
    )
    ctx.expect_origin_gap(
        wheel,
        body,
        axis="z",
        min_gap=0.039,
        max_gap=0.045,
        name="thumb wheel sits above the insert body",
    )

    limits = cap_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({cap_hinge: limits.upper}):
            opened_cap_pos = elem_center(cap, "cap_shell")

        ctx.check(
            "cap opens upward and rearward",
            closed_cap_pos is not None
            and opened_cap_pos is not None
            and opened_cap_pos[2] > closed_cap_pos[2] + 0.002
            and opened_cap_pos[0] < closed_cap_pos[0] - 0.010,
            details=f"closed={closed_cap_pos}, opened={opened_cap_pos}",
        )

    return ctx.report()


object_model = build_object_model()
