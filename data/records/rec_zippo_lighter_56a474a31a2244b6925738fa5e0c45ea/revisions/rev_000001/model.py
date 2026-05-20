from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    mesh_from_cadquery,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CASE_W = 0.038
CASE_D = 0.017
BODY_H = 0.042
CAP_H = 0.018
WALL = 0.0012
HINGE_R = 0.0018
OPEN_ANGLE = 1.85


def _rot_xy(x: float, y: float, angle: float = OPEN_ANGLE) -> tuple[float, float]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x * ca - y * sa, x * sa + y * ca)


def _open_origin(x: float, y: float, z: float) -> Origin:
    ox, oy = _rot_xy(x, y)
    return Origin(xyz=(ox, oy, z), rpy=(0.0, 0.0, OPEN_ANGLE))


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder_z(radius: float, height: float, center: tuple[float, float, float]):
    return cq.Workplane("XY").cylinder(height, radius).translate(center)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windproof_lighter")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.73, 0.70, 1.0))
    polished = model.material("polished_edges", rgba=(0.90, 0.90, 0.86, 1.0))
    brass = model.material("warm_brass", rgba=(0.75, 0.55, 0.24, 1.0))
    dark = model.material("dark_hollows", rgba=(0.015, 0.014, 0.012, 1.0))
    wheel_metal = model.material("dark_knurled_steel", rgba=(0.18, 0.18, 0.17, 1.0))

    case = model.part("case")
    # Connected open-top rectangular shell with its side hinge leaf and barrels.
    hinge_x = -CASE_W / 2.0 - HINGE_R
    case_shape = _cq_box((CASE_W, CASE_D, WALL), (0.0, 0.0, WALL / 2.0))
    for shape in (
        _cq_box((CASE_W, WALL, BODY_H), (0.0, -CASE_D / 2.0 + WALL / 2.0, BODY_H / 2.0)),
        _cq_box((CASE_W, WALL, BODY_H), (0.0, CASE_D / 2.0 - WALL / 2.0, BODY_H / 2.0)),
        _cq_box((WALL, CASE_D, BODY_H), (-CASE_W / 2.0 + WALL / 2.0, 0.0, BODY_H / 2.0)),
        _cq_box((WALL, CASE_D, BODY_H), (CASE_W / 2.0 - WALL / 2.0, 0.0, BODY_H / 2.0)),
        _cq_box((HINGE_R + WALL, 0.0060, BODY_H - 0.006), (-CASE_W / 2.0 - HINGE_R / 2.0, 0.0, BODY_H / 2.0)),
        _cq_cylinder_z(HINGE_R, 0.012, (hinge_x, 0.0, 0.014)),
        _cq_cylinder_z(HINGE_R, 0.012, (hinge_x, 0.0, 0.032)),
    ):
        case_shape = case_shape.union(shape)
    case.visual(
        mesh_from_cadquery(case_shape, "case_shell", tolerance=0.00045),
        material=chrome,
        name="case_shell",
    )

    # Fixed insert, perforated wind chimney, and wheel yoke.
    insert = model.part("insert")
    insert_z = BODY_H + 0.0017
    ch_w = 0.020
    ch_d = 0.006
    ch_h = 0.0125
    ch_center_y = 0.0025
    ch_z0 = BODY_H + 0.00335
    ch_zc = ch_z0 + ch_h / 2.0
    front_y = ch_center_y - ch_d / 2.0
    rear_y = ch_center_y + ch_d / 2.0
    wheel_y = -0.0040
    wheel_z = BODY_H + 0.0091
    wheel_thick = 0.0055
    insert_shape = _cq_box((0.032, CASE_D - 0.001, 0.0034), (0.0, 0.0, insert_z))
    for shape in (
        _cq_box((ch_w, WALL, ch_h), (0.0, front_y, ch_zc)),
        _cq_box((ch_w, WALL, ch_h), (0.0, rear_y, ch_zc)),
        _cq_box((WALL, ch_d, ch_h), (-ch_w / 2.0 + WALL / 2.0, ch_center_y, ch_zc)),
        _cq_box((WALL, ch_d, ch_h), (ch_w / 2.0 - WALL / 2.0, ch_center_y, ch_zc)),
        _cq_box((0.0010, 0.0026, 0.0072), (-wheel_thick / 2.0 - 0.00075, wheel_y, BODY_H + 0.0053)),
        _cq_box((0.0010, 0.0026, 0.0072), (wheel_thick / 2.0 + 0.00075, wheel_y, BODY_H + 0.0053)),
    ):
        insert_shape = insert_shape.union(shape)
    hole_xs = (-0.006, 0.0, 0.006)
    hole_zs = (ch_z0 + 0.0028, ch_z0 + 0.0060, ch_z0 + 0.0092)
    for z in hole_zs:
        for x in hole_xs:
            front_cutter = (
                cq.Workplane("XY")
                .cylinder(WALL * 3.0, 0.00075)
                .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
                .translate((x, front_y, z))
            )
            rear_cutter = (
                cq.Workplane("XY")
                .cylinder(WALL * 3.0, 0.00075)
                .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
                .translate((x, rear_y, z))
            )
            insert_shape = insert_shape.cut(front_cutter).cut(rear_cutter)
    insert.visual(
        mesh_from_cadquery(insert_shape, "insert_body", tolerance=0.00035),
        material=polished,
        name="insert_body",
    )

    wick = model.part("wick")
    wick.visual(
        Cylinder(radius=0.0009, length=0.009),
        origin=Origin(),
        material=dark,
        name="wick_tip",
    )

    cap = model.part("cap")
    hinge_to_closed_center = HINGE_R + CASE_W / 2.0
    cap.visual(
        Box((CASE_W, WALL, CAP_H)),
        origin=_open_origin(hinge_to_closed_center, -CASE_D / 2.0 + WALL / 2.0, 0.0),
        material=chrome,
        name="cap_front",
    )
    cap.visual(
        Box((CASE_W, WALL, CAP_H)),
        origin=_open_origin(hinge_to_closed_center, CASE_D / 2.0 - WALL / 2.0, 0.0),
        material=chrome,
        name="cap_back",
    )
    cap.visual(
        Box((WALL, CASE_D, CAP_H)),
        origin=_open_origin(HINGE_R + WALL / 2.0, 0.0, 0.0),
        material=chrome,
        name="cap_hinge_side",
    )
    cap.visual(
        Box((WALL, CASE_D, CAP_H)),
        origin=_open_origin(HINGE_R + CASE_W - WALL / 2.0, 0.0, 0.0),
        material=chrome,
        name="cap_free_side",
    )
    cap.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=_open_origin(hinge_to_closed_center, 0.0, CAP_H / 2.0 - WALL / 2.0),
        material=chrome,
        name="cap_roof",
    )
    cap.visual(
        Box((HINGE_R + WALL, 0.0060, CAP_H * 0.72)),
        origin=_open_origin(HINGE_R / 2.0, 0.0, 0.0),
        material=chrome,
        name="cap_hinge_leaf",
    )
    cap.visual(
        Cylinder(radius=HINGE_R, length=CAP_H * 0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=polished,
        name="cap_hinge_barrel",
    )

    wheel = model.part("thumb_wheel")
    wheel.visual(
        Cylinder(radius=0.0027, length=wheel_thick),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_metal,
        name="wheel_rim",
    )
    wheel.visual(
        Cylinder(radius=0.00065, length=0.0064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="wheel_axle",
    )
    tooth_count = 20
    for idx in range(tooth_count):
        theta = 2.0 * math.pi * idx / tooth_count
        r_center = 0.0030
        wheel.visual(
            Box((0.0052, 0.00042, 0.00080)),
            origin=Origin(
                xyz=(0.0, -r_center * math.sin(theta), r_center * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=wheel_metal,
            name=f"tooth_{idx}",
        )

    model.articulation(
        "case_to_cap",
        ArticulationType.REVOLUTE,
        parent=case,
        child=cap,
        origin=Origin(xyz=(hinge_x, 0.0, BODY_H + CAP_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-OPEN_ANGLE, upper=0.0),
    )
    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "insert_to_wick",
        ArticulationType.FIXED,
        parent=insert,
        child=wick,
        origin=Origin(xyz=(0.0, ch_center_y - 0.001, insert_z + 0.0017 + 0.0045)),
    )
    model.articulation(
        "case_to_thumb_wheel",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=wheel,
        origin=Origin(xyz=(0.0, wheel_y, wheel_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=20.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    cap = object_model.get_part("cap")
    wheel = object_model.get_part("thumb_wheel")
    cap_joint = object_model.get_articulation("case_to_cap")
    wheel_joint = object_model.get_articulation("case_to_thumb_wheel")

    ctx.check(
        "cap has a single side hinge revolute joint",
        cap_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in cap_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={cap_joint.articulation_type}, axis={cap_joint.axis}",
    )
    ctx.check(
        "thumb wheel is continuous on a transverse shaft",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    # The authored rest pose leaves the cap flipped open so the chimney and
    # wheel are visible. At the lower limit it closes onto the rectangular case.
    with ctx.pose({cap_joint: -OPEN_ANGLE}):
        ctx.expect_gap(
            cap,
            case,
            axis="z",
            positive_elem="cap_front",
            negative_elem="case_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed cap front edge seats on the case rim",
        )
        ctx.expect_overlap(
            cap,
            case,
            axes="xy",
            elem_a="cap_roof",
            elem_b="case_shell",
            min_overlap=0.012,
            name="closed cap roof covers the case footprint",
        )
        ctx.expect_gap(
            cap,
            insert,
            axis="z",
            positive_elem="cap_roof",
            negative_elem="insert_body",
            min_gap=0.0005,
            max_gap=0.004,
            name="closed cap roof clears the wind chimney",
        )

    with ctx.pose({wheel_joint: math.pi}):
        ctx.expect_overlap(
            wheel,
            insert,
            axes="yz",
            elem_a="wheel_rim",
            elem_b="insert_body",
            min_overlap=0.001,
            name="wheel remains beside the chimney opening while spinning",
        )

    return ctx.report()


object_model = build_object_model()
