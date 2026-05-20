from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.038
BODY_D = 0.018
BODY_H = 0.040
SHELL_T = 0.0015

CAP_W = 0.039
CAP_D = 0.0225
CAP_H = 0.022
HINGE_X = -0.0207
CAP_CENTER_Z = BODY_H + CAP_H / 2.0
CAP_CENTER_X_FROM_HINGE = -HINGE_X


def _centered_box(size, center):
    return cq.Workplane("XY").box(*size).translate(center)


def _y_axis_cutter(radius: float, length: float, center):
    """CadQuery cylinder centered on `center` with its axis along world Y."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate(center)
    )


def _lower_shell_shape():
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(0.0020)
        .translate((0.0, 0.0, BODY_H / 2.0))
    )
    cavity = _centered_box(
        (BODY_W - 2.0 * SHELL_T, BODY_D - 2.0 * SHELL_T, BODY_H + 0.004),
        (0.0, 0.0, SHELL_T + (BODY_H + 0.004) / 2.0),
    )
    shell = outer.cut(cavity)

    # Fine raised pressings on the polished case faces, integrated into the shell.
    for y in (-BODY_D / 2.0 - 0.00018, BODY_D / 2.0 + 0.00018):
        for z in (0.006, BODY_H - 0.004):
            shell = shell.union(
                _centered_box((BODY_W - 0.006, 0.00045, 0.00075), (0.0, y, z))
            )

    # A short lower hinge barrel, slightly buried into the side wall so it is mounted.
    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.0020)
        .extrude(0.027)
        .translate((HINGE_X, 0.0, 0.0125))
    )
    hinge_leaf = _centered_box((0.0048, 0.0055, 0.023), (HINGE_X + 0.0025, 0.0, 0.025))
    shell = shell.union(hinge_leaf).union(hinge_barrel)
    return shell


def _cap_shape():
    outer = (
        cq.Workplane("XY")
        .box(CAP_W, CAP_D, CAP_H)
        .edges("|Z")
        .fillet(0.0020)
        .translate((CAP_CENTER_X_FROM_HINGE, 0.0, 0.0))
    )
    cavity = _centered_box(
        (CAP_W - 2.0 * SHELL_T, CAP_D - 2.0 * SHELL_T, CAP_H),
        (CAP_CENTER_X_FROM_HINGE, 0.0, -SHELL_T),
    )
    cap = outer.cut(cavity)

    for y in (-CAP_D / 2.0 - 0.00018, CAP_D / 2.0 + 0.00018):
        cap = cap.union(
            _centered_box(
                (CAP_W - 0.006, 0.00045, 0.00075),
                (CAP_CENTER_X_FROM_HINGE, y, -CAP_H / 2.0 + 0.003),
            )
        )

    cap_hinge_leaf = _centered_box((0.0048, 0.0055, 0.017), (0.0025, 0.0, 0.001))
    cap_hinge_barrel = cq.Workplane("XY").circle(0.0020).extrude(CAP_H).translate(
        (0.0, 0.0, -CAP_H / 2.0)
    )
    cap = cap.union(cap_hinge_leaf).union(cap_hinge_barrel)
    return cap


def _insert_shape():
    metal = _centered_box((0.028, 0.010, 0.006), (0.0, 0.0, BODY_H - 0.0025))

    ch_w = 0.023
    ch_d = 0.0105
    ch_h = 0.017
    wall_t = 0.0008
    ch_center_z = BODY_H + ch_h / 2.0

    front = _centered_box((ch_w, wall_t, ch_h), (0.0, -ch_d / 2.0, ch_center_z))
    rear = _centered_box((ch_w, wall_t, ch_h), (0.0, ch_d / 2.0, ch_center_z))
    side_a = _centered_box((wall_t, ch_d, ch_h), (-ch_w / 2.0, 0.0, ch_center_z))
    side_b = _centered_box((wall_t, ch_d, ch_h), (ch_w / 2.0, 0.0, ch_center_z))
    top_front = _centered_box(
        (ch_w, wall_t, 0.0014), (0.0, -ch_d / 2.0, BODY_H + ch_h + 0.0007)
    )
    top_rear = _centered_box(
        (ch_w, wall_t, 0.0014), (0.0, ch_d / 2.0, BODY_H + ch_h + 0.0007)
    )

    metal = metal.union(front).union(rear).union(side_a).union(side_b).union(top_front).union(top_rear)

    # Round perforations through both broad chimney faces.
    for x in (-0.0075, 0.0, 0.0075):
        for z in (BODY_H + 0.0045, BODY_H + 0.0090, BODY_H + 0.0135):
            for y in (-ch_d / 2.0, ch_d / 2.0):
                metal = metal.cut(_y_axis_cutter(0.00115, 0.004, (x, y, z)))

    # Fork cheeks that cradle the thumb wheel axle in front of the wick opening.
    yoke_z = BODY_H + 0.008
    for y in (-0.00935, -0.00410):
        metal = metal.union(_centered_box((0.012, 0.0005, 0.012), (0.0, y, yoke_z)))
    metal = metal.union(_centered_box((0.013, 0.0060, 0.0020), (0.0, -0.00670, BODY_H + 0.0020)))
    for x in (-0.0064, 0.0064):
        metal = metal.union(_centered_box((0.0012, 0.0090, 0.0030), (x, -0.0048, BODY_H + 0.0020)))

    return metal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="polished_pocket_lighter")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.78, 0.74, 1.0))
    darker_steel = model.material("brushed_insert_steel", rgba=(0.58, 0.57, 0.54, 1.0))
    dark_groove = model.material("shadowed_knurl", rgba=(0.18, 0.17, 0.16, 1.0))
    wick_cotton = model.material("charred_cotton_wick", rgba=(0.74, 0.67, 0.55, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_lower_shell_shape(), "lower_shell", tolerance=0.00035),
        material=chrome,
        name="lower_shell",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_shape(), "perforated_insert", tolerance=0.00025),
        material=darker_steel,
        name="perforated_insert",
    )
    model.articulation(
        "shell_to_insert",
        ArticulationType.FIXED,
        parent=shell,
        child=insert,
        origin=Origin(),
    )

    wick = model.part("wick")
    wick.visual(
        Cylinder(radius=0.00115, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=wick_cotton,
        name="wick_fiber",
    )
    model.articulation(
        "insert_to_wick",
        ArticulationType.FIXED,
        parent=insert,
        child=wick,
        origin=Origin(xyz=(0.0035, 0.0, BODY_H + 0.0005)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_cap_shape(), "hinged_cap", tolerance=0.00035),
        material=chrome,
        name="cap_shell",
    )
    model.articulation(
        "shell_to_cap",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=cap,
        origin=Origin(xyz=(HINGE_X, 0.0, CAP_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=2.60),
    )

    thumb_wheel = model.part("thumb_wheel")
    thumb_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.0105,
                0.0044,
                body_style="cylindrical",
                edge_radius=0.00035,
                grip=KnobGrip(style="ribbed", count=28, depth=0.00055, width=0.00065),
            ),
            "ribbed_thumb_wheel",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_groove,
        name="ribbed_wheel",
    )
    thumb_wheel.visual(
        Cylinder(radius=0.0012, length=0.0076),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_steel,
        name="short_axle",
    )
    model.articulation(
        "insert_to_thumb_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=thumb_wheel,
        origin=Origin(xyz=(0.0, -0.0066, BODY_H + 0.0080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    insert = object_model.get_part("insert")
    cap = object_model.get_part("cap")
    wheel = object_model.get_part("thumb_wheel")
    cap_hinge = object_model.get_articulation("shell_to_cap")
    wheel_joint = object_model.get_articulation("insert_to_thumb_wheel")

    ctx.expect_gap(
        cap,
        shell,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed cap sits on the lower shell seam",
    )
    ctx.expect_within(
        insert,
        cap,
        axes="xy",
        margin=0.002,
        name="chimney insert fits inside closed cap cavity",
    )
    ctx.expect_overlap(
        cap,
        shell,
        axes="xy",
        min_overlap=0.012,
        name="closed cap aligns with boxy lower shell footprint",
    )

    ctx.check(
        "thumb wheel uses a continuous axle joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {wheel_joint.articulation_type}",
    )

    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "thumb wheel is mounted above the insert body",
        wheel_aabb is not None and wheel_aabb[0][2] > BODY_H + 0.002,
        details=f"wheel_aabb={wheel_aabb}",
    )

    closed_aabb = ctx.part_world_aabb(cap)
    closed_x = None
    closed_y = None
    if closed_aabb is not None:
        closed_x = (closed_aabb[0][0] + closed_aabb[1][0]) / 2.0
        closed_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    with ctx.pose({cap_hinge: 2.60}):
        ctx.expect_gap(
            insert,
            cap,
            axis="x",
            min_gap=0.001,
            name="fully open cap clears the chimney insert to the side",
        )
        open_aabb = ctx.part_world_aabb(cap)
        open_x = None
        open_y = None
        if open_aabb is not None:
            open_x = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
            open_y = (open_aabb[0][1] + open_aabb[1][1]) / 2.0
    ctx.check(
        "cap swings outward around the narrow side hinge",
        closed_x is not None
        and open_x is not None
        and closed_y is not None
        and open_y is not None
        and open_x < closed_x - 0.020
        and open_y > closed_y + 0.005,
        details=f"closed=({closed_x}, {closed_y}), open=({open_x}, {open_y})",
    )

    return ctx.report()


object_model = build_object_model()
