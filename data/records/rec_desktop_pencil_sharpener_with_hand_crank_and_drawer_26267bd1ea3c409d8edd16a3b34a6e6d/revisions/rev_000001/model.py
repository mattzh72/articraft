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
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_X = 0.160
BODY_Y = 0.110
BODY_Z = 0.105
FRONT_X = -BODY_X / 2.0
SIDE_Y = BODY_Y / 2.0
PORT_X = -0.043
PORT_Z = 0.073
CRANK_X = 0.030
CRANK_Z = 0.062


def _sharpener_body_shape() -> cq.Workplane:
    """One continuous boxy shell with real drawer, pencil-port, and axle openings."""
    body = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_Z, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.118, 0.090, 0.039)
        .translate((FRONT_X + 0.118 / 2.0 - 0.003, 0.0, 0.010 + 0.039 / 2.0))
    )
    body = body.cut(drawer_cavity)

    pencil_port = (
        cq.Workplane("XY")
        .cylinder(0.050, 0.0175)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((FRONT_X + 0.010, 0.0, PORT_Z))
    )
    body = body.cut(pencil_port)

    crank_axle_hole = (
        cq.Workplane("XY")
        .cylinder(0.040, 0.011)
        .rotate((0, 0, 0), (1, 0, 0), -90)
        .translate((CRANK_X, SIDE_Y, CRANK_Z))
    )
    body = body.cut(crank_axle_hole)

    return body


def _port_bezel_shape() -> cq.Workplane:
    """A raised annular metal lip around the front pencil entry."""
    return (
        cq.Workplane("XY")
        .circle(0.0225)
        .circle(0.0160)
        .extrude(0.006)
        .rotate((0, 0, 0), (0, 1, 0), 90)
        .translate((FRONT_X - 0.004, 0.0, PORT_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_pencil_sharpener")

    red_enamel = model.material("red_enamel", rgba=(0.62, 0.05, 0.035, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.035, 0.038, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.70, 0.70, 0.66, 1.0))
    selector_gray = model.material("selector_gray", rgba=(0.28, 0.30, 0.31, 1.0))
    white_mark = model.material("white_mark", rgba=(0.94, 0.91, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_sharpener_body_shape(), "boxy_body", tolerance=0.0008),
        material=red_enamel,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_port_bezel_shape(), "pencil_port_bezel", tolerance=0.0005),
        material=brushed_metal,
        name="pencil_port_bezel",
    )
    for i, x_offset in enumerate((-0.024, -0.012, 0.0, 0.012, 0.024)):
        body.visual(
            Box((0.0025, 0.010, 0.0012)),
            origin=Origin(
                xyz=(PORT_X + x_offset, -0.027, BODY_Z + 0.0006),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=white_mark,
            name=f"size_tick_{i}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.008, 0.088, 0.034)),
        origin=Origin(xyz=(-0.005, 0.0, 0.027)),
        material=dark_plastic,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.083, 0.074, 0.004)),
        origin=Origin(xyz=(0.0395, 0.0, 0.012)),
        material=dark_plastic,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.083, 0.005, 0.022)),
        origin=Origin(xyz=(0.0395, 0.0395, 0.024)),
        material=dark_plastic,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.083, 0.005, 0.022)),
        origin=Origin(xyz=(0.0395, -0.0395, 0.024)),
        material=dark_plastic,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.006, 0.074, 0.024)),
        origin=Origin(xyz=(0.081, 0.0, 0.025)),
        material=dark_plastic,
        name="tray_back",
    )
    drawer.visual(
        Cylinder(radius=0.0055, length=0.050),
        origin=Origin(xyz=(-0.013, 0.0, 0.028), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="drawer_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="side_axle",
    )
    crank.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="axle_collar",
    )
    crank.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="crank_hub",
    )
    crank.visual(
        Box((0.010, 0.008, 0.052)),
        origin=Origin(xyz=(0.0, 0.034, -0.026)),
        material=brushed_metal,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(0.0, 0.047, -0.052), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="handle_pin",
    )
    crank.visual(
        Cylinder(radius=0.009, length=0.032),
        origin=Origin(xyz=(0.0, 0.066, -0.052), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="hand_grip",
    )

    size_knob = model.part("size_knob")
    size_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.016,
                body_style="faceted",
                base_diameter=0.040,
                top_diameter=0.032,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "size_selector_knob",
        ),
        material=selector_gray,
        name="knob_cap",
    )
    size_knob.visual(
        Box((0.005, 0.028, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0170)),
        material=white_mark,
        name="pointer_mark",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(FRONT_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.045),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(CRANK_X, SIDE_Y, CRANK_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )
    model.articulation(
        "body_to_size_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=size_knob,
        origin=Origin(xyz=(PORT_X, 0.0, BODY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    size_knob = object_model.get_part("size_knob")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    knob_joint = object_model.get_articulation("body_to_size_knob")

    ctx.check(
        "primary mechanisms are present",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected a sliding drawer plus continuous crank and size-selector knob.",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        inner_elem="tray_floor",
        outer_elem="body_shell",
        margin=0.004,
        name="drawer tray fits the front cavity cross-section",
    )
    ctx.expect_gap(
        body,
        drawer,
        axis="x",
        positive_elem="body_shell",
        negative_elem="drawer_front",
        max_penetration=0.0,
        name="drawer face sits just in front of the body",
    )
    ctx.expect_contact(
        crank,
        body,
        elem_a="axle_collar",
        elem_b="body_shell",
        contact_tol=0.0015,
        name="crank collar is mounted against the side body hole",
    )
    ctx.expect_contact(
        size_knob,
        body,
        elem_a="knob_cap",
        elem_b="body_shell",
        contact_tol=0.0015,
        name="size selector knob is seated on the top surface",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.045}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="body_shell",
            min_overlap=0.030,
            name="extended drawer remains retained in the body",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="tray_floor",
            outer_elem="body_shell",
            margin=0.004,
            name="extended drawer remains aligned with its cavity",
        )
    ctx.check(
        "drawer slides out from the front",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] < rest_drawer_pos[0] - 0.035,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(crank, elem="hand_grip")
    with ctx.pose({crank_joint: pi / 2.0}):
        rotated_handle_aabb = ctx.part_element_world_aabb(crank, elem="hand_grip")
    if rest_handle_aabb is not None and rotated_handle_aabb is not None:
        rest_center_z = (rest_handle_aabb[0][2] + rest_handle_aabb[1][2]) / 2.0
        rotated_center_x = (rotated_handle_aabb[0][0] + rotated_handle_aabb[1][0]) / 2.0
        rest_center_x = (rest_handle_aabb[0][0] + rest_handle_aabb[1][0]) / 2.0
    else:
        rest_center_z = rotated_center_x = rest_center_x = None
    ctx.check(
        "crank handle orbits its side axle",
        rest_center_z is not None
        and rest_center_z < CRANK_Z - 0.040
        and rotated_center_x is not None
        and rest_center_x is not None
        and rotated_center_x < rest_center_x - 0.040,
        details=f"rest_aabb={rest_handle_aabb}, rotated_aabb={rotated_handle_aabb}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(size_knob, elem="pointer_mark")
    with ctx.pose({knob_joint: pi / 2.0}):
        rotated_pointer_aabb = ctx.part_element_world_aabb(size_knob, elem="pointer_mark")
    if rest_pointer_aabb is not None and rotated_pointer_aabb is not None:
        rest_x = rest_pointer_aabb[1][0] - rest_pointer_aabb[0][0]
        rest_y = rest_pointer_aabb[1][1] - rest_pointer_aabb[0][1]
        rotated_x = rotated_pointer_aabb[1][0] - rotated_pointer_aabb[0][0]
        rotated_y = rotated_pointer_aabb[1][1] - rotated_pointer_aabb[0][1]
    else:
        rest_x = rest_y = rotated_x = rotated_y = None
    ctx.check(
        "size selector pointer rotates about the top axis",
        rest_x is not None
        and rest_y is not None
        and rotated_x is not None
        and rotated_y is not None
        and rest_y > rest_x * 3.0
        and rotated_x > rotated_y * 3.0,
        details=f"rest={rest_pointer_aabb}, rotated={rotated_pointer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
