from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.080
BODY_D = 0.130
BODY_H = 0.075
FRONT_Y = -BODY_D / 2.0
SIDE_X = BODY_W / 2.0
ENTRY_Z = 0.041
DRAWER_TRAVEL = 0.052


def _make_body_shell() -> cq.Workplane:
    """Cast-metal housing with true cut-through pencil and drawer cavities."""

    body = cq.Workplane("XY").box(
        BODY_W, BODY_D, BODY_H, centered=(True, True, False)
    )

    # Rounded cast edges; fall back to a clean box if a CAD kernel fillet fails.
    try:
        body = body.edges("|Z").fillet(0.006)
        body = body.edges(">Z").fillet(0.003)
    except Exception:
        body = cq.Workplane("XY").box(
            BODY_W, BODY_D, BODY_H, centered=(True, True, False)
        )

    # Rectangular shavings drawer pocket, larger than the moving tray so the
    # drawer is visibly nested in a real lower-body opening.
    drawer_slot = (
        cq.Workplane("XY")
        .box(0.070, 0.108, 0.029, centered=(True, True, True))
        .translate((0.0, FRONT_Y - 0.004 + 0.108 / 2.0, 0.0185))
    )

    # A circular pencil entry that opens into a broader internal cone.  The
    # cone is intentionally larger toward the rear so the front hole reads as a
    # real passage into a sharpening chamber rather than a shallow dimple.
    entry_tunnel = (
        cq.Workplane("XZ")
        .center(0.0, ENTRY_Z)
        .circle(0.0115)
        .extrude(BODY_D + 0.040, both=True)
    )
    try:
        sharpening_cavity = cq.Workplane(
            cq.Solid.makeCone(
                0.012,
                0.026,
                0.090,
                cq.Vector(0.0, FRONT_Y - 0.006, ENTRY_Z),
                cq.Vector(0.0, 1.0, 0.0),
            )
        )
    except Exception:
        sharpening_cavity = (
            cq.Workplane("XZ")
            .center(0.0, ENTRY_Z)
            .circle(0.023)
            .extrude(0.095, both=True)
        )

    # A small rear relief keeps the interior visibly open behind the cone.
    rear_relief = (
        cq.Workplane("XY")
        .box(0.048, 0.038, 0.034, centered=(True, True, True))
        .translate((0.0, 0.028, ENTRY_Z))
    )

    return body.cut(drawer_slot).cut(entry_tunnel).cut(sharpening_cavity).cut(rear_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_desktop_pencil_sharpener")

    cast_metal = model.material("cast_metal", rgba=(0.45, 0.48, 0.47, 1.0))
    darker_metal = model.material("darkened_metal", rgba=(0.18, 0.19, 0.19, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.012, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    cream_mark = model.material("cream_mark", rgba=(0.93, 0.87, 0.68, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "body_shell", tolerance=0.0006),
        material=cast_metal,
        name="body_shell",
    )
    # Bearing boss for the side crank, proud of the right side face.
    body.visual(
        Cylinder(radius=0.014, length=0.013),
        origin=Origin(xyz=(SIDE_X + 0.0065, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_metal,
        name="side_boss",
    )
    # Rounded metal ring emphasizes the true hollow pencil entry.
    body.visual(
        mesh_from_geometry(TorusGeometry(0.0140, 0.0025, radial_segments=32), "entry_bezel"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0010, ENTRY_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="entry_bezel",
    )
    # Recess shadow just inside the opening; the center remains open.
    body.visual(
        mesh_from_geometry(TorusGeometry(0.0200, 0.0014, radial_segments=32), "cavity_shadow_ring"),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.018, ENTRY_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_metal,
        name="cavity_shadow_ring",
    )
    # Front screw heads and rubber feet are mounted to the cast body.
    for i, (x, z) in enumerate(((-0.030, 0.061), (0.030, 0.061))):
        body.visual(
            Cylinder(radius=0.0036, length=0.0022),
            origin=Origin(xyz=(x, FRONT_Y - 0.0011, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_steel,
            name=f"front_screw_{i}",
        )
    for i, (x, y) in enumerate(
        ((-0.028, -0.044), (0.028, -0.044), (-0.028, 0.044), (0.028, 0.044))
    ):
        body.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, y, -0.002), rpy=(0.0, 0.0, 0.0)),
            material=rubber,
            name=f"foot_{i}",
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.068, 0.006, 0.027)),
        origin=Origin(xyz=(0.0, -0.003, 0.0185)),
        material=black_plastic,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.058, 0.094, 0.003)),
        origin=Origin(xyz=(0.0, 0.045, 0.008)),
        material=darker_metal,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.003, 0.094, 0.018)),
        origin=Origin(xyz=(-0.0305, 0.045, 0.017)),
        material=darker_metal,
        name="drawer_wall_0",
    )
    drawer.visual(
        Box((0.003, 0.094, 0.018)),
        origin=Origin(xyz=(0.0305, 0.045, 0.017)),
        material=darker_metal,
        name="drawer_wall_1",
    )
    drawer.visual(
        Box((0.058, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.090, 0.017)),
        material=darker_metal,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.028, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.008, 0.019)),
        material=rubber,
        name="drawer_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="crank_hub",
    )
    crank.visual(
        Box((0.030, 0.010, 0.012)),
        origin=Origin(xyz=(0.019, 0.0, -0.006)),
        material=polished_steel,
        name="crank_web",
    )
    crank.visual(
        Box((0.006, 0.010, 0.052)),
        origin=Origin(xyz=(0.026, 0.0, -0.026)),
        material=polished_steel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.0025, length=0.038),
        origin=Origin(xyz=(0.042, 0.0, -0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="spinner_axle",
    )
    crank.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.034, 0.0, -0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="spinner_washer",
    )

    spinner = model.part("spinner")
    spinner.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.018,
                0.032,
                body_style="hourglass",
                grip=KnobGrip(style="ribbed", count=10, depth=0.0007),
                bore=KnobBore(style="round", diameter=0.006),
            ),
            "spinner_knob",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="spinner_knob",
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.021,
                0.006,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=12, depth=0.00035),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                bore=KnobBore(style="round", diameter=0.003),
            ),
            "selector_dial",
        ),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=darker_metal,
        name="selector_dial",
    )
    selector.visual(
        Box((0.003, 0.0012, 0.009)),
        origin=Origin(xyz=(0.0, -0.0067, 0.004)),
        material=cream_mark,
        name="selector_mark",
    )

    drawer_joint = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, FRONT_Y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.25, lower=0.0, upper=DRAWER_TRAVEL),
    )
    crank_joint = model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(SIDE_X + 0.013, 0.0, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    spinner_joint = model.articulation(
        "crank_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=spinner,
        origin=Origin(xyz=(0.052, 0.0, -0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )
    selector_joint = model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0002, 0.063)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0),
    )

    # Keep named local variables visible to linters and readers; the model owns
    # the returned articulation objects.
    _ = (drawer_joint, crank_joint, spinner_joint, selector_joint)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    spinner = object_model.get_part("spinner")
    selector = object_model.get_part("selector")

    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    spinner_joint = object_model.get_articulation("crank_to_spinner")
    selector_joint = object_model.get_articulation("body_to_selector")

    ctx.check(
        "primary mechanisms are articulated",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and spinner_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"drawer={drawer_joint.articulation_type}, crank={crank_joint.articulation_type}, "
            f"spinner={spinner_joint.articulation_type}, selector={selector_joint.articulation_type}"
        ),
    )

    ctx.expect_gap(
        body,
        drawer,
        axis="y",
        positive_elem="body_shell",
        negative_elem="drawer_face",
        max_gap=0.0015,
        max_penetration=0.0,
        name="drawer front seats flush with housing front",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        inner_elem="drawer_floor",
        outer_elem="body_shell",
        margin=0.004,
        name="drawer tray fits inside lower cavity",
    )
    ctx.expect_contact(
        crank,
        spinner,
        elem_a="spinner_axle",
        elem_b="spinner_knob",
        contact_tol=0.004,
        name="spinner is carried by crank axle",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_floor",
            elem_b="body_shell",
            min_overlap=0.020,
            name="extended drawer remains retained in housing",
        )

    ctx.check(
        "drawer slides out from the front",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.040,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    ctx.expect_contact(
        selector,
        body,
        elem_a="selector_dial",
        elem_b="body_shell",
        contact_tol=0.0025,
        name="selector dial is mounted on front face above entry",
    )

    return ctx.report()


object_model = build_object_model()
