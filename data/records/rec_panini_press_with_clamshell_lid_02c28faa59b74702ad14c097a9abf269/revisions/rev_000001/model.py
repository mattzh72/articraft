from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    superellipse_side_loft,
)
import cadquery as cq


def _cast_base_shell():
    """One connected cast lower frame with a raised perimeter lip."""
    body = cq.Workplane("XY").box(0.320, 0.340, 0.040).translate((0.0, 0.0, 0.020))
    try:
        body = body.edges().fillet(0.010)
    except Exception:
        pass

    front_lip = cq.Workplane("XY").box(0.034, 0.300, 0.026).translate((0.143, 0.0, 0.053))
    rear_lip = cq.Workplane("XY").box(0.026, 0.300, 0.022).translate((-0.143, 0.0, 0.051))
    side_lip_0 = cq.Workplane("XY").box(0.245, 0.030, 0.022).translate((0.0, -0.155, 0.051))
    side_lip_1 = cq.Workplane("XY").box(0.245, 0.030, 0.022).translate((0.0, 0.155, 0.051))

    return body.union(front_lip).union(rear_lip).union(side_lip_0).union(side_lip_1)


def _upper_lid_shell() -> MeshGeometry:
    """Smooth pillow-like upper heated lid, local frame at the hinge line."""
    sections = [
        (-0.165, -0.010, 0.030, 0.235),
        (-0.120, -0.014, 0.047, 0.282),
        (-0.050, -0.014, 0.056, 0.292),
        (0.050, -0.014, 0.056, 0.292),
        (0.120, -0.014, 0.047, 0.282),
        (0.165, -0.010, 0.030, 0.235),
    ]
    shell = superellipse_side_loft(sections, exponents=3.2, segments=72, cap=True)
    shell.translate(0.165, 0.0, 0.0)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_sandwich_clamshell_press")

    cast = model.material("cast_charcoal", rgba=(0.075, 0.074, 0.068, 1.0))
    nonstick = model.material("seasoned_nonstick", rgba=(0.015, 0.016, 0.015, 1.0))
    warm_metal = model.material("warm_brushed_metal", rgba=(0.70, 0.68, 0.60, 1.0))
    stainless = model.material("spring_stainless", rgba=(0.78, 0.76, 0.70, 1.0))
    black = model.material("black_bakelite", rgba=(0.02, 0.019, 0.017, 1.0))
    white = model.material("white_indicator", rgba=(0.92, 0.90, 0.84, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        mesh_from_cadquery(_cast_base_shell(), "cast_lower_frame", tolerance=0.0015),
        material=cast,
        name="cast_lower_frame",
    )

    # Two distinct non-stick sandwich pockets, separated by a cast divider.
    for index, y, plate_name in (
        (0, -0.073, "lower_plate_0"),
        (1, 0.073, "lower_plate_1"),
    ):
        lower_frame.visual(
            Box((0.184, 0.098, 0.010)),
            origin=Origin(xyz=(-0.006, y, 0.044)),
            material=cast,
            name=f"plate_support_{index}",
        )
        lower_frame.visual(
            Box((0.214, 0.124, 0.006)),
            origin=Origin(xyz=(-0.006, y, 0.052)),
            material=nonstick,
            name=plate_name,
        )
        lower_frame.visual(
            Box((0.198, 0.008, 0.005)),
            origin=Origin(xyz=(-0.006, y, 0.0575), rpy=(0.0, 0.0, 0.47)),
            material=warm_metal,
            name=f"lower_diagonal_ridge_{index}",
        )
        lower_frame.visual(
            Box((0.198, 0.006, 0.004)),
            origin=Origin(xyz=(-0.006, y, 0.058), rpy=(0.0, 0.0, -0.47)),
            material=warm_metal,
            name=f"lower_faint_ridge_{index}",
        )

    lower_frame.visual(
        Box((0.224, 0.014, 0.016)),
        origin=Origin(xyz=(-0.006, 0.0, 0.048)),
        material=cast,
        name="center_divider",
    )

    # Rear hinge support: two fixed outer knuckles and cast ears on the frame.
    for index, y in enumerate((-0.122, 0.122)):
        lower_frame.visual(
            Box((0.030, 0.060, 0.038)),
            origin=Origin(xyz=(-0.153, y, 0.062)),
            material=cast,
            name=f"rear_hinge_ear_{index}",
        )
        lower_frame.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(-0.160, y, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"rear_hinge_knuckle_{index}",
        )

    # Front latch hinge ears on the base lip.
    for index, y in enumerate((-0.044, 0.044)):
        lower_frame.visual(
            Box((0.018, 0.022, 0.018)),
            origin=Origin(xyz=(0.162, y, 0.040)),
            material=cast,
            name=f"latch_hinge_ear_{index}",
        )
        lower_frame.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(xyz=(0.168, y, 0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"latch_hinge_knuckle_{index}",
        )

    # Right-front rotary-control socket.
    lower_frame.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.105, -0.176, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast,
        name="knob_socket",
    )

    upper_lid = model.part("upper_lid")
    upper_lid.visual(
        mesh_from_geometry(_upper_lid_shell(), "smooth_upper_lid"),
        material=warm_metal,
        name="smooth_upper_lid",
    )
    for index, y, plate_name in (
        (0, -0.073, "upper_plate_0"),
        (1, 0.073, "upper_plate_1"),
    ):
        upper_lid.visual(
            Box((0.214, 0.124, 0.006)),
            origin=Origin(xyz=(0.160, y, -0.015)),
            material=nonstick,
            name=plate_name,
        )
        upper_lid.visual(
            Box((0.190, 0.006, 0.004)),
            origin=Origin(xyz=(0.160, y, -0.019), rpy=(0.0, 0.0, 0.47)),
            material=warm_metal,
            name=f"upper_diagonal_ridge_{index}",
        )

    upper_lid.visual(
        Cylinder(radius=0.009, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_hinge_barrel",
    )
    upper_lid.visual(
        Box((0.046, 0.132, 0.007)),
        origin=Origin(xyz=(0.028, 0.0, -0.001)),
        material=stainless,
        name="rear_hinge_leaf",
    )
    upper_lid.visual(
        Box((0.014, 0.070, 0.018)),
        origin=Origin(xyz=(0.316, 0.0, 0.030)),
        material=stainless,
        name="front_latch_catch",
    )

    latch_strap = model.part("latch_strap")
    latch_strap.visual(
        Cylinder(radius=0.0052, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="latch_hinge_barrel",
    )
    latch_strap.visual(
        Box((0.008, 0.052, 0.088)),
        origin=Origin(xyz=(-0.004, 0.0, 0.043)),
        material=stainless,
        name="strap_plate",
    )
    latch_strap.visual(
        Box((0.012, 0.058, 0.012)),
        origin=Origin(xyz=(0.001, 0.0, 0.077)),
        material=stainless,
        name="catch_hook",
    )
    latch_strap.visual(
        Box((0.018, 0.046, 0.010)),
        origin=Origin(xyz=(0.003, 0.0, 0.020)),
        material=stainless,
        name="lower_reinforcement",
    )

    browning_knob = model.part("browning_knob")
    knob_mesh = KnobGeometry(
        0.040,
        0.022,
        body_style="skirted",
        top_diameter=0.032,
        edge_radius=0.001,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
    )
    browning_knob.visual(
        mesh_from_geometry(knob_mesh, "browning_knob_cap"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    browning_knob.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_shaft",
    )
    browning_knob.visual(
        Box((0.004, 0.0012, 0.018)),
        origin=Origin(xyz=(0.0, -0.0118, 0.006)),
        material=white,
        name="pointer_mark",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=upper_lid,
        origin=Origin(xyz=(-0.160, 0.0, 0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.20),
    )

    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=latch_strap,
        origin=Origin(xyz=(0.168, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=1.35),
    )

    model.articulation(
        "browning_axis",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=browning_knob,
        origin=Origin(xyz=(0.105, -0.202, 0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_frame")
    lid = object_model.get_part("upper_lid")
    latch = object_model.get_part("latch_strap")
    knob = object_model.get_part("browning_knob")
    rear_hinge = object_model.get_articulation("rear_hinge")
    latch_hinge = object_model.get_articulation("latch_hinge")
    browning_axis = object_model.get_articulation("browning_axis")

    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="smooth_upper_lid",
        elem_b="cast_lower_frame",
        min_overlap=0.22,
        name="closed lid covers cast frame footprint",
    )
    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        positive_elem="upper_plate_0",
        negative_elem="lower_plate_0",
        min_gap=0.004,
        max_gap=0.012,
        name="closed plates leave a shallow sandwich gap",
    )
    ctx.expect_contact(
        knob,
        lower,
        elem_a="knob_shaft",
        elem_b="knob_socket",
        contact_tol=0.0015,
        name="browning knob shaft seats in side socket",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="smooth_upper_lid")
    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="strap_plate")
    with ctx.pose({rear_hinge: 1.0, latch_hinge: 1.0, browning_axis: math.pi}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="smooth_upper_lid")
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="strap_plate")

    ctx.check(
        "rear hinge lifts upper lid",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.060,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "latch strap swings forward to unlatch",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][0] > closed_latch_aabb[1][0] + 0.035,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
