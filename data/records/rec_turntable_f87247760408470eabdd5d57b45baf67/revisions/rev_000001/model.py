from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    satin_black = Material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    warm_wood = Material("walnut_plinth", rgba=(0.43, 0.23, 0.11, 1.0))
    brushed_metal = Material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    vinyl_black = Material("black_vinyl", rgba=(0.001, 0.001, 0.001, 1.0))
    paper_label = Material("cream_label", rgba=(0.87, 0.77, 0.48, 1.0))
    cartridge_red = Material("red_cartridge", rgba=(0.55, 0.04, 0.03, 1.0))

    plinth = model.part("plinth")
    plinth_body = ExtrudeGeometry(
        rounded_rect_profile(0.72, 0.46, 0.055, corner_segments=10),
        0.070,
        cap=True,
        center=True,
    )
    plinth.visual(
        mesh_from_geometry(plinth_body, "plinth_body_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=warm_wood,
        name="plinth_body",
    )

    for i, (x, y) in enumerate(
        ((-0.285, -0.170), (0.285, -0.170), (-0.285, 0.170), (0.285, 0.170))
    ):
        plinth.visual(
            Cylinder(radius=0.032, length=0.022),
            origin=Origin(xyz=(x, y, 0.010)),
            material=dark_rubber,
            name=f"foot_{i}",
        )

    # Fixed guard hoop around the rotating platter.  Four short posts visibly
    # carry the ring, so the hoop reads as a mounted protective frame rather
    # than a floating wire.
    guard_hoop = TorusGeometry(
        0.188, 0.006, radial_segments=18, tubular_segments=96
    )
    plinth.visual(
        mesh_from_geometry(guard_hoop, "guard_hoop_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=brushed_metal,
        name="guard_hoop",
    )
    for i, (x, y) in enumerate(((0.188, 0.0), (-0.188, 0.0), (0.0, 0.188), (0.0, -0.188))):
        plinth.visual(
            Cylinder(radius=0.0055, length=0.024),
            origin=Origin(xyz=(x, y, 0.101)),
            material=brushed_metal,
            name=f"guard_post_{i}",
        )

    # Tonearm base fixed on the right-rear corner of the plinth.
    pivot_x, pivot_y, pivot_z = 0.245, 0.145, 0.163
    plinth.visual(
        Cylinder(radius=0.054, length=0.018),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.098)),
        material=satin_black,
        name="tonearm_base",
    )
    plinth.visual(
        Cylinder(radius=0.027, length=0.057),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.1345)),
        material=brushed_metal,
        name="pivot_column",
    )

    # A small fixed arm rest on the side of the base, mounted into the plinth.
    plinth.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.285, 0.070, 0.109)),
        material=satin_black,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.040, 0.014, 0.012)),
        origin=Origin(xyz=(0.266, 0.070, 0.136)),
        material=satin_black,
        name="arm_rest_cradle",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.160, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=vinyl_black,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.037, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=paper_label,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brushed_metal,
        name="center_spindle",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    tonearm = model.part("tonearm")
    arm_points = [
        (0.000, 0.000, 0.012),
        (-0.045, -0.004, 0.013),
        (-0.125, -0.045, 0.003),
        (-0.205, -0.112, -0.010),
    ]
    tonearm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                arm_points,
                radius=0.0055,
                samples_per_segment=14,
                closed_spline=False,
                radial_segments=16,
            ),
            "tonearm_tube_mesh",
        ),
        material=brushed_metal,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_metal,
        name="pivot_hub",
    )

    counter_theta = math.atan2(0.112, 0.205)
    tonearm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.012), (0.062, 0.034, 0.012)],
                radius=0.005,
                samples_per_segment=8,
                closed_spline=False,
                radial_segments=16,
            ),
            "counter_stub_mesh",
        ),
        material=brushed_metal,
        name="counter_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.023, length=0.054),
        origin=Origin(
            xyz=(0.055, 0.030, 0.012),
            rpy=(0.0, math.pi / 2.0, counter_theta),
        ),
        material=satin_black,
        name="counterweight",
    )

    head_theta = math.atan2(-0.112, -0.205)
    tonearm.visual(
        Box((0.046, 0.024, 0.010)),
        origin=Origin(xyz=(-0.205, -0.112, -0.010), rpy=(0.0, 0.0, head_theta)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.022, 0.014, 0.010)),
        origin=Origin(xyz=(-0.214, -0.117, -0.020), rpy=(0.0, 0.0, head_theta)),
        material=cartridge_red,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0018, length=0.005),
        origin=Origin(xyz=(-0.220, -0.121, -0.026)),
        material=brushed_metal,
        name="stylus",
    )

    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(pivot_x, pivot_y, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=1.5, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    spin = object_model.get_articulation("platter_spin")
    swing = object_model.get_articulation("tonearm_swing")

    ctx.check(
        "platter has continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "tonearm has limited vertical pivot",
        swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(swing.axis) == (0.0, 0.0, 1.0)
        and swing.motion_limits is not None
        and swing.motion_limits.lower < 0.0
        and swing.motion_limits.upper > 0.0,
        details=f"type={swing.articulation_type}, axis={swing.axis}, limits={swing.motion_limits}",
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="plinth_body",
        min_gap=0.004,
        max_gap=0.010,
        name="platter rides just above plinth",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="plinth_body",
        min_overlap=0.25,
        name="platter is centered on plinth footprint",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_hub",
        elem_b="pivot_column",
        contact_tol=0.002,
        name="tonearm hub is seated on base column",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="record_disc",
        min_gap=0.0002,
        max_gap=0.004,
        name="stylus hovers just above record surface",
    )

    guard = ctx.part_element_world_aabb(plinth, elem="guard_hoop")
    plate = ctx.part_element_world_aabb(platter, elem="platter_body")
    if guard is not None and plate is not None:
        guard_dx = guard[1][0] - guard[0][0]
        guard_dy = guard[1][1] - guard[0][1]
        plate_dx = plate[1][0] - plate[0][0]
        plate_dy = plate[1][1] - plate[0][1]
        ctx.check(
            "fixed guard hoop surrounds rotating stage",
            guard_dx > plate_dx + 0.045 and guard_dy > plate_dy + 0.045,
            details=f"guard=({guard_dx:.3f},{guard_dy:.3f}) platter=({plate_dx:.3f},{plate_dy:.3f})",
        )
    else:
        ctx.fail("fixed guard hoop surrounds rotating stage", "missing guard or platter element aabb")

    def _aabb_center_xy(bounds):
        return ((bounds[0][0] + bounds[1][0]) * 0.5, (bounds[0][1] + bounds[1][1]) * 0.5)

    rest_head = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({swing: 0.45}):
        moved_head = ctx.part_element_world_aabb(tonearm, elem="headshell")
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="stylus",
            negative_elem="record_disc",
            min_gap=0.0002,
            max_gap=0.020,
            name="tonearm swing keeps stylus above record",
        )
    if rest_head is not None and moved_head is not None:
        rest_xy = _aabb_center_xy(rest_head)
        moved_xy = _aabb_center_xy(moved_head)
        travel = math.hypot(moved_xy[0] - rest_xy[0], moved_xy[1] - rest_xy[1])
        ctx.check(
            "tonearm head moves in swing arc",
            travel > 0.070,
            details=f"travel={travel:.3f}, rest={rest_xy}, moved={moved_xy}",
        )
    else:
        ctx.fail("tonearm head moves in swing arc", "missing headshell aabb")

    return ctx.report()


object_model = build_object_model()
