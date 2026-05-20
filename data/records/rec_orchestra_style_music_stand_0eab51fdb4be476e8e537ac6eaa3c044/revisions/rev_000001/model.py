from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _trapezoid_sheet(
    *,
    bottom_width: float,
    top_width: float,
    height: float,
    thickness: float,
    y_center: float,
    z_center: float,
) -> MeshGeometry:
    """Closed, thin trapezoidal music-desk plate in local X/Y/Z."""

    z0 = z_center - height / 2.0
    z1 = z_center + height / 2.0
    y0 = y_center - thickness / 2.0
    y1 = y_center + thickness / 2.0
    xb0 = bottom_width / 2.0
    xt0 = top_width / 2.0

    verts = [
        (-xb0, y0, z0),
        (xb0, y0, z0),
        (xt0, y0, z1),
        (-xt0, y0, z1),
        (-xb0, y1, z0),
        (xb0, y1, z0),
        (xt0, y1, z1),
        (-xt0, y1, z1),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return MeshGeometry(vertices=verts, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="symphony_music_stand")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_metal = model.material("dark_burnished_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    worn_edge = model.material("rubbed_metal_edge", rgba=(0.32, 0.33, 0.34, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.175, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=satin_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.165, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber,
        name="rubber_foot_ring",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=dark_metal,
        name="center_boss",
    )

    lower_tube_shell = LatheGeometry.from_shell_profiles(
        [(0.018, 0.050), (0.018, 0.800)],
        [(0.0128, 0.050), (0.0128, 0.800)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(lower_tube_shell, "lower_tube_shell"),
        material=dark_metal,
        name="lower_tube_shell",
    )

    height_collar_shell = LatheGeometry.from_shell_profiles(
        [(0.033, 0.735), (0.034, 0.755), (0.034, 0.817), (0.031, 0.833)],
        [(0.0145, 0.735), (0.0145, 0.755), (0.0145, 0.817), (0.0145, 0.833)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(height_collar_shell, "height_collar_shell"),
        material=satin_black,
        name="height_collar",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.043, 0.785),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="clamp_boss",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.010, length=0.750),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=dark_metal,
        name="inner_mast",
    )
    mast.visual(
        Box((0.006, 0.004, 0.055)),
        origin=Origin(xyz=(0.0, 0.0108, -0.380)),
        material=worn_edge,
        name="guide_shoe_front",
    )
    mast.visual(
        Box((0.006, 0.004, 0.055)),
        origin=Origin(xyz=(0.0, -0.0108, -0.380)),
        material=worn_edge,
        name="guide_shoe_rear",
    )
    mast.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.314)),
        material=satin_black,
        name="head_socket",
    )
    mast.visual(
        Box((0.135, 0.036, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.323)),
        material=satin_black,
        name="tilt_yoke_bridge",
    )
    mast.visual(
        Box((0.012, 0.064, 0.072)),
        origin=Origin(xyz=(-0.067, 0.0, 0.360)),
        material=satin_black,
        name="yoke_plate_0",
    )
    mast.visual(
        Box((0.012, 0.064, 0.072)),
        origin=Origin(xyz=(0.067, 0.0, 0.360)),
        material=satin_black,
        name="yoke_plate_1",
    )
    mast.visual(
        Cylinder(radius=0.0065, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edge,
        name="tilt_axis_pin",
    )

    knob = model.part("clamp_knob")
    knob.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.014, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=worn_edge,
        name="threaded_stem",
    )
    clamp_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.026,
            body_style="lobed",
            base_diameter=0.030,
            top_diameter=0.040,
            crown_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=10, depth=0.0014, width=0.003),
            bore=KnobBore(style="round", diameter=0.009),
            center=False,
        ),
        "lobed_clamp_knob",
    )
    knob.visual(
        clamp_knob_mesh,
        origin=Origin(
            xyz=(0.0, 0.027, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="lobed_knob",
    )

    desk = model.part("desk")
    desk_panel = _trapezoid_sheet(
        bottom_width=0.500,
        top_width=0.540,
        height=0.355,
        thickness=0.006,
        y_center=0.088,
        z_center=-0.030,
    )
    desk.visual(
        mesh_from_geometry(desk_panel, "solid_desk_panel"),
        material=satin_black,
        name="solid_desk_panel",
    )
    desk.visual(
        Box((0.558, 0.064, 0.010)),
        origin=Origin(xyz=(0.0, 0.057, -0.210)),
        material=satin_black,
        name="score_shelf",
    )
    desk.visual(
        Box((0.558, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, 0.020, -0.191)),
        material=satin_black,
        name="lower_score_lip",
    )
    desk.visual(
        Box((0.014, 0.012, 0.335)),
        origin=Origin(xyz=(-0.262, 0.085, -0.030)),
        material=worn_edge,
        name="side_rim_0",
    )
    desk.visual(
        Box((0.014, 0.012, 0.335)),
        origin=Origin(xyz=(0.262, 0.085, -0.030)),
        material=worn_edge,
        name="side_rim_1",
    )
    desk.visual(
        Box((0.510, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.085, 0.150)),
        material=worn_edge,
        name="top_rim",
    )
    desk.visual(
        Cylinder(radius=0.015, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edge,
        name="tilt_barrel",
    )
    desk.visual(
        Box((0.105, 0.077, 0.026)),
        origin=Origin(xyz=(0.0, 0.0505, 0.0)),
        material=satin_black,
        name="head_lug",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.350),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.45, upper=0.70),
    )
    model.articulation(
        "base_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.0, 0.052, 0.785)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    knob = object_model.get_part("clamp_knob")
    height_slide = object_model.get_articulation("base_to_mast")
    tilt = object_model.get_articulation("mast_to_desk")
    knob_spin = object_model.get_articulation("base_to_clamp_knob")

    ctx.allow_overlap(
        mast,
        desk,
        elem_a="tilt_axis_pin",
        elem_b="tilt_barrel",
        reason="The visible tilt-head axle is intentionally captured inside the desk barrel bearing.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="lower_tube_shell",
        margin=0.0,
        name="inner mast is centered inside the lower tube",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="lower_tube_shell",
        min_overlap=0.30,
        name="collapsed mast has long retained insertion",
    )
    ctx.expect_contact(
        knob,
        base,
        elem_a="threaded_stem",
        elem_b="clamp_boss",
        contact_tol=0.001,
        name="clamp knob stem seats on height-collar boss",
    )
    ctx.expect_contact(
        desk,
        mast,
        elem_a="tilt_barrel",
        elem_b="tilt_axis_pin",
        contact_tol=0.012,
        name="desk tilt barrel sits on the visible horizontal head axis",
    )

    rest_pos = ctx.part_world_position(mast)
    with ctx.pose({height_slide: 0.350}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="lower_tube_shell",
            min_overlap=0.08,
            name="extended mast remains inserted in lower tube",
        )
        extended_pos = ctx.part_world_position(mast)

    ctx.check(
        "prismatic mast extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "clamp knob is a continuous rotary joint",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )

    closed_aabb = ctx.part_world_aabb(desk)
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk rotates on horizontal tilt axis",
        closed_aabb is not None
        and tilted_aabb is not None
        and abs((tilted_aabb[0][1] + tilted_aabb[1][1]) - (closed_aabb[0][1] + closed_aabb[1][1]))
        > 0.04,
        details=f"closed={closed_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
