from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_geometry(
    *,
    axis: str,
    length: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 48,
) -> MeshGeometry:
    """Closed annular tube centered on the origin and aligned to X or Y."""
    geom = MeshGeometry()
    outer_start = []
    outer_end = []
    inner_start = []
    inner_end = []
    half = length / 2.0

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)

        if axis == "x":
            outer_start.append(geom.add_vertex(-half, outer_radius * c, outer_radius * s))
            outer_end.append(geom.add_vertex(half, outer_radius * c, outer_radius * s))
            inner_start.append(geom.add_vertex(-half, inner_radius * c, inner_radius * s))
            inner_end.append(geom.add_vertex(half, inner_radius * c, inner_radius * s))
        elif axis == "y":
            outer_start.append(geom.add_vertex(outer_radius * c, -half, outer_radius * s))
            outer_end.append(geom.add_vertex(outer_radius * c, half, outer_radius * s))
            inner_start.append(geom.add_vertex(inner_radius * c, -half, inner_radius * s))
            inner_end.append(geom.add_vertex(inner_radius * c, half, inner_radius * s))
        else:
            raise ValueError(f"unsupported tube axis {axis!r}")

    for i in range(segments):
        j = (i + 1) % segments
        # Outer cylindrical wall.
        geom.add_face(outer_start[i], outer_start[j], outer_end[j])
        geom.add_face(outer_start[i], outer_end[j], outer_end[i])
        # Inner cylindrical wall, reversed so the visible normal faces the bore.
        geom.add_face(inner_start[i], inner_end[j], inner_start[j])
        geom.add_face(inner_start[i], inner_end[i], inner_end[j])
        # Start annular face.
        geom.add_face(outer_start[i], inner_start[j], outer_start[j])
        geom.add_face(outer_start[i], inner_start[i], inner_start[j])
        # End annular face.
        geom.add_face(outer_end[i], outer_end[j], inner_end[j])
        geom.add_face(outer_end[i], inner_end[j], inner_end[i])

    return geom


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _lower_jaw_with_screw_hole() -> cq.Workplane:
    """Lower C-clamp jaw with a real vertical bore for the clamp screw."""
    width_x = 0.085
    depth_y = 0.070
    height_z = 0.014
    screw_y_offset = -0.025
    jaw = cq.Workplane("XY").box(width_x, depth_y, height_z).edges("|Z").fillet(0.003)
    return (
        jaw.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .center(0.0, screw_y_offset)
        .circle(0.008)
        .cutThruAll()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_metal_classroom_pencil_sharpener")

    cast = model.material("aged_cast_green_gray", rgba=(0.34, 0.42, 0.36, 1.0))
    dark_cast = model.material("dark_cast_shadow", rgba=(0.05, 0.055, 0.05, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.65, 1.0))
    worn_steel = model.material("worn_threaded_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    black = model.material("blackened_drawer", rgba=(0.035, 0.038, 0.035, 1.0))
    wood = model.material("varnished_wood_handle", rgba=(0.54, 0.30, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_rounded_box((0.220, 0.170, 0.025), 0.004), "base_plinth"),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=cast,
        name="base_plinth",
    )
    housing.visual(
        mesh_from_cadquery(_rounded_box((0.016, 0.154, 0.108), 0.003), "side_wall"),
        origin=Origin(xyz=(-0.086, 0.0, 0.128)),
        material=cast,
        name="side_wall",
    )
    # The crank side is split around a real clearance window for the supported axle.
    for visual_name, y, depth, z, height in (
        ("side_wall_front", -0.0495, 0.053, 0.128, 0.108),
        ("side_wall_rear", 0.0495, 0.053, 0.128, 0.108),
        ("side_wall_lower", 0.0, 0.056, 0.0935, 0.037),
        ("side_wall_upper", 0.0, 0.056, 0.169, 0.022),
    ):
        housing.visual(
            mesh_from_cadquery(_rounded_box((0.016, depth, height), 0.0025), visual_name),
            origin=Origin(xyz=(0.086, y, z)),
            material=cast,
            name=visual_name,
        )
    housing.visual(
        mesh_from_cadquery(_rounded_box((0.184, 0.154, 0.020), 0.004), "top_cover"),
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
        material=cast,
        name="top_cover",
    )
    housing.visual(
        mesh_from_cadquery(_rounded_box((0.184, 0.016, 0.108), 0.003), "rear_wall"),
        origin=Origin(xyz=(0.0, 0.069, 0.128)),
        material=cast,
        name="rear_wall",
    )
    # Four front castings leave the lower drawer mouth and a central pencil-entry void.
    for visual_name, x, sx, z, sz in (
        ("front_face_left", -0.060, 0.060, 0.140, 0.082),
        ("front_face_right", 0.060, 0.060, 0.140, 0.082),
        ("front_face_top", 0.0, 0.060, 0.174, 0.016),
        ("front_face_lower", 0.0, 0.060, 0.105, 0.020),
    ):
        housing.visual(
            mesh_from_cadquery(_rounded_box((sx, 0.014, sz), 0.0025), visual_name),
            origin=Origin(xyz=(x, -0.075, z)),
            material=cast,
            name=visual_name,
        )
    housing.visual(
        mesh_from_geometry(
            _tube_geometry(axis="y", length=0.018, outer_radius=0.032, inner_radius=0.017),
            "entry_ring",
        ),
        origin=Origin(xyz=(0.0, -0.083, 0.145)),
        material=cast,
        name="entry_ring",
    )
    housing.visual(
        mesh_from_geometry(
            _tube_geometry(axis="y", length=0.060, outer_radius=0.021, inner_radius=0.015),
            "cutter_tunnel",
        ),
        origin=Origin(xyz=(0.0, -0.047, 0.145)),
        material=dark_cast,
        name="cutter_tunnel",
    )
    housing.visual(
        mesh_from_geometry(
            _tube_geometry(axis="x", length=0.026, outer_radius=0.023, inner_radius=0.0105),
            "axle_bearing",
        ),
        origin=Origin(xyz=(0.099, 0.0, 0.135)),
        material=cast,
        name="axle_bearing",
    )
    # Small drawer guides mounted inside the lower mouth.
    for visual_name, x in (("drawer_rail_0", -0.075), ("drawer_rail_1", 0.075)):
        housing.visual(
            Box((0.006, 0.105, 0.006)),
            origin=Origin(xyz=(x, -0.010, 0.083)),
            material=dark_cast,
            name=visual_name,
        )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_rounded_box((0.138, 0.012, 0.040), 0.003), "drawer_front"),
        origin=Origin(xyz=(0.0, -0.091, 0.096)),
        material=black,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.120, 0.126, 0.006)),
        origin=Origin(xyz=(0.0, -0.022, 0.082)),
        material=black,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.006, 0.126, 0.022)),
        origin=Origin(xyz=(-0.063, -0.022, 0.093)),
        material=black,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.006, 0.126, 0.022)),
        origin=Origin(xyz=(0.063, -0.022, 0.093)),
        material=black,
        name="drawer_side_1",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, -0.099, 0.096), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drawer_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )
    crank.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="crank_hub",
    )
    crank.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_collar",
    )
    crank.visual(
        Box((0.010, 0.014, 0.086)),
        origin=Origin(xyz=(0.055, 0.0, -0.040)),
        material=steel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.038),
        origin=Origin(xyz=(0.076, 0.0, -0.082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood,
        name="handle_grip",
    )

    clamp_bracket = model.part("clamp_bracket")
    clamp_bracket.visual(
        mesh_from_cadquery(_rounded_box((0.092, 0.048, 0.010), 0.0025), "clamp_top_flange"),
        origin=Origin(xyz=(0.0, 0.047, 0.045)),
        material=cast,
        name="clamp_top_flange",
    )
    clamp_bracket.visual(
        mesh_from_cadquery(_rounded_box((0.026, 0.016, 0.100), 0.0025), "clamp_rear_spine"),
        origin=Origin(xyz=(0.0, 0.074, -0.010)),
        material=cast,
        name="clamp_rear_spine",
    )
    clamp_bracket.visual(
        mesh_from_cadquery(_lower_jaw_with_screw_hole(), "clamp_lower_jaw"),
        origin=Origin(xyz=(0.0, 0.040, -0.060)),
        material=cast,
        name="clamp_lower_jaw",
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.0055, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=worn_steel,
        name="screw_rod",
    )
    clamp_screw.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="screw_collar",
    )
    for i, z in enumerate((-0.022, -0.010, 0.002, 0.014, 0.026, 0.038, 0.050)):
        clamp_screw.visual(
            Cylinder(radius=0.0064, length=0.0016),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=worn_steel,
            name=f"thread_ridge_{i}",
        )
    clamp_screw.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=steel,
        name="pressure_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.0042, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tommy_bar",
    )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.075),
    )
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(0.099, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )
    model.articulation(
        "housing_to_clamp_bracket",
        ArticulationType.FIXED,
        parent=housing,
        child=clamp_bracket,
        origin=Origin(),
    )
    model.articulation(
        "clamp_bracket_to_screw",
        ArticulationType.CONTINUOUS,
        parent=clamp_bracket,
        child=clamp_screw,
        origin=Origin(xyz=(0.0, 0.015, -0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    bracket = object_model.get_part("clamp_bracket")
    screw = object_model.get_part("clamp_screw")
    drawer_joint = object_model.get_articulation("housing_to_drawer")
    crank_joint = object_model.get_articulation("housing_to_crank")
    screw_joint = object_model.get_articulation("clamp_bracket_to_screw")

    ctx.check(
        "primary mechanisms are articulated",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and screw_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"drawer={drawer_joint.articulation_type}, crank={crank_joint.articulation_type}, screw={screw_joint.articulation_type}",
    )
    ctx.check(
        "pencil entry has a through bore and cutter tunnel",
        housing.get_visual("entry_ring") is not None and housing.get_visual("cutter_tunnel") is not None,
        details="entry_ring and cutter_tunnel visuals must remain as hollow annular mesh elements",
    )
    ctx.expect_gap(
        housing,
        bracket,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="base_plinth",
        negative_elem="clamp_top_flange",
        name="bench clamp bracket is mounted under the base",
    )
    ctx.expect_within(
        crank,
        housing,
        axes="yz",
        inner_elem="axle",
        outer_elem="axle_bearing",
        margin=0.003,
        name="crank axle is centered in the side bearing",
    )
    ctx.expect_overlap(
        crank,
        housing,
        axes="x",
        min_overlap=0.015,
        elem_a="axle",
        elem_b="axle_bearing",
        name="crank axle remains supported by the bearing sleeve",
    )
    ctx.expect_within(
        screw,
        bracket,
        axes="xy",
        inner_elem="screw_rod",
        outer_elem="clamp_lower_jaw",
        margin=0.004,
        name="clamp screw is centered in the lower jaw bore",
    )
    ctx.expect_overlap(
        screw,
        bracket,
        axes="z",
        min_overlap=0.010,
        elem_a="screw_rod",
        elem_b="clamp_lower_jaw",
        name="clamp screw passes through the threaded jaw",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.075}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            housing,
            axes="y",
            min_overlap=0.030,
            elem_a="drawer_floor",
            elem_b="base_plinth",
            name="extended drawer retains hidden insertion",
        )
    ctx.check(
        "drawer slides out from the front",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.070,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    rest_crank_pos = ctx.part_world_position(crank)
    with ctx.pose({crank_joint: math.pi / 2.0, screw_joint: math.pi / 2.0}):
        rotated_crank_pos = ctx.part_world_position(crank)
        rotated_screw_pos = ctx.part_world_position(screw)
    ctx.check(
        "continuous rotations keep their axes fixed",
        rest_crank_pos is not None
        and rotated_crank_pos is not None
        and rotated_screw_pos is not None
        and abs(rest_crank_pos[0] - rotated_crank_pos[0]) < 1e-6,
        details=f"crank_rest={rest_crank_pos}, crank_rotated={rotated_crank_pos}, screw_rotated={rotated_screw_pos}",
    )

    return ctx.report()


object_model = build_object_model()
