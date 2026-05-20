from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_roof_vent_tower")

    galvanized = model.material("hot_dip_galvanized", rgba=(0.56, 0.59, 0.57, 1.0))
    dark_galv = model.material("dark_worn_galvanized", rgba=(0.28, 0.30, 0.30, 1.0))
    weathered = model.material("weathered_flap_plate", rgba=(0.40, 0.43, 0.42, 1.0))
    safety_yellow = model.material("replaceable_yellow_wear_parts", rgba=(0.92, 0.69, 0.08, 1.0))
    black_rubber = model.material("black_rubber_stops", rgba=(0.02, 0.02, 0.018, 1.0))
    bolt_steel = model.material("dark_fastener_heads", rgba=(0.08, 0.08, 0.075, 1.0))

    tower = model.part("vent_tower")

    # Roof curb, flashing, and a rectangular hollow tower made from wall plates.
    tower.visual(Box((1.02, 1.12, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=dark_galv, name="roof_flashing")
    tower.visual(Box((0.78, 0.88, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=galvanized, name="curb_base")
    tower.visual(Box((0.62, 0.72, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.205)), material=galvanized, name="raised_curb")

    # Main tower walls.  The front is assembled around the framed service/vent opening.
    tower.visual(Box((0.045, 0.62, 1.00)), origin=Origin(xyz=(-0.2375, 0.0, 0.75)), material=galvanized, name="rear_wall")
    tower.visual(Box((0.52, 0.045, 1.00)), origin=Origin(xyz=(0.0, -0.2875, 0.75)), material=galvanized, name="side_wall_0")
    tower.visual(Box((0.52, 0.045, 1.00)), origin=Origin(xyz=(0.0, 0.2875, 0.75)), material=galvanized, name="side_wall_1")
    tower.visual(Box((0.045, 0.62, 0.22)), origin=Origin(xyz=(0.2375, 0.0, 0.36)), material=galvanized, name="front_sill_panel")
    tower.visual(Box((0.045, 0.62, 0.20)), origin=Origin(xyz=(0.2375, 0.0, 1.15)), material=galvanized, name="front_header_panel")
    tower.visual(Box((0.045, 0.065, 0.62)), origin=Origin(xyz=(0.2375, -0.2775, 0.76)), material=galvanized, name="front_jamb_0")
    tower.visual(Box((0.045, 0.065, 0.62)), origin=Origin(xyz=(0.2375, 0.2775, 0.76)), material=galvanized, name="front_jamb_1")

    front_frame = BezelGeometry(
        opening_size=(0.54, 0.42),
        outer_size=(0.70, 0.58),
        depth=0.035,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.012,
        outer_corner_radius=0.020,
        face=BezelFace(style="radiused_step", front_lip=0.006, fillet=0.003),
    )
    tower.visual(
        mesh_from_geometry(front_frame, "front_frame"),
        origin=Origin(xyz=(0.282, 0.0, 0.78), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_galv,
        name="front_frame",
    )

    # Top collar remains open at the center like a real vent stack.
    tower.visual(Box((0.62, 0.070, 0.080)), origin=Origin(xyz=(0.0, -0.275, 1.29)), material=dark_galv, name="top_collar_0")
    tower.visual(Box((0.62, 0.070, 0.080)), origin=Origin(xyz=(0.0, 0.275, 1.29)), material=dark_galv, name="top_collar_1")
    tower.visual(Box((0.070, 0.62, 0.080)), origin=Origin(xyz=(-0.275, 0.0, 1.29)), material=dark_galv, name="top_collar_2")
    tower.visual(Box((0.070, 0.62, 0.080)), origin=Origin(xyz=(0.275, 0.0, 1.29)), material=dark_galv, name="top_collar_3")

    # Replaceable wear rails and bolted side service grilles communicate field serviceability.
    for y, name in [(-0.322, "wear_rail_0"), (0.322, "wear_rail_1")]:
        tower.visual(Box((0.050, 0.030, 0.72)), origin=Origin(xyz=(0.315, y, 0.76)), material=safety_yellow, name=name)

    for y, side_name, rot in [(-0.322, "side_grille_0", pi / 2.0), (0.322, "side_grille_1", -pi / 2.0)]:
        side_frame = BezelGeometry(
            opening_size=(0.28, 0.30),
            outer_size=(0.40, 0.44),
            depth=0.024,
            opening_shape="rect",
            outer_shape="rounded_rect",
            outer_corner_radius=0.014,
        )
        tower.visual(
            mesh_from_geometry(side_frame, side_name),
            origin=Origin(xyz=(0.0, y, 0.76), rpy=(rot, 0.0, 0.0)),
            material=dark_galv,
            name=side_name,
        )
        for i, z in enumerate((0.66, 0.76, 0.86)):
            tower.visual(
                Box((0.30, 0.024, 0.030)),
                origin=Origin(xyz=(0.0, y * 1.02, z)),
                material=galvanized,
                name=f"{side_name}_slat_{i}",
            )

    # Rubber closed stops and chunky open limit blocks are mounted to welded pads.
    for y, name in [(-0.215, "closed_stop_0"), (0.215, "closed_stop_1")]:
        tower.visual(Box((0.032, 0.070, 0.050)), origin=Origin(xyz=(0.344, y, 0.455)), material=black_rubber, name=name)
        tower.visual(Box((0.018, 0.090, 0.072)), origin=Origin(xyz=(0.319, y, 0.455)), material=dark_galv, name=f"{name}_backer")

    # Welded service-face rails tie the exterior wear parts, stops, grilles, and hinge leaf back into the tower shell.
    tower.visual(Box((0.082, 0.045, 0.92)), origin=Origin(xyz=(0.296, -0.305, 0.76)), material=dark_galv, name="service_side_rail_0")
    tower.visual(Box((0.082, 0.045, 0.92)), origin=Origin(xyz=(0.296, 0.305, 0.76)), material=dark_galv, name="service_side_rail_1")
    tower.visual(Box((0.084, 0.545, 0.060)), origin=Origin(xyz=(0.298, 0.0, 0.455)), material=dark_galv, name="stop_carrier")
    tower.visual(Box((0.090, 0.590, 0.080)), origin=Origin(xyz=(0.260, 0.0, 1.125)), material=dark_galv, name="hinge_backer")

    for y, suffix in [(-0.318, "0"), (0.318, "1")]:
        tower.visual(Box((0.040, 0.042, 0.460)), origin=Origin(xyz=(0.0, y, 0.76)), material=dark_galv, name=f"grille_center_post_{suffix}")

    for y, name in [(-0.300, "open_stop_0"), (0.300, "open_stop_1")]:
        tower.visual(Box((0.13, 0.045, 0.060)), origin=Origin(xyz=(0.382, y, 1.165), rpy=(0.0, 0.42, 0.0)), material=black_rubber, name=name)
        tower.visual(Box((0.105, 0.055, 0.075)), origin=Origin(xyz=(0.334, y, 1.118)), material=dark_galv, name=f"{name}_weldment")

    # Exposed parent-side hinge leaf and alternating hinge barrels.
    hinge_x = 0.335
    hinge_z = 1.10
    tower.visual(Box((0.046, 0.58, 0.085)), origin=Origin(xyz=(0.283, 0.0, 1.125)), material=dark_galv, name="tower_hinge_leaf")
    for i, y in enumerate((-0.220, 0.0, 0.220)):
        tower.visual(
            Cylinder(radius=0.026, length=0.085),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_galv,
            name=f"tower_knuckle_{i}",
        )
        tower.visual(
            Box((0.035, 0.085, 0.018)),
            origin=Origin(xyz=(0.318, y, 1.074)),
            material=dark_galv,
            name=f"tower_hinge_bridge_{i}",
        )

    # Bolts on the fixed wear rails and hinge plate.  Heads are slightly embedded in their plates.
    for i, (y, z) in enumerate([(-0.322, 0.50), (-0.322, 0.76), (-0.322, 1.02), (0.322, 0.50), (0.322, 0.76), (0.322, 1.02)]):
        tower.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.344, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"rail_bolt_{i}",
        )
    for i, y in enumerate((-0.250, -0.125, 0.125, 0.250)):
        tower.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(0.309, y, 1.145), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"hinge_leaf_bolt_{i}",
        )

    flap = model.part("weather_flap")
    # The flap frame is coincident with the hinge line at q=0.  Closed geometry extends down in local -Z.
    flap.visual(Box((0.050, 0.560, 0.640)), origin=Origin(xyz=(0.050, 0.0, -0.380)), material=weathered, name="flap_skin")
    flap.visual(Box((0.074, 0.600, 0.060)), origin=Origin(xyz=(0.057, 0.0, -0.095)), material=dark_galv, name="top_stiffener")
    flap.visual(Box((0.068, 0.050, 0.620)), origin=Origin(xyz=(0.057, -0.285, -0.390)), material=dark_galv, name="side_stiffener_0")
    flap.visual(Box((0.068, 0.050, 0.620)), origin=Origin(xyz=(0.057, 0.285, -0.390)), material=dark_galv, name="side_stiffener_1")
    flap.visual(Box((0.068, 0.570, 0.055)), origin=Origin(xyz=(0.057, 0.0, -0.690)), material=dark_galv, name="bottom_wear_bar")
    flap.visual(Box((0.030, 0.485, 0.050)), origin=Origin(xyz=(0.084, 0.0, -0.385), rpy=(0.52, 0.0, 0.0)), material=dark_galv, name="diagonal_rib_0")
    flap.visual(Box((0.030, 0.485, 0.050)), origin=Origin(xyz=(0.084, 0.0, -0.385), rpy=(-0.52, 0.0, 0.0)), material=dark_galv, name="diagonal_rib_1")

    flap.visual(Box((0.058, 0.545, 0.070)), origin=Origin(xyz=(0.052, 0.0, -0.046)), material=dark_galv, name="flap_hinge_leaf")
    for i, y in enumerate((-0.110, 0.110)):
        flap.visual(
            Cylinder(radius=0.026, length=0.090),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_galv,
            name=f"flap_knuckle_{i}",
        )
        flap.visual(
            Box((0.078, 0.090, 0.018)),
            origin=Origin(xyz=(0.030, y, -0.020)),
            material=dark_galv,
            name=f"flap_hinge_bridge_{i}",
        )

    # A welded service handle gives a clear, gloved-hand maintenance point.
    flap.visual(Cylinder(radius=0.016, length=0.350), origin=Origin(xyz=(0.132, 0.0, -0.405), rpy=(pi / 2.0, 0.0, 0.0)), material=bolt_steel, name="service_handle_bar")
    for i, y in enumerate((-0.175, 0.175)):
        flap.visual(Cylinder(radius=0.013, length=0.075), origin=Origin(xyz=(0.096, y, -0.405), rpy=(0.0, pi / 2.0, 0.0)), material=bolt_steel, name=f"service_handle_stem_{i}")
        flap.visual(Box((0.015, 0.060, 0.055)), origin=Origin(xyz=(0.070, y, -0.405)), material=dark_galv, name=f"handle_mount_{i}")

    for i, (y, z) in enumerate([(-0.205, -0.155), (0.205, -0.155), (-0.205, -0.625), (0.205, -0.625)]):
        flap.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.079, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"flap_bolt_{i}",
        )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=1.2, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("vent_tower")
    flap = object_model.get_part("weather_flap")
    hinge = object_model.get_articulation("tower_to_flap")

    ctx.check(
        "flap has serviceable hinge travel",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and 1.0 <= hinge.motion_limits.upper <= 1.3,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            tower,
            axes="yz",
            elem_a="flap_skin",
            elem_b="front_frame",
            min_overlap=0.42,
            name="closed flap covers framed opening",
        )
        ctx.expect_gap(
            flap,
            tower,
            axis="x",
            positive_elem="flap_skin",
            negative_elem="closed_stop_0",
            max_gap=0.003,
            max_penetration=0.001,
            name="flap seats on lower rubber stop",
        )
        closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_skin")

    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_skin")

    ctx.check(
        "flap opens outward and upward to the stop range",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.30
        and open_aabb[0][2] > closed_aabb[0][2] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
