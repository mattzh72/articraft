from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_roof_vent_tower")

    painted = model.material("textured_olive_paint", rgba=(0.23, 0.29, 0.24, 1.0))
    edge_paint = model.material("darker_wear_edges", rgba=(0.12, 0.15, 0.13, 1.0))
    gasket = model.material("black_rubber_gasket", rgba=(0.015, 0.018, 0.016, 1.0))
    metal = model.material("zinc_plated_fasteners", rgba=(0.72, 0.70, 0.62, 1.0))
    shadow = model.material("dark_interior", rgba=(0.025, 0.028, 0.026, 1.0))

    tower = model.part("tower")

    def add_box(part, name: str, size, xyz, material: Material = painted, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cyl(part, name: str, radius: float, length: float, xyz, material: Material = metal, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Heavy roof curb and flashing flange.  The tower is intentionally bulky:
    # thick sheet-metal walls, wide roof flashing, serviceable bolt heads, and
    # protective corner bars suitable for an exposed utility roof installation.
    add_box(tower, "base_flange", (0.72, 0.60, 0.050), (0.0, 0.0, 0.025), painted)
    add_box(tower, "curb_front", (0.46, 0.040, 0.120), (0.0, -0.220, 0.110), painted)
    add_box(tower, "curb_rear", (0.46, 0.040, 0.120), (0.0, 0.220, 0.110), painted)
    add_box(tower, "curb_side_0", (0.040, 0.44, 0.120), (-0.230, 0.0, 0.110), painted)
    add_box(tower, "curb_side_1", (0.040, 0.44, 0.120), (0.230, 0.0, 0.110), painted)

    width = 0.42
    depth = 0.34
    height = 0.62
    wall = 0.035
    z0 = 0.050
    zc = z0 + height / 2.0
    front_y = -depth / 2.0
    rear_y = depth / 2.0
    side_x = width / 2.0

    # Rectangular vent tower shell built from connected wall sections rather
    # than a solid block, leaving a real front outlet aperture.
    add_box(tower, "side_wall_0", (wall, depth, height), (-side_x + wall / 2.0, 0.0, zc), painted)
    add_box(tower, "side_wall_1", (wall, depth, height), (side_x - wall / 2.0, 0.0, zc), painted)
    add_box(tower, "rear_wall", (width, wall, height), (0.0, rear_y - wall / 2.0, zc), painted)
    add_box(tower, "front_lower", (width, wall, 0.290), (0.0, front_y + wall / 2.0, z0 + 0.145), painted)
    add_box(tower, "front_header", (width, wall, 0.100), (0.0, front_y + wall / 2.0, z0 + 0.570), painted)
    add_box(tower, "front_jamb_0", (0.065, wall, 0.230), (-0.1775, front_y + wall / 2.0, z0 + 0.405), painted)
    add_box(tower, "front_jamb_1", (0.065, wall, 0.230), (0.1775, front_y + wall / 2.0, z0 + 0.405), painted)

    # Welded-looking corner bars and reinforcement bands.
    for idx, x in enumerate((-0.207, 0.207)):
        add_box(tower, f"front_corner_{idx}", (0.026, 0.028, height), (x, front_y - 0.010, zc), edge_paint)
        add_box(tower, f"rear_corner_{idx}", (0.026, 0.028, height), (x, rear_y + 0.006, zc), edge_paint)
    for idx, z in enumerate((0.245, 0.515)):
        add_box(tower, f"side_band_{idx}_0", (0.030, depth + 0.024, 0.028), (-side_x - 0.006, 0.0, z), edge_paint)
        add_box(tower, f"side_band_{idx}_1", (0.030, depth + 0.024, 0.028), (side_x + 0.006, 0.0, z), edge_paint)
        add_box(tower, f"rear_band_{idx}", (width + 0.040, 0.028, 0.028), (0.0, rear_y + 0.007, z), edge_paint)

    # Finished framed outlet opening with a radiused step and black gasket.
    outlet_frame = BezelGeometry(
        opening_size=(0.290, 0.230),
        outer_size=(0.390, 0.330),
        depth=0.035,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.018,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
        center=True,
    )
    tower.visual(
        mesh_from_geometry(outlet_frame, "outlet_frame_mesh"),
        origin=Origin(xyz=(0.0, -0.184, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=edge_paint,
        name="outlet_frame",
    )
    add_box(tower, "gasket_top", (0.340, 0.006, 0.014), (0.0, -0.205, 0.584), gasket)
    add_box(tower, "gasket_bottom", (0.340, 0.006, 0.014), (0.0, -0.205, 0.326), gasket)
    add_box(tower, "gasket_side_0", (0.014, 0.006, 0.244), (-0.164, -0.205, 0.455), gasket)
    add_box(tower, "gasket_side_1", (0.014, 0.006, 0.244), (0.164, -0.205, 0.455), gasket)

    # Recessed dark baffle slats and a center mullion keep the outlet visibly
    # functional without closing the aperture.
    add_box(tower, "center_mullion", (0.018, 0.035, 0.220), (0.0, -0.145, 0.455), shadow)
    for idx, z in enumerate((0.385, 0.455, 0.525)):
        add_box(tower, f"baffle_slat_{idx}", (0.310, 0.072, 0.014), (0.0, -0.143, z), shadow, rpy=(math.radians(-14.0), 0.0, 0.0))

    # Flange and frame fasteners.
    for idx, (x, y) in enumerate(
        (
            (-0.300, -0.235),
            (0.300, -0.235),
            (-0.300, 0.235),
            (0.300, 0.235),
            (-0.145, -0.245),
            (0.145, -0.245),
            (-0.145, 0.245),
            (0.145, 0.245),
        )
    ):
        add_cyl(tower, f"flange_bolt_{idx}", 0.012, 0.008, (x, y, 0.054), metal)

    frame_fasteners = [
        (-0.165, -0.205, 0.590),
        (0.165, -0.205, 0.590),
        (-0.165, -0.205, 0.320),
        (0.165, -0.205, 0.320),
        (-0.190, -0.205, 0.455),
        (0.190, -0.205, 0.455),
    ]
    for idx, xyz in enumerate(frame_fasteners):
        add_cyl(tower, f"frame_screw_{idx}", 0.009, 0.007, xyz, metal, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Explicit exposed hinge hardware: two fixed knuckles, a full hinge pin, and
    # fixed leaves bolted to the tower header.
    hinge_y = -0.205
    hinge_z = 0.642
    add_box(tower, "fixed_leaf_0", (0.100, 0.008, 0.070), (-0.130, -0.195, hinge_z - 0.027), edge_paint)
    add_box(tower, "fixed_leaf_1", (0.100, 0.008, 0.070), (0.130, -0.195, hinge_z - 0.027), edge_paint)
    add_cyl(tower, "fixed_knuckle_0", 0.014, 0.082, (-0.130, hinge_y, hinge_z), edge_paint, rpy=(0.0, math.pi / 2.0, 0.0))
    add_cyl(tower, "fixed_knuckle_1", 0.014, 0.082, (0.130, hinge_y, hinge_z), edge_paint, rpy=(0.0, math.pi / 2.0, 0.0))
    add_cyl(tower, "hinge_pin", 0.0055, 0.380, (0.0, hinge_y, hinge_z), metal, rpy=(0.0, math.pi / 2.0, 0.0))
    add_cyl(tower, "pin_head_0", 0.012, 0.010, (-0.198, hinge_y, hinge_z), metal, rpy=(0.0, math.pi / 2.0, 0.0))
    add_cyl(tower, "pin_head_1", 0.012, 0.010, (0.198, hinge_y, hinge_z), metal, rpy=(0.0, math.pi / 2.0, 0.0))
    for idx, x in enumerate((-0.148, -0.112, 0.112, 0.148)):
        add_cyl(tower, f"leaf_screw_{idx}", 0.0065, 0.006, (x, -0.202, hinge_z - 0.037), metal, rpy=(math.pi / 2.0, 0.0, 0.0))

    flap = model.part("flap")
    # The flap's part frame is the physical hinge axis.  Closed geometry hangs
    # downward in local -Z and stands just proud of the outlet gasket.
    add_box(flap, "weather_panel", (0.370, 0.028, 0.310), (0.0, -0.027, -0.155), painted)
    add_box(flap, "panel_top_rib", (0.390, 0.018, 0.030), (0.0, -0.049, -0.018), edge_paint)
    add_box(flap, "panel_bottom_rib", (0.390, 0.018, 0.030), (0.0, -0.049, -0.292), edge_paint)
    add_box(flap, "panel_side_0", (0.030, 0.018, 0.284), (-0.180, -0.049, -0.155), edge_paint)
    add_box(flap, "panel_side_1", (0.030, 0.018, 0.284), (0.180, -0.049, -0.155), edge_paint)
    add_box(flap, "center_stiffener", (0.030, 0.022, 0.250), (0.0, -0.055, -0.170), edge_paint)
    add_box(flap, "drip_lip", (0.400, 0.042, 0.018), (0.0, -0.052, -0.318), edge_paint)
    add_box(flap, "moving_leaf", (0.128, 0.010, 0.076), (0.0, -0.018, -0.039), edge_paint)
    add_cyl(flap, "moving_knuckle", 0.014, 0.112, (0.0, 0.0, 0.0), edge_paint, rpy=(0.0, math.pi / 2.0, 0.0))
    for idx, x in enumerate((-0.160, -0.055, 0.055, 0.160)):
        add_cyl(flap, f"flap_screw_{idx}", 0.0075, 0.007, (x, -0.0435, -0.105 if abs(x) < 0.06 else -0.245), metal, rpy=(math.pi / 2.0, 0.0, 0.0))
    add_cyl(flap, "leaf_screw_0", 0.0065, 0.006, (-0.042, -0.026, -0.052), metal, rpy=(math.pi / 2.0, 0.0, 0.0))
    add_cyl(flap, "leaf_screw_1", 0.0065, 0.006, (0.042, -0.026, -0.052), metal, rpy=(math.pi / 2.0, 0.0, 0.0))

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed flap extends down local -Z.  Rotating about -X swings the
        # free edge outward toward negative Y and upward, like a weather flap.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.18),
        motion_properties=MotionProperties(damping=0.35, friction=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("tower_to_flap")

    ctx.allow_overlap(
        tower,
        flap,
        elem_a="hinge_pin",
        elem_b="moving_knuckle",
        reason="The metal hinge pin is intentionally captured inside the rotating flap knuckle.",
    )
    ctx.expect_within(
        tower,
        flap,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="moving_knuckle",
        margin=0.002,
        name="hinge pin is centered in moving knuckle",
    )
    ctx.expect_overlap(
        tower,
        flap,
        axes="x",
        elem_a="hinge_pin",
        elem_b="moving_knuckle",
        min_overlap=0.10,
        name="hinge pin spans the moving knuckle",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            tower,
            axes="xz",
            elem_a="weather_panel",
            elem_b="outlet_frame",
            min_overlap=0.22,
            name="closed flap covers framed outlet",
        )
        ctx.expect_gap(
            tower,
            flap,
            axis="y",
            min_gap=0.006,
            max_gap=0.030,
            positive_elem="outlet_frame",
            negative_elem="weather_panel",
            name="closed flap stands proud of gasket",
        )
        closed_aabb = ctx.part_element_world_aabb(flap, elem="weather_panel")

    with ctx.pose({hinge: 1.05}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="weather_panel")
        ctx.expect_gap(
            tower,
            flap,
            axis="y",
            min_gap=0.004,
            positive_elem="outlet_frame",
            negative_elem="weather_panel",
            name="opened flap clears outlet face",
        )

    ctx.check(
        "flap swings outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.080
        and open_aabb[0][2] > closed_aabb[0][2] + 0.080,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
