from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


ROT_CYL_X = (0.0, math.pi / 2.0, 0.0)
ROT_CYL_Y = (math.pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_roof_vent_tower")

    galvanized = model.material("weathered_galvanized", rgba=(0.58, 0.62, 0.61, 1.0))
    dark_galv = model.material("dark_seamed_edges", rgba=(0.34, 0.37, 0.36, 1.0))
    roof_black = model.material("aged_roof_membrane", rgba=(0.045, 0.048, 0.045, 1.0))
    rubber = model.material("black_rubber_stops", rgba=(0.01, 0.01, 0.009, 1.0))
    bolt_steel = model.material("dull_stainless_bolts", rgba=(0.78, 0.77, 0.72, 1.0))
    primer = model.material("oxide_primer_marks", rgba=(0.53, 0.18, 0.10, 1.0))

    tower = model.part("tower")

    def box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        tower.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        tower.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Legacy roof patch and curb: the tower is visibly tied into a roof deck,
    # not floating as a freestanding box.
    box("roof_patch", (0.92, 0.78, 0.035), (0.0, 0.0, 0.0175), roof_black)
    box("wide_flashing", (0.68, 0.58, 0.026), (0.0, 0.0, 0.055), dark_galv)
    box("front_flashing_lip", (0.68, 0.035, 0.030), (0.0, -0.307, 0.073), galvanized)
    box("rear_flashing_lip", (0.68, 0.035, 0.030), (0.0, 0.307, 0.073), galvanized)
    box("side_flashing_lip_0", (0.035, 0.58, 0.030), (-0.357, 0.0, 0.073), galvanized)
    box("side_flashing_lip_1", (0.035, 0.58, 0.030), (0.357, 0.0, 0.073), galvanized)

    # Bolted adapter frame around the roof penetration.
    box("front_adapter", (0.58, 0.060, 0.050), (0.0, -0.245, 0.112), dark_galv)
    box("rear_adapter", (0.58, 0.060, 0.050), (0.0, 0.245, 0.112), dark_galv)
    box("side_adapter_0", (0.060, 0.50, 0.050), (-0.300, 0.0, 0.112), dark_galv)
    box("side_adapter_1", (0.060, 0.50, 0.050), (0.300, 0.0, 0.112), dark_galv)
    box("front_curb_riser", (0.54, 0.040, 0.145), (0.0, -0.188, 0.092), galvanized)
    box("rear_curb_riser", (0.54, 0.040, 0.145), (0.0, 0.188, 0.092), galvanized)
    box("side_curb_riser_0", (0.040, 0.42, 0.145), (-0.248, 0.0, 0.092), galvanized)
    box("side_curb_riser_1", (0.040, 0.42, 0.145), (0.248, 0.0, 0.092), galvanized)

    for i, x in enumerate((-0.235, -0.080, 0.080, 0.235)):
        cyl(f"front_adapter_bolt_{i}", 0.010, 0.012, (x, -0.255, 0.136), bolt_steel)
        cyl(f"rear_adapter_bolt_{i}", 0.010, 0.012, (x, 0.255, 0.136), bolt_steel)
    for i, y in enumerate((-0.170, 0.0, 0.170)):
        cyl(f"side_adapter_bolt_0_{i}", 0.010, 0.012, (-0.315, y, 0.136), bolt_steel)
        cyl(f"side_adapter_bolt_1_{i}", 0.010, 0.012, (0.315, y, 0.136), bolt_steel)

    # Main tower shell dimensions.  Front and rear are built as real framed
    # openings; side sheets close the hollow shaft.
    w = 0.46
    d = 0.38
    wall = 0.018
    bottom = 0.150
    height = 0.870
    top = bottom + height
    front_y = -d / 2.0 + wall / 2.0
    rear_y = d / 2.0 - wall / 2.0

    box("side_wall_0", (wall, d, height), (-w / 2.0 + wall / 2.0, 0.0, bottom + height / 2.0), galvanized)
    box("side_wall_1", (wall, d, height), (w / 2.0 - wall / 2.0, 0.0, bottom + height / 2.0), galvanized)

    # Front framed weather opening.
    box("front_post_0", (0.064, wall, height), (-0.198, front_y, bottom + height / 2.0), galvanized)
    box("front_post_1", (0.064, wall, height), (0.198, front_y, bottom + height / 2.0), galvanized)
    box("front_sill", (w, wall, 0.205), (0.0, front_y, bottom + 0.1025), galvanized)
    box("front_header", (w, wall, 0.205), (0.0, front_y, top - 0.1025), galvanized)
    box("front_inner_frame", (0.395, 0.012, 0.020), (0.0, front_y - 0.016, 0.372), dark_galv)
    box("front_top_frame", (0.395, 0.012, 0.020), (0.0, front_y - 0.016, 0.797), dark_galv)
    box("front_side_frame_0", (0.020, 0.012, 0.425), (-0.188, front_y - 0.016, 0.5845), dark_galv)
    box("front_side_frame_1", (0.020, 0.012, 0.425), (0.188, front_y - 0.016, 0.5845), dark_galv)

    for i, z in enumerate((0.470, 0.585, 0.700)):
        box(
            f"front_louver_{i}",
            (0.390, 0.032, 0.018),
            (0.0, front_y - 0.027, z),
            dark_galv,
            rpy=(math.radians(-18.0), 0.0, 0.0),
        )

    # Rear framed opening for cross-flow and visible retrofit register.
    box("rear_post_0", (0.072, wall, height), (-0.194, rear_y, bottom + height / 2.0), galvanized)
    box("rear_post_1", (0.072, wall, height), (0.194, rear_y, bottom + height / 2.0), galvanized)
    box("rear_sill", (w, wall, 0.260), (0.0, rear_y, bottom + 0.130), galvanized)
    box("rear_header", (w, wall, 0.250), (0.0, rear_y, top - 0.125), galvanized)
    box("rear_top_frame", (0.365, 0.012, 0.020), (0.0, rear_y + 0.010, 0.770), dark_galv)
    box("rear_inner_frame", (0.365, 0.012, 0.020), (0.0, rear_y + 0.010, 0.430), dark_galv)
    box("rear_side_frame_0", (0.020, 0.012, 0.340), (-0.174, rear_y + 0.010, 0.600), dark_galv)
    box("rear_side_frame_1", (0.020, 0.012, 0.340), (0.174, rear_y + 0.010, 0.600), dark_galv)
    for i, z in enumerate((0.505, 0.600, 0.695)):
        box(
            f"rear_louver_{i}",
            (0.365, 0.030, 0.018),
            (0.0, rear_y + 0.022, z),
            dark_galv,
            rpy=(math.radians(18.0), 0.0, 0.0),
        )

    # Top cap and seam rails keep the silhouette recognizable as a roof vent tower.
    box("cap_plate", (0.56, 0.48, 0.040), (0.0, 0.0, top + 0.020), galvanized)
    box("raised_cap_seam", (0.52, 0.035, 0.030), (0.0, 0.0, top + 0.055), dark_galv)
    box("front_drip_edge", (0.56, 0.024, 0.030), (0.0, -0.252, top + 0.002), dark_galv)
    box("rear_drip_edge", (0.56, 0.024, 0.030), (0.0, 0.252, top + 0.002), dark_galv)

    # Service hatches: bolted covers on the side walls, with raised rims and
    # handles but no invented articulation.
    box("service_hatch_0", (0.014, 0.210, 0.245), (w / 2.0 + 0.004, -0.075, 0.555), dark_galv)
    box("service_hatch_rim_0", (0.016, 0.245, 0.024), (w / 2.0 + 0.012, -0.075, 0.690), galvanized)
    box("service_hatch_rim_1", (0.016, 0.245, 0.024), (w / 2.0 + 0.012, -0.075, 0.420), galvanized)
    box("service_hatch_rim_2", (0.016, 0.024, 0.245), (w / 2.0 + 0.012, -0.202, 0.555), galvanized)
    box("service_hatch_rim_3", (0.016, 0.024, 0.245), (w / 2.0 + 0.012, 0.030, 0.555), galvanized)
    box("service_hatch_handle", (0.020, 0.105, 0.022), (w / 2.0 + 0.016, -0.075, 0.555), bolt_steel)

    box("filter_hatch", (0.014, 0.190, 0.185), (-w / 2.0 - 0.004, 0.070, 0.470), dark_galv)
    box("filter_hatch_handle", (0.020, 0.085, 0.020), (-w / 2.0 - 0.016, 0.070, 0.470), bolt_steel)

    for i, (y, z) in enumerate(((-0.170, 0.660), (0.020, 0.660), (-0.170, 0.450), (0.020, 0.450))):
        cyl(f"service_bolt_{i}", 0.007, 0.014, (w / 2.0 + 0.018, y, z), bolt_steel, rpy=(0.0, math.pi / 2.0, 0.0))
    for i, (y, z) in enumerate(((0.000, 0.540), (0.140, 0.540), (0.000, 0.400), (0.140, 0.400))):
        cyl(f"filter_bolt_{i}", 0.0065, 0.014, (-w / 2.0 - 0.018, y, z), bolt_steel, rpy=(0.0, math.pi / 2.0, 0.0))

    # Angle irons, strap plates, and gussets show practical reinforcement at
    # the adapter/body interface.
    for i, x in enumerate((-0.235, 0.235)):
        box(f"front_corner_strap_{i}", (0.026, 0.016, 0.860), (x, front_y - 0.020, bottom + height / 2.0), dark_galv)
        box(f"rear_corner_strap_{i}", (0.026, 0.016, 0.860), (x, rear_y + 0.014, bottom + height / 2.0), dark_galv)
    for i, x in enumerate((-0.180, 0.180)):
        box(f"base_gusset_{i}", (0.065, 0.075, 0.165), (x, -0.220, 0.195), primer)
        box(f"rear_gusset_{i}", (0.065, 0.075, 0.165), (x, 0.220, 0.195), primer)

    # Hinge hardware on the fixed tower: two outer knuckles, real leaves, a
    # closed stop rail, and open stop lugs.
    hinge_y = -0.224
    hinge_z = 0.810
    cyl("fixed_knuckle_0", 0.014, 0.130, (-0.160, hinge_y, hinge_z), dark_galv, rpy=ROT_CYL_X)
    cyl("fixed_knuckle_1", 0.014, 0.130, (0.160, hinge_y, hinge_z), dark_galv, rpy=ROT_CYL_X)
    box("fixed_leaf_0", (0.140, 0.032, 0.105), (-0.160, -0.205, 0.758), dark_galv)
    box("fixed_leaf_1", (0.140, 0.032, 0.105), (0.160, -0.205, 0.758), dark_galv)
    box("closed_stop_rail", (0.405, 0.025, 0.030), (0.0, -0.2115, 0.430), rubber)
    box("open_stop_lug_0", (0.034, 0.070, 0.085), (-0.238, -0.225, 0.845), rubber)
    box("open_stop_lug_1", (0.034, 0.070, 0.085), (0.238, -0.225, 0.845), rubber)

    for i, x in enumerate((-0.195, -0.125, 0.125, 0.195)):
        cyl(f"hinge_leaf_bolt_{i}", 0.0065, 0.008, (x, -0.224, 0.775), bolt_steel, rpy=ROT_CYL_Y)

    flap = model.part("flap")

    def flap_box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        flap.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def flap_cyl(
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        flap.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Child frame is exactly the hinge line.  The flap panel extends downward
    # from that line and sits proud of the old framed opening.
    flap_box("flap_panel", (0.500, 0.018, 0.340), (0.0, -0.010, -0.225), galvanized)
    flap_box("flap_outer_rib", (0.530, 0.026, 0.032), (0.0, -0.020, -0.050), dark_galv)
    flap_box("flap_bottom_rib", (0.530, 0.026, 0.030), (0.0, -0.020, -0.380), dark_galv)
    flap_box("flap_side_rib_0", (0.030, 0.026, 0.340), (-0.250, -0.020, -0.215), dark_galv)
    flap_box("flap_side_rib_1", (0.030, 0.026, 0.340), (0.250, -0.020, -0.215), dark_galv)
    flap_box("moving_leaf", (0.190, 0.013, 0.120), (0.0, -0.008, -0.060), dark_galv)
    flap_cyl("moving_knuckle", 0.014, 0.160, (0.0, 0.0, 0.0), dark_galv, rpy=ROT_CYL_X)
    flap_box("moving_stop_tab_0", (0.026, 0.032, 0.105), (-0.265, -0.018, -0.070), rubber)
    flap_box("moving_stop_tab_1", (0.026, 0.032, 0.105), (0.265, -0.018, -0.070), rubber)

    for i, (x, z) in enumerate(((-0.185, -0.105), (0.185, -0.105), (-0.185, -0.300), (0.185, -0.300))):
        flap_cyl(f"flap_rivet_{i}", 0.0065, 0.006, (x, -0.020, z), bolt_steel, rpy=ROT_CYL_Y)
    for i, x in enumerate((-0.060, 0.060)):
        flap_cyl(f"moving_leaf_bolt_{i}", 0.0065, 0.006, (x, -0.020, -0.055), bolt_steel, rpy=ROT_CYL_Y)

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=0.0, upper=1.15),
        motion_properties=MotionProperties(damping=0.35, friction=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("tower_to_flap")

    ctx.allow_overlap(
        flap,
        tower,
        elem_a="moving_stop_tab_0",
        elem_b="open_stop_lug_0",
        reason="At the upper limit the rubber moving stop tab intentionally compresses against the fixed open-stop lug.",
    )
    ctx.allow_overlap(
        flap,
        tower,
        elem_a="moving_stop_tab_1",
        elem_b="open_stop_lug_1",
        reason="At the upper limit the paired rubber stop tab intentionally compresses against the fixed open-stop lug.",
    )

    ctx.check(
        "primary flap hinge has practical stops",
        hinge.motion_limits.lower == 0.0 and 1.0 <= hinge.motion_limits.upper <= 1.25,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            tower,
            flap,
            axis="y",
            positive_elem="closed_stop_rail",
            negative_elem="flap_panel",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed flap rests just off rubber stop",
        )
        ctx.expect_overlap(
            flap,
            tower,
            axes="xz",
            elem_a="flap_panel",
            elem_b="front_side_frame_0",
            min_overlap=0.018,
            name="flap covers framed opening edge",
        )

        closed_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        ctx.expect_gap(
            tower,
            flap,
            axis="y",
            positive_elem="open_stop_lug_0",
            negative_elem="moving_stop_tab_0",
            max_penetration=0.030,
            name="open stop tab bears on fixed lug",
        )
        ctx.expect_gap(
            tower,
            flap,
            axis="y",
            positive_elem="open_stop_lug_1",
            negative_elem="moving_stop_tab_1",
            max_penetration=0.030,
            name="paired open stop bears on fixed lug",
        )
        open_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")

    ctx.check(
        "flap opens outward and upward",
        closed_panel is not None
        and open_panel is not None
        and open_panel[0][1] < closed_panel[0][1] - 0.10
        and open_panel[0][2] > closed_panel[0][2] + 0.10,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    return ctx.report()


object_model = build_object_model()
