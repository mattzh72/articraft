from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


def _gable_cap_geometry(depth: float, width: float, skirt: float, rise: float) -> MeshGeometry:
    """Simple closed gabled rain cap with a vertical drip skirt."""
    hx = depth / 2.0
    hy = width / 2.0
    g = MeshGeometry()
    # Bottom rectangle, eave skirt top rectangle, and ridge points.
    pts = [
        (-hx, -hy, 0.0),
        (hx, -hy, 0.0),
        (hx, hy, 0.0),
        (-hx, hy, 0.0),
        (-hx, -hy, skirt),
        (hx, -hy, skirt),
        (hx, hy, skirt),
        (-hx, hy, skirt),
        (0.0, -hy, rise),
        (0.0, hy, rise),
    ]
    for p in pts:
        g.add_vertex(*p)
    for face in (
        (0, 1, 2),
        (0, 2, 3),
        (0, 4, 7),
        (0, 7, 3),
        (1, 2, 6),
        (1, 6, 5),
        (0, 1, 5),
        (0, 5, 8),
        (0, 8, 4),
        (3, 7, 9),
        (3, 9, 6),
        (3, 6, 2),
        (4, 8, 9),
        (4, 9, 7),
        (8, 5, 6),
        (8, 6, 9),
    ):
        g.add_face(*face)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_roof_vent_tower")

    galvanized = model.material("galvanized_zinc", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_inside = model.material("dark_vent_void", rgba=(0.035, 0.04, 0.045, 1.0))
    gasket = model.material("black_epdm_gasket", rgba=(0.01, 0.012, 0.012, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.82, 0.82, 0.78, 1.0))
    rubber = model.material("dark_rubber_stop", rgba=(0.015, 0.015, 0.014, 1.0))

    tower = model.part("tower")

    # Broad roof flashing and raised curb: the tower has a practical footprint and
    # a low sealed base rather than an arbitrary pedestal.
    tower.visual(
        Box((0.86, 0.70, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=galvanized,
        name="roof_flashing",
    )
    tower.visual(
        Box((0.66, 0.52, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=galvanized,
        name="raised_base_seam",
    )
    tower.visual(
        Box((0.34, 0.42, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_inside,
        name="vent_throat_shadow",
    )

    # Hollow tower shell with a real front opening, side walls, and rear wall.
    wall_h = 0.690
    wall_z = 0.047 + wall_h / 2.0
    tower.visual(
        Box((0.36, 0.040, wall_h)),
        origin=Origin(xyz=(0.0, -0.250, wall_z)),
        material=galvanized,
        name="side_wall_0",
    )
    tower.visual(
        Box((0.36, 0.040, wall_h)),
        origin=Origin(xyz=(0.0, 0.250, wall_z)),
        material=galvanized,
        name="side_wall_1",
    )
    tower.visual(
        Box((0.040, 0.500, wall_h)),
        origin=Origin(xyz=(0.180, 0.0, wall_z)),
        material=galvanized,
        name="rear_wall",
    )
    tower.visual(
        Box((0.044, 0.500, 0.165)),
        origin=Origin(xyz=(-0.180, 0.0, 0.128)),
        material=galvanized,
        name="front_sill_wall",
    )
    tower.visual(
        Box((0.044, 0.500, 0.130)),
        origin=Origin(xyz=(-0.180, 0.0, 0.672)),
        material=galvanized,
        name="front_head_wall",
    )
    tower.visual(
        Box((0.044, 0.080, 0.415)),
        origin=Origin(xyz=(-0.180, -0.210, 0.392)),
        material=galvanized,
        name="front_jamb_0",
    )
    tower.visual(
        Box((0.044, 0.080, 0.415)),
        origin=Origin(xyz=(-0.180, 0.210, 0.392)),
        material=galvanized,
        name="front_jamb_1",
    )

    # Proud framed front opening and continuous EPDM sealing land.
    tower.visual(
        Box((0.055, 0.485, 0.055)),
        origin=Origin(xyz=(-0.211, 0.0, 0.613)),
        material=galvanized,
        name="front_frame_top",
    )
    tower.visual(
        Box((0.055, 0.485, 0.055)),
        origin=Origin(xyz=(-0.211, 0.0, 0.183)),
        material=galvanized,
        name="front_frame_sill",
    )
    tower.visual(
        Box((0.055, 0.055, 0.440)),
        origin=Origin(xyz=(-0.211, -0.218, 0.398)),
        material=galvanized,
        name="front_frame_side_0",
    )
    tower.visual(
        Box((0.055, 0.055, 0.440)),
        origin=Origin(xyz=(-0.211, 0.218, 0.398)),
        material=galvanized,
        name="front_frame_side_1",
    )
    tower.visual(
        Box((0.012, 0.382, 0.018)),
        origin=Origin(xyz=(-0.244, 0.0, 0.584)),
        material=gasket,
        name="top_gasket",
    )
    tower.visual(
        Box((0.012, 0.382, 0.018)),
        origin=Origin(xyz=(-0.244, 0.0, 0.212)),
        material=gasket,
        name="bottom_gasket",
    )
    tower.visual(
        Box((0.012, 0.018, 0.385)),
        origin=Origin(xyz=(-0.244, -0.190, 0.398)),
        material=gasket,
        name="side_gasket_0",
    )
    tower.visual(
        Box((0.012, 0.018, 0.385)),
        origin=Origin(xyz=(-0.244, 0.190, 0.398)),
        material=gasket,
        name="side_gasket_1",
    )
    tower.visual(
        Box((0.118, 0.600, 0.034)),
        origin=Origin(xyz=(-0.241, 0.0, 0.664)),
        material=galvanized,
        name="front_drip_hood",
    )

    # Rubber closed stops just proud of the frame; the flap bears on these at the
    # end of travel instead of on bare sheet metal.
    for idx, y in enumerate((-0.188, 0.188)):
        tower.visual(
            Box((0.005, 0.025, 0.075)),
            origin=Origin(xyz=(-0.247, y * 1.09, 0.306)),
            material=rubber,
            name=f"closed_stop_{idx}",
        )

    # Side louver grilles give the tower secondary framed openings while keeping
    # slats rain-shedding and protected inside the side faces.
    side_grille_mesh = mesh_from_geometry(
        VentGrilleGeometry(
            (0.205, 0.155),
            frame=0.014,
            face_thickness=0.004,
            duct_depth=0.020,
            duct_wall=0.003,
            slat_pitch=0.024,
            slat_width=0.010,
            slat_angle_deg=35.0,
            slats=VentGrilleSlats(profile="boxed", direction="down", divider_count=1),
            frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
            sleeve=VentGrilleSleeve(style="short", depth=0.018),
        ),
        "side_louver_grille",
    )
    tower.visual(
        side_grille_mesh,
        origin=Origin(xyz=(0.025, -0.274, 0.430), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="side_louver_0",
    )
    tower.visual(
        side_grille_mesh,
        origin=Origin(xyz=(0.025, 0.274, 0.430), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="side_louver_1",
    )

    # Overhanging gable cap with a skirt and small drip ribs under the eaves.
    tower.visual(
        mesh_from_geometry(_gable_cap_geometry(0.535, 0.650, 0.026, 0.086), "gabled_rain_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.733)),
        material=galvanized,
        name="gabled_rain_cap",
    )
    tower.visual(
        Box((0.018, 0.650, 0.018)),
        origin=Origin(xyz=(-0.269, 0.0, 0.730)),
        material=galvanized,
        name="cap_front_drip_edge",
    )
    tower.visual(
        Box((0.018, 0.650, 0.018)),
        origin=Origin(xyz=(0.269, 0.0, 0.730)),
        material=galvanized,
        name="cap_rear_drip_edge",
    )
    tower.visual(
        Box((0.535, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.326, 0.730)),
        material=galvanized,
        name="cap_side_drip_0",
    )
    tower.visual(
        Box((0.535, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.326, 0.730)),
        material=galvanized,
        name="cap_side_drip_1",
    )

    # Visible stainless fasteners on the roof flashing and hinge leaves.
    for idx, (x, y) in enumerate(
        [(-0.330, -0.250), (-0.330, 0.250), (0.330, -0.250), (0.330, 0.250)]
    ):
        tower.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(x, y, 0.036), rpy=(0.0, 0.0, 0.0)),
            material=stainless,
            name=f"flashing_screw_{idx}",
        )

    # Fixed half of the exposed hinge: three protected knuckles and their plates.
    hinge_y = [-0.180, 0.0, 0.180]
    for idx, y in enumerate(hinge_y):
        fixed_barrel_name = (
            "fixed_hinge_barrel_0"
            if idx == 0
            else "fixed_hinge_barrel_1"
            if idx == 1
            else "fixed_hinge_barrel_2"
        )
        tower.visual(
            Box((0.025, 0.082, 0.052)),
            origin=Origin(xyz=(-0.237, y, 0.625)),
            material=stainless,
            name=f"fixed_hinge_plate_{idx}",
        )
        tower.visual(
            Cylinder(radius=0.016, length=0.080),
            origin=Origin(xyz=(-0.263, y, 0.625), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=fixed_barrel_name,
        )
        tower.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(-0.249, y - 0.022, 0.637), rpy=(0.0, pi / 2.0, 0.0)),
            material=stainless,
            name=f"fixed_hinge_screw_{idx}_0",
        )
        tower.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(-0.249, y + 0.022, 0.613), rpy=(0.0, pi / 2.0, 0.0)),
            material=stainless,
            name=f"fixed_hinge_screw_{idx}_1",
        )

    # Open-limit stop ears are welded to the side of the frame, not left floating.
    for idx, y in enumerate((-0.276, 0.276)):
        tower.visual(
            Box((0.060, 0.028, 0.118)),
            origin=Origin(xyz=(-0.207, y, 0.590)),
            material=galvanized,
            name=f"open_stop_ear_{idx}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.026, 0.450, 0.400)),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=galvanized,
        name="flap_panel",
    )
    flap.visual(
        Box((0.075, 0.520, 0.030)),
        origin=Origin(xyz=(-0.030, 0.0, -0.055)),
        material=galvanized,
        name="flap_top_drip_lip",
    )
    flap.visual(
        Box((0.050, 0.018, 0.390)),
        origin=Origin(xyz=(-0.025, -0.234, -0.240)),
        material=galvanized,
        name="flap_side_return_0",
    )
    flap.visual(
        Box((0.050, 0.018, 0.390)),
        origin=Origin(xyz=(-0.025, 0.234, -0.240)),
        material=galvanized,
        name="flap_side_return_1",
    )
    flap.visual(
        Box((0.066, 0.490, 0.030)),
        origin=Origin(xyz=(-0.025, 0.0, -0.435)),
        material=galvanized,
        name="flap_bottom_drip_hem",
    )
    flap.visual(
        Box((0.010, 0.340, 0.016)),
        origin=Origin(xyz=(0.017, 0.0, -0.065)),
        material=gasket,
        name="flap_top_gasket",
    )
    flap.visual(
        Box((0.010, 0.340, 0.016)),
        origin=Origin(xyz=(0.017, 0.0, -0.390)),
        material=gasket,
        name="flap_bottom_gasket",
    )
    flap.visual(
        Box((0.010, 0.016, 0.325)),
        origin=Origin(xyz=(0.017, -0.170, -0.227)),
        material=gasket,
        name="flap_side_gasket_0",
    )
    flap.visual(
        Box((0.010, 0.016, 0.325)),
        origin=Origin(xyz=(0.017, 0.170, -0.227)),
        material=gasket,
        name="flap_side_gasket_1",
    )

    # Moving half of the hinge: two interleaved barrels and leaf straps that are
    # physically riveted to the flap panel.
    for idx, y in enumerate((-0.090, 0.090)):
        moving_barrel_name = "moving_hinge_barrel_0" if idx == 0 else "moving_hinge_barrel_1"
        flap.visual(
            Box((0.026, 0.076, 0.060)),
            origin=Origin(xyz=(-0.005, y, -0.030)),
            material=stainless,
            name=f"moving_hinge_plate_{idx}",
        )
        flap.visual(
            Cylinder(radius=0.016, length=0.076),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=moving_barrel_name,
        )
        flap.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(-0.016, y - 0.022, -0.052), rpy=(0.0, pi / 2.0, 0.0)),
            material=stainless,
            name=f"moving_hinge_screw_{idx}_0",
        )
        flap.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(-0.016, y + 0.022, -0.018), rpy=(0.0, pi / 2.0, 0.0)),
            material=stainless,
            name=f"moving_hinge_screw_{idx}_1",
        )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=flap,
        origin=Origin(xyz=(-0.263, 0.0, 0.625)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.05),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("flap_hinge")

    ctx.expect_overlap(
        flap,
        tower,
        axes="y",
        elem_a="flap_panel",
        elem_b="top_gasket",
        min_overlap=0.34,
        name="closed flap covers sealed opening width",
    )
    ctx.expect_gap(
        tower,
        flap,
        axis="x",
        positive_elem="top_gasket",
        negative_elem="flap_panel",
        max_penetration=0.0,
        max_gap=0.004,
        name="flap sits just proud of top gasket",
    )

    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({hinge: 1.05}):
        open_aabb = ctx.part_world_aabb(flap)
        ctx.expect_overlap(
            flap,
            tower,
            axes="xz",
            elem_a="moving_hinge_barrel_0",
            elem_b="fixed_hinge_barrel_1",
            min_overlap=0.015,
            name="barrels stay aligned on hinge line",
        )

    ctx.check(
        "hinged flap opens outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.055
        and open_aabb[0][2] > closed_aabb[0][2] + 0.055,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
