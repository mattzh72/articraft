from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blower_graphics_card")

    pcb = Material("matte_black_pcb", rgba=(0.025, 0.035, 0.030, 1.0))
    gold = Material("gold_edge_contacts", rgba=(1.0, 0.72, 0.25, 1.0))
    shroud_mat = Material("satin_black_shroud", rgba=(0.015, 0.016, 0.018, 1.0))
    dark = Material("dark_grey_plastic", rgba=(0.07, 0.075, 0.08, 1.0))
    metal = Material("brushed_aluminium", rgba=(0.62, 0.64, 0.66, 1.0))
    fin_mat = Material("dull_aluminium_fins", rgba=(0.47, 0.49, 0.50, 1.0))
    screw_mat = Material("black_screw_heads", rgba=(0.004, 0.004, 0.004, 1.0))

    card = model.part("card")

    # Main board and edge connector.
    card.visual(
        Box((0.290, 0.112, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=pcb,
        name="pcb",
    )
    card.visual(
        Box((0.106, 0.010, 0.0024)),
        origin=Origin(xyz=(-0.010, -0.061, 0.0013)),
        material=gold,
        name="pcie_edge",
    )
    for i, x in enumerate((-0.060, -0.040, -0.020, 0.000, 0.020, 0.040)):
        card.visual(
            Box((0.007, 0.012, 0.0026)),
            origin=Origin(xyz=(x, -0.061, 0.00145)),
            material=pcb,
            name=f"edge_key_{i}",
        )

    # Low heatsink bed under the shroud tunnel.
    card.visual(
        Box((0.175, 0.084, 0.006)),
        origin=Origin(xyz=(0.035, 0.0, 0.005)),
        material=metal,
        name="heatsink_base",
    )
    for i, x in enumerate(
        (-0.044, -0.032, -0.020, -0.008, 0.004, 0.016, 0.028, 0.040, 0.052, 0.064, 0.076, 0.088, 0.100, 0.112)
    ):
        card.visual(
            Box((0.0032, 0.076, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.020)),
            material=fin_mat,
            name=f"fin_{i}",
        )

    # Rectangular covered heatsink tunnel, hollow at the rear exhaust end.
    card.visual(
        Box((0.168, 0.098, 0.004)),
        origin=Origin(xyz=(0.048, 0.0, 0.038)),
        material=shroud_mat,
        name="tunnel_roof",
    )
    for i, y in enumerate((-0.051, 0.051)):
        card.visual(
            Box((0.168, 0.004, 0.036)),
            origin=Origin(xyz=(0.048, y, 0.020)),
            material=shroud_mat,
            name=f"tunnel_side_{i}",
        )
    card.visual(
        Box((0.018, 0.098, 0.036)),
        origin=Origin(xyz=(-0.036, 0.0, 0.020)),
        material=shroud_mat,
        name="blower_to_tunnel_throat",
    )

    # Circular volute/intake housing around the centrifugal blower.
    intake_ring = BezelGeometry(
        (0.086, 0.086),
        (0.126, 0.126),
        0.020,
        opening_shape="circle",
        outer_shape="circle",
        face=None,
        center=True,
    )
    card.visual(
        mesh_from_geometry(intake_ring, "volute_intake_ring"),
        origin=Origin(xyz=(-0.095, 0.0, 0.028)),
        material=shroud_mat,
        name="intake_ring",
    )
    # Tangential scroll tongue and dark recess show that this is a blower volute,
    # not merely an axial fan guard.
    card.visual(
        Box((0.052, 0.016, 0.018)),
        origin=Origin(xyz=(-0.052, 0.040, 0.027)),
        material=shroud_mat,
        name="volute_tongue",
    )
    card.visual(
        Box((0.076, 0.028, 0.003)),
        origin=Origin(xyz=(-0.049, 0.0, 0.0375)),
        material=dark,
        name="outlet_shadow",
    )

    # Fixed central shaft and retainers.  The rotating impeller has an annular
    # hub around this shaft; the upper cap clips it in without touching at rest.
    card.visual(
        Cylinder(radius=0.0045, length=0.035),
        origin=Origin(xyz=(-0.095, 0.0, 0.0195)),
        material=metal,
        name="hub_shaft",
    )
    card.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(-0.095, 0.0, 0.0155)),
        material=metal,
        name="lower_bearing",
    )
    card.visual(
        Cylinder(radius=0.010, length=0.0036),
        origin=Origin(xyz=(-0.095, 0.0, 0.0360)),
        material=metal,
        name="retainer_cap",
    )

    # Rear expansion bracket with a real exhaust slot/grille at the tunnel exit.
    card.visual(
        Box((0.006, 0.0165, 0.070)),
        origin=Origin(xyz=(0.148, -0.054, 0.037)),
        material=metal,
        name="bracket_side_0",
    )
    card.visual(
        Box((0.006, 0.0165, 0.070)),
        origin=Origin(xyz=(0.148, 0.054, 0.037)),
        material=metal,
        name="bracket_side_1",
    )
    card.visual(
        Box((0.006, 0.125, 0.016)),
        origin=Origin(xyz=(0.148, 0.0, 0.010)),
        material=metal,
        name="bracket_lower_rail",
    )
    card.visual(
        Box((0.006, 0.125, 0.016)),
        origin=Origin(xyz=(0.148, 0.0, 0.064)),
        material=metal,
        name="bracket_upper_rail",
    )
    exhaust = VentGrilleGeometry(
        (0.090, 0.035),
        frame=0.006,
        face_thickness=0.0025,
        duct_depth=0.010,
        duct_wall=0.002,
        slat_pitch=0.0075,
        slat_width=0.003,
        slat_angle_deg=25.0,
        corner_radius=0.002,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1, divider_width=0.003),
        frame_profile=VentGrilleFrame(style="beveled", depth=0.0008),
    )
    card.visual(
        mesh_from_geometry(exhaust, "rear_exhaust_grille"),
        origin=Origin(xyz=(0.144, 0.0, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="rear_exhaust",
    )

    # Small fasteners and board hardware, seated on real surfaces.
    for i, (x, y) in enumerate(((-0.125, -0.043), (-0.125, 0.043), (0.108, -0.043), (0.108, 0.043))):
        card.visual(
            Cylinder(radius=0.0042, length=0.002),
            origin=Origin(xyz=(x, y, 0.039)),
            material=screw_mat,
            name=f"shroud_screw_{i}",
        )
    for i, (x, y) in enumerate(((0.020, -0.047), (0.055, -0.047), (0.090, -0.047))):
        card.visual(
            Cylinder(radius=0.003, length=0.010),
            origin=Origin(xyz=(x, y, 0.007)),
            material=dark,
            name=f"capacitor_{i}",
        )

    impeller = model.part("impeller")
    blower_wheel = BlowerWheelGeometry(
        0.037,
        0.014,
        0.014,
        26,
        blade_thickness=0.0018,
        blade_sweep_deg=34.0,
        backplate=True,
        shroud=True,
        center=True,
    )
    impeller.visual(
        mesh_from_geometry(blower_wheel, "blower_wheel"),
        origin=Origin(),
        material=dark,
        name="blower_wheel",
    )
    rotor_hub = BezelGeometry(
        (0.012, 0.012),
        (0.032, 0.032),
        0.016,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    impeller.visual(
        mesh_from_geometry(rotor_hub, "rotor_hub_ring"),
        origin=Origin(),
        material=dark,
        name="rotor_hub",
    )

    model.articulation(
        "blower_spin",
        ArticulationType.CONTINUOUS,
        parent=card,
        child=impeller,
        origin=Origin(xyz=(-0.095, 0.0, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=260.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    card = object_model.get_part("card")
    impeller = object_model.get_part("impeller")
    spin = object_model.get_articulation("blower_spin")

    joint_type = getattr(spin, "articulation_type", getattr(spin, "type", None))
    ctx.check(
        "blower impeller has continuous rotation",
        joint_type == ArticulationType.CONTINUOUS or str(joint_type).endswith("CONTINUOUS"),
        details=f"joint_type={joint_type}",
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_within(
            impeller,
            card,
            axes="xy",
            inner_elem="blower_wheel",
            outer_elem="intake_ring",
            margin=0.001,
            name="impeller sits inside the volute ring footprint",
        )
        ctx.expect_overlap(
            impeller,
            card,
            axes="z",
            elem_a="blower_wheel",
            elem_b="intake_ring",
            min_overlap=0.010,
            name="impeller spins within the fixed volute depth",
        )
        ctx.expect_within(
            card,
            impeller,
            axes="xy",
            inner_elem="hub_shaft",
            outer_elem="rotor_hub",
            margin=0.001,
            name="central shaft stays inside the annular rotor hub",
        )
        ctx.expect_overlap(
            card,
            impeller,
            axes="z",
            elem_a="hub_shaft",
            elem_b="rotor_hub",
            min_overlap=0.014,
            name="shaft passes through the rotor hub height",
        )
        ctx.expect_gap(
            card,
            impeller,
            axis="z",
            positive_elem="retainer_cap",
            negative_elem="rotor_hub",
            min_gap=0.00002,
            max_gap=0.001,
            name="retainer cap clips the hub with a running clearance",
        )

    rest_pos = ctx.part_world_position(impeller)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(impeller)
        ctx.expect_within(
            impeller,
            card,
            axes="xy",
            inner_elem="blower_wheel",
            outer_elem="intake_ring",
            margin=0.001,
            name="rotated impeller remains captured in the volute",
        )
        ctx.expect_gap(
            card,
            impeller,
            axis="z",
            positive_elem="retainer_cap",
            negative_elem="rotor_hub",
            min_gap=0.00002,
            max_gap=0.001,
            name="rotated hub remains below the retainer cap",
        )

    ctx.check(
        "continuous spin does not translate the impeller",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-7,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
