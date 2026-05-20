from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
    ExtrudeGeometry,
)


WAND_TILT = math.atan2(0.34, -0.67)


def _body_shell():
    """Rounded canister hull, authored as a superellipse loft in real scale."""
    sections = (
        (-0.285, 0.070, 0.240, 0.225),
        (-0.185, 0.030, 0.305, 0.335),
        (0.020, 0.025, 0.345, 0.365),
        (0.190, 0.045, 0.315, 0.335),
        (0.285, 0.075, 0.250, 0.245),
    )
    return superellipse_side_loft(sections, exponents=2.75, segments=72).rotate_z(
        -math.pi / 2.0
    )


def _hollow_sleeve(outer_radius: float, inner_radius: float, length: float):
    outer = [
        (outer_radius, 0.000),
        (outer_radius, length),
    ]
    inner = [
        (inner_radius, 0.012),
        (inner_radius, length - 0.012),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=48,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_canister_vacuum")

    shell_mat = model.material("deep_blue_shell", rgba=(0.04, 0.08, 0.13, 1.0))
    black = model.material("soft_black", rgba=(0.01, 0.011, 0.012, 1.0))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.10, 0.11, 0.115, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.74, 0.72, 1.0))
    blue_trim = model.material("blue_trim", rgba=(0.05, 0.28, 0.65, 1.0))
    clear_bin = model.material("smoked_clear_bin", rgba=(0.53, 0.72, 0.83, 0.46))
    red = model.material("red_release", rgba=(0.85, 0.05, 0.025, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell(), "canister_rounded_shell"),
        material=shell_mat,
        name="rounded_shell",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.230),
        origin=Origin(xyz=(0.085, 0.0, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_bin,
        name="clear_dust_bin",
    )
    body.visual(
        Cylinder(radius=0.111, length=0.020),
        origin=Origin(xyz=(-0.040, 0.0, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="bin_rear_cap",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.084),
        origin=Origin(xyz=(0.292, 0.0, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="hose_socket",
    )
    body.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.337, 0.0, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="socket_gasket",
    )
    body.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.165, 0.0, 0.305), (-0.105, 0.0, 0.425), (0.150, 0.0, 0.425), (0.230, 0.0, 0.315)],
                radius=0.017,
                samples_per_segment=18,
                radial_segments=20,
            ),
            "body_carry_handle",
        ),
        material=black,
        name="carry_handle",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.150, 0.075),
                frame=0.010,
                face_thickness=0.004,
                duct_depth=0.018,
                slat_pitch=0.014,
                slat_width=0.006,
                slat_angle_deg=28.0,
                corner_radius=0.006,
                slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.001),
            ),
            "side_exhaust_grille",
        ),
        origin=Origin(xyz=(0.015, -0.187, 0.210), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="side_exhaust_grille",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(-0.080, 0.0, 0.350)),
        material=dark_grey,
        name="power_button_well",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.079,
            0.040,
            inner_radius=0.049,
            tread=TireTread(style="ribbed", depth=0.0035, count=24, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.006, radius=0.0025),
        ),
        "rear_wheel_tire",
    )
    for y, suffix in ((0.183, "0"), (-0.183, "1")):
        body.visual(
            tire_mesh,
            origin=Origin(xyz=(-0.195, y, 0.079), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name=f"rear_wheel_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.029, length=0.046),
            origin=Origin(xyz=(-0.195, y, 0.079), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=blue_trim,
            name=f"wheel_hub_{suffix}",
        )
    body.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.210, 0.0, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_caster_wheel",
    )
    body.visual(
        Box((0.060, 0.090, 0.030)),
        origin=Origin(xyz=(0.210, 0.0, 0.072)),
        material=dark_grey,
        name="caster_fork",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=red,
        name="button_cap",
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(-0.080, 0.0, 0.355)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.006, upper=0.0, effort=8.0, velocity=0.05),
    )

    hose = model.part("hose")
    hose_path = [
        (0.070, 0.0, 0.000),
        (0.145, 0.0, 0.225),
        (0.360, 0.0, 0.570),
        (0.500, 0.0, 0.638),
    ]
    hose.visual(
        Cylinder(radius=0.039, length=0.080),
        origin=Origin(xyz=(0.049, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="body_coupler",
    )
    hose.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                hose_path,
                radius=0.024,
                samples_per_segment=18,
                radial_segments=24,
            ),
            "flexible_hose_tube",
        ),
        material=black,
        name="corrugated_tube",
    )
    ridge_mesh = mesh_from_geometry(TorusGeometry(0.026, 0.0038, radial_segments=16, tubular_segments=36), "hose_ridge")
    for i, (x, z, angle) in enumerate(
        [
            (0.115, 0.110, 0.32),
            (0.145, 0.198, 0.32),
            (0.195, 0.300, 0.50),
            (0.245, 0.390, 0.50),
            (0.300, 0.485, 0.50),
            (0.365, 0.565, 0.80),
            (0.440, 0.615, 1.05),
            (0.505, 0.648, 1.18),
        ]
    ):
        hose.visual(
            ridge_mesh,
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, angle, 0.0)),
            material=dark_grey,
            name=f"hose_ridge_{i}",
        )
    hose.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(0.512, 0.0, 0.646), rpy=(0.0, 1.12, 0.0)),
        material=dark_grey,
        name="handle_coupler",
    )
    model.articulation(
        "body_to_hose",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hose,
        origin=Origin(xyz=(0.334, 0.0, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=4.0, velocity=1.2),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.035, length=0.075),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="hose_receiver",
    )
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.055, 0.0, -0.020), (0.066, 0.0, -0.066), (0.080, 0.0, -0.120)],
                radius=0.026,
                samples_per_segment=14,
                radial_segments=22,
            ),
            "angled_wand_socket",
        ),
        material=dark_grey,
        name="wand_socket",
    )
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.020, 0.0, 0.030),
                    (-0.050, 0.0, 0.122),
                    (0.035, 0.0, 0.195),
                    (0.160, 0.0, 0.075),
                    (0.090, 0.0, -0.062),
                ],
                radius=0.016,
                samples_per_segment=18,
                radial_segments=20,
            ),
            "ergonomic_handle_loop",
        ),
        material=black,
        name="grip_loop",
    )
    handle.visual(
        Box((0.105, 0.034, 0.040)),
        origin=Origin(xyz=(0.095, 0.0, 0.096), rpy=(0.0, 0.0, 0.0)),
        material=black,
        name="rubber_grip_pad",
    )
    model.articulation(
        "hose_to_handle",
        ArticulationType.FIXED,
        parent=hose,
        child=handle,
        origin=Origin(xyz=(0.540, 0.0, 0.660)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.035, 0.022, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, 0.006)),
        material=red,
        name="trigger_paddle",
    )
    model.articulation(
        "handle_to_trigger",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=trigger,
        origin=Origin(xyz=(0.060, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.004, upper=0.0, effort=3.0, velocity=0.06),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        mesh_from_geometry(_hollow_sleeve(0.023, 0.0185, 0.345), "upper_wand_hollow_sleeve"),
        material=aluminum,
        name="upper_sleeve",
    )
    upper_wand.visual(
        Cylinder(radius=0.031, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_grey,
        name="upper_socket_collar",
    )
    upper_wand.visual(
        mesh_from_geometry(_hollow_sleeve(0.030, 0.0185, 0.038), "telescoping_lock_collar_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=blue_trim,
        name="telescoping_lock_collar",
    )
    upper_wand.visual(
        Box((0.018, 0.010, 0.028)),
        origin=Origin(xyz=(0.024, 0.0, 0.312)),
        material=red,
        name="lock_release_button",
    )
    model.articulation(
        "handle_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=upper_wand,
        origin=Origin(xyz=(0.080, 0.0, -0.120), rpy=(0.0, WAND_TILT, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.45, effort=12.0, velocity=1.3),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.0145, length=0.605),
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
        material=aluminum,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=dark_grey,
        name="lower_slide_collar",
    )
    lower_wand.visual(
        Cylinder(radius=0.021, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.438)),
        material=dark_grey,
        name="floor_connector_pin",
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.PRISMATIC,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=50.0, velocity=0.18),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.026, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="pivot_barrel",
    )
    floor_head.visual(
        Box((0.086, 0.022, 0.066)),
        origin=Origin(xyz=(0.018, 0.064, -0.030)),
        material=dark_grey,
        name="pivot_cheek_0",
    )
    floor_head.visual(
        Box((0.086, 0.022, 0.066)),
        origin=Origin(xyz=(0.018, -0.064, -0.030)),
        material=dark_grey,
        name="pivot_cheek_1",
    )
    floor_head.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.345, 0.292, 0.036), 0.065, cap=True, center=True),
            "floor_head_rounded_nozzle",
        ),
        origin=Origin(xyz=(0.220, 0.0, -0.010)),
        material=dark_grey,
        name="nozzle_shell",
    )
    floor_head.visual(
        Box((0.310, 0.020, 0.026)),
        origin=Origin(xyz=(0.225, 0.150, 0.003)),
        material=black,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.310, 0.020, 0.026)),
        origin=Origin(xyz=(0.225, -0.150, 0.003)),
        material=black,
        name="rear_bumper",
    )
    floor_head.visual(
        Box((0.215, 0.238, 0.010)),
        origin=Origin(xyz=(0.227, 0.0, 0.025)),
        material=blue_trim,
        name="transparent_brush_window",
    )
    floor_head.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.235, 0.080),
                0.004,
                slot_size=(0.030, 0.006),
                pitch=(0.040, 0.016),
                frame=0.010,
                corner_radius=0.006,
                slot_angle_deg=0.0,
            ),
            "nozzle_suction_slot_plate",
        ),
        origin=Origin(xyz=(0.230, 0.0, -0.048)),
        material=black,
        name="suction_slot_plate",
    )
    floor_head.visual(
        Cylinder(radius=0.016, length=0.280),
        origin=Origin(xyz=(0.195, 0.0, -0.051), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_trim,
        name="brush_roll",
    )
    for y, suffix in ((0.117, "0"), (-0.117, "1")):
        floor_head.visual(
            Cylinder(radius=0.022, length=0.030),
            origin=Origin(xyz=(0.040, y, -0.029), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"nozzle_wheel_{suffix}",
        )
    model.articulation(
        "lower_wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, 0.460), rpy=(0.0, -WAND_TILT, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.65, effort=14.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hose = object_model.get_part("hose")
    handle = object_model.get_part("handle")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_head = object_model.get_part("floor_head")
    power_button = object_model.get_part("power_button")

    hose_swivel = object_model.get_articulation("body_to_hose")
    wand_hinge = object_model.get_articulation("handle_to_upper_wand")
    wand_slide = object_model.get_articulation("upper_to_lower_wand")
    floor_hinge = object_model.get_articulation("lower_wand_to_floor_head")
    power_press = object_model.get_articulation("body_to_power_button")

    ctx.allow_overlap(
        lower_wand,
        floor_head,
        elem_a="floor_connector_pin",
        elem_b="pivot_barrel",
        reason="The nozzle pitch trunnion is modeled as a captured pin seated through the floor-head pivot barrel.",
    )
    ctx.allow_overlap(
        handle,
        upper_wand,
        elem_a="wand_socket",
        elem_b="upper_sleeve",
        reason="The upper wand is intentionally seated a short distance inside the molded handle socket.",
    )
    ctx.allow_overlap(
        hose,
        handle,
        elem_a="handle_coupler",
        elem_b="hose_receiver",
        reason="The hose coupling is intentionally inserted into the handle receiver socket.",
    )
    ctx.allow_overlap(
        upper_wand,
        lower_wand,
        elem_a="telescoping_lock_collar",
        elem_b="lower_slide_collar",
        reason="The lower wand collar is captured under the locking cuff, represented with a small hidden seated overlap.",
    )

    ctx.expect_contact(body, hose, elem_a="socket_gasket", elem_b="body_coupler", contact_tol=0.003, name="hose coupler seats in body socket")
    ctx.expect_contact(hose, handle, elem_a="handle_coupler", elem_b="hose_receiver", contact_tol=0.004, name="hose connects to handle receiver")
    ctx.expect_contact(handle, upper_wand, elem_a="wand_socket", elem_b="upper_sleeve", contact_tol=0.006, name="handle socket carries upper wand")
    ctx.expect_overlap(handle, upper_wand, axes="xz", elem_a="wand_socket", elem_b="upper_sleeve", min_overlap=0.015, name="upper wand remains captured in handle socket")
    ctx.expect_within(lower_wand, upper_wand, axes="y", inner_elem="lower_tube", outer_elem="upper_sleeve", margin=0.006, name="telescoping tube stays centered in sleeve")
    ctx.expect_overlap(lower_wand, upper_wand, axes="z", elem_a="lower_tube", elem_b="upper_sleeve", min_overlap=0.080, name="collapsed wand retains sleeve insertion")
    ctx.expect_overlap(lower_wand, upper_wand, axes="xy", elem_a="lower_slide_collar", elem_b="telescoping_lock_collar", min_overlap=0.020, name="locking cuff captures lower wand collar")
    ctx.expect_overlap(lower_wand, floor_head, axes="xy", elem_a="floor_connector_pin", elem_b="pivot_barrel", min_overlap=0.015, name="floor head pivot captures wand pin")
    ctx.expect_gap(power_button, body, axis="z", min_gap=-0.002, max_gap=0.010, positive_elem="button_cap", negative_elem="power_button_well", name="power button sits in its well")

    rest_lower = ctx.part_world_position(lower_wand)
    with ctx.pose({wand_slide: 0.120}):
        extended_lower = ctx.part_world_position(lower_wand)
        ctx.expect_overlap(lower_wand, upper_wand, axes="z", elem_a="lower_tube", elem_b="upper_sleeve", min_overlap=0.020, name="extended wand keeps retained insertion")
    ctx.check(
        "telescoping wand extends along the tube",
        rest_lower is not None
        and extended_lower is not None
        and extended_lower[0] > rest_lower[0] + 0.035
        and extended_lower[2] < rest_lower[2] - 0.070,
        details=f"rest={rest_lower}, extended={extended_lower}",
    )

    rest_head_aabb = ctx.part_world_aabb(floor_head)
    with ctx.pose({floor_hinge: 0.55}):
        pitched_head_aabb = ctx.part_world_aabb(floor_head)
    ctx.check(
        "floor head pitch changes nozzle attitude",
        rest_head_aabb is not None
        and pitched_head_aabb is not None
        and (
            abs(pitched_head_aabb[0][2] - rest_head_aabb[0][2]) > 0.030
            or abs(pitched_head_aabb[1][2] - rest_head_aabb[1][2]) > 0.030
            or abs(pitched_head_aabb[0][0] - rest_head_aabb[0][0]) > 0.030
        ),
        details=f"rest={rest_head_aabb}, pitched={pitched_head_aabb}",
    )

    rest_wand_pos = ctx.part_world_position(floor_head)
    with ctx.pose({wand_hinge: 0.30}):
        tilted_wand_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "handle hinge changes wand working angle",
        rest_wand_pos is not None
        and tilted_wand_pos is not None
        and abs(tilted_wand_pos[0] - rest_wand_pos[0]) > 0.04,
        details=f"rest={rest_wand_pos}, tilted={tilted_wand_pos}",
    )

    with ctx.pose({hose_swivel: 0.45}):
        swung_handle = ctx.part_world_position(handle)
    base_handle = ctx.part_world_position(handle)
    ctx.check(
        "hose swivel swings the handle sideways",
        base_handle is not None
        and swung_handle is not None
        and abs(swung_handle[1] - base_handle[1]) > 0.10,
        details=f"base={base_handle}, swung={swung_handle}",
    )

    with ctx.pose({power_press: -0.006}):
        ctx.expect_gap(power_button, body, axis="z", min_gap=-0.008, max_gap=0.006, positive_elem="button_cap", negative_elem="power_button_well", name="power button depresses into well")

    return ctx.report()


object_model = build_object_model()
