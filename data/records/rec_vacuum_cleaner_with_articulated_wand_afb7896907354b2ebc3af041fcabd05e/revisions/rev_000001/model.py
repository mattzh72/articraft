from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder descriptor is Z-aligned; rotate it to run left-right across Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _tube_angle(dx: float, dz: float) -> float:
    """Return the Y rotation that aligns a Z cylinder with a vector in XZ."""
    return math.atan2(dx, dz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_vacuum")

    molded_blue = model.material("molded_blue", rgba=(0.05, 0.14, 0.22, 1.0))
    black_poly = model.material("black_poly", rgba=(0.025, 0.026, 0.024, 1.0))
    rubber = model.material("ribbed_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    steel = model.material("zinc_plated_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    orange = model.material("orange_latches", rgba=(0.95, 0.36, 0.04, 1.0))
    grey = model.material("molded_grey", rgba=(0.33, 0.36, 0.37, 1.0))

    main_body = model.part("main_body")
    body_shell = superellipse_side_loft(
        [
            (-0.36, 0.26, 0.245, 0.27),
            (-0.24, 0.37, 0.285, 0.39),
            (-0.02, 0.43, 0.315, 0.43),
            (0.22, 0.40, 0.305, 0.39),
            (0.39, 0.28, 0.265, 0.30),
        ],
        exponents=2.8,
        segments=64,
    )
    body_shell.rotate_z(-math.pi / 2.0)
    main_body.visual(_mesh(body_shell, "main_body_shell"), material=molded_blue, name="body_shell")
    main_body.visual(Box((0.72, 0.44, 0.11)), origin=Origin(xyz=(0.00, 0.0, 0.075)), material=black_poly, name="skid_base")
    main_body.visual(Box((0.08, 0.46, 0.19)), origin=Origin(xyz=(0.405, 0.0, 0.185)), material=black_poly, name="front_bumper")
    main_body.visual(Box((0.06, 0.41, 0.16)), origin=Origin(xyz=(-0.385, 0.0, 0.18)), material=black_poly, name="rear_bumper")

    # Large rear service wheels and a through-axle are static here: they are rugged
    # carry hardware, while the visible wand pivots are the functional articulation.
    main_body.visual(Cylinder(radius=0.092, length=0.050), origin=_y_cylinder_origin(-0.245, 0.245, 0.115), material=rubber, name="wheel_0_tire")
    main_body.visual(Cylinder(radius=0.092, length=0.050), origin=_y_cylinder_origin(-0.245, -0.245, 0.115), material=rubber, name="wheel_1_tire")
    main_body.visual(Cylinder(radius=0.045, length=0.060), origin=_y_cylinder_origin(-0.245, 0.248, 0.115), material=steel, name="wheel_0_hub")
    main_body.visual(Cylinder(radius=0.045, length=0.060), origin=_y_cylinder_origin(-0.245, -0.248, 0.115), material=steel, name="wheel_1_hub")
    main_body.visual(Cylinder(radius=0.018, length=0.54), origin=_y_cylinder_origin(-0.245, 0.0, 0.115), material=dark_steel, name="rear_axle")
    main_body.visual(Box((0.09, 0.54, 0.055)), origin=Origin(xyz=(-0.245, 0.0, 0.115)), material=black_poly, name="axle_saddle")

    # Side filter grilles with a proud frame and fasteners.
    grille = SlotPatternPanelGeometry(
        (0.24, 0.145),
        0.006,
        slot_size=(0.035, 0.008),
        pitch=(0.052, 0.024),
        frame=0.014,
        corner_radius=0.006,
        slot_angle_deg=10.0,
        stagger=True,
    )
    main_body.visual(
        _mesh(grille, "right_filter_grille"),
        origin=Origin(xyz=(0.02, 0.222, 0.315), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_poly,
        name="right_filter_grille",
    )
    main_body.visual(
        _mesh(grille, "left_filter_grille"),
        origin=Origin(xyz=(0.02, -0.222, 0.315), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_poly,
        name="left_filter_grille",
    )
    for y in (-0.228, 0.228):
        for x in (-0.10, 0.14):
            for z in (0.255, 0.375):
                main_body.visual(Cylinder(radius=0.009, length=0.007), origin=_y_cylinder_origin(x, y, z), material=steel, name=f"grille_screw_{x}_{y}_{z}")

    # Service latches, front port, and a raised carry handle with molded anchors.
    main_body.visual(Cylinder(radius=0.072, length=0.035), origin=_x_cylinder_origin(0.417, 0.0, 0.282), material=black_poly, name="hose_port")
    main_body.visual(Cylinder(radius=0.048, length=0.038), origin=_x_cylinder_origin(0.425, 0.0, 0.282), material=grey, name="port_collar")
    main_body.visual(Box((0.030, 0.105, 0.080)), origin=Origin(xyz=(0.12, 0.230, 0.345)), material=orange, name="latch_0")
    main_body.visual(Box((0.030, 0.105, 0.080)), origin=Origin(xyz=(0.12, -0.230, 0.345)), material=orange, name="latch_1")
    main_body.visual(Box((0.030, 0.055, 0.035)), origin=Origin(xyz=(0.12, 0.256, 0.395)), material=steel, name="latch_0_pin")
    main_body.visual(Box((0.030, 0.055, 0.035)), origin=Origin(xyz=(0.12, -0.256, 0.395)), material=steel, name="latch_1_pin")
    handle = tube_from_spline_points(
        [(-0.245, 0.0, 0.455), (-0.170, 0.0, 0.605), (0.130, 0.0, 0.635), (0.270, 0.0, 0.505)],
        radius=0.024,
        samples_per_segment=18,
        radial_segments=20,
    )
    main_body.visual(_mesh(handle, "carry_handle"), material=black_poly, name="carry_handle")
    main_body.visual(Box((0.105, 0.112, 0.160)), origin=Origin(xyz=(-0.255, 0.0, 0.405)), material=black_poly, name="rear_handle_foot")
    main_body.visual(Box((0.105, 0.112, 0.130)), origin=Origin(xyz=(0.278, 0.0, 0.420)), material=black_poly, name="front_handle_foot")

    # Front shoulder yoke: cheek plates, bridge, capture pin, and visible bolt heads.
    main_body.visual(Box((0.118, 0.190, 0.050)), origin=Origin(xyz=(0.405, 0.0, 0.425)), material=black_poly, name="front_pivot_bridge")
    main_body.visual(Box((0.082, 0.025, 0.145)), origin=Origin(xyz=(0.430, 0.078, 0.505)), material=black_poly, name="front_yoke_cheek_0")
    main_body.visual(Box((0.082, 0.025, 0.145)), origin=Origin(xyz=(0.430, -0.078, 0.505)), material=black_poly, name="front_yoke_cheek_1")
    main_body.visual(Cylinder(radius=0.014, length=0.198), origin=_y_cylinder_origin(0.430, 0.0, 0.505), material=steel, name="front_pivot_pin")
    for y in (-0.093, 0.093):
        main_body.visual(Cylinder(radius=0.018, length=0.009), origin=_y_cylinder_origin(0.430, y, 0.505), material=steel, name=f"front_pivot_washer_{y}")
        main_body.visual(Cylinder(radius=0.008, length=0.007), origin=_y_cylinder_origin(0.430, y, 0.555), material=steel, name=f"front_yoke_bolt_{y}")

    # A short ribbed service hose is rigidly stowed between the port and shoulder yoke.
    hose = tube_from_spline_points(
        [(0.405, 0.0, 0.282), (0.442, 0.0, 0.340), (0.425, 0.0, 0.392), (0.385, 0.0, 0.425)],
        radius=0.029,
        samples_per_segment=14,
        radial_segments=18,
    )
    main_body.visual(_mesh(hose, "short_flex_hose"), material=rubber, name="short_flex_hose")

    upper_wand = model.part("upper_wand")
    upper_wand.visual(Cylinder(radius=0.047, length=0.096), origin=_y_cylinder_origin(0.0, 0.0, 0.0), material=dark_steel, name="shoulder_hub")
    upper_wand.visual(Cylinder(radius=0.034, length=0.132), origin=_y_cylinder_origin(0.0, 0.0, 0.0), material=steel, name="shoulder_bushing")
    upper_tube = tube_from_spline_points(
        [(0.018, 0.0, 0.0), (0.22, 0.0, -0.035), (0.40, 0.0, -0.082), (0.535, 0.0, -0.108)],
        radius=0.032,
        samples_per_segment=12,
        radial_segments=20,
    )
    upper_wand.visual(_mesh(upper_tube, "upper_wand_tube"), material=grey, name="upper_wand_tube")
    theta_upper = _tube_angle(0.620, -0.130)
    upper_wand.visual(Cylinder(radius=0.044, length=0.105), origin=Origin(xyz=(0.075, 0.0, -0.014), rpy=(0.0, theta_upper, 0.0)), material=black_poly, name="shoulder_reinforcement")
    upper_wand.visual(Cylinder(radius=0.041, length=0.092), origin=Origin(xyz=(0.390, 0.0, -0.080), rpy=(0.0, theta_upper, 0.0)), material=black_poly, name="grip_sleeve")
    for offset in (0.340, 0.390, 0.440):
        upper_wand.visual(Cylinder(radius=0.044, length=0.011), origin=Origin(xyz=(offset, 0.0, -0.130 * offset / 0.620), rpy=(0.0, theta_upper, 0.0)), material=rubber, name=f"grip_rib_{offset}")
    upper_wand.visual(Box((0.060, 0.180, 0.058)), origin=Origin(xyz=(0.535, 0.0, -0.112)), material=black_poly, name="elbow_fork_bridge")
    upper_wand.visual(Box((0.118, 0.025, 0.112)), origin=Origin(xyz=(0.620, 0.078, -0.130)), material=black_poly, name="elbow_fork_cheek_0")
    upper_wand.visual(Box((0.118, 0.025, 0.112)), origin=Origin(xyz=(0.620, -0.078, -0.130)), material=black_poly, name="elbow_fork_cheek_1")
    upper_wand.visual(Cylinder(radius=0.014, length=0.198), origin=_y_cylinder_origin(0.620, 0.0, -0.130), material=steel, name="elbow_pivot_pin")
    for y in (-0.093, 0.093):
        upper_wand.visual(Cylinder(radius=0.017, length=0.009), origin=_y_cylinder_origin(0.620, y, -0.130), material=steel, name=f"elbow_washer_{y}")

    lower_wand = model.part("lower_wand")
    lower_wand.visual(Cylinder(radius=0.045, length=0.094), origin=_y_cylinder_origin(0.0, 0.0, 0.0), material=dark_steel, name="elbow_hub")
    lower_wand.visual(Cylinder(radius=0.031, length=0.120), origin=_y_cylinder_origin(0.0, 0.0, 0.0), material=steel, name="elbow_bushing")
    lower_tube = tube_from_spline_points(
        [(0.018, 0.0, 0.0), (0.24, 0.0, -0.060), (0.50, 0.0, -0.158), (0.680, 0.0, -0.222)],
        radius=0.029,
        samples_per_segment=12,
        radial_segments=20,
    )
    lower_wand.visual(_mesh(lower_tube, "lower_wand_tube"), material=grey, name="lower_wand_tube")
    theta_lower = _tube_angle(0.760, -0.250)
    lower_wand.visual(Cylinder(radius=0.039, length=0.095), origin=Origin(xyz=(0.100, 0.0, -0.032), rpy=(0.0, theta_lower, 0.0)), material=black_poly, name="lower_collar")
    lower_wand.visual(Cylinder(radius=0.039, length=0.110), origin=Origin(xyz=(0.555, 0.0, -0.182), rpy=(0.0, theta_lower, 0.0)), material=black_poly, name="service_collar")
    lower_wand.visual(Box((0.075, 0.174, 0.055)), origin=Origin(xyz=(0.667, 0.0, -0.220)), material=black_poly, name="nozzle_fork_bridge")
    lower_wand.visual(Box((0.114, 0.025, 0.106)), origin=Origin(xyz=(0.760, 0.078, -0.250)), material=black_poly, name="nozzle_fork_cheek_0")
    lower_wand.visual(Box((0.114, 0.025, 0.106)), origin=Origin(xyz=(0.760, -0.078, -0.250)), material=black_poly, name="nozzle_fork_cheek_1")
    lower_wand.visual(Cylinder(radius=0.0135, length=0.198), origin=_y_cylinder_origin(0.760, 0.0, -0.250), material=steel, name="nozzle_pivot_pin")
    for y in (-0.093, 0.093):
        lower_wand.visual(Cylinder(radius=0.016, length=0.009), origin=_y_cylinder_origin(0.760, y, -0.250), material=steel, name=f"nozzle_washer_{y}")

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(Cylinder(radius=0.044, length=0.092), origin=_y_cylinder_origin(0.0, 0.0, 0.0), material=dark_steel, name="nozzle_hub")
    floor_nozzle.visual(Box((0.105, 0.124, 0.100)), origin=Origin(xyz=(0.030, 0.0, -0.035)), material=black_poly, name="trunnion_tower")
    floor_nozzle.visual(Box((0.430, 0.520, 0.060)), origin=Origin(xyz=(0.165, 0.0, -0.095)), material=molded_blue, name="nozzle_shell")
    floor_nozzle.visual(Box((0.455, 0.545, 0.035)), origin=Origin(xyz=(0.170, 0.0, -0.125)), material=black_poly, name="rubber_skirt")
    floor_nozzle.visual(Box((0.430, 0.450, 0.016)), origin=Origin(xyz=(0.215, 0.0, -0.065)), material=grey, name="top_wear_plate")
    floor_nozzle.visual(Box((0.050, 0.440, 0.036)), origin=Origin(xyz=(0.365, 0.0, -0.122)), material=orange, name="front_service_lip")
    floor_nozzle.visual(Cylinder(radius=0.022, length=0.430), origin=_y_cylinder_origin(0.090, 0.0, -0.132), material=rubber, name="rear_roller")
    floor_nozzle.visual(Cylinder(radius=0.018, length=0.400), origin=_y_cylinder_origin(0.320, 0.0, -0.132), material=rubber, name="front_brush_roller")
    floor_nozzle.visual(Cylinder(radius=0.010, length=0.480), origin=_y_cylinder_origin(0.205, 0.0, -0.132), material=dark_steel, name="roller_axle")
    for x in (0.03, 0.30):
        for y in (-0.215, 0.215):
            floor_nozzle.visual(Cylinder(radius=0.010, length=0.008), origin=_y_cylinder_origin(x, y, -0.052), material=steel, name=f"nozzle_screw_{x}_{y}")

    shoulder = model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.430, 0.0, 0.505)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.45, upper=0.85),
    )
    elbow = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.620, 0.0, -0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.6, lower=-0.85, upper=1.05),
    )
    nozzle = model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.760, 0.0, -0.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.70, upper=0.95),
    )

    # Store useful stable names for tests without forcing lookup by construction locals.
    model.meta["primary_joints"] = (shoulder.name, elbow.name, nozzle.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    main_body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    nozzle = object_model.get_articulation("nozzle_pitch")

    ctx.allow_overlap(
        main_body,
        upper_wand,
        elem_a="front_pivot_pin",
        elem_b="shoulder_hub",
        reason="The plated shoulder pin is intentionally captured through the wand trunnion hub.",
    )
    ctx.expect_within(
        main_body,
        upper_wand,
        axes="xz",
        inner_elem="front_pivot_pin",
        outer_elem="shoulder_hub",
        margin=0.002,
        name="shoulder pin runs through hub bore",
    )
    ctx.expect_overlap(
        main_body,
        upper_wand,
        axes="y",
        elem_a="front_pivot_pin",
        elem_b="shoulder_hub",
        min_overlap=0.080,
        name="shoulder pin spans the hub",
    )
    ctx.allow_overlap(
        main_body,
        upper_wand,
        elem_a="front_pivot_pin",
        elem_b="shoulder_bushing",
        reason="The visible metal bushing is coaxial with the captured shoulder pin.",
    )
    ctx.expect_within(
        main_body,
        upper_wand,
        axes="xz",
        inner_elem="front_pivot_pin",
        outer_elem="shoulder_bushing",
        margin=0.002,
        name="shoulder pin is centered in bushing",
    )

    ctx.allow_overlap(
        upper_wand,
        lower_wand,
        elem_a="elbow_pivot_pin",
        elem_b="elbow_hub",
        reason="The elbow pivot pin is modeled as a captured metal axle inside the lower wand hub.",
    )
    ctx.expect_within(
        upper_wand,
        lower_wand,
        axes="xz",
        inner_elem="elbow_pivot_pin",
        outer_elem="elbow_hub",
        margin=0.002,
        name="elbow pin runs through hub bore",
    )
    ctx.expect_overlap(
        upper_wand,
        lower_wand,
        axes="y",
        elem_a="elbow_pivot_pin",
        elem_b="elbow_hub",
        min_overlap=0.078,
        name="elbow pin spans the hub",
    )
    ctx.allow_overlap(
        upper_wand,
        lower_wand,
        elem_a="elbow_pivot_pin",
        elem_b="elbow_bushing",
        reason="The elbow bushing surrounds the captured pivot pin.",
    )
    ctx.expect_within(
        upper_wand,
        lower_wand,
        axes="xz",
        inner_elem="elbow_pivot_pin",
        outer_elem="elbow_bushing",
        margin=0.002,
        name="elbow pin is centered in bushing",
    )

    ctx.allow_overlap(
        lower_wand,
        floor_nozzle,
        elem_a="nozzle_pivot_pin",
        elem_b="nozzle_hub",
        reason="The nozzle pitch axle is intentionally captured inside the nozzle trunnion hub.",
    )
    ctx.expect_within(
        lower_wand,
        floor_nozzle,
        axes="xz",
        inner_elem="nozzle_pivot_pin",
        outer_elem="nozzle_hub",
        margin=0.002,
        name="nozzle pin runs through hub bore",
    )
    ctx.expect_overlap(
        lower_wand,
        floor_nozzle,
        axes="y",
        elem_a="nozzle_pivot_pin",
        elem_b="nozzle_hub",
        min_overlap=0.078,
        name="nozzle pin spans the hub",
    )
    ctx.allow_overlap(
        lower_wand,
        floor_nozzle,
        elem_a="nozzle_pivot_pin",
        elem_b="trunnion_tower",
        reason="The nozzle tower is the reinforced plastic bearing block around the captured axle.",
    )
    ctx.expect_within(
        lower_wand,
        floor_nozzle,
        axes="xz",
        inner_elem="nozzle_pivot_pin",
        outer_elem="trunnion_tower",
        margin=0.002,
        name="nozzle axle sits in tower bearing block",
    )

    ctx.expect_gap(
        floor_nozzle,
        main_body,
        axis="x",
        min_gap=0.60,
        name="wand carries nozzle forward of body",
    )
    ctx.expect_gap(
        floor_nozzle,
        main_body,
        axis="z",
        max_gap=-0.20,
        max_penetration=1.0,
        name="floor nozzle sits below main body",
    )

    rest_elbow = ctx.part_world_position(lower_wand)
    rest_nozzle = ctx.part_world_position(floor_nozzle)
    with ctx.pose({shoulder: 0.55, elbow: 0.35, nozzle: -0.25}):
        lifted_elbow = ctx.part_world_position(lower_wand)
        lifted_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "articulated wand lifts at pivots",
        rest_elbow is not None
        and lifted_elbow is not None
        and rest_nozzle is not None
        and lifted_nozzle is not None
        and lifted_elbow[2] > rest_elbow[2] + 0.18
        and lifted_nozzle[2] > rest_nozzle[2] + 0.10,
        details=f"rest_elbow={rest_elbow}, lifted_elbow={lifted_elbow}, rest_nozzle={rest_nozzle}, lifted_nozzle={lifted_nozzle}",
    )

    return ctx.report()


object_model = build_object_model()
