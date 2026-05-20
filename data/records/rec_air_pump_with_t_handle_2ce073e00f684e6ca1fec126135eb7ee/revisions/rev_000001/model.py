from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    """Closed thin-walled tube, centered on local Z, with annular end faces."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
            [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_station_pump")

    red = model.material("red_enamel", rgba=(0.78, 0.06, 0.035, 1.0))
    dark_red = model.material("dark_red", rgba=(0.45, 0.025, 0.02, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    brass = model.material("brass", rgba=(0.92, 0.66, 0.24, 1.0))
    white = model.material("gauge_face", rgba=(0.94, 0.91, 0.82, 1.0))
    glass = model.material("smoky_glass", rgba=(0.65, 0.85, 0.95, 0.45))

    body = model.part("body")

    # Floor pads and cast base for a service-bay sized hand pump.
    body.visual(
        Box((0.22, 0.13, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_red,
        name="base_bridge",
    )
    body.visual(
        Box((0.26, 0.10, 0.032)),
        origin=Origin(xyz=(-0.20, 0.0, 0.016)),
        material=black,
        name="foot_pad_0",
    )
    body.visual(
        Box((0.26, 0.10, 0.032)),
        origin=Origin(xyz=(0.20, 0.0, 0.016)),
        material=black,
        name="foot_pad_1",
    )
    body.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_red,
        name="base_socket",
    )

    # The vertical barrel is a real hollow tube, not a solid cylinder.  The
    # piston rod fits through its bore and stays captured at full stroke.
    body.visual(
        _annular_tube_mesh("barrel_tube", outer_radius=0.055, inner_radius=0.038, length=0.820),
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        material=red,
        name="barrel",
    )
    body.visual(
        _annular_tube_mesh("top_collar_tube", outer_radius=0.075, inner_radius=0.026, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        material=dark_red,
        name="top_collar",
    )
    body.visual(
        _annular_tube_mesh("top_guide_bushing", outer_radius=0.035, inner_radius=0.0118, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=brass,
        name="top_bushing",
    )
    body.visual(
        _annular_tube_mesh("bottom_collar_tube", outer_radius=0.078, inner_radius=0.040, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark_red,
        name="bottom_collar",
    )

    # Side-mounted pressure gauge, bracketed to the barrel above the foot pads.
    body.visual(
        Box((0.075, 0.070, 0.090)),
        origin=Origin(xyz=(0.0, -0.066, 0.520)),
        material=dark_red,
        name="gauge_neck",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.058),
        origin=Origin(xyz=(0.0, -0.118, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=red,
        name="gauge_housing",
    )
    body.visual(
        Cylinder(radius=0.073, length=0.005),
        origin=Origin(xyz=(0.0, -0.149, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=white,
        name="gauge_dial",
    )
    body.visual(
        _annular_tube_mesh("gauge_bezel_ring", outer_radius=0.089, inner_radius=0.075, length=0.015),
        origin=Origin(xyz=(0.0, -0.150, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.072, length=0.003),
        origin=Origin(xyz=(0.0, -0.151, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="gauge_lens",
    )
    for idx, (x, z, sx, sz) in enumerate(
        [
            (-0.040, 0.565, 0.006, 0.022),
            (0.000, 0.580, 0.006, 0.024),
            (0.040, 0.565, 0.006, 0.022),
            (-0.058, 0.520, 0.020, 0.006),
            (0.058, 0.520, 0.020, 0.006),
        ]
    ):
        body.visual(
            Box((sx, 0.006, sz)),
            origin=Origin(xyz=(x, -0.150, z)),
            material=black,
            name=f"gauge_tick_{idx}",
        )
    body.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, -0.158, 0.520), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="gauge_pivot",
    )

    # Parked service hose: the outlet, hose, nozzle, and spring clip are all
    # mounted to the lower body so the hose reads as clipped when stowed.
    body.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(xyz=(0.044, 0.078, 0.245), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hose_outlet",
    )
    hose_geom = tube_from_spline_points(
        [
            (0.044, 0.108, 0.245),
            (0.185, 0.075, 0.180),
            (0.220, -0.020, 0.075),
            (0.030, -0.115, 0.060),
            (-0.175, -0.040, 0.135),
            (-0.120, 0.070, 0.265),
            (-0.040, 0.104, 0.324),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(hose_geom, "stowed_hose_mesh"),
        material=black,
        name="hose",
    )
    body.visual(
        Box((0.090, 0.052, 0.078)),
        origin=Origin(xyz=(-0.025, 0.071, 0.323)),
        material=dark_red,
        name="hose_clip",
    )
    body.visual(
        Box((0.080, 0.020, 0.024)),
        origin=Origin(xyz=(-0.025, 0.105, 0.342)),
        material=steel,
        name="clip_lip",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.020, 0.116, 0.334), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="hose_nozzle",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.012, length=0.990),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=steel,
        name="piston_rod",
    )
    handle.visual(
        Cylinder(radius=0.020, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, 0.620), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="top_bar",
    )
    handle.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=steel,
        name="bar_boss",
    )

    grip = model.part("grip")
    grip.visual(
        _annular_tube_mesh("folding_grip_sleeve", outer_radius=0.032, inner_radius=0.0196, length=0.160),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="sleeve",
    )
    grip.visual(
        Box((0.014, 0.050, 0.014)),
        origin=Origin(xyz=(-0.078, 0.040, -0.026)),
        material=black,
        name="lug_0",
    )
    grip.visual(
        Box((0.014, 0.050, 0.014)),
        origin=Origin(xyz=(0.078, 0.040, -0.026)),
        material=black,
        name="lug_1",
    )
    grip.visual(
        Box((0.014, 0.016, 0.070)),
        origin=Origin(xyz=(-0.078, 0.060, -0.062)),
        material=black,
        name="side_arm_0",
    )
    grip.visual(
        Box((0.014, 0.016, 0.070)),
        origin=Origin(xyz=(0.078, 0.060, -0.062)),
        material=black,
        name="side_arm_1",
    )
    grip.visual(
        Cylinder(radius=0.018, length=0.170),
        origin=Origin(xyz=(0.0, 0.060, -0.098), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="folding_bar",
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.074, 0.004, 0.006)),
        origin=Origin(xyz=(0.030, -0.004, 0.0)),
        material=black,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.250),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.130, 0.0, 0.620)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )
    model.articulation(
        "body_to_needle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=needle,
        origin=Origin(xyz=(0.0, -0.161, 0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")
    handle_slide = object_model.get_articulation("body_to_handle")
    grip_spin = object_model.get_articulation("handle_to_grip")
    needle_spin = object_model.get_articulation("body_to_needle")

    ctx.check(
        "primary mechanisms are articulated",
        handle_slide.articulation_type == ArticulationType.PRISMATIC
        and grip_spin.articulation_type == ArticulationType.CONTINUOUS
        and needle_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected prismatic handle, continuous grip, and continuous gauge needle.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="top_bushing",
        elem_b="piston_rod",
        reason="The piston rod is intentionally shown as a close sliding fit through the brass guide bushing.",
    )
    ctx.allow_overlap(
        body,
        needle,
        elem_a="gauge_pivot",
        elem_b="hub",
        reason="The gauge needle hub is captured on a short pivot post at the dial center.",
    )
    ctx.allow_overlap(
        handle,
        grip,
        elem_a="top_bar",
        elem_b="sleeve",
        reason="The folding grip sleeve is modeled as a close bearing fit around the crossbar.",
    )

    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel",
        margin=0.002,
        name="piston rod is centered inside tubular barrel",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="top_bushing",
        margin=0.002,
        name="piston rod stays in guide bushing",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="top_bushing",
        min_overlap=0.040,
        name="guide bushing supports the sliding rod",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel",
        min_overlap=0.30,
        name="collapsed piston rod remains inserted",
    )
    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.250}):
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel",
            min_overlap=0.10,
            name="extended piston rod remains inserted",
        )
        extended_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle translates upward along pump axis",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.20,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    ctx.expect_overlap(
        grip,
        handle,
        axes="x",
        elem_a="sleeve",
        elem_b="top_bar",
        min_overlap=0.15,
        name="grip sleeve spans the crossbar",
    )
    ctx.expect_within(
        handle,
        grip,
        axes="yz",
        inner_elem="top_bar",
        outer_elem="sleeve",
        margin=0.002,
        name="crossbar is captured inside grip sleeve envelope",
    )
    ctx.expect_overlap(
        needle,
        body,
        axes="y",
        elem_a="hub",
        elem_b="gauge_pivot",
        min_overlap=0.001,
        name="needle hub is retained on gauge pivot",
    )

    needle_rest = ctx.part_world_aabb(needle)
    with ctx.pose({needle_spin: pi / 2.0, grip_spin: pi / 2.0}):
        needle_rotated = ctx.part_world_aabb(needle)
        grip_rotated = ctx.part_world_position(grip)
    ctx.check(
        "gauge needle rotates about its face pivot",
        needle_rest is not None and needle_rotated is not None and needle_rest != needle_rotated,
        details=f"rest={needle_rest}, rotated={needle_rotated}",
    )
    ctx.check(
        "folding grip rotates without leaving the crossbar axis",
        grip_rotated is not None,
        details="Grip pose could not be sampled.",
    )

    hose_aabb = ctx.part_element_world_aabb(body, elem="hose")
    clip_aabb = ctx.part_element_world_aabb(body, elem="hose_clip")
    ctx.check(
        "stowed hose passes through lower body clip",
        hose_aabb is not None
        and clip_aabb is not None
        and hose_aabb[0][1] < clip_aabb[1][1]
        and hose_aabb[1][1] > clip_aabb[0][1]
        and hose_aabb[0][2] < clip_aabb[1][2]
        and hose_aabb[1][2] > clip_aabb[0][2],
        details=f"hose={hose_aabb}, clip={clip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
