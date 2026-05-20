from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _cylinder_y(part, *, radius, length, xyz, material, name):
    """Cylinder whose axis runs along world Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, *, radius, length, xyz, material, name):
    """Cylinder whose axis runs along world X."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _tube_between_xz(part, p1, p2, *, radius, material, name):
    """Slender round tube between two points in an X-Z plane."""
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    dx = x2 - x1
    dz = z2 - z1
    length = math.sqrt(dx * dx + dz * dz)
    theta_y = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x1 + x2) * 0.5, (y1 + y2) * 0.5, (z1 + z2) * 0.5),
            rpy=(0.0, theta_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_pedal_geometry(part, *, side_sign: float, material_pad, material_metal):
    """Foot pedal body with a visible bearing spindle."""
    _cylinder_y(
        part,
        radius=0.007,
        length=0.066,
        xyz=(0.0, side_sign * 0.033, 0.0),
        material=material_metal,
        name="pedal_spindle",
    )
    part.visual(
        Box((0.145, 0.062, 0.030)),
        origin=Origin(xyz=(0.0, side_sign * 0.094, 0.0)),
        material=material_pad,
        name="pedal_pad",
    )
    part.visual(
        Box((0.118, 0.066, 0.006)),
        origin=Origin(xyz=(0.0, side_sign * 0.094, 0.018)),
        material=material_metal,
        name="pedal_top_plate",
    )
    for x in (-0.052, 0.052):
        part.visual(
            Box((0.010, 0.068, 0.010)),
            origin=Origin(xyz=(x, side_sign * 0.094, -0.020)),
            material=material_metal,
            name=f"pedal_end_cleat_{0 if x < 0 else 1}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_exercise_bike")

    matte_graphite = model.material("matte_graphite", rgba=(0.055, 0.058, 0.062, 1.0))
    satin_black = model.material("satin_black", rgba=(0.010, 0.011, 0.012, 1.0))
    satin_charcoal = model.material("satin_charcoal", rgba=(0.16, 0.17, 0.17, 1.0))
    warm_black = model.material("warm_black", rgba=(0.025, 0.023, 0.021, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.66, 0.62, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.48, 0.50, 0.50, 1.0))
    muted_blue = model.material("muted_blue_display", rgba=(0.035, 0.085, 0.115, 1.0))

    frame = model.part("frame")

    # Stable base and central monocoque/flywheel cover.
    _cylinder_y(
        frame,
        radius=0.036,
        length=0.720,
        xyz=(0.555, 0.0, 0.058),
        material=satin_black,
        name="front_stabilizer",
    )
    _cylinder_y(
        frame,
        radius=0.036,
        length=0.660,
        xyz=(-0.555, 0.0, 0.058),
        material=satin_black,
        name="rear_stabilizer",
    )
    for i, x in enumerate((0.555, -0.555)):
        for j, y in enumerate((-0.355, 0.355)):
            frame.visual(
                Box((0.185, 0.118, 0.036)),
                origin=Origin(xyz=(x, y, 0.025)),
                material=dark_rubber,
                name=f"leveling_foot_{i}_{j}",
            )

    frame.visual(
        Box((1.145, 0.170, 0.072)),
        origin=Origin(xyz=(0.000, 0.0, 0.086)),
        material=satin_black,
        name="base_spine",
    )
    frame.visual(
        Box((0.450, 0.215, 0.090)),
        origin=Origin(xyz=(-0.070, 0.0, 0.145)),
        material=matte_graphite,
        name="lower_transition_fairing",
    )

    _cylinder_y(
        frame,
        radius=0.305,
        length=0.182,
        xyz=(0.070, 0.0, 0.365),
        material=matte_graphite,
        name="flywheel_housing",
    )
    _cylinder_y(
        frame,
        radius=0.288,
        length=0.010,
        xyz=(0.070, -0.101, 0.365),
        material=satin_charcoal,
        name="left_cover_panel",
    )
    _cylinder_y(
        frame,
        radius=0.288,
        length=0.010,
        xyz=(0.070, 0.101, 0.365),
        material=satin_charcoal,
        name="right_cover_panel",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.291, 0.0045, radial_segments=20, tubular_segments=72), "left_cover_seam"),
        origin=Origin(xyz=(0.070, -0.108, 0.365), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="left_cover_seam",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.291, 0.0045, radial_segments=20, tubular_segments=72), "right_cover_seam"),
        origin=Origin(xyz=(0.070, 0.108, 0.365), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="right_cover_seam",
    )
    _cylinder_y(
        frame,
        radius=0.052,
        length=0.035,
        xyz=(0.070, -0.123, 0.365),
        material=brushed_steel,
        name="crank_axle_boss",
    )
    _cylinder_y(
        frame,
        radius=0.041,
        length=0.032,
        xyz=(0.070, 0.122, 0.365),
        material=brushed_steel,
        name="flywheel_axle_boss",
    )
    for i, (dx, dz) in enumerate(((0.000, 0.238), (0.202, 0.125), (0.202, -0.125), (0.000, -0.238), (-0.202, -0.125), (-0.202, 0.125))):
        _cylinder_y(
            frame,
            radius=0.010,
            length=0.014,
            xyz=(0.070 + dx, -0.113, 0.365 + dz),
            material=brushed_steel,
            name=f"cover_fastener_{i}",
        )

    # Seat mast sleeve: four walls around a clear square opening.
    sleeve_x = -0.325
    for i, sx in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.013, 0.078, 0.335)),
            origin=Origin(xyz=(sleeve_x + sx * 0.033, 0.0, 0.625)),
            material=satin_black,
            name=f"seat_sleeve_side_{i}",
        )
    for i, sy in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.078, 0.013, 0.335)),
            origin=Origin(xyz=(sleeve_x, sy * 0.033, 0.625)),
            material=satin_black,
            name=f"seat_sleeve_face_{i}",
        )
    for zc, h, label in ((0.465, 0.035, "lower"), (0.796, 0.022, "top")):
        for i, sx in enumerate((-1.0, 1.0)):
            frame.visual(
                Box((0.010, 0.096, h)),
                origin=Origin(xyz=(sleeve_x + sx * 0.043, 0.0, zc)),
                material=satin_charcoal,
                name=f"seat_{label}_collar_side_{i}",
            )
        for i, sy in enumerate((-1.0, 1.0)):
            frame.visual(
                Box((0.096, 0.010, h)),
                origin=Origin(xyz=(sleeve_x, sy * 0.043, zc)),
                material=satin_charcoal,
                name=f"seat_{label}_collar_face_{i}",
            )
    _tube_between_xz(
        frame,
        (-0.560, 0.0, 0.105),
        (sleeve_x, 0.0, 0.492),
        radius=0.026,
        material=satin_black,
        name="rear_riser_tube",
    )
    _tube_between_xz(
        frame,
        (0.015, -0.055, 0.555),
        (sleeve_x, -0.055, 0.735),
        radius=0.023,
        material=satin_black,
        name="top_frame_tube_0",
    )
    _tube_between_xz(
        frame,
        (0.015, 0.055, 0.555),
        (sleeve_x, 0.055, 0.735),
        radius=0.023,
        material=satin_black,
        name="top_frame_tube_1",
    )

    # Handlebar mast, console support, and refined interface hardware.
    _tube_between_xz(
        frame,
        (0.255, 0.0, 0.520),
        (0.455, 0.0, 1.050),
        radius=0.027,
        material=satin_black,
        name="handlebar_mast",
    )
    frame.visual(
        Box((0.080, 0.088, 0.050)),
        origin=Origin(xyz=(0.325, 0.0, 0.710)),
        material=satin_charcoal,
        name="mast_adjust_collar",
    )
    _cylinder_y(
        frame,
        radius=0.021,
        length=0.052,
        xyz=(0.325, -0.070, 0.710),
        material=brushed_steel,
        name="mast_knob_boss",
    )
    _cylinder_y(
        frame,
        radius=0.020,
        length=0.610,
        xyz=(0.470, 0.0, 1.075),
        material=satin_black,
        name="handlebar_crossbar",
    )
    _cylinder_x(
        frame,
        radius=0.019,
        length=0.255,
        xyz=(0.555, -0.296, 1.084),
        material=satin_black,
        name="handle_grip_0",
    )
    _cylinder_x(
        frame,
        radius=0.019,
        length=0.255,
        xyz=(0.555, 0.296, 1.084),
        material=satin_black,
        name="handle_grip_1",
    )
    frame.visual(
        Box((0.050, 0.140, 0.050)),
        origin=Origin(xyz=(0.515, 0.0, 1.030)),
        material=satin_black,
        name="console_neck",
    )
    frame.visual(
        Box((0.082, 0.158, 0.050)),
        origin=Origin(xyz=(0.478, 0.0, 1.058)),
        material=satin_black,
        name="console_mount_plate",
    )
    frame.visual(
        Box((0.160, 0.235, 0.092)),
        origin=Origin(xyz=(0.610, 0.0, 1.028)),
        material=matte_graphite,
        name="console_body",
    )
    frame.visual(
        Box((0.012, 0.176, 0.050)),
        origin=Origin(xyz=(0.695, 0.0, 1.040)),
        material=muted_blue,
        name="display_lens",
    )
    _cylinder_x(
        frame,
        radius=0.025,
        length=0.024,
        xyz=(0.698, 0.0, 0.982),
        material=brushed_steel,
        name="resistance_dial_boss",
    )

    # Seat sleeve locking boss is fixed to the sleeve; the hand knob below rotates.
    _cylinder_y(
        frame,
        radius=0.020,
        length=0.058,
        xyz=(sleeve_x, -0.067, 0.665),
        material=brushed_steel,
        name="seat_knob_boss",
    )

    # Height-adjustable seatpost and saddle.
    seatpost = model.part("seatpost")
    seatpost.visual(
        Cylinder(radius=0.020, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_aluminum,
        name="inner_post",
    )
    seatpost.visual(
        Box((0.053, 0.053, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=satin_charcoal,
        name="lower_guide_bushing",
    )
    seatpost.visual(
        Box((0.070, 0.068, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=satin_aluminum,
        name="saddle_clamp_block",
    )
    _cylinder_x(
        seatpost,
        radius=0.0085,
        length=0.255,
        xyz=(0.005, -0.032, 0.310),
        material=brushed_steel,
        name="saddle_rail_0",
    )
    _cylinder_x(
        seatpost,
        radius=0.0085,
        length=0.255,
        xyz=(0.005, 0.032, 0.310),
        material=brushed_steel,
        name="saddle_rail_1",
    )
    seatpost.visual(
        Box((0.225, 0.112, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, 0.324)),
        material=satin_black,
        name="saddle_base_plate",
    )
    saddle_profile = [
        (0.170, 0.000),
        (0.143, 0.034),
        (0.056, 0.066),
        (-0.120, 0.092),
        (-0.168, 0.066),
        (-0.182, 0.000),
        (-0.168, -0.066),
        (-0.120, -0.092),
        (0.056, -0.066),
        (0.143, -0.034),
    ]
    seatpost.visual(
        mesh_from_geometry(ExtrudeGeometry(saddle_profile, 0.046, center=True), "saddle_cushion"),
        origin=Origin(xyz=(-0.010, 0.0, 0.355)),
        material=warm_black,
        name="saddle_cushion",
    )

    # Visible flywheel with speed holes/spokes, mounted just outside the right cover.
    flywheel = model.part("flywheel")
    flywheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.238,
                0.036,
                rim=WheelRim(inner_radius=0.156, flange_height=0.006, flange_thickness=0.004, bead_seat_depth=0.002),
                hub=WheelHub(
                    radius=0.045,
                    width=0.034,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.055, hole_diameter=0.006),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="split_y", count=6, thickness=0.006, window_radius=0.018),
                bore=WheelBore(style="round", diameter=0.018),
            ),
            "external_flywheel",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=brushed_steel,
        name="flywheel_mass",
    )

    # Crank spider, hub, and arms rotate on the obvious left-side bottom bracket.
    crank = model.part("crank")
    _cylinder_y(
        crank,
        radius=0.010,
        length=0.411,
        xyz=(0.0, 0.1785, 0.0),
        material=brushed_steel,
        name="through_axle",
    )
    _cylinder_y(
        crank,
        radius=0.044,
        length=0.054,
        xyz=(0.0, 0.0, 0.0),
        material=brushed_steel,
        name="crank_hub",
    )
    _cylinder_y(
        crank,
        radius=0.073,
        length=0.014,
        xyz=(0.0, -0.021, 0.0),
        material=satin_aluminum,
        name="crank_spider_disc",
    )
    crank.visual(
        Box((0.205, 0.022, 0.028)),
        origin=Origin(xyz=(0.102, -0.018, 0.0)),
        material=satin_aluminum,
        name="front_arm",
    )
    crank.visual(
        Box((0.205, 0.022, 0.028)),
        origin=Origin(xyz=(-0.102, 0.395, 0.0)),
        material=satin_aluminum,
        name="rear_arm",
    )
    _cylinder_y(
        crank,
        radius=0.020,
        length=0.028,
        xyz=(0.205, -0.030, 0.0),
        material=brushed_steel,
        name="front_pedal_anchor",
    )
    _cylinder_y(
        crank,
        radius=0.020,
        length=0.028,
        xyz=(-0.205, 0.395, 0.0),
        material=brushed_steel,
        name="rear_pedal_anchor",
    )

    pedal_0 = model.part("pedal_0")
    _add_pedal_geometry(pedal_0, side_sign=-1.0, material_pad=dark_rubber, material_metal=brushed_steel)
    pedal_1 = model.part("pedal_1")
    _add_pedal_geometry(pedal_1, side_sign=1.0, material_pad=dark_rubber, material_metal=brushed_steel)

    # User-facing rotary adjustment hardware.
    seat_knob = model.part("seat_knob")
    seat_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.034,
                body_style="lobed",
                top_diameter=0.052,
                crown_radius=0.0015,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0012, width=0.002),
                center=False,
            ),
            "seat_knob_cap",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="seat_knob_cap",
    )
    mast_knob = model.part("mast_knob")
    mast_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.030,
                body_style="lobed",
                top_diameter=0.046,
                crown_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=10, depth=0.001),
                center=False,
            ),
            "mast_knob_cap",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="mast_knob_cap",
    )
    resistance_dial = model.part("resistance_dial")
    resistance_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.030,
                body_style="faceted",
                top_diameter=0.050,
                base_diameter=0.062,
                edge_radius=0.001,
                grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "resistance_dial_cap",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_charcoal,
        name="resistance_dial_cap",
    )

    model.articulation(
        "frame_to_seatpost",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seatpost,
        origin=Origin(xyz=(sleeve_x, 0.0, 0.795)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.160, effort=180.0, velocity=0.18),
        motion_properties=MotionProperties(damping=0.8, friction=0.6),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.070, -0.1675, 0.365)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )
    model.articulation(
        "frame_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.070, 0.158, 0.365)),
        axis=(0.0, 1.0, 0.0),
        mimic=Mimic(joint="frame_to_crank", multiplier=2.8, offset=0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=25.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    model.articulation(
        "crank_to_pedal_0",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_0,
        origin=Origin(xyz=(0.205, -0.044, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.03, friction=0.01),
    )
    model.articulation(
        "crank_to_pedal_1",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_1,
        origin=Origin(xyz=(-0.205, 0.409, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=12.0),
        motion_properties=MotionProperties(damping=0.03, friction=0.01),
    )
    model.articulation(
        "frame_to_seat_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=seat_knob,
        origin=Origin(xyz=(sleeve_x, -0.093, 0.665)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.10),
    )
    model.articulation(
        "frame_to_mast_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=mast_knob,
        origin=Origin(xyz=(0.325, -0.096, 0.710)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.10),
    )
    model.articulation(
        "frame_to_resistance_dial",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=resistance_dial,
        origin=Origin(xyz=(0.707, 0.0, 0.982)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=2.0, velocity=3.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seatpost = object_model.get_part("seatpost")
    crank = object_model.get_part("crank")
    flywheel = object_model.get_part("flywheel")
    seat_joint = object_model.get_articulation("frame_to_seatpost")
    crank_joint = object_model.get_articulation("frame_to_crank")
    dial_joint = object_model.get_articulation("frame_to_resistance_dial")

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="flywheel_housing",
        elem_b="through_axle",
        reason="The crank axle is intentionally represented as a hidden shaft passing through the flywheel housing bearing tunnel.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_axle_boss",
        elem_b="through_axle",
        reason="The crank axle is intentionally captured inside the visible bottom-bracket bearing boss.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="flywheel_axle_boss",
        elem_b="through_axle",
        reason="The shared axle is intentionally captured by the flywheel-side bearing boss.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="left_cover_panel",
        elem_b="through_axle",
        reason="The hidden crank shaft passes through the molded cover's central bearing opening, represented by a solid cover proxy.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="right_cover_panel",
        elem_b="through_axle",
        reason="The hidden crank shaft also crosses the opposite cover's central bearing opening, represented by a solid cover proxy.",
    )
    ctx.allow_overlap(
        crank,
        flywheel,
        elem_a="through_axle",
        elem_b="flywheel_mass",
        reason="The hidden crank shaft passes through the flywheel hub bore in this coaxial drive representation.",
    )

    ctx.expect_gap(
        flywheel,
        frame,
        axis="y",
        min_gap=0.012,
        elem_a="flywheel_mass",
        elem_b="right_cover_panel",
        name="visible flywheel clears right cover",
    )
    ctx.expect_gap(
        frame,
        crank,
        axis="y",
        max_penetration=0.0,
        positive_elem="crank_axle_boss",
        negative_elem="crank_hub",
        name="crank hub is seated outside bottom bracket boss",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="through_axle",
        outer_elem="flywheel_housing",
        margin=0.004,
        name="hidden crank axle stays inside flywheel housing bore",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="through_axle",
        elem_b="flywheel_housing",
        min_overlap=0.16,
        name="crank axle is retained through housing width",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="through_axle",
        outer_elem="crank_axle_boss",
        margin=0.001,
        name="crank axle is centered in bearing boss",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="through_axle",
        elem_b="crank_axle_boss",
        min_overlap=0.02,
        name="crank axle is captured by bearing boss",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="through_axle",
        outer_elem="flywheel_axle_boss",
        margin=0.001,
        name="crank axle is centered in flywheel boss",
    )
    ctx.expect_overlap(
        crank,
        flywheel,
        axes="y",
        elem_a="through_axle",
        elem_b="flywheel_mass",
        min_overlap=0.02,
        name="coaxial flywheel hub surrounds shaft",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="through_axle",
        outer_elem="left_cover_panel",
        margin=0.001,
        name="shaft crosses cover at central bearing opening",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="through_axle",
        outer_elem="right_cover_panel",
        margin=0.001,
        name="shaft crosses opposite cover bearing opening",
    )

    # Seatpost is a real retained slider: centered in the sleeve, still inserted at full height.
    with ctx.pose({seat_joint: 0.0}):
        ctx.expect_overlap(
            seatpost,
            frame,
            axes="z",
            min_overlap=0.16,
            elem_a="inner_post",
            elem_b="seat_sleeve_side_0",
            name="lowered seatpost remains captured in sleeve",
        )
    with ctx.pose({seat_joint: 0.160}):
        ctx.expect_overlap(
            seatpost,
            frame,
            axes="z",
            min_overlap=0.08,
            elem_a="inner_post",
            elem_b="seat_sleeve_side_0",
            name="raised seatpost retains insertion",
        )

    rest_aabb = ctx.part_element_world_aabb(crank, elem="front_arm")
    with ctx.pose({crank_joint: math.pi / 2.0, dial_joint: 0.75}):
        quarter_aabb = ctx.part_element_world_aabb(crank, elem="front_arm")
    ctx.check(
        "crank arm visibly changes orientation",
        rest_aabb is not None
        and quarter_aabb is not None
        and abs((rest_aabb[1][0] - rest_aabb[0][0]) - (quarter_aabb[1][0] - quarter_aabb[0][0])) > 0.09,
        details=f"rest={rest_aabb}, quarter={quarter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
