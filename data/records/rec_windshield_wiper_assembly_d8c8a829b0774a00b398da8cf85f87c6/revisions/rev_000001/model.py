from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_bolt(part, xyz, material, *, radius=0.010, height=0.010, name=None) -> None:
    part.visual(
        Cylinder(radius=radius, length=height),
        # Seat the bolt head a millimeter into the mounting face so it is
        # mechanically attached rather than an exact-contact visual island.
        origin=Origin(xyz=(xyz[0], xyz[1], xyz[2] + height * 0.5 - 0.001)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wiper_assembly")

    phosphate = model.material("phosphate_black", rgba=(0.04, 0.045, 0.05, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.52, 0.55, 0.56, 1.0))
    zinc = model.material("zinc_plated", rgba=(0.72, 0.70, 0.64, 1.0))
    cast = model.material("cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    service_yellow = model.material("service_yellow", rgba=(0.95, 0.72, 0.10, 1.0))
    grease_blue = model.material("grease_blue", rgba=(0.08, 0.23, 0.55, 1.0))
    rubber = model.material("rubber_black", rgba=(0.005, 0.006, 0.007, 1.0))
    bronze = model.material("oil_bronze", rgba=(0.62, 0.42, 0.18, 1.0))
    label_red = model.material("red_lock_tab", rgba=(0.78, 0.05, 0.04, 1.0))

    rod_eye_mesh = mesh_from_geometry(
        TorusGeometry(0.022, 0.006, radial_segments=20, tubular_segments=36),
        "replaceable_rod_eye_bushing",
    )
    large_seal_mesh = mesh_from_geometry(
        TorusGeometry(0.032, 0.007, radial_segments=24, tubular_segments=40),
        "spindle_grease_seal",
    )

    frame = model.part("service_frame")
    frame.visual(
        Box((1.54, 0.22, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=phosphate,
        name="cowl_rail",
    )
    frame.visual(
        Box((1.42, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.095, 0.080)),
        material=cast,
        name="front_lip",
    )
    frame.visual(
        Box((1.42, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, -0.095, 0.080)),
        material=cast,
        name="rear_lip",
    )
    frame.visual(
        Box((1.54, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=zinc,
        name="replaceable_slide_rail",
    )

    for x in (-0.68, -0.24, 0.24, 0.68):
        frame.visual(
            Box((0.105, 0.29, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.014)),
            material=cast,
            name=f"mount_foot_{x:+.2f}",
        )
        _add_bolt(frame, (x - 0.028, 0.090, 0.028), zinc)
        _add_bolt(frame, (x + 0.028, -0.090, 0.028), zinc)

    # Motor and serviceable gearbox: a chunky, bolted module sitting on the rail.
    frame.visual(
        Cylinder(radius=0.052, length=0.26),
        origin=Origin(xyz=(0.0, -0.190, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast,
        name="motor_can",
    )
    frame.visual(
        Box((0.22, 0.17, 0.09)),
        origin=Origin(xyz=(0.0, -0.100, 0.095)),
        material=cast,
        name="gearbox_body",
    )
    frame.visual(
        Cylinder(radius=0.100, length=0.024),
        origin=Origin(xyz=(0.0, -0.100, 0.151)),
        material=zinc,
        name="gearbox_top_plate",
    )
    for sx in (-0.074, 0.074):
        for sy in (-0.052, 0.052):
            _add_bolt(frame, (sx, -0.100 + sy, 0.151), zinc, radius=0.006, height=0.012)
    frame.visual(
        Box((0.36, 0.038, 0.028)),
        origin=Origin(xyz=(0.0, -0.237, 0.063)),
        material=cast,
        name="motor_saddle",
    )
    frame.visual(
        Box((0.060, 0.030, 0.040)),
        origin=Origin(xyz=(0.150, -0.300, 0.096)),
        material=phosphate,
        name="sealed_connector",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.29),
        origin=Origin(xyz=(0.075, -0.285, 0.097), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rubber_cable_boot",
    )

    # Supported pivot towers with split side blocks and grease seals; the spindle
    # shafts run through the visible central clearance rather than through solids.
    for i, x in enumerate((-0.46, 0.46)):
        for sx in (-0.048, 0.048):
            frame.visual(
                Box((0.028, 0.112, 0.090)),
                origin=Origin(xyz=(x + sx, 0.028, 0.095)),
                material=cast,
                name=f"spindle_cheek_{i}_{sx:+.2f}",
            )
        frame.visual(
            Box((0.128, 0.025, 0.050)),
            origin=Origin(xyz=(x, 0.093, 0.080)),
            material=cast,
            name=f"spindle_bridge_{i}",
        )
        frame.visual(
            large_seal_mesh,
            origin=Origin(xyz=(x, 0.028, 0.146)),
            material=grease_blue,
            name=f"blue_grease_seal_{i}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(x + 0.072, 0.028, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=service_yellow,
            name=f"grease_nipple_{i}",
        )
        _add_bolt(frame, (x - 0.075, -0.020, 0.050), zinc, radius=0.006, height=0.012)
        _add_bolt(frame, (x + 0.075, -0.020, 0.050), zinc, radius=0.006, height=0.012)

    # Maintenance lid hinge brackets on the rear of the gearbox.
    frame.visual(
        Box((0.54, 0.160, 0.018)),
        origin=Origin(xyz=(0.0, -0.170, 0.059)),
        material=cast,
        name="rear_hinge_shelf",
    )
    for x in (-0.225, 0.225):
        frame.visual(
            Box((0.050, 0.035, 0.160)),
            origin=Origin(xyz=(x, -0.220, 0.140)),
            material=cast,
            name=f"cover_hinge_stand_{x:+.2f}",
        )
    frame.visual(
        Box((0.34, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.310, 0.202)),
        material=rubber,
        name="cover_gasket_lip",
    )
    frame.visual(
        Box((0.34, 0.018, 0.142)),
        origin=Origin(xyz=(0.0, -0.310, 0.130)),
        material=cast,
        name="cover_gasket_riser",
    )

    service_cover = model.part("service_cover")
    service_cover.visual(
        Box((0.38, 0.205, 0.018)),
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
        material=service_yellow,
        name="hinged_cover_panel",
    )
    service_cover.visual(
        Cylinder(radius=0.012, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="cover_hinge_barrel",
    )
    service_cover.visual(
        Box((0.092, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, -0.183, 0.018)),
        material=phosphate,
        name="quarter_turn_latch",
    )
    service_cover.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, -0.183, 0.037)),
        material=label_red,
        name="red_latch_button",
    )

    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_steel,
        name="splined_hub",
    )
    _add_member(
        motor_crank,
        (0.0, 0.0, 0.018),
        (-0.120, 0.040, 0.018),
        0.015,
        zinc,
        name="forged_crank_throw",
    )
    motor_crank.visual(
        Cylinder(radius=0.012, length=0.058),
        origin=Origin(xyz=(-0.120, 0.040, 0.038)),
        material=satin_steel,
        name="crank_pin",
    )
    motor_crank.visual(
        Box((0.050, 0.032, 0.022)),
        origin=Origin(xyz=(0.035, -0.014, 0.014)),
        material=cast,
        name="balance_weight",
    )

    drive_link = model.part("drive_link")
    drive_link.visual(rod_eye_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=bronze, name="crank_eye")
    _add_member(drive_link, (-0.024, -0.00015, 0.0), (-0.316, -0.00185, 0.0), 0.010, zinc, name="adjustable_tube")
    drive_link.visual(
        rod_eye_mesh,
        origin=Origin(xyz=(-0.340, -0.002, 0.0)),
        material=bronze,
        name="rocker_eye",
    )
    drive_link.visual(
        Box((0.070, 0.018, 0.018)),
        origin=Origin(xyz=(-0.175, -0.001, 0.0), rpy=(0.0, 0.0, math.atan2(-0.002, -0.340))),
        material=phosphate,
        name="threaded_adjuster_flat",
    )

    spindles = []
    for i, x in enumerate((-0.46, 0.46)):
        spindle = model.part(f"spindle_{i}")
        spindle.visual(
            Cylinder(radius=0.018, length=0.122),
            origin=Origin(xyz=(0.0, 0.0, 0.061)),
            material=satin_steel,
            name="vertical_spindle",
        )
        spindle.visual(
            Cylinder(radius=0.042, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, 0.130)),
            material=zinc,
            name="knurled_drive_hub",
        )
        spindle.visual(
            Box((0.050, 0.530, 0.022)),
            origin=Origin(xyz=(0.0, 0.300, 0.150)),
            material=phosphate,
            name="spring_steel_arm",
        )
        spindle.visual(
            Box((0.076, 0.075, 0.026)),
            origin=Origin(xyz=(0.0, 0.055, 0.145)),
            material=cast,
            name="tapered_arm_root",
        )
        _add_member(spindle, (-0.018, 0.060, 0.159), (-0.018, 0.255, 0.175), 0.007, zinc)
        _add_member(spindle, (0.018, 0.060, 0.159), (0.018, 0.255, 0.175), 0.007, zinc)
        _add_member(spindle, (0.0, 0.0, 0.075), (0.0, -0.090, 0.075), 0.012, zinc, name="drive_rocker")
        spindle.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.0, -0.090, 0.108)),
            material=satin_steel,
            name="rocker_pin",
        )
        spindle.visual(
            Cylinder(radius=0.016, length=0.090),
            origin=Origin(xyz=(0.0, -0.090, 0.165)),
            material=satin_steel,
            name="tie_pin",
        )
        spindle.visual(
            Box((0.016, 0.064, 0.042)),
            origin=Origin(xyz=(-0.034, 0.585, 0.150)),
            material=cast,
            name="blade_fork_cheek_a",
        )
        spindle.visual(
            Box((0.016, 0.064, 0.042)),
            origin=Origin(xyz=(0.034, 0.585, 0.150)),
            material=cast,
            name="blade_fork_cheek_b",
        )
        spindle.visual(
            Box((0.078, 0.020, 0.032)),
            origin=Origin(xyz=(0.0, 0.552, 0.150)),
            material=cast,
            name="blade_fork_bridge",
        )
        spindle.visual(
            Cylinder(radius=0.006, length=0.088),
            origin=Origin(xyz=(0.0, 0.590, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name="blade_hinge_pin",
        )
        spindles.append(spindle)

    cross_link = model.part("cross_link")
    cross_link.visual(rod_eye_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=bronze, name="near_eye")
    _add_member(cross_link, (0.024, 0.0, 0.0), (0.896, 0.0, 0.0), 0.011, zinc, name="synchronizer_tube")
    cross_link.visual(rod_eye_mesh, origin=Origin(xyz=(0.920, 0.0, 0.0)), material=bronze, name="far_eye")
    cross_link.visual(
        Box((0.082, 0.020, 0.018)),
        origin=Origin(xyz=(0.460, 0.0, 0.0)),
        material=phosphate,
        name="jam_nut_flat",
    )

    for i in range(2):
        carrier = model.part(f"blade_carrier_{i}")
        carrier.visual(
            Box((0.036, 0.036, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=cast,
            name="replaceable_adapter_lug",
        )
        carrier.visual(
            Box((0.040, 0.050, 0.022)),
            origin=Origin(xyz=(0.0, 0.043, -0.010)),
            material=zinc,
            name="adapter_neck",
        )
        carrier.visual(
            Box((0.430, 0.032, 0.024)),
            origin=Origin(xyz=(0.0, 0.070, -0.030)),
            material=phosphate,
            name="stainless_backbone",
        )
        carrier.visual(
            Box((0.470, 0.018, 0.052)),
            origin=Origin(xyz=(0.0, 0.083, -0.074)),
            material=rubber,
            name="rubber_squeegee",
        )
        carrier.visual(
            Box((0.500, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, 0.090, -0.103)),
            material=rubber,
            name="wiping_edge",
        )
        for sx in (-0.215, 0.215):
            carrier.visual(
                Box((0.024, 0.036, 0.030)),
                origin=Origin(xyz=(sx, 0.070, -0.034)),
                material=service_yellow,
                name=f"end_clip_{sx:+.2f}",
            )
        carrier.visual(
            Box((0.055, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.086, -0.022)),
            material=label_red,
            name="release_tab",
        )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_cover,
        origin=Origin(xyz=(0.0, -0.220, 0.220)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "motor_crank",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=motor_crank,
        origin=Origin(xyz=(0.0, -0.100, 0.158)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=5.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "crank_pin_joint",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(-0.120, 0.040, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-0.80, upper=0.80),
    )
    for i, spindle in enumerate(spindles):
        model.articulation(
            f"spindle_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=spindle,
            origin=Origin(xyz=(-0.46 if i == 0 else 0.46, 0.028, 0.088)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=35.0, velocity=3.0, lower=-0.58, upper=0.58),
            mimic=Mimic(joint="motor_crank", multiplier=0.55, offset=0.0),
        )
    model.articulation(
        "cross_link_pin",
        ArticulationType.REVOLUTE,
        parent=spindles[0],
        child=cross_link,
        origin=Origin(xyz=(0.0, -0.090, 0.165)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.55, upper=0.55),
    )
    for i, spindle in enumerate(spindles):
        model.articulation(
            f"blade_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=spindle,
            child=f"blade_carrier_{i}",
            origin=Origin(xyz=(0.0, 0.590, 0.150)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.18, upper=0.18),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crank = object_model.get_articulation("motor_crank")
    cover = object_model.get_articulation("cover_hinge")
    blade_0 = object_model.get_part("blade_carrier_0")
    blade_1 = object_model.get_part("blade_carrier_1")
    frame = object_model.get_part("service_frame")
    drive_link = object_model.get_part("drive_link")
    cross_link = object_model.get_part("cross_link")

    ctx.allow_overlap(
        "motor_crank",
        frame,
        elem_a="splined_hub",
        elem_b="gearbox_top_plate",
        reason="The crank hub is intentionally seated over the gearbox output bearing with a small hidden insertion.",
    )
    for i in range(2):
        ctx.allow_overlap(
            f"blade_carrier_{i}",
            f"spindle_{i}",
            elem_a="replaceable_adapter_lug",
            elem_b="blade_hinge_pin",
            reason="The replaceable blade adapter is captured by the hinge pin through its lug bore.",
        )
    ctx.allow_overlap(
        cross_link,
        "spindle_0",
        elem_a="near_eye",
        elem_b="tie_pin",
        reason="The synchronizer link eye is captured on the stacked tie pin with a small bushing-seat overlap.",
    )
    ctx.allow_overlap(
        cross_link,
        "spindle_1",
        elem_a="far_eye",
        elem_b="tie_pin",
        reason="The far synchronizer eye is captured on the second stacked tie pin with a small bushing-seat overlap.",
    )

    ctx.expect_overlap(
        "motor_crank",
        drive_link,
        axes="xy",
        elem_a="crank_pin",
        elem_b="crank_eye",
        min_overlap=0.020,
        name="crank pin sits inside replaceable drive-link eye",
    )
    ctx.expect_gap(
        "motor_crank",
        frame,
        axis="z",
        max_penetration=0.006,
        positive_elem="splined_hub",
        negative_elem="gearbox_top_plate",
        name="crank hub has only a shallow bearing-seat insertion",
    )
    for i in range(2):
        ctx.expect_overlap(
            f"blade_carrier_{i}",
            f"spindle_{i}",
            axes="xy",
            elem_a="replaceable_adapter_lug",
            elem_b="blade_hinge_pin",
            min_overlap=0.010,
            name=f"blade adapter {i} is pinned in its service fork",
        )
    ctx.expect_overlap(
        cross_link,
        "spindle_0",
        axes="xy",
        elem_a="near_eye",
        elem_b="tie_pin",
        min_overlap=0.020,
        name="near synchronizer eye is retained by its tie pin",
    )
    ctx.expect_overlap(
        cross_link,
        "spindle_1",
        axes="xy",
        elem_a="far_eye",
        elem_b="tie_pin",
        min_overlap=0.020,
        name="far synchronizer eye is retained by its tie pin",
    )
    ctx.expect_overlap(
        "spindle_0",
        drive_link,
        axes="xy",
        elem_a="rocker_pin",
        elem_b="rocker_eye",
        min_overlap=0.020,
        name="drive link reaches supported spindle rocker pin",
    )
    ctx.expect_overlap(
        "spindle_1",
        cross_link,
        axes="xy",
        elem_a="rocker_pin",
        elem_b="far_eye",
        min_overlap=0.020,
        name="cross link reaches second supported spindle pin",
    )
    ctx.expect_gap(
        "service_cover",
        frame,
        axis="z",
        max_penetration=0.001,
        max_gap=0.025,
        positive_elem="hinged_cover_panel",
        negative_elem="cover_gasket_lip",
        name="maintenance cover sits on service gasket without collision",
    )

    with ctx.pose({crank: -1.05}):
        low_0 = ctx.part_world_position(blade_0)
        low_1 = ctx.part_world_position(blade_1)
    with ctx.pose({crank: 1.05}):
        high_0 = ctx.part_world_position(blade_0)
        high_1 = ctx.part_world_position(blade_1)
    ctx.check(
        "motor crank sweeps both blade carriers through spindle pivots",
        low_0 is not None
        and high_0 is not None
        and low_1 is not None
        and high_1 is not None
        and abs(high_0[0] - low_0[0]) > 0.55
        and abs(high_1[0] - low_1[0]) > 0.55,
        details=f"blade_0 low/high={low_0}/{high_0}, blade_1 low/high={low_1}/{high_1}",
    )

    with ctx.pose({cover: 1.0}):
        ctx.expect_gap(
            "service_cover",
            frame,
            axis="z",
            min_gap=0.03,
            positive_elem="quarter_turn_latch",
            negative_elem="gearbox_top_plate",
            name="opened cover clears gearbox for maintenance access",
        )

    return ctx.report()


object_model = build_object_model()
