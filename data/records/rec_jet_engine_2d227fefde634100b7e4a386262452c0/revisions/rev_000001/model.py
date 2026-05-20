from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    FanRotorShroud,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireTread,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="auxiliary_jet_engine_dolly")

    model.material("brushed_steel", rgba=(0.56, 0.58, 0.58, 1.0))
    model.material("dark_burnt_metal", rgba=(0.10, 0.095, 0.085, 1.0))
    model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("safety_yellow", rgba=(1.0, 0.70, 0.06, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("bearing_grey", rgba=(0.36, 0.38, 0.39, 1.0))
    model.material("hot_nozzle", rgba=(0.20, 0.17, 0.13, 1.0))

    dolly = model.part("dolly")
    engine_body = model.part("engine_body")
    intake_rotor = model.part("intake_rotor")
    exhaust_sleeve = model.part("exhaust_sleeve")

    # Compact test dolly: a welded rectangular frame with fixed utility wheels,
    # two cradle stations, and rubber saddle pads carrying the cylindrical engine.
    dolly.visual(
        Box((1.46, 0.052, 0.055)),
        origin=Origin(xyz=(0.0, -0.245, 0.145)),
        material="safety_yellow",
        name="side_rail_0",
    )
    dolly.visual(
        Box((1.46, 0.052, 0.055)),
        origin=Origin(xyz=(0.0, 0.245, 0.145)),
        material="safety_yellow",
        name="side_rail_1",
    )
    for i, x in enumerate((-0.58, -0.05, 0.48)):
        dolly.visual(
            Box((0.075, 0.545, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.145)),
            material="safety_yellow",
            name=f"cross_beam_{i}",
        )
    dolly.visual(
        Box((1.12, 0.36, 0.032)),
        origin=Origin(xyz=(-0.02, 0.0, 0.186)),
        material="brushed_steel",
        name="service_deck",
    )

    for station, x in enumerate((-0.28, 0.22)):
        for side, y in enumerate((-0.135, 0.135)):
            dolly.visual(
                Box((0.055, 0.050, 0.250)),
                origin=Origin(xyz=(x, y, 0.327)),
                material="safety_yellow",
                name=f"cradle_post_{station}_{side}",
            )
        dolly.visual(
            Box((0.115, 0.330, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.463)),
            material="rubber_black",
            name=f"saddle_pad_{station}",
        )

    axle_origin = (-math.pi / 2.0, 0.0, 0.0)
    for i, x in enumerate((-0.53, 0.48)):
        dolly.visual(
            Cylinder(radius=0.018, length=0.66),
            origin=Origin(xyz=(x, 0.0, 0.075), rpy=axle_origin),
            material="bearing_grey",
            name=f"axle_{i}",
        )
        for iy, y in enumerate((-0.245, 0.245)):
            dolly.visual(
                Box((0.050, 0.038, 0.120)),
                origin=Origin(xyz=(x, y, 0.103)),
                material="safety_yellow",
                name=f"axle_hanger_{i}_{iy}",
            )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.082,
            0.045,
            inner_radius=0.055,
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.56),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "dolly_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.058,
            0.047,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(radius=0.020, width=0.032, cap_style="domed"),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0028),
        ),
        "dolly_wheel",
    )
    for ix, x in enumerate((-0.53, 0.48)):
        for iy, y in enumerate((-0.335, 0.335)):
            dolly.visual(
                tire_mesh,
                origin=Origin(xyz=(x, y, 0.075), rpy=(0.0, 0.0, math.pi / 2.0)),
                material="rubber_black",
                name=f"tire_{ix}_{iy}",
            )
            dolly.visual(
                wheel_mesh,
                origin=Origin(xyz=(x, y, 0.075), rpy=(0.0, 0.0, math.pi / 2.0)),
                material="bearing_grey",
                name=f"wheel_{ix}_{iy}",
            )
            dolly.visual(
                Cylinder(radius=0.056, length=0.052),
                origin=Origin(xyz=(x, y, 0.075), rpy=axle_origin),
                material="bearing_grey",
                name=f"wheel_core_{ix}_{iy}",
            )

    # The engine body is a thin-walled lathed duct: flared intake lip, constant
    # compressor case, and a taper down to the exhaust nozzle socket.
    body_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.178, -0.585),
            (0.205, -0.552),
            (0.194, -0.500),
            (0.184, -0.285),
            (0.182, -0.035),
            (0.166, 0.180),
            (0.136, 0.345),
        ],
        inner_profile=[
            (0.136, -0.585),
            (0.150, -0.552),
            (0.148, -0.500),
            (0.135, -0.285),
            (0.124, -0.035),
            (0.112, 0.180),
            (0.098, 0.345),
        ],
        segments=72,
        start_cap="round",
        end_cap="flat",
    )
    engine_body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="body_shell",
    )

    exhaust_nozzle = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.114, 0.210),
            (0.108, 0.310),
            (0.100, 0.500),
            (0.086, 0.660),
        ],
        inner_profile=[
            (0.079, 0.210),
            (0.079, 0.310),
            (0.074, 0.500),
            (0.064, 0.660),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    engine_body.visual(
        mesh_from_geometry(exhaust_nozzle, "exhaust_nozzle"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="hot_nozzle",
        name="exhaust_nozzle",
    )

    # Stationary intake spiders and bearing nose behind the spinning rotor.
    engine_body.visual(
        Box((0.018, 0.306, 0.018)),
        origin=Origin(xyz=(-0.525, 0.0, 0.0)),
        material="dark_burnt_metal",
        name="intake_spider_y",
    )
    engine_body.visual(
        Box((0.018, 0.018, 0.306)),
        origin=Origin(xyz=(-0.525, 0.0, 0.0)),
        material="dark_burnt_metal",
        name="intake_spider_z",
    )
    engine_body.visual(
        Cylinder(radius=0.038, length=0.072),
        origin=Origin(xyz=(-0.525, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bearing_grey",
        name="front_bearing",
    )
    engine_body.visual(
        Cylinder(radius=0.006, length=0.150),
        origin=Origin(xyz=(-0.575, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="bearing_grey",
        name="rotor_shaft",
    )
    engine_body.visual(
        Box((0.300, 0.026, 0.035)),
        origin=Origin(xyz=(0.510, 0.0, 0.120)),
        material="bearing_grey",
        name="top_sleeve_guide",
    )
    for i, y in enumerate((-0.125, 0.125)):
        engine_body.visual(
            Box((0.300, 0.040, 0.030)),
            origin=Origin(xyz=(0.510, y, 0.0)),
            material="bearing_grey",
            name=f"side_sleeve_guide_{i}",
        )

    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.128,
            0.034,
            8,
            thickness=0.030,
            blade_pitch_deg=33.0,
            blade_sweep_deg=25.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.12),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.010, bore_diameter=0.009),
            shroud=FanRotorShroud(thickness=0.004, depth=0.012, clearance=0.002),
        ),
        "intake_rotor",
    )
    intake_rotor.visual(
        rotor_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_burnt_metal",
        name="rotor_blades",
    )

    sleeve_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.144, 0.000),
            (0.160, 0.030),
            (0.160, 0.335),
            (0.146, 0.380),
        ],
        inner_profile=[
            (0.124, 0.000),
            (0.134, 0.030),
            (0.134, 0.335),
            (0.121, 0.380),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
    )
    exhaust_sleeve.visual(
        mesh_from_geometry(sleeve_shell, "sleeve_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_burnt_metal",
        name="sleeve_shell",
    )
    exhaust_sleeve.visual(
        Box((0.300, 0.026, 0.018)),
        origin=Origin(xyz=(0.190, 0.0, 0.162)),
        material="bearing_grey",
        name="top_slide_rail",
    )
    for i, y in enumerate((-0.164, 0.164)):
        exhaust_sleeve.visual(
            Box((0.270, 0.016, 0.030)),
            origin=Origin(xyz=(0.195, y, 0.0)),
            material="bearing_grey",
            name=f"side_slide_rail_{i}",
        )

    model.articulation(
        "dolly_to_engine",
        ArticulationType.FIXED,
        parent=dolly,
        child=engine_body,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
    )
    model.articulation(
        "engine_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=engine_body,
        child=intake_rotor,
        origin=Origin(xyz=(-0.595, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=180.0),
    )
    model.articulation(
        "engine_to_sleeve",
        ArticulationType.PRISMATIC,
        parent=engine_body,
        child=exhaust_sleeve,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    dolly = object_model.get_part("dolly")
    engine_body = object_model.get_part("engine_body")
    intake_rotor = object_model.get_part("intake_rotor")
    exhaust_sleeve = object_model.get_part("exhaust_sleeve")
    rotor_joint = object_model.get_articulation("engine_to_rotor")
    sleeve_joint = object_model.get_articulation("engine_to_sleeve")

    ctx.check(
        "intake rotor is continuous",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotor_joint.articulation_type}",
    )
    ctx.check(
        "exhaust sleeve is prismatic",
        sleeve_joint.articulation_type == ArticulationType.PRISMATIC
        and sleeve_joint.axis == (1.0, 0.0, 0.0),
        details=f"type={sleeve_joint.articulation_type}, axis={sleeve_joint.axis}",
    )

    for pad_name in ("saddle_pad_0", "saddle_pad_1"):
        ctx.allow_overlap(
            dolly,
            engine_body,
            elem_a=pad_name,
            elem_b="body_shell",
            reason="The rubber cradle pad is slightly compressed against the engine casing.",
        )
        ctx.expect_gap(
            engine_body,
            dolly,
            axis="z",
            positive_elem="body_shell",
            negative_elem=pad_name,
            max_penetration=0.030,
            max_gap=0.004,
            name=f"{pad_name} seats under the engine casing",
        )
        ctx.expect_overlap(
            dolly,
            engine_body,
            axes="xy",
            elem_a=pad_name,
            elem_b="body_shell",
            min_overlap=0.08,
            name=f"{pad_name} is under the cylindrical body",
        )

    # The fan is visibly captured on a small stationary shaft; the tiny local
    # overlap stands in for the bore/bearing fit rather than a free-floating fan.
    ctx.allow_overlap(
        engine_body,
        intake_rotor,
        elem_a="rotor_shaft",
        elem_b="rotor_blades",
        reason="The intake fan hub is intentionally captured on the engine shaft/bearing.",
    )
    ctx.expect_overlap(
        intake_rotor,
        engine_body,
        axes="x",
        elem_a="rotor_blades",
        elem_b="rotor_shaft",
        min_overlap=0.020,
        name="rotor remains engaged on the shaft",
    )
    ctx.expect_within(
        engine_body,
        intake_rotor,
        axes="yz",
        inner_elem="rotor_shaft",
        outer_elem="rotor_blades",
        margin=0.002,
        name="shaft stays centered in the rotor hub envelope",
    )

    # Bronze/steel guide shoes are represented as local seated fits against the
    # sleeve shell; they keep the sleeve grounded while allowing axial travel.
    for guide_name in ("top_sleeve_guide", "side_sleeve_guide_0", "side_sleeve_guide_1"):
        ctx.allow_overlap(
            engine_body,
            exhaust_sleeve,
            elem_a=guide_name,
            elem_b="sleeve_shell",
            reason="The exhaust sleeve slides over fixed low-friction guide shoes.",
        )
        ctx.expect_overlap(
            engine_body,
            exhaust_sleeve,
            axes="x",
            elem_a=guide_name,
            elem_b="sleeve_shell",
            min_overlap=0.10,
            name=f"{guide_name} has axial bearing length in the sleeve",
        )
        ctx.expect_within(
            engine_body,
            exhaust_sleeve,
            axes="yz",
            inner_elem=guide_name,
            outer_elem="sleeve_shell",
            margin=0.004,
            name=f"{guide_name} is contained by the sleeve envelope",
        )

    ctx.expect_overlap(
        exhaust_sleeve,
        engine_body,
        axes="x",
        elem_a="sleeve_shell",
        elem_b="exhaust_nozzle",
        min_overlap=0.20,
        name="collapsed sleeve overlaps the exhaust nozzle length",
    )
    ctx.expect_within(
        engine_body,
        exhaust_sleeve,
        axes="yz",
        inner_elem="exhaust_nozzle",
        outer_elem="sleeve_shell",
        margin=0.004,
        name="exhaust nozzle sits inside the sleeve envelope",
    )

    rest_pos = ctx.part_world_position(exhaust_sleeve)
    with ctx.pose({sleeve_joint: 0.180}):
        ctx.expect_overlap(
            exhaust_sleeve,
            engine_body,
            axes="x",
            elem_a="sleeve_shell",
            elem_b="exhaust_nozzle",
            min_overlap=0.08,
            name="extended sleeve keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(exhaust_sleeve)

    ctx.check(
        "sleeve extends aft along the engine axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
