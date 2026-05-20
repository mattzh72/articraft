from __future__ import annotations

import math

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


def _hollow_cylinder_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    segments: int = 72,
):
    """Thin-walled revolved sleeve with a real open bore, aligned to local Z."""
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=1,
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_rotary_wrist_study")

    model.material("ground_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("black_oxide", rgba=(0.01, 0.012, 0.014, 1.0))
    model.material("warm_steel", rgba=(0.36, 0.35, 0.32, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("safety_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    model.material("cable_rubber", rgba=(0.015, 0.015, 0.018, 1.0))

    # Root linear-axis structure: a welded/machined bed, supported guide rods,
    # a separate exposed screw, end bearings, and end-stop blocks.
    base = model.part("base_frame")
    base.visual(
        Box((1.22, 0.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="dark_anodized",
        name="bed_plate",
    )
    base.visual(
        Box((1.16, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, -0.196, 0.050)),
        material="warm_steel",
        name="front_edge_stiffener",
    )
    base.visual(
        Box((1.16, 0.038, 0.030)),
        origin=Origin(xyz=(0.0, 0.196, 0.050)),
        material="warm_steel",
        name="rear_edge_stiffener",
    )

    for x in (-0.48, 0.48):
        for y in (-0.125, 0.125):
            base.visual(
                Box((0.075, 0.070, 0.043)),
                origin=Origin(xyz=(x, y, 0.0565)),
                material="machined_aluminum",
                name=f"rail_pedestal_{x}_{y}",
            )
            base.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(x - 0.020, y, 0.080), rpy=(0.0, 0.0, 0.0)),
                material="black_oxide",
                name=f"pedestal_screw_a_{x}_{y}",
            )
            base.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(x + 0.020, y, 0.080), rpy=(0.0, 0.0, 0.0)),
                material="black_oxide",
                name=f"pedestal_screw_b_{x}_{y}",
            )

    for y, rail_name in ((-0.125, "front_rail"), (0.125, "rear_rail")):
        base.visual(
            Cylinder(radius=0.018, length=1.08),
            origin=Origin(xyz=(0.0, y, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="ground_steel",
            name=rail_name,
        )
        base.visual(
            Box((1.06, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.055)),
            material="warm_steel",
            name=f"{rail_name}_key_strip",
        )

    base.visual(
        Cylinder(radius=0.012, length=1.14),
        origin=Origin(xyz=(0.0, 0.0, 0.074), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="ground_steel",
        name="exposed_drive_screw",
    )
    for x, name in ((-0.555, "fixed_bearing_block"), (0.555, "motor_bearing_block")):
        base.visual(
            Box((0.060, 0.110, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material="machined_aluminum",
            name=name,
        )
        base.visual(
            Cylinder(radius=0.023, length=0.010),
            origin=Origin(
                xyz=(x + (0.032 if x < 0 else -0.032), 0.0, 0.074),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="black_oxide",
            name=f"{name}_cap",
        )

    for x, bumper_dir, tag in ((-0.53, 1.0, "negative"), (0.53, -1.0, "positive")):
        base.visual(
            Box((0.052, 0.090, 0.060)),
            origin=Origin(xyz=(x, 0.178, 0.065)),
            material="safety_yellow",
            name=f"{tag}_end_stop",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.030),
            origin=Origin(
                xyz=(x + bumper_dir * 0.035, 0.178, 0.066),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="black_oxide",
            name=f"{tag}_stop_bumper",
        )

    base.visual(
        Box((0.86, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, -0.062, 0.104)),
        material="warm_steel",
        name="screw_access_cover",
    )
    base.visual(
        Box((0.86, 0.006, 0.062)),
        origin=Origin(xyz=(0.0, -0.082, 0.066)),
        material="warm_steel",
        name="cover_front_flange",
    )
    base.visual(
        Box((0.86, 0.006, 0.062)),
        origin=Origin(xyz=(0.0, -0.042, 0.066)),
        material="warm_steel",
        name="cover_rear_flange",
    )
    for x in (-0.36, -0.18, 0.0, 0.18, 0.36):
        base.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, -0.062, 0.113)),
            material="black_oxide",
            name=f"cover_fastener_{x}",
        )

    # Moving carriage: two hollow linear bearing sleeves around the guide rods,
    # a rigid saddle plate, a framed wrist mount, and a cable-loop stand-in.
    carriage = model.part("carriage")
    for y, sleeve_name in ((-0.125, "front_bushing"), (0.125, "rear_bushing")):
        carriage.visual(
            _hollow_cylinder_mesh(
                outer_radius=0.035,
                inner_radius=0.0215,
                length=0.230,
                name=f"{sleeve_name}_mesh",
            ),
            origin=Origin(xyz=(0.0, y, 0.098), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="machined_aluminum",
            name=sleeve_name,
        )
        carriage.visual(
            Box((0.230, 0.075, 0.034)),
            origin=Origin(xyz=(0.0, y, 0.132)),
            material="machined_aluminum",
            name=f"{sleeve_name}_saddle",
        )
        carriage.visual(
            Box((0.050, 0.075, 0.034)),
            origin=Origin(xyz=(-0.070, y, 0.148)),
            material="dark_anodized",
            name=f"{sleeve_name}_clamp_a",
        )
        carriage.visual(
            Box((0.050, 0.075, 0.034)),
            origin=Origin(xyz=(0.070, y, 0.148)),
            material="dark_anodized",
            name=f"{sleeve_name}_clamp_b",
        )
        carriage.visual(
            Box((0.190, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, y, 0.123)),
            material="ground_steel",
            name=f"{sleeve_name}_wear_pad",
        )

    carriage.visual(
        Box((0.285, 0.360, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material="machined_aluminum",
        name="saddle_plate",
    )
    carriage.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.028,
            inner_radius=0.0155,
            length=0.115,
            name="drive_nut_mesh",
            segments=60,
        ),
        origin=Origin(xyz=(-0.005, 0.0, 0.074), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="warm_steel",
        name="drive_nut",
    )
    carriage.visual(
        Box((0.120, 0.055, 0.058)),
        origin=Origin(xyz=(-0.005, 0.0, 0.116)),
        material="machined_aluminum",
        name="nut_yoke",
    )

    carriage.visual(
        Box((0.180, 0.200, 0.009)),
        origin=Origin(xyz=(-0.040, 0.0, 0.177)),
        material="warm_steel",
        name="top_access_cover",
    )
    for x in (-0.108, 0.028):
        for y in (-0.072, 0.072):
            carriage.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.1835)),
                material="black_oxide",
                name=f"top_cover_screw_{x}_{y}",
            )

    # Fabricated rectangular wrist-support frame: leaves a true open center for
    # the wrist shaft while visibly tying the bearing housing into the carriage.
    carriage.visual(
        Box((0.048, 0.034, 0.220)),
        origin=Origin(xyz=(0.165, -0.148, 0.255)),
        material="machined_aluminum",
        name="front_frame_column_0",
    )
    carriage.visual(
        Box((0.048, 0.034, 0.220)),
        origin=Origin(xyz=(0.165, 0.148, 0.255)),
        material="machined_aluminum",
        name="front_frame_column_1",
    )
    carriage.visual(
        Box((0.048, 0.330, 0.036)),
        origin=Origin(xyz=(0.165, 0.0, 0.359)),
        material="machined_aluminum",
        name="front_frame_bridge_top",
    )
    carriage.visual(
        Box((0.048, 0.330, 0.030)),
        origin=Origin(xyz=(0.165, 0.0, 0.153)),
        material="machined_aluminum",
        name="front_frame_bridge_bottom",
    )
    carriage.visual(
        Box((0.180, 0.030, 0.150)),
        origin=Origin(xyz=(0.085, -0.170, 0.245)),
        material="warm_steel",
        name="side_gusset_0",
    )
    carriage.visual(
        Box((0.180, 0.030, 0.150)),
        origin=Origin(xyz=(0.085, 0.170, 0.245)),
        material="warm_steel",
        name="side_gusset_1",
    )

    carriage.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.076,
            inner_radius=0.040,
            length=0.060,
            name="bearing_housing_mesh",
            segments=96,
        ),
        origin=Origin(xyz=(0.185, 0.0, 0.236), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_anodized",
        name="bearing_housing",
    )
    carriage.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.058,
            inner_radius=0.040,
            length=0.066,
            name="bearing_race_mesh",
            segments=96,
        ),
        origin=Origin(xyz=(0.188, 0.0, 0.236), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="ground_steel",
        name="bearing_race",
    )
    for y, z, name in (
        (0.0, 0.324, "upper_bearing_lug"),
        (0.0, 0.148, "lower_bearing_lug"),
        (-0.103, 0.236, "side_bearing_lug_0"),
        (0.103, 0.236, "side_bearing_lug_1"),
    ):
        carriage.visual(
            Box((0.055, 0.056, 0.042)),
            origin=Origin(xyz=(0.185, y, z)),
            material="machined_aluminum",
            name=name,
        )

    cable_mesh = tube_from_spline_points(
        [
            (-0.105, -0.196, 0.185),
            (-0.160, -0.270, 0.270),
            (-0.020, -0.310, 0.330),
            (0.105, -0.248, 0.280),
            (0.130, -0.196, 0.205),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    carriage.visual(
        mesh_from_geometry(cable_mesh, "cable_loop_mesh"),
        material="cable_rubber",
        name="cable_loop",
    )
    carriage.visual(
        Box((0.050, 0.024, 0.036)),
        origin=Origin(xyz=(-0.105, -0.182, 0.184)),
        material="black_oxide",
        name="cable_clamp_0",
    )
    carriage.visual(
        Box((0.050, 0.024, 0.036)),
        origin=Origin(xyz=(0.130, -0.182, 0.205)),
        material="black_oxide",
        name="cable_clamp_1",
    )

    # Rotary end-effector: independent rotor with shaft, hub, machined
    # faceplate, bolt circle, and tooling pilot.
    rotor = model.part("wrist_rotor")
    rotor.visual(
        Cylinder(radius=0.032, length=0.205),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="ground_steel",
        name="rotor_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.042, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="ground_steel",
        name="thrust_collar",
    )
    rotor.visual(
        Cylinder(radius=0.056, length=0.060),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_anodized",
        name="drive_hub",
    )
    rotor.visual(
        Cylinder(radius=0.108, length=0.024),
        origin=Origin(xyz=(0.172, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="tooling_faceplate",
    )
    rotor.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="ground_steel",
        name="tooling_pilot",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        y = 0.072 * math.cos(angle)
        z = 0.072 * math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(0.187, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="black_oxide",
            name=f"tooling_socket_{i}",
        )
    for i in range(4):
        angle = (i + 0.5) * math.tau / 4.0
        y = 0.043 * math.cos(angle)
        z = 0.043 * math.sin(angle)
        rotor.visual(
            Box((0.006, 0.010, 0.022)),
            origin=Origin(xyz=(0.187, y, z), rpy=(angle, 0.0, 0.0)),
            material="black_oxide",
            name=f"key_slot_{i}",
        )

    model.articulation(
        "linear_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.225, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.55, lower=0.0, upper=0.450),
    )
    model.articulation(
        "wrist_axis",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=rotor,
        origin=Origin(xyz=(0.185, 0.0, 0.236)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=4.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    carriage = object_model.get_part("carriage")
    rotor = object_model.get_part("wrist_rotor")
    linear_axis = object_model.get_articulation("linear_axis")
    wrist_axis = object_model.get_articulation("wrist_axis")

    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="front_bushing",
        elem_b="front_rail",
        min_overlap=0.200,
        name="front bushing stays engaged on rail at home",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="rear_bushing",
        elem_b="rear_rail",
        min_overlap=0.200,
        name="rear bushing stays engaged on rail at home",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="yz",
        inner_elem="front_bushing",
        outer_elem="front_rail",
        margin=0.020,
        name="front bushing concentric with rail envelope",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="yz",
        inner_elem="rear_bushing",
        outer_elem="rear_rail",
        margin=0.020,
        name="rear bushing concentric with rail envelope",
    )

    ctx.expect_within(
        rotor,
        carriage,
        axes="yz",
        inner_elem="rotor_shaft",
        outer_elem="bearing_housing",
        margin=0.006,
        name="wrist shaft centered in bearing housing",
    )
    ctx.expect_gap(
        rotor,
        carriage,
        axis="x",
        positive_elem="tooling_faceplate",
        negative_elem="bearing_housing",
        min_gap=0.040,
        name="tooling plate projects beyond bearing housing",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({linear_axis: 0.450}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="front_bushing",
            elem_b="front_rail",
            min_overlap=0.200,
            name="front bushing retained at full travel",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="rear_bushing",
            elem_b="rear_rail",
            min_overlap=0.200,
            name="rear bushing retained at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "linear joint translates carriage along rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.400,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rotor_origin = ctx.part_world_position(rotor)
    with ctx.pose({wrist_axis: 1.25}):
        rotated_origin = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            carriage,
            axes="yz",
            inner_elem="rotor_shaft",
            outer_elem="bearing_housing",
            margin=0.006,
            name="rotated shaft remains in bearing",
        )
    ctx.check(
        "wrist joint rotates without translating rotor origin",
        rotor_origin is not None
        and rotated_origin is not None
        and abs(rotor_origin[0] - rotated_origin[0]) < 1e-6
        and abs(rotor_origin[1] - rotated_origin[1]) < 1e-6
        and abs(rotor_origin[2] - rotated_origin[2]) < 1e-6,
        details=f"rest={rotor_origin}, rotated={rotated_origin}",
    )

    return ctx.report()


object_model = build_object_model()
