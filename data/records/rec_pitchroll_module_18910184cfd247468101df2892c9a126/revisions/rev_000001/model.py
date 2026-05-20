from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_prism(outer: float, inner: float, thickness: float, segments: int = 128) -> MeshGeometry:
    geom = MeshGeometry()
    z0 = -thickness / 2.0
    z1 = thickness / 2.0
    outer_front: list[int] = []
    outer_back: list[int] = []
    inner_front: list[int] = []
    inner_back: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_front.append(geom.add_vertex(outer * ca, outer * sa, z0))
        outer_back.append(geom.add_vertex(outer * ca, outer * sa, z1))
        inner_front.append(geom.add_vertex(inner * ca, inner * sa, z0))
        inner_back.append(geom.add_vertex(inner * ca, inner * sa, z1))
    for i in range(segments):
        j = (i + 1) % segments
        # outer cylindrical wall
        geom.add_face(outer_front[i], outer_front[j], outer_back[j])
        geom.add_face(outer_front[i], outer_back[j], outer_back[i])
        # inner cylindrical wall around the clear central opening
        geom.add_face(inner_front[j], inner_front[i], inner_back[i])
        geom.add_face(inner_front[j], inner_back[i], inner_back[j])
        # front and rear machined annular faces
        geom.add_face(outer_front[j], outer_front[i], inner_front[i])
        geom.add_face(outer_front[j], inner_front[i], inner_front[j])
        geom.add_face(outer_back[i], outer_back[j], inner_back[j])
        geom.add_face(outer_back[i], inner_back[j], inner_back[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_two_axis_instrument_head")

    dark_anodized = model.material("dark_anodized", rgba=(0.07, 0.08, 0.09, 1.0))
    bead_blasted = model.material("bead_blasted", rgba=(0.66, 0.68, 0.70, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.95, 0.42, 0.09, 1.0))

    # A flat machined annulus, not a round torus: the broad front and rear faces
    # make the outer roll package read like a compact instrument bearing ring.
    roll_ring_mesh = mesh_from_geometry(
        _annular_prism(0.235, 0.170, 0.066),
        "roll_ring_annulus",
    )
    roll_hub_mesh = mesh_from_geometry(
        _annular_prism(0.190, 0.150, 0.032),
        "roll_hub_annulus",
    )
    roll_cap_mesh = mesh_from_geometry(
        _annular_prism(0.180, 0.154, 0.018),
        "roll_cap_annulus",
    )
    roll_journal_mesh = mesh_from_geometry(
        _annular_prism(0.182, 0.150, 0.088),
        "roll_journal_annulus",
    )
    pedestal_bearing_mesh = mesh_from_geometry(
        _annular_prism(0.200, 0.145, 0.050),
        "pedestal_roll_bearing",
    )
    pedestal_cover_mesh = mesh_from_geometry(
        _annular_prism(0.208, 0.150, 0.012),
        "pedestal_bearing_cover",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.205, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=dark_anodized,
        name="round_base",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.365),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=bead_blasted,
        name="short_pedestal",
    )
    pedestal.visual(
        Box((0.450, 0.095, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.408)),
        material=dark_anodized,
        name="low_crosshead",
    )
    for x, suffix in ((0.190, "front"), (-0.190, "rear")):
        pedestal.visual(
            Box((0.060, 0.094, 0.047)),
            origin=Origin(xyz=(x, 0.0, 0.4515)),
            material=dark_anodized,
            name=f"{suffix}_bearing_pillar",
        )
        pedestal.visual(
            pedestal_bearing_mesh,
            origin=Origin(xyz=(x, 0.0, 0.675), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"{suffix}_roll_bearing",
        )
        pedestal.visual(
            pedestal_cover_mesh,
            origin=Origin(xyz=(x + (0.031 if x > 0 else -0.031), 0.0, 0.675), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cover_gray,
            name=f"{suffix}_bearing_cover",
        )
    pedestal.visual(
        Box((0.155, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.055, 0.355)),
        material=cover_gray,
        name="service_cover",
    )
    pedestal.visual(
        Box((0.035, 0.018, 0.018)),
        origin=Origin(xyz=(0.070, 0.057, 0.374)),
        material=warning_orange,
        name="index_mark",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.370, 0.260, 0.730)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
    )

    roll_ring = model.part("roll_ring")
    roll_ring.visual(
        roll_ring_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bead_blasted,
        name="machined_ring",
    )
    for x, suffix in ((0.068, "front"), (-0.068, "rear")):
        roll_ring.visual(
            roll_hub_mesh,
            origin=Origin(xyz=(0.047 if x > 0 else -0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"{suffix}_roll_hub",
        )
        roll_ring.visual(
            roll_cap_mesh,
            origin=Origin(xyz=(0.072 if x > 0 else -0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_anodized,
            name=f"{suffix}_roll_cap",
        )
        roll_ring.visual(
            roll_journal_mesh,
            origin=Origin(xyz=(0.123 if x > 0 else -0.123, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"{suffix}_roll_journal",
        )
    roll_ring.visual(
        Cylinder(radius=0.052, length=0.058),
        origin=Origin(xyz=(0.0, 0.147, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="side_0_pitch_boss",
    )
    roll_ring.visual(
        Cylinder(radius=0.063, length=0.014),
        origin=Origin(xyz=(0.0, 0.183, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="side_0_trunnion_cap",
    )
    roll_ring.visual(
        Cylinder(radius=0.052, length=0.058),
        origin=Origin(xyz=(0.0, -0.147, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="side_1_pitch_boss",
    )
    roll_ring.visual(
        Cylinder(radius=0.063, length=0.014),
        origin=Origin(xyz=(0.0, -0.183, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="side_1_trunnion_cap",
    )
    for z, suffix in ((0.166, "upper"), (-0.166, "lower")):
        roll_ring.visual(
            Box((0.010, 0.086, 0.028)),
            origin=Origin(xyz=(0.038, 0.0, z)),
            material=cover_gray,
            name=f"{suffix}_face_cover",
        )
    for y, suffix in ((0.090, "side_0"), (-0.090, "side_1")):
        roll_ring.visual(
            Box((0.012, 0.038, 0.018)),
            origin=Origin(xyz=(-0.038, y, -0.168)),
            material=warning_orange,
            name=f"{suffix}_roll_stop",
        )
    roll_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=0.250),
        mass=2.4,
        origin=Origin(),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.020, length=0.230),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="pitch_shaft",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.0, 0.128, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="side_0_shaft_cap",
    )
    pitch_cradle.visual(
        Box((0.060, 0.016, 0.158)),
        origin=Origin(xyz=(0.020, 0.09344, 0.0)),
        material=dark_anodized,
        name="side_0_cheek",
    )
    pitch_cradle.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.0, -0.128, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="side_1_shaft_cap",
    )
    pitch_cradle.visual(
        Box((0.060, 0.016, 0.158)),
        origin=Origin(xyz=(0.020, -0.09344, 0.0)),
        material=dark_anodized,
        name="side_1_cheek",
    )
    for z, suffix in ((0.074, "upper"), (-0.074, "lower")):
        pitch_cradle.visual(
            Box((0.055, 0.210, 0.020)),
            origin=Origin(xyz=(0.022, 0.0, z)),
            material=dark_anodized,
            name=f"{suffix}_bridge",
        )
    pitch_cradle.visual(
        Box((0.016, 0.180, 0.090)),
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        material=dark_anodized,
        name="tool_mount",
    )
    pitch_cradle.visual(
        Box((0.030, 0.118, 0.030)),
        origin=Origin(xyz=(-0.018, 0.0, -0.094)),
        material=cover_gray,
        name="cable_cover",
    )
    pitch_cradle.visual(
        Box((0.026, 0.025, 0.032)),
        origin=Origin(xyz=(-0.010, 0.0, 0.099)),
        material=warning_orange,
        name="pitch_stop_flag",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.115, 0.265, 0.190)),
        mass=1.15,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    tool_plate = model.part("tool_plate")
    for y, z, standoff_name in (
        (-0.048, -0.030, "standoff_n_n"),
        (-0.048, 0.030, "standoff_n_p"),
        (0.048, -0.030, "standoff_p_n"),
    ):
        tool_plate.visual(
            Cylinder(radius=0.006, length=0.090),
            origin=Origin(xyz=(0.045, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_steel,
            name=standoff_name,
        )
    tool_plate.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.045, 0.048, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="standoff_p_p",
    )
    tool_plate.visual(
        Box((0.014, 0.138, 0.094)),
        origin=Origin(xyz=(0.067, 0.0, 0.0)),
        material=bead_blasted,
        name="tool_face",
    )
    tool_plate.visual(
        Box((0.018, 0.080, 0.012)),
        origin=Origin(xyz=(0.076, 0.0, 0.031)),
        material=satin_black,
        name="sensor_slot",
    )
    tool_plate.visual(
        Cylinder(radius=0.012, length=0.009),
        origin=Origin(xyz=(0.0785, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="tool_boss",
    )
    tool_plate.inertial = Inertial.from_geometry(
        Box((0.060, 0.150, 0.105)),
        mass=0.45,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=roll_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.2,
            lower=math.radians(-70.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=roll_ring,
        child=pitch_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=math.radians(-30.0),
            upper=math.radians(30.0),
        ),
    )
    model.articulation(
        "plate_mount",
        ArticulationType.FIXED,
        parent=pitch_cradle,
        child=tool_plate,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    roll_ring = object_model.get_part("roll_ring")
    pitch_cradle = object_model.get_part("pitch_cradle")
    tool_plate = object_model.get_part("tool_plate")
    roll_axis = object_model.get_articulation("roll_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")

    for cap_elem, boss_elem in (
        ("side_0_shaft_cap", "side_0_pitch_boss"),
        ("side_1_shaft_cap", "side_1_pitch_boss"),
    ):
        ctx.allow_overlap(
            pitch_cradle,
            roll_ring,
            elem_a=cap_elem,
            elem_b=boss_elem,
            reason="The pitch shaft cap is intentionally seated inside the roll-ring trunnion bearing boss.",
        )
        ctx.expect_within(
            pitch_cradle,
            roll_ring,
            axes="xz",
            inner_elem=cap_elem,
            outer_elem=boss_elem,
            margin=0.002,
            name=f"{cap_elem} is concentric in its pitch boss",
        )
        ctx.expect_overlap(
            pitch_cradle,
            roll_ring,
            axes="y",
            elem_a=cap_elem,
            elem_b=boss_elem,
            min_overlap=0.010,
            name=f"{cap_elem} remains inserted in its pitch boss",
        )

    ctx.check(
        "roll axis is front to back",
        tuple(round(v, 3) for v in roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll_axis.axis}",
    )
    ctx.check(
        "pitch axis is left to right",
        tuple(round(v, 3) for v in pitch_axis.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pitch_axis.axis}",
    )
    ctx.expect_within(
        pitch_cradle,
        roll_ring,
        axes="yz",
        margin=0.010,
        name="pitch cradle nests inside roll package",
    )
    ctx.expect_within(
        tool_plate,
        roll_ring,
        axes="yz",
        margin=0.010,
        name="tool plate stays inside ring envelope",
    )
    ctx.expect_gap(
        roll_ring,
        pedestal,
        axis="z",
        min_gap=-0.010,
        name="roll package clears pedestal shoulder",
        positive_elem="machined_ring",
        negative_elem="low_crosshead",
    )
    ctx.expect_contact(
        tool_plate,
        pitch_cradle,
        elem_a="standoff_p_p",
        elem_b="tool_mount",
        contact_tol=0.002,
        name="tool standoffs seat on cradle mount",
    )

    with ctx.pose({roll_axis: math.radians(70.0), pitch_axis: math.radians(30.0)}):
        ctx.expect_within(
            tool_plate,
            roll_ring,
            axes="yz",
            margin=0.055,
            name="tilted plate remains within rotating ring envelope",
        )
        upper_pos = ctx.part_world_position(tool_plate)

    with ctx.pose({roll_axis: math.radians(-70.0), pitch_axis: math.radians(30.0)}):
        opposite_roll_pos = ctx.part_world_position(tool_plate)

    with ctx.pose({roll_axis: 0.0, pitch_axis: math.radians(-30.0)}):
        negative_pitch_pos = ctx.part_world_position(tool_plate)

    ctx.check(
        "roll motion swings the tool plate sideward",
        upper_pos is not None
        and opposite_roll_pos is not None
        and abs(upper_pos[1] - opposite_roll_pos[1]) > 0.030,
        details=f"upper={upper_pos}, opposite={opposite_roll_pos}",
    )
    ctx.check(
        "pitch motion nods the tool plate",
        upper_pos is not None
        and negative_pitch_pos is not None
        and abs(upper_pos[2] - negative_pitch_pos[2]) > 0.020,
        details=f"upper={upper_pos}, negative_pitch={negative_pitch_pos}",
    )

    return ctx.report()


object_model = build_object_model()
