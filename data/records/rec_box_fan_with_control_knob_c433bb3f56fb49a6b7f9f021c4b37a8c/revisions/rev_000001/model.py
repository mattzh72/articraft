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
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_ring_segments(
    part,
    *,
    radius: float,
    y: float,
    wire: float,
    segments: int,
    material: Material,
    name_prefix: str,
) -> None:
    """Approximate a circular wire ring in the XZ plane with overlapping bars."""

    for i in range(segments):
        a0 = 2.0 * math.pi * i / segments
        a1 = 2.0 * math.pi * (i + 1) / segments
        x0, z0 = radius * math.cos(a0), radius * math.sin(a0)
        x1, z1 = radius * math.cos(a1), radius * math.sin(a1)
        mid_x, mid_z = (x0 + x1) * 0.5, (z0 + z1) * 0.5
        dx, dz = x1 - x0, z1 - z0
        length = math.sqrt(dx * dx + dz * dz) + wire * 0.9
        angle = math.atan2(dz, dx)
        part.visual(
            Box((length, wire, wire)),
            origin=Origin(xyz=(mid_x, y, mid_z), rpy=(0.0, -angle, 0.0)),
            material=material,
            name=f"{name_prefix}_{i}",
        )


def _add_radial_bar(
    part,
    *,
    inner_radius: float,
    outer_radius: float,
    angle: float,
    y: float,
    thickness: float,
    material: Material,
    name: str,
) -> None:
    """Add one straight bar in the XZ plane, tangent-free and radial."""

    mid_r = (inner_radius + outer_radius) * 0.5
    length = outer_radius - inner_radius + thickness
    x = mid_r * math.cos(angle)
    z = mid_r * math.sin(angle)
    # A local +X box rotated about local/world Y lies in the XZ plane.
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, -angle, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_exhaust_box_fan")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.62, 1.0))
    dark_metal = model.material("dark_motor_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    wire_chrome = model.material("stainless_wire", rgba=(0.82, 0.84, 0.82, 1.0))
    blade_finish = model.material("dark_blades", rgba=(0.04, 0.045, 0.05, 1.0))
    knob_black = model.material("black_knob", rgba=(0.015, 0.015, 0.014, 1.0))
    label_white = model.material("white_position_marks", rgba=(0.9, 0.9, 0.82, 1.0))

    housing = model.part("housing")

    # Square commercial sheet-metal exhaust box: about 800 mm across with a
    # deep, open center and welded corner overlaps.
    housing.visual(
        Box((0.080, 0.320, 0.800)),
        origin=Origin(xyz=(-0.360, 0.0, 0.0)),
        material=galvanized,
        name="side_rail_0",
    )
    housing.visual(
        Box((0.080, 0.320, 0.800)),
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        material=galvanized,
        name="side_rail_1",
    )
    housing.visual(
        Box((0.800, 0.320, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=galvanized,
        name="top_rail",
    )
    housing.visual(
        Box((0.800, 0.320, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=galvanized,
        name="bottom_rail",
    )

    # A dark round shroud/bellmouth inside the square box stays open through the
    # center and lightly intersects the inner rails where it is welded.
    shroud = LatheGeometry.from_shell_profiles(
        [(0.335, -0.125), (0.332, 0.125)],
        [(0.302, -0.125), (0.302, 0.125)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    housing.visual(
        mesh_from_geometry(shroud, "fan_shroud"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="fan_shroud",
    )

    # Rear motor can, with four radial supports to the shroud/frame.
    housing.visual(
        Cylinder(radius=0.082, length=0.120),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="motor_can",
    )
    housing.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_boss",
    )
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_radial_bar(
            housing,
            inner_radius=0.070,
            outer_radius=0.315,
            angle=angle,
            y=0.105,
            thickness=0.024,
            material=dark_metal,
            name=f"motor_strut_{idx}",
        )

    # Wire guard in front of the impeller.  The concentric rings and radial
    # spokes are modeled as slender overlapping bars so the cage is physically
    # continuous and clipped to the square frame.
    guard_y = -0.192
    for ring_idx, radius in enumerate((0.090, 0.170, 0.250, 0.330)):
        _add_ring_segments(
            housing,
            radius=radius,
            y=guard_y,
            wire=0.006,
            segments=32 if radius > 0.12 else 24,
            material=wire_chrome,
            name_prefix=f"guard_ring_{ring_idx}",
        )
    for idx, angle in enumerate(i * math.pi / 4.0 for i in range(8)):
        _add_radial_bar(
            housing,
            inner_radius=0.028,
            outer_radius=0.338,
            angle=angle,
            y=guard_y,
            thickness=0.006,
            material=wire_chrome,
            name=f"guard_spoke_{idx}",
        )
    housing.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, guard_y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wire_chrome,
        name="guard_center_cap",
    )
    # Four stout clips visibly tie the outer guard ring back into the housing.
    for idx, (x, z, sx, sz) in enumerate(
        (
            (0.0, 0.350, 0.120, 0.060),
            (0.0, -0.350, 0.120, 0.060),
            (-0.350, 0.0, 0.060, 0.120),
            (0.350, 0.0, 0.060, 0.120),
        )
    ):
        housing.visual(
            Box((sx, 0.070, sz)),
            origin=Origin(xyz=(x, -0.177, z)),
            material=wire_chrome,
            name=f"guard_clip_{idx}",
        )

    # Side-mounted control plate and three detent marks for the dial.
    housing.visual(
        Box((0.017, 0.180, 0.180)),
        origin=Origin(xyz=(0.4065, -0.030, 0.205)),
        material=galvanized,
        name="dial_plate",
    )
    for idx, angle in enumerate((-math.pi / 3.0, 0.0, math.pi / 3.0)):
        mark_r = 0.068
        mark_y = -0.030 + mark_r * math.cos(angle)
        mark_z = 0.205 + mark_r * math.sin(angle)
        housing.visual(
            Box((0.004, 0.030, 0.006)),
            origin=Origin(xyz=(0.414, mark_y, mark_z), rpy=(angle, 0.0, 0.0)),
            material=label_white,
            name=f"dial_mark_{idx}",
        )

    impeller = model.part("impeller")
    rotor = FanRotorGeometry(
        0.270,
        0.070,
        6,
        thickness=0.060,
        blade_pitch_deg=32.0,
        blade_sweep_deg=-34.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18, tip_clearance=0.010),
        hub=FanRotorHub(style="capped", rear_collar_height=0.020, rear_collar_radius=0.052, bore_diameter=0.014),
        center=True,
    )
    impeller.visual(
        mesh_from_geometry(rotor, "six_blade_impeller"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_finish,
        name="six_blade_impeller",
    )
    # A short rotating axle stub aligns the impeller with the rear bearing.
    impeller.visual(
        Cylinder(radius=0.014, length=0.130),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_finish,
        name="axle_stub",
    )

    dial = model.part("dial")
    dial_knob = KnobGeometry(
        0.072,
        0.034,
        body_style="skirted",
        top_diameter=0.056,
        skirt=KnobSkirt(0.082, 0.006, flare=0.06, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=18, depth=0.0016),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_knob, "three_position_dial"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="dial_cap",
    )

    model.articulation(
        "axle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=impeller,
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=60.0, lower=0.0, upper=2.0 * math.pi),
    )
    model.articulation(
        "dial_axis",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.415, -0.030, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0, lower=-math.pi / 3.0, upper=math.pi / 3.0),
        meta={"detents": [-math.pi / 3.0, 0.0, math.pi / 3.0], "positions": ["off", "low", "high"]},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    impeller = object_model.get_part("impeller")
    dial = object_model.get_part("dial")
    axle = object_model.get_articulation("axle")
    dial_axis = object_model.get_articulation("dial_axis")

    ctx.allow_overlap(
        housing,
        impeller,
        elem_a="bearing_boss",
        elem_b="axle_stub",
        reason="The rotating axle stub is intentionally captured inside the stationary bearing boss.",
    )

    ctx.expect_within(
        impeller,
        housing,
        axes="xz",
        inner_elem="six_blade_impeller",
        outer_elem="fan_shroud",
        margin=0.040,
        name="impeller fits inside round shroud",
    )
    ctx.expect_gap(
        impeller,
        housing,
        axis="y",
        positive_elem="six_blade_impeller",
        negative_elem="guard_center_cap",
        min_gap=0.070,
        name="wire guard sits in front of impeller",
    )
    ctx.expect_gap(
        housing,
        impeller,
        axis="y",
        positive_elem="bearing_boss",
        negative_elem="axle_stub",
        max_gap=0.002,
        max_penetration=0.006,
        name="axle is seated in bearing boss",
    )
    ctx.expect_overlap(
        housing,
        impeller,
        axes="xz",
        elem_a="bearing_boss",
        elem_b="axle_stub",
        min_overlap=0.020,
        name="axle is centered in bearing boss",
    )
    ctx.expect_gap(
        dial,
        housing,
        axis="x",
        positive_elem="dial_cap",
        negative_elem="dial_plate",
        min_gap=-0.001,
        max_gap=0.004,
        name="dial mounts on side plate",
    )

    rest_impeller = ctx.part_world_position(impeller)
    with ctx.pose({axle: math.pi / 2.0}):
        turned_impeller = ctx.part_world_position(impeller)
        ctx.expect_within(
            impeller,
            housing,
            axes="xz",
            inner_elem="six_blade_impeller",
            outer_elem="fan_shroud",
            margin=0.040,
            name="rotated impeller remains in shroud",
        )
    ctx.check(
        "axle rotates about fixed center",
        rest_impeller is not None
        and turned_impeller is not None
        and abs(rest_impeller[0] - turned_impeller[0]) < 1e-6
        and abs(rest_impeller[1] - turned_impeller[1]) < 1e-6
        and abs(rest_impeller[2] - turned_impeller[2]) < 1e-6,
        details=f"rest={rest_impeller}, turned={turned_impeller}",
    )

    limits = dial_axis.motion_limits
    ctx.check(
        "dial has three detented positions",
        limits is not None
        and abs((limits.upper or 0.0) - math.pi / 3.0) < 1e-6
        and abs((limits.lower or 0.0) + math.pi / 3.0) < 1e-6
        and len(dial_axis.meta.get("detents", ())) == 3,
        details=f"limits={limits}, meta={dial_axis.meta}",
    )
    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_axis: math.pi / 3.0}):
        dial_turned = ctx.part_world_position(dial)
    ctx.check(
        "dial pivots on side normal",
        dial_rest is not None
        and dial_turned is not None
        and abs(dial_rest[0] - dial_turned[0]) < 1e-6
        and abs(dial_rest[1] - dial_turned[1]) < 1e-6
        and abs(dial_rest[2] - dial_turned[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    return ctx.report()


object_model = build_object_model()
