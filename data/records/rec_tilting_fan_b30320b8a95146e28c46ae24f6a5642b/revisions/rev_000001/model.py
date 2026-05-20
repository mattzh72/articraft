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
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


Z_TO_X = (0.0, math.pi / 2.0, 0.0)
Z_TO_NEG_X = (0.0, -math.pi / 2.0, 0.0)
Z_TO_Y = (-math.pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tilting_calibration_fan")

    dark = model.material("matte_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    blue = model.material("anodized_blue", rgba=(0.10, 0.20, 0.34, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black = model.material("black_oxide", rgba=(0.01, 0.012, 0.014, 1.0))
    white = model.material("etched_white", rgba=(0.88, 0.90, 0.86, 1.0))
    amber = model.material("amber_index", rgba=(1.0, 0.62, 0.10, 1.0))

    # Fixed calibration base and yoke.  The child head pivots about the
    # trunnion center at z=0.72 m.
    base = model.part("base")
    base.visual(
        Box((0.68, 0.42, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark,
        name="datum_base",
    )
    base.visual(
        Box((0.56, 0.018, 0.005)),
        origin=Origin(xyz=(0.0, -0.165, 0.047)),
        material=white,
        name="front_datum_line",
    )
    for i, x in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        base.visual(
            Box((0.006, 0.040, 0.0055)),
            origin=Origin(xyz=(x, -0.165, 0.0475)),
            material=white if i != 2 else amber,
            name=f"base_tick_{i}",
        )
    base.visual(
        Box((0.13, 0.11, 0.355)),
        origin=Origin(xyz=(0.0, 0.0, 0.2225)),
        material=blue,
        name="square_column",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.56, 0.11, 0.42),
            span_width=0.45,
            trunnion_diameter=0.052,
            trunnion_center_z=0.32,
            base_thickness=0.055,
            corner_radius=0.006,
            center=False,
        ),
        "trunnion_yoke",
    )
    base.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=blue,
        name="trunnion_yoke",
    )
    for side, x in (("left", -0.281), ("right", 0.281)):
        base.visual(
            mesh_from_geometry(TorusGeometry(0.033, 0.0045), f"{side}_bearing_bezel"),
            origin=Origin(xyz=(x, 0.0, 0.72), rpy=Z_TO_X),
            material=steel,
            name=f"{side}_bearing_bezel",
        )
    base.visual(
        Box((0.004, 0.052, 0.118)),
        origin=Origin(xyz=(-0.226, 0.0, 0.720)),
        material=steel,
        name="left_inner_datum",
    )
    base.visual(
        Box((0.004, 0.052, 0.118)),
        origin=Origin(xyz=(0.226, 0.0, 0.720)),
        material=steel,
        name="right_inner_datum",
    )
    # Fixed angle index marks on the right yoke cheek, arranged around the
    # pivot center so the moving amber pointer has a repeatable datum.
    for i, deg in enumerate((-30, -20, -10, 0, 10, 20, 30)):
        angle = math.radians(deg)
        radius = 0.112
        y = radius * math.sin(angle)
        z = 0.72 + radius * math.cos(angle)
        tick_len = 0.036 if deg in (-30, 0, 30) else 0.024
        base.visual(
            Box((0.006, 0.006, tick_len)),
            origin=Origin(xyz=(0.2790, y, z), rpy=(angle, 0.0, 0.0)),
            material=white if deg != 0 else amber,
            name=f"tilt_tick_{i}",
        )
    for i, (x, y) in enumerate(((-0.27, -0.16), (0.27, -0.16), (-0.27, 0.16), (0.27, 0.16))):
        base.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=black,
            name=f"leveling_foot_{i}",
        )

    # Tilting fan head.  The local origin is the trunnion/tilt axis.  The
    # cage is intentionally a guarded assembly, with front/rear rings,
    # perimeter stand-offs, radial guard bars, and a separate motor body.
    head = model.part("head")
    head.visual(
        mesh_from_geometry(TorusGeometry(0.214, 0.006), "front_guard_outer"),
        origin=Origin(xyz=(0.0, 0.068, 0.0), rpy=Z_TO_Y),
        material=black,
        name="front_guard_outer",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(0.214, 0.006), "rear_guard_outer"),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=Z_TO_Y),
        material=black,
        name="rear_guard_outer",
    )
    for r, tube in ((0.154, 0.0032), (0.092, 0.0030)):
        head.visual(
            mesh_from_geometry(TorusGeometry(r, tube), f"front_guard_ring_{int(r*1000)}"),
            origin=Origin(xyz=(0.0, 0.074, 0.0), rpy=Z_TO_Y),
            material=black,
            name=f"front_guard_ring_{int(r*1000)}",
        )
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        x = 0.214 * math.cos(theta)
        z = 0.214 * math.sin(theta)
        head.visual(
            Cylinder(radius=0.0035, length=0.136),
            origin=Origin(xyz=(x, 0.0, z), rpy=Z_TO_Y),
            material=black,
            name=f"guard_standoff_{i}",
        )
    for i in range(12):
        theta = 2.0 * math.pi * i / 12.0
        mid_r = 0.128
        length = 0.166
        head.visual(
            Box((length, 0.008, 0.008)),
            origin=Origin(
                xyz=(mid_r * math.cos(theta), 0.074, mid_r * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=black,
            name=f"front_spoke_{i}",
        )
    head.visual(
        Cylinder(radius=0.074, length=0.120),
        origin=Origin(xyz=(0.0, -0.088, 0.0), rpy=Z_TO_Y),
        material=blue,
        name="motor_shell",
    )
    for i, theta in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        head.visual(
            Box((0.148, 0.014, 0.014)),
            origin=Origin(
                xyz=(0.145 * math.cos(theta), -0.068, 0.145 * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=blue,
            name=f"rear_motor_spoke_{i}",
        )
    head.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=Z_TO_Y),
        material=steel,
        name="front_bearing",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(0.022, 0.004), "shaft_bearing_ring"),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=Z_TO_Y),
        material=steel,
        name="shaft_bearing_ring",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.094),
        origin=Origin(xyz=(0.253, 0.0, 0.0), rpy=Z_TO_X),
        material=steel,
        name="right_tilt_journal",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(-0.213, 0.0, 0.0), rpy=Z_TO_X),
        material=steel,
        name="left_pivot_collar",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.213, 0.0, 0.0), rpy=Z_TO_X),
        material=steel,
        name="right_pivot_collar",
    )
    head.visual(
        Box((0.014, 0.006, 0.120)),
        origin=Origin(xyz=(0.212, 0.0, 0.060)),
        material=amber,
        name="tilt_pointer",
    )

    # Spinning rotor: separate link with its own continuous spin axis and a
    # visible shaft/hub.  The blade mesh spins about the child Y axis.
    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.174,
                0.038,
                7,
                thickness=0.024,
                blade_pitch_deg=31.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.012, rear_collar_radius=0.030),
                shroud=FanRotorShroud(thickness=0.004, depth=0.012, clearance=0.001, lip_depth=0.0015),
            ),
            "rotor_blades",
        ),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=Z_TO_Y),
        material=steel,
        name="rotor_blades",
    )
    rotor.visual(
        Cylinder(radius=0.012, length=0.058),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=Z_TO_Y),
        material=steel,
        name="rotor_shaft",
    )

    # Locking knob on the left trunnion, separate and articulated as an
    # adjustment feature.  Its screw shank runs through the yoke bore.
    lock_knob = model.part("tilt_knob")
    lock_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.032,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0020),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "tilt_knob_cap",
        ),
        origin=Origin(rpy=Z_TO_NEG_X),
        material=black,
        name="knob_cap",
    )
    lock_knob.visual(
        Cylinder(radius=0.008, length=0.088),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=Z_TO_X),
        material=steel,
        name="clamp_screw",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.7, lower=-0.52, upper=0.52),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=80.0),
    )
    model.articulation(
        "head_to_tilt_knob",
        ArticulationType.REVOLUTE,
        parent=head,
        child=lock_knob,
        origin=Origin(xyz=(-0.310, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("tilt_knob")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")
    knob_spin = object_model.get_articulation("head_to_tilt_knob")

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="front_bearing",
        elem_b="rotor_shaft",
        reason="The rotor shaft is intentionally captured through the modeled front bearing bore.",
    )

    ctx.check(
        "tilt has calibration range",
        tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < -0.50
        and tilt.motion_limits.upper > 0.50,
        details=f"limits={tilt.motion_limits}",
    )
    ctx.check(
        "rotor spin is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_blades",
        outer_elem="front_guard_outer",
        margin=0.010,
        name="rotor stays inside guarded envelope",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_shaft",
        outer_elem="shaft_bearing_ring",
        margin=0.004,
        name="rotor shaft is centered in bearing",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_shaft",
        outer_elem="front_bearing",
        margin=0.002,
        name="rotor shaft is centered in front bearing",
    )
    ctx.expect_overlap(
        rotor,
        head,
        axes="y",
        elem_a="rotor_shaft",
        elem_b="front_bearing",
        min_overlap=0.020,
        name="rotor shaft remains captured in front bearing",
    )
    ctx.expect_overlap(
        rotor,
        head,
        axes="y",
        elem_a="rotor_shaft",
        elem_b="shaft_bearing_ring",
        min_overlap=0.006,
        name="rotor shaft passes through bearing ring",
    )
    ctx.expect_gap(
        base,
        head,
        axis="x",
        positive_elem="right_inner_datum",
        negative_elem="right_pivot_collar",
        min_gap=0.000,
        max_gap=0.008,
        name="right pivot has controlled datum gap",
    )
    ctx.expect_gap(
        head,
        base,
        axis="x",
        positive_elem="left_pivot_collar",
        negative_elem="left_inner_datum",
        min_gap=0.000,
        max_gap=0.008,
        name="left pivot has controlled datum gap",
    )
    ctx.expect_overlap(
        knob,
        base,
        axes="x",
        elem_a="clamp_screw",
        elem_b="trunnion_yoke",
        min_overlap=0.025,
        name="knob screw is retained in yoke bore",
    )
    ctx.expect_contact(
        knob,
        head,
        elem_a="clamp_screw",
        elem_b="left_pivot_collar",
        contact_tol=0.0015,
        name="tilt knob screw seats against pivot collar",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="rotor_blades")
    with ctx.pose({tilt: 0.42, spin: 1.2, knob_spin: 0.8}):
        tilted_aabb = ctx.part_element_world_aabb(rotor, elem="rotor_blades")
        ctx.expect_within(
            rotor,
            head,
            axes="xz",
            inner_elem="rotor_blades",
            outer_elem="front_guard_outer",
            margin=0.010,
            name="rotor remains guarded while tilted and spinning",
        )

    rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 if rest_aabb else None
    tilted_center_z = (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5 if tilted_aabb else None
    ctx.check(
        "positive tilt visibly raises rotor plane",
        rest_center_z is not None and tilted_center_z is not None and tilted_center_z > rest_center_z + 0.006,
        details=f"rest_z={rest_center_z}, tilted_z={tilted_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
