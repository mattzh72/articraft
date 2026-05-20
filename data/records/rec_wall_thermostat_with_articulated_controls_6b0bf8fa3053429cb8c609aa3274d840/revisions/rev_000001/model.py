from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wall_thermostat")

    dark_polymer = model.material("dark_polymer", rgba=(0.055, 0.060, 0.060, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.05, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.01, 0.012, 0.012, 1.0))
    white_marking = model.material("white_marking", rgba=(0.93, 0.94, 0.88, 1.0))
    warning_red = model.material("warning_red", rgba=(0.78, 0.08, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.190, 0.255, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="wall_plate",
    )
    body.visual(
        Box((0.160, 0.215, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_polymer,
        name="sealed_housing",
    )
    body.visual(
        Box((0.176, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.112, 0.020)),
        material=brushed_steel,
        name="top_mount_flange",
    )
    body.visual(
        Box((0.176, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, -0.112, 0.020)),
        material=brushed_steel,
        name="bottom_mount_flange",
    )

    # A real annular retainer, not a solid disk, leaves a visible running
    # clearance around the dial shaft.
    retainer_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.032, 0.032),
            (0.058, 0.058),
            0.006,
            opening_shape="circle",
            outer_shape="circle",
            center=False,
        ),
        "shaft_retainer",
    )
    body.visual(
        retainer_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=brushed_steel,
        name="shaft_retainer",
    )

    # Guard cage: four standoff blocks carry load back into the face plate,
    # while raised rails keep hands/tools off the dial rim.
    body.visual(
        Box((0.014, 0.160, 0.016)),
        origin=Origin(xyz=(-0.065, 0.0, 0.062)),
        material=safety_yellow,
        name="guard_rail_0",
    )
    body.visual(
        Box((0.014, 0.160, 0.016)),
        origin=Origin(xyz=(0.065, 0.0, 0.062)),
        material=safety_yellow,
        name="guard_rail_1",
    )
    for i, y in enumerate((-0.082, 0.082)):
        body.visual(
            Box((0.144, 0.014, 0.016)),
            origin=Origin(xyz=(0.0, y, 0.062)),
            material=safety_yellow,
            name=f"guard_crossbar_{i}",
        )
    standoff_index = 0
    for x in (-0.065, 0.065):
        for y in (-0.082, 0.082):
            body.visual(
                Box((0.030, 0.030, 0.018)),
                origin=Origin(xyz=(x, y, 0.049)),
                material=brushed_steel,
                name=f"guard_standoff_{standoff_index}",
            )
            standoff_index += 1

    # Diagonal braces make the load path from the guard cage into the housing
    # legible instead of decorative.
    for i, (x, y, yaw) in enumerate(
        (
            (-0.050, 0.064, -0.60),
            (0.050, 0.064, 0.60),
            (-0.050, -0.064, 0.60),
            (0.050, -0.064, -0.60),
        )
    ):
        body.visual(
            Box((0.048, 0.007, 0.008)),
            origin=Origin(xyz=(x, y, 0.050), rpy=(0.0, 0.0, yaw)),
            material=brushed_steel,
            name=f"guard_brace_{i}",
        )

    # Mechanical over-travel stops bracket the rotating dial lug at the motion
    # limits.  They are mounted directly on the reinforced front face.
    for i, x in enumerate((-0.062, 0.062)):
        body.visual(
            Box((0.014, 0.030, 0.022)),
            origin=Origin(xyz=(x, -0.004, 0.051)),
            material=warning_red,
            name=f"overtravel_stop_{i}",
        )

    # Visible fastening logic: each screw head is seated into the fixed plate
    # or guard standoff, with a dark slot on top.
    screw_points = [
        (-0.074, 0.106),
        (0.074, 0.106),
        (-0.074, -0.106),
        (0.074, -0.106),
        (-0.065, 0.082),
        (0.065, 0.082),
        (-0.065, -0.082),
        (0.065, -0.082),
    ]
    for i, (x, y) in enumerate(screw_points):
        body.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(x, y, 0.0415)),
            material=brushed_steel,
            name=f"screw_head_{i}",
        )
        body.visual(
            Box((0.010, 0.0022, 0.0016)),
            origin=Origin(xyz=(x, y, 0.0444), rpy=(0.0, 0.0, 0.785 if i % 2 else 0.0)),
            material=black_oxide,
            name=f"screw_slot_{i}",
        )

    # Scale ticks are raised paint/engraving sitting on the housing face, not
    # separate floating fragments.
    for i, (x, y, yaw) in enumerate(
        (
            (-0.038, 0.044, -0.72),
            (-0.020, 0.055, -0.35),
            (0.000, 0.059, 0.0),
            (0.020, 0.055, 0.35),
            (0.038, 0.044, 0.72),
        )
    ):
        body.visual(
            Box((0.004, 0.018, 0.0018)),
            origin=Origin(xyz=(x, y, 0.0408), rpy=(0.0, 0.0, yaw)),
            material=white_marking,
            name=f"scale_tick_{i}",
        )

    # Lockout post and padlock hasp for industrial safety cycling.
    body.visual(
        Box((0.052, 0.050, 0.014)),
        origin=Origin(xyz=(0.086, -0.108, 0.035)),
        material=brushed_steel,
        name="lockout_mount",
    )
    body.visual(
        Cylinder(radius=0.0055, length=0.021),
        origin=Origin(xyz=(0.086, -0.108, 0.0505)),
        material=brushed_steel,
        name="lockout_pin",
    )
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.010, 0.010),
                (0.030, 0.030),
                0.006,
                opening_shape="circle",
                outer_shape="rounded_rect",
                outer_corner_radius=0.004,
                center=False,
            ),
            "lockout_hasp",
        ),
        origin=Origin(xyz=(0.108, -0.108, 0.040)),
        material=brushed_steel,
        name="lockout_hasp",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_steel,
        name="shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.074,
                0.028,
                body_style="faceted",
                base_diameter=0.078,
                top_diameter=0.064,
                edge_radius=0.001,
                grip=KnobGrip(style="knurled", count=36, depth=0.0010, helix_angle_deg=18.0),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                bore=KnobBore(style="round", diameter=0.013),
                center=False,
            ),
            "dial_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=black_oxide,
        name="dial_cap",
    )
    dial.visual(
        Box((0.006, 0.036, 0.0022)),
        origin=Origin(xyz=(0.0, 0.014, 0.043)),
        material=white_marking,
        name="pointer_bar",
    )
    dial.visual(
        Box((0.016, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.044, 0.021)),
        material=warning_red,
        name="stop_lug",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.2, lower=-1.65, upper=1.65),
    )

    lockout_pawl = model.part("lockout_pawl")
    lockout_pawl.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.014, 0.014),
                (0.028, 0.028),
                0.006,
                opening_shape="circle",
                outer_shape="circle",
                center=False,
            ),
            "pawl_hub",
        ),
        origin=Origin(),
        material=brushed_steel,
        name="pawl_hub",
    )
    lockout_pawl.visual(
        Box((0.010, 0.052, 0.006)),
        origin=Origin(xyz=(-0.026, -0.030, 0.003), rpy=(0.0, 0.0, -0.72)),
        material=safety_yellow,
        name="pawl_arm",
    )
    lockout_pawl.visual(
        Box((0.016, 0.012, 0.008)),
        origin=Origin(xyz=(-0.046, -0.053, 0.004), rpy=(0.0, 0.0, -0.72)),
        material=warning_red,
        name="pawl_tooth",
    )

    model.articulation(
        "body_to_lockout_pawl",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lockout_pawl,
        origin=Origin(xyz=(0.086, -0.108, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.35, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    lockout_pawl = object_model.get_part("lockout_pawl")
    dial_joint = object_model.get_articulation("body_to_dial")
    lockout_joint = object_model.get_articulation("body_to_lockout_pawl")

    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=0.001,
        name="dial rotates on the thermostat center axis",
    )
    ctx.expect_within(
        dial,
        body,
        axes="xy",
        inner_elem="shaft",
        outer_elem="shaft_retainer",
        margin=0.0,
        name="shaft remains centered inside retainer envelope",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="z",
        elem_a="shaft",
        elem_b="shaft_retainer",
        min_overlap=0.005,
        name="shaft visibly passes through retainer depth",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="shaft_retainer",
        min_gap=0.006,
        max_gap=0.010,
        name="dial cap stands proud of fixed retainer",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="x",
        positive_elem="guard_rail_1",
        negative_elem="dial_cap",
        min_gap=0.012,
        name="guard rail clears dial on positive side",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="x",
        positive_elem="dial_cap",
        negative_elem="guard_rail_0",
        min_gap=0.012,
        name="guard rail clears dial on negative side",
    )
    ctx.expect_overlap(
        lockout_pawl,
        body,
        axes="z",
        elem_a="pawl_hub",
        elem_b="lockout_pin",
        min_overlap=0.004,
        name="lockout pawl is retained on its pin",
    )

    rest_lug = ctx.part_element_world_aabb(dial, elem="stop_lug")
    with ctx.pose({dial_joint: 1.65}):
        limit_lug = ctx.part_element_world_aabb(dial, elem="stop_lug")
    ctx.check(
        "dial stop lug sweeps toward an over-travel stop",
        rest_lug is not None
        and limit_lug is not None
        and limit_lug[0][0] < rest_lug[0][0] - 0.030,
        details=f"rest={rest_lug}, limit={limit_lug}",
    )

    rest_tooth = ctx.part_element_world_aabb(lockout_pawl, elem="pawl_tooth")
    with ctx.pose({lockout_joint: -0.35}):
        armed_tooth = ctx.part_element_world_aabb(lockout_pawl, elem="pawl_tooth")
    ctx.check(
        "lockout pawl has a real pivoting control motion",
        rest_tooth is not None
        and armed_tooth is not None
        and armed_tooth[0][0] < rest_tooth[0][0] - 0.006,
        details=f"rest={rest_tooth}, armed={armed_tooth}",
    )

    return ctx.report()


object_model = build_object_model()
