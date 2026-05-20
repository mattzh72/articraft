from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


CAN_TO_YOKE = "can_to_yoke"
HATCH_JOINT = "can_to_rear_hatch"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_yoke_spotlight")

    dark_cast = model.material("dark_cast_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    black_wrinkle = model.material("black_wrinkle_paint", rgba=(0.025, 0.028, 0.030, 1.0))
    service_blue = model.material("service_blue_enamel", rgba=(0.06, 0.12, 0.18, 1.0))
    worn_steel = model.material("brushed_wear_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = model.material("replaceable_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("ribbed_glass", rgba=(0.65, 0.85, 1.00, 0.38))
    warm_reflector = model.material("warm_reflector", rgba=(0.92, 0.78, 0.48, 1.0))
    warning_yellow = model.material("service_yellow", rgba=(0.95, 0.70, 0.08, 1.0))

    stand = model.part("stand_yoke")
    stand.visual(
        Box((0.86, 0.56, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_cast,
        name="base_plate",
    )
    for index, (x, y) in enumerate(
        ((-0.30, -0.19), (-0.30, 0.19), (0.30, -0.19), (0.30, 0.19))
    ):
        stand.visual(
            Box((0.22, 0.13, 0.020)),
            origin=Origin(xyz=(x, y, 0.000)),
            material=rubber,
            name=f"skid_pad_{index}",
        )
    stand.visual(
        Cylinder(radius=0.090, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=worn_steel,
        name="service_turntable",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark_cast,
        name="welded_mast",
    )
    stand.visual(
        Box((0.34, 0.060, 0.28)),
        origin=Origin(xyz=(0.0, -0.080, 0.270), rpy=(math.radians(18), 0.0, 0.0)),
        material=dark_cast,
        name="front_gusset",
    )
    stand.visual(
        Box((0.34, 0.060, 0.28)),
        origin=Origin(xyz=(0.0, 0.080, 0.270), rpy=(math.radians(-18), 0.0, 0.0)),
        material=dark_cast,
        name="rear_gusset",
    )

    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.72, 0.18, 0.58),
            span_width=0.52,
            trunnion_diameter=0.110,
            trunnion_center_z=0.430,
            base_thickness=0.065,
            corner_radius=0.020,
            center=False,
        ),
        "chunky_trunnion_yoke",
    )
    stand.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=dark_cast,
        name="trunnion_yoke",
    )
    stand.visual(
        Box((0.58, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.090, 0.600)),
        material=worn_steel,
        name="front_service_rail",
    )
    stand.visual(
        Box((0.58, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, 0.090, 0.600)),
        material=worn_steel,
        name="rear_service_rail",
    )

    can = model.part("spotlight_can")
    can_shell = LatheGeometry.from_shell_profiles(
        [
            (0.166, -0.315),
            (0.180, -0.290),
            (0.196, -0.100),
            (0.213, 0.170),
            (0.226, 0.305),
            (0.218, 0.330),
        ],
        [
            (0.134, -0.300),
            (0.156, -0.275),
            (0.172, -0.090),
            (0.188, 0.170),
            (0.194, 0.292),
            (0.180, 0.314),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    can.visual(
        mesh_from_geometry(can_shell, "tapered_spotlight_can"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=service_blue,
        name="can_shell",
    )
    reflector = LatheGeometry.from_shell_profiles(
        [
            (0.060, -0.090),
            (0.110, 0.035),
            (0.172, 0.235),
            (0.196, 0.292),
        ],
        [
            (0.048, -0.082),
            (0.092, 0.040),
            (0.154, 0.225),
            (0.172, 0.282),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    can.visual(
        mesh_from_geometry(reflector, "deep_reflector_bowl"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_reflector,
        name="reflector_bowl",
    )
    can.visual(
        Cylinder(radius=0.202, length=0.012),
        origin=Origin(xyz=(0.0, 0.314, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    can.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(0.223, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="side_boss_0",
    )
    can.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(-0.223, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="side_boss_1",
    )
    can.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, -0.140, 0.238)),
        material=worn_steel,
        name="handle_post_0",
    )
    can.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, 0.120, 0.246)),
        material=worn_steel,
        name="handle_post_1",
    )
    can.visual(
        Cylinder(radius=0.023, length=0.280),
        origin=Origin(xyz=(0.0, -0.010, 0.290), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="top_service_handle",
    )
    can.visual(
        Box((0.240, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, -0.302, 0.155)),
        material=worn_steel,
        name="rear_hinge_bridge",
    )
    can.visual(
        Box((0.075, 0.018, 0.035)),
        origin=Origin(xyz=(-0.105, -0.323, 0.178)),
        material=worn_steel,
        name="rear_hinge_leaf_0",
    )
    can.visual(
        Box((0.075, 0.018, 0.035)),
        origin=Origin(xyz=(0.105, -0.323, 0.178)),
        material=worn_steel,
        name="rear_hinge_leaf_1",
    )
    can.visual(
        Cylinder(radius=0.013, length=0.075),
        origin=Origin(xyz=(-0.105, -0.335, 0.183), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="rear_hinge_knuckle_0",
    )
    can.visual(
        Cylinder(radius=0.013, length=0.075),
        origin=Origin(xyz=(0.105, -0.335, 0.183), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="rear_hinge_knuckle_1",
    )
    can.visual(
        Cylinder(radius=0.008, length=0.245),
        origin=Origin(xyz=(0.0, -0.335, 0.183), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="rear_hinge_pin",
    )

    rear_hatch = model.part("rear_hatch")
    rear_hatch.visual(
        Box((0.250, 0.020, 0.190)),
        origin=Origin(xyz=(0.0, -0.025, -0.115)),
        material=black_wrinkle,
        name="hatch_panel",
    )
    rear_hatch.visual(
        Box((0.100, 0.016, 0.032)),
        origin=Origin(xyz=(0.0, -0.019, -0.022)),
        material=worn_steel,
        name="hatch_hinge_leaf",
    )
    rear_hatch.visual(
        Cylinder(radius=0.013, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_steel,
        name="hatch_knuckle",
    )
    rear_hatch.visual(
        Box((0.070, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.036, -0.205)),
        material=warning_yellow,
        name="quarter_turn_latch",
    )
    rear_hatch.visual(
        Box((0.215, 0.008, 0.155)),
        origin=Origin(xyz=(0.0, -0.039, -0.115)),
        material=rubber,
        name="service_gasket",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.118,
            0.052,
            body_style="lobed",
            base_diameter=0.074,
            top_diameter=0.105,
            crown_radius=0.003,
            edge_radius=0.0015,
            center=False,
        ),
        "lobed_lock_knob",
    )
    for index, sign in enumerate((1.0, -1.0)):
        lock = model.part(f"tilt_lock_{index}")
        lock.visual(
            Cylinder(radius=0.021, length=0.118),
            origin=Origin(xyz=(sign * 0.059, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name="threaded_stem",
        )
        lock.visual(
            Cylinder(radius=0.075, length=0.012),
            origin=Origin(xyz=(sign * 0.117, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name="friction_washer",
        )
        lock.visual(
            knob_mesh,
            origin=Origin(
                xyz=(sign * 0.123, 0.0, 0.0),
                rpy=(0.0, sign * math.pi / 2.0, 0.0),
            ),
            material=black_wrinkle,
            name="lobed_knob",
        )

        model.articulation(
            f"can_to_tilt_lock_{index}",
            ArticulationType.CONTINUOUS,
            parent=can,
            child=lock,
            origin=Origin(xyz=(sign * 0.248, 0.0, 0.0)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=4.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.6),
        )

    model.articulation(
        CAN_TO_YOKE,
        ArticulationType.REVOLUTE,
        parent=stand,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=math.radians(-42.0),
            upper=math.radians(58.0),
        ),
        motion_properties=MotionProperties(damping=0.8, friction=4.0),
    )

    model.articulation(
        HATCH_JOINT,
        ArticulationType.REVOLUTE,
        parent=can,
        child=rear_hatch,
        origin=Origin(xyz=(0.0, -0.335, 0.183)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand_yoke")
    can = object_model.get_part("spotlight_can")
    hatch = object_model.get_part("rear_hatch")
    tilt = object_model.get_articulation(CAN_TO_YOKE)
    hatch_joint = object_model.get_articulation(HATCH_JOINT)

    ctx.allow_overlap(
        "tilt_lock_0",
        stand,
        elem_a="friction_washer",
        elem_b="trunnion_yoke",
        reason="The lock washer is intentionally modeled with slight compression against the yoke wear face.",
    )
    ctx.allow_overlap(
        "tilt_lock_1",
        stand,
        elem_a="friction_washer",
        elem_b="trunnion_yoke",
        reason="The opposite lock washer is intentionally modeled with slight compression against the yoke wear face.",
    )
    ctx.allow_overlap(
        can,
        hatch,
        elem_a="rear_hinge_pin",
        elem_b="hatch_knuckle",
        reason="The service hatch knuckle is captured on the fixed hinge pin.",
    )

    ctx.expect_gap(
        "tilt_lock_0",
        stand,
        axis="x",
        positive_elem="friction_washer",
        negative_elem="trunnion_yoke",
        max_penetration=0.002,
        name="right lock washer has controlled clamp compression",
    )
    ctx.expect_gap(
        stand,
        "tilt_lock_1",
        axis="x",
        positive_elem="trunnion_yoke",
        negative_elem="friction_washer",
        max_penetration=0.002,
        name="left lock washer has controlled clamp compression",
    )
    ctx.expect_overlap(
        can,
        hatch,
        axes="x",
        elem_a="rear_hinge_pin",
        elem_b="hatch_knuckle",
        min_overlap=0.070,
        name="rear hinge pin captures hatch knuckle",
    )
    ctx.expect_overlap(
        can,
        stand,
        axes="xz",
        elem_a="side_boss_0",
        elem_b="trunnion_yoke",
        min_overlap=0.030,
        name="side boss aligns through yoke cheek",
    )
    ctx.expect_overlap(
        "tilt_lock_0",
        stand,
        axes="x",
        elem_a="threaded_stem",
        elem_b="trunnion_yoke",
        min_overlap=0.070,
        name="lock stem traverses yoke cheek",
    )
    ctx.expect_overlap(
        hatch,
        can,
        axes="xz",
        elem_a="hatch_panel",
        elem_b="can_shell",
        min_overlap=0.090,
        name="rear access hatch covers service opening",
    )

    closed_front = ctx.part_element_world_aabb(can, elem="front_lens")
    with ctx.pose({tilt: math.radians(45.0)}):
        raised_front = ctx.part_element_world_aabb(can, elem="front_lens")
    ctx.check(
        "tilt joint raises the beam through yoke pivots",
        closed_front is not None
        and raised_front is not None
        and (raised_front[0][2] + raised_front[1][2]) * 0.5
        > (closed_front[0][2] + closed_front[1][2]) * 0.5 + 0.15,
        details=f"closed_front={closed_front}, raised_front={raised_front}",
    )

    closed_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: math.radians(85.0)}):
        open_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "maintenance hatch swings rearward for access",
        closed_hatch is not None
        and open_hatch is not None
        and (open_hatch[0][1] + open_hatch[1][1]) * 0.5
        < (closed_hatch[0][1] + closed_hatch[1][1]) * 0.5 - 0.08,
        details=f"closed_hatch={closed_hatch}, open_hatch={open_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
