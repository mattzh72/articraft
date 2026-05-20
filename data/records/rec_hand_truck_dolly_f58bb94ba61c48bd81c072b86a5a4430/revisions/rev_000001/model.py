from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


WHEEL_Y = 0.325
MAIN_AXLE_X = 0.080
MAIN_AXLE_Z = 0.200
CASTER_PIVOT_X = -0.305
CASTER_Y = 0.165
CASTER_PIVOT_Z = 0.004
CASTER_WHEEL_Z = -0.080
SWING_HINGE_X = -0.045
SWING_HINGE_Z = 0.079
SWING_LIMIT = 1.10


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hybrid_hand_truck")

    steel = model.material("powder_coated_steel", rgba=(0.05, 0.12, 0.18, 1.0))
    plate_finish = model.material("scuffed_blue_plate", rgba=(0.08, 0.23, 0.36, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    rim_finish = model.material("zinc_plated_rim", rgba=(0.72, 0.74, 0.70, 1.0))
    hardware = model.material("dark_hardware", rgba=(0.16, 0.16, 0.15, 1.0))
    grip = model.material("worn_grip_rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    frame = model.part("frame")

    # Upright tubular back, cross members, and top handle.
    for y in (-0.220, 0.220):
        frame.visual(
            Box((0.038, 0.038, 1.120)),
            origin=Origin(xyz=(0.070, y, 0.755)),
            material=steel,
            name=f"upright_{'neg' if y < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.030, 0.030, 0.500)),
            origin=Origin(xyz=(-0.045, y, 0.340), rpy=(0.0, 0.50, 0.0)),
            material=steel,
            name=f"toe_brace_{'neg' if y < 0 else 'pos'}",
        )
        frame.visual(
            Box((0.070, 0.044, 0.044)),
            origin=Origin(xyz=(0.070, y, 1.325)),
            material=grip,
            name=f"hand_grip_{'neg' if y < 0 else 'pos'}",
        )

    for z, depth, label in (
        (0.245, 0.045, "lower_crossbar"),
        (0.560, 0.034, "middle_crossbar"),
        (0.920, 0.034, "upper_crossbar"),
        (1.285, 0.040, "handle_crossbar"),
    ):
        frame.visual(
            Box((depth, 0.505, 0.036)),
            origin=Origin(xyz=(0.070, 0.0, z)),
            material=steel,
            name=label,
        )

    # The load toe plate is a continuous shelf with a raised heel lip welded to the frame.
    frame.visual(
        Box((0.520, 0.520, 0.034)),
        origin=Origin(xyz=(-0.205, 0.0, 0.110)),
        material=plate_finish,
        name="toe_plate",
    )
    frame.visual(
        Box((0.040, 0.520, 0.185)),
        origin=Origin(xyz=(0.045, 0.0, 0.178)),
        material=plate_finish,
        name="heel_lip",
    )
    frame.visual(
        Box((0.060, 0.390, 0.028)),
        origin=Origin(xyz=(-0.040, 0.0, SWING_HINGE_Z + 0.028)),
        material=steel,
        name="hinge_backer",
    )

    # Main transport axle runs laterally across the back of the truck.
    frame.visual(
        Cylinder(radius=0.0232, length=0.805),
        origin=Origin(xyz=(MAIN_AXLE_X, 0.0, MAIN_AXLE_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="main_axle",
    )
    for y in (-0.235, 0.235):
        frame.visual(
            Box((0.050, 0.028, 0.095)),
            origin=Origin(xyz=(MAIN_AXLE_X, y, MAIN_AXLE_Z + 0.025)),
            material=steel,
            name=f"axle_hanger_{'neg' if y < 0 else 'pos'}",
        )

    # Fixed hinge ears cradle the retractable caster module under the toe plate.
    for y in (-0.230, 0.230):
        frame.visual(
            Box((0.040, 0.024, 0.070)),
            origin=Origin(xyz=(SWING_HINGE_X, y, SWING_HINGE_Z)),
            material=hardware,
            name=f"caster_hinge_ear_{'neg' if y < 0 else 'pos'}",
        )

    # Large pneumatic-style transport wheels.
    for y, suffix in ((-WHEEL_Y, "0"), (WHEEL_Y, "1")):
        wheel = model.part(f"transport_wheel_{suffix}")
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.150,
                    0.070,
                    rim=WheelRim(inner_radius=0.108, flange_height=0.010, flange_thickness=0.004),
                    hub=WheelHub(
                        radius=0.055,
                        width=0.046,
                        cap_style="domed",
                        bolt_pattern=BoltPattern(count=5, circle_diameter=0.070, hole_diameter=0.006),
                    ),
                    face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.020),
                    bore=WheelBore(style="round", diameter=0.046),
                ),
                f"transport_wheel_{suffix}_rim",
            ),
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=rim_finish,
            name="rim",
        )
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.200,
                    0.076,
                    inner_radius=0.146,
                    tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.55),
                    grooves=(TireGroove(center_offset=0.0, width=0.007, depth=0.004),),
                    sidewall=TireSidewall(style="rounded", bulge=0.06),
                    shoulder=TireShoulder(width=0.010, radius=0.004),
                ),
                f"transport_wheel_{suffix}_tire",
            ),
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        model.articulation(
            f"axle_to_transport_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(MAIN_AXLE_X, y, MAIN_AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=18.0),
        )

    swing_arm = model.part("caster_swing_arm")
    swing_arm.visual(
        Cylinder(radius=0.014, length=0.436),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    swing_arm.visual(
        Box((0.285, 0.026, 0.020)),
        origin=Origin(xyz=(-0.1425, -CASTER_Y, 0.000)),
        material=steel,
        name="swing_arm_neg",
    )
    swing_arm.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(CASTER_PIVOT_X, -CASTER_Y, -0.009), rpy=(0.0, 0.0, 0.0)),
        material=hardware,
        name="swivel_boss_neg",
    )
    swing_arm.visual(
        Box((0.285, 0.026, 0.020)),
        origin=Origin(xyz=(-0.1425, CASTER_Y, 0.000)),
        material=steel,
        name="swing_arm_pos",
    )
    swing_arm.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(CASTER_PIVOT_X, CASTER_Y, -0.009), rpy=(0.0, 0.0, 0.0)),
        material=hardware,
        name="swivel_boss_pos",
    )
    swing_arm.visual(
        Box((0.040, 0.365, 0.022)),
        origin=Origin(xyz=(-0.180, 0.0, -0.019)),
        material=steel,
        name="caster_cross_tube",
    )
    model.articulation(
        "frame_to_caster_swing_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing_arm,
        origin=Origin(xyz=(SWING_HINGE_X, 0.0, SWING_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.2, lower=0.0, upper=SWING_LIMIT),
    )

    for y, suffix in ((-CASTER_Y, "0"), (CASTER_Y, "1")):
        fork = model.part(f"caster_fork_{suffix}")
        fork.visual(
            Cylinder(radius=0.010, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=hardware,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.058, 0.062, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=hardware,
            name="fork_bridge",
        )
        fork.visual(
            Box((0.024, 0.008, 0.096)),
            origin=Origin(xyz=(0.0, -0.026, -0.065)),
            material=hardware,
            name="fork_cheek_neg",
        )
        fork.visual(
            Box((0.024, 0.008, 0.096)),
            origin=Origin(xyz=(0.0, 0.026, -0.065)),
            material=hardware,
            name="fork_cheek_pos",
        )
        fork.visual(
            Cylinder(radius=0.0072, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, CASTER_WHEEL_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name="caster_axle",
        )
        model.articulation(
            f"swing_arm_to_caster_fork_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=swing_arm,
            child=fork,
            origin=Origin(xyz=(CASTER_PIVOT_X, y, CASTER_PIVOT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=6.0),
        )

        caster = model.part(f"caster_wheel_{suffix}")
        caster.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.027,
                    0.026,
                    rim=WheelRim(inner_radius=0.019, flange_height=0.003, flange_thickness=0.0015),
                    hub=WheelHub(radius=0.011, width=0.020, cap_style="flat"),
                    face=WheelFace(dish_depth=0.002, front_inset=0.001),
                    spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.004),
                    bore=WheelBore(style="round", diameter=0.014),
                ),
                f"caster_wheel_{suffix}_rim",
            ),
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=rim_finish,
            name="rim",
        )
        caster.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.038,
                    0.032,
                    inner_radius=0.026,
                    tread=TireTread(style="circumferential", depth=0.0025, count=3),
                    sidewall=TireSidewall(style="rounded", bulge=0.04),
                    shoulder=TireShoulder(width=0.0025, radius=0.0015),
                ),
                f"caster_wheel_{suffix}_tire",
            ),
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        model.articulation(
            f"caster_fork_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=caster,
            origin=Origin(xyz=(0.0, 0.0, CASTER_WHEEL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.8, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    swing_arm = object_model.get_part("caster_swing_arm")
    hinge = object_model.get_articulation("frame_to_caster_swing_arm")

    ctx.allow_overlap(
        frame,
        "transport_wheel_0",
        elem_a="main_axle",
        elem_b="rim",
        reason="The transport wheel hub is intentionally captured around the solid axle proxy.",
    )
    ctx.allow_overlap(
        frame,
        "transport_wheel_1",
        elem_a="main_axle",
        elem_b="rim",
        reason="The transport wheel hub is intentionally captured around the solid axle proxy.",
    )
    for suffix in ("0", "1"):
        ctx.allow_overlap(
            f"caster_fork_{suffix}",
            f"caster_wheel_{suffix}",
            elem_a="caster_axle",
            elem_b="rim",
            reason="The caster wheel rim is intentionally represented as spinning around the fork axle proxy.",
        )
    ctx.allow_overlap(
        "caster_fork_0",
        swing_arm,
        elem_a="swivel_stem",
        elem_b="swivel_boss_neg",
        reason="The caster fork swivel stem is intentionally captured inside the swing-arm pivot boss.",
    )
    ctx.allow_overlap(
        "caster_fork_1",
        swing_arm,
        elem_a="swivel_stem",
        elem_b="swivel_boss_pos",
        reason="The caster fork swivel stem is intentionally captured inside the swing-arm pivot boss.",
    )

    ctx.check(
        "primary_mechanisms_present",
        all(object_model.get_articulation(name) is not None for name in (
            "axle_to_transport_wheel_0",
            "axle_to_transport_wheel_1",
            "frame_to_caster_swing_arm",
            "swing_arm_to_caster_fork_0",
            "swing_arm_to_caster_fork_1",
            "caster_fork_to_wheel_0",
            "caster_fork_to_wheel_1",
        )),
        "Expected both large wheels, two caster swivels, two caster wheel axles, and the folding module hinge.",
    )
    ctx.expect_overlap(
        "transport_wheel_0",
        frame,
        axes="y",
        elem_a="rim",
        elem_b="main_axle",
        min_overlap=0.020,
        name="left transport wheel rides on main axle",
    )
    ctx.expect_overlap(
        "transport_wheel_1",
        frame,
        axes="y",
        elem_a="rim",
        elem_b="main_axle",
        min_overlap=0.020,
        name="right transport wheel rides on main axle",
    )
    for suffix in ("0", "1"):
        ctx.expect_within(
            f"caster_wheel_{suffix}",
            f"caster_fork_{suffix}",
            axes="y",
            margin=0.004,
            inner_elem="tire",
            outer_elem="fork_bridge",
            name=f"caster wheel {suffix} is centered inside its fork",
        )
        ctx.expect_overlap(
            f"caster_wheel_{suffix}",
            f"caster_fork_{suffix}",
            axes="y",
            elem_a="rim",
            elem_b="caster_axle",
            min_overlap=0.020,
            name=f"caster wheel {suffix} is retained on its axle",
        )
        ctx.expect_overlap(
            f"caster_wheel_{suffix}",
            f"caster_fork_{suffix}",
            axes="z",
            elem_a="tire",
            elem_b="fork_cheek_pos",
            min_overlap=0.025,
            name=f"caster wheel {suffix} sits between fork cheeks",
        )
    ctx.expect_within(
        "caster_fork_0",
        swing_arm,
        axes="xy",
        inner_elem="swivel_stem",
        outer_elem="swivel_boss_neg",
        margin=0.0,
        name="caster fork 0 swivel stem is centered in its boss",
    )
    ctx.expect_overlap(
        "caster_fork_0",
        swing_arm,
        axes="z",
        elem_a="swivel_stem",
        elem_b="swivel_boss_neg",
        min_overlap=0.015,
        name="caster fork 0 swivel stem remains inserted",
    )
    ctx.expect_within(
        "caster_fork_1",
        swing_arm,
        axes="xy",
        inner_elem="swivel_stem",
        outer_elem="swivel_boss_pos",
        margin=0.0,
        name="caster fork 1 swivel stem is centered in its boss",
    )
    ctx.expect_overlap(
        "caster_fork_1",
        swing_arm,
        axes="z",
        elem_a="swivel_stem",
        elem_b="swivel_boss_pos",
        min_overlap=0.015,
        name="caster fork 1 swivel stem remains inserted",
    )

    rest_pos = ctx.part_world_position(object_model.get_part("caster_fork_0"))
    with ctx.pose({hinge: SWING_LIMIT}):
        folded_pos = ctx.part_world_position(object_model.get_part("caster_fork_0"))
        ctx.expect_gap(
            frame,
            swing_arm,
            axis="z",
            positive_elem="hinge_backer",
            negative_elem="hinge_barrel",
            max_gap=0.003,
            max_penetration=0.0005,
            name="folding caster hinge barrel seats under frame backer",
        )
        ctx.expect_overlap(
            swing_arm,
            frame,
            axes="y",
            elem_a="hinge_barrel",
            elem_b="hinge_backer",
            min_overlap=0.250,
            name="folding caster module is carried by a broad hinge",
        )

    ctx.check(
        "caster module folds downward",
        rest_pos is not None and folded_pos is not None and folded_pos[2] < rest_pos[2] - 0.10,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
