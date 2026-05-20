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
)


def _spot_shell_geometry() -> LatheGeometry:
    """Thin-walled cylindrical spotlight can, open at the front rim."""
    outer_profile = [
        (0.038, -0.028),
        (0.044, -0.048),
        (0.050, -0.112),
        (0.052, -0.138),
    ]
    inner_profile = [
        (0.026, -0.036),
        (0.034, -0.052),
        (0.041, -0.112),
        (0.043, -0.132),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )


def _reflector_geometry() -> LatheGeometry:
    """Shallow metallic reflector nested inside the spotlight can."""
    return LatheGeometry(
        [
            (0.012, -0.054),
            (0.038, -0.126),
            (0.043, -0.126),
            (0.017, -0.052),
            (0.012, -0.054),
        ],
        segments=56,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_arm_branching_ceiling_fixture")

    satin_white = model.material("satin_white", rgba=(0.86, 0.84, 0.78, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    warm_lens = model.material("warm_lens", rgba=(1.0, 0.84, 0.46, 0.82))
    reflector = model.material("reflector", rgba=(0.86, 0.83, 0.76, 1.0))

    plate = model.part("ceiling_plate")
    plate.visual(
        Cylinder(radius=0.160, length=0.028),
        origin=Origin(),
        material=satin_white,
        name="canopy_disk",
    )
    plate.visual(
        Cylinder(radius=0.168, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=satin_white,
        name="rolled_rim",
    )
    for index, sx in enumerate((-1.0, 1.0)):
        plate.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(sx * 0.082, 0.0, -0.0155)),
            material=brushed_metal,
            name=f"screw_cap_{index}",
        )

    central = model.part("central_rod")
    central.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_metal,
        name="swivel_collar",
    )
    central.visual(
        Cylinder(radius=0.017, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=brushed_metal,
        name="drop_stem",
    )
    central.visual(
        Cylinder(radius=0.033, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=brushed_metal,
        name="branch_hub",
    )
    central.visual(
        Cylinder(radius=0.013, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, -0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="arm_bar",
    )
    for index, sx in enumerate((-1.0, 1.0)):
        tip_x = sx * 0.335
        central.visual(
            Cylinder(radius=0.023, length=0.045),
            origin=Origin(xyz=(tip_x, 0.0, -0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name=f"tip_collar_{index}",
        )
        central.visual(
            Box((0.034, 0.112, 0.014)),
            origin=Origin(xyz=(tip_x, 0.0, -0.220)),
            material=brushed_metal,
            name=f"yoke_bridge_{index}",
        )
        central.visual(
            Box((0.030, 0.010, 0.070)),
            origin=Origin(xyz=(tip_x, 0.052, -0.252)),
            material=brushed_metal,
            name=f"yoke_cheek_{index}_pos",
        )
        central.visual(
            Box((0.030, 0.010, 0.070)),
            origin=Origin(xyz=(tip_x, -0.052, -0.252)),
            material=brushed_metal,
            name=f"yoke_cheek_{index}_neg",
        )
        central.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(tip_x, 0.058, -0.252), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"outer_bushing_{index}_pos",
        )
        central.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(tip_x, -0.058, -0.252), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"outer_bushing_{index}_neg",
        )

    spot_shell = mesh_from_geometry(_spot_shell_geometry(), "spot_head_shell")
    spot_reflector = mesh_from_geometry(_reflector_geometry(), "spot_head_reflector")
    for index in range(2):
        head = model.part(f"spot_head_{index}")
        head.visual(
            spot_shell,
            origin=Origin(),
            material=dark_metal,
            name="outer_shell",
        )
        head.visual(
            spot_reflector,
            origin=Origin(),
            material=reflector,
            name="inner_reflector",
        )
        head.visual(
            Cylinder(radius=0.046, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.128)),
            material=warm_lens,
            name="glass_lens",
        )
        head.visual(
            Cylinder(radius=0.034, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.031)),
            material=dark_metal,
            name="rear_cap",
        )
        head.visual(
            Box((0.045, 0.072, 0.028)),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=dark_metal,
            name="hinge_saddle",
        )
        head.visual(
            Cylinder(radius=0.008, length=0.094),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="trunnion",
        )

    central_joint = model.articulation(
        "central_rotation",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=central,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    central_joint.meta["description"] = "Swivels the branching rod assembly at the round ceiling plate."

    for index, sx in enumerate((-1.0, 1.0)):
        model.articulation(
            f"spot_tilt_{index}",
            ArticulationType.REVOLUTE,
            parent=central,
            child=f"spot_head_{index}",
            origin=Origin(xyz=(sx * 0.335, 0.0, -0.252)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=1.2,
                lower=-0.85,
                upper=0.85,
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("ceiling_plate")
    central = object_model.get_part("central_rod")
    head_0 = object_model.get_part("spot_head_0")
    head_1 = object_model.get_part("spot_head_1")
    central_rotation = object_model.get_articulation("central_rotation")
    tilt_0 = object_model.get_articulation("spot_tilt_0")
    tilt_1 = object_model.get_articulation("spot_tilt_1")

    ctx.check("round plate and two heads present", all([plate, central, head_0, head_1]))
    ctx.check("three requested revolute joints present", all([central_rotation, tilt_0, tilt_1]))

    ctx.expect_gap(
        plate,
        central,
        axis="z",
        positive_elem="canopy_disk",
        negative_elem="swivel_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel collar seats against ceiling plate",
    )
    ctx.expect_overlap(
        central,
        plate,
        axes="xy",
        elem_a="swivel_collar",
        elem_b="canopy_disk",
        min_overlap=0.090,
        name="central swivel is centered under round plate",
    )

    for index, head in enumerate((head_0, head_1)):
        ctx.expect_gap(
            central,
            head,
            axis="y",
            positive_elem=f"yoke_cheek_{index}_pos",
            negative_elem="trunnion",
            max_gap=0.004,
            max_penetration=0.0,
            name=f"spot head {index} positive trunnion clearance",
        )
        ctx.expect_gap(
            head,
            central,
            axis="y",
            positive_elem="trunnion",
            negative_elem=f"yoke_cheek_{index}_neg",
            max_gap=0.004,
            max_penetration=0.0,
            name=f"spot head {index} negative trunnion clearance",
        )
        ctx.expect_overlap(
            head,
            central,
            axes="xz",
            elem_a="trunnion",
            elem_b=f"yoke_cheek_{index}_pos",
            min_overlap=0.008,
            name=f"spot head {index} trunnion aligns with yoke bore",
        )

    rest_arm_aabb = ctx.part_element_world_aabb(central, elem="arm_bar")
    with ctx.pose({central_rotation: math.pi / 2.0}):
        turned_arm_aabb = ctx.part_element_world_aabb(central, elem="arm_bar")
    if rest_arm_aabb is not None and turned_arm_aabb is not None:
        rest_min, rest_max = rest_arm_aabb
        turn_min, turn_max = turned_arm_aabb
        rest_x = float(rest_max[0] - rest_min[0])
        rest_y = float(rest_max[1] - rest_min[1])
        turn_x = float(turn_max[0] - turn_min[0])
        turn_y = float(turn_max[1] - turn_min[1])
        ctx.check(
            "central rotation swings the two-arm branch",
            rest_x > 0.55 and rest_y < 0.05 and turn_y > 0.55 and turn_x < 0.05,
            details=f"rest=({rest_x:.3f},{rest_y:.3f}), turned=({turn_x:.3f},{turn_y:.3f})",
        )

    shell_rest = ctx.part_element_world_aabb(head_0, elem="outer_shell")
    with ctx.pose({tilt_0: 0.60, tilt_1: -0.60}):
        shell_tilted = ctx.part_element_world_aabb(head_0, elem="outer_shell")
    if shell_rest is not None and shell_tilted is not None:
        rest_min, rest_max = shell_rest
        tilt_min, tilt_max = shell_tilted
        rest_center_z = 0.5 * float(rest_min[2] + rest_max[2])
        tilted_center_z = 0.5 * float(tilt_min[2] + tilt_max[2])
        ctx.check(
            "spot heads tilt upward about their arm-tip hinges",
            tilted_center_z > rest_center_z + 0.006,
            details=f"rest_z={rest_center_z:.3f}, tilted_z={tilted_center_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
