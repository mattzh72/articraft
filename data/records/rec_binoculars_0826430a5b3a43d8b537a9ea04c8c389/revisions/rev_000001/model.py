from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="giant_astronomical_binoculars")

    painted_steel = model.material("painted_steel", rgba=(0.79, 0.81, 0.84, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.25, 0.28, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.13, 0.14, 0.16, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    anodized = model.material("anodized", rgba=(0.57, 0.60, 0.64, 1.0))

    pillar = model.part("pillar")
    pillar.visual(
        Cylinder(radius=0.75, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=gunmetal,
        name="foundation_disk",
    )
    pillar.visual(
        Cylinder(radius=0.22, length=1.34),
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        material=gunmetal,
        name="pillar_shaft",
    )
    pillar.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.53)),
        material=dark_metal,
        name="bearing_base",
    )
    pillar.visual(
        Cylinder(radius=0.40, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.67)),
        material=anodized,
        name="bearing_cap",
    )
    pillar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.40, length=1.72),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
    )

    fork_mount = model.part("fork_mount")
    fork_mount.visual(
        Cylinder(radius=0.44, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_metal,
        name="turntable_drum",
    )
    fork_mount.visual(
        Box((0.84, 0.98, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=gunmetal,
        name="yoke_base",
    )
    fork_mount.visual(
        Box((0.18, 0.82, 0.18)),
        origin=Origin(xyz=(-0.29, 0.0, 0.84)),
        material=gunmetal,
        name="rear_tie_beam",
    )
    fork_mount.visual(
        Box((0.42, 0.55, 0.28)),
        origin=Origin(xyz=(-0.04, 0.0, 0.40)),
        material=dark_metal,
        name="azimuth_drive_case",
    )
    for side_name, side_y in (("left", 0.46), ("right", -0.46)):
        fork_mount.visual(
            Box((0.92, 0.04, 0.78)),
            origin=Origin(xyz=(0.13, side_y, 0.67)),
            material=gunmetal,
            name=f"{side_name}_side_web",
        )
        fork_mount.visual(
            Box((0.90, 0.12, 0.10)),
            origin=Origin(xyz=(0.10, side_y, 0.35), rpy=(0.0, 0.30, 0.0)),
            material=painted_steel,
            name=f"{side_name}_lower_rail",
        )
        fork_mount.visual(
            Box((0.92, 0.12, 0.10)),
            origin=Origin(xyz=(0.16, side_y, 0.95), rpy=(0.0, 0.30, 0.0)),
            material=painted_steel,
            name=f"{side_name}_upper_rail",
        )
        fork_mount.visual(
            Box((0.12, 0.12, 0.68)),
            origin=Origin(xyz=(0.56, side_y, 0.76)),
            material=painted_steel,
            name="left_front_post" if side_name == "left" else "right_front_post",
        )
        fork_mount.visual(
            Box((0.12, 0.12, 0.72)),
            origin=Origin(xyz=(-0.30, side_y, 0.52)),
            material=painted_steel,
            name=f"{side_name}_rear_post",
        )
        fork_mount.visual(
            Cylinder(radius=0.11, length=0.08),
            origin=Origin(xyz=(0.56, 0.50 if side_name == "left" else -0.50, 1.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=anodized,
            name=f"{side_name}_bearing_boss",
        )
    fork_mount.inertial = Inertial.from_geometry(
        Box((1.02, 1.06, 1.24)),
        mass=260.0,
        origin=Origin(xyz=(0.05, 0.0, 0.62)),
    )

    binocular_cradle = model.part("binocular_cradle")
    binocular_cradle.visual(
        Cylinder(radius=0.09, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="trunnion_barrel",
    )
    binocular_cradle.visual(
        Box((0.24, 0.48, 0.20)),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_hub",
    )
    for side_name, side_y in (("left", 0.31), ("right", -0.31)):
        binocular_cradle.visual(
            Box((0.20, 0.18, 0.20)),
            origin=Origin(xyz=(0.0, side_y, 0.0)),
            material=gunmetal,
            name="left_trunnion_cheek" if side_name == "left" else "right_trunnion_cheek",
        )
    binocular_cradle.visual(
        Box((0.60, 0.50, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 0.09)),
        material=gunmetal,
        name="bridge_plate",
    )
    binocular_cradle.visual(
        Box((0.24, 0.46, 0.16)),
        origin=Origin(xyz=(-0.28, 0.0, 0.23)),
        material=gunmetal,
        name="rear_bridge_block",
    )
    binocular_cradle.visual(
        Box((0.16, 0.28, 0.10)),
        origin=Origin(xyz=(0.74, 0.0, 0.28)),
        material=gunmetal,
        name="front_bridge_block",
    )
    for side_name, side_y in (("left", 0.22), ("right", -0.22)):
        binocular_cradle.visual(
            Box((0.26, 0.18, 0.20)),
            origin=Origin(xyz=(0.02, side_y, 0.20)),
            material=gunmetal,
            name=f"{side_name}_tube_saddle",
        )
        binocular_cradle.visual(
            Cylinder(radius=0.16, length=1.38),
            origin=Origin(xyz=(0.28, side_y, 0.28), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_steel,
            name=f"{side_name}_main_tube",
        )
        binocular_cradle.visual(
            Cylinder(radius=0.18, length=0.34),
            origin=Origin(xyz=(1.14, side_y, 0.28), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_steel,
            name=f"{side_name}_objective_shield",
        )
        binocular_cradle.visual(
            Box((0.38, 0.22, 0.28)),
            origin=Origin(xyz=(-0.60, side_y, 0.30)),
            material=gunmetal,
            name=f"{side_name}_prism_housing",
        )
        binocular_cradle.visual(
            Cylinder(radius=0.055, length=0.18),
            origin=Origin(xyz=(-0.69, side_y, 0.50)),
            material=dark_metal,
            name=f"{side_name}_focuser",
        )
        binocular_cradle.visual(
            Cylinder(radius=0.045, length=0.10),
            origin=Origin(xyz=(-0.69, side_y, 0.64)),
            material=black_rubber,
            name=f"{side_name}_eyecup",
        )
    binocular_cradle.visual(
        Cylinder(radius=0.028, length=0.34),
        origin=Origin(xyz=(-0.69, 0.0, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="eyepiece_bridge",
    )
    binocular_cradle.inertial = Inertial.from_geometry(
        Box((2.10, 0.96, 0.92)),
        mass=180.0,
        origin=Origin(xyz=(0.25, 0.0, 0.27)),
    )

    model.articulation(
        "pillar_to_fork_azimuth",
        ArticulationType.CONTINUOUS,
        parent=pillar,
        child=fork_mount,
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=0.8),
    )
    model.articulation(
        "fork_to_binoculars_altitude",
        ArticulationType.REVOLUTE,
        parent=fork_mount,
        child=binocular_cradle,
        origin=Origin(xyz=(0.56, 0.0, 1.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.8,
            lower=-0.20,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    pillar = object_model.get_part("pillar")
    fork_mount = object_model.get_part("fork_mount")
    binocular_cradle = object_model.get_part("binocular_cradle")
    azimuth = object_model.get_articulation("pillar_to_fork_azimuth")
    altitude = object_model.get_articulation("fork_to_binoculars_altitude")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "all prompt parts are present",
        all(part is not None for part in (pillar, fork_mount, binocular_cradle)),
        details=f"parts={[pillar.name, fork_mount.name, binocular_cradle.name]}",
    )
    with ctx.pose({altitude: 0.0}):
        ctx.expect_gap(
            fork_mount,
            pillar,
            axis="z",
            positive_elem="turntable_drum",
            negative_elem="bearing_cap",
            min_gap=0.0,
            max_gap=0.003,
            name="turntable sits on the azimuth bearing",
        )
        ctx.expect_overlap(
            fork_mount,
            pillar,
            axes="xy",
            elem_a="turntable_drum",
            elem_b="bearing_cap",
            min_overlap=0.35,
            name="turntable remains centered over the pillar bearing",
        )
        ctx.expect_contact(
            binocular_cradle,
            fork_mount,
            elem_a="left_trunnion_cheek",
            elem_b="left_front_post",
            name="left cradle cheek seats against the fork upright",
        )
        ctx.expect_contact(
            binocular_cradle,
            fork_mount,
            elem_a="right_trunnion_cheek",
            elem_b="right_front_post",
            name="right cradle cheek seats against the fork upright",
        )

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        rest_objective = ctx.part_element_world_aabb(binocular_cradle, elem="left_objective_shield")
    with ctx.pose({azimuth: 0.0, altitude: 1.10}):
        raised_objective = ctx.part_element_world_aabb(binocular_cradle, elem="left_objective_shield")
    rest_center = aabb_center(rest_objective)
    raised_center = aabb_center(raised_objective)
    ctx.check(
        "altitude joint raises the objective end",
        rest_center is not None
        and raised_center is not None
        and raised_center[2] > rest_center[2] + 0.65
        and raised_center[0] < rest_center[0] - 0.40,
        details=f"rest_center={rest_center}, raised_center={raised_center}",
    )

    with ctx.pose({azimuth: 0.0, altitude: 0.35}):
        forward_objective = ctx.part_element_world_aabb(binocular_cradle, elem="left_objective_shield")
    with ctx.pose({azimuth: math.pi / 2.0, altitude: 0.35}):
        quarter_turn_objective = ctx.part_element_world_aabb(binocular_cradle, elem="left_objective_shield")
    forward_center = aabb_center(forward_objective)
    quarter_turn_center = aabb_center(quarter_turn_objective)
    ctx.check(
        "azimuth bearing swings the binoculars around the pillar",
        forward_center is not None
        and quarter_turn_center is not None
        and abs(quarter_turn_center[2] - forward_center[2]) < 0.02
        and abs(quarter_turn_center[0] - forward_center[0]) > 0.80
        and abs(quarter_turn_center[1] - forward_center[1]) > 0.80,
        details=f"forward_center={forward_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
