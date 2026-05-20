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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _build_barrel_shell():
    outer_profile = [
        (0.14, -0.58),
        (0.22, -0.48),
        (0.28, -0.24),
        (0.31, 0.08),
        (0.29, 0.52),
        (0.26, 1.18),
        (0.22, 2.20),
        (0.19, 3.10),
        (0.17, 3.72),
        (0.18, 3.94),
        (0.16, 4.08),
    ]
    inner_profile = [
        (0.00, -0.48),
        (0.04, -0.36),
        (0.08, -0.16),
        (0.10, 0.32),
        (0.112, 1.20),
        (0.118, 2.48),
        (0.123, 4.08),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
        "smoothbore_barrel_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coastal_defense_embrasure_cannon")

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.69, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.40, 0.31, 0.22, 1.0))
    carriage_iron = model.material("carriage_iron", rgba=(0.28, 0.30, 0.32, 1.0))
    gun_iron = model.material("gun_iron", rgba=(0.16, 0.17, 0.18, 1.0))

    barbette = model.part("barbette")
    barbette.visual(
        Cylinder(radius=2.10, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=concrete,
        name="concrete_drum",
    )
    barbette.visual(
        Cylinder(radius=2.28, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=concrete,
        name="bearing_plinth",
    )
    barbette.inertial = Inertial.from_geometry(
        Box((4.56, 4.56, 1.06)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )

    carriage_platform = model.part("carriage_platform")
    carriage_platform.visual(
        Cylinder(radius=1.32, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=carriage_iron,
        name="rotating_skirt",
    )
    carriage_platform.visual(
        Box((3.10, 2.40, 0.16)),
        origin=Origin(xyz=(0.15, 0.0, 0.20)),
        material=weathered_wood,
        name="platform_deck",
    )
    carriage_platform.visual(
        Box((2.25, 0.28, 0.12)),
        origin=Origin(xyz=(0.30, 0.46, 0.34)),
        material=carriage_iron,
        name="left_slide_beam",
    )
    carriage_platform.visual(
        Box((2.25, 0.28, 0.12)),
        origin=Origin(xyz=(0.30, -0.46, 0.34)),
        material=carriage_iron,
        name="right_slide_beam",
    )
    carriage_platform.visual(
        Box((1.70, 0.18, 0.92)),
        origin=Origin(xyz=(0.20, 0.49, 0.74)),
        material=carriage_iron,
        name="left_cheek",
    )
    carriage_platform.visual(
        Box((1.70, 0.18, 0.92)),
        origin=Origin(xyz=(0.20, -0.49, 0.74)),
        material=carriage_iron,
        name="right_cheek",
    )
    carriage_platform.visual(
        Box((0.30, 1.02, 0.30)),
        origin=Origin(xyz=(-0.55, 0.0, 0.43)),
        material=carriage_iron,
        name="rear_transom",
    )
    carriage_platform.visual(
        Box((0.24, 0.84, 0.20)),
        origin=Origin(xyz=(0.90, 0.0, 0.50)),
        material=carriage_iron,
        name="front_transom",
    )
    carriage_platform.visual(
        Box((0.28, 0.10, 0.20)),
        origin=Origin(xyz=(0.0, 0.45, 0.92)),
        material=carriage_iron,
        name="left_bearing_jaw",
    )
    carriage_platform.visual(
        Box((0.28, 0.10, 0.20)),
        origin=Origin(xyz=(0.0, -0.45, 0.92)),
        material=carriage_iron,
        name="right_bearing_jaw",
    )
    carriage_platform.visual(
        Box((0.74, 0.56, 0.14)),
        origin=Origin(xyz=(-0.92, 0.0, 0.35)),
        material=carriage_iron,
        name="elevation_bed",
    )
    carriage_platform.visual(
        Cylinder(radius=0.10, length=0.34),
        origin=Origin(xyz=(-0.78, 0.0, 0.59)),
        material=carriage_iron,
        name="elevation_pedestal",
    )
    carriage_platform.inertial = Inertial.from_geometry(
        Box((3.10, 2.40, 1.02)),
        mass=6200.0,
        origin=Origin(xyz=(0.15, 0.0, 0.51)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _build_barrel_shell(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gun_iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.08, length=0.24),
        origin=Origin(xyz=(-0.62, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gun_iron,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.12),
        origin=Origin(xyz=(-0.84, 0.0, 0.0)),
        material=gun_iron,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.33, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gun_iron,
        name="trunnion_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(4.00, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gun_iron,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.10, length=0.13),
        origin=Origin(xyz=(0.0, 0.335, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gun_iron,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.10, length=0.13),
        origin=Origin(xyz=(0.0, -0.335, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gun_iron,
        name="right_trunnion",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=4.60),
        mass=4300.0,
        origin=Origin(xyz=(1.95, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "traverse_bearing",
        ArticulationType.CONTINUOUS,
        parent=barbette,
        child=carriage_platform,
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45000.0, velocity=0.35),
    )
    model.articulation(
        "elevating_trunnion",
        ArticulationType.REVOLUTE,
        parent=carriage_platform,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.45,
            lower=math.radians(-7.0),
            upper=math.radians(33.0),
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

    barbette = object_model.get_part("barbette")
    carriage_platform = object_model.get_part("carriage_platform")
    barrel = object_model.get_part("barrel")
    traverse = object_model.get_articulation("traverse_bearing")
    elevate = object_model.get_articulation("elevating_trunnion")

    with ctx.pose({traverse: 0.0, elevate: 0.0}):
        ctx.expect_gap(
            carriage_platform,
            barbette,
            axis="z",
            positive_elem="rotating_skirt",
            negative_elem="bearing_plinth",
            min_gap=0.0,
            max_gap=0.0,
            name="turntable skirt seats on the concrete plinth",
        )
        ctx.expect_overlap(
            carriage_platform,
            barbette,
            axes="xy",
            elem_a="rotating_skirt",
            elem_b="bearing_plinth",
            min_overlap=2.50,
            name="turntable skirt remains fully over the barbette bearing",
        )
        ctx.expect_contact(
            barrel,
            carriage_platform,
            elem_a="left_trunnion",
            elem_b="left_bearing_jaw",
            name="left trunnion bears against the carriage jaw",
        )
        ctx.expect_contact(
            barrel,
            carriage_platform,
            elem_a="right_trunnion",
            elem_b="right_bearing_jaw",
            name="right trunnion bears against the carriage jaw",
        )
        ctx.expect_gap(
            barrel,
            carriage_platform,
            axis="z",
            positive_elem="barrel_shell",
            negative_elem="platform_deck",
            min_gap=0.30,
            name="barrel clears the gun platform deck at rest",
        )

        rest_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_band")

    with ctx.pose({elevate: math.radians(25.0)}):
        elevated_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_band")

    rest_muzzle_center = None
    elevated_muzzle_center = None
    if rest_muzzle_aabb is not None:
        rest_muzzle_center = tuple(
            0.5 * (rest_muzzle_aabb[0][axis] + rest_muzzle_aabb[1][axis]) for axis in range(3)
        )
    if elevated_muzzle_aabb is not None:
        elevated_muzzle_center = tuple(
            0.5 * (elevated_muzzle_aabb[0][axis] + elevated_muzzle_aabb[1][axis])
            for axis in range(3)
        )
    ctx.check(
        "positive elevation lifts the muzzle",
        rest_muzzle_center is not None
        and elevated_muzzle_center is not None
        and elevated_muzzle_center[2] > rest_muzzle_center[2] + 1.0,
        details=f"rest={rest_muzzle_center}, elevated={elevated_muzzle_center}",
    )

    with ctx.pose({traverse: math.radians(35.0), elevate: 0.0}):
        traversed_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_band")

    traversed_muzzle_center = None
    if traversed_muzzle_aabb is not None:
        traversed_muzzle_center = tuple(
            0.5 * (traversed_muzzle_aabb[0][axis] + traversed_muzzle_aabb[1][axis])
            for axis in range(3)
        )
    ctx.check(
        "traverse swings the muzzle laterally around the barbette",
        rest_muzzle_center is not None
        and traversed_muzzle_center is not None
        and abs(traversed_muzzle_center[1]) > 1.9
        and abs(math.hypot(traversed_muzzle_center[0], traversed_muzzle_center[1]) - math.hypot(rest_muzzle_center[0], rest_muzzle_center[1])) < 0.2,
        details=f"rest={rest_muzzle_center}, traversed={traversed_muzzle_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
