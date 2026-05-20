from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


POLE_HEIGHT = 16.0
POLE_RADIUS = 0.18
ARM_LENGTH = 3.2
ARM_RADIUS = 0.075
LAMP_X_OFFSETS = (-1.25, -0.5, 0.5, 1.25)


def _add_lamp_head(
    model: ArticulatedObject,
    arm,
    *,
    index: int,
    x_offset: float,
    steel_material: str,
    housing_material: str,
    glass_material: str,
):
    lamp = model.part(f"lamp_head_{index}")

    lamp.visual(
        Cylinder(radius=0.02, length=0.45),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_material,
        name=f"lamp_{index}_trunnion",
    )
    lamp.visual(
        Box((0.18, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.04, -0.02)),
        material=steel_material,
        name=f"lamp_{index}_mount_core",
    )
    lamp.visual(
        Box((0.41, 0.22, 0.34)),
        origin=Origin(xyz=(0.0, 0.17, -0.04)),
        material=housing_material,
        name=f"lamp_{index}_body",
    )
    lamp.visual(
        Box((0.39, 0.03, 0.29)),
        origin=Origin(xyz=(0.0, 0.282, -0.04)),
        material=housing_material,
        name=f"lamp_{index}_front_bezel",
    )
    lamp.visual(
        Box((0.35, 0.006, 0.25)),
        origin=Origin(xyz=(0.0, 0.291, -0.04)),
        material=glass_material,
        name=f"lamp_{index}_front_glass",
    )
    lamp.visual(
        Box((0.24, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, 0.065, -0.04)),
        material=housing_material,
        name=f"lamp_{index}_driver_box",
    )
    lamp.visual(
        Box((0.39, 0.035, 0.03)),
        origin=Origin(xyz=(0.0, 0.245, 0.095)),
        material=housing_material,
        name=f"lamp_{index}_visor",
    )

    model.articulation(
        f"cross_arm_to_lamp_{index}",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=lamp,
        origin=Origin(xyz=(x_offset, 0.04, 0.02)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.1,
        ),
    )

    return lamp


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stadium_floodlight_mast")

    galvanized_steel = model.material("galvanized_steel", rgba=(0.65, 0.68, 0.70, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    lamp_housing = model.material("lamp_housing", rgba=(0.18, 0.19, 0.21, 1.0))
    lamp_glass = model.material("lamp_glass", rgba=(0.58, 0.70, 0.76, 0.55))

    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=POLE_RADIUS, length=POLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT / 2.0)),
        material=galvanized_steel,
        name="mast_tube",
    )
    pole.visual(
        Cylinder(radius=0.34, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=painted_steel,
        name="base_flange",
    )
    pole.visual(
        Box((0.12, 0.025, 0.45)),
        origin=Origin(xyz=(0.0, 0.17, 1.4)),
        material=painted_steel,
        name="service_door",
    )

    arm = model.part("cross_arm")
    arm.visual(
        Cylinder(radius=0.14, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=painted_steel,
        name="arm_hub",
    )
    arm.visual(
        Cylinder(radius=ARM_RADIUS, length=ARM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.26), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="arm_tube",
    )

    for index, x_offset in enumerate(LAMP_X_OFFSETS, start=1):
        arm.visual(
            Box((0.56, 0.12, 0.05)),
            origin=Origin(xyz=(x_offset, 0.0, 0.16)),
            material=painted_steel,
            name=f"lamp_{index}_saddle",
        )
        arm.visual(
            Box((0.03, 0.10, 0.19)),
            origin=Origin(xyz=(x_offset - 0.24, 0.04, 0.04)),
            material=painted_steel,
            name=f"lamp_{index}_bracket_left",
        )
        arm.visual(
            Box((0.03, 0.10, 0.19)),
            origin=Origin(xyz=(x_offset + 0.24, 0.04, 0.04)),
            material=painted_steel,
            name=f"lamp_{index}_bracket_right",
        )

    model.articulation(
        "pole_to_cross_arm",
        ArticulationType.FIXED,
        parent=pole,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT)),
    )

    for index, x_offset in enumerate(LAMP_X_OFFSETS, start=1):
        _add_lamp_head(
            model,
            arm,
            index=index,
            x_offset=x_offset,
            steel_material=painted_steel,
            housing_material=lamp_housing,
            glass_material=lamp_glass,
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

    pole = object_model.get_part("pole")
    arm = object_model.get_part("cross_arm")

    ctx.expect_contact(
        arm,
        pole,
        elem_a="arm_hub",
        elem_b="mast_tube",
        name="cross-arm hub seats directly on the mast top",
    )

    for index in range(1, 5):
        lamp = object_model.get_part(f"lamp_head_{index}")
        tilt_joint = object_model.get_articulation(f"cross_arm_to_lamp_{index}")

        ctx.expect_contact(
            lamp,
            arm,
            elem_a=f"lamp_{index}_trunnion",
            elem_b=f"lamp_{index}_bracket_left",
            name=f"lamp {index} trunnion is supported by its left bracket plate",
        )
        ctx.expect_contact(
            lamp,
            arm,
            elem_a=f"lamp_{index}_trunnion",
            elem_b=f"lamp_{index}_bracket_right",
            name=f"lamp {index} trunnion is supported by its right bracket plate",
        )

        rest_aabb = ctx.part_element_world_aabb(lamp, elem=f"lamp_{index}_front_glass")
        with ctx.pose({tilt_joint: 0.8}):
            tilted_aabb = ctx.part_element_world_aabb(lamp, elem=f"lamp_{index}_front_glass")
            ctx.expect_gap(
                arm,
                lamp,
                axis="z",
                positive_elem="arm_tube",
                negative_elem=f"lamp_{index}_body",
                min_gap=0.08,
                name=f"lamp {index} body stays clear of the cross-arm tube when tilted down",
            )

        rest_center_z = None if rest_aabb is None else 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
        tilted_center_z = (
            None if tilted_aabb is None else 0.5 * (tilted_aabb[0][2] + tilted_aabb[1][2])
        )

        ctx.check(
            f"lamp {index} positive tilt pitches the beam downward",
            rest_center_z is not None
            and tilted_center_z is not None
            and tilted_center_z < rest_center_z - 0.08,
            details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
