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

WING_POS_GLASS = "wing_pos_glass"
WING_NEG_GLASS = "wing_neg_glass"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_wing_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    brake_black = model.material("brake_black", rgba=(0.11, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.67, 0.83, 0.92, 0.30))
    gasket = model.material("gasket", rgba=(0.15, 0.16, 0.17, 1.0))

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=1.12, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 2.34)),
        material=aluminum,
        name="drum_shell",
    )
    drum.visual(
        Box((0.78, 0.06, 2.22)),
        origin=Origin(xyz=(0.0, 1.03, 1.11)),
        material=aluminum,
        name="side_frame_pos_y",
    )
    drum.visual(
        Box((0.78, 0.06, 2.22)),
        origin=Origin(xyz=(0.0, -1.03, 1.11)),
        material=aluminum,
        name="side_frame_neg_y",
    )
    drum.visual(
        Box((0.69, 0.014, 2.06)),
        origin=Origin(xyz=(0.0, 1.03, 1.11)),
        material=glass,
        name="side_glass_pos_y",
    )
    drum.visual(
        Box((0.69, 0.014, 2.06)),
        origin=Origin(xyz=(0.0, -1.03, 1.11)),
        material=glass,
        name="side_glass_neg_y",
    )
    drum.visual(
        Box((0.18, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, 0.30, 2.14)),
        material=brake_black,
        name="brake_housing_pos_y",
    )
    drum.visual(
        Box((0.18, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -0.30, 2.14)),
        material=brake_black,
        name="brake_housing_neg_y",
    )
    drum.visual(
        Box((0.10, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.30, 2.06)),
        material=gasket,
        name="brake_pad_pos_y",
    )
    drum.visual(
        Box((0.10, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, -0.30, 2.06)),
        material=gasket,
        name="brake_pad_neg_y",
    )
    drum.inertial = Inertial.from_geometry(
        Box((2.24, 2.12, 2.46)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
    )

    wing_assembly = model.part("wing_assembly")
    wing_assembly.visual(
        Cylinder(radius=0.22, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=dark_metal,
        name="brake_drum_rotor",
    )
    wing_assembly.visual(
        Cylinder(radius=0.055, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 1.95)),
        material=dark_metal,
        name="central_post",
    )
    wing_assembly.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="floor_pivot",
    )

    wing_outer_center = 0.90
    wing_inner_center = 0.14
    rail_center = 0.52
    rail_length = 0.81
    arm_length = 0.18

    for side_name, sign, glass_name in (
        ("pos", 1.0, WING_POS_GLASS),
        ("neg", -1.0, WING_NEG_GLASS),
    ):
        wing_assembly.visual(
            Box((arm_length, 0.08, 0.06)),
            origin=Origin(xyz=(sign * 0.09, 0.0, 2.05)),
            material=dark_metal,
            name=f"top_spider_arm_{side_name}",
        )
        wing_assembly.visual(
            Box((0.16, 0.08, 0.05)),
            origin=Origin(xyz=(sign * 0.08, 0.0, 0.08)),
            material=dark_metal,
            name=f"bottom_spider_arm_{side_name}",
        )
        wing_assembly.visual(
            Box((0.05, 0.05, 2.00)),
            origin=Origin(xyz=(sign * wing_inner_center, 0.0, 1.05)),
            material=aluminum,
            name=f"inner_stile_{side_name}",
        )
        wing_assembly.visual(
            Box((0.05, 0.05, 2.00)),
            origin=Origin(xyz=(sign * wing_outer_center, 0.0, 1.05)),
            material=aluminum,
            name=f"outer_stile_{side_name}",
        )
        wing_assembly.visual(
            Box((rail_length, 0.05, 0.05)),
            origin=Origin(xyz=(sign * rail_center, 0.0, 2.03)),
            material=aluminum,
            name=f"top_rail_{side_name}",
        )
        wing_assembly.visual(
            Box((rail_length, 0.05, 0.05)),
            origin=Origin(xyz=(sign * rail_center, 0.0, 0.08)),
            material=aluminum,
            name=f"bottom_rail_{side_name}",
        )
        wing_assembly.visual(
            Box((0.82, 0.014, 1.92)),
            origin=Origin(xyz=(sign * rail_center, 0.0, 1.06)),
            material=glass,
            name=glass_name,
        )
    wing_assembly.inertial = Inertial.from_geometry(
        Box((1.86, 0.18, 2.26)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
    )

    model.articulation(
        "drum_to_wings",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=wing_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.8),
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

    drum = object_model.get_part("drum")
    wing_assembly = object_model.get_part("wing_assembly")
    spin = object_model.get_articulation("drum_to_wings")

    ctx.check("drum part exists", drum is not None)
    ctx.check("wing assembly part exists", wing_assembly is not None)
    ctx.check(
        "door uses continuous rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={spin.motion_limits}"
        ),
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            inner_elem=WING_POS_GLASS,
            outer_elem="drum_shell",
            margin=0.0,
            name="positive wing stays inside drum footprint at rest",
        )
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            inner_elem=WING_NEG_GLASS,
            outer_elem="drum_shell",
            margin=0.0,
            name="negative wing stays inside drum footprint at rest",
        )
        ctx.expect_gap(
            drum,
            wing_assembly,
            axis="y",
            positive_elem="brake_housing_pos_y",
            negative_elem="brake_drum_rotor",
            min_gap=0.015,
            max_gap=0.030,
            name="positive brake housing clears rotating brake drum",
        )
        ctx.expect_gap(
            wing_assembly,
            drum,
            axis="y",
            positive_elem="brake_drum_rotor",
            negative_elem="brake_housing_neg_y",
            min_gap=0.015,
            max_gap=0.030,
            name="negative brake housing clears rotating brake drum",
        )

        rest_aabb = ctx.part_element_world_aabb(wing_assembly, elem=WING_POS_GLASS)

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            inner_elem=WING_POS_GLASS,
            outer_elem="drum_shell",
            margin=0.0,
            name="positive wing stays inside drum footprint at quarter turn",
        )
        ctx.expect_within(
            wing_assembly,
            drum,
            axes="xy",
            inner_elem=WING_NEG_GLASS,
            outer_elem="drum_shell",
            margin=0.0,
            name="negative wing stays inside drum footprint at quarter turn",
        )
        quarter_aabb = ctx.part_element_world_aabb(wing_assembly, elem=WING_POS_GLASS)

    def _center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    rest_center = _center(rest_aabb)
    quarter_center = _center(quarter_aabb)
    ctx.check(
        "positive wing rotates from +x toward +y",
        rest_center is not None
        and quarter_center is not None
        and rest_center[0] > 0.45
        and abs(rest_center[1]) < 0.03
        and abs(quarter_center[0]) < 0.03
        and quarter_center[1] > 0.45,
        details=f"rest_center={rest_center}, quarter_center={quarter_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
