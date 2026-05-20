from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_bell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.022),
            (0.040, 0.010),
            (0.060, -0.016),
            (0.072, -0.050),
            (0.080, -0.082),
            (0.084, -0.092),
        ],
        [
            (0.000, 0.020),
            (0.016, 0.016),
            (0.026, 0.006),
            (0.040, -0.020),
            (0.056, -0.056),
            (0.070, -0.086),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stone_bell_tower", assets=ASSETS)

    stone = model.material("stone", rgba=(0.63, 0.63, 0.60, 1.0))
    dark_stone = model.material("dark_stone", rgba=(0.52, 0.52, 0.49, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.46, 0.20, 1.0))
    wood = model.material("wood", rgba=(0.36, 0.24, 0.14, 1.0))
    iron = model.material("iron", rgba=(0.20, 0.21, 0.23, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.46, 0.46, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_stone,
        name="plinth",
    )
    tower.visual(
        Box((0.34, 0.34, 0.84)),
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        material=stone,
        name="shaft",
    )
    tower.visual(
        Box((0.42, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=dark_stone,
        name="belfry_sill",
    )

    pier_size = 0.055
    pier_offset = 0.1825
    pier_height = 0.34
    pier_center_z = 1.15
    for pier_name, x_pos, y_pos in [
        ("front_left_pier", -pier_offset, pier_offset),
        ("front_right_pier", pier_offset, pier_offset),
        ("rear_left_pier", -pier_offset, -pier_offset),
        ("rear_right_pier", pier_offset, -pier_offset),
    ]:
        tower.visual(
            Box((pier_size, pier_size, pier_height)),
            origin=Origin(xyz=(x_pos, y_pos, pier_center_z)),
            material=stone,
            name=pier_name,
        )

    tower.visual(
        Box((0.31, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, 0.145, 1.20)),
        material=dark_stone,
        name="front_support",
    )
    tower.visual(
        Box((0.31, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, -0.145, 1.20)),
        material=dark_stone,
        name="rear_support",
    )
    tower.visual(
        Box((0.42, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=dark_stone,
        name="roof_slab",
    )
    tower.visual(
        Box((0.26, 0.26, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.41)),
        material=stone,
        name="cap_block",
    )
    tower.visual(
        Cylinder(radius=0.028, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=dark_stone,
        name="finial",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 1.54)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
    )

    bell_assembly = model.part("bell_assembly")
    bell_assembly.visual(
        Box((0.05, 0.23, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wood,
        name="yoke_beam",
    )
    bell_assembly.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(0.0, 0.125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="front_pin",
    )
    bell_assembly.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(0.0, -0.125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="rear_pin",
    )
    bell_assembly.visual(
        Box((0.05, 0.08, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=wood,
        name="crown_block",
    )
    bell_assembly.visual(
        _save_mesh("bronze_bell.obj", _build_bell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=bronze,
        name="bell_body",
    )
    bell_assembly.visual(
        Cylinder(radius=0.008, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=iron,
        name="clapper_rod",
    )
    bell_assembly.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.0, 0.0, -0.197)),
        material=iron,
        name="clapper_head",
    )
    bell_assembly.inertial = Inertial.from_geometry(
        Box((0.18, 0.24, 0.24)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    bell_assembly = object_model.get_part("bell_assembly")
    bell_swing = object_model.get_articulation("bell_swing")
    belfry_sill = tower.get_visual("belfry_sill")
    roof_slab = tower.get_visual("roof_slab")
    front_left_pier = tower.get_visual("front_left_pier")
    front_right_pier = tower.get_visual("front_right_pier")
    rear_left_pier = tower.get_visual("rear_left_pier")
    front_support = tower.get_visual("front_support")
    rear_support = tower.get_visual("rear_support")
    yoke_beam = bell_assembly.get_visual("yoke_beam")
    front_pin = bell_assembly.get_visual("front_pin")
    rear_pin = bell_assembly.get_visual("rear_pin")
    bell_body = bell_assembly.get_visual("bell_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.13)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        tower,
        tower,
        axis="x",
        min_gap=0.30,
        positive_elem=front_right_pier,
        negative_elem=front_left_pier,
        name="front_open_belfry_width",
    )
    ctx.expect_gap(
        tower,
        tower,
        axis="y",
        min_gap=0.30,
        positive_elem=front_left_pier,
        negative_elem=rear_left_pier,
        name="side_open_belfry_depth",
    )
    ctx.expect_gap(
        tower,
        tower,
        axis="z",
        min_gap=0.33,
        positive_elem=roof_slab,
        negative_elem=belfry_sill,
        name="open_belfry_height",
    )
    ctx.expect_within(
        bell_assembly,
        tower,
        axes="xy",
        inner_elem=bell_body,
        outer_elem=belfry_sill,
        name="bell_centered_in_belfry",
    )
    ctx.expect_gap(
        bell_assembly,
        bell_assembly,
        axis="z",
        min_gap=0.04,
        positive_elem=yoke_beam,
        negative_elem=bell_body,
        name="bell_hangs_below_yoke",
    )
    ctx.expect_contact(
        tower,
        bell_assembly,
        elem_a=front_support,
        elem_b=front_pin,
        name="front_pin_seated_in_support",
    )
    ctx.expect_contact(
        tower,
        bell_assembly,
        elem_a=rear_support,
        elem_b=rear_pin,
        name="rear_pin_seated_in_support",
    )
    ctx.expect_gap(
        bell_assembly,
        tower,
        axis="z",
        min_gap=0.03,
        positive_elem=bell_body,
        negative_elem=belfry_sill,
        name="bell_clear_of_sill_at_rest",
    )
    ctx.expect_gap(
        tower,
        bell_assembly,
        axis="z",
        min_gap=0.18,
        positive_elem=roof_slab,
        negative_elem=bell_body,
        name="roof_above_bell_at_rest",
    )
    with ctx.pose({bell_swing: 0.35}):
        ctx.expect_within(
            bell_assembly,
            tower,
            axes="xy",
            inner_elem=bell_body,
            outer_elem=belfry_sill,
            name="bell_stays_within_belfry_when_swung",
        )
        ctx.expect_gap(
            bell_assembly,
            tower,
            axis="z",
            min_gap=0.01,
            positive_elem=bell_body,
            negative_elem=belfry_sill,
            name="bell_clear_of_sill_when_swung",
        )
        ctx.expect_gap(
            tower,
            bell_assembly,
            axis="z",
            min_gap=0.16,
            positive_elem=roof_slab,
            negative_elem=bell_body,
            name="roof_above_bell_when_swung",
        )
    with ctx.pose({bell_swing: -0.35}):
        ctx.expect_within(
            bell_assembly,
            tower,
            axes="xy",
            inner_elem=bell_body,
            outer_elem=belfry_sill,
            name="bell_stays_within_belfry_when_swung_opposite",
        )
        ctx.expect_gap(
            bell_assembly,
            tower,
            axis="z",
            min_gap=0.01,
            positive_elem=bell_body,
            negative_elem=belfry_sill,
            name="bell_clear_of_sill_when_swung_opposite",
        )
        ctx.expect_gap(
            tower,
            bell_assembly,
            axis="z",
            min_gap=0.16,
            positive_elem=roof_slab,
            negative_elem=bell_body,
            name="roof_above_bell_when_swung_opposite",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
