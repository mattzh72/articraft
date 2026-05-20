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
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _square_loop(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _build_spire_mesh():
    return repair_loft(
        section_loft(
            [
                _square_loop(0.28, 0.0),
                _square_loop(0.20, 0.10),
                _square_loop(0.10, 0.24),
                _square_loop(0.016, 0.38),
            ],
            cap=True,
            solid=True,
        )
    )


def _build_bell_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.050, -0.132),
            (0.048, -0.110),
            (0.041, -0.078),
            (0.033, -0.042),
            (0.020, -0.012),
        ],
        [
            (0.041, -0.126),
            (0.037, -0.106),
            (0.030, -0.076),
            (0.022, -0.041),
            (0.009, -0.014),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_church_bell_tower", assets=ASSETS)

    weathered_wood = model.material("weathered_wood", rgba=(0.46, 0.31, 0.18, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.30, 0.20, 0.11, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.64, 0.48, 0.20, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.20, 0.22, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.34, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_timber,
        name="base_plinth",
    )
    tower.visual(
        Box((0.28, 0.28, 0.43)),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=weathered_wood,
        name="lower_body",
    )
    tower.visual(
        Box((0.31, 0.31, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.495)),
        material=dark_timber,
        name="belt_course",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((0.04, 0.04, 0.29)),
                origin=Origin(xyz=(0.15 * x_sign, 0.15 * y_sign, 0.655)),
                material=weathered_wood,
                name=f"corner_post_{'l' if x_sign < 0 else 'r'}_{'rear' if y_sign < 0 else 'front'}",
            )

    tower.visual(
        Box((0.26, 0.02, 0.20)),
        origin=Origin(xyz=(0.0, -0.16, 0.64)),
        material=weathered_wood,
        name="rear_panel",
    )
    tower.visual(
        Box((0.02, 0.26, 0.18)),
        origin=Origin(xyz=(-0.16, 0.0, 0.63)),
        material=weathered_wood,
        name="left_side_panel",
    )
    tower.visual(
        Box((0.02, 0.26, 0.18)),
        origin=Origin(xyz=(0.16, 0.0, 0.63)),
        material=weathered_wood,
        name="right_side_panel",
    )
    tower.visual(
        Box((0.26, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.16, 0.49)),
        material=dark_timber,
        name="opening_sill",
    )
    tower.visual(
        Box((0.26, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.16, 0.775)),
        material=dark_timber,
        name="opening_header",
    )
    tower.visual(
        Box((0.27, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.03, 0.716)),
        material=dark_timber,
        name="pivot_support_beam",
    )
    tower.visual(
        Box((0.05, 0.05, 0.05)),
        origin=Origin(xyz=(-0.135, 0.03, 0.70)),
        material=dark_timber,
        name="left_bearing",
    )
    tower.visual(
        Box((0.05, 0.05, 0.05)),
        origin=Origin(xyz=(0.135, 0.03, 0.70)),
        material=dark_timber,
        name="right_bearing",
    )
    tower.visual(
        Box((0.30, 0.30, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=dark_timber,
        name="top_plate",
    )
    tower.visual(
        _save_mesh(_build_spire_mesh(), "church_bell_tower_spire.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        material=weathered_wood,
        name="spire",
    )
    tower.visual(
        Cylinder(radius=0.010, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 1.235)),
        material=dark_metal,
        name="finial",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 1.24)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.004, length=0.22),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_bar",
    )
    bell.visual(
        Box((0.12, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, -0.02, -0.012)),
        material=dark_timber,
        name="yoke_beam",
    )
    bell.visual(
        Box((0.018, 0.03, 0.085)),
        origin=Origin(xyz=(-0.04, -0.02, -0.055)),
        material=dark_timber,
        name="left_hanger",
    )
    bell.visual(
        Box((0.018, 0.03, 0.085)),
        origin=Origin(xyz=(0.04, -0.02, -0.055)),
        material=dark_timber,
        name="right_hanger",
    )
    bell.visual(
        Cylinder(radius=0.014, length=0.05),
        origin=Origin(xyz=(0.0, -0.03, -0.060)),
        material=dark_timber,
        name="crown_block",
    )
    bell.visual(
        _save_mesh(_build_bell_shell_mesh(), "church_bell_shell.obj"),
        origin=Origin(xyz=(0.0, -0.03, -0.033)),
        material=aged_bronze,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.004, length=0.070),
        origin=Origin(xyz=(0.0, -0.03, -0.090)),
        material=dark_metal,
        name="clapper",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.16),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
    )

    model.articulation(
        "tower_to_bell",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.03, 0.70)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    bell_hinge = object_model.get_articulation("tower_to_bell")

    left_post = tower.get_visual("corner_post_l_front")
    right_post = tower.get_visual("corner_post_r_front")
    sill = tower.get_visual("opening_sill")
    header = tower.get_visual("opening_header")
    left_bearing = tower.get_visual("left_bearing")
    right_bearing = tower.get_visual("right_bearing")
    rear_panel = tower.get_visual("rear_panel")
    top_plate = tower.get_visual("top_plate")
    spire = tower.get_visual("spire")

    pivot_bar = bell.get_visual("pivot_bar")
    bell_shell = bell.get_visual("bell_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
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
        positive_elem=right_post,
        negative_elem=left_post,
        min_gap=0.24,
        max_gap=0.28,
        name="belfry opening spans most of the front bay",
    )
    ctx.expect_gap(
        tower,
        tower,
        axis="z",
        positive_elem=header,
        negative_elem=sill,
        min_gap=0.18,
        max_gap=0.24,
        name="belfry opening is tall enough to read clearly",
    )
    ctx.expect_gap(
        tower,
        tower,
        axis="z",
        positive_elem=spire,
        negative_elem=top_plate,
        max_gap=0.001,
        max_penetration=0.0,
        name="spire sits directly on the tower top plate",
    )
    ctx.expect_contact(
        bell,
        tower,
        elem_a=pivot_bar,
        elem_b=left_bearing,
        name="pivot bar seats on the left bearing block",
    )
    ctx.expect_contact(
        bell,
        tower,
        elem_a=pivot_bar,
        elem_b=right_bearing,
        name="pivot bar seats on the right bearing block",
    )
    ctx.expect_within(
        bell,
        tower,
        axes="x",
        inner_elem=bell_shell,
        outer_elem=header,
        name="bell shell stays within the front opening width",
    )
    ctx.expect_gap(
        bell,
        bell,
        axis="z",
        positive_elem=pivot_bar,
        negative_elem=bell_shell,
        min_gap=0.02,
        max_gap=0.08,
        name="bell hangs below its pivot bar",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="z",
        positive_elem=bell_shell,
        negative_elem=sill,
        min_gap=0.0,
        max_gap=0.04,
        name="bell mouth hangs just above the belfry sill",
    )
    ctx.expect_gap(
        tower,
        bell,
        axis="y",
        positive_elem=header,
        negative_elem=bell_shell,
        min_gap=0.01,
        name="bell sits behind the front facade plane",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="y",
        positive_elem=bell_shell,
        negative_elem=rear_panel,
        min_gap=0.03,
        name="bell clears the rear wall of the belfry",
    )
    with ctx.pose({bell_hinge: 0.45}):
        ctx.expect_gap(
            tower,
            bell,
            axis="y",
            positive_elem=header,
            negative_elem=bell_shell,
            min_gap=0.004,
            name="swung bell still stays behind the opening front",
        )
        ctx.expect_gap(
            bell,
            tower,
            axis="y",
            positive_elem=bell_shell,
            negative_elem=rear_panel,
            min_gap=0.012,
            name="swung bell still clears the rear wall",
        )
    with ctx.pose({bell_hinge: -0.45}):
        ctx.expect_gap(
            tower,
            bell,
            axis="y",
            positive_elem=header,
            negative_elem=bell_shell,
            min_gap=0.004,
            name="reverse swung bell still stays behind the opening front",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
