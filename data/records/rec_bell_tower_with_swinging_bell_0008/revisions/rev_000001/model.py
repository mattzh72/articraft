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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bell_tower", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.69, 0.70, 0.72, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.66, 0.48, 0.24, 1.0))

    outer_bell = LatheGeometry(
        [
            (0.0, 0.000),
            (0.020, 0.004),
            (0.034, -0.006),
            (0.046, -0.020),
            (0.060, -0.048),
            (0.072, -0.090),
            (0.082, -0.145),
            (0.094, -0.196),
            (0.108, -0.226),
            (0.114, -0.238),
            (0.0, -0.238),
        ],
        segments=72,
    )
    inner_bell = LatheGeometry(
        [
            (0.0, -0.016),
            (0.018, -0.018),
            (0.030, -0.028),
            (0.044, -0.052),
            (0.056, -0.092),
            (0.064, -0.146),
            (0.074, -0.196),
            (0.088, -0.242),
            (0.0, -0.258),
        ],
        segments=72,
    )
    bell_shell_mesh = mesh_from_geometry(
        boolean_difference(outer_bell, inner_bell),
        ASSETS.mesh_path("bell_shell.obj"),
    )

    tower = model.part("tower")
    tower.visual(
        Box((0.46, 0.40, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        Box((0.34, 0.30, 0.82)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=concrete,
        name="lower_shaft",
    )
    tower.visual(
        Box((0.045, 0.30, 0.50)),
        origin=Origin(xyz=(-0.1475, 0.0, 1.15)),
        material=concrete,
        name="left_wall",
    )
    tower.visual(
        Box((0.045, 0.30, 0.50)),
        origin=Origin(xyz=(0.1475, 0.0, 1.15)),
        material=concrete,
        name="right_wall",
    )
    tower.visual(
        Box((0.25, 0.035, 0.11)),
        origin=Origin(xyz=(0.0, 0.1325, 1.345)),
        material=concrete,
        name="front_header",
    )
    tower.visual(
        Box((0.25, 0.035, 0.11)),
        origin=Origin(xyz=(0.0, -0.1325, 1.345)),
        material=concrete,
        name="rear_header",
    )
    tower.visual(
        Box((0.40, 0.36, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.425)),
        material=concrete,
        name="cap_slab",
    )
    tower.visual(
        Box((0.04, 0.12, 0.08)),
        origin=Origin(xyz=(-0.105, 0.0, 1.19)),
        material=weathered_steel,
        name="left_bearing",
    )
    tower.visual(
        Box((0.04, 0.12, 0.08)),
        origin=Origin(xyz=(0.105, 0.0, 1.19)),
        material=weathered_steel,
        name="right_bearing",
    )
    tower.visual(
        Box((0.21, 0.03, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.24)),
        material=weathered_steel,
        name="frame_tie",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.46, 0.40, 1.45)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
    )

    bell_assembly = model.part("bell_assembly")
    bell_assembly.visual(
        Cylinder(radius=0.012, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    bell_assembly.visual(
        Box((0.15, 0.038, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=weathered_steel,
        name="yoke_beam",
    )
    bell_assembly.visual(
        Box((0.13, 0.010, 0.11)),
        origin=Origin(xyz=(0.0, 0.028, -0.060)),
        material=weathered_steel,
        name="front_cheek",
    )
    bell_assembly.visual(
        Box((0.13, 0.010, 0.11)),
        origin=Origin(xyz=(0.0, -0.028, -0.060)),
        material=weathered_steel,
        name="rear_cheek",
    )
    bell_assembly.visual(
        Box((0.05, 0.07, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_steel,
        name="crown_block",
    )
    bell_assembly.visual(
        bell_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=bell_bronze,
        name="bell_shell",
    )
    bell_assembly.visual(
        Cylinder(radius=0.006, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=dark_steel,
        name="clapper_rod",
    )
    bell_assembly.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.214)),
        material=dark_steel,
        name="clapper_ball",
    )
    bell_assembly.inertial = Inertial.from_geometry(
        Box((0.23, 0.18, 0.42)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=1.8,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    bell_assembly = object_model.get_part("bell_assembly")
    bell_swing = object_model.get_articulation("bell_swing")
    lower_shaft = tower.get_visual("lower_shaft")
    cap_slab = tower.get_visual("cap_slab")
    left_bearing = tower.get_visual("left_bearing")
    right_bearing = tower.get_visual("right_bearing")
    bell_shell = bell_assembly.get_visual("bell_shell")
    axle = bell_assembly.get_visual("axle")
    yoke_beam = bell_assembly.get_visual("yoke_beam")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.035)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_within(bell_assembly, tower, axes="xy", inner_elem=bell_shell)
    ctx.expect_origin_distance(bell_assembly, tower, axes="xy", max_dist=0.02)
    ctx.expect_contact(bell_assembly, tower, elem_a=axle, elem_b=left_bearing)
    ctx.expect_contact(bell_assembly, tower, elem_a=axle, elem_b=right_bearing)
    ctx.expect_gap(
        bell_assembly,
        tower,
        axis="z",
        min_gap=0.01,
        max_gap=0.06,
        positive_elem=bell_shell,
        negative_elem=lower_shaft,
        name="bell_hangs_low_in_open_stage",
    )
    ctx.expect_gap(
        tower,
        bell_assembly,
        axis="z",
        min_gap=0.12,
        max_gap=0.30,
        positive_elem=cap_slab,
        negative_elem=yoke_beam,
        name="tower_cap_sits_above_yoke",
    )
    with ctx.pose({bell_swing: 0.45}):
        ctx.expect_contact(bell_assembly, tower, elem_a=axle, elem_b=left_bearing)
        ctx.expect_contact(bell_assembly, tower, elem_a=axle, elem_b=right_bearing)
        ctx.expect_gap(
            bell_assembly,
            tower,
            axis="z",
            min_gap=0.003,
            max_gap=0.09,
            positive_elem=bell_shell,
            negative_elem=lower_shaft,
            name="swung_bell_clears_lower_shaft",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
