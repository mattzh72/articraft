from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

HOUSING_WIDTH = 0.34
HOUSING_DEPTH = 0.40
HOUSING_HEIGHT = 1.02
BASE_WIDTH = 0.42
BASE_DEPTH = 0.48
BASE_HEIGHT = 0.10
TOP_CAP_HEIGHT = 0.04

HINGE_Z = 0.88
HINGE_MOUNT_X = 0.20
HINGE_MOUNT_Y = HOUSING_DEPTH / 2.0 - 0.02
HINGE_ORIGIN_Y = HOUSING_DEPTH / 2.0 + 0.04
HINGE_MOUNT_SIZE = (0.08, 0.04, 0.14)

BOOM_LENGTH = 3.40
BOOM_WIDTH = 0.09
BOOM_HEIGHT = 0.055
BOOM_HUB_RADIUS = 0.03
BOOM_HUB_LENGTH = 0.04
BOOM_HUB_CENTER_Y = BOOM_HUB_LENGTH / 2.0
BOOM_BODY_Y = 0.04
BOOM_START_X = 0.12
STRIPE_COUNT = 8


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toll_booth_barrier_gate", assets=ASSETS)

    yellow = model.material("safety_yellow", rgba=(0.95, 0.77, 0.14, 1.0))
    dark_yellow = model.material("deep_yellow", rgba=(0.84, 0.65, 0.08, 1.0))
    white = model.material("paint_white", rgba=(0.97, 0.97, 0.97, 1.0))
    red = model.material("signal_red", rgba=(0.82, 0.10, 0.10, 1.0))
    black = model.material("rubber_black", rgba=(0.11, 0.11, 0.11, 1.0))
    metal = model.material("hinge_metal", rgba=(0.42, 0.45, 0.49, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=dark_yellow,
        name="base_plinth",
    )
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + HOUSING_HEIGHT / 2.0)),
        material=yellow,
        name="main_shell",
    )
    housing.visual(
        Box((HOUSING_WIDTH + 0.05, HOUSING_DEPTH + 0.04, TOP_CAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + HOUSING_HEIGHT + TOP_CAP_HEIGHT / 2.0)),
        material=dark_yellow,
        name="top_cap",
    )
    housing.visual(
        Box((0.20, 0.012, 0.24)),
        origin=Origin(xyz=(0.0, HOUSING_DEPTH / 2.0 + 0.006, 0.52)),
        material=black,
        name="door_seam",
    )
    housing.visual(
        Box(HINGE_MOUNT_SIZE),
        origin=Origin(xyz=(HINGE_MOUNT_X, HINGE_MOUNT_Y, HINGE_Z)),
        material=yellow,
        name="hinge_mount",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(
            xyz=(HINGE_MOUNT_X + HINGE_MOUNT_SIZE[0] / 2.0, HINGE_ORIGIN_Y - 0.02, HINGE_Z),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material=metal,
        name="pivot_boss",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT + HOUSING_HEIGHT + TOP_CAP_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + HOUSING_HEIGHT + TOP_CAP_HEIGHT) / 2.0)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=BOOM_HUB_RADIUS, length=BOOM_HUB_LENGTH),
        origin=Origin(
            xyz=(0.0, BOOM_HUB_CENTER_Y, 0.0),
            rpy=(1.57079632679, 0.0, 0.0),
        ),
        material=metal,
        name="boom_hub",
    )
    boom.visual(
        Box((0.16, 0.08, 0.07)),
        origin=Origin(xyz=(0.08, BOOM_BODY_Y, 0.0)),
        material=black,
        name="root_collar",
    )

    stripe_length = BOOM_LENGTH / STRIPE_COUNT
    for idx in range(STRIPE_COUNT):
        center_x = BOOM_START_X + stripe_length * (idx + 0.5)
        boom.visual(
            Box((stripe_length, BOOM_WIDTH, BOOM_HEIGHT)),
            origin=Origin(xyz=(center_x, BOOM_BODY_Y, 0.0)),
            material=white if idx % 2 == 0 else red,
            name=f"stripe_{idx}",
        )

    boom.visual(
        Box((0.05, BOOM_WIDTH * 0.8, BOOM_HEIGHT * 0.8)),
        origin=Origin(xyz=(BOOM_START_X + BOOM_LENGTH + 0.025, BOOM_BODY_Y, 0.0)),
        material=black,
        name="end_cap",
    )
    boom.inertial = Inertial.from_geometry(
        Box((BOOM_LENGTH + 0.20, 0.14, 0.10)),
        mass=7.0,
        origin=Origin(xyz=(1.77, 0.07, 0.0)),
    )

    model.articulation(
        "boom_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=boom,
        origin=Origin(xyz=(HINGE_MOUNT_X + HINGE_MOUNT_SIZE[0] / 2.0, HINGE_ORIGIN_Y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    boom = object_model.get_part("boom")
    boom_hinge = object_model.get_articulation("boom_hinge")

    base_plinth = housing.get_visual("base_plinth")
    main_shell = housing.get_visual("main_shell")
    top_cap = housing.get_visual("top_cap")
    hinge_mount = housing.get_visual("hinge_mount")
    pivot_boss = housing.get_visual("pivot_boss")

    boom_hub = boom.get_visual("boom_hub")
    root_collar = boom.get_visual("root_collar")
    stripe_0 = boom.get_visual("stripe_0")
    stripe_7 = boom.get_visual("stripe_7")
    end_cap = boom.get_visual("end_cap")

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
        housing,
        housing,
        axis="z",
        positive_elem=main_shell,
        negative_elem=base_plinth,
        max_gap=0.001,
        max_penetration=1e-6,
        name="housing shell sits on plinth",
    )
    ctx.expect_gap(
        housing,
        housing,
        axis="z",
        positive_elem=top_cap,
        negative_elem=main_shell,
        max_gap=0.001,
        max_penetration=1e-6,
        name="top cap sits on housing shell",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="y",
        positive_elem=boom_hub,
        negative_elem=pivot_boss,
        max_gap=0.001,
        max_penetration=1e-6,
        name="boom hub seats against pivot boss",
    )
    ctx.expect_overlap(
        boom,
        housing,
        axes="xz",
        elem_a=boom_hub,
        elem_b=hinge_mount,
        min_overlap=0.003,
        name="boom hinge overlaps hinge mount footprint",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="x",
        positive_elem=root_collar,
        negative_elem=hinge_mount,
        max_gap=0.001,
        max_penetration=1e-6,
        name="boom root begins at the hinge end of the housing",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="x",
        positive_elem=stripe_7,
        negative_elem=hinge_mount,
        min_gap=2.6,
        name="barrier pole projects far across the lane",
    )
    ctx.expect_gap(
        boom,
        housing,
        axis="x",
        positive_elem=end_cap,
        negative_elem=hinge_mount,
        min_gap=3.0,
        name="boom end cap extends beyond housing",
    )
    ctx.expect_overlap(
        boom,
        boom,
        axes="yz",
        elem_a=stripe_0,
        elem_b=stripe_7,
        min_overlap=0.004,
        name="striped boom segments stay collinear",
    )
    ctx.expect_gap(
        boom,
        boom,
        axis="x",
        positive_elem=stripe_7,
        negative_elem=stripe_0,
        min_gap=2.5,
        name="boom reads as a long horizontal pole",
    )

    with ctx.pose({boom_hinge: 1.25}):
        ctx.expect_gap(
            boom,
            housing,
            axis="z",
            positive_elem=end_cap,
            negative_elem=top_cap,
            min_gap=2.0,
            name="raised boom clears high above housing",
        )
        ctx.expect_overlap(
            boom,
            housing,
            axes="xz",
            elem_a=boom_hub,
            elem_b=pivot_boss,
            min_overlap=0.03,
            name="boom remains hinged while raised",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
