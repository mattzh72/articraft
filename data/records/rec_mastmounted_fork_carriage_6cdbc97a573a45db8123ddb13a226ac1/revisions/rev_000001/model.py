from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MAST_HEIGHT = 2.25
UPRIGHT_CENTER_X = 0.44
UPRIGHT_WIDTH = 0.11
UPRIGHT_DEPTH = 0.10
BASE_SHOE_WIDTH = 0.18
BASE_SHOE_DEPTH = 0.18
BASE_SHOE_HEIGHT = 0.05
LOWER_TIE_WIDTH = 0.84
LOWER_TIE_DEPTH = 0.08
LOWER_TIE_HEIGHT = 0.10
LOWER_TIE_CENTER_Z = 0.20
TOP_CROSSHEAD_WIDTH = 0.99
TOP_CROSSHEAD_DEPTH = 0.10
TOP_CROSSHEAD_HEIGHT = 0.12
TOP_CROSSHEAD_CENTER_Z = MAST_HEIGHT - TOP_CROSSHEAD_HEIGHT / 2.0
GUIDE_WIDTH = 0.028
GUIDE_DEPTH = 0.02
GUIDE_HEIGHT = 1.92
GUIDE_CENTER_Z = 1.14
GUIDE_CENTER_Y = UPRIGHT_DEPTH / 2.0 + GUIDE_DEPTH / 2.0

CARRIAGE_JOINT_Y = GUIDE_CENTER_Y + GUIDE_DEPTH / 2.0
CARRIAGE_BASE_Z = 0.05
CARRIAGE_TRAVEL = 0.80

CARRIAGE_SIDE_X = 0.355
CARRIAGE_SIDE_WIDTH = 0.07
CARRIAGE_SIDE_DEPTH = 0.07
CARRIAGE_SIDE_HEIGHT = 0.55
CARRIAGE_SIDE_CENTER_Z = 0.60
GUIDE_PAD_WIDTH = 0.09
GUIDE_PAD_DEPTH = 0.03
GUIDE_PAD_HEIGHT = 0.56
GUIDE_PAD_CENTER_Y = 0.015
TOP_BAR_WIDTH = 0.78
TOP_BAR_DEPTH = 0.06
TOP_BAR_HEIGHT = 0.08
TOP_BAR_CENTER_Z = 0.74
BOTTOM_BAR_WIDTH = 0.78
BOTTOM_BAR_DEPTH = 0.06
BOTTOM_BAR_HEIGHT = 0.09
BOTTOM_BAR_CENTER_Z = 0.45
BACKREST_TOP_WIDTH = 0.70
BACKREST_TOP_DEPTH = 0.03
BACKREST_TOP_HEIGHT = 0.06
BACKREST_TOP_CENTER_Z = 1.21
BACKREST_MID_WIDTH = 0.62
BACKREST_MID_DEPTH = 0.03
BACKREST_MID_HEIGHT = 0.05
BACKREST_MID_CENTER_Z = 1.00
BACKREST_SLAT_WIDTH = 0.035
BACKREST_SLAT_DEPTH = 0.025
BACKREST_SLAT_HEIGHT = 0.47
BACKREST_SLAT_CENTER_Z = 0.975
BACKREST_SLAT_XS = (-0.24, -0.12, 0.0, 0.12, 0.24)

FORK_XS = (-0.22, 0.22)
FORK_SHANK_WIDTH = 0.10
FORK_SHANK_DEPTH = 0.06
FORK_SHANK_HEIGHT = 0.48
FORK_SHANK_CENTER_Y = 0.075
FORK_SHANK_CENTER_Z = 0.285
FORK_MAIN_LENGTH = 0.79
FORK_WIDTH = 0.10
FORK_MAIN_HEIGHT = 0.045
FORK_MAIN_CENTER_Y = 0.445
FORK_MAIN_CENTER_Z = FORK_MAIN_HEIGHT / 2.0
FORK_TIP_LENGTH = 0.16
FORK_TIP_HEIGHT = 0.025
FORK_TIP_CENTER_Y = 0.92
FORK_TIP_CENTER_Z = FORK_TIP_HEIGHT / 2.0


def _box_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z))


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
):
    part.visual(Box(size), origin=_box_origin(*xyz), material=material, name=name)


def _fork_blade_mesh(mesh_name: str):
    blade_profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.0, 0.0),
                (0.0, FORK_MAIN_HEIGHT),
                (FORK_MAIN_LENGTH - FORK_TIP_LENGTH, FORK_MAIN_HEIGHT),
                (FORK_MAIN_LENGTH, FORK_TIP_HEIGHT),
                (FORK_MAIN_LENGTH, 0.0),
            ]
        )
        .close()
    )
    blade = blade_profile.extrude(FORK_WIDTH / 2.0, both=True)
    return mesh_from_cadquery(blade, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="forklift_mast_carriage")

    model.material("mast_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("carriage_steel", rgba=(0.46, 0.48, 0.52, 1.0))
    model.material("fork_steel", rgba=(0.10, 0.10, 0.11, 1.0))

    mast = model.part("mast_frame")
    for side_name, x_center in (("left", -UPRIGHT_CENTER_X), ("right", UPRIGHT_CENTER_X)):
        _add_box(
            mast,
            size=(UPRIGHT_WIDTH, UPRIGHT_DEPTH, MAST_HEIGHT),
            xyz=(x_center, 0.0, MAST_HEIGHT / 2.0),
            material="mast_steel",
            name=f"{side_name}_upright",
        )
        guide_x = x_center + (GUIDE_WIDTH / 2.0 if side_name == "left" else -GUIDE_WIDTH / 2.0)
        _add_box(
            mast,
            size=(GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT),
            xyz=(guide_x, GUIDE_CENTER_Y, GUIDE_CENTER_Z),
            material="mast_steel",
            name=f"{side_name}_guide",
        )
        _add_box(
            mast,
            size=(BASE_SHOE_WIDTH, BASE_SHOE_DEPTH, BASE_SHOE_HEIGHT),
            xyz=(x_center, 0.0, BASE_SHOE_HEIGHT / 2.0),
            material="mast_steel",
            name=f"{side_name}_base_shoe",
        )

    _add_box(
        mast,
        size=(TOP_CROSSHEAD_WIDTH, TOP_CROSSHEAD_DEPTH, TOP_CROSSHEAD_HEIGHT),
        xyz=(0.0, 0.0, TOP_CROSSHEAD_CENTER_Z),
        material="mast_steel",
        name="crosshead",
    )
    _add_box(
        mast,
        size=(LOWER_TIE_WIDTH, LOWER_TIE_DEPTH, LOWER_TIE_HEIGHT),
        xyz=(0.0, 0.0, LOWER_TIE_CENTER_Z),
        material="mast_steel",
        name="lower_tie",
    )
    mast.inertial = Inertial.from_geometry(
        Box((TOP_CROSSHEAD_WIDTH, BASE_SHOE_DEPTH, MAST_HEIGHT)),
        mass=230.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("fork_carriage")
    _add_box(
        carriage,
        size=(TOP_BAR_WIDTH, TOP_BAR_DEPTH, TOP_BAR_HEIGHT),
        xyz=(0.0, 0.055, TOP_BAR_CENTER_Z),
        material="carriage_steel",
        name="top_bar",
    )
    _add_box(
        carriage,
        size=(BOTTOM_BAR_WIDTH, BOTTOM_BAR_DEPTH, BOTTOM_BAR_HEIGHT),
        xyz=(0.0, 0.055, BOTTOM_BAR_CENTER_Z),
        material="carriage_steel",
        name="bottom_bar",
    )
    _add_box(
        carriage,
        size=(BACKREST_TOP_WIDTH, BACKREST_TOP_DEPTH, BACKREST_TOP_HEIGHT),
        xyz=(0.0, 0.045, BACKREST_TOP_CENTER_Z),
        material="carriage_steel",
        name="backrest_top",
    )
    _add_box(
        carriage,
        size=(BACKREST_MID_WIDTH, BACKREST_MID_DEPTH, BACKREST_MID_HEIGHT),
        xyz=(0.0, 0.045, BACKREST_MID_CENTER_Z),
        material="carriage_steel",
        name="backrest_mid",
    )

    for side_name, x_center in (("left", -CARRIAGE_SIDE_X), ("right", CARRIAGE_SIDE_X)):
        _add_box(
            carriage,
            size=(CARRIAGE_SIDE_WIDTH, CARRIAGE_SIDE_DEPTH, CARRIAGE_SIDE_HEIGHT),
            xyz=(x_center, 0.055, CARRIAGE_SIDE_CENTER_Z),
            material="carriage_steel",
            name=f"{side_name}_side_plate",
        )
        pad_x = -0.38 if side_name == "left" else 0.38
        _add_box(
            carriage,
            size=(GUIDE_PAD_WIDTH, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT),
            xyz=(pad_x, GUIDE_PAD_CENTER_Y, CARRIAGE_SIDE_CENTER_Z),
            material="carriage_steel",
            name=f"{side_name}_guide_pad",
        )

    for index, x_center in enumerate(BACKREST_SLAT_XS, start=1):
        _add_box(
            carriage,
            size=(BACKREST_SLAT_WIDTH, BACKREST_SLAT_DEPTH, BACKREST_SLAT_HEIGHT),
            xyz=(x_center, 0.045, BACKREST_SLAT_CENTER_Z),
            material="carriage_steel",
            name=f"backrest_slat_{index}",
        )

    for side_name, x_center in (("left", FORK_XS[0]), ("right", FORK_XS[1])):
        _add_box(
            carriage,
            size=(FORK_SHANK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT),
            xyz=(x_center, FORK_SHANK_CENTER_Y, FORK_SHANK_CENTER_Z),
            material="fork_steel",
            name=f"{side_name}_fork_shank",
        )
        carriage.visual(
            _fork_blade_mesh(f"{side_name}_fork_blade"),
            origin=Origin(xyz=(x_center, 0.05, 0.0)),
            material="fork_steel",
            name=f"{side_name}_fork_main",
        )

    carriage.inertial = Inertial.from_geometry(
        Box((0.82, 1.00, 1.25)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.50, 0.625)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_JOINT_Y, CARRIAGE_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=12000.0,
            velocity=0.35,
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

    mast = object_model.get_part("mast_frame")
    carriage = object_model.get_part("fork_carriage")
    mast_to_carriage = object_model.get_articulation("mast_to_carriage")

    left_guide = mast.get_visual("left_guide")
    right_guide = mast.get_visual("right_guide")
    crosshead = mast.get_visual("crosshead")
    left_pad = carriage.get_visual("left_guide_pad")
    right_pad = carriage.get_visual("right_guide_pad")
    backrest_top = carriage.get_visual("backrest_top")
    left_fork = carriage.get_visual("left_fork_main")
    right_fork = carriage.get_visual("right_fork_main")

    ctx.expect_contact(
        carriage,
        mast,
        elem_a=left_pad,
        elem_b=left_guide,
        name="left guide pad bears on the left mast rail",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=right_pad,
        elem_b=right_guide,
        name="right guide pad bears on the right mast rail",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="xz",
        elem_a=left_pad,
        elem_b=left_guide,
        min_overlap=0.01,
        name="left guide pad overlaps the rail in both width and engaged height",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        margin=0.02,
        name="carriage stays centered between the two uprights",
    )

    left_fork_aabb = ctx.part_element_world_aabb(carriage, elem=left_fork.name)
    right_fork_aabb = ctx.part_element_world_aabb(carriage, elem=right_fork.name)
    fork_gap = None
    if left_fork_aabb is not None and right_fork_aabb is not None:
        fork_gap = right_fork_aabb[0][0] - left_fork_aabb[1][0]
    ctx.check(
        "fork pair remains visibly separated",
        fork_gap is not None and fork_gap > 0.12,
        details=f"fork_gap={fork_gap}, left={left_fork_aabb}, right={right_fork_aabb}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({mast_to_carriage: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=left_pad,
            elem_b=left_guide,
            name="left guide pad stays in contact with the rail at full lift",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=right_pad,
            elem_b=right_guide,
            name="right guide pad stays in contact with the rail at full lift",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a=left_pad,
            elem_b=left_guide,
            min_overlap=0.45,
            name="left guide pad retains substantial engagement with the mast rail",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=crosshead,
            negative_elem=backrest_top,
            min_gap=0.03,
            max_gap=0.10,
            name="backrest rises close to but below the crosshead",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage lifts upward along +Z",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + CARRIAGE_TRAVEL - 0.01,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
