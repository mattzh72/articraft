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


BASE_RAIL_LEN = 2.00
MID_RAIL_LEN = 1.85
TOP_RAIL_LEN = 1.70

BASE_RAIL_SEP = 0.304
MID_RAIL_SEP = 0.290
TOP_RAIL_SEP = 0.276

BASE_RAIL_W = 0.028
MID_RAIL_W = 0.024
TOP_RAIL_W = 0.022

BASE_RAIL_D = 0.022
MID_RAIL_D = 0.019
TOP_RAIL_D = 0.016

MID_STAGE_Y = 0.029
TOP_STAGE_REL_Y = 0.029

BASE_TO_MID_Z = 0.20
MID_TO_TOP_Z = 0.18
MID_TRAVEL = 0.95
TOP_TRAVEL = 0.88

BASE_RUNGS = (0.30, 0.58, 0.86, 1.14, 1.42, 1.70)
MID_RUNGS = (0.26, 0.52, 0.78, 1.04, 1.30, 1.56)
TOP_RUNGS = (0.24, 0.50, 0.76, 1.02, 1.28, 1.54)

COLLAR_WALL = 0.0035
COLLAR_SIDE_CLEARANCE = 0.002
BASE_COLLAR_HEIGHT = 0.11
MID_COLLAR_HEIGHT = 0.10
BASE_COLLAR_ZS = (1.64, 1.82)
MID_COLLAR_ZS = (1.46, 1.62)

FOOT_BARREL_RADIUS = 0.006
FOOT_BARREL_LENGTH = 0.032
FOOT_PAD_SIZE = (0.034, 0.046, 0.018)
FOOT_LOWER = -0.25
FOOT_UPPER = 0.95


def _add_rung(part, *, z: float, length: float, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_front_guide_collars(
    part,
    *,
    parent_rail_d: float,
    child_rail_sep: float,
    child_rail_w: float,
    child_rail_d: float,
    child_stage_y: float,
    z_positions: tuple[float, ...],
    collar_height: float,
    wall_t: float,
    side_clearance: float,
    material,
) -> None:
    parent_front = parent_rail_d * 0.5
    child_back = child_stage_y - child_rail_d * 0.5
    child_front = child_stage_y + child_rail_d * 0.5
    bridge_depth = child_back - parent_front
    side_depth = child_front + side_clearance - parent_front
    bridge_width = child_rail_w + 2.0 * side_clearance + 2.0 * wall_t
    side_offset_x = child_rail_w * 0.5 + side_clearance + wall_t * 0.5
    bridge_center_y = parent_front + bridge_depth * 0.5
    side_center_y = parent_front + side_depth * 0.5

    for collar_index, z in enumerate(z_positions):
        for sign, side_name in ((-1.0, "left"), (1.0, "right")):
            child_x = sign * child_rail_sep * 0.5
            prefix = f"{side_name}_guide_{collar_index}"
            part.visual(
                Box((bridge_width, bridge_depth, collar_height)),
                origin=Origin(xyz=(child_x, bridge_center_y, z)),
                material=material,
                name=f"{prefix}_bridge",
            )
            part.visual(
                Box((wall_t, side_depth, collar_height)),
                origin=Origin(xyz=(child_x - side_offset_x, side_center_y, z)),
                material=material,
                name=f"{prefix}_outer_wall",
            )
            part.visual(
                Box((wall_t, side_depth, collar_height)),
                origin=Origin(xyz=(child_x + side_offset_x, side_center_y, z)),
                material=material,
                name=f"{prefix}_inner_wall",
            )


def _add_stage(
    part,
    *,
    rail_sep: float,
    rail_w: float,
    rail_d: float,
    rail_len: float,
    rung_positions: tuple[float, ...],
    rung_radius: float,
    rail_material,
    guide_material=None,
    child_guide: dict[str, object] | None = None,
) -> None:
    part.visual(
        Box((rail_w, rail_d, rail_len)),
        origin=Origin(xyz=(-rail_sep * 0.5, 0.0, rail_len * 0.5)),
        material=rail_material,
        name="left_rail",
    )
    part.visual(
        Box((rail_w, rail_d, rail_len)),
        origin=Origin(xyz=(rail_sep * 0.5, 0.0, rail_len * 0.5)),
        material=rail_material,
        name="right_rail",
    )

    rung_length = rail_sep - rail_w
    for rung_index, z in enumerate(rung_positions):
        _add_rung(
            part,
            z=z,
            length=rung_length,
            radius=rung_radius,
            material=rail_material,
            name=f"rung_{rung_index}",
        )

    if child_guide is not None and guide_material is not None:
        _add_front_guide_collars(
            part,
            parent_rail_d=rail_d,
            child_rail_sep=child_guide["rail_sep"],
            child_rail_w=child_guide["rail_w"],
            child_rail_d=child_guide["rail_d"],
            child_stage_y=child_guide["stage_y"],
            z_positions=child_guide["collar_zs"],
            collar_height=child_guide["collar_height"],
            wall_t=child_guide["wall_t"],
            side_clearance=child_guide["side_clearance"],
            material=guide_material,
        )


def _add_upper_standoff_foot(part, *, material) -> None:
    part.visual(
        Cylinder(radius=FOOT_BARREL_RADIUS, length=FOOT_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="hinge_barrel",
    )
    part.visual(
        Box(FOOT_PAD_SIZE),
        origin=Origin(
            xyz=(0.0, FOOT_BARREL_RADIUS + FOOT_PAD_SIZE[1] * 0.5, 0.0),
        ),
        material=material,
        name="pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="confined_access_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.84, 1.0))
    guide_polymer = model.material("guide_polymer", rgba=(0.18, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base_stage")
    _add_stage(
        base,
        rail_sep=BASE_RAIL_SEP,
        rail_w=BASE_RAIL_W,
        rail_d=BASE_RAIL_D,
        rail_len=BASE_RAIL_LEN,
        rung_positions=BASE_RUNGS,
        rung_radius=0.009,
        rail_material=aluminum,
        guide_material=guide_polymer,
        child_guide={
            "rail_sep": MID_RAIL_SEP,
            "rail_w": MID_RAIL_W,
            "rail_d": MID_RAIL_D,
            "stage_y": MID_STAGE_Y,
            "collar_zs": BASE_COLLAR_ZS,
            "collar_height": BASE_COLLAR_HEIGHT,
            "wall_t": COLLAR_WALL,
            "side_clearance": COLLAR_SIDE_CLEARANCE,
        },
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_RAIL_SEP + BASE_RAIL_W, 0.050, BASE_RAIL_LEN)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_LEN * 0.5)),
    )

    mid = model.part("mid_fly_stage")
    _add_stage(
        mid,
        rail_sep=MID_RAIL_SEP,
        rail_w=MID_RAIL_W,
        rail_d=MID_RAIL_D,
        rail_len=MID_RAIL_LEN,
        rung_positions=MID_RUNGS,
        rung_radius=0.0085,
        rail_material=aluminum,
        guide_material=guide_polymer,
        child_guide={
            "rail_sep": TOP_RAIL_SEP,
            "rail_w": TOP_RAIL_W,
            "rail_d": TOP_RAIL_D,
            "stage_y": TOP_STAGE_REL_Y,
            "collar_zs": MID_COLLAR_ZS,
            "collar_height": MID_COLLAR_HEIGHT,
            "wall_t": COLLAR_WALL,
            "side_clearance": COLLAR_SIDE_CLEARANCE,
        },
    )
    mid.inertial = Inertial.from_geometry(
        Box((MID_RAIL_SEP + MID_RAIL_W, 0.050, MID_RAIL_LEN)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, MID_RAIL_LEN * 0.5)),
    )

    top = model.part("top_fly_stage")
    _add_stage(
        top,
        rail_sep=TOP_RAIL_SEP,
        rail_w=TOP_RAIL_W,
        rail_d=TOP_RAIL_D,
        rail_len=TOP_RAIL_LEN,
        rung_positions=TOP_RUNGS,
        rung_radius=0.008,
        rail_material=aluminum,
    )
    top.inertial = Inertial.from_geometry(
        Box((TOP_RAIL_SEP + TOP_RAIL_W, 0.040, TOP_RAIL_LEN)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, TOP_RAIL_LEN * 0.5)),
    )

    left_foot = model.part("left_upper_standoff_foot")
    _add_upper_standoff_foot(left_foot, material=rubber)
    left_foot.inertial = Inertial.from_geometry(
        Box((FOOT_PAD_SIZE[0], FOOT_PAD_SIZE[1] + FOOT_BARREL_RADIUS, FOOT_PAD_SIZE[2])),
        mass=0.18,
        origin=Origin(
            xyz=(0.0, FOOT_BARREL_RADIUS + FOOT_PAD_SIZE[1] * 0.5, 0.0),
        ),
    )

    right_foot = model.part("right_upper_standoff_foot")
    _add_upper_standoff_foot(right_foot, material=rubber)
    right_foot.inertial = Inertial.from_geometry(
        Box((FOOT_PAD_SIZE[0], FOOT_PAD_SIZE[1] + FOOT_BARREL_RADIUS, FOOT_PAD_SIZE[2])),
        mass=0.18,
        origin=Origin(
            xyz=(0.0, FOOT_BARREL_RADIUS + FOOT_PAD_SIZE[1] * 0.5, 0.0),
        ),
    )

    model.articulation(
        "base_to_mid_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mid,
        origin=Origin(xyz=(0.0, MID_STAGE_Y, BASE_TO_MID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.45,
            lower=0.0,
            upper=MID_TRAVEL,
        ),
    )
    model.articulation(
        "mid_to_top_fly",
        ArticulationType.PRISMATIC,
        parent=mid,
        child=top,
        origin=Origin(xyz=(0.0, TOP_STAGE_REL_Y, MID_TO_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.45,
            lower=0.0,
            upper=TOP_TRAVEL,
        ),
    )
    model.articulation(
        "top_to_left_standoff_foot",
        ArticulationType.REVOLUTE,
        parent=top,
        child=left_foot,
        origin=Origin(
            xyz=(-TOP_RAIL_SEP * 0.5, TOP_RAIL_D * 0.5, TOP_RAIL_LEN + FOOT_BARREL_RADIUS),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=FOOT_LOWER,
            upper=FOOT_UPPER,
        ),
    )
    model.articulation(
        "top_to_right_standoff_foot",
        ArticulationType.REVOLUTE,
        parent=top,
        child=right_foot,
        origin=Origin(
            xyz=(TOP_RAIL_SEP * 0.5, TOP_RAIL_D * 0.5, TOP_RAIL_LEN + FOOT_BARREL_RADIUS),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=FOOT_LOWER,
            upper=FOOT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_stage")
    mid = object_model.get_part("mid_fly_stage")
    top = object_model.get_part("top_fly_stage")
    left_foot = object_model.get_part("left_upper_standoff_foot")
    right_foot = object_model.get_part("right_upper_standoff_foot")
    base_to_mid = object_model.get_articulation("base_to_mid_fly")
    mid_to_top = object_model.get_articulation("mid_to_top_fly")
    left_foot_hinge = object_model.get_articulation("top_to_left_standoff_foot")
    right_foot_hinge = object_model.get_articulation("top_to_right_standoff_foot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fly joints extend vertically",
        base_to_mid.axis == (0.0, 0.0, 1.0) and mid_to_top.axis == (0.0, 0.0, 1.0),
        details=f"base_to_mid={base_to_mid.axis}, mid_to_top={mid_to_top.axis}",
    )
    ctx.check(
        "upper stand-off feet hinge about the stile width axis",
        left_foot_hinge.axis == (-1.0, 0.0, 0.0)
        and right_foot_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"left={left_foot_hinge.axis}, right={right_foot_hinge.axis}",
    )

    ctx.expect_contact(mid, base, name="mid fly stage bears in the base guide collars")
    ctx.expect_contact(top, mid, name="top fly stage bears in the mid guide collars")
    ctx.expect_contact(left_foot, top, name="left upper stand-off foot is mounted to the fly tip")
    ctx.expect_contact(right_foot, top, name="right upper stand-off foot is mounted to the fly tip")

    ctx.expect_gap(
        mid,
        base,
        axis="y",
        positive_elem="left_rail",
        negative_elem="left_rail",
        min_gap=0.008,
        max_gap=0.0095,
        name="mid fly rails run just ahead of the base stiles",
    )
    ctx.expect_gap(
        top,
        mid,
        axis="y",
        positive_elem="left_rail",
        negative_elem="left_rail",
        min_gap=0.011,
        max_gap=0.0125,
        name="top fly rails run just ahead of the mid stiles",
    )

    mid_rest = ctx.part_world_position(mid)
    top_rest = ctx.part_world_position(top)
    with ctx.pose({base_to_mid: MID_TRAVEL, mid_to_top: TOP_TRAVEL}):
        mid_extended = ctx.part_world_position(mid)
        top_extended = ctx.part_world_position(top)
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at full ladder extension")
        ctx.expect_overlap(
            mid,
            base,
            axes="z",
            elem_a="left_rail",
            elem_b="left_rail",
            min_overlap=0.84,
            name="mid fly retains insertion in the base at full extension",
        )
        ctx.expect_overlap(
            top,
            mid,
            axes="z",
            elem_a="left_rail",
            elem_b="left_rail",
            min_overlap=0.78,
            name="top fly retains insertion in the mid stage at full extension",
        )
    ctx.check(
        "fly stages extend upward",
        mid_rest is not None
        and top_rest is not None
        and mid_extended is not None
        and top_extended is not None
        and mid_extended[2] > mid_rest[2] + 0.90
        and top_extended[2] > top_rest[2] + 1.75,
        details=(
            f"mid_rest={mid_rest}, mid_extended={mid_extended}, "
            f"top_rest={top_rest}, top_extended={top_extended}"
        ),
    )

    rest_pad = ctx.part_element_world_aabb(left_foot, elem="pad")
    with ctx.pose({left_foot_hinge: FOOT_UPPER}):
        deployed_pad = ctx.part_element_world_aabb(left_foot, elem="pad")
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with a deployed upper stand-off foot")

    def _z_center(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_z = _z_center(rest_pad)
    deployed_z = _z_center(deployed_pad)
    ctx.check(
        "upper stand-off foot pitches downward when deployed",
        rest_z is not None and deployed_z is not None and deployed_z < rest_z - 0.015,
        details=f"rest_z={rest_z}, deployed_z={deployed_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
