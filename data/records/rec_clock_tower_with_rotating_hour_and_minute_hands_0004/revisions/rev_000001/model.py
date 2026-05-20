from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

PLINTH_SPAN = 2.05
PLINTH_HEIGHT = 0.42
SHAFT_SPAN = 1.58
SHAFT_HEIGHT = 3.95
CLOCK_STAGE_SPAN = 1.74
CLOCK_STAGE_HEIGHT = 1.02
CORNICE_SPAN = 1.92
CORNICE_HEIGHT = 0.18
TOWER_HEIGHT = PLINTH_HEIGHT + SHAFT_HEIGHT + CLOCK_STAGE_HEIGHT + CORNICE_HEIGHT

CLOCK_CENTER_Z = PLINTH_HEIGHT + SHAFT_HEIGHT + CLOCK_STAGE_HEIGHT * 0.52
CLOCK_PLAQUE_SPAN = 0.92
CLOCK_PLAQUE_THICK = 0.08
DIAL_RADIUS = 0.40
DIAL_BEZEL_RADIUS = 0.44
SIDE_DIAL_THICK = 0.04

ROOF_BASE_SPAN = 1.98
ROOF_BASE_THICK = 0.06
ROOF_HEIGHT = 1.10
ROOF_MID_SPAN = 0.86
ROOF_PEAK_SPAN = 0.12
FINIAL_RADIUS = 0.055
FINIAL_HEIGHT = 0.18

HOUR_HAND_THICK = 0.012
MINUTE_HAND_THICK = 0.010
HOUR_JOINT_Y = 0.112
MINUTE_JOINT_Y = HOUR_JOINT_Y + HOUR_HAND_THICK


def _square_loop(span: float, z: float) -> list[tuple[float, float, float]]:
    half = span * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _roof_mesh():
    roof_geom = section_loft(
        [
            _square_loop(ROOF_BASE_SPAN, 0.0),
            _square_loop(ROOF_MID_SPAN, ROOF_HEIGHT * 0.64),
            _square_loop(ROOF_PEAK_SPAN, ROOF_HEIGHT),
        ]
    )
    return mesh_from_geometry(roof_geom, ASSETS.mesh_path("station_clock_tower_roof.obj"))


def _hand_section(width: float, thickness: float, z: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_t = thickness * 0.5
    return [
        (-half_w, -half_t, z),
        (half_w, -half_t, z),
        (half_w, half_t, z),
        (-half_w, half_t, z),
    ]


def _hand_blade_mesh(
    filename: str,
    sections: list[tuple[float, float, float]],
):
    loops = [_hand_section(width, thickness, z) for z, width, thickness in sections]
    return mesh_from_geometry(section_loft(loops), ASSETS.mesh_path(filename))


def _add_static_side_dial(
    tower_body,
    *,
    name_prefix: str,
    axis: str,
    sign: float,
    stone,
    dial_cream,
    trim_dark,
) -> None:
    face_offset = CLOCK_STAGE_SPAN * 0.5
    plaque_center = sign * face_offset
    bezel_center = sign * (face_offset + 0.016)
    disk_center = sign * (face_offset + 0.010)

    if axis == "x":
        plaque_size = (SIDE_DIAL_THICK, CLOCK_PLAQUE_SPAN, CLOCK_PLAQUE_SPAN)
        plaque_origin = Origin(xyz=(plaque_center, 0.0, CLOCK_CENTER_Z))
        cyl_origin = Origin(
            xyz=(bezel_center, 0.0, CLOCK_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        )
        disk_origin = Origin(
            xyz=(disk_center, 0.0, CLOCK_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        )
    else:
        plaque_size = (CLOCK_PLAQUE_SPAN, SIDE_DIAL_THICK, CLOCK_PLAQUE_SPAN)
        plaque_origin = Origin(xyz=(0.0, plaque_center, CLOCK_CENTER_Z))
        cyl_origin = Origin(
            xyz=(0.0, bezel_center, CLOCK_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        disk_origin = Origin(
            xyz=(0.0, disk_center, CLOCK_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )

    tower_body.visual(
        Box(plaque_size),
        origin=plaque_origin,
        material=stone,
        name=f"{name_prefix}_plaque",
    )
    tower_body.visual(
        Cylinder(radius=DIAL_BEZEL_RADIUS, length=0.028),
        origin=cyl_origin,
        material=trim_dark,
        name=f"{name_prefix}_bezel",
    )
    tower_body.visual(
        Cylinder(radius=DIAL_RADIUS, length=0.012),
        origin=disk_origin,
        material=dial_cream,
        name=f"{name_prefix}_dial",
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_station_clock_tower", assets=ASSETS)

    brick = model.material("brick", rgba=(0.58, 0.25, 0.18, 1.0))
    stone = model.material("stone", rgba=(0.74, 0.70, 0.62, 1.0))
    slate = model.material("slate", rgba=(0.23, 0.25, 0.29, 1.0))
    dial_cream = model.material("dial_cream", rgba=(0.93, 0.91, 0.84, 1.0))
    hand_black = model.material("hand_black", rgba=(0.12, 0.11, 0.10, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.28, 0.24, 0.21, 1.0))

    tower_body = model.part("tower_body")
    tower_body.visual(
        Box((PLINTH_SPAN, PLINTH_SPAN, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=stone,
        name="plinth",
    )
    tower_body.visual(
        Box((SHAFT_SPAN, SHAFT_SPAN, SHAFT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + SHAFT_HEIGHT * 0.5)),
        material=brick,
        name="main_shaft",
    )
    tower_body.visual(
        Box((1.76, 1.76, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT + SHAFT_HEIGHT + 0.03)),
        material=stone,
        name="clock_stage_sill",
    )
    tower_body.visual(
        Box((CLOCK_STAGE_SPAN, CLOCK_STAGE_SPAN, CLOCK_STAGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PLINTH_HEIGHT + SHAFT_HEIGHT + CLOCK_STAGE_HEIGHT * 0.5,
            )
        ),
        material=brick,
        name="clock_stage",
    )
    tower_body.visual(
        Box((CORNICE_SPAN, CORNICE_SPAN, CORNICE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PLINTH_HEIGHT + SHAFT_HEIGHT + CLOCK_STAGE_HEIGHT + CORNICE_HEIGHT * 0.5,
            )
        ),
        material=stone,
        name="cornice",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower_body.visual(
                Box((0.16, 0.16, SHAFT_HEIGHT + 0.20)),
                origin=Origin(
                    xyz=(
                        x_sign * (SHAFT_SPAN * 0.5 - 0.08),
                        y_sign * (SHAFT_SPAN * 0.5 - 0.08),
                        PLINTH_HEIGHT + (SHAFT_HEIGHT + 0.20) * 0.5,
                    )
                ),
                material=stone,
                name=f"shaft_quoin_{int((x_sign + 1) * 0.5)}_{int((y_sign + 1) * 0.5)}",
            )
            tower_body.visual(
                Box((0.14, 0.14, CLOCK_STAGE_HEIGHT)),
                origin=Origin(
                    xyz=(
                        x_sign * (CLOCK_STAGE_SPAN * 0.5 - 0.07),
                        y_sign * (CLOCK_STAGE_SPAN * 0.5 - 0.07),
                        PLINTH_HEIGHT + SHAFT_HEIGHT + CLOCK_STAGE_HEIGHT * 0.5,
                    )
                ),
                material=stone,
                name=f"stage_pilaster_{int((x_sign + 1) * 0.5)}_{int((y_sign + 1) * 0.5)}",
            )

    _add_static_side_dial(
        tower_body,
        name_prefix="back_clock",
        axis="y",
        sign=-1.0,
        stone=stone,
        dial_cream=dial_cream,
        trim_dark=trim_dark,
    )
    _add_static_side_dial(
        tower_body,
        name_prefix="left_clock",
        axis="x",
        sign=-1.0,
        stone=stone,
        dial_cream=dial_cream,
        trim_dark=trim_dark,
    )
    _add_static_side_dial(
        tower_body,
        name_prefix="right_clock",
        axis="x",
        sign=1.0,
        stone=stone,
        dial_cream=dial_cream,
        trim_dark=trim_dark,
    )

    tower_body.inertial = Inertial.from_geometry(
        Box((PLINTH_SPAN, PLINTH_SPAN, TOWER_HEIGHT)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT * 0.5)),
    )

    roof_cap = model.part("roof_cap")
    roof_cap.visual(
        Box((ROOF_BASE_SPAN, ROOF_BASE_SPAN, ROOF_BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE_THICK * 0.5)),
        material=trim_dark,
        name="roof_plinth",
    )
    roof_cap.visual(
        _roof_mesh(),
        origin=Origin(xyz=(0.0, 0.0, ROOF_BASE_THICK)),
        material=slate,
        name="roof_shell",
    )
    roof_cap.visual(
        Cylinder(radius=FINIAL_RADIUS, length=FINIAL_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, ROOF_BASE_THICK + ROOF_HEIGHT + FINIAL_HEIGHT * 0.5)
        ),
        material=slate,
        name="finial",
    )
    roof_cap.inertial = Inertial.from_geometry(
        Box((ROOF_BASE_SPAN, ROOF_BASE_SPAN, ROOF_BASE_THICK + ROOF_HEIGHT + FINIAL_HEIGHT)),
        mass=18.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (ROOF_BASE_THICK + ROOF_HEIGHT + FINIAL_HEIGHT) * 0.5,
            )
        ),
    )

    clock_face = model.part("clock_face")
    clock_face.visual(
        Box((CLOCK_PLAQUE_SPAN, CLOCK_PLAQUE_THICK, CLOCK_PLAQUE_SPAN)),
        origin=Origin(xyz=(0.0, CLOCK_PLAQUE_THICK * 0.5, 0.0)),
        material=stone,
        name="clock_plaque",
    )
    clock_face.visual(
        Cylinder(radius=DIAL_BEZEL_RADIUS, length=0.036),
        origin=Origin(
            xyz=(0.0, 0.094, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="dial_bezel",
    )
    clock_face.visual(
        Cylinder(radius=DIAL_RADIUS, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.098, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dial_cream,
        name="dial_disk",
    )
    for mark_index in range(12):
        angle = mark_index * math.tau / 12.0
        radius = 0.31
        is_quarter = mark_index % 3 == 0
        mark_width = 0.022 if is_quarter else 0.012
        mark_length = 0.072 if is_quarter else 0.042
        clock_face.visual(
            Box((mark_width, 0.006, mark_length)),
            origin=Origin(
                xyz=(math.sin(angle) * radius, 0.103, math.cos(angle) * radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=trim_dark,
            name=f"hour_mark_{mark_index}",
        )
    clock_face.visual(
        Cylinder(radius=0.055, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.107, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="hub_boss",
    )
    clock_face.inertial = Inertial.from_geometry(
        Box((CLOCK_PLAQUE_SPAN, CLOCK_PLAQUE_THICK, CLOCK_PLAQUE_SPAN)),
        mass=4.0,
        origin=Origin(xyz=(0.0, CLOCK_PLAQUE_THICK * 0.5, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    hour_stem_mesh = _hand_blade_mesh(
        "station_clock_hour_stem.obj",
        [
            (-0.030, 0.056, HOUR_HAND_THICK),
            (0.045, 0.050, HOUR_HAND_THICK * 0.95),
            (0.150, 0.034, HOUR_HAND_THICK * 0.90),
        ],
    )
    hour_tip_mesh = _hand_blade_mesh(
        "station_clock_hour_tip.obj",
        [
            (0.120, 0.040, HOUR_HAND_THICK * 0.92),
            (0.200, 0.028, HOUR_HAND_THICK * 0.80),
            (0.270, 0.008, HOUR_HAND_THICK * 0.55),
        ],
    )
    hour_hand.visual(
        Box((0.10, HOUR_HAND_THICK, 0.05)),
        origin=Origin(xyz=(0.0, HOUR_HAND_THICK * 0.5, -0.040)),
        material=hand_black,
        name="hour_counterweight",
    )
    hour_hand.visual(
        hour_stem_mesh,
        origin=Origin(xyz=(0.0, HOUR_HAND_THICK * 0.5, 0.0)),
        material=hand_black,
        name="hour_stem",
    )
    hour_hand.visual(
        hour_tip_mesh,
        origin=Origin(xyz=(0.0, HOUR_HAND_THICK * 0.5, 0.0)),
        material=hand_black,
        name="hour_tip",
    )
    hour_hand.visual(
        Cylinder(radius=0.050, length=HOUR_HAND_THICK),
        origin=Origin(
            xyz=(0.0, HOUR_HAND_THICK * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hand_black,
        name="hour_hub",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.10, HOUR_HAND_THICK, 0.34)),
        mass=0.08,
        origin=Origin(xyz=(0.0, HOUR_HAND_THICK * 0.5, 0.080)),
    )

    minute_hand = model.part("minute_hand")
    minute_stem_mesh = _hand_blade_mesh(
        "station_clock_minute_stem.obj",
        [
            (-0.030, 0.044, MINUTE_HAND_THICK),
            (0.080, 0.036, MINUTE_HAND_THICK * 0.95),
            (0.210, 0.022, MINUTE_HAND_THICK * 0.82),
        ],
    )
    minute_tip_mesh = _hand_blade_mesh(
        "station_clock_minute_tip.obj",
        [
            (0.210, 0.024, MINUTE_HAND_THICK * 0.82),
            (0.300, 0.016, MINUTE_HAND_THICK * 0.72),
            (0.370, 0.006, MINUTE_HAND_THICK * 0.48),
        ],
    )
    minute_hand.visual(
        Box((0.08, MINUTE_HAND_THICK, 0.045)),
        origin=Origin(xyz=(0.0, MINUTE_HAND_THICK * 0.5, -0.045)),
        material=hand_black,
        name="minute_counterweight",
    )
    minute_hand.visual(
        minute_stem_mesh,
        origin=Origin(xyz=(0.0, MINUTE_HAND_THICK * 0.5, 0.0)),
        material=hand_black,
        name="minute_stem",
    )
    minute_hand.visual(
        minute_tip_mesh,
        origin=Origin(xyz=(0.0, MINUTE_HAND_THICK * 0.5, 0.0)),
        material=hand_black,
        name="minute_tip",
    )
    minute_hand.visual(
        Cylinder(radius=0.035, length=MINUTE_HAND_THICK),
        origin=Origin(
            xyz=(0.0, MINUTE_HAND_THICK * 0.5, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hand_black,
        name="minute_hub",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.08, MINUTE_HAND_THICK, 0.45)),
        mass=0.06,
        origin=Origin(xyz=(0.0, MINUTE_HAND_THICK * 0.5, 0.120)),
    )

    model.articulation(
        "tower_to_roof",
        ArticulationType.FIXED,
        parent=tower_body,
        child=roof_cap,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT)),
    )
    model.articulation(
        "tower_to_clock_face",
        ArticulationType.FIXED,
        parent=tower_body,
        child=clock_face,
        origin=Origin(xyz=(0.0, CLOCK_STAGE_SPAN * 0.5, CLOCK_CENTER_Z)),
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(xyz=(0.0, HOUR_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=1.5,
            lower=0.0,
            upper=math.tau,
        ),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(xyz=(0.0, MINUTE_JOINT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=0.0,
            upper=math.tau,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_body = object_model.get_part("tower_body")
    roof_cap = object_model.get_part("roof_cap")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")

    hour_joint = object_model.get_articulation("clock_face_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_face_to_minute_hand")

    clock_stage = tower_body.get_visual("clock_stage")
    cornice = tower_body.get_visual("cornice")
    roof_plinth = roof_cap.get_visual("roof_plinth")
    clock_plaque = clock_face.get_visual("clock_plaque")
    dial_disk = clock_face.get_visual("dial_disk")
    hub_boss = clock_face.get_visual("hub_boss")
    hour_tip = hour_hand.get_visual("hour_tip")
    hour_hub = hour_hand.get_visual("hour_hub")
    minute_tip = minute_hand.get_visual("minute_tip")
    minute_hub = minute_hand.get_visual("minute_hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        roof_cap,
        tower_body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=roof_plinth,
        negative_elem=cornice,
    )
    ctx.expect_overlap(
        roof_cap,
        tower_body,
        axes="xy",
        min_overlap=1.60,
        elem_a=roof_plinth,
        elem_b=cornice,
    )

    ctx.expect_gap(
        clock_face,
        tower_body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=clock_plaque,
        negative_elem=clock_stage,
    )
    ctx.expect_overlap(
        clock_face,
        tower_body,
        axes="xz",
        min_overlap=0.90,
        elem_a=clock_plaque,
        elem_b=clock_stage,
    )
    ctx.expect_within(
        clock_face,
        tower_body,
        axes="xz",
        inner_elem=clock_plaque,
        outer_elem=clock_stage,
    )

    ctx.expect_origin_distance(hour_hand, minute_hand, axes="xz", max_dist=0.001)
    ctx.expect_gap(
        hour_hand,
        clock_face,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hour_hub,
        negative_elem=hub_boss,
    )
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=minute_hub,
        negative_elem=hour_hub,
    )
    ctx.expect_within(
        hour_hand,
        clock_face,
        axes="xz",
        inner_elem=hour_tip,
        outer_elem=dial_disk,
        margin=0.010,
    )
    ctx.expect_within(
        minute_hand,
        clock_face,
        axes="xz",
        inner_elem=minute_tip,
        outer_elem=dial_disk,
        margin=0.010,
    )

    ctx.check(
        "hand_joint_axes",
        hour_joint.axis == (0.0, 1.0, 0.0) and minute_joint.axis == (0.0, 1.0, 0.0),
        details=f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}",
    )

    hour_limits = hour_joint.motion_limits
    minute_limits = minute_joint.motion_limits
    ctx.check(
        "hand_joint_full_turn_limits",
        hour_limits is not None
        and minute_limits is not None
        and hour_limits.lower == 0.0
        and minute_limits.lower == 0.0
        and math.isclose(hour_limits.upper or -1.0, math.tau)
        and math.isclose(minute_limits.upper or -1.0, math.tau),
        details=f"hour_limits={hour_limits}, minute_limits={minute_limits}",
    )

    tower_aabb = ctx.part_world_aabb(tower_body)
    roof_aabb = ctx.part_world_aabb(roof_cap)
    if tower_aabb is None or roof_aabb is None:
        ctx.fail("tower_aabb_available", "Could not measure tower or roof AABB.")
    else:
        overall_height = max(tower_aabb[1][2], roof_aabb[1][2]) - min(tower_aabb[0][2], roof_aabb[0][2])
        overall_width = max(tower_aabb[1][0], roof_aabb[1][0]) - min(tower_aabb[0][0], roof_aabb[0][0])
        ctx.check(
            "tower_height_realistic",
            6.0 <= overall_height <= 7.0,
            details=f"overall_height={overall_height:.3f}m",
        )
        ctx.check(
            "tower_plan_realistic",
            1.9 <= overall_width <= 2.2,
            details=f"overall_width={overall_width:.3f}m",
        )

    dial_aabb = ctx.part_element_world_aabb(clock_face, elem=dial_disk)
    if dial_aabb is None:
        ctx.fail("dial_aabb_available", "Could not measure the front dial.")
    else:
        dial_diameter = dial_aabb[1][0] - dial_aabb[0][0]
        ctx.check(
            "dial_scale_realistic",
            0.78 <= dial_diameter <= 0.82,
            details=f"dial_diameter={dial_diameter:.3f}m",
        )

    with ctx.pose({hour_joint: math.pi / 3.0, minute_joint: math.pi / 2.0}):
        ctx.fail_if_isolated_parts(name="hands_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="hands_pose_no_overlap")
        ctx.expect_gap(
            hour_hand,
            clock_face,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=hour_hub,
            negative_elem=hub_boss,
        )
        ctx.expect_gap(
            minute_hand,
            hour_hand,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=minute_hub,
            negative_elem=hour_hub,
        )
        ctx.expect_within(
            hour_hand,
            clock_face,
            axes="xz",
            inner_elem=hour_tip,
            outer_elem=dial_disk,
            margin=0.010,
        )
        ctx.expect_within(
            minute_hand,
            clock_face,
            axes="xz",
            inner_elem=minute_tip,
            outer_elem=dial_disk,
            margin=0.010,
        )
        hour_tip_aabb = ctx.part_element_world_aabb(hour_hand, elem=hour_tip)
        minute_tip_aabb = ctx.part_element_world_aabb(minute_hand, elem=minute_tip)
        if hour_tip_aabb is None or minute_tip_aabb is None:
            ctx.fail("posed_tip_aabbs_available", "Could not measure hand tips in articulated pose.")
        else:
            hour_center = _aabb_center(hour_tip_aabb)
            minute_center = _aabb_center(minute_tip_aabb)
            ctx.check(
                "hour_hand_sixty_degree_pose_reads",
                hour_center[0] > 0.12 and hour_center[2] > CLOCK_CENTER_Z + 0.06,
                details=f"hour_tip_center={hour_center}",
            )
            ctx.check(
                "minute_hand_quarter_turn_pose_reads",
                minute_center[0] > 0.25 and abs(minute_center[2] - CLOCK_CENTER_Z) < 0.08,
                details=f"minute_tip_center={minute_center}",
            )

    with ctx.pose({hour_joint: math.pi, minute_joint: math.pi * 1.5}):
        hour_tip_aabb = ctx.part_element_world_aabb(hour_hand, elem=hour_tip)
        minute_tip_aabb = ctx.part_element_world_aabb(minute_hand, elem=minute_tip)
        if hour_tip_aabb is None or minute_tip_aabb is None:
            ctx.fail("half_turn_tip_aabbs_available", "Could not measure hand tips at second pose.")
        else:
            hour_center = _aabb_center(hour_tip_aabb)
            minute_center = _aabb_center(minute_tip_aabb)
            ctx.check(
                "hour_hand_half_turn_pose_reads",
                hour_center[2] < CLOCK_CENTER_Z - 0.14,
                details=f"hour_tip_center={hour_center}",
            )
            ctx.check(
                "minute_hand_three_quarter_turn_pose_reads",
                minute_center[0] < -0.25 and abs(minute_center[2] - CLOCK_CENTER_Z) < 0.08,
                details=f"minute_tip_center={minute_center}",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
