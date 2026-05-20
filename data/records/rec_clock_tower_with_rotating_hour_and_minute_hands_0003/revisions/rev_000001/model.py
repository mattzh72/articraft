from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

SCALE = 16.0


def s(value: float) -> float:
    return value * SCALE


PLINTH_SIZE = s(0.56)
SHAFT_SIZE = s(0.38)
SHAFT_HEIGHT = s(0.96)
STAGE_SIZE = s(0.62)
STAGE_HEIGHT = s(0.31)
STAGE_BASE_SIZE = s(0.54)
STAGE_BASE_HEIGHT = s(0.08)
UPPER_CORNICE_SIZE = s(0.68)
UPPER_CORNICE_HEIGHT = s(0.07)
ROOF_BASE_SIZE = s(0.48)
ROOF_HEIGHT = s(0.30)
DIAL_RADIUS = s(0.152)
BEZEL_RADIUS = s(0.165)
WINDOW_HEIGHT = s(0.22)


def _square_section(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size * 0.5
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _dial_origin(side: str, outward: float) -> Origin:
    stage_half = STAGE_SIZE * 0.5
    dial_z = s(1.34)
    if side == "front":
        return Origin(xyz=(0.0, stage_half + outward, dial_z), rpy=(-math.pi / 2.0, 0.0, 0.0))
    if side == "rear":
        return Origin(xyz=(0.0, -stage_half - outward, dial_z), rpy=(math.pi / 2.0, 0.0, 0.0))
    if side == "right":
        return Origin(xyz=(stage_half + outward, 0.0, dial_z), rpy=(0.0, math.pi / 2.0, 0.0))
    return Origin(xyz=(-stage_half - outward, 0.0, dial_z), rpy=(0.0, -math.pi / 2.0, 0.0))


def _face_center(side: str) -> tuple[float, float, float]:
    pivot_offset = s(0.344)
    dial_z = s(1.34)
    if side == "front":
        return (0.0, pivot_offset, dial_z)
    if side == "rear":
        return (0.0, -pivot_offset, dial_z)
    if side == "right":
        return (pivot_offset, 0.0, dial_z)
    return (-pivot_offset, 0.0, dial_z)


def _default_hand_angle(side: str, kind: str) -> float:
    if kind == "minute":
        return math.radians(60.0 if side in {"front", "right"} else -60.0)
    return math.radians(-55.0 if side in {"front", "right"} else 55.0)


def _clock_axes(side: str) -> str:
    return "xz" if side in {"front", "rear"} else "yz"


def _add_dial(tower, side: str, stone_trim, clock_face, dark_trim, gold_trim) -> None:
    tower.visual(
        Cylinder(radius=BEZEL_RADIUS, length=s(0.022)),
        origin=_dial_origin(side, s(0.011)),
        material=dark_trim,
        name=f"{side}_bezel",
    )
    tower.visual(
        Cylinder(radius=DIAL_RADIUS, length=s(0.010)),
        origin=_dial_origin(side, s(0.021)),
        material=clock_face,
        name=f"{side}_dial_face",
    )
    tower.visual(
        Cylinder(radius=s(0.024), length=s(0.012)),
        origin=_dial_origin(side, s(0.028)),
        material=gold_trim,
        name=f"{side}_pivot_boss",
    )


def _hand_box_origin(side: str, normal_offset: float, center_distance: float, angle: float) -> Origin:
    if side in {"front", "rear"}:
        signed_offset = normal_offset if side == "front" else -normal_offset
        return Origin(
            xyz=(
                center_distance * math.sin(angle),
                signed_offset,
                center_distance * math.cos(angle),
            ),
            rpy=(0.0, angle, 0.0),
        )
    signed_offset = normal_offset if side == "right" else -normal_offset
    return Origin(
        xyz=(
            signed_offset,
            -center_distance * math.sin(angle),
            center_distance * math.cos(angle),
        ),
        rpy=(angle, 0.0, 0.0),
    )


def _hub_origin(side: str, offset: float) -> Origin:
    if side == "front":
        return Origin(xyz=(0.0, offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0))
    if side == "rear":
        return Origin(xyz=(0.0, -offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0))
    if side == "right":
        return Origin(xyz=(offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0))
    return Origin(xyz=(-offset, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0))


def _add_hand(model: ArticulatedObject, tower, side: str, kind: str, hand_material, hub_material):
    if kind == "hour":
        hub_radius = s(0.022)
        hub_thickness = s(0.004)
        arm_length = s(0.088)
        arm_width = s(0.020)
        tail_length = s(0.030)
        layer_start = 0.0
        hand_length = arm_length
        hand_center_z = arm_length * 0.5 - s(0.010)
    else:
        hub_radius = s(0.018)
        hub_thickness = s(0.003)
        arm_length = s(0.126)
        arm_width = s(0.014)
        tail_length = s(0.040)
        layer_start = s(0.004)
        hand_length = arm_length + tail_length
        hand_center_z = (arm_length - tail_length) * 0.5 - s(0.010)

    hand = model.part(f"{side}_{kind}_hand")
    thickness_center = layer_start + hub_thickness * 0.5
    box_size = (
        (arm_width, hub_thickness, hand_length)
        if side in {"front", "rear"}
        else (hub_thickness, arm_width, hand_length)
    )
    angle = _default_hand_angle(side, kind)

    hand.visual(
        Cylinder(radius=hub_radius, length=hub_thickness),
        origin=_hub_origin(side, thickness_center),
        material=hub_material,
        name="hub",
    )
    hand.visual(
        Box(box_size),
        origin=_hand_box_origin(side, thickness_center, hand_center_z, angle),
        material=hand_material,
        name="hand",
    )
    if kind == "hour":
        tail_size = (
            (arm_width * 0.7, hub_thickness, tail_length)
            if side in {"front", "rear"}
            else (hub_thickness, arm_width * 0.7, tail_length)
        )
        hand.visual(
            Box(tail_size),
            origin=_hand_box_origin(side, thickness_center, -tail_length * 0.5 - s(0.010), angle),
            material=hand_material,
            name="counterweight",
        )

    face_x, face_y, face_z = _face_center(side)
    axis = {
        "front": (0.0, 1.0, 0.0),
        "rear": (0.0, -1.0, 0.0),
        "right": (1.0, 0.0, 0.0),
        "left": (-1.0, 0.0, 0.0),
    }[side]
    model.articulation(
        f"tower_to_{side}_{kind}",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hand,
        origin=Origin(xyz=(face_x, face_y, face_z)),
        axis=axis,
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return hand


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="victorian_clock_tower", assets=ASSETS)

    weathered_stone = model.material("weathered_stone", rgba=(0.70, 0.68, 0.62, 1.0))
    light_stone = model.material("light_stone", rgba=(0.78, 0.76, 0.70, 1.0))
    slate = model.material("slate", rgba=(0.23, 0.25, 0.29, 1.0))
    clock_face = model.material("clock_face", rgba=(0.93, 0.91, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.09, 0.09, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.63, 0.27, 1.0))
    window_dark = model.material("window_dark", rgba=(0.16, 0.15, 0.16, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((PLINTH_SIZE, PLINTH_SIZE, s(0.12))),
        origin=Origin(xyz=(0.0, 0.0, s(0.06))),
        material=light_stone,
        name="plinth",
    )
    tower.visual(
        Box((s(0.46), s(0.46), s(0.06))),
        origin=Origin(xyz=(0.0, 0.0, s(0.15))),
        material=weathered_stone,
        name="base_course",
    )
    tower.visual(
        Box((SHAFT_SIZE, SHAFT_SIZE, SHAFT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, s(0.60))),
        material=weathered_stone,
        name="shaft",
    )
    tower.visual(
        Box((s(0.44), s(0.44), s(0.05))),
        origin=Origin(xyz=(0.0, 0.0, s(1.105))),
        material=light_stone,
        name="upper_string_course",
    )

    for side in ("front", "rear"):
        sign = 1.0 if side == "front" else -1.0
        tower.visual(
            Box((s(0.072), s(0.010), WINDOW_HEIGHT)),
            origin=Origin(xyz=(0.0, sign * (SHAFT_SIZE * 0.5 + s(0.005)), s(0.63))),
            material=window_dark,
            name=f"{side}_shaft_window",
        )
    for side in ("right", "left"):
        sign = 1.0 if side == "right" else -1.0
        tower.visual(
            Box((s(0.010), s(0.072), WINDOW_HEIGHT)),
            origin=Origin(xyz=(sign * (SHAFT_SIZE * 0.5 + s(0.005)), 0.0, s(0.63))),
            material=window_dark,
            name=f"{side}_shaft_window",
        )

    corbel_z = s(1.085)
    tower.visual(
        Box((s(0.08), s(0.20), s(0.09))),
        origin=Origin(xyz=(0.0, s(0.17), corbel_z)),
        material=light_stone,
        name="north_corbel",
    )
    tower.visual(
        Box((s(0.08), s(0.20), s(0.09))),
        origin=Origin(xyz=(0.0, -s(0.17), corbel_z)),
        material=light_stone,
        name="south_corbel",
    )
    tower.visual(
        Box((s(0.20), s(0.08), s(0.09))),
        origin=Origin(xyz=(s(0.17), 0.0, corbel_z)),
        material=light_stone,
        name="east_corbel",
    )
    tower.visual(
        Box((s(0.20), s(0.08), s(0.09))),
        origin=Origin(xyz=(-s(0.17), 0.0, corbel_z)),
        material=light_stone,
        name="west_corbel",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((s(0.09), s(0.09), s(0.08))),
                origin=Origin(xyz=(x_sign * s(0.15), y_sign * s(0.15), s(1.08))),
                material=light_stone,
                name=f"corner_corbel_{'e' if x_sign > 0 else 'w'}{'n' if y_sign > 0 else 's'}",
            )

    tower.visual(
        Box((STAGE_BASE_SIZE, STAGE_BASE_SIZE, STAGE_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, s(1.17))),
        material=light_stone,
        name="clock_stage_base",
    )
    tower.visual(
        Box((STAGE_SIZE, STAGE_SIZE, STAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, s(1.34))),
        material=weathered_stone,
        name="clock_stage",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((s(0.06), s(0.06), s(0.24))),
                origin=Origin(xyz=(x_sign * s(0.25), y_sign * s(0.25), s(1.34))),
                material=light_stone,
                name=f"stage_pilaster_{'e' if x_sign > 0 else 'w'}{'n' if y_sign > 0 else 's'}",
            )

    for side in ("front", "rear", "right", "left"):
        _add_dial(tower, side, light_stone, clock_face, dark_trim, brass)

    tower.visual(
        Box((UPPER_CORNICE_SIZE, UPPER_CORNICE_SIZE, UPPER_CORNICE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, s(1.505))),
        material=light_stone,
        name="upper_cornice",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower.visual(
                Box((s(0.08), s(0.08), s(0.10))),
                origin=Origin(xyz=(x_sign * s(0.25), y_sign * s(0.25), s(1.59))),
                material=light_stone,
                name=f"pinnacle_{'e' if x_sign > 0 else 'w'}{'n' if y_sign > 0 else 's'}",
            )
    roof_mesh = mesh_from_geometry(
        section_loft(
            [
                _square_section(ROOF_BASE_SIZE, 0.0),
                _square_section(ROOF_BASE_SIZE * 0.72, ROOF_HEIGHT * 0.45),
                _square_section(ROOF_BASE_SIZE * 0.24, ROOF_HEIGHT),
            ]
        ),
        ASSETS.mesh_path("victorian_clock_tower_roof.obj"),
    )
    tower.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, s(1.54))),
        material=slate,
        name="roof_spire",
    )
    tower.visual(
        Cylinder(radius=s(0.018), length=s(0.14)),
        origin=Origin(xyz=(0.0, 0.0, s(1.91))),
        material=brass,
        name="finial",
    )

    for side in ("front", "rear", "right", "left"):
        _add_hand(model, tower, side, "hour", dark_trim, brass)
        _add_hand(model, tower, side, "minute", dark_trim, brass)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower")
    plinth = tower.get_visual("plinth")
    shaft = tower.get_visual("shaft")
    clock_stage = tower.get_visual("clock_stage")
    upper_cornice = tower.get_visual("upper_cornice")
    roof_spire = tower.get_visual("roof_spire")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    plinth_aabb = ctx.part_element_world_aabb(tower, elem=plinth)
    shaft_aabb = ctx.part_element_world_aabb(tower, elem=shaft)
    clock_stage_aabb = ctx.part_element_world_aabb(tower, elem=clock_stage)
    upper_cornice_aabb = ctx.part_element_world_aabb(tower, elem=upper_cornice)
    roof_spire_aabb = ctx.part_element_world_aabb(tower, elem=roof_spire)

    if (
        plinth_aabb is None
        or shaft_aabb is None
        or clock_stage_aabb is None
        or upper_cornice_aabb is None
        or roof_spire_aabb is None
    ):
        ctx.fail("tower_visual_bounds_available", "Expected tower visual bounds were unavailable.")
        return ctx.report()

    plinth_width = plinth_aabb[1][0] - plinth_aabb[0][0]
    shaft_width = shaft_aabb[1][0] - shaft_aabb[0][0]
    clock_stage_width = clock_stage_aabb[1][0] - clock_stage_aabb[0][0]
    upper_cornice_width = upper_cornice_aabb[1][0] - upper_cornice_aabb[0][0]
    roof_width = roof_spire_aabb[1][0] - roof_spire_aabb[0][0]

    ctx.check(
        "plinth_broader_than_shaft",
        plinth_width > shaft_width + s(0.12),
        details=f"plinth_width={plinth_width:.3f}, shaft_width={shaft_width:.3f}",
    )
    ctx.check(
        "clock_stage_broader_than_shaft",
        clock_stage_width > shaft_width + s(0.20),
        details=f"clock_stage_width={clock_stage_width:.3f}, shaft_width={shaft_width:.3f}",
    )
    ctx.check(
        "roof_nests_within_cornice",
        roof_width < upper_cornice_width - s(0.12),
        details=f"roof_width={roof_width:.3f}, cornice_width={upper_cornice_width:.3f}",
    )

    ctx.expect_within(
        tower,
        tower,
        inner_elem=shaft,
        outer_elem=clock_stage,
        axes="xy",
        name="shaft_sits_within_clock_stage_footprint",
    )
    ctx.expect_within(
        tower,
        tower,
        inner_elem=clock_stage,
        outer_elem=upper_cornice,
        axes="xy",
        name="clock_stage_sits_within_cornice_footprint",
    )
    ctx.expect_within(
        tower,
        tower,
        inner_elem=roof_spire,
        outer_elem=upper_cornice,
        axes="xy",
        name="roof_spire_sits_over_cornice",
    )

    representative_angles = {
        "front": (math.radians(40.0), math.radians(-125.0)),
        "rear": (math.radians(-28.0), math.radians(140.0)),
        "right": (math.radians(55.0), math.radians(-105.0)),
        "left": (math.radians(-48.0), math.radians(118.0)),
    }
    expected_axes = {
        "front": (0.0, 1.0, 0.0),
        "rear": (0.0, -1.0, 0.0),
        "right": (1.0, 0.0, 0.0),
        "left": (-1.0, 0.0, 0.0),
    }

    for side in ("front", "rear", "right", "left"):
        hour = object_model.get_part(f"{side}_hour_hand")
        minute = object_model.get_part(f"{side}_minute_hand")
        hour_joint = object_model.get_articulation(f"tower_to_{side}_hour")
        minute_joint = object_model.get_articulation(f"tower_to_{side}_minute")
        dial = tower.get_visual(f"{side}_dial_face")
        boss = tower.get_visual(f"{side}_pivot_boss")
        hour_hub = hour.get_visual("hub")
        minute_hub = minute.get_visual("hub")
        hour_blade = hour.get_visual("hand")
        minute_blade = minute.get_visual("hand")
        axes = _clock_axes(side)

        ctx.check(
            f"{side}_hour_joint_axis",
            tuple(hour_joint.axis) == expected_axes[side],
            details=f"axis={hour_joint.axis}, expected={expected_axes[side]}",
        )
        ctx.check(
            f"{side}_minute_joint_axis",
            tuple(minute_joint.axis) == expected_axes[side],
            details=f"axis={minute_joint.axis}, expected={expected_axes[side]}",
        )

        ctx.expect_contact(
            hour,
            tower,
            elem_a=hour_hub,
            elem_b=boss,
            name=f"{side}_hour_hand_seated_on_boss",
        )
        ctx.expect_contact(
            minute,
            hour,
            elem_a=minute_hub,
            elem_b=hour_hub,
            name=f"{side}_minute_hand_stacked_on_hour_hub",
        )
        ctx.expect_within(
            hour,
            tower,
            inner_elem=hour_blade,
            outer_elem=dial,
            axes=axes,
            name=f"{side}_hour_hand_within_dial",
        )
        ctx.expect_within(
            minute,
            tower,
            inner_elem=minute_blade,
            outer_elem=dial,
            axes=axes,
            name=f"{side}_minute_hand_within_dial",
        )

        ctx.expect_within(
            tower,
            tower,
            inner_elem=dial,
            outer_elem=clock_stage,
            axes=axes,
            name=f"{side}_dial_within_clock_stage",
        )

        hour_limits = hour_joint.motion_limits
        minute_limits = minute_joint.motion_limits
        if hour_limits is not None and hour_limits.lower is not None and hour_limits.upper is not None:
            with ctx.pose({hour_joint: hour_limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{side}_hour_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{side}_hour_lower_no_floating")
            with ctx.pose({hour_joint: hour_limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{side}_hour_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{side}_hour_upper_no_floating")
        if minute_limits is not None and minute_limits.lower is not None and minute_limits.upper is not None:
            with ctx.pose({minute_joint: minute_limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{side}_minute_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{side}_minute_lower_no_floating")
            with ctx.pose({minute_joint: minute_limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{side}_minute_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{side}_minute_upper_no_floating")

        posed_hour, posed_minute = representative_angles[side]
        with ctx.pose({hour_joint: posed_hour, minute_joint: posed_minute}):
            ctx.expect_contact(
                hour,
                tower,
                elem_a=hour_hub,
                elem_b=boss,
                name=f"{side}_hour_hub_remains_seated_in_pose",
            )
            ctx.expect_contact(
                minute,
                hour,
                elem_a=minute_hub,
                elem_b=hour_hub,
                name=f"{side}_minute_hub_remains_stacked_in_pose",
            )
            ctx.expect_within(
                hour,
                tower,
                inner_elem=hour_blade,
                outer_elem=dial,
                axes=axes,
                name=f"{side}_hour_hand_stays_within_dial_in_pose",
            )
            ctx.expect_within(
                minute,
                tower,
                inner_elem=minute_blade,
                outer_elem=dial,
                axes=axes,
                name=f"{side}_minute_hand_stays_within_dial_in_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
