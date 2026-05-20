from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CASE_WIDTH = 0.182
CASE_DEPTH = 0.118
BOTTOM_THICKNESS = 0.003
WALL_THICKNESS = 0.003
DECK_THICKNESS = 0.002
DECK_TOP_Z = 0.027
DECK_CENTER_Z = DECK_TOP_Z - (DECK_THICKNESS / 2.0)

DECK_PLATE_WIDTH = 0.178
DECK_PLATE_DEPTH = 0.082
DECK_PLATE_CENTER_Y = -0.016

POD_WIDTH = 0.176
POD_DEPTH = 0.034
POD_HEIGHT = 0.011
POD_CENTER_Y = 0.041
POD_CENTER_Z = DECK_TOP_Z + (POD_HEIGHT / 2.0)
POD_TOP_Z = DECK_TOP_Z + POD_HEIGHT

KEY_COLUMNS = (-0.0345, -0.0115, 0.0115, 0.0345)
KEY_ROWS = (-0.032, -0.010, 0.012)
KEY_TRAVEL = 0.004
KEY_HOLE_SIZE = 0.016
KEY_STEM_SIZE = 0.013
KEY_STEM_HEIGHT = 0.008
KEY_STEM_CENTER_Z = 0.001
KEY_GUIDE_SIZE = 0.022
KEY_GUIDE_HEIGHT = 0.002
KEY_GUIDE_CENTER_Z = -0.003
KEY_CAP_SIZE = 0.018
KEY_CAP_HEIGHT = 0.007
KEY_CAP_CENTER_Z = 0.0085

ENCODER_POSITIONS = (
    ("encoder_left", (-0.054, 0.041)),
    ("encoder_center", (0.000, 0.041)),
    ("encoder_right", (0.054, 0.041)),
)


def _rect_profile(width: float, depth: float, *, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    hx = width / 2.0
    hy = depth / 2.0
    return [
        (cx - hx, cy - hy),
        (cx + hx, cy - hy),
        (cx + hx, cy + hy),
        (cx - hx, cy + hy),
    ]


def _key_name(row_idx: int, col_idx: int) -> str:
    return f"key_r{row_idx}_c{col_idx}"


def _all_key_names() -> list[str]:
    return [
        _key_name(row_idx, col_idx)
        for row_idx in range(1, len(KEY_ROWS) + 1)
        for col_idx in range(1, len(KEY_COLUMNS) + 1)
    ]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_control_keyboard")

    case_material = model.material("case_matte_black", rgba=(0.14, 0.15, 0.17, 1.0))
    key_material = model.material("key_dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    knob_material = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_material = model.material("indicator_light", rgba=(0.80, 0.82, 0.84, 1.0))

    case = model.part("case")

    wall_height = (DECK_TOP_Z - DECK_THICKNESS) - BOTTOM_THICKNESS
    wall_center_z = BOTTOM_THICKNESS + (wall_height / 2.0)

    case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS / 2.0)),
        material=case_material,
        name="elem_bottom",
    )
    case.visual(
        Box((WALL_THICKNESS, CASE_DEPTH, wall_height)),
        origin=Origin(xyz=((CASE_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), 0.0, wall_center_z)),
        material=case_material,
        name="elem_right_wall",
    )
    case.visual(
        Box((WALL_THICKNESS, CASE_DEPTH, wall_height)),
        origin=Origin(xyz=(-(CASE_WIDTH / 2.0) + (WALL_THICKNESS / 2.0), 0.0, wall_center_z)),
        material=case_material,
        name="elem_left_wall",
    )
    case.visual(
        Box((CASE_WIDTH, WALL_THICKNESS, wall_height)),
        origin=Origin(xyz=(0.0, (-(CASE_DEPTH / 2.0)) + (WALL_THICKNESS / 2.0), wall_center_z)),
        material=case_material,
        name="elem_front_wall",
    )
    case.visual(
        Box((CASE_WIDTH, WALL_THICKNESS, wall_height)),
        origin=Origin(xyz=(0.0, ((CASE_DEPTH / 2.0) - (WALL_THICKNESS / 2.0)), wall_center_z)),
        material=case_material,
        name="elem_rear_wall",
    )

    deck_holes: list[list[tuple[float, float]]] = []
    for key_y in KEY_ROWS:
        for key_x in KEY_COLUMNS:
            deck_holes.append(
                _rect_profile(
                    KEY_HOLE_SIZE,
                    KEY_HOLE_SIZE,
                    center=(key_x, key_y - DECK_PLATE_CENTER_Y),
                )
            )

    deck_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(DECK_PLATE_WIDTH, DECK_PLATE_DEPTH),
            deck_holes,
            DECK_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        "compact_keyboard_deck_plate",
    )
    case.visual(
        deck_plate_mesh,
        origin=Origin(xyz=(0.0, DECK_PLATE_CENTER_Y, DECK_CENTER_Z)),
        material=case_material,
        name="elem_deck_plate",
    )

    case.visual(
        Box((POD_WIDTH, POD_DEPTH, POD_HEIGHT)),
        origin=Origin(xyz=(0.0, POD_CENTER_Y, POD_CENTER_Z)),
        material=case_material,
        name="elem_pod",
    )

    for row_idx, key_y in enumerate(KEY_ROWS, start=1):
        for col_idx, key_x in enumerate(KEY_COLUMNS, start=1):
            key_name = _key_name(row_idx, col_idx)
            key = model.part(key_name)
            key.visual(
                Box((KEY_STEM_SIZE, KEY_STEM_SIZE, KEY_STEM_HEIGHT)),
                origin=Origin(xyz=(0.0, 0.0, KEY_STEM_CENTER_Z)),
                material=key_material,
                name="stem",
            )
            key.visual(
                Box((KEY_GUIDE_SIZE, KEY_GUIDE_SIZE, KEY_GUIDE_HEIGHT)),
                origin=Origin(xyz=(0.0, 0.0, KEY_GUIDE_CENTER_Z)),
                material=key_material,
                name="guide",
            )
            key.visual(
                Box((KEY_CAP_SIZE, KEY_CAP_SIZE, KEY_CAP_HEIGHT)),
                origin=Origin(xyz=(0.0, 0.0, KEY_CAP_CENTER_Z)),
                material=key_material,
                name="cap",
            )
            model.articulation(
                f"case_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key,
                origin=Origin(xyz=(key_x, key_y, DECK_TOP_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=12.0,
                    velocity=0.20,
                    lower=0.0,
                    upper=KEY_TRAVEL,
                ),
            )

    for encoder_name, (encoder_x, encoder_y) in ENCODER_POSITIONS:
        encoder = model.part(encoder_name)
        encoder.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=knob_material,
            name="shaft",
        )
        encoder.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=knob_material,
            name="knob",
        )
        encoder.visual(
            Box((0.003, 0.009, 0.002)),
            origin=Origin(xyz=(0.0, 0.009, 0.021)),
            material=accent_material,
            name="marker",
        )
        model.articulation(
            f"case_to_{encoder_name}",
            ArticulationType.CONTINUOUS,
            parent=case,
            child=encoder,
            origin=Origin(xyz=(encoder_x, encoder_y, POD_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=8.0,
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

    case = object_model.get_part("case")

    for key_name in _all_key_names():
        key = object_model.get_part(key_name)
        key_joint = object_model.get_articulation(f"case_to_{key_name}")
        ctx.check(
            f"{key_name} uses a prismatic plunger joint",
            key_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(key_joint.axis) == (0.0, 0.0, -1.0),
            details=f"type={key_joint.articulation_type}, axis={key_joint.axis}",
        )
        ctx.expect_within(
            key,
            case,
            axes="xy",
            margin=0.001,
            name=f"{key_name} stays over the keyboard footprint",
        )
        ctx.expect_contact(
            key,
            case,
            elem_a="guide",
            elem_b="elem_deck_plate",
            name=f"{key_name} guide rides on the deck plate underside",
        )
        ctx.expect_gap(
            key,
            case,
            axis="z",
            positive_elem="cap",
            negative_elem="elem_deck_plate",
            min_gap=0.004,
            max_gap=0.0065,
            name=f"{key_name} sits just above the deck plate",
        )

    for encoder_name, _ in ENCODER_POSITIONS:
        encoder = object_model.get_part(encoder_name)
        encoder_joint = object_model.get_articulation(f"case_to_{encoder_name}")
        ctx.check(
            f"{encoder_name} uses a continuous rotary joint",
            encoder_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(encoder_joint.axis) == (0.0, 0.0, 1.0),
            details=f"type={encoder_joint.articulation_type}, axis={encoder_joint.axis}",
        )
        ctx.expect_within(
            encoder,
            case,
            axes="xy",
            margin=0.002,
            name=f"{encoder_name} stays within the pod span",
        )
        ctx.expect_contact(
            encoder,
            case,
            elem_a="shaft",
            elem_b="elem_pod",
            name=f"{encoder_name} shaft seats on the top pod",
        )

    center_key = object_model.get_part("key_r2_c2")
    center_key_joint = object_model.get_articulation("case_to_key_r2_c2")
    rest_key_pos = ctx.part_world_position(center_key)
    with ctx.pose({center_key_joint: KEY_TRAVEL}):
        pressed_key_pos = ctx.part_world_position(center_key)
        ctx.expect_gap(
            center_key,
            case,
            axis="z",
            positive_elem="cap",
            negative_elem="elem_deck_plate",
            min_gap=0.0005,
            max_gap=0.0025,
            name="center key can depress nearly to the deck",
        )
    ctx.check(
        "center key moves downward when pressed",
        rest_key_pos is not None and pressed_key_pos is not None and pressed_key_pos[2] < rest_key_pos[2] - 0.003,
        details=f"rest={rest_key_pos}, pressed={pressed_key_pos}",
    )

    left_encoder = object_model.get_part("encoder_left")
    left_encoder_joint = object_model.get_articulation("case_to_encoder_left")
    rest_marker_center = _aabb_center(ctx.part_element_world_aabb(left_encoder, elem="marker"))
    with ctx.pose({left_encoder_joint: 1.0}):
        turned_marker_center = _aabb_center(ctx.part_element_world_aabb(left_encoder, elem="marker"))
    ctx.check(
        "left encoder rotates visibly about its shaft",
        rest_marker_center is not None
        and turned_marker_center is not None
        and turned_marker_center[0] < rest_marker_center[0] - 0.005,
        details=f"rest_marker_center={rest_marker_center}, turned_marker_center={turned_marker_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
