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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_WIDTH = 0.324
CASE_DEPTH = 0.132
CASE_HEIGHT = 0.021
BOTTOM_THICKNESS = 0.003
WALL_THICKNESS = 0.004
CORNER_RADIUS = 0.006
TOP_PLATE_THICKNESS = 0.0025
TOP_PLATE_TOP_Z = CASE_HEIGHT
TOP_PLATE_CENTER_Z = TOP_PLATE_TOP_Z - TOP_PLATE_THICKNESS * 0.5
KEY_TRAVEL = 0.0018
KEY_TOP_HEIGHT = 0.0034
KEY_SKIRT_HEIGHT = 0.0032
KEY_STEM_HEIGHT = 0.0048


def _rect_profile(width: float, depth: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (cx - half_w, cy - half_d),
        (cx + half_w, cy - half_d),
        (cx + half_w, cy + half_d),
        (cx - half_w, cy + half_d),
    ]


def _build_key_specs() -> list[dict[str, object]]:
    specs: list[dict[str, object]] = []

    row_positions = [0.041, 0.013, -0.015, -0.043]

    for row_index, y in enumerate(row_positions, start=1):
        specs.append(
            {
                "name": f"macro_key_{row_index}",
                "x": -0.128,
                "y": y,
                "cap_w": 0.023,
                "cap_d": 0.023,
                "material": "macro",
            }
        )

    main_columns = [-0.095, -0.067, -0.039, -0.011, 0.017, 0.045]
    for row_index, y in enumerate(row_positions, start=1):
        for column_index, x in enumerate(main_columns, start=1):
            specs.append(
                {
                    "name": f"main_key_r{row_index}c{column_index}",
                    "x": x,
                    "y": y,
                    "cap_w": 0.024,
                    "cap_d": 0.022,
                    "material": "main",
                }
            )

    specs.extend(
        [
            {"name": "nav_key_top", "x": 0.078, "y": 0.041, "cap_w": 0.024, "cap_d": 0.022, "material": "accent"},
            {"name": "nav_key_mid", "x": 0.078, "y": 0.013, "cap_w": 0.024, "cap_d": 0.022, "material": "accent"},
            {"name": "arrow_up", "x": 0.108, "y": -0.015, "cap_w": 0.024, "cap_d": 0.022, "material": "accent"},
            {"name": "arrow_left", "x": 0.080, "y": -0.043, "cap_w": 0.024, "cap_d": 0.022, "material": "accent"},
            {"name": "arrow_down", "x": 0.108, "y": -0.043, "cap_w": 0.024, "cap_d": 0.022, "material": "accent"},
            {"name": "arrow_right", "x": 0.136, "y": -0.043, "cap_w": 0.024, "cap_d": 0.022, "material": "accent"},
        ]
    )

    return specs


def _build_top_plate_mesh(key_specs: list[dict[str, object]]):
    outer_profile = rounded_rect_profile(
        CASE_WIDTH - 2.0 * WALL_THICKNESS,
        CASE_DEPTH - 2.0 * WALL_THICKNESS,
        radius=0.005,
        corner_segments=8,
    )

    hole_profiles: list[list[tuple[float, float]]] = []
    for spec in key_specs:
        cap_w = float(spec["cap_w"])
        cap_d = float(spec["cap_d"])
        x = float(spec["x"])
        y = float(spec["y"])
        skirt_w = max(0.016, cap_w - 0.0032)
        skirt_d = max(0.014, cap_d - 0.0032)
        hole_profiles.append(_rect_profile(skirt_w, skirt_d, cx=x, cy=y))

    plate_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=TOP_PLATE_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(plate_geom, "compact_keyboard_top_plate")


def _add_key(
    model: ArticulatedObject,
    case_part,
    spec: dict[str, object],
    materials: dict[str, object],
) -> None:
    name = str(spec["name"])
    cap_w = float(spec["cap_w"])
    cap_d = float(spec["cap_d"])
    x = float(spec["x"])
    y = float(spec["y"])
    material_key = str(spec["material"])

    skirt_w = max(0.016, cap_w - 0.0032)
    skirt_d = max(0.014, cap_d - 0.0032)

    key_part = model.part(name)
    key_part.visual(
        Box((cap_w, cap_d, KEY_TOP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.0037)),
        material=materials[material_key],
        name="cap",
    )
    key_part.visual(
        Box((skirt_w, skirt_d, KEY_SKIRT_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=materials[material_key],
        name="skirt",
    )
    key_part.visual(
        Box((0.010, 0.010, KEY_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.0029)),
        material=materials["stem"],
        name="stem",
    )
    key_part.inertial = Inertial.from_geometry(
        Box((cap_w, cap_d, 0.009)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.0023)),
    )

    model.articulation(
        f"case_to_{name}",
        ArticulationType.PRISMATIC,
        parent=case_part,
        child=key_part,
        origin=Origin(xyz=(x, y, TOP_PLATE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.04,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def _add_rear_foot(
    model: ArticulatedObject,
    case_part,
    *,
    name: str,
    x: float,
    foot_material,
    pad_material,
) -> None:
    foot_part = model.part(name)
    foot_part.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(
            xyz=(0.0, -0.0035, -0.0008),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=foot_material,
        name="foot_knuckle",
    )
    foot_part.visual(
        Box((0.012, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.008, -0.0016)),
        material=foot_material,
        name="foot_link",
    )
    foot_part.visual(
        Box((0.042, 0.030, 0.0030)),
        origin=Origin(xyz=(0.0, -0.015, -0.0018)),
        material=foot_material,
        name="foot_arm",
    )
    foot_part.visual(
        Box((0.034, 0.010, 0.0024)),
        origin=Origin(xyz=(0.0, -0.028, -0.0042)),
        material=pad_material,
        name="foot_pad",
    )
    foot_part.inertial = Inertial.from_geometry(
        Box((0.042, 0.030, 0.006)),
        mass=0.018,
        origin=Origin(xyz=(0.0, -0.016, -0.0028)),
    )

    model.articulation(
        f"case_to_{name}",
        ArticulationType.REVOLUTE,
        parent=case_part,
        child=foot_part,
        origin=Origin(xyz=(x, 0.058, 0.0003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_programming_keyboard")

    case_dark = model.material("case_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    case_black = model.material("case_black", rgba=(0.10, 0.11, 0.12, 1.0))
    key_graphite = model.material("key_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    macro_blue = model.material("macro_blue", rgba=(0.20, 0.44, 0.60, 1.0))
    accent_key = model.material("accent_key", rgba=(0.36, 0.39, 0.44, 1.0))
    stem_dark = model.material("stem_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.16, 0.16, 0.17, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    materials = {
        "main": key_graphite,
        "macro": macro_blue,
        "accent": accent_key,
        "stem": stem_dark,
    }

    key_specs = _build_key_specs()
    top_plate_mesh = _build_top_plate_mesh(key_specs)

    case_part = model.part("case")
    case_part.visual(
        Box((CASE_WIDTH - 0.012, CASE_DEPTH - 0.012, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=case_dark,
        name="bottom_panel",
    )
    case_part.visual(
        Box((CASE_WIDTH - 2.0 * CORNER_RADIUS, WALL_THICKNESS, CASE_HEIGHT)),
        origin=Origin(xyz=(0.0, -(CASE_DEPTH * 0.5 - WALL_THICKNESS * 0.5), CASE_HEIGHT * 0.5)),
        material=case_dark,
        name="front_wall",
    )
    case_part.visual(
        Box((CASE_WIDTH - 2.0 * CORNER_RADIUS, WALL_THICKNESS, CASE_HEIGHT)),
        origin=Origin(xyz=(0.0, CASE_DEPTH * 0.5 - WALL_THICKNESS * 0.5, CASE_HEIGHT * 0.5)),
        material=case_dark,
        name="rear_wall",
    )
    case_part.visual(
        Box((WALL_THICKNESS, CASE_DEPTH - 2.0 * CORNER_RADIUS, CASE_HEIGHT)),
        origin=Origin(xyz=(-(CASE_WIDTH * 0.5 - WALL_THICKNESS * 0.5), 0.0, CASE_HEIGHT * 0.5)),
        material=case_dark,
        name="left_wall",
    )
    case_part.visual(
        Box((WALL_THICKNESS, CASE_DEPTH - 2.0 * CORNER_RADIUS, CASE_HEIGHT)),
        origin=Origin(xyz=(CASE_WIDTH * 0.5 - WALL_THICKNESS * 0.5, 0.0, CASE_HEIGHT * 0.5)),
        material=case_dark,
        name="right_wall",
    )

    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            corner_name = f"corner_{'r' if sign_x > 0 else 'l'}_{'rear' if sign_y > 0 else 'front'}"
            case_part.visual(
                Cylinder(radius=CORNER_RADIUS, length=CASE_HEIGHT),
                origin=Origin(
                    xyz=(
                        sign_x * (CASE_WIDTH * 0.5 - CORNER_RADIUS),
                        sign_y * (CASE_DEPTH * 0.5 - CORNER_RADIUS),
                        CASE_HEIGHT * 0.5,
                    )
                ),
                material=case_dark,
                name=corner_name,
            )

    case_part.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_CENTER_Z)),
        material=case_black,
        name="top_plate",
    )
    case_part.visual(
        Cylinder(radius=0.010, length=0.0030),
        origin=Origin(xyz=(0.112, 0.037, TOP_PLATE_TOP_Z - 0.0015)),
        material=case_black,
        name="knob_collar",
    )
    case_part.visual(
        Cylinder(radius=0.0045, length=0.050),
        origin=Origin(
            xyz=(-0.096, 0.058, 0.0046),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=case_dark,
        name="left_hinge_barrel",
    )
    case_part.visual(
        Cylinder(radius=0.0045, length=0.050),
        origin=Origin(
            xyz=(0.096, 0.058, 0.0046),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=case_dark,
        name="right_hinge_barrel",
    )
    case_part.visual(
        Box((0.020, 0.010, 0.0016)),
        origin=Origin(xyz=(-0.090, -0.042, 0.0008)),
        material=rubber,
        name="front_pad_left",
    )
    case_part.visual(
        Box((0.020, 0.010, 0.0016)),
        origin=Origin(xyz=(0.090, -0.042, 0.0008)),
        material=rubber,
        name="front_pad_right",
    )
    case_part.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT * 0.5)),
    )

    for spec in key_specs:
        _add_key(model, case_part, spec, materials)

    knob_part = model.part("rotary_knob")
    knob_part.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=knob_dark,
        name="knob_body",
    )
    knob_part.visual(
        Cylinder(radius=0.0145, length=0.0022),
        origin=Origin(xyz=(0.0, 0.0, 0.0151)),
        material=knob_cap,
        name="knob_cap",
    )
    knob_part.visual(
        Box((0.0020, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, 0.010, 0.0160)),
        material=knob_cap,
        name="indicator_line",
    )
    knob_part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.014),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    model.articulation(
        "case_to_rotary_knob",
        ArticulationType.CONTINUOUS,
        parent=case_part,
        child=knob_part,
        origin=Origin(xyz=(0.112, 0.037, TOP_PLATE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=8.0,
        ),
    )

    _add_rear_foot(
        model,
        case_part,
        name="left_rear_foot",
        x=-0.096,
        foot_material=case_dark,
        pad_material=rubber,
    )
    _add_rear_foot(
        model,
        case_part,
        name="right_rear_foot",
        x=0.096,
        foot_material=case_dark,
        pad_material=rubber,
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

    case_part = object_model.get_part("case")
    macro_key = object_model.get_part("macro_key_1")
    main_key = object_model.get_part("main_key_r2c3")
    arrow_up = object_model.get_part("arrow_up")
    rotary_knob = object_model.get_part("rotary_knob")
    left_foot = object_model.get_part("left_rear_foot")
    right_foot = object_model.get_part("right_rear_foot")

    macro_slide = object_model.get_articulation("case_to_macro_key_1")
    main_slide = object_model.get_articulation("case_to_main_key_r2c3")
    knob_joint = object_model.get_articulation("case_to_rotary_knob")
    left_hinge = object_model.get_articulation("case_to_left_rear_foot")
    right_hinge = object_model.get_articulation("case_to_right_rear_foot")

    macro_pos = ctx.part_world_position(macro_key)
    main_pos = ctx.part_world_position(main_key)
    arrow_pos = ctx.part_world_position(arrow_up)
    knob_pos = ctx.part_world_position(rotary_knob)

    ctx.check(
        "macro column sits left of main deck",
        macro_pos is not None and main_pos is not None and macro_pos[0] < main_pos[0] - 0.05,
        details=f"macro={macro_pos}, main={main_pos}",
    )
    ctx.check(
        "knob sits above arrow cluster",
        knob_pos is not None
        and arrow_pos is not None
        and knob_pos[1] > arrow_pos[1] + 0.04
        and knob_pos[0] > arrow_pos[0] - 0.01,
        details=f"knob={knob_pos}, arrow_up={arrow_pos}",
    )
    ctx.check(
        "knob uses vertical continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )
    ctx.expect_gap(
        macro_key,
        case_part,
        axis="z",
        positive_elem="cap",
        negative_elem="top_plate",
        min_gap=0.0018,
        max_gap=0.0022,
        name="macro key cap hovers just above top plate",
    )
    ctx.expect_overlap(
        macro_key,
        case_part,
        axes="xy",
        elem_a="cap",
        elem_b="top_plate",
        min_overlap=0.018,
        name="macro key remains seated over the keyboard deck",
    )

    main_rest = ctx.part_world_position(main_key)
    with ctx.pose({main_slide: KEY_TRAVEL}):
        main_pressed = ctx.part_world_position(main_key)
        ctx.expect_gap(
            main_key,
            case_part,
            axis="z",
            positive_elem="cap",
            negative_elem="top_plate",
            min_gap=0.0001,
            max_gap=0.0005,
            name="pressed key nearly bottoms out on the top plate",
        )
    ctx.check(
        "main key plunges downward",
        main_rest is not None and main_pressed is not None and main_pressed[2] < main_rest[2] - 0.0015,
        details=f"rest={main_rest}, pressed={main_pressed}",
    )

    left_rest_aabb = ctx.part_world_aabb(left_foot)
    right_rest_aabb = ctx.part_world_aabb(right_foot)
    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0}):
        left_open_aabb = ctx.part_world_aabb(left_foot)
        right_open_aabb = ctx.part_world_aabb(right_foot)
        ctx.expect_gap(
            case_part,
            left_foot,
            axis="z",
            negative_elem="foot_pad",
            min_gap=0.010,
            name="left rear foot swings below the case when opened",
        )
        ctx.expect_gap(
            case_part,
            right_foot,
            axis="z",
            negative_elem="foot_pad",
            min_gap=0.010,
            name="right rear foot swings below the case when opened",
        )
    ctx.check(
        "left rear foot folds downward",
        left_rest_aabb is not None
        and left_open_aabb is not None
        and left_open_aabb[0][2] < left_rest_aabb[0][2] - 0.010,
        details=f"rest={left_rest_aabb}, open={left_open_aabb}",
    )
    ctx.check(
        "right rear foot folds downward",
        right_rest_aabb is not None
        and right_open_aabb is not None
        and right_open_aabb[0][2] < right_rest_aabb[0][2] - 0.010,
        details=f"rest={right_rest_aabb}, open={right_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
