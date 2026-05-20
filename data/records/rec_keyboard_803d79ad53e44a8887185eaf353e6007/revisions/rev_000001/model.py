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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


HOUSING_WIDTH = 0.405
HOUSING_DEPTH = 0.175
FLOOR_THICKNESS = 0.004
WALL_THICKNESS = 0.005
WALL_HEIGHT = 0.017
DECK_THICKNESS = 0.003
DECK_TOP_Z = 0.020
DECK_CENTER_Z = DECK_TOP_Z - (DECK_THICKNESS * 0.5)

TRACKBALL_CENTER = (0.145, -0.002, 0.0265)
TRACKBALL_RADIUS = 0.022
TRACKBALL_OPENING_RADIUS = math.sqrt(
    (TRACKBALL_RADIUS * TRACKBALL_RADIUS) - ((TRACKBALL_CENTER[2] - DECK_TOP_Z) ** 2)
)
TRACKBALL_RING_OUTER_RADIUS = 0.035
TRACKBALL_RING_INNER_RADIUS = 0.024

JOG_WHEEL_CENTER = (0.089, 0.064, 0.037)
JOG_WHEEL_RADIUS = 0.015
JOG_WHEEL_LENGTH = 0.010

KEY_TRAVEL = 0.0025
KEY_REST_GAP = 0.0


def rect_profile(width: float, depth: float, center: tuple[float, float]) -> list[tuple[float, float]]:
    cx, cy = center
    hx = width * 0.5
    hy = depth * 0.5
    return [
        (cx - hx, cy - hy),
        (cx + hx, cy - hy),
        (cx + hx, cy + hy),
        (cx - hx, cy + hy),
    ]


def circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + (radius * math.cos((2.0 * math.pi * i) / segments)),
            cy + (radius * math.sin((2.0 * math.pi * i) / segments)),
        )
        for i in range(segments)
    ]


def make_key_specs() -> list[dict[str, object]]:
    specs: list[dict[str, object]] = []

    function_y = 0.051
    for index, x in enumerate((-0.130, -0.100, -0.070, -0.040, -0.010), start=1):
        specs.append(
            {
                "name": f"fkey_{index}",
                "center": (x, function_y),
                "cap_size": (0.016, 0.012),
                "skirt_size": (0.0128, 0.0092),
                "hole_size": (0.0128, 0.0092),
            }
        )

    main_x_positions = (-0.128, -0.106, -0.084, -0.062, -0.040, -0.018, 0.004)
    main_y_positions = (0.026, 0.004, -0.018, -0.040)
    for row_index, y in enumerate(main_y_positions, start=1):
        for col_index, x in enumerate(main_x_positions, start=1):
            specs.append(
                {
                    "name": f"key_r{row_index}_c{col_index}",
                    "center": (x, y),
                    "cap_size": (0.0175, 0.0175),
                    "skirt_size": (0.0148, 0.0148),
                    "hole_size": (0.0148, 0.0148),
                }
            )

    for index, y in enumerate((0.004, -0.018, -0.040), start=1):
        specs.append(
            {
                "name": f"media_{index}",
                "center": (0.036, y),
                "cap_size": (0.0165, 0.0165),
                "skirt_size": (0.0138, 0.0138),
                "hole_size": (0.0138, 0.0138),
            }
        )

    return specs


def add_key(model: ArticulatedObject, housing, key_material, stem_material, spec: dict[str, object]) -> None:
    name = str(spec["name"])
    center_x, center_y = spec["center"]
    cap_x, cap_y = spec["cap_size"]
    skirt_x, skirt_y = spec["skirt_size"]

    key = model.part(name)
    key.visual(
        Box((cap_x, cap_y, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=key_material,
        name="keycap",
    )
    key.visual(
        Box((skirt_x, skirt_y, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=key_material,
        name="key_skirt",
    )
    key.visual(
        Box((min(skirt_x - 0.0022, 0.0115), min(skirt_y - 0.0022, 0.0115), 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=stem_material,
        name="key_stem",
    )

    model.articulation(
        f"housing_to_{name}",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=key,
        origin=Origin(xyz=(center_x, center_y, DECK_TOP_Z + KEY_REST_GAP)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=KEY_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="media_keyboard_trackball")

    housing_material = model.material("housing", rgba=(0.16, 0.17, 0.19, 1.0))
    deck_material = model.material("deck", rgba=(0.20, 0.22, 0.24, 1.0))
    trim_material = model.material("trim", rgba=(0.12, 0.13, 0.15, 1.0))
    key_material = model.material("keycap", rgba=(0.13, 0.13, 0.14, 1.0))
    stem_material = model.material("switch_stem", rgba=(0.28, 0.29, 0.30, 1.0))
    trackball_material = model.material("trackball", rgba=(0.18, 0.43, 0.78, 1.0))
    wheel_tire_material = model.material("wheel_tire", rgba=(0.08, 0.08, 0.09, 1.0))
    wheel_hub_material = model.material("wheel_hub", rgba=(0.62, 0.64, 0.67, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS * 0.5)),
        material=housing_material,
        name="tray_floor",
    )
    housing.visual(
        Box((WALL_THICKNESS, HOUSING_DEPTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -((HOUSING_WIDTH * 0.5) - (WALL_THICKNESS * 0.5)),
                0.0,
                WALL_HEIGHT * 0.5,
            )
        ),
        material=housing_material,
        name="left_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, HOUSING_DEPTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                (HOUSING_WIDTH * 0.5) - (WALL_THICKNESS * 0.5),
                0.0,
                WALL_HEIGHT * 0.5,
            )
        ),
        material=housing_material,
        name="right_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -((HOUSING_DEPTH * 0.5) - (WALL_THICKNESS * 0.5)),
                WALL_HEIGHT * 0.5,
            )
        ),
        material=housing_material,
        name="front_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (HOUSING_DEPTH * 0.5) - (WALL_THICKNESS * 0.5),
                WALL_HEIGHT * 0.5,
            )
        ),
        material=housing_material,
        name="rear_wall",
    )

    top_deck_holes = [
        rect_profile(spec["hole_size"][0], spec["hole_size"][1], spec["center"])
        for spec in make_key_specs()
    ]
    top_deck_holes.append(circle_profile(TRACKBALL_OPENING_RADIUS, center=TRACKBALL_CENTER[:2], segments=96))
    top_deck_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(
                HOUSING_WIDTH - (2.0 * WALL_THICKNESS),
                HOUSING_DEPTH - (2.0 * WALL_THICKNESS),
                0.012,
                corner_segments=10,
            ),
            top_deck_holes,
            DECK_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        "keyboard_top_deck",
    )
    housing.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_material,
        name="top_deck",
    )

    rear_top_strip_width = 0.118
    housing.visual(
        Box((HOUSING_WIDTH - 0.040, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0765, 0.024)),
        material=trim_material,
        name="rear_top_strip",
    )

    trackball_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            circle_profile(TRACKBALL_RING_OUTER_RADIUS, segments=48),
            [circle_profile(TRACKBALL_RING_INNER_RADIUS, segments=48)],
            0.004,
            cap=True,
            center=True,
            closed=True,
        ),
        "trackball_ring",
    )
    housing.visual(
        trackball_ring_mesh,
        origin=Origin(xyz=(TRACKBALL_CENTER[0], TRACKBALL_CENTER[1], 0.022)),
        material=trim_material,
        name="trackball_ring",
    )
    housing.visual(
        Sphere(0.003),
        origin=Origin(
            xyz=(
                TRACKBALL_CENTER[0],
                TRACKBALL_CENTER[1],
                TRACKBALL_CENTER[2] - TRACKBALL_RADIUS - 0.003,
            )
        ),
        material=trim_material,
        name="trackball_bearing",
    )

    housing.visual(
        Box((0.048, 0.006, 0.018)),
        origin=Origin(xyz=(JOG_WHEEL_CENTER[0], 0.053, 0.029)),
        material=trim_material,
        name="wheel_front_cheek",
    )
    housing.visual(
        Box((0.048, 0.006, 0.018)),
        origin=Origin(xyz=(JOG_WHEEL_CENTER[0], 0.075, 0.029)),
        material=trim_material,
        name="wheel_rear_cheek",
    )
    housing.visual(
        Box((0.010, 0.022, 0.008)),
        origin=Origin(xyz=(0.061, JOG_WHEEL_CENTER[1], 0.024)),
        material=trim_material,
        name="wheel_left_shoulder",
    )
    housing.visual(
        Box((0.010, 0.022, 0.008)),
        origin=Origin(xyz=(0.117, JOG_WHEEL_CENTER[1], 0.024)),
        material=trim_material,
        name="wheel_right_shoulder",
    )

    for spec in make_key_specs():
        add_key(model, housing, key_material, stem_material, spec)

    trackball = model.part("trackball")
    trackball.visual(
        Sphere(TRACKBALL_RADIUS),
        material=trackball_material,
        name="ball",
    )
    model.articulation(
        "housing_to_trackball",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=trackball,
        origin=Origin(xyz=TRACKBALL_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=12.0),
    )

    jog_wheel = model.part("jog_wheel")
    jog_wheel.visual(
        Cylinder(radius=JOG_WHEEL_RADIUS, length=JOG_WHEEL_LENGTH),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=wheel_tire_material,
        name="wheel_tire",
    )
    jog_wheel.visual(
        Cylinder(radius=0.009, length=JOG_WHEEL_LENGTH + 0.001),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=wheel_hub_material,
        name="wheel_hub",
    )
    model.articulation(
        "housing_to_jog_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=jog_wheel,
        origin=Origin(xyz=JOG_WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0),
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

    housing = object_model.get_part("housing")
    trackball = object_model.get_part("trackball")
    jog_wheel = object_model.get_part("jog_wheel")
    trackball_joint = object_model.get_articulation("housing_to_trackball")
    jog_wheel_joint = object_model.get_articulation("housing_to_jog_wheel")

    key_specs = make_key_specs()
    ctx.check(
        "keyboard has a dense field of moving keys",
        len(key_specs) >= 30,
        details=f"key_count={len(key_specs)}",
    )

    sample_keys = (
        "fkey_3",
        "key_r2_c4",
        "media_2",
    )
    for key_name in sample_keys:
        key = object_model.get_part(key_name)
        key_joint = object_model.get_articulation(f"housing_to_{key_name}")
        limits = key_joint.motion_limits
        ctx.check(
            f"{key_name} is a short-travel plunger",
            key_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(key_joint.axis) == (0.0, 0.0, -1.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.0035,
            details=f"axis={key_joint.axis}, limits={limits}",
        )
        with ctx.pose({key_joint: 0.0}):
            ctx.expect_gap(
                key,
                housing,
                axis="z",
                positive_elem="key_skirt",
                negative_elem="top_deck",
                min_gap=0.0,
                max_gap=0.0002,
                name=f"{key_name} sits flush in the deck opening",
            )
            rest_pos = ctx.part_world_position(key)
        with ctx.pose({key_joint: KEY_TRAVEL}):
            pressed_pos = ctx.part_world_position(key)
        ctx.check(
            f"{key_name} moves downward when pressed",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.check(
        "trackball uses continuous spin",
        trackball_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(trackball_joint.axis) == (0.0, 0.0, 1.0)
        and trackball_joint.motion_limits is not None
        and trackball_joint.motion_limits.lower is None
        and trackball_joint.motion_limits.upper is None,
        details=f"axis={trackball_joint.axis}, limits={trackball_joint.motion_limits}",
    )
    ctx.check(
        "jog wheel spins on a front-to-back axis",
        jog_wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(jog_wheel_joint.axis) == (0.0, 1.0, 0.0)
        and jog_wheel_joint.motion_limits is not None
        and jog_wheel_joint.motion_limits.lower is None
        and jog_wheel_joint.motion_limits.upper is None,
        details=f"axis={jog_wheel_joint.axis}, limits={jog_wheel_joint.motion_limits}",
    )

    with ctx.pose({trackball_joint: 0.0}):
        ctx.expect_contact(
            trackball,
            housing,
            elem_a="ball",
            elem_b="trackball_bearing",
            contact_tol=1e-6,
            name="trackball rests on its bearing support",
        )
        ctx.expect_overlap(
            trackball,
            housing,
            axes="xy",
            elem_a="ball",
            elem_b="trackball_ring",
            min_overlap=0.040,
            name="trackball stays centered in its side cup",
        )

    with ctx.pose({jog_wheel_joint: 0.0}):
        ctx.expect_gap(
            jog_wheel,
            housing,
            axis="z",
            positive_elem="wheel_tire",
            negative_elem="top_deck",
            min_gap=0.0010,
            max_gap=0.0040,
            name="jog wheel stands proud of the deck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
