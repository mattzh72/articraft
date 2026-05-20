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
    ExtrudeGeometry,
    Inertial,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HALF_OUTLINE_RIGHT = (
    (0.000, 0.004),
    (0.046, 0.008),
    (0.128, 0.004),
    (0.158, -0.042),
    (0.148, -0.110),
    (0.084, -0.144),
    (0.010, -0.120),
    (-0.006, -0.074),
)
FRONT_Y = -0.144
REAR_Y = 0.008
MAX_OUTER_X = 0.158


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _deck_height(x: float, y: float) -> float:
    rear_fraction = (y - FRONT_Y) / (REAR_Y - FRONT_Y)
    rear_fraction = max(0.0, min(1.0, rear_fraction))
    outer_fraction = max(0.0, min(1.0, abs(x) / MAX_OUTER_X))
    return 0.010 + 0.006 * rear_fraction + 0.003 * outer_fraction


def _half_outline(side_sign: int) -> list[tuple[float, float]]:
    mirrored = [(side_sign * x, y) for x, y in HALF_OUTLINE_RIGHT]
    if side_sign < 0:
        mirrored.reverse()
    return mirrored


def _outline_x_bounds(outline: list[tuple[float, float]], y: float) -> tuple[float, float]:
    intersections: list[float] = []
    count = len(outline)
    for index in range(count):
        x1, y1 = outline[index]
        x2, y2 = outline[(index + 1) % count]
        if abs(y2 - y1) < 1e-9:
            continue
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            intersections.append(x1 + (x2 - x1) * t)

    intersections.sort()
    deduped: list[float] = []
    for x in intersections:
        if not deduped or abs(x - deduped[-1]) > 1e-6:
            deduped.append(x)

    if len(deduped) < 2:
        raise ValueError(f"Could not resolve outline bounds at y={y}")
    return deduped[0], deduped[-1]


def _build_half_shell_mesh(side_sign: int) -> MeshGeometry:
    outline = _half_outline(side_sign)
    geom = MeshGeometry()

    sample_count = 19
    y_min = FRONT_Y + 0.0001
    y_max = REAR_Y - 0.0001
    y_samples = [
        y_min + (y_max - y_min) * index / (sample_count - 1)
        for index in range(sample_count)
    ]

    section_ids: list[tuple[int, int, int, int]] = []
    for y in y_samples:
        x_lo, x_hi = _outline_x_bounds(outline, y)
        left_bottom = geom.add_vertex(x_lo, y, 0.0)
        right_bottom = geom.add_vertex(x_hi, y, 0.0)
        left_top = geom.add_vertex(x_lo, y, _deck_height(x_lo, y))
        right_top = geom.add_vertex(x_hi, y, _deck_height(x_hi, y))
        section_ids.append((left_bottom, right_bottom, left_top, right_top))

    for index in range(len(section_ids) - 1):
        left_bottom_a, right_bottom_a, left_top_a, right_top_a = section_ids[index]
        left_bottom_b, right_bottom_b, left_top_b, right_top_b = section_ids[index + 1]
        _add_quad(geom, left_bottom_a, left_bottom_b, right_bottom_b, right_bottom_a)
        _add_quad(geom, left_top_a, right_top_a, right_top_b, left_top_b)
        _add_quad(geom, left_bottom_a, left_top_a, left_top_b, left_bottom_b)
        _add_quad(geom, right_bottom_a, right_bottom_b, right_top_b, right_top_a)

    first_left_bottom, first_right_bottom, first_left_top, first_right_top = section_ids[0]
    last_left_bottom, last_right_bottom, last_left_top, last_right_top = section_ids[-1]
    _add_quad(geom, first_left_bottom, first_right_bottom, first_right_top, first_left_top)
    _add_quad(geom, last_left_bottom, last_left_top, last_right_top, last_right_bottom)

    return geom


def _build_keycap_mesh(name: str, *, width: float, depth: float, height: float):
    lower = rounded_rect_profile(width, depth, min(width, depth) * 0.18, corner_segments=5)
    upper = rounded_rect_profile(
        width - 0.0022,
        depth - 0.0022,
        min(width - 0.0022, depth - 0.0022) * 0.18,
        corner_segments=5,
    )
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, height) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _build_thumb_key_mesh(name: str):
    lower = rounded_rect_profile(0.024, 0.021, 0.0038, corner_segments=5)
    upper = rounded_rect_profile(0.0215, 0.0185, 0.0034, corner_segments=5)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, 0.0054) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_ergonomic_keyboard")

    housing = model.material("housing", rgba=(0.15, 0.16, 0.18, 1.0))
    bridge_metal = model.material("bridge_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    bridge_trim = model.material("bridge_trim", rgba=(0.32, 0.34, 0.37, 1.0))
    keycap = model.material("keycap", rgba=(0.86, 0.87, 0.88, 1.0))
    thumb_keycap = model.material("thumb_keycap", rgba=(0.74, 0.76, 0.79, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.090, 0.022, 0.010)),
        origin=Origin(xyz=(0.000, -0.006, 0.028)),
        material=bridge_metal,
        name="bridge_spine",
    )
    bridge.visual(
        Box((0.034, 0.030, 0.012)),
        origin=Origin(xyz=(0.000, -0.006, 0.029)),
        material=bridge_trim,
        name="bridge_collar",
    )
    bridge.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.000, -0.006, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_trim,
        name="bridge_cable_hump",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.100, 0.040, 0.050)),
        mass=0.22,
        origin=Origin(xyz=(0.000, -0.006, 0.025)),
    )

    main_key_mesh = _build_keycap_mesh(
        "ergo_main_keycap",
        width=0.018,
        depth=0.018,
        height=0.0052,
    )
    thumb_key_mesh = _build_thumb_key_mesh("ergo_thumb_keycap")

    key_columns = (0.022, 0.0425, 0.063, 0.0835, 0.104)
    key_rows = (-0.030, -0.050, -0.070, -0.090)
    column_stagger = (0.007, 0.003, -0.002, -0.005, 0.000)
    thumb_positions = (
        (0.020, -0.108),
        (0.040, -0.123),
        (0.061, -0.117),
    )

    for side_name, side_sign, half_offset_x in (
        ("left", -1, -0.045),
        ("right", 1, 0.045),
    ):
        half = model.part(f"{side_name}_half")
        half.visual(
            mesh_from_geometry(
                _build_half_shell_mesh(side_sign),
                f"{side_name}_half_shell",
            ),
            material=housing,
            name="housing_shell",
        )
        half.visual(
            Box((0.016, 0.022, 0.018)),
            origin=Origin(xyz=(side_sign * 0.008, -0.006, 0.021)),
            material=bridge_trim,
            name="bridge_mount",
        )
        half.visual(
            Box((0.032, 0.018, 0.008)),
            origin=Origin(xyz=(side_sign * 0.086, -0.008, 0.004)),
            material=bridge_trim,
            name="foot_hinge_mount",
        )
        half.visual(
            Box((0.018, 0.010, 0.002)),
            origin=Origin(xyz=(side_sign * 0.024, -0.126, -0.001)),
            material=rubber,
            name="front_inner_pad",
        )
        half.visual(
            Box((0.018, 0.010, 0.002)),
            origin=Origin(xyz=(side_sign * 0.118, -0.118, -0.001)),
            material=rubber,
            name="front_outer_pad",
        )
        half.inertial = Inertial.from_geometry(
            Box((0.172, 0.156, 0.038)),
            mass=0.60,
            origin=Origin(xyz=(side_sign * 0.076, -0.068, 0.019)),
        )
        model.articulation(
            f"bridge_to_{side_name}_half",
            ArticulationType.FIXED,
            parent=bridge,
            child=half,
            origin=Origin(xyz=(half_offset_x, 0.0, 0.0)),
        )

        foot = model.part(f"{side_name}_rear_foot")
        foot.visual(
            Cylinder(radius=0.0035, length=0.026),
            origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=foot_dark,
            name="hinge_barrel",
        )
        foot.visual(
            Box((0.010, 0.054, 0.007)),
            origin=Origin(xyz=(0.000, -0.027, -0.0035)),
            material=foot_dark,
            name="leg_bar",
        )
        foot.visual(
            Box((0.018, 0.018, 0.004)),
            origin=Origin(xyz=(0.000, -0.053, -0.007)),
            material=rubber,
            name="foot_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.022, 0.070, 0.018)),
            mass=0.03,
            origin=Origin(xyz=(0.000, -0.032, -0.007)),
        )
        model.articulation(
            f"{side_name}_half_to_{side_name}_rear_foot",
            ArticulationType.REVOLUTE,
            parent=half,
            child=foot,
            origin=Origin(xyz=(side_sign * 0.086, -0.008, -0.0035)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=1.5,
                lower=0.0,
                upper=1.95,
            ),
        )

        for column_index, column_x in enumerate(key_columns):
            for row_index, base_row_y in enumerate(key_rows):
                key_y = base_row_y + column_stagger[column_index]
                key_x = side_sign * column_x
                key_z = _deck_height(key_x, key_y) + 0.0014
                key_part = model.part(f"{side_name}_main_key_{column_index}_{row_index}")
                key_part.visual(
                    main_key_mesh,
                    origin=Origin(xyz=(0.000, 0.000, 0.0006)),
                    material=keycap,
                    name="cap",
                )
                key_part.visual(
                    Box((0.0010, 0.0060, 0.0060)),
                    origin=Origin(xyz=(side_sign * 0.0088, 0.000, 0.0030)),
                    material=foot_dark,
                    name="guide_fin",
                )
                key_part.inertial = Inertial.from_geometry(
                    Box((0.018, 0.018, 0.006)),
                    mass=0.008,
                    origin=Origin(xyz=(0.000, 0.000, 0.003)),
                )
                half.visual(
                    Box((0.0010, 0.0060, 0.0080)),
                    origin=Origin(
                        xyz=(
                            key_x + side_sign * 0.0098,
                            key_y,
                            key_z + 0.0020,
                        )
                    ),
                    material=bridge_trim,
                    name=f"main_guide_{column_index}_{row_index}",
                )
                model.articulation(
                    f"{side_name}_half_to_main_key_{column_index}_{row_index}",
                    ArticulationType.PRISMATIC,
                    parent=half,
                    child=key_part,
                    origin=Origin(xyz=(key_x, key_y, key_z)),
                    axis=(0.0, 0.0, -1.0),
                    motion_limits=MotionLimits(
                        effort=1.0,
                        velocity=0.06,
                        lower=0.0,
                        upper=0.0020,
                    ),
                )

        for thumb_index, (thumb_x, thumb_y) in enumerate(thumb_positions):
            key_x = side_sign * thumb_x
            key_z = _deck_height(key_x, thumb_y) + 0.0013
            thumb_part = model.part(f"{side_name}_thumb_key_{thumb_index}")
            thumb_part.visual(
                thumb_key_mesh,
                origin=Origin(xyz=(0.000, 0.000, 0.0005)),
                material=thumb_keycap,
                name="cap",
            )
            thumb_part.visual(
                Box((0.0012, 0.0070, 0.0060)),
                origin=Origin(xyz=(side_sign * 0.0118, 0.000, 0.0030)),
                material=foot_dark,
                name="guide_fin",
            )
            thumb_part.inertial = Inertial.from_geometry(
                Box((0.024, 0.021, 0.006)),
                mass=0.010,
                origin=Origin(xyz=(0.000, 0.000, 0.003)),
            )
            half.visual(
                Box((0.0012, 0.0070, 0.0080)),
                origin=Origin(
                    xyz=(
                        key_x + side_sign * 0.0130,
                        thumb_y,
                        key_z + 0.0020,
                    )
                ),
                material=bridge_trim,
                name=f"thumb_guide_{thumb_index}",
            )
            model.articulation(
                f"{side_name}_half_to_thumb_key_{thumb_index}",
                ArticulationType.PRISMATIC,
                parent=half,
                child=thumb_part,
                origin=Origin(
                    xyz=(key_x, thumb_y, key_z),
                    rpy=(0.0, 0.0, -side_sign * 0.45),
                ),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.0,
                    velocity=0.06,
                    lower=0.0,
                    upper=0.0018,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("bridge")
    left_half = object_model.get_part("left_half")
    right_half = object_model.get_part("right_half")
    left_foot = object_model.get_part("left_rear_foot")
    right_foot = object_model.get_part("right_rear_foot")
    left_foot_joint = object_model.get_articulation("left_half_to_left_rear_foot")
    right_foot_joint = object_model.get_articulation("right_half_to_right_rear_foot")
    left_key = object_model.get_part("left_main_key_2_1")
    left_key_joint = object_model.get_articulation("left_half_to_main_key_2_1")
    right_thumb = object_model.get_part("right_thumb_key_1")
    right_thumb_joint = object_model.get_articulation("right_half_to_thumb_key_1")

    ctx.expect_contact(
        bridge,
        left_half,
        elem_a="bridge_spine",
        elem_b="bridge_mount",
        name="bridge spine contacts left half",
    )
    ctx.expect_contact(
        bridge,
        right_half,
        elem_a="bridge_spine",
        elem_b="bridge_mount",
        name="bridge spine contacts right half",
    )
    ctx.expect_overlap(
        left_key,
        left_half,
        axes="xy",
        elem_a="cap",
        elem_b="housing_shell",
        min_overlap=0.014,
        name="left main key sits over the left key field",
    )
    ctx.expect_overlap(
        right_thumb,
        right_half,
        axes="xy",
        elem_a="cap",
        elem_b="housing_shell",
        min_overlap=0.020,
        name="right thumb key sits over the thumb cluster",
    )

    left_key_rest = ctx.part_world_aabb(left_key)
    with ctx.pose({left_key_joint: 0.0020}):
        ctx.expect_overlap(
            left_key,
            left_half,
            axes="xy",
            elem_a="cap",
            elem_b="housing_shell",
            min_overlap=0.014,
            name="left main key stays registered over the key field at full press",
        )
        left_key_pressed = ctx.part_world_aabb(left_key)
    ctx.check(
        "left main key plunges downward",
        left_key_rest is not None
        and left_key_pressed is not None
        and left_key_pressed[0][2] < left_key_rest[0][2] - 0.0017,
        details=f"rest={left_key_rest}, pressed={left_key_pressed}",
    )

    right_thumb_rest = ctx.part_world_aabb(right_thumb)
    with ctx.pose({right_thumb_joint: 0.0018}):
        right_thumb_pressed = ctx.part_world_aabb(right_thumb)
    ctx.check(
        "right thumb key plunges downward",
        right_thumb_rest is not None
        and right_thumb_pressed is not None
        and right_thumb_pressed[0][2] < right_thumb_rest[0][2] - 0.0014,
        details=f"rest={right_thumb_rest}, pressed={right_thumb_pressed}",
    )

    left_foot_closed = ctx.part_element_world_aabb(left_foot, elem="foot_pad")
    with ctx.pose({left_foot_joint: 1.95}):
        left_foot_open = ctx.part_element_world_aabb(left_foot, elem="foot_pad")
    ctx.check(
        "left rear foot swings down and back",
        left_foot_closed is not None
        and left_foot_open is not None
        and left_foot_open[0][2] < left_foot_closed[0][2] - 0.030
        and left_foot_open[1][1] > left_foot_closed[1][1] + 0.020,
        details=f"closed={left_foot_closed}, open={left_foot_open}",
    )

    right_foot_closed = ctx.part_element_world_aabb(right_foot, elem="foot_pad")
    with ctx.pose({right_foot_joint: 1.95}):
        right_foot_open = ctx.part_element_world_aabb(right_foot, elem="foot_pad")
    ctx.check(
        "right rear foot swings down and back",
        right_foot_closed is not None
        and right_foot_open is not None
        and right_foot_open[0][2] < right_foot_closed[0][2] - 0.030
        and right_foot_open[1][1] > right_foot_closed[1][1] + 0.020,
        details=f"closed={right_foot_closed}, open={right_foot_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
