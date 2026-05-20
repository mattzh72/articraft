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
    ExtrudeGeometry,
    Inertial,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_rear_housing_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    vertices = [
        (-0.250, 0.000, 0.000),
        (0.250, 0.000, 0.000),
        (0.250, 0.095, 0.000),
        (-0.250, 0.095, 0.000),
        (-0.246, 0.000, 0.028),
        (0.246, 0.000, 0.028),
        (0.248, 0.095, 0.044),
        (-0.248, 0.095, 0.044),
    ]
    ids = [geom.add_vertex(*vertex) for vertex in vertices]

    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])  # bottom
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])  # top
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])  # front
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])  # right
    _add_quad(geom, ids[2], ids[6], ids[7], ids[3])  # back
    _add_quad(geom, ids[3], ids[7], ids[4], ids[0])  # left
    return geom


def _arc_points_2d(
    cx: float,
    cy: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return points


def _build_white_key_outline(
    *,
    width: float,
    depth: float,
    rear_width: float,
    shoulder_y: float,
    left_relief: bool,
    right_relief: bool,
) -> list[tuple[float, float]]:
    half_width = width * 0.5
    rear_half_width = min(half_width - 0.0008, rear_width * 0.5)
    front_y = -depth * 0.5
    rear_y = depth * 0.5
    shoulder_y = min(rear_y - 0.004, max(front_y + 0.006, shoulder_y))
    front_radius = min(0.0025, half_width * 0.45, depth * 0.12)

    right_rear_x = rear_half_width if right_relief else half_width
    left_rear_x = -rear_half_width if left_relief else -half_width

    outline: list[tuple[float, float]] = [(right_rear_x, rear_y)]
    if right_relief:
        outline.append((right_rear_x, shoulder_y))
        outline.append((half_width, shoulder_y))
    outline.append((half_width, front_y + front_radius))
    outline.extend(
        _arc_points_2d(
            half_width - front_radius,
            front_y + front_radius,
            front_radius,
            0.0,
            -math.pi / 2.0,
            segments=4,
        )[1:]
    )
    outline.append((-half_width + front_radius, front_y))
    outline.extend(
        _arc_points_2d(
            -half_width + front_radius,
            front_y + front_radius,
            front_radius,
            -math.pi / 2.0,
            -math.pi,
            segments=4,
        )[1:]
    )
    outline.append((-half_width, shoulder_y))
    if left_relief:
        outline.append((left_rear_x, shoulder_y))
        outline.append((left_rear_x, rear_y))
    else:
        outline.append((-half_width, rear_y))
    return outline


def _write_white_key_mesh(
    mesh_name: str,
    *,
    width: float,
    depth: float,
    height: float,
    rear_width: float,
    shoulder_y: float,
    left_relief: bool,
    right_relief: bool,
):
    lower_outline = _build_white_key_outline(
        width=width,
        depth=depth,
        rear_width=rear_width,
        shoulder_y=shoulder_y,
        left_relief=left_relief,
        right_relief=right_relief,
    )
    upper_outline = _build_white_key_outline(
        width=width - 0.0018,
        depth=depth - 0.0016,
        rear_width=max(rear_width - 0.0020, 0.0080),
        shoulder_y=shoulder_y + 0.0006,
        left_relief=left_relief,
        right_relief=right_relief,
    )
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower_outline],
            [(x, y, height) for x, y in upper_outline],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def _build_black_key_outline(
    *,
    width: float,
    front_depth: float,
    rear_depth: float,
    front_radius: float,
) -> list[tuple[float, float]]:
    half_width = width * 0.5
    rear_y = rear_depth
    front_y = -front_depth
    front_radius = min(front_radius, half_width * 0.45, front_depth * 0.35)

    outline: list[tuple[float, float]] = [(half_width, rear_y)]
    outline.append((half_width, front_y + front_radius))
    outline.extend(
        _arc_points_2d(
            half_width - front_radius,
            front_y + front_radius,
            front_radius,
            0.0,
            -math.pi / 2.0,
            segments=4,
        )[1:]
    )
    outline.append((-half_width + front_radius, front_y))
    outline.extend(
        _arc_points_2d(
            -half_width + front_radius,
            front_y + front_radius,
            front_radius,
            -math.pi / 2.0,
            -math.pi,
            segments=4,
        )[1:]
    )
    outline.append((-half_width, rear_y))
    return outline


def _write_black_key_mesh(mesh_name: str):
    lower_profile = _build_black_key_outline(
        width=0.0132,
        front_depth=0.042,
        rear_depth=0.005,
        front_radius=0.0024,
    )
    shoulder_profile = _build_black_key_outline(
        width=0.0128,
        front_depth=0.036,
        rear_depth=0.004,
        front_radius=0.0022,
    )
    upper_profile = _build_black_key_outline(
        width=0.0104,
        front_depth=0.026,
        rear_depth=0.003,
        front_radius=0.0019,
    )
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower_profile],
            [(x, y - 0.001, 0.006) for x, y in shoulder_profile],
            [(x, y - 0.003, 0.012) for x, y in upper_profile],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="midi_keyboard", assets=ASSETS)

    body = model.material("body_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    deck = model.material("deck_black", rgba=(0.10, 0.11, 0.12, 1.0))
    key_white = model.material("key_white", rgba=(0.94, 0.95, 0.96, 1.0))
    key_black = model.material("key_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.22, 0.23, 0.25, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.29, 0.34, 0.45))
    accent = model.material("accent_grey", rgba=(0.44, 0.46, 0.49, 1.0))

    chassis = model.part("chassis")
    rear_housing_mesh = mesh_from_geometry(
        _build_rear_housing_mesh(),
        ASSETS.mesh_path("midi_keyboard_rear_housing.obj"),
    )
    rear_housing = chassis.visual(rear_housing_mesh, material=body, name="rear_housing")
    chassis.visual(
        Box((0.500, 0.100, 0.018)),
        origin=Origin(xyz=(0.000, -0.050, 0.009)),
        material=body,
        name="front_base",
    )
    chassis.visual(
        Box((0.382, 0.010, 0.020)),
        origin=Origin(xyz=(0.000, 0.030, 0.010)),
        material=deck,
        name="rear_key_wall",
    )
    chassis.visual(
        Box((0.010, 0.115, 0.024)),
        origin=Origin(xyz=(-0.186, -0.026, 0.012)),
        material=deck,
        name="left_key_cheek",
    )
    chassis.visual(
        Box((0.010, 0.115, 0.024)),
        origin=Origin(xyz=(0.186, -0.026, 0.012)),
        material=deck,
        name="right_key_cheek",
    )
    chassis.visual(
        Box((0.358, 0.114, 0.002)),
        origin=Origin(xyz=(0.000, -0.026, 0.017)),
        material=deck,
        name="keybed_seat",
    )
    chassis.visual(
        Box((0.322, 0.034, 0.002)),
        origin=Origin(xyz=(0.000, -0.004, 0.023)),
        material=deck,
        name="black_key_seat",
    )
    chassis.visual(
        Box((0.496, 0.008, 0.006)),
        origin=Origin(xyz=(0.000, -0.094, 0.015)),
        material=accent,
        name="front_trim",
    )
    chassis.visual(
        Box((0.358, 0.114, 0.002)),
        origin=Origin(xyz=(0.000, -0.026, 0.019)),
        material=deck,
        name="white_key_support",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.500, 0.195, 0.055)),
        mass=2.8,
        origin=Origin(xyz=(0.000, 0.000, 0.0275)),
    )

    white_key_width = 0.0222
    white_key_pitch = 0.0235
    white_key_centers = [(-7 + index) * white_key_pitch for index in range(15)]
    black_key_positions = {0, 1, 3, 4, 5, 7, 8, 10, 11, 12}
    white_key_mesh_cache: dict[tuple[bool, bool], object] = {}
    for index, center_x in enumerate(white_key_centers):
        left_relief = (index - 1) in black_key_positions
        right_relief = index in black_key_positions
        cache_key = (left_relief, right_relief)
        white_key_mesh = white_key_mesh_cache.get(cache_key)
        if white_key_mesh is None:
            variant_name = f"white_key_{int(left_relief)}{int(right_relief)}.obj"
            white_key_mesh = _write_white_key_mesh(
                variant_name,
                width=white_key_width * 0.92,
                depth=0.080,
                height=0.014,
                rear_width=white_key_width * 0.58,
                shoulder_y=-0.010,
                left_relief=left_relief,
                right_relief=right_relief,
            )
            white_key_mesh_cache[cache_key] = white_key_mesh
        white_key_part = model.part(f"white_key_{index}_part")
        white_key_part.visual(
            white_key_mesh,
            origin=Origin(xyz=(0.000, -0.018, 0.001)),
            material=key_white,
            name="white_key",
        )
        white_key_part.inertial = Inertial.from_geometry(
            Box((white_key_width * 0.92, 0.080, 0.014)),
            mass=0.043,
            origin=Origin(xyz=(0.000, -0.018, 0.008)),
        )
        model.articulation(
            f"chassis_to_white_key_{index}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=white_key_part,
            origin=Origin(xyz=(center_x, -0.026, 0.019)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0045,
            ),
        )

    black_key_mesh = _write_black_key_mesh("midi_black_key.obj")
    for visual_index, white_index in enumerate(sorted(black_key_positions)):
        key_center_x = (white_key_centers[white_index] + white_key_centers[white_index + 1]) / 2.0
        black_key_part = model.part(f"black_key_{visual_index}_part")
        black_key_part.visual(
            Box((0.0086, 0.020, 0.008)),
            origin=Origin(
                xyz=(
                    0.000,
                    -0.010,
                    0.004,
                )
            ),
            material=key_black,
            name="black_key_filler",
        )
        black_key_part.visual(
            black_key_mesh,
            origin=Origin(
                xyz=(
                    0.000,
                    -0.005,
                    0.008,
                )
            ),
            material=key_black,
            name="black_key",
        )
        black_key_part.inertial = Inertial.from_geometry(
            Box((0.0132, 0.047, 0.020)),
            mass=0.028,
            origin=Origin(xyz=(0.000, -0.008, 0.010)),
        )
        model.articulation(
            f"chassis_to_black_key_{visual_index}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=black_key_part,
            origin=Origin(xyz=(key_center_x, -0.004, 0.026)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.8,
                velocity=0.08,
                lower=0.0,
                upper=0.0032,
            ),
        )

    def _mount_to_chassis_surface(
        child_part,
        *,
        articulation_name: str,
        point_hint: tuple[float, float, float],
        clearance: float = 0.0,
        articulation_type: ArticulationType = ArticulationType.FIXED,
        axis: tuple[float, float, float] | None = None,
        motion_limits: MotionLimits | None = None,
    ) -> None:
        model.articulation(
            articulation_name,
            articulation_type,
            parent=chassis,
            child=child_part,
            origin=place_on_surface(
                child_part,
                rear_housing,
                point_hint=point_hint,
                clearance=clearance,
                asset_root=ASSETS.asset_root,
                prefer_collisions=False,
                child_prefer_collisions=False,
            ),
            axis=axis,
            motion_limits=motion_limits,
        )

    display_part = model.part("display_module")
    display_part.visual(
        Box((0.050, 0.024, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=display_glass,
        name="display",
    )
    display_part.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.006)),
        mass=0.04,
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
    )
    _mount_to_chassis_surface(
        display_part,
        articulation_name="chassis_to_display_module",
        point_hint=(-0.012, 0.060, 0.080),
    )

    first_knob_part = None
    for row_index, local_y in enumerate((-0.014, 0.014)):
        for column_index, local_x in enumerate((-0.040, -0.010, 0.020, 0.050)):
            knob_part = model.part(f"knob_{row_index}_{column_index}_part")
            knob_part.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(0.000, 0.000, 0.005)),
                material=knob_dark,
                name="knob",
            )
            knob_part.inertial = Inertial.from_geometry(
                Cylinder(radius=0.008, length=0.010),
                mass=0.012,
                origin=Origin(xyz=(0.000, 0.000, 0.005)),
            )
            _mount_to_chassis_surface(
                knob_part,
                articulation_name=f"chassis_to_knob_{row_index}_{column_index}",
                point_hint=(0.055 + local_x, 0.056 + local_y, 0.080),
                articulation_type=ArticulationType.REVOLUTE,
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=0.08,
                    velocity=4.0,
                    lower=-2.2,
                    upper=2.2,
                ),
            )
            if first_knob_part is None:
                first_knob_part = knob_part

    first_pad_part = None
    for row_index, local_y in enumerate((-0.014, 0.014)):
        for column_index, local_x in enumerate((0.098, 0.128)):
            pad_part = model.part(f"pad_{row_index}_{column_index}_part")
            pad_part.visual(
                Box((0.024, 0.024, 0.005)),
                origin=Origin(xyz=(0.000, 0.000, 0.0025)),
                material=pad_rubber,
                name="pad",
            )
            pad_part.inertial = Inertial.from_geometry(
                Box((0.024, 0.024, 0.005)),
                mass=0.02,
                origin=Origin(xyz=(0.000, 0.000, 0.0025)),
            )
            _mount_to_chassis_surface(
                pad_part,
                articulation_name=f"chassis_to_pad_{row_index}_{column_index}",
                point_hint=(0.055 + local_x, 0.056 + local_y, 0.080),
                articulation_type=ArticulationType.PRISMATIC,
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=8.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=0.0025,
                ),
            )
            if first_pad_part is None:
                first_pad_part = pad_part

    for index, local_x in enumerate((0.066, 0.088, 0.110)):
        button_part = model.part(f"transport_button_{index}_part")
        button_part.visual(
            Box((0.010, 0.026, 0.004)),
            origin=Origin(xyz=(0.000, 0.000, 0.002)),
            material=accent,
            name="transport_button",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((0.010, 0.026, 0.004)),
            mass=0.006,
            origin=Origin(xyz=(0.000, 0.000, 0.002)),
        )
        _mount_to_chassis_surface(
            button_part,
            articulation_name=f"chassis_to_transport_button_{index}",
            point_hint=(0.055 + local_x, 0.024, 0.070),
            articulation_type=ArticulationType.PRISMATIC,
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0018,
            ),
        )

    pitch_wheel_housing_part = model.part("pitch_wheel_housing_part")
    pitch_wheel_housing_part.visual(
        Box((0.012, 0.032, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=deck,
        name="pitch_wheel_floor",
    )
    pitch_wheel_housing_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(-0.0065, 0.000, 0.005)),
        material=deck,
        name="pitch_wheel_left_cheek",
    )
    pitch_wheel_housing_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(0.0065, 0.000, 0.005)),
        material=deck,
        name="pitch_wheel_right_cheek",
    )
    pitch_wheel_housing_part.inertial = Inertial.from_geometry(
        Box((0.016, 0.032, 0.010)),
        mass=0.012,
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
    )
    _mount_to_chassis_surface(
        pitch_wheel_housing_part,
        articulation_name="chassis_to_pitch_wheel_housing",
        point_hint=(-0.224, 0.026, 0.070),
    )

    pitch_wheel_part = model.part("pitch_wheel_part")
    pitch_wheel_part.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="pitch_wheel",
    )
    pitch_wheel_part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.010),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
    )
    _mount_to_chassis_surface(
        pitch_wheel_part,
        articulation_name="chassis_to_pitch_wheel",
        point_hint=(-0.224, 0.026, 0.070),
        articulation_type=ArticulationType.REVOLUTE,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.7,
            upper=0.7,
        ),
    )

    mod_wheel_housing_part = model.part("mod_wheel_housing_part")
    mod_wheel_housing_part.visual(
        Box((0.012, 0.032, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=deck,
        name="mod_wheel_floor",
    )
    mod_wheel_housing_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(-0.0065, 0.000, 0.005)),
        material=deck,
        name="mod_wheel_left_cheek",
    )
    mod_wheel_housing_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(0.0065, 0.000, 0.005)),
        material=deck,
        name="mod_wheel_right_cheek",
    )
    mod_wheel_housing_part.inertial = Inertial.from_geometry(
        Box((0.016, 0.032, 0.010)),
        mass=0.012,
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
    )
    _mount_to_chassis_surface(
        mod_wheel_housing_part,
        articulation_name="chassis_to_mod_wheel_housing",
        point_hint=(-0.198, 0.026, 0.070),
    )

    mod_wheel_part = model.part("mod_wheel_part")
    mod_wheel_part.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="mod_wheel",
    )
    mod_wheel_part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.010),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
    )
    _mount_to_chassis_surface(
        mod_wheel_part,
        articulation_name="chassis_to_mod_wheel",
        point_hint=(-0.198, 0.026, 0.070),
        articulation_type=ArticulationType.REVOLUTE,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.7,
            upper=0.7,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    first_white_part = object_model.get_part("white_key_0_part")
    last_white_part = object_model.get_part("white_key_14_part")
    first_black_part = object_model.get_part("black_key_0_part")
    display_part = object_model.get_part("display_module")
    first_knob_part = object_model.get_part("knob_0_0_part")
    first_pad_part = object_model.get_part("pad_0_0_part")
    pitch_wheel_housing_part = object_model.get_part("pitch_wheel_housing_part")
    pitch_wheel_part = object_model.get_part("pitch_wheel_part")
    mod_wheel_housing_part = object_model.get_part("mod_wheel_housing_part")
    mod_wheel_part = object_model.get_part("mod_wheel_part")

    rear_housing = chassis.get_visual("rear_housing")
    keybed_seat = chassis.get_visual("keybed_seat")
    white_key_support = chassis.get_visual("white_key_support")
    first_white = first_white_part.get_visual("white_key")
    last_white = last_white_part.get_visual("white_key")
    first_black = first_black_part.get_visual("black_key")
    display = display_part.get_visual("display")
    first_knob = first_knob_part.get_visual("knob")
    first_pad = first_pad_part.get_visual("pad")
    pitch_wheel_floor = pitch_wheel_housing_part.get_visual("pitch_wheel_floor")
    pitch_wheel = pitch_wheel_part.get_visual("pitch_wheel")
    mod_wheel_floor = mod_wheel_housing_part.get_visual("mod_wheel_floor")
    mod_wheel = mod_wheel_part.get_visual("mod_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    assert (
        object_model.get_articulation("chassis_to_white_key_0").articulation_type
        == ArticulationType.PRISMATIC
    )
    assert (
        object_model.get_articulation("chassis_to_white_key_14").articulation_type
        == ArticulationType.PRISMATIC
    )
    assert (
        object_model.get_articulation("chassis_to_black_key_0").articulation_type
        == ArticulationType.PRISMATIC
    )
    assert (
        object_model.get_articulation("chassis_to_knob_0_0").articulation_type
        == ArticulationType.REVOLUTE
    )
    assert (
        object_model.get_articulation("chassis_to_pad_0_0").articulation_type
        == ArticulationType.PRISMATIC
    )
    assert (
        object_model.get_articulation("chassis_to_transport_button_0").articulation_type
        == ArticulationType.PRISMATIC
    )
    assert (
        object_model.get_articulation("chassis_to_pitch_wheel").articulation_type
        == ArticulationType.REVOLUTE
    )
    assert (
        object_model.get_articulation("chassis_to_mod_wheel").articulation_type
        == ArticulationType.REVOLUTE
    )

    first_white_rest = ctx.part_world_position("white_key_0_part")
    last_white_rest = ctx.part_world_position("white_key_14_part")
    first_black_rest = ctx.part_world_position("black_key_0_part")

    ctx.expect_overlap(first_white_part, chassis, axes="xy", min_overlap=0.020)
    ctx.expect_overlap(last_white_part, chassis, axes="xy", min_overlap=0.020)
    ctx.expect_overlap(first_black_part, first_white_part, axes="xy", min_overlap=0.005)
    ctx.expect_within(first_white_part, chassis, axes="xy")
    ctx.expect_within(last_white_part, chassis, axes="xy")
    ctx.expect_within(first_black_part, chassis, axes="xy")
    ctx.expect_within(display_part, chassis, axes="xy")
    ctx.expect_within(first_knob_part, chassis, axes="xy")
    ctx.expect_within(first_pad_part, chassis, axes="xy")
    ctx.expect_within(pitch_wheel_part, chassis, axes="xy")
    ctx.expect_within(mod_wheel_part, chassis, axes="xy")
    ctx.expect_gap(
        first_white_part,
        chassis,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem=first_white,
        negative_elem=keybed_seat,
        name="white_keys_proud_of_body",
    )
    ctx.expect_gap(
        first_black_part,
        first_white_part,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=first_black,
        negative_elem=first_white,
        name="black_keys_hover_above_white_keys",
    )
    with ctx.pose(chassis_to_white_key_0=0.0045):
        first_white_pressed = ctx.part_world_position("white_key_0_part")
        assert first_white_pressed[2] < first_white_rest[2] - 0.001
    with ctx.pose(chassis_to_white_key_14=0.0045):
        last_white_pressed = ctx.part_world_position("white_key_14_part")
        assert last_white_pressed[2] < last_white_rest[2] - 0.001
    with ctx.pose(chassis_to_black_key_0=0.0032):
        first_black_pressed = ctx.part_world_position("black_key_0_part")
        assert first_black_pressed[2] < first_black_rest[2] - 0.0007
    ctx.expect_contact(
        first_knob_part,
        chassis,
        contact_tol=1e-4,
        elem_a=first_knob,
        elem_b=rear_housing,
        name="knob_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        first_pad_part,
        chassis,
        contact_tol=1e-4,
        elem_a=first_pad,
        elem_b=rear_housing,
        name="pad_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        display_part,
        chassis,
        contact_tol=1e-4,
        elem_a=display,
        elem_b=rear_housing,
        name="display_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        pitch_wheel_housing_part,
        chassis,
        contact_tol=1e-4,
        elem_a=pitch_wheel_floor,
        elem_b=rear_housing,
        name="pitch_wheel_housing_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        mod_wheel_housing_part,
        chassis,
        contact_tol=1e-4,
        elem_a=mod_wheel_floor,
        elem_b=rear_housing,
        name="mod_wheel_housing_directly_mounted_to_housing",
    )
    ctx.expect_gap(
        last_white_part,
        first_white_part,
        axis="x",
        min_gap=0.30,
        max_gap=0.32,
        positive_elem=last_white,
        negative_elem=first_white,
        name="white_key_span_reads_as_compact_two_octave_layout",
    )
    ctx.expect_gap(
        display_part,
        mod_wheel_part,
        axis="x",
        min_gap=0.15,
        max_gap=0.18,
        positive_elem=display,
        negative_elem=mod_wheel,
        name="wheels_are_clearly_left_of_control_section",
    )
    ctx.expect_gap(
        mod_wheel_part,
        pitch_wheel_part,
        axis="x",
        min_gap=0.012,
        max_gap=0.016,
        positive_elem=mod_wheel,
        negative_elem=pitch_wheel,
        name="mod_wheel_sits_just_right_of_pitch_wheel",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
