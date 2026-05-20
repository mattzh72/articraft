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
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    segments: int = 24,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_keycap_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
) -> object:
    base_radius = min(width, depth) * 0.13
    lower = rounded_rect_profile(width, depth, base_radius, corner_segments=6)
    mid = rounded_rect_profile(width * 0.965, depth * 0.965, base_radius * 1.05, corner_segments=6)
    upper = rounded_rect_profile(width * 0.915, depth * 0.915, base_radius * 1.15, corner_segments=6)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, height * 0.55) for x, y in mid],
            [(x, y, height) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _build_encoder_knob_mesh(name: str) -> object:
    profile = [
        (0.0, 0.0),
        (0.0070, 0.0),
        (0.0108, 0.0012),
        (0.0114, 0.0038),
        (0.0111, 0.0095),
        (0.0102, 0.0132),
        (0.0088, 0.0153),
        (0.0, 0.0153),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=56, closed=True),
        name,
    )


def _build_stem_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
) -> object:
    profile = rounded_rect_profile(width, depth, 0.0016, corner_segments=6)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in profile],
            [(x, y, height) for x, y in profile],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _macro_key_positions() -> list[tuple[float, float]]:
    x_positions = (-0.042, -0.018, 0.006)
    y_positions = (0.031, 0.007, -0.017, -0.041)
    return [(x, y) for y in y_positions for x in x_positions]


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_keypad")

    housing_w = 0.150
    housing_d = 0.118
    housing_h = 0.026
    wall_t = 0.003
    deck_t = 0.0028
    bottom_t = 0.0025
    bottom_z = 0.0040

    key_well = 0.0144
    keycap_w = 0.0186
    keycap_h = 0.0104
    key_travel = 0.0035
    knob_positions = ((0.045, 0.019), (0.045, -0.029))
    knob_hole_radius = 0.0058
    leg_width = 0.094
    leg_arm_len = 0.056
    leg_arm_w = 0.006
    leg_thickness = 0.003
    hinge_radius = 0.0032

    housing_mat = model.material("housing_graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    deck_mat = model.material("deck_black", rgba=(0.10, 0.11, 0.12, 1.0))
    key_mat = model.material("key_pbt", rgba=(0.86, 0.87, 0.89, 1.0))
    key_shadow_mat = model.material("key_stem", rgba=(0.20, 0.21, 0.23, 1.0))
    knob_mat = model.material("knob_black", rgba=(0.14, 0.15, 0.16, 1.0))
    indicator_mat = model.material("indicator_silver", rgba=(0.77, 0.78, 0.80, 1.0))
    rubber_mat = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))

    housing = model.part("housing")

    top_outer = rounded_rect_profile(housing_w, housing_d, 0.010, corner_segments=10)
    hole_profiles = [
        _translated_profile(
            rounded_rect_profile(key_well, key_well, 0.0023, corner_segments=6),
            x,
            y,
        )
        for x, y in _macro_key_positions()
    ]
    for knob_x, knob_y in knob_positions:
        hole_profiles.append(_translated_profile(_circle_profile(knob_hole_radius, segments=28), knob_x, knob_y))
    top_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_outer,
            hole_profiles,
            deck_t,
            cap=True,
            center=True,
            closed=True,
        ),
        "macro_keypad_top_frame",
    )
    housing.visual(
        top_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, housing_h - deck_t * 0.5)),
        material=deck_mat,
        name="top_frame",
    )

    wall_h = housing_h - deck_t - bottom_z
    wall_center_z = bottom_z + wall_h * 0.5
    housing.visual(
        Box((housing_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -housing_d * 0.5 + wall_t * 0.5, wall_center_z)),
        material=housing_mat,
        name="front_wall",
    )
    housing.visual(
        Box((housing_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, housing_d * 0.5 - wall_t * 0.5, wall_center_z)),
        material=housing_mat,
        name="rear_wall",
    )
    housing.visual(
        Box((wall_t, housing_d, wall_h)),
        origin=Origin(xyz=(-housing_w * 0.5 + wall_t * 0.5, 0.0, wall_center_z)),
        material=housing_mat,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, housing_d, wall_h)),
        origin=Origin(xyz=(housing_w * 0.5 - wall_t * 0.5, 0.0, wall_center_z)),
        material=housing_mat,
        name="right_wall",
    )

    bottom_center_z = bottom_z + bottom_t * 0.5
    housing.visual(
        Box((housing_w - 2.0 * wall_t, 0.040, bottom_t)),
        origin=Origin(xyz=(0.0, -0.026, bottom_center_z)),
        material=housing_mat,
        name="bottom_front_plate",
    )
    housing.visual(
        Box((0.018, 0.054, bottom_t)),
        origin=Origin(xyz=(-0.054, 0.012, bottom_center_z)),
        material=housing_mat,
        name="bottom_left_runner",
    )
    housing.visual(
        Box((0.018, 0.054, bottom_t)),
        origin=Origin(xyz=(0.054, 0.012, bottom_center_z)),
        material=housing_mat,
        name="bottom_right_runner",
    )
    housing.visual(
        Box((housing_w - 2.0 * wall_t, 0.010, bottom_t)),
        origin=Origin(xyz=(0.0, 0.050, bottom_center_z)),
        material=housing_mat,
        name="bottom_rear_strip",
    )
    housing.visual(
        Box((0.092, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0510, 0.0112)),
        material=housing_mat,
        name="hinge_rail",
    )
    housing.visual(
        Box((housing_w - 0.020, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -housing_d * 0.5 + 0.006, housing_h - 0.006)),
        material=housing_mat,
        name="front_lip",
    )

    for foot_name, foot_x, foot_y in (
        ("foot_front_left", -0.056, -0.043),
        ("foot_front_right", 0.056, -0.043),
        ("foot_rear_left", -0.062, 0.047),
        ("foot_rear_right", 0.062, 0.047),
    ):
        housing.visual(
            Cylinder(radius=0.0044, length=0.0040),
            origin=Origin(xyz=(foot_x, foot_y, 0.0020)),
            material=rubber_mat,
            name=foot_name,
        )

    housing.inertial = Inertial.from_geometry(
        Box((housing_w, housing_d, housing_h)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, housing_h * 0.5)),
    )

    keycap_mesh = _build_keycap_mesh(
        "macro_keycap_mesh",
        width=keycap_w,
        depth=keycap_w,
        height=keycap_h,
    )
    stem_mesh = _build_stem_mesh(
        "macro_key_stem_mesh",
        width=key_well,
        depth=key_well,
        height=0.0042,
    )
    for index, (key_x, key_y) in enumerate(_macro_key_positions(), start=1):
        key_part = model.part(f"keycap_{index}")
        key_part.visual(
            stem_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.0042)),
            material=key_shadow_mat,
            name="cap_stem",
        )
        key_part.visual(
            Box((0.0100, 0.0100, 0.0042)),
            origin=Origin(xyz=(0.0, 0.0, 0.0021)),
            material=key_shadow_mat,
            name="cap_post",
        )
        key_part.visual(
            keycap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0042)),
            material=key_mat,
            name="cap_shell",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((keycap_w, keycap_w, 0.014)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        )
        model.articulation(
            f"housing_to_keycap_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key_part,
            origin=Origin(xyz=(key_x, key_y, housing_h)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.08,
                lower=0.0,
                upper=key_travel,
            ),
        )

    knob_mesh = _build_encoder_knob_mesh("macro_encoder_knob")
    for knob_name, joint_name, (knob_x, knob_y) in (
        ("encoder_left", "housing_to_encoder_left", knob_positions[0]),
        ("encoder_right", "housing_to_encoder_right", knob_positions[1]),
    ):
        knob_part = model.part(knob_name)
        knob_part.visual(
            Cylinder(radius=0.0016, length=0.0200),
            origin=Origin(xyz=(0.0, 0.0, 0.0040)),
            material=knob_mat,
            name="encoder_shaft",
        )
        knob_part.visual(
            knob_mesh,
            origin=Origin(),
            material=knob_mat,
            name="knob_body",
        )
        knob_part.visual(
            Box((0.0060, 0.0012, 0.0012)),
            origin=Origin(xyz=(0.0024, 0.0, 0.0134)),
            material=indicator_mat,
            name="indicator_mark",
        )
        knob_part.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0115, length=0.0153),
            mass=0.021,
            origin=Origin(xyz=(0.0, 0.0, 0.0076)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob_part,
            origin=Origin(xyz=(knob_x, knob_y, housing_h)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.10,
                velocity=8.0,
            ),
        )

    tilt_leg = model.part("tilt_leg")
    tilt_leg.visual(
        Cylinder(radius=hinge_radius, length=leg_width),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=housing_mat,
        name="hinge_barrel",
    )
    tilt_leg.visual(
        Box((leg_arm_w, leg_arm_len, leg_thickness)),
        origin=Origin(xyz=(-0.039, -0.028, -0.0043)),
        material=housing_mat,
        name="left_arm",
    )
    tilt_leg.visual(
        Box((leg_arm_w, leg_arm_len, leg_thickness)),
        origin=Origin(xyz=(0.039, -0.028, -0.0043)),
        material=housing_mat,
        name="right_arm",
    )
    tilt_leg.visual(
        Box((leg_width - 0.010, 0.008, leg_thickness)),
        origin=Origin(xyz=(0.0, -0.055, -0.0043)),
        material=housing_mat,
        name="foot_bar",
    )
    tilt_leg.visual(
        Box((leg_width - 0.016, 0.004, 0.0015)),
        origin=Origin(xyz=(0.0, -0.055, -0.0065)),
        material=rubber_mat,
        name="foot_pad",
    )
    tilt_leg.inertial = Inertial.from_geometry(
        Box((leg_width, leg_arm_len + 0.010, 0.010)),
        mass=0.065,
        origin=Origin(xyz=(0.0, -0.025, -0.003)),
    )
    model.articulation(
        "housing_to_tilt_leg",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=tilt_leg,
        origin=Origin(xyz=(0.0, 0.050, 0.0080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=0.0,
            upper=1.10,
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

    housing = object_model.get_part("housing")
    keycap_1 = object_model.get_part("keycap_1")
    keycap_12 = object_model.get_part("keycap_12")
    encoder_left = object_model.get_part("encoder_left")
    encoder_right = object_model.get_part("encoder_right")
    tilt_leg = object_model.get_part("tilt_leg")

    key_joint = object_model.get_articulation("housing_to_keycap_1")
    left_encoder_joint = object_model.get_articulation("housing_to_encoder_left")
    right_encoder_joint = object_model.get_articulation("housing_to_encoder_right")
    leg_joint = object_model.get_articulation("housing_to_tilt_leg")

    for index in range(1, 13):
        ctx.check(
            f"keycap_{index} exists",
            object_model.get_part(f"keycap_{index}") is not None,
        )

    for joint_name in ("housing_to_encoder_left", "housing_to_encoder_right"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous vertical rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )

    ctx.expect_gap(
        keycap_1,
        housing,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="top_frame",
        min_gap=0.0,
        max_gap=0.0045,
        name="top-left keycap sits just above the switch plate",
    )
    ctx.expect_gap(
        keycap_12,
        housing,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="top_frame",
        min_gap=0.0,
        max_gap=0.0045,
        name="bottom-right keycap sits just above the switch plate",
    )

    key_rest = ctx.part_world_position(keycap_1)
    with ctx.pose({key_joint: key_joint.motion_limits.upper if key_joint.motion_limits else 0.0}):
        key_pressed = ctx.part_world_position(keycap_1)
    ctx.check(
        "representative macro key travels downward",
        key_rest is not None
        and key_pressed is not None
        and key_pressed[2] < key_rest[2] - 0.003,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    ctx.expect_gap(
        encoder_left,
        housing,
        axis="z",
        positive_elem="knob_body",
        negative_elem="top_frame",
        min_gap=0.0,
        max_gap=0.0005,
        name="left encoder knob seats on the housing deck",
    )
    ctx.expect_gap(
        encoder_right,
        housing,
        axis="z",
        positive_elem="knob_body",
        negative_elem="top_frame",
        min_gap=0.0,
        max_gap=0.0005,
        name="right encoder knob seats on the housing deck",
    )

    leg_closed = _aabb_center(ctx.part_element_world_aabb(tilt_leg, elem="foot_bar"))
    with ctx.pose({leg_joint: leg_joint.motion_limits.upper if leg_joint.motion_limits else 0.0}):
        leg_open = _aabb_center(ctx.part_element_world_aabb(tilt_leg, elem="foot_bar"))
        ctx.expect_overlap(
            tilt_leg,
            housing,
            axes="x",
            elem_a="hinge_barrel",
            elem_b="hinge_rail",
            min_overlap=0.080,
            name="tilt leg hinge stays aligned across the housing width",
        )
    ctx.check(
        "tilt leg deploys downward from the underside",
        leg_closed is not None
        and leg_open is not None
        and leg_open[2] < leg_closed[2] - 0.015
        and leg_open[1] > leg_closed[1] + 0.020,
        details=f"closed={leg_closed}, open={leg_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
