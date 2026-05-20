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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HALF_YAW = math.radians(17.0)
KEY_TRAVEL = 0.002
FOOT_FOLD_TRAVEL = math.radians(62.0)


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rotate_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _build_keycap_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
    taper: float,
    name: str,
):
    lower = rounded_rect_profile(width, depth, radius, corner_segments=6)
    upper = [(x * taper, y * taper, height) for x, y in lower]
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            upper,
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _grid_slots() -> list[tuple[str, float, float]]:
    column_xs = (0.034, 0.058, 0.082, 0.106)
    row_ys = (-0.020, 0.004, 0.028)
    column_stagger = (-0.006, -0.002, 0.004, 0.010)
    slots: list[tuple[str, float, float]] = []
    for column_index, x_pos in enumerate(column_xs):
        for row_index, base_y in enumerate(row_ys):
            slots.append(
                (
                    f"key_r{row_index}_c{column_index}",
                    x_pos,
                    base_y + column_stagger[column_index],
                )
            )
    return slots


def _thumb_slots() -> list[tuple[str, float, float]]:
    return [
        ("thumb_0", 0.030, -0.046),
        ("thumb_1", 0.052, -0.058),
        ("thumb_2", 0.076, -0.050),
    ]


def _all_part_names() -> list[str]:
    names = ["bridge", "left_half", "right_half", "left_foot", "right_foot"]
    for side_name in ("left", "right"):
        for local_name, _, _ in _grid_slots():
            names.append(f"{side_name}_{local_name}")
        for local_name, _, _ in _thumb_slots():
            names.append(f"{side_name}_{local_name}")
    return names


def _all_key_part_names(side_name: str) -> list[str]:
    names: list[str] = []
    for local_name, _, _ in _grid_slots():
        names.append(f"{side_name}_{local_name}")
    for local_name, _, _ in _thumb_slots():
        names.append(f"{side_name}_{local_name}")
    return names


def _build_switch_plate_mesh(
    *,
    side_name: str,
    side_sign: float,
    plate_center_x: float,
    plate_center_y: float,
):
    outer = rounded_rect_profile(0.124, 0.106, 0.014, corner_segments=8)
    standard_hole = rounded_rect_profile(0.014, 0.014, 0.002, corner_segments=5)
    thumb_hole = rounded_rect_profile(0.018, 0.022, 0.003, corner_segments=5)

    holes: list[list[tuple[float, float]]] = []
    for _, x_pos, y_pos in _grid_slots():
        holes.append(
            _offset_profile(
                standard_hole,
                side_sign * x_pos - plate_center_x,
                y_pos - plate_center_y,
            )
        )
    for _, x_pos, y_pos in _thumb_slots():
        holes.append(
            _offset_profile(
                thumb_hole,
                side_sign * x_pos - plate_center_x,
                y_pos - plate_center_y,
            )
        )

    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, 0.002, center=True),
        f"{side_name}_switch_plate",
    )


def _add_half(
    model: ArticulatedObject,
    *,
    side_name: str,
    side_sign: float,
    bridge,
    body_material,
    plate_material,
    bridge_material,
    key_top_material,
    key_side_material,
    foot_material,
    foot_pad_material,
    standard_key_mesh,
    thumb_key_mesh,
) -> None:
    half_name = f"{side_name}_half"
    half = model.part(half_name)

    plate_center_x = side_sign * 0.076
    plate_center_y = -0.010
    switch_plate_mesh = _build_switch_plate_mesh(
        side_name=side_name,
        side_sign=side_sign,
        plate_center_x=plate_center_x,
        plate_center_y=plate_center_y,
    )

    half.visual(
        Box((0.146, 0.128, 0.015)),
        origin=Origin(xyz=(side_sign * 0.074, 0.000, 0.0075)),
        material=body_material,
        name="case_base",
    )
    half.visual(
        Box((0.122, 0.030, 0.008)),
        origin=Origin(xyz=(side_sign * 0.078, -0.060, 0.011)),
        material=body_material,
        name="front_lip",
    )
    half.visual(
        Box((0.122, 0.024, 0.008)),
        origin=Origin(xyz=(side_sign * 0.076, 0.053, 0.013)),
        material=body_material,
        name="rear_rise",
    )
    half.visual(
        Box((0.026, 0.036, 0.012)),
        origin=Origin(xyz=(side_sign * 0.012, 0.004, 0.010)),
        material=bridge_material,
        name="bridge_socket",
    )
    half.visual(
        Box((0.054, 0.024, 0.006)),
        origin=Origin(xyz=(side_sign * 0.046, -0.043, 0.018)),
        material=body_material,
        name="thumb_deck",
    )
    half.visual(
        Box((0.124, 0.008, 0.008)),
        origin=Origin(xyz=(plate_center_x, -0.051, 0.017)),
        material=body_material,
        name="front_rim",
    )
    half.visual(
        Box((0.124, 0.008, 0.008)),
        origin=Origin(xyz=(plate_center_x, 0.055, 0.017)),
        material=body_material,
        name="rear_rim",
    )
    half.visual(
        Box((0.008, 0.090, 0.008)),
        origin=Origin(xyz=(side_sign * 0.136, 0.002, 0.017)),
        material=body_material,
        name="outer_rim",
    )
    half.visual(
        Box((0.008, 0.070, 0.008)),
        origin=Origin(xyz=(side_sign * 0.016, 0.018, 0.017)),
        material=body_material,
        name="inner_rim",
    )
    half.visual(
        switch_plate_mesh,
        origin=Origin(xyz=(plate_center_x, plate_center_y, 0.020)),
        material=plate_material,
        name="switch_plate",
    )
    half.visual(
        Cylinder(radius=0.0042, length=0.038),
        origin=Origin(
            xyz=(side_sign * 0.094, 0.050, -0.003),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bridge_material,
        name="foot_hinge_boss",
    )
    half.inertial = Inertial.from_geometry(
        Box((0.150, 0.130, 0.028)),
        mass=0.65,
        origin=Origin(xyz=(side_sign * 0.074, 0.000, 0.014)),
    )

    model.articulation(
        f"bridge_to_{half_name}",
        ArticulationType.FIXED,
        parent=bridge,
        child=half,
        origin=Origin(xyz=(side_sign * 0.040, 0.000, 0.000), rpy=(0.0, 0.0, side_sign * HALF_YAW)),
    )

    foot_name = f"{side_name}_foot"
    foot = model.part(foot_name)
    deployed_angle = -math.radians(58.0)
    arm_center_left = _rotate_x((-0.008, 0.014, -0.002), deployed_angle)
    arm_center_right = _rotate_x((0.008, 0.014, -0.002), deployed_angle)
    crossbar_center = _rotate_x((0.0, 0.028, -0.002), deployed_angle)
    pad_center = _rotate_x((0.0, 0.035, -0.0035), deployed_angle)

    foot.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=foot_material,
        name="foot_hinge_barrel",
    )
    foot.visual(
        Box((0.004, 0.028, 0.004)),
        origin=Origin(xyz=arm_center_left, rpy=(deployed_angle, 0.0, 0.0)),
        material=foot_material,
        name="foot_arm_left",
    )
    foot.visual(
        Box((0.004, 0.028, 0.004)),
        origin=Origin(xyz=arm_center_right, rpy=(deployed_angle, 0.0, 0.0)),
        material=foot_material,
        name="foot_arm_right",
    )
    foot.visual(
        Box((0.020, 0.008, 0.004)),
        origin=Origin(xyz=crossbar_center, rpy=(deployed_angle, 0.0, 0.0)),
        material=foot_material,
        name="foot_crossbar",
    )
    foot.visual(
        Box((0.024, 0.010, 0.003)),
        origin=Origin(xyz=pad_center, rpy=(deployed_angle, 0.0, 0.0)),
        material=foot_pad_material,
        name="foot_pad",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.024, 0.040, 0.026)),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.018, -0.012)),
    )

    model.articulation(
        f"{half_name}_to_{foot_name}",
        ArticulationType.REVOLUTE,
        parent=half,
        child=foot,
        origin=Origin(xyz=(side_sign * 0.094, 0.050, -0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=FOOT_FOLD_TRAVEL,
        ),
    )

    for local_name, x_pos, y_pos in _grid_slots():
        key_name = f"{side_name}_{local_name}"
        key_part = model.part(key_name)
        key_part.visual(
            standard_key_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=key_top_material,
            name="key_top",
        )
        key_part.visual(
            Box((0.013, 0.013, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
            material=key_side_material,
            name="key_skirt",
        )
        key_part.visual(
            Box((0.017, 0.017, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0185)),
            material=key_side_material,
            name="stop_flange",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((0.017, 0.017, 0.008)),
            mass=0.010,
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
        )
        model.articulation(
            f"{half_name}_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=half,
            child=key_part,
            origin=Origin(xyz=(side_sign * x_pos, y_pos, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=0.06,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )

    for local_name, x_pos, y_pos in _thumb_slots():
        key_name = f"{side_name}_{local_name}"
        key_part = model.part(key_name)
        key_part.visual(
            thumb_key_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=key_top_material,
            name="key_top",
        )
        key_part.visual(
            Box((0.016, 0.020, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
            material=key_side_material,
            name="key_skirt",
        )
        key_part.visual(
            Box((0.021, 0.025, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0185)),
            material=key_side_material,
            name="stop_flange",
        )
        key_part.inertial = Inertial.from_geometry(
            Box((0.021, 0.025, 0.008)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
        )
        model.articulation(
            f"{half_name}_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=half,
            child=key_part,
            origin=Origin(xyz=(side_sign * x_pos, y_pos, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=0.5,
                velocity=0.06,
                lower=0.0,
                upper=KEY_TRAVEL,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_ergonomic_keyboard")

    body_material = model.material("body_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    plate_material = model.material("plate_black", rgba=(0.08, 0.09, 0.10, 1.0))
    bridge_material = model.material("bridge_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    key_top_material = model.material("key_top_grey", rgba=(0.76, 0.78, 0.80, 1.0))
    key_side_material = model.material("key_side_dark", rgba=(0.54, 0.56, 0.59, 1.0))
    foot_material = model.material("foot_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    foot_pad_material = model.material("foot_pad_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    standard_key_mesh = _build_keycap_mesh(
        width=0.016,
        depth=0.016,
        height=0.003,
        radius=0.0024,
        taper=0.88,
        name="standard_keycap",
    )
    thumb_key_mesh = _build_keycap_mesh(
        width=0.021,
        depth=0.025,
        height=0.003,
        radius=0.0030,
        taper=0.90,
        name="thumb_keycap",
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.058, 0.048, 0.014)),
        origin=Origin(xyz=(0.0, 0.000, 0.007)),
        material=bridge_material,
        name="bridge_core",
    )
    bridge.visual(
        Box((0.038, 0.032, 0.010)),
        origin=Origin(xyz=(-0.046, 0.004, 0.008)),
        material=bridge_material,
        name="left_bridge_tongue",
    )
    bridge.visual(
        Box((0.038, 0.032, 0.010)),
        origin=Origin(xyz=(0.046, 0.004, 0.008)),
        material=bridge_material,
        name="right_bridge_tongue",
    )
    bridge.visual(
        Box((0.094, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.018, 0.015)),
        material=body_material,
        name="bridge_top_cover",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.110, 0.060, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.005, 0.010)),
    )

    _add_half(
        model,
        side_name="left",
        side_sign=-1.0,
        bridge=bridge,
        body_material=body_material,
        plate_material=plate_material,
        bridge_material=bridge_material,
        key_top_material=key_top_material,
        key_side_material=key_side_material,
        foot_material=foot_material,
        foot_pad_material=foot_pad_material,
        standard_key_mesh=standard_key_mesh,
        thumb_key_mesh=thumb_key_mesh,
    )
    _add_half(
        model,
        side_name="right",
        side_sign=1.0,
        bridge=bridge,
        body_material=body_material,
        plate_material=plate_material,
        bridge_material=bridge_material,
        key_top_material=key_top_material,
        key_side_material=key_side_material,
        foot_material=foot_material,
        foot_pad_material=foot_pad_material,
        standard_key_mesh=standard_key_mesh,
        thumb_key_mesh=thumb_key_mesh,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bridge = object_model.get_part("bridge")
    left_half = object_model.get_part("left_half")
    right_half = object_model.get_part("right_half")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")

    left_foot_joint = object_model.get_articulation("left_half_to_left_foot")
    right_foot_joint = object_model.get_articulation("right_half_to_right_foot")
    sample_key = object_model.get_part("left_key_r1_c1")
    sample_key_joint = object_model.get_articulation("left_half_to_left_key_r1_c1")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        bridge,
        left_half,
        reason="Hidden bridge tongue nests into the left inner socket cover.",
    )
    ctx.allow_overlap(
        bridge,
        right_half,
        reason="Hidden bridge tongue nests into the right inner socket cover.",
    )
    ctx.allow_overlap(
        left_half,
        left_foot,
        elem_a="foot_hinge_boss",
        elem_b="foot_hinge_barrel",
        reason="Rear tenting foot uses a coaxial hinge barrel seated inside the half.",
    )
    ctx.allow_overlap(
        right_half,
        right_foot,
        elem_a="foot_hinge_boss",
        elem_b="foot_hinge_barrel",
        reason="Rear tenting foot uses a coaxial hinge barrel seated inside the half.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    part_names = {part.name for part in object_model.parts}
    for expected_name in _all_part_names():
        ctx.check(
            f"has_{expected_name}",
            expected_name in part_names,
            details=f"Missing required part {expected_name}",
        )

    ctx.expect_origin_distance(left_half, right_half, axes="x", min_dist=0.06, max_dist=0.12)
    ctx.expect_overlap(bridge, left_half, axes="y", min_overlap=0.02)
    ctx.expect_overlap(bridge, right_half, axes="y", min_overlap=0.02)
    ctx.expect_within(left_foot, left_half, axes="xy", margin=0.03)
    ctx.expect_within(right_foot, right_half, axes="xy", margin=0.03)

    for foot_joint in (left_foot_joint, right_foot_joint):
        ctx.check(
            f"{foot_joint.name}_axis",
            tuple(foot_joint.axis) == (1.0, 0.0, 0.0),
            details=f"{foot_joint.name} should hinge around local x.",
        )
        limits = foot_joint.motion_limits
        assert limits is not None
        ctx.check(
            f"{foot_joint.name}_limits",
            limits.lower == 0.0 and limits.upper is not None and limits.upper > 1.0,
            details=f"{foot_joint.name} should fold from open to tucked-away.",
        )

    for side_name, half in (("left", left_half), ("right", right_half)):
        for key_name in _all_key_part_names(side_name):
            key_part = object_model.get_part(key_name)
            key_joint = object_model.get_articulation(f"{side_name}_half_to_{key_name}")
            ctx.expect_contact(
                key_part,
                half,
                elem_a="stop_flange",
                elem_b="switch_plate",
                name=f"{key_name}_supported",
            )
            ctx.check(
                f"{key_joint.name}_axis",
                tuple(key_joint.axis) == (0.0, 0.0, -1.0),
                details=f"{key_joint.name} should plunge vertically into the half.",
            )
            limits = key_joint.motion_limits
            assert limits is not None
            ctx.check(
                f"{key_joint.name}_travel",
                limits.lower == 0.0 and limits.upper is not None and 0.0015 <= limits.upper <= 0.0025,
                details=f"{key_joint.name} should have short key travel.",
            )

    sample_key_rest_aabb = ctx.part_world_aabb(sample_key)
    assert sample_key_rest_aabb is not None
    ctx.expect_gap(
        sample_key,
        left_half,
        axis="z",
        min_gap=0.0018,
        max_gap=0.0022,
        positive_elem="key_top",
        negative_elem="switch_plate",
        name="sample_key_rest_clearance",
    )
    with ctx.pose({sample_key_joint: KEY_TRAVEL}):
        sample_key_pressed_aabb = ctx.part_world_aabb(sample_key)
        assert sample_key_pressed_aabb is not None
        ctx.check(
            "sample_key_moves_down",
            sample_key_pressed_aabb[0][2] < sample_key_rest_aabb[0][2] - 0.0015,
            details="Representative key should descend when pressed.",
        )
        ctx.expect_gap(
            sample_key,
            left_half,
            axis="z",
            max_penetration=0.00001,
            max_gap=0.0002,
            positive_elem="key_top",
            negative_elem="switch_plate",
            name="sample_key_pressed_seats_on_plate",
        )

    left_foot_open_aabb = ctx.part_world_aabb(left_foot)
    right_foot_open_aabb = ctx.part_world_aabb(right_foot)
    assert left_foot_open_aabb is not None
    assert right_foot_open_aabb is not None
    with ctx.pose(
        {
            left_foot_joint: FOOT_FOLD_TRAVEL,
            right_foot_joint: FOOT_FOLD_TRAVEL,
        }
    ):
        left_foot_folded_aabb = ctx.part_world_aabb(left_foot)
        right_foot_folded_aabb = ctx.part_world_aabb(right_foot)
        assert left_foot_folded_aabb is not None
        assert right_foot_folded_aabb is not None
        ctx.check(
            "left_foot_folds_up",
            left_foot_folded_aabb[0][2] > left_foot_open_aabb[0][2] + 0.010,
            details="Left tenting foot should tuck upward when folded.",
        )
        ctx.check(
            "right_foot_folds_up",
            right_foot_folded_aabb[0][2] > right_foot_open_aabb[0][2] + 0.010,
            details="Right tenting foot should tuck upward when folded.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
