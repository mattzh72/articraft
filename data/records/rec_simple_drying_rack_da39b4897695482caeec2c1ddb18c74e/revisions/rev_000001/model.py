from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rod_mesh(name: str, start, end, radius: float, *, radial_segments: int = 18):
    return mesh_from_geometry(
        tube_from_spline_points(
            [start, end],
            radius=radius,
            samples_per_segment=2,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _add_rod(part, name: str, start, end, radius: float, material, *, visual_name: str | None = None):
    part.visual(
        _rod_mesh(name, start, end, radius),
        material=material,
        name=visual_name,
    )


def _add_top_rack(part, prefix: str, material, *, width: float, depth: float, z: float, rail_radius: float) -> None:
    x0 = -width * 0.5
    x1 = width * 0.5
    y0 = -depth * 0.5
    y1 = depth * 0.5
    _add_rod(part, f"{prefix}_left_side", (x0, y0, z), (x0, y1, z), rail_radius, material)
    _add_rod(part, f"{prefix}_right_side", (x1, y0, z), (x1, y1, z), rail_radius, material)
    _add_rod(part, f"{prefix}_front_end", (x0, y1, z), (x1, y1, z), rail_radius, material)
    _add_rod(part, f"{prefix}_rear_end", (x0, y0, z), (x1, y0, z), rail_radius, material)
    for index, y in enumerate((-0.24, -0.14, -0.04, 0.06, 0.16, 0.26)):
        _add_rod(
            part,
            f"{prefix}_cross_{index}",
            (x0, y, z),
            (x1, y, z),
            rail_radius * 0.92,
            material,
        )


def _add_support_panel(part, prefix: str, material, *, y: float, top_half_width: float, foot_half_width: float) -> None:
    top_z = 0.92
    foot_z = 0.05
    _add_rod(
        part,
        f"{prefix}_foot",
        (-foot_half_width, y, foot_z),
        (foot_half_width, y, foot_z),
        0.011,
        material,
    )
    _add_rod(
        part,
        f"{prefix}_left_leg",
        (-top_half_width, y, top_z),
        (-foot_half_width, y, foot_z),
        0.010,
        material,
    )
    _add_rod(
        part,
        f"{prefix}_right_leg",
        (top_half_width, y, top_z),
        (foot_half_width, y, foot_z),
        0.010,
        material,
    )


def _build_central_frame(model: ArticulatedObject, steel, foot_cap):
    frame = model.part("central_frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.82, 0.95)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )

    _add_top_rack(frame, "central_top", steel, width=0.46, depth=0.72, z=0.92, rail_radius=0.010)
    _add_support_panel(frame, "front_panel", steel, y=0.37, top_half_width=0.23, foot_half_width=0.33)
    _add_support_panel(frame, "rear_panel", steel, y=-0.37, top_half_width=0.23, foot_half_width=0.33)

    _add_rod(frame, "left_floor_rail", (-0.33, -0.37, 0.05), (-0.33, 0.37, 0.05), 0.010, steel)
    _add_rod(frame, "right_floor_rail", (0.33, -0.37, 0.05), (0.33, 0.37, 0.05), 0.010, steel)
    _add_rod(frame, "front_upper_brace", (-0.18, 0.14, 0.48), (0.18, 0.14, 0.48), 0.009, steel)
    _add_rod(frame, "left_lower_upright_front", (-0.18, 0.14, 0.48), (-0.18, 0.14, 0.92), 0.009, steel)
    _add_rod(frame, "left_lower_upright_rear", (-0.18, -0.14, 0.48), (-0.18, -0.14, 0.92), 0.009, steel)
    _add_rod(frame, "center_lower_upright_rear", (0.0, -0.14, 0.48), (0.0, -0.14, 0.92), 0.0085, steel)
    _add_rod(frame, "right_lower_upright_front", (0.18, 0.14, 0.48), (0.18, 0.14, 0.92), 0.009, steel)
    _add_rod(frame, "right_lower_upright_rear", (0.18, -0.14, 0.48), (0.18, -0.14, 0.92), 0.009, steel)
    _add_rod(frame, "left_lower_side_brace", (-0.18, -0.14, 0.48), (-0.18, 0.14, 0.48), 0.0085, steel)
    _add_rod(frame, "right_lower_side_brace", (0.18, -0.14, 0.48), (0.18, 0.14, 0.48), 0.0085, steel)

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        hinge_x = sign * 0.255
        side_x = sign * 0.23
        for index, (y0, y1) in enumerate(((-0.30, -0.18), (-0.06, 0.06), (0.18, 0.30))):
            _add_rod(
                frame,
                f"{side_name}_wing_hinge_barrel_{index}",
                (hinge_x, y0, 0.92),
                (hinge_x, y1, 0.92),
                0.012,
                steel,
            )
            y_mid = 0.5 * (y0 + y1)
            _add_rod(
                frame,
                f"{side_name}_wing_hinge_stub_{index}",
                (side_x, y_mid, 0.92),
                (hinge_x, y_mid, 0.92),
                0.008,
                steel,
            )

    for index, (x0, x1) in enumerate(((-0.18, -0.10), (-0.04, 0.04), (0.10, 0.18))):
        _add_rod(
            frame,
            f"lower_support_hinge_barrel_{index}",
            (x0, -0.14, 0.48),
            (x1, -0.14, 0.48),
            0.011,
            steel,
        )

    frame.visual(
        Box((0.05, 0.018, 0.018)),
        origin=Origin(xyz=(-0.33, 0.37, 0.05)),
        material=foot_cap,
        name="front_left_foot_cap",
    )
    frame.visual(
        Box((0.05, 0.018, 0.018)),
        origin=Origin(xyz=(0.33, 0.37, 0.05)),
        material=foot_cap,
        name="front_right_foot_cap",
    )
    frame.visual(
        Box((0.05, 0.018, 0.018)),
        origin=Origin(xyz=(-0.33, -0.37, 0.05)),
        material=foot_cap,
        name="rear_left_foot_cap",
    )
    frame.visual(
        Box((0.05, 0.018, 0.018)),
        origin=Origin(xyz=(0.33, -0.37, 0.05)),
        material=foot_cap,
        name="rear_right_foot_cap",
    )
    return frame


def _build_wing(model: ArticulatedObject, name: str, steel, *, side_sign: float):
    wing = model.part(name)
    wing.inertial = Inertial.from_geometry(
        Box((0.24, 0.74, 0.05)),
        mass=1.1,
        origin=Origin(xyz=(side_sign * 0.12, 0.0, 0.0)),
    )

    inner_x = side_sign * 0.027
    outer_x = side_sign * 0.235
    y0 = -0.36
    y1 = 0.36
    _add_rod(
        wing,
        f"{name}_inner_rail",
        (inner_x, y0, 0.0),
        (inner_x, y1, 0.0),
        0.0085,
        steel,
        visual_name="inner_rail",
    )
    _add_rod(
        wing,
        f"{name}_outer_rail",
        (outer_x, y0, 0.0),
        (outer_x, y1, 0.0),
        0.0095,
        steel,
        visual_name="outer_rail",
    )
    _add_rod(
        wing,
        f"{name}_front_end",
        (inner_x, y1, 0.0),
        (outer_x, y1, 0.0),
        0.009,
        steel,
    )
    _add_rod(
        wing,
        f"{name}_rear_end",
        (inner_x, y0, 0.0),
        (outer_x, y0, 0.0),
        0.009,
        steel,
    )
    for index, y in enumerate((-0.25, -0.125, 0.0, 0.125, 0.25)):
        _add_rod(
            wing,
            f"{name}_cross_{index}",
            (inner_x, y, 0.0),
            (outer_x, y, 0.0),
            0.0082,
            steel,
        )
    for index, (y0_barrel, y1_barrel) in enumerate(((-0.18, -0.06), (0.06, 0.18))):
        _add_rod(
            wing,
            f"{name}_hinge_barrel_{index}",
            (0.0, y0_barrel, 0.0),
            (0.0, y1_barrel, 0.0),
            0.012,
            steel,
        )
        y_mid = 0.5 * (y0_barrel + y1_barrel)
        _add_rod(
            wing,
            f"{name}_hinge_stub_{index}",
            (0.0, y_mid, 0.0),
            (inner_x, y_mid, 0.0),
            0.0075,
            steel,
        )
    return wing


def _build_lower_support(model: ArticulatedObject, steel):
    support = model.part("lower_support")
    support.inertial = Inertial.from_geometry(
        Box((0.38, 0.26, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.13, 0.0)),
    )

    x0 = -0.16
    x1 = 0.16
    y_rear = 0.015
    y_front = 0.24
    _add_rod(
        support,
        "lower_support_rear_rail",
        (x0, y_rear, 0.0),
        (x1, y_rear, 0.0),
        0.0085,
        steel,
        visual_name="rear_rail",
    )
    _add_rod(
        support,
        "lower_support_front_rail",
        (x0, y_front, 0.0),
        (x1, y_front, 0.0),
        0.0085,
        steel,
        visual_name="front_rail",
    )
    _add_rod(
        support,
        "lower_support_left_side",
        (x0, y_rear, 0.0),
        (x0, y_front, 0.0),
        0.0082,
        steel,
    )
    _add_rod(
        support,
        "lower_support_right_side",
        (x1, y_rear, 0.0),
        (x1, y_front, 0.0),
        0.0082,
        steel,
    )
    for index, y in enumerate((0.07, 0.125, 0.18)):
        _add_rod(
            support,
            f"lower_support_cross_{index}",
            (x0, y, 0.0),
            (x1, y, 0.0),
            0.0078,
            steel,
        )
    for index, (x0_barrel, x1_barrel) in enumerate(((-0.10, -0.04), (0.04, 0.10))):
        _add_rod(
            support,
            f"lower_support_hinge_barrel_{index}",
            (x0_barrel, 0.0, 0.0),
            (x1_barrel, 0.0, 0.0),
            0.011,
            steel,
        )
        x_mid = 0.5 * (x0_barrel + x1_barrel)
        _add_rod(
            support,
            f"lower_support_hinge_stub_{index}",
            (x_mid, 0.0, 0.0),
            (x_mid, y_rear, 0.0),
            0.0075,
            steel,
        )
    return support


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")
    steel = model.material("powder_coated_steel", rgba=(0.91, 0.92, 0.94, 1.0))
    foot_cap = model.material("plastic_cap", rgba=(0.15, 0.16, 0.18, 1.0))
    central_frame = _build_central_frame(model, steel, foot_cap)
    left_wing = _build_wing(model, "left_wing", steel, side_sign=-1.0)
    right_wing = _build_wing(model, "right_wing", steel, side_sign=1.0)
    lower_support = _build_lower_support(model, steel)

    model.articulation(
        "left_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=left_wing,
        origin=Origin(xyz=(-0.255, 0.0, 0.92)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.18),
    )
    model.articulation(
        "right_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=right_wing,
        origin=Origin(xyz=(0.255, 0.0, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.18),
    )
    model.articulation(
        "lower_support_hinge",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=lower_support,
        origin=Origin(xyz=(0.0, -0.14, 0.48)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=1.30),
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

    central_frame = object_model.get_part("central_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    lower_support = object_model.get_part("lower_support")
    left_hinge = object_model.get_articulation("left_wing_hinge")
    right_hinge = object_model.get_articulation("right_wing_hinge")
    support_hinge = object_model.get_articulation("lower_support_hinge")

    ctx.expect_contact(
        left_wing,
        central_frame,
        contact_tol=0.0015,
        name="left wing remains mounted at the hinge line",
    )
    ctx.expect_contact(
        right_wing,
        central_frame,
        contact_tol=0.0015,
        name="right wing remains mounted at the hinge line",
    )
    ctx.expect_contact(
        lower_support,
        central_frame,
        contact_tol=0.0015,
        name="lower support remains mounted on its rear hinge",
    )
    ctx.expect_overlap(
        left_wing,
        right_wing,
        axes="y",
        min_overlap=0.68,
        name="wing spans stay aligned across the rack depth",
    )

    left_rest = ctx.part_world_position(left_wing)
    right_rest = ctx.part_world_position(right_wing)
    left_outer_rest = ctx.part_element_world_aabb(left_wing, elem="outer_rail")
    right_outer_rest = ctx.part_element_world_aabb(right_wing, elem="outer_rail")
    rest_front = ctx.part_element_world_aabb(lower_support, elem="front_rail")

    with ctx.pose({left_hinge: 0.88, right_hinge: 0.88}):
        left_outer_raised = ctx.part_element_world_aabb(left_wing, elem="outer_rail")
        right_outer_raised = ctx.part_element_world_aabb(right_wing, elem="outer_rail")

    left_outer_rest_z = None if left_outer_rest is None else 0.5 * (left_outer_rest[0][2] + left_outer_rest[1][2])
    right_outer_rest_z = None if right_outer_rest is None else 0.5 * (right_outer_rest[0][2] + right_outer_rest[1][2])
    left_outer_raised_z = None if left_outer_raised is None else 0.5 * (left_outer_raised[0][2] + left_outer_raised[1][2])
    right_outer_raised_z = None if right_outer_raised is None else 0.5 * (right_outer_raised[0][2] + right_outer_raised[1][2])

    ctx.check(
        "both wings lift upward with positive motion",
        left_outer_rest_z is not None
        and right_outer_rest_z is not None
        and left_outer_raised_z is not None
        and right_outer_raised_z is not None
        and left_outer_raised_z > left_outer_rest_z + 0.05
        and right_outer_raised_z > right_outer_rest_z + 0.05,
        details=(
            f"left_outer_rest_z={left_outer_rest_z}, left_outer_raised_z={left_outer_raised_z}, "
            f"right_outer_rest_z={right_outer_rest_z}, right_outer_raised_z={right_outer_raised_z}"
        ),
    )

    with ctx.pose({support_hinge: 1.05}):
        folded_front = ctx.part_element_world_aabb(lower_support, elem="front_rail")

    rest_front_z = None if rest_front is None else 0.5 * (rest_front[0][2] + rest_front[1][2])
    folded_front_z = None if folded_front is None else 0.5 * (folded_front[0][2] + folded_front[1][2])
    ctx.check(
        "lower support shelf folds upward from the rear hinge",
        rest_front_z is not None and folded_front_z is not None and folded_front_z > rest_front_z + 0.08,
        details=f"rest_front_z={rest_front_z}, folded_front_z={folded_front_z}",
    )

    if left_rest is not None and right_rest is not None:
        ctx.check(
            "open rack stays nearly symmetric about the center plane",
            abs(left_rest[0] + right_rest[0]) < 0.01
            and abs(left_rest[1] - right_rest[1]) < 0.001
            and abs(left_rest[2] - right_rest[2]) < 0.001,
            details=f"left_rest={left_rest}, right_rest={right_rest}",
        )
    else:
        ctx.fail("open rack stays nearly symmetric about the center plane", f"left_rest={left_rest}, right_rest={right_rest}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
