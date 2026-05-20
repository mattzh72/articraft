from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


INV_SQRT2 = 1.0 / math.sqrt(2.0)


def _loop_at_z(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _shell_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    thickness: float,
    radius: float,
    top_inset: float,
) -> object:
    base = rounded_rect_profile(width, depth, radius, corner_segments=8)
    shoulder = rounded_rect_profile(width, depth, radius, corner_segments=8)
    top = rounded_rect_profile(
        width - 2.0 * top_inset,
        depth - 2.0 * top_inset,
        max(radius - top_inset * 0.75, radius * 0.65),
        corner_segments=8,
    )
    geom = LoftGeometry(
        [
            _loop_at_z(base, 0.0),
            _loop_at_z(shoulder, min(thickness * 0.24, 0.0026)),
            _loop_at_z(top, thickness),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _barrel_segment_mesh(
    name: str,
    *,
    center: tuple[float, float, float],
    axis: tuple[float, float, float],
    length: float,
    radius: float,
) -> object:
    ax, ay, az = axis
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    ux, uy, uz = ax / mag, ay / mag, az / mag
    half = 0.5 * length
    start = (center[0] - ux * half, center[1] - uy * half, center[2] - uz * half)
    end = (center[0] + ux * half, center[1] + uy * half, center[2] + uz * half)
    geom = wire_from_points([start, end], radius=radius, cap_ends=True)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_pivot_flip_phone")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.65, 0.69, 1.0))
    key_dark = model.material("key_dark", rgba=(0.13, 0.14, 0.16, 1.0))
    key_light = model.material("key_light", rgba=(0.75, 0.77, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.14, 0.24, 0.29, 0.48))

    body_size = 0.064
    lower_thickness = 0.013
    upper_thickness = 0.0105
    hinge_clearance = 0.0036
    hinge_radius = 0.0021
    hinge_origin = (0.0310, 0.0310, lower_thickness + hinge_clearance)
    hinge_axis = (-INV_SQRT2, INV_SQRT2, 0.0)
    upper_center = (-body_size * 0.5, -body_size * 0.5, 0.0)

    lower_shell_mesh = _shell_mesh(
        "lower_phone_shell",
        width=body_size,
        depth=body_size,
        thickness=lower_thickness,
        radius=0.010,
        top_inset=0.0016,
    )
    upper_shell_mesh = _shell_mesh(
        "upper_phone_shell",
        width=body_size,
        depth=body_size,
        thickness=upper_thickness,
        radius=0.010,
        top_inset=0.0018,
    )

    lower_knuckle_a = _barrel_segment_mesh(
        "lower_knuckle_a",
        center=(
            hinge_origin[0] - hinge_axis[0] * 0.00435,
            hinge_origin[1] - hinge_axis[1] * 0.00435,
            hinge_origin[2],
        ),
        axis=hinge_axis,
        length=0.0034,
        radius=hinge_radius,
    )
    lower_knuckle_b = _barrel_segment_mesh(
        "lower_knuckle_b",
        center=(
            hinge_origin[0] + hinge_axis[0] * 0.00435,
            hinge_origin[1] + hinge_axis[1] * 0.00435,
            hinge_origin[2],
        ),
        axis=hinge_axis,
        length=0.0034,
        radius=hinge_radius,
    )
    upper_knuckle = _barrel_segment_mesh(
        "upper_knuckle",
        center=(0.0, 0.0, 0.0),
        axis=hinge_axis,
        length=0.0044,
        radius=hinge_radius,
    )

    lower = model.part("lower_body")
    lower.visual(lower_shell_mesh, material=body_dark, name="lower_shell")
    lower.visual(
        Box((0.050, 0.050, 0.0010)),
        origin=Origin(xyz=(-0.0040, -0.0030, lower_thickness + 0.0005)),
        material=trim_dark,
        name="keypad_deck",
    )
    lower.visual(
        Box((0.010, 0.010, hinge_clearance)),
        origin=Origin(
            xyz=(
                hinge_origin[0] - 0.0010,
                hinge_origin[1] - 0.0010,
                lower_thickness + hinge_clearance * 0.5,
            )
        ),
        material=body_dark,
        name="lower_hinge_pedestal",
    )
    lower.visual(lower_knuckle_a, material=hinge_metal, name="lower_hinge_knuckle_a")
    lower.visual(lower_knuckle_b, material=hinge_metal, name="lower_hinge_knuckle_b")
    lower.visual(
        Box((0.016, 0.006, 0.0013)),
        origin=Origin(xyz=(-0.004, 0.010, lower_thickness + 0.00165)),
        material=key_dark,
        name="nav_bar_horizontal",
    )
    lower.visual(
        Box((0.006, 0.016, 0.0013)),
        origin=Origin(xyz=(-0.004, 0.010, lower_thickness + 0.00165)),
        material=key_dark,
        name="nav_bar_vertical",
    )
    lower.visual(
        Box((0.006, 0.006, 0.0015)),
        origin=Origin(xyz=(-0.004, 0.010, lower_thickness + 0.00175)),
        material=key_light,
        name="nav_center",
    )
    lower.visual(
        Box((0.009, 0.0045, 0.0013)),
        origin=Origin(xyz=(-0.016, 0.010, lower_thickness + 0.00165)),
        material=key_light,
        name="answer_key",
    )
    lower.visual(
        Box((0.009, 0.0045, 0.0013)),
        origin=Origin(xyz=(0.008, 0.010, lower_thickness + 0.00165)),
        material=key_dark,
        name="end_key",
    )

    key_x = (-0.016, -0.004, 0.008)
    key_y = (0.000, -0.010, -0.020, -0.029)
    for row, y in enumerate(key_y, start=1):
        for col, x in enumerate(key_x, start=1):
            lower.visual(
                Box((0.009, 0.006, 0.0013)),
                origin=Origin(xyz=(x, y, lower_thickness + 0.00165)),
                material=key_light if row < 4 else key_dark,
                name=f"key_{row}_{col}",
            )

    lower.visual(
        Box((0.016, 0.003, 0.0007)),
        origin=Origin(xyz=(-0.004, -0.0245, lower_thickness + 0.00035)),
        material=trim_dark,
        name="microphone_slit",
    )
    lower.inertial = Inertial.from_geometry(
        Box((body_size, body_size, 0.017)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
    )

    upper = model.part("upper_body")
    upper.visual(
        upper_shell_mesh,
        origin=Origin(xyz=upper_center),
        material=body_dark,
        name="upper_shell",
    )
    upper.visual(
        Box((0.008, 0.008, 0.0040)),
        origin=Origin(xyz=(-0.0010, -0.0010, 0.0020)),
        material=body_dark,
        name="upper_hinge_brace",
    )
    upper.visual(upper_knuckle, material=hinge_metal, name="upper_hinge_knuckle")
    upper.visual(
        Box((0.050, 0.050, 0.0010)),
        origin=Origin(
            xyz=(
                upper_center[0] - 0.0010,
                upper_center[1] - 0.0020,
                0.0005,
            )
        ),
        material=trim_dark,
        name="display_bezel",
    )
    upper.visual(
        Box((0.039, 0.039, 0.0008)),
        origin=Origin(
            xyz=(
                upper_center[0] - 0.0010,
                upper_center[1] - 0.0040,
                0.0014,
            )
        ),
        material=glass,
        name="display_glass",
    )
    upper.visual(
        Box((0.015, 0.003, 0.0008)),
        origin=Origin(
            xyz=(
                upper_center[0] - 0.0010,
                upper_center[1] - 0.0200,
                0.0004,
            )
        ),
        material=trim_dark,
        name="earpiece_slot",
    )
    upper.inertial = Inertial.from_geometry(
        Box((body_size, body_size, upper_thickness)),
        mass=0.11,
        origin=Origin(xyz=(upper_center[0], upper_center[1], upper_thickness * 0.5)),
    )

    model.articulation(
        "corner_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=hinge_origin),
        axis=hinge_axis,
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_body")
    hinge = object_model.get_articulation("corner_hinge")
    keypad_deck = lower.get_visual("keypad_deck")
    display_glass = upper.get_visual("display_glass")
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "hinge is a diagonal corner pivot",
        hinge.axis is not None
        and abs(hinge.axis[0] + INV_SQRT2) < 1e-6
        and abs(hinge.axis[1] - INV_SQRT2) < 1e-6
        and abs(hinge.axis[2]) < 1e-6
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 1.8,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            elem_a=display_glass,
            elem_b=keypad_deck,
            min_overlap=0.036,
            name="closed display stays over the keypad body",
        )
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem=display_glass,
            negative_elem=keypad_deck,
            min_gap=0.0015,
            max_gap=0.0040,
            name="closed lid keeps a slim clearance above the keypad",
        )
        closed_display_center = _aabb_center(ctx.part_element_world_aabb(upper, elem=display_glass))

    open_angle = hinge.motion_limits.upper if hinge.motion_limits is not None else 2.0
    with ctx.pose({hinge: open_angle}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened phone remains clear of self-overlap")
        open_display_center = _aabb_center(ctx.part_element_world_aabb(upper, elem=display_glass))

    ctx.check(
        "display rises when opened",
        closed_display_center is not None
        and open_display_center is not None
        and open_display_center[2] > closed_display_center[2] + 0.028,
        details=f"closed={closed_display_center}, open={open_display_center}",
    )
    ctx.check(
        "display swings diagonally away from the hinge corner",
        closed_display_center is not None
        and open_display_center is not None
        and open_display_center[0] > closed_display_center[0] + 0.025
        and open_display_center[1] > closed_display_center[1] + 0.025,
        details=f"closed={closed_display_center}, open={open_display_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
