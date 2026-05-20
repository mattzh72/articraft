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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PHONE_WIDTH = 0.064
LOWER_HEIGHT = 0.070
MID_HEIGHT = 0.018
UPPER_HEIGHT = 0.084
BODY_THICKNESS = 0.011
CORNER_RADIUS = 0.008
BARREL_RADIUS = 0.0032
BARREL_OUTER_LENGTH = 0.016
BARREL_CENTER_LENGTH = 0.020
BARREL_X_OFFSET = 0.020
HINGE_Z = -BODY_THICKNESS / 2.0 + BARREL_RADIUS
CHILD_SHELL_Z = BODY_THICKNESS / 2.0 - BARREL_RADIUS


def _rounded_slab_mesh(name: str, width: float, height: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, CORNER_RADIUS, corner_segments=10),
            thickness,
        ),
        name,
    )


def _hinge_barrel_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _add_outer_barrels(part, *, y: float, z: float, material, name_prefix: str) -> None:
    for side, x in (("left", -BARREL_X_OFFSET), ("right", BARREL_X_OFFSET)):
        part.visual(
            Cylinder(radius=BARREL_RADIUS, length=BARREL_OUTER_LENGTH),
            origin=_hinge_barrel_origin(x, y, z),
            material=material,
            name=f"{name_prefix}_{side}",
        )


def _add_center_barrel(part, *, y: float, z: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_CENTER_LENGTH),
        origin=_hinge_barrel_origin(0.0, y, z),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fold_phone")

    body_graphite = model.material("body_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.58, 0.60, 0.65, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.08, 0.09, 0.10, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.06, 0.09, 0.13, 1.0))
    keypad_black = model.material("keypad_black", rgba=(0.11, 0.12, 0.14, 1.0))
    keypad_gray = model.material("keypad_gray", rgba=(0.19, 0.20, 0.23, 1.0))
    lens_black = model.material("lens_black", rgba=(0.03, 0.03, 0.04, 1.0))

    lower_shell_mesh = _rounded_slab_mesh("lower_body_shell", PHONE_WIDTH, LOWER_HEIGHT, BODY_THICKNESS)
    mid_shell_mesh = _rounded_slab_mesh("mid_panel_shell", PHONE_WIDTH, MID_HEIGHT, BODY_THICKNESS)
    upper_shell_mesh = _rounded_slab_mesh("upper_body_shell", PHONE_WIDTH, UPPER_HEIGHT, BODY_THICKNESS)

    lower_body = model.part("lower_body")
    lower_body.visual(lower_shell_mesh, material=body_graphite, name="lower_shell")
    lower_body.visual(
        Box((PHONE_WIDTH - 0.006, LOWER_HEIGHT - 0.006, 0.0014)),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS / 2.0 + 0.0007)),
        material=fascia_black,
        name="keypad_fascia",
    )
    lower_body.visual(
        Cylinder(radius=0.0105, length=0.0018),
        origin=Origin(xyz=(0.0, 0.014, BODY_THICKNESS / 2.0 + 0.0009)),
        material=keypad_gray,
        name="nav_ring",
    )
    lower_body.visual(
        Cylinder(radius=0.0043, length=0.0020),
        origin=Origin(xyz=(0.0, 0.014, BODY_THICKNESS / 2.0 + 0.0010)),
        material=keypad_black,
        name="select_key",
    )
    lower_body.visual(
        Box((0.011, 0.006, 0.0018)),
        origin=Origin(xyz=(-0.016, 0.014, BODY_THICKNESS / 2.0 + 0.0009)),
        material=keypad_black,
        name="soft_key_left",
    )
    lower_body.visual(
        Box((0.011, 0.006, 0.0018)),
        origin=Origin(xyz=(0.016, 0.014, BODY_THICKNESS / 2.0 + 0.0009)),
        material=keypad_black,
        name="soft_key_right",
    )

    row_ys = (-0.002, -0.014, -0.026, -0.038)
    col_xs = (-0.018, 0.0, 0.018)
    for row_index, y in enumerate(row_ys, start=1):
        for col_index, x in enumerate(col_xs, start=1):
            lower_body.visual(
                Box((0.013, 0.0082, 0.0018)),
                origin=Origin(xyz=(x, y, BODY_THICKNESS / 2.0 + 0.0009)),
                material=keypad_black,
                name=f"key_{row_index}_{col_index}",
            )

    _add_outer_barrels(
        lower_body,
        y=LOWER_HEIGHT / 2.0,
        z=HINGE_Z,
        material=hinge_metal,
        name_prefix="lower_hinge_knuckle",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, LOWER_HEIGHT, BODY_THICKNESS + 0.004)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    mid_panel = model.part("mid_panel")
    mid_panel.visual(
        mid_shell_mesh,
        origin=Origin(xyz=(0.0, MID_HEIGHT / 2.0, CHILD_SHELL_Z)),
        material=hinge_metal,
        name="mid_shell",
    )
    mid_panel.visual(
        Box((PHONE_WIDTH - 0.008, MID_HEIGHT - 0.004, 0.0014)),
        origin=Origin(xyz=(0.0, MID_HEIGHT / 2.0, CHILD_SHELL_Z + BODY_THICKNESS / 2.0 + 0.0007)),
        material=body_graphite,
        name="mid_front_trim",
    )
    mid_panel.visual(
        Box((PHONE_WIDTH - 0.010, MID_HEIGHT - 0.006, 0.0012)),
        origin=Origin(xyz=(0.0, MID_HEIGHT / 2.0, CHILD_SHELL_Z - BODY_THICKNESS / 2.0 - 0.0006)),
        material=body_graphite,
        name="mid_back_trim",
    )
    _add_center_barrel(mid_panel, y=0.0, z=0.0, material=hinge_metal, name="lower_center_knuckle")
    _add_outer_barrels(
        mid_panel,
        y=MID_HEIGHT,
        z=0.0,
        material=hinge_metal,
        name_prefix="upper_hinge_knuckle",
    )
    mid_panel.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, MID_HEIGHT, BODY_THICKNESS + 0.004)),
        mass=0.03,
        origin=Origin(xyz=(0.0, MID_HEIGHT / 2.0, CHILD_SHELL_Z)),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        upper_shell_mesh,
        origin=Origin(xyz=(0.0, UPPER_HEIGHT / 2.0, CHILD_SHELL_Z)),
        material=body_graphite,
        name="upper_shell",
    )
    upper_body.visual(
        Box((PHONE_WIDTH - 0.006, UPPER_HEIGHT - 0.006, 0.0014)),
        origin=Origin(xyz=(0.0, UPPER_HEIGHT / 2.0, CHILD_SHELL_Z + BODY_THICKNESS / 2.0 + 0.0007)),
        material=fascia_black,
        name="display_bezel",
    )
    upper_body.visual(
        Box((PHONE_WIDTH - 0.014, UPPER_HEIGHT - 0.020, 0.0010)),
        origin=Origin(xyz=(0.0, UPPER_HEIGHT / 2.0 - 0.002, CHILD_SHELL_Z + BODY_THICKNESS / 2.0 + 0.0011)),
        material=screen_glass,
        name="main_display",
    )
    upper_body.visual(
        Box((0.014, 0.0028, 0.0010)),
        origin=Origin(xyz=(0.0, UPPER_HEIGHT - 0.010, CHILD_SHELL_Z + BODY_THICKNESS / 2.0 + 0.0012)),
        material=keypad_gray,
        name="earpiece_slot",
    )
    upper_body.visual(
        Cylinder(radius=0.0023, length=0.0014),
        origin=Origin(xyz=(0.020, UPPER_HEIGHT - 0.010, CHILD_SHELL_Z + BODY_THICKNESS / 2.0 + 0.0012)),
        material=lens_black,
        name="front_camera",
    )
    upper_body.visual(
        Cylinder(radius=0.0032, length=0.0018),
        origin=Origin(
            xyz=(0.022, UPPER_HEIGHT - 0.014, CHILD_SHELL_Z - BODY_THICKNESS / 2.0 - 0.0009),
        ),
        material=lens_black,
        name="rear_camera",
    )
    _add_center_barrel(upper_body, y=0.0, z=0.0, material=hinge_metal, name="upper_center_knuckle")
    upper_body.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, UPPER_HEIGHT, BODY_THICKNESS + 0.004)),
        mass=0.10,
        origin=Origin(xyz=(0.0, UPPER_HEIGHT / 2.0, CHILD_SHELL_Z)),
    )

    model.articulation(
        "lower_to_mid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=mid_panel,
        origin=Origin(xyz=(0.0, LOWER_HEIGHT / 2.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=3.05,
        ),
    )
    model.articulation(
        "mid_to_upper_hinge",
        ArticulationType.REVOLUTE,
        parent=mid_panel,
        child=upper_body,
        origin=Origin(xyz=(0.0, MID_HEIGHT, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-3.05,
            upper=3.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    mid_panel = object_model.get_part("mid_panel")
    upper_body = object_model.get_part("upper_body")
    lower_to_mid = object_model.get_articulation("lower_to_mid_hinge")
    mid_to_upper = object_model.get_articulation("mid_to_upper_hinge")

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

    ctx.expect_contact(lower_body, mid_panel, name="lower_and_mid_are_physically_connected")
    ctx.expect_contact(mid_panel, upper_body, name="mid_and_upper_are_physically_connected")
    ctx.expect_overlap(lower_body, mid_panel, axes="x", min_overlap=0.05, name="lower_mid_share_hinge_band")
    ctx.expect_overlap(mid_panel, upper_body, axes="x", min_overlap=0.05, name="mid_upper_share_hinge_band")

    lower_limits = lower_to_mid.motion_limits
    upper_limits = mid_to_upper.motion_limits
    ctx.check(
        "hinges_rotate_about_phone_width_axis",
        tuple(lower_to_mid.axis) == (1.0, 0.0, 0.0) and tuple(mid_to_upper.axis) == (1.0, 0.0, 0.0),
        details=f"axes were {lower_to_mid.axis} and {mid_to_upper.axis}",
    )
    ctx.check(
        "hinges_have_phone_like_fold_range",
        lower_limits is not None
        and upper_limits is not None
        and lower_limits.upper is not None
        and upper_limits.lower is not None
        and upper_limits.upper is not None
        and lower_limits.upper >= 2.9
        and upper_limits.lower <= -2.9
        and upper_limits.upper >= 2.9,
        details=f"limits were lower={lower_limits} upper={upper_limits}",
    )

    open_upper_aabb = ctx.part_world_aabb(upper_body)
    assert open_upper_aabb is not None

    with ctx.pose({mid_to_upper: 1.2}):
        folded_upper_aabb = ctx.part_world_aabb(upper_body)
        assert folded_upper_aabb is not None
        ctx.check(
            "upper_hinge_lifts_display_out_of_plane",
            folded_upper_aabb[1][2] > open_upper_aabb[1][2] + 0.03,
            details=f"open_zmax={open_upper_aabb[1][2]:.4f}, folded_zmax={folded_upper_aabb[1][2]:.4f}",
        )

    open_mid_aabb = ctx.part_world_aabb(mid_panel)
    assert open_mid_aabb is not None
    with ctx.pose({lower_to_mid: 1.1}):
        folded_mid_aabb = ctx.part_world_aabb(mid_panel)
        assert folded_mid_aabb is not None
        ctx.check(
            "lower_hinge_lifts_mid_panel_out_of_plane",
            folded_mid_aabb[1][2] > open_mid_aabb[1][2] + 0.01,
            details=f"open_zmax={open_mid_aabb[1][2]:.4f}, folded_zmax={folded_mid_aabb[1][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
