from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_section(
    size_x: float,
    size_y: float,
    z: float,
    *,
    radius: float,
    x_offset: float = 0.0,
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (x + x_offset, y, z)
        for x, y in rounded_rect_profile(size_x, size_y, radius, corner_segments=8)
    )


def _rugged_body_mesh(
    *,
    length: float,
    width: float,
    thickness: float,
    z_min: float,
    z_max: float,
    x_offset: float = 0.0,
    shoulder_growth: float = 0.004,
    radius: float = 0.006,
):
    return section_loft(
        (
            _rounded_section(
                thickness,
                width + shoulder_growth,
                z_min,
                radius=radius,
                x_offset=x_offset,
            ),
            _rounded_section(
                thickness,
                width + shoulder_growth * 0.55,
                z_min + length * 0.12,
                radius=radius * 0.92,
                x_offset=x_offset,
            ),
            _rounded_section(
                thickness,
                width,
                z_min + length * 0.28,
                radius=radius * 0.86,
                x_offset=x_offset,
            ),
            _rounded_section(
                thickness,
                width,
                z_max - length * 0.18,
                radius=radius * 0.86,
                x_offset=x_offset,
            ),
            _rounded_section(
                thickness,
                width + shoulder_growth * 0.45,
                z_max,
                radius=radius * 0.95,
                x_offset=x_offset,
            ),
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_flip_phone")

    model.material("rubber_armor", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("hard_shell", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("screen_glass", rgba=(0.08, 0.11, 0.14, 1.0))
    model.material("key_rubber", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("hinge_metal", rgba=(0.45, 0.48, 0.50, 1.0))
    model.material("seal_gray", rgba=(0.26, 0.28, 0.29, 1.0))

    body_width = 0.060
    lower_thickness = 0.026
    upper_thickness = 0.023
    lower_length = 0.074
    upper_length = 0.062
    hinge_x = -0.006
    hinge_z = 0.082
    upper_body_x = 0.0305
    upper_inner_face_x = 0.019
    cover_height = 0.020
    cover_width = 0.020
    cover_thickness = 0.003

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_geometry(
            _rugged_body_mesh(
                length=lower_length,
                width=body_width,
                thickness=lower_thickness,
                z_min=0.0,
                z_max=lower_length,
                shoulder_growth=0.004,
                radius=0.006,
            ),
            "lower_body_shell",
        ),
        material="rubber_armor",
        name="lower_shell",
    )
    lower_body.visual(
        Box((0.003, 0.050, 0.058)),
        origin=Origin(xyz=(-0.0145, 0.0, 0.034)),
        material="hard_shell",
        name="lower_back_spine",
    )
    lower_body.visual(
        Box((0.010, 0.004, 0.048)),
        origin=Origin(xyz=(-0.001, 0.032, 0.037)),
        material="rubber_armor",
        name="lower_left_grip",
    )
    lower_body.visual(
        Box((0.010, 0.004, 0.048)),
        origin=Origin(xyz=(-0.001, -0.032, 0.037)),
        material="rubber_armor",
        name="lower_right_grip",
    )
    lower_body.visual(
        Box((0.0028, 0.046, 0.052)),
        origin=Origin(xyz=(0.0116, 0.0, 0.033)),
        material="hard_shell",
        name="keypad_deck",
    )
    lower_body.visual(
        Box((0.004, 0.028, 0.010)),
        origin=Origin(xyz=(0.013, 0.0, 0.055)),
        material="key_rubber",
        name="nav_pad",
    )
    button_size = (0.0032, 0.011, 0.0066)
    button_z = (0.043, 0.033, 0.023, 0.013)
    button_y = (-0.014, 0.0, 0.014)
    for row, z_pos in enumerate(button_z):
        for col, y_pos in enumerate(button_y):
            lower_body.visual(
                Box(button_size),
                origin=Origin(xyz=(0.0134, y_pos, z_pos)),
                material="key_rubber",
                name=f"key_{row}_{col}",
            )
    lower_body.visual(
        Box((0.0012, 0.044, 0.050)),
        origin=Origin(xyz=(0.0124, 0.0, 0.032)),
        material="seal_gray",
        name="lower_seal",
    )

    lower_barrel_y = (-0.022, 0.022)
    for index, y_pos in enumerate(lower_barrel_y):
        lower_body.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(xyz=(hinge_x, y_pos, hinge_z), rpy=(pi / 2.0, 0.0, 0.0)),
            material="hinge_metal",
            name=f"lower_barrel_{index}",
        )
        lower_body.visual(
            Box((0.010, 0.014, 0.010)),
            origin=Origin(xyz=(hinge_x, y_pos, hinge_z - 0.004)),
            material="rubber_armor",
            name=f"lower_barrel_bridge_{index}",
        )

    lower_body.inertial = Inertial.from_geometry(
        Box((lower_thickness, body_width + 0.004, lower_length)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, lower_length / 2.0)),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        mesh_from_geometry(
            _rugged_body_mesh(
                length=upper_length,
                width=body_width,
                thickness=upper_thickness,
                z_min=-(upper_length - 0.004),
                z_max=0.004,
                x_offset=upper_body_x,
                shoulder_growth=0.003,
                radius=0.0055,
            ),
            "upper_body_shell",
        ),
        material="rubber_armor",
        name="upper_shell",
    )
    upper_body.visual(
        Box((0.003, 0.050, 0.050)),
        origin=Origin(xyz=(0.0435, 0.0, -0.033)),
        material="hard_shell",
        name="upper_back_spine",
    )
    upper_body.visual(
        Box((0.010, 0.004, 0.040)),
        origin=Origin(xyz=(0.034, 0.032, -0.033)),
        material="rubber_armor",
        name="upper_left_grip",
    )
    upper_body.visual(
        Box((0.010, 0.004, 0.040)),
        origin=Origin(xyz=(0.034, -0.032, -0.033)),
        material="rubber_armor",
        name="upper_right_grip",
    )
    upper_body.visual(
        Box((0.0022, 0.046, 0.044)),
        origin=Origin(xyz=(upper_inner_face_x + 0.0011, 0.0, -0.034)),
        material="seal_gray",
        name="upper_seal",
    )
    upper_body.visual(
        Box((0.0018, 0.042, 0.036)),
        origin=Origin(xyz=(upper_inner_face_x + 0.0017, 0.0, -0.034)),
        material="screen_glass",
        name="screen_glass",
    )
    upper_body.visual(
        Box((0.013, 0.0008, 0.018)),
        origin=Origin(xyz=(0.031, 0.0296, -0.018)),
        material="seal_gray",
        name="speaker_pad",
    )
    upper_body.visual(
        Box((0.010, 0.0012, 0.012)),
        origin=Origin(xyz=(0.031, 0.0290, -0.018)),
        material="hard_shell",
        name="speaker_grille",
    )

    upper_barrel_y = (-0.008, 0.008)
    for index, y_pos in enumerate(upper_barrel_y):
        upper_body.visual(
            Cylinder(radius=0.0042, length=0.014),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="hinge_metal",
            name=f"upper_barrel_{index}",
        )
        upper_body.visual(
            Box((0.010, 0.014, 0.008)),
            origin=Origin(xyz=(0.005, y_pos, 0.004)),
            material="rubber_armor",
            name=f"upper_barrel_collar_{index}",
        )
        upper_body.visual(
            Box((0.014, 0.014, 0.008)),
            origin=Origin(xyz=(0.017, y_pos, 0.004)),
            material="rubber_armor",
            name=f"upper_barrel_cheek_{index}",
        )

    upper_body.inertial = Inertial.from_geometry(
        Box((0.043, body_width + 0.003, upper_length)),
        mass=0.13,
        origin=Origin(xyz=(0.0215, 0.0, -upper_length / 2.0)),
    )

    speaker_cover = model.part("speaker_cover")
    speaker_cover.visual(
        Box((cover_width, cover_thickness, cover_height)),
        origin=Origin(xyz=(cover_width / 2.0, -cover_thickness / 2.0, 0.0)),
        material="rubber_armor",
        name="cover_panel",
    )
    speaker_cover.visual(
        Cylinder(radius=0.0016, length=cover_height),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material="hinge_metal",
        name="cover_hinge_pin",
    )
    speaker_cover.visual(
        Box((0.004, 0.0014, 0.010)),
        origin=Origin(xyz=(0.016, -0.0022, 0.0)),
        material="seal_gray",
        name="cover_latch_lip",
    )
    speaker_cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_thickness, cover_height)),
        mass=0.01,
        origin=Origin(xyz=(cover_width / 2.0, -cover_thickness / 2.0, 0.0)),
    )

    main_hinge = model.articulation(
        "main_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.5, lower=0.0, upper=2.75),
    )
    model.articulation(
        "speaker_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=speaker_cover,
        origin=Origin(xyz=(0.021, 0.033, -0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=0.0, upper=1.35),
    )

    model.meta["primary_hinge"] = main_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    speaker_cover = object_model.get_part("speaker_cover")
    main_hinge = object_model.get_articulation("main_hinge")
    speaker_cover_hinge = object_model.get_articulation("speaker_cover_hinge")

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

    with ctx.pose({main_hinge: 0.0, speaker_cover_hinge: 0.0}):
        ctx.expect_contact(
            upper_body,
            lower_body,
            elem_a="upper_seal",
            elem_b="lower_seal",
            contact_tol=0.0006,
            name="phone halves close onto the sealing lands",
        )
        ctx.expect_contact(
            speaker_cover,
            upper_body,
            elem_a="cover_panel",
            elem_b="speaker_pad",
            contact_tol=0.0006,
            name="speaker cover seals against the side speaker pad",
        )
        ctx.expect_overlap(
            upper_body,
            lower_body,
            axes="yz",
            elem_a="upper_seal",
            elem_b="lower_seal",
            min_overlap=0.030,
            name="closed phone halves overlap broadly across the sealed footprint",
        )

    closed_screen_aabb = None
    open_screen_aabb = None
    with ctx.pose({main_hinge: 0.0}):
        closed_screen_aabb = ctx.part_element_world_aabb(upper_body, elem="screen_glass")
    with ctx.pose({main_hinge: main_hinge.motion_limits.upper}):
        open_screen_aabb = ctx.part_element_world_aabb(upper_body, elem="screen_glass")
    ctx.check(
        "main hinge lifts the screen up into the open pose",
        closed_screen_aabb is not None
        and open_screen_aabb is not None
        and open_screen_aabb[1][2] > closed_screen_aabb[1][2] + 0.045
        and abs(open_screen_aabb[0][0] - closed_screen_aabb[0][0]) > 0.015,
        details=f"closed_screen_aabb={closed_screen_aabb}, open_screen_aabb={open_screen_aabb}",
    )

    closed_cover_aabb = None
    open_cover_aabb = None
    with ctx.pose({speaker_cover_hinge: 0.0}):
        closed_cover_aabb = ctx.part_element_world_aabb(speaker_cover, elem="cover_panel")
    with ctx.pose({speaker_cover_hinge: speaker_cover_hinge.motion_limits.upper}):
        open_cover_aabb = ctx.part_element_world_aabb(speaker_cover, elem="cover_panel")
    ctx.check(
        "speaker cover swings outward from the left edge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.012,
        details=f"closed_cover_aabb={closed_cover_aabb}, open_cover_aabb={open_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
