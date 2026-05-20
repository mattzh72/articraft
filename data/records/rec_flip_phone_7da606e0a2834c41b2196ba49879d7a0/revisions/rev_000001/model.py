from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.116
BODY_DEPTH = 0.092
LOWER_THICKNESS = 0.019
UPPER_THICKNESS = 0.017
BODY_CORNER_RADIUS = 0.012

DISPLAY_HINGE_RADIUS = 0.0045
DISPLAY_HINGE_Y = -BODY_DEPTH * 0.5 - DISPLAY_HINGE_RADIUS
DISPLAY_HINGE_Z = LOWER_THICKNESS + 0.0025

SPEAKER_COVER_WIDTH = 0.072
SPEAKER_COVER_HEIGHT = 0.018
SPEAKER_COVER_THICKNESS = 0.0028
SPEAKER_HINGE_RADIUS = 0.0024
SPEAKER_HINGE_Y = -BODY_DEPTH * 0.5 - SPEAKER_HINGE_RADIUS
SPEAKER_HINGE_Z = 0.003


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_slab_mesh(
    width: float,
    depth: float,
    thickness: float,
    radius: float,
):
    return ExtrudeGeometry.centered(
        rounded_rect_profile(width, depth, min(radius, width * 0.25, depth * 0.25)),
        thickness,
        cap=True,
        closed=True,
    )


def _build_keypad_mesh():
    plate_width = 0.082
    plate_depth = 0.064
    plate_thickness = 0.0014
    key_width = 0.016
    key_depth = 0.011
    key_height = 0.0014
    x_centers = (-0.0285, -0.0095, 0.0095, 0.0285)
    y_centers = (-0.0225, -0.0070, 0.0085, 0.0240)

    geom = _rounded_slab_mesh(plate_width, plate_depth, plate_thickness, 0.0045)
    key_center_z = plate_thickness * 0.5 + key_height * 0.5
    for row_index, center_y in enumerate(y_centers):
        for column_index, center_x in enumerate(x_centers):
            width_scale = 1.08 if row_index == 3 and column_index in (1, 2) else 1.0
            button = BoxGeometry((key_width * width_scale, key_depth, key_height))
            button.translate(center_x, center_y, key_center_z)
            geom.merge(button)
    return geom


def _build_slotted_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    slot_width: float,
    slot_height: float,
    slot_centers_y: tuple[float, ...],
):
    outer = rounded_rect_profile(width, height, corner_radius)
    holes = [
        _translate_profile(
            rounded_rect_profile(slot_width, slot_height, min(slot_height * 0.5, slot_width * 0.25)),
            0.0,
            center_y,
        )
        for center_y in slot_centers_y
    ]
    return ExtrudeWithHolesGeometry(
        outer,
        holes,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_flip_communicator")

    shell_graphite = model.material("shell_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    trim_gunmetal = model.material("trim_gunmetal", rgba=(0.29, 0.31, 0.34, 1.0))
    keypad_rubber = model.material("keypad_rubber", rgba=(0.08, 0.09, 0.10, 1.0))
    display_glass = model.material("display_glass", rgba=(0.12, 0.23, 0.30, 0.72))
    speaker_black = model.material("speaker_black", rgba=(0.04, 0.04, 0.05, 1.0))
    cover_silver = model.material("cover_silver", rgba=(0.66, 0.69, 0.73, 1.0))

    lower_body = model.part("lower_body")
    lower_shell_mesh = mesh_from_geometry(
        _rounded_slab_mesh(BODY_WIDTH, BODY_DEPTH, LOWER_THICKNESS, BODY_CORNER_RADIUS),
        "lower_body_shell",
    )
    keypad_mesh = mesh_from_geometry(_build_keypad_mesh(), "lower_keypad_panel")
    speaker_panel_mesh = mesh_from_geometry(
        _build_slotted_panel_mesh(
            width=0.064,
            height=0.014,
            thickness=0.0016,
            corner_radius=0.0024,
            slot_width=0.049,
            slot_height=0.0018,
            slot_centers_y=(-0.0038, 0.0, 0.0038),
        ),
        "rear_speaker_panel",
    )

    lower_body.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_THICKNESS * 0.5)),
        material=shell_graphite,
        name="lower_shell",
    )
    lower_body.visual(
        keypad_mesh,
        origin=Origin(xyz=(0.0, 0.010, 0.0183)),
        material=keypad_rubber,
        name="keypad_panel",
    )
    lower_body.visual(
        speaker_panel_mesh,
        origin=Origin(
            xyz=(0.0, -BODY_DEPTH * 0.5 + 0.0008, 0.011),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=speaker_black,
        name="speaker_port_panel",
    )
    lower_body.visual(
        Cylinder(radius=DISPLAY_HINGE_RADIUS, length=0.024),
        origin=Origin(
            xyz=(-0.040, DISPLAY_HINGE_Y, DISPLAY_HINGE_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="lower_left_barrel",
    )
    lower_body.visual(
        Cylinder(radius=DISPLAY_HINGE_RADIUS, length=0.024),
        origin=Origin(
            xyz=(0.040, DISPLAY_HINGE_Y, DISPLAY_HINGE_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="lower_right_barrel",
    )
    lower_body.visual(
        Box((0.028, 0.008, 0.008)),
        origin=Origin(xyz=(-0.040, -0.049, 0.019)),
        material=trim_gunmetal,
        name="lower_left_hinge_bridge",
    )
    lower_body.visual(
        Box((0.028, 0.008, 0.008)),
        origin=Origin(xyz=(0.040, -0.049, 0.019)),
        material=trim_gunmetal,
        name="lower_right_hinge_bridge",
    )
    lower_body.visual(
        Cylinder(radius=SPEAKER_HINGE_RADIUS, length=0.022),
        origin=Origin(
            xyz=(-0.026, SPEAKER_HINGE_Y, SPEAKER_HINGE_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="lower_cover_left_lug",
    )
    lower_body.visual(
        Cylinder(radius=SPEAKER_HINGE_RADIUS, length=0.022),
        origin=Origin(
            xyz=(0.026, SPEAKER_HINGE_Y, SPEAKER_HINGE_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="lower_cover_right_lug",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, LOWER_THICKNESS + 0.010)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
    )

    upper_body = model.part("upper_body")
    upper_shell_mesh = mesh_from_geometry(
        _rounded_slab_mesh(BODY_WIDTH, BODY_DEPTH, UPPER_THICKNESS, BODY_CORNER_RADIUS),
        "upper_display_shell",
    )
    display_mesh = mesh_from_geometry(
        _rounded_slab_mesh(0.078, 0.062, 0.0012, 0.006),
        "upper_display_glass",
    )
    upper_body.visual(
        upper_shell_mesh,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 + DISPLAY_HINGE_RADIUS,
                UPPER_THICKNESS * 0.5,
            )
        ),
        material=shell_graphite,
        name="upper_shell",
    )
    upper_body.visual(
        display_mesh,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 + DISPLAY_HINGE_RADIUS + 0.004,
                UPPER_THICKNESS - 0.0010,
            )
        ),
        material=display_glass,
        name="display_glass",
    )
    upper_body.visual(
        Box((0.038, 0.0035, 0.0016)),
        origin=Origin(
            xyz=(
                0.0,
                DISPLAY_HINGE_RADIUS + 0.018,
                UPPER_THICKNESS - 0.0008,
            )
        ),
        material=speaker_black,
        name="earpiece_slot",
    )
    upper_body.visual(
        Cylinder(radius=DISPLAY_HINGE_RADIUS, length=0.024),
        origin=Origin(
            xyz=(-0.016, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="upper_left_barrel",
    )
    upper_body.visual(
        Cylinder(radius=DISPLAY_HINGE_RADIUS, length=0.024),
        origin=Origin(
            xyz=(0.016, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="upper_right_barrel",
    )
    upper_body.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(-0.016, 0.006, 0.005)),
        material=trim_gunmetal,
        name="upper_left_hinge_rib",
    )
    upper_body.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.016, 0.006, 0.005)),
        material=trim_gunmetal,
        name="upper_right_hinge_rib",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH + 0.010, UPPER_THICKNESS)),
        mass=0.22,
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 + DISPLAY_HINGE_RADIUS,
                UPPER_THICKNESS * 0.5,
            )
        ),
    )

    speaker_cover = model.part("speaker_cover")
    speaker_cover_mesh = mesh_from_geometry(
        _build_slotted_panel_mesh(
            width=SPEAKER_COVER_WIDTH,
            height=SPEAKER_COVER_HEIGHT,
            thickness=SPEAKER_COVER_THICKNESS,
            corner_radius=0.003,
            slot_width=0.052,
            slot_height=0.0018,
            slot_centers_y=(-0.0042, 0.0, 0.0042),
        ),
        "speaker_cover_panel",
    )
    speaker_cover.visual(
        speaker_cover_mesh,
        origin=Origin(
            xyz=(
                0.0,
                SPEAKER_HINGE_RADIUS - SPEAKER_COVER_THICKNESS * 0.5,
                SPEAKER_COVER_HEIGHT * 0.5,
            ),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=cover_silver,
        name="cover_panel",
    )
    speaker_cover.visual(
        Cylinder(radius=SPEAKER_HINGE_RADIUS, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_gunmetal,
        name="cover_barrel",
    )
    speaker_cover.visual(
        Box((0.016, 0.004, 0.006)),
        origin=Origin(xyz=(-0.015, 0.0006, 0.003)),
        material=cover_silver,
        name="cover_left_rib",
    )
    speaker_cover.visual(
        Box((0.016, 0.004, 0.006)),
        origin=Origin(xyz=(0.015, 0.0006, 0.003)),
        material=cover_silver,
        name="cover_right_rib",
    )
    speaker_cover.visual(
        Box((0.026, 0.0032, 0.0028)),
        origin=Origin(
            xyz=(
                0.0,
                SPEAKER_HINGE_RADIUS - SPEAKER_COVER_THICKNESS * 0.5 - 0.0004,
                SPEAKER_COVER_HEIGHT - 0.0008,
            )
        ),
        material=cover_silver,
        name="cover_pull_lip",
    )
    speaker_cover.inertial = Inertial.from_geometry(
        Box((SPEAKER_COVER_WIDTH, SPEAKER_COVER_THICKNESS + 0.004, SPEAKER_COVER_HEIGHT)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0005, SPEAKER_COVER_HEIGHT * 0.5)),
    )

    model.articulation(
        "lower_to_upper_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, DISPLAY_HINGE_Y, DISPLAY_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.8,
            lower=0.0,
            upper=2.2,
        ),
    )
    model.articulation(
        "lower_to_speaker_cover",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=speaker_cover,
        origin=Origin(xyz=(0.0, SPEAKER_HINGE_Y, SPEAKER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    speaker_cover = object_model.get_part("speaker_cover")
    upper_hinge = object_model.get_articulation("lower_to_upper_hinge")
    cover_hinge = object_model.get_articulation("lower_to_speaker_cover")

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

    upper_limits = upper_hinge.motion_limits
    cover_limits = cover_hinge.motion_limits
    ctx.check(
        "communicator parts exist",
        lower_body is not None and upper_body is not None and speaker_cover is not None,
        details="Missing one or more authored parts.",
    )
    ctx.check(
        "display hinge is a wide x-axis flip joint",
        upper_hinge.axis == (1.0, 0.0, 0.0)
        and upper_limits is not None
        and upper_limits.lower == 0.0
        and upper_limits.upper is not None
        and upper_limits.upper >= 2.0,
        details=f"axis={upper_hinge.axis}, limits={upper_limits}",
    )
    ctx.check(
        "speaker cover hinge is a rear x-axis flap joint",
        cover_hinge.axis == (1.0, 0.0, 0.0)
        and cover_limits is not None
        and cover_limits.lower == 0.0
        and cover_limits.upper is not None
        and cover_limits.upper >= 1.0,
        details=f"axis={cover_hinge.axis}, limits={cover_limits}",
    )

    with ctx.pose({upper_hinge: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="z",
            positive_elem="upper_shell",
            negative_elem="lower_shell",
            max_gap=0.003,
            max_penetration=0.0,
            name="upper shell closes with a slim seam above the keypad body",
        )
        ctx.expect_overlap(
            upper_body,
            lower_body,
            axes="xy",
            elem_a="upper_shell",
            elem_b="lower_shell",
            min_overlap=0.085,
            name="upper and lower bodies share the same wide footprint when closed",
        )
        ctx.expect_contact(
            lower_body,
            upper_body,
            elem_a="lower_left_barrel",
            elem_b="upper_left_barrel",
            contact_tol=1e-5,
            name="left display hinge barrels physically meet",
        )
        ctx.expect_contact(
            lower_body,
            upper_body,
            elem_a="lower_right_barrel",
            elem_b="upper_right_barrel",
            contact_tol=1e-5,
            name="right display hinge barrels physically meet",
        )
        ctx.expect_contact(
            lower_body,
            speaker_cover,
            elem_a="lower_cover_left_lug",
            elem_b="cover_barrel",
            contact_tol=1e-5,
            name="speaker cover barrel stays mounted on the left lug",
        )
        ctx.expect_gap(
            lower_body,
            speaker_cover,
            axis="y",
            positive_elem="speaker_port_panel",
            negative_elem="cover_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="speaker cover rests flush over the rear port panel",
        )

        closed_display_aabb = ctx.part_element_world_aabb(upper_body, elem="display_glass")
        closed_cover_aabb = ctx.part_element_world_aabb(speaker_cover, elem="cover_panel")

    open_display_angle = 1.4 if upper_limits is None or upper_limits.upper is None else min(1.4, upper_limits.upper)
    open_cover_angle = 0.95 if cover_limits is None or cover_limits.upper is None else min(0.95, cover_limits.upper)

    with ctx.pose({upper_hinge: open_display_angle, cover_hinge: open_cover_angle}):
        open_display_aabb = ctx.part_element_world_aabb(upper_body, elem="display_glass")
        open_cover_aabb = ctx.part_element_world_aabb(speaker_cover, elem="cover_panel")

    ctx.check(
        "upper display body flips upward from the hinge line",
        closed_display_aabb is not None
        and open_display_aabb is not None
        and open_display_aabb[1][2] > closed_display_aabb[1][2] + 0.035,
        details=f"closed_display_aabb={closed_display_aabb}, open_display_aabb={open_display_aabb}",
    )
    ctx.check(
        "speaker cover pivots rearward away from the port panel",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][1] < closed_cover_aabb[0][1] - 0.010,
        details=f"closed_cover_aabb={closed_cover_aabb}, open_cover_aabb={open_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
