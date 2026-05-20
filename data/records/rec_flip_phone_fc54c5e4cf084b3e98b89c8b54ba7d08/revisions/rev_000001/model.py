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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_slab_mesh(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                length,
                width,
                corner_radius,
                corner_segments=10,
            ),
            thickness,
        ),
        name,
    )


def _tube_shell_mesh(
    name: str,
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -length / 2.0),
                (outer_radius, length / 2.0),
            ],
            [
                (inner_radius, -length / 2.0),
                (inner_radius, length / 2.0),
            ],
            segments=40,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_flip_phone")

    shell_silver = model.material("shell_silver", rgba=(0.73, 0.74, 0.78, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    keypad_gray = model.material("keypad_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    button_silver = model.material("button_silver", rgba=(0.82, 0.83, 0.85, 1.0))
    screen_blue = model.material("screen_blue", rgba=(0.13, 0.25, 0.46, 1.0))
    antenna_metal = model.material("antenna_metal", rgba=(0.63, 0.65, 0.68, 1.0))

    lower_length = 0.105
    lower_width = 0.049
    lower_thickness = 0.014
    upper_length = 0.092
    upper_width = 0.047
    upper_thickness = 0.011
    rear_offset = 0.004
    antenna_sleeve_length = 0.017
    mast_length = 0.044

    lower_shell_mesh = _rounded_slab_mesh(
        "lower_phone_shell",
        length=lower_length,
        width=lower_width,
        thickness=lower_thickness,
        corner_radius=0.007,
    )
    upper_shell_mesh = _rounded_slab_mesh(
        "upper_phone_shell",
        length=upper_length,
        width=upper_width,
        thickness=upper_thickness,
        corner_radius=0.0065,
    )
    antenna_sleeve_mesh = _tube_shell_mesh(
        "antenna_sleeve",
        length=antenna_sleeve_length,
        outer_radius=0.0036,
        inner_radius=0.00245,
    )

    lower_body = model.part("lower_body")
    lower_body.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(rear_offset + lower_length / 2.0, 0.0, -0.0085)),
        material=shell_silver,
        name="lower_shell",
    )
    lower_body.visual(
        Box((0.090, 0.041, 0.0014)),
        origin=Origin(xyz=(0.059, 0.0, -0.0008)),
        material=keypad_gray,
        name="keypad_deck",
    )
    lower_body.visual(
        Cylinder(radius=0.0095, length=0.0016),
        origin=Origin(xyz=(0.032, 0.0, 0.0004)),
        material=button_silver,
        name="navigation_pad",
    )
    lower_body.visual(
        Cylinder(radius=0.0042, length=0.0018),
        origin=Origin(xyz=(0.032, 0.0, 0.00055)),
        material=graphite,
        name="select_button",
    )

    for row_index, x_pos in enumerate((0.055, 0.069, 0.083, 0.097)):
        for col_index, y_pos in enumerate((-0.0135, 0.0, 0.0135)):
            lower_body.visual(
                Box((0.011, 0.0085, 0.0012)),
                origin=Origin(xyz=(x_pos, y_pos, 0.0005)),
                material=button_silver,
                name=f"key_{row_index}_{col_index}",
            )

    lower_body.visual(
        Box((0.015, 0.0032, 0.0008)),
        origin=Origin(xyz=(0.100, 0.0, -0.0003)),
        material=graphite,
        name="microphone_slot",
    )
    lower_body.visual(
        Box((0.012, 0.012, 0.008)),
        origin=Origin(xyz=(0.007, 0.0155, -0.002)),
        material=graphite,
        name="hinge_cheek_left",
    )
    lower_body.visual(
        Box((0.012, 0.012, 0.008)),
        origin=Origin(xyz=(0.007, -0.0155, -0.002)),
        material=graphite,
        name="hinge_cheek_right",
    )
    lower_body.visual(
        Box((0.012, 0.018, 0.004)),
        origin=Origin(xyz=(0.006, 0.0, -0.006)),
        material=graphite,
        name="hinge_cradle_bridge",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((0.112, 0.052, 0.022)),
        mass=0.085,
        origin=Origin(xyz=(0.056, 0.0, -0.006)),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        upper_shell_mesh,
        origin=Origin(xyz=(rear_offset + upper_length / 2.0, 0.0, 0.0080)),
        material=shell_silver,
        name="upper_shell",
    )
    upper_body.visual(
        Box((0.078, 0.039, 0.0016)),
        origin=Origin(xyz=(0.049, 0.0, 0.0033)),
        material=graphite,
        name="display_frame",
    )
    upper_body.visual(
        Box((0.060, 0.031, 0.0008)),
        origin=Origin(xyz=(0.051, 0.0, 0.0038)),
        material=screen_blue,
        name="display_glass",
    )
    upper_body.visual(
        Box((0.018, 0.0032, 0.0008)),
        origin=Origin(xyz=(0.081, 0.0, 0.0032)),
        material=keypad_gray,
        name="earpiece_slot",
    )
    upper_body.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    upper_body.visual(
        Box((0.011, 0.020, 0.006)),
        origin=Origin(xyz=(0.0055, 0.0, 0.0018)),
        material=graphite,
        name="hinge_spine",
    )
    upper_body.visual(
        antenna_sleeve_mesh,
        origin=Origin(
            xyz=(rear_offset + upper_length + (antenna_sleeve_length / 2.0), 0.013, 0.0080),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="antenna_sleeve",
    )
    upper_body.visual(
        Box((0.004, 0.008, 0.008)),
        origin=Origin(xyz=(rear_offset + upper_length - 0.002, 0.013, 0.0080)),
        material=graphite,
        name="antenna_base_block",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((0.108, 0.048, 0.022)),
        mass=0.065,
        origin=Origin(xyz=(0.050, 0.0, 0.006)),
    )

    antenna_mast = model.part("antenna_mast")
    antenna_mast.visual(
        Cylinder(radius=0.0021, length=mast_length),
        origin=Origin(xyz=(mast_length / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_metal,
        name="mast_stem",
    )
    antenna_mast.visual(
        Cylinder(radius=0.00244, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_metal,
        name="mast_guide",
    )
    antenna_mast.visual(
        Cylinder(radius=0.0028, length=0.008),
        origin=Origin(xyz=(0.037, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_metal,
        name="mast_grip",
    )
    antenna_mast.visual(
        Sphere(radius=0.003),
        origin=Origin(xyz=(mast_length, 0.0, 0.0)),
        material=antenna_metal,
        name="mast_tip",
    )
    antenna_mast.inertial = Inertial.from_geometry(
        Box((0.050, 0.008, 0.008)),
        mass=0.008,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )

    model.articulation(
        "body_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "antenna_slide",
        ArticulationType.PRISMATIC,
        parent=upper_body,
        child=antenna_mast,
        origin=Origin(xyz=(rear_offset + upper_length, 0.013, 0.0080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    antenna_mast = object_model.get_part("antenna_mast")
    body_hinge = object_model.get_articulation("body_hinge")
    antenna_slide = object_model.get_articulation("antenna_slide")

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

    ctx.expect_contact(
        upper_body,
        lower_body,
        elem_a="hinge_barrel",
        elem_b="hinge_cradle_bridge",
        contact_tol=0.0008,
        name="center hinge barrel seats in the lower cradle",
    )
    ctx.expect_contact(
        antenna_mast,
        upper_body,
        elem_a="mast_guide",
        elem_b="antenna_base_block",
        contact_tol=1e-6,
        name="antenna mast rests against the upper housing guide block",
    )

    with ctx.pose({body_hinge: 0.0}):
        ctx.expect_overlap(
            upper_body,
            lower_body,
            axes="xy",
            min_overlap=0.040,
            name="closed clamshell bodies share a tight footprint",
        )
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="z",
            positive_elem="display_frame",
            negative_elem="navigation_pad",
            min_gap=0.0008,
            max_gap=0.0030,
            name="closed display half clears the keypad crown",
        )
        ctx.expect_within(
            antenna_mast,
            upper_body,
            axes="yz",
            inner_elem="mast_stem",
            outer_elem="antenna_sleeve",
            margin=0.0010,
            name="retracted antenna stays centered in its sleeve",
        )
        ctx.expect_overlap(
            antenna_mast,
            upper_body,
            axes="x",
            elem_a="mast_stem",
            elem_b="antenna_sleeve",
            min_overlap=0.015,
            name="retracted antenna remains well engaged in the sleeve",
        )

    closed_display_center = _aabb_center(ctx.part_element_world_aabb(upper_body, elem="display_glass"))
    with ctx.pose({body_hinge: math.radians(90.0)}):
        half_open_display_center = _aabb_center(ctx.part_element_world_aabb(upper_body, elem="display_glass"))
    ctx.check(
        "upper body flips upward on the hinge axis",
        closed_display_center is not None
        and half_open_display_center is not None
        and half_open_display_center[2] > closed_display_center[2] + 0.030
        and half_open_display_center[0] < closed_display_center[0] - 0.040,
        details=f"closed={closed_display_center}, half_open={half_open_display_center}",
    )

    retracted_antenna_pos = ctx.part_world_position(antenna_mast)
    slide_upper = antenna_slide.motion_limits.upper if antenna_slide.motion_limits else None
    with ctx.pose({antenna_slide: 0.010 if slide_upper is None else slide_upper}):
        extended_antenna_pos = ctx.part_world_position(antenna_mast)
        ctx.expect_within(
            antenna_mast,
            upper_body,
            axes="yz",
            inner_elem="mast_stem",
            outer_elem="antenna_sleeve",
            margin=0.0010,
            name="extended antenna stays coaxial with the sleeve",
        )
        ctx.expect_overlap(
            antenna_mast,
            upper_body,
            axes="x",
            elem_a="mast_stem",
            elem_b="antenna_sleeve",
            min_overlap=0.006,
            name="extended antenna still retains insertion in the sleeve",
        )
    ctx.check(
        "antenna extends outward from the upper body's top edge",
        retracted_antenna_pos is not None
        and extended_antenna_pos is not None
        and extended_antenna_pos[0] > retracted_antenna_pos[0] + 0.008,
        details=f"retracted={retracted_antenna_pos}, extended={extended_antenna_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
