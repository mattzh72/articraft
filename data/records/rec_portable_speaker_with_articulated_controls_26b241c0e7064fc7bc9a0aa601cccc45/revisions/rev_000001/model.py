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
    tube_from_spline_points,
)


def _rounded_cabinet_mesh(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            depth,
            cap=True,
            center=True,
        ).rotate_x(math.pi / 2.0),
        name,
    )


def _handle_arch_mesh(span: float, rise: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (-span, 0.018, 0.004),
                (-(span * 0.88), 0.044, rise * 0.34),
                (-(span * 0.70), 0.072, rise * 0.72),
                (-(span * 0.38), 0.088, rise * 0.94),
                (0.0, 0.094, rise),
                (span * 0.38, 0.088, rise * 0.94),
                (span * 0.70, 0.072, rise * 0.72),
                (span * 0.88, 0.044, rise * 0.34),
                (span, 0.018, 0.004),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_boombox_speaker")

    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    grille_black = model.material("grille_black", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.68, 0.70, 0.72, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.18, 0.19, 0.20, 1.0))
    dial_glass = model.material("dial_glass", rgba=(0.30, 0.48, 0.56, 0.55))

    body_width = 0.44
    body_depth = 0.17
    body_height = 0.24
    cabinet_radius = 0.028
    front_face_y = -(body_depth * 0.5)
    half_width = body_width * 0.5
    pivot_axis_z = 0.086
    panel_depth = 0.010

    enclosure = model.part("enclosure")
    enclosure.visual(
        _rounded_cabinet_mesh(
            body_width,
            body_depth,
            body_height,
            cabinet_radius,
            "boombox_cabinet_shell",
        ),
        material=shell_dark,
        name="cabinet_shell",
    )
    enclosure.visual(
        _rounded_cabinet_mesh(
            body_width - 0.030,
            0.008,
            body_height - 0.050,
            0.022,
            "boombox_front_baffle",
        ),
        origin=Origin(xyz=(0.0, front_face_y - 0.004, 0.0)),
        material=panel_black,
        name="front_baffle",
    )
    enclosure.visual(
        Cylinder(radius=0.073, length=0.010),
        origin=Origin(
            xyz=(-0.125, front_face_y - 0.005, -0.030),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_black,
        name="left_speaker_grille",
    )
    enclosure.visual(
        Cylinder(radius=0.073, length=0.010),
        origin=Origin(
            xyz=(0.105, front_face_y - 0.005, -0.030),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_black,
        name="right_speaker_grille",
    )
    enclosure.visual(
        Box((0.205, panel_depth, 0.078)),
        origin=Origin(xyz=(0.068, front_face_y - (panel_depth * 0.5), 0.058)),
        material=panel_black,
        name="control_panel",
    )
    enclosure.visual(
        Box((0.086, 0.004, 0.020)),
        origin=Origin(xyz=(0.006, front_face_y - panel_depth - 0.002, 0.071)),
        material=dial_glass,
        name="tuning_scale",
    )
    enclosure.visual(
        Box((0.016, 0.004, 0.010)),
        origin=Origin(xyz=(-0.018, front_face_y - panel_depth - 0.002, 0.044)),
        material=trim_silver,
        name="preset_button_left",
    )
    enclosure.visual(
        Box((0.016, 0.004, 0.010)),
        origin=Origin(xyz=(0.004, front_face_y - panel_depth - 0.002, 0.044)),
        material=trim_silver,
        name="preset_button_center",
    )
    enclosure.visual(
        Box((0.016, 0.004, 0.010)),
        origin=Origin(xyz=(0.026, front_face_y - panel_depth - 0.002, 0.044)),
        material=trim_silver,
        name="preset_button_right",
    )
    enclosure.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(
            xyz=(half_width + 0.004, 0.0, pivot_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_silver,
        name="right_pivot_cap",
    )
    enclosure.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(
            xyz=(-(half_width + 0.004), 0.0, pivot_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_silver,
        name="left_pivot_cap",
    )
    enclosure.visual(
        Box((0.050, 0.018, 0.012)),
        origin=Origin(xyz=(-0.145, 0.0, -(body_height * 0.5) + 0.006)),
        material=grip_rubber,
        name="left_foot",
    )
    enclosure.visual(
        Box((0.050, 0.018, 0.012)),
        origin=Origin(xyz=(0.145, 0.0, -(body_height * 0.5) + 0.006)),
        material=grip_rubber,
        name="right_foot",
    )
    enclosure.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=4.8,
    )

    carry_handle = model.part("carry_handle")
    handle_span = half_width + 0.023
    handle_rise = 0.134
    hub_length = 0.022
    carry_handle.visual(
        _handle_arch_mesh(handle_span, handle_rise, "boombox_carry_handle_arch"),
        material=handle_black,
        name="handle_arch",
    )
    carry_handle.visual(
        Cylinder(radius=0.015, length=hub_length),
        origin=Origin(
            xyz=(-handle_span, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_silver,
        name="left_hub",
    )
    carry_handle.visual(
        Cylinder(radius=0.015, length=hub_length),
        origin=Origin(
            xyz=(handle_span, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_silver,
        name="right_hub",
    )
    carry_handle.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(
            xyz=(0.0, 0.086, 0.129),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=grip_rubber,
        name="handle_grip",
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((body_width + 0.070, 0.040, handle_rise + 0.030)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    tuning_knob = model.part("tuning_knob")
    shaft_length = 0.012
    knob_length = 0.028
    face_length = 0.006
    tuning_knob.visual(
        Cylinder(radius=0.010, length=shaft_length),
        origin=Origin(
            xyz=(0.0, -(shaft_length * 0.5), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_silver,
        name="knob_shaft",
    )
    tuning_knob.visual(
        Cylinder(radius=0.040, length=knob_length),
        origin=Origin(
            xyz=(0.0, -(shaft_length + (knob_length * 0.5)), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_black,
        name="knob_body",
    )
    tuning_knob.visual(
        Cylinder(radius=0.031, length=face_length),
        origin=Origin(
            xyz=(0.0, -(shaft_length + knob_length + (face_length * 0.5)), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_silver,
        name="knob_face",
    )
    tuning_knob.visual(
        Box((0.008, 0.004, 0.018)),
        origin=Origin(
            xyz=(0.0, -(shaft_length + knob_length + face_length + 0.002), 0.020),
        ),
        material=trim_silver,
        name="indicator",
    )
    tuning_knob.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.090)),
        mass=0.09,
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
    )

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.0, pivot_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.20,
        ),
    )
    model.articulation(
        "tuning_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=tuning_knob,
        origin=Origin(xyz=(0.138, front_face_y - panel_depth, 0.056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=7.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    enclosure = object_model.get_part("enclosure")
    carry_handle = object_model.get_part("carry_handle")
    tuning_knob = object_model.get_part("tuning_knob")
    handle_pivot = object_model.get_articulation("handle_pivot")
    tuning_knob_spin = object_model.get_articulation("tuning_knob_spin")

    ctx.expect_contact(
        carry_handle,
        enclosure,
        elem_a="left_hub",
        elem_b="left_pivot_cap",
        name="left handle hub contacts left pivot cap",
    )
    ctx.expect_contact(
        carry_handle,
        enclosure,
        elem_a="right_hub",
        elem_b="right_pivot_cap",
        name="right handle hub contacts right pivot cap",
    )
    ctx.expect_contact(
        tuning_knob,
        enclosure,
        elem_a="knob_shaft",
        elem_b="control_panel",
        name="tuning knob shaft seats on control panel",
    )
    ctx.expect_overlap(
        tuning_knob,
        enclosure,
        axes="xz",
        elem_a="knob_body",
        elem_b="control_panel",
        min_overlap=0.050,
        name="tuning knob body stays over control panel footprint",
    )

    rest_handle_aabb = ctx.part_world_aabb(carry_handle)
    lower_handle_limit = handle_pivot.motion_limits.lower if handle_pivot.motion_limits is not None else None
    stowed_handle_aabb = None
    if lower_handle_limit is not None:
        with ctx.pose({handle_pivot: lower_handle_limit}):
            stowed_handle_aabb = ctx.part_world_aabb(carry_handle)
    ctx.check(
        "carry handle lowers when pivoted to stowed pose",
        rest_handle_aabb is not None
        and stowed_handle_aabb is not None
        and rest_handle_aabb[1][2] > stowed_handle_aabb[1][2] + 0.035,
        details=f"rest={rest_handle_aabb}, stowed={stowed_handle_aabb}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    pointer_rest = _aabb_center(ctx.part_element_world_aabb(tuning_knob, elem="indicator"))
    knob_origin_rest = ctx.part_world_position(tuning_knob)
    with ctx.pose({tuning_knob_spin: math.pi / 2.0}):
        pointer_turned = _aabb_center(ctx.part_element_world_aabb(tuning_knob, elem="indicator"))
        knob_origin_turned = ctx.part_world_position(tuning_knob)
    ctx.check(
        "tuning knob indicator sweeps around the knob axis",
        pointer_rest is not None
        and pointer_turned is not None
        and knob_origin_rest is not None
        and knob_origin_turned is not None
        and abs(pointer_rest[0] - pointer_turned[0]) > 0.012
        and abs(pointer_rest[2] - pointer_turned[2]) > 0.012
        and abs(knob_origin_rest[1] - knob_origin_turned[1]) < 1e-6,
        details=(
            f"pointer_rest={pointer_rest}, pointer_turned={pointer_turned}, "
            f"origin_rest={knob_origin_rest}, origin_turned={knob_origin_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
