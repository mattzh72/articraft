from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _lofted_rounded_rect_mesh(
    name: str,
    *,
    lower_size: tuple[float, float],
    upper_size: tuple[float, float],
    height: float,
    lower_radius: float,
    upper_radius: float,
):
    lower_profile = rounded_rect_profile(
        lower_size[0],
        lower_size[1],
        lower_radius,
        corner_segments=6,
    )
    upper_profile = rounded_rect_profile(
        upper_size[0],
        upper_size[1],
        upper_radius,
        corner_segments=6,
    )
    return mesh_from_geometry(
        LoftGeometry(
            [
                [(x, y, 0.0) for x, y in lower_profile],
                [(x, y, height) for x, y in upper_profile],
            ],
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="grid_midi_pad_controller")

    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    shell_frame = model.material("shell_frame", rgba=(0.22, 0.23, 0.26, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.86, 0.86, 0.82, 1.0))
    fader_cap = model.material("fader_cap", rgba=(0.73, 0.74, 0.76, 1.0))
    fader_body = model.material("fader_body", rgba=(0.56, 0.58, 0.60, 1.0))

    width = 0.300
    depth = 0.300
    base_thickness = 0.005
    wall_thickness = 0.010
    wall_height = 0.013
    lip_thickness = 0.002
    lip_center_z = base_thickness + wall_height - lip_thickness * 0.5
    lip_underside_z = lip_center_z - lip_thickness * 0.5

    housing = model.part("housing")

    lower_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, 0.020, corner_segments=10),
            base_thickness,
            cap=True,
            closed=True,
        ),
        "controller_lower_shell",
    )
    housing.visual(lower_shell_mesh, material=shell_dark, name="lower_shell")

    wall_center_z = base_thickness + wall_height * 0.5
    housing.visual(
        Box((width, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -(depth * 0.5) + wall_thickness * 0.5, wall_center_z)),
        material=shell_dark,
        name="front_wall",
    )
    housing.visual(
        Box((width, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, (depth * 0.5) - wall_thickness * 0.5, wall_center_z)),
        material=shell_dark,
        name="rear_wall",
    )
    housing.visual(
        Box((wall_thickness, depth - (2.0 * wall_thickness), wall_height)),
        origin=Origin(xyz=(-(width * 0.5) + wall_thickness * 0.5, 0.0, wall_center_z)),
        material=shell_dark,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, depth - (2.0 * wall_thickness), wall_height)),
        origin=Origin(xyz=((width * 0.5) - wall_thickness * 0.5, 0.0, wall_center_z)),
        material=shell_dark,
        name="right_wall",
    )
    housing.visual(
        Box((width - (2.0 * wall_thickness), 0.010, wall_height)),
        origin=Origin(xyz=(0.0, -0.094, wall_center_z)),
        material=shell_dark,
        name="control_divider",
    )

    pad_bar_thickness = 0.004
    pad_opening = 0.024
    pad_pitch = pad_bar_thickness + pad_opening
    pad_grid_left = -0.114
    pad_grid_bottom = -0.086
    pad_grid_width = (8.0 * pad_opening) + (9.0 * pad_bar_thickness)
    pad_grid_depth = pad_grid_width

    housing.visual(
        Box((0.026, 0.236, lip_thickness)),
        origin=Origin(xyz=(-0.127, 0.028, lip_center_z)),
        material=shell_frame,
        name="pad_left_bridge",
    )
    housing.visual(
        Box((0.026, 0.236, lip_thickness)),
        origin=Origin(xyz=(0.127, 0.028, lip_center_z)),
        material=shell_frame,
        name="pad_right_bridge",
    )
    housing.visual(
        Box((0.280, 0.008, lip_thickness)),
        origin=Origin(xyz=(0.0, 0.146, lip_center_z)),
        material=shell_frame,
        name="pad_top_bridge",
    )
    housing.visual(
        Box((0.280, 0.006, lip_thickness)),
        origin=Origin(xyz=(0.0, -0.0865, lip_center_z)),
        material=shell_frame,
        name="pad_bottom_bridge",
    )

    for index in range(9):
        x_center = pad_grid_left + (pad_bar_thickness * 0.5) + (index * pad_pitch)
        housing.visual(
            Box((pad_bar_thickness, pad_grid_depth, lip_thickness)),
            origin=Origin(xyz=(x_center, 0.028, lip_center_z)),
            material=shell_frame,
        )

    for index in range(9):
        y_center = pad_grid_bottom + (pad_bar_thickness * 0.5) + (index * pad_pitch)
        housing.visual(
            Box((pad_grid_width, pad_bar_thickness, lip_thickness)),
            origin=Origin(xyz=(0.0, y_center, lip_center_z)),
            material=shell_frame,
        )

    slot_separator = 0.010
    slot_length = 0.018
    slot_width = 0.004
    slot_pitch = slot_separator + slot_length
    slot_total_width = (8.0 * slot_length) + (9.0 * slot_separator)
    slot_left = -0.117
    slot_center_y = -0.122

    housing.visual(
        Box((0.023, 0.022, lip_thickness)),
        origin=Origin(xyz=(-0.1285, slot_center_y, lip_center_z)),
        material=shell_frame,
        name="slot_left_bridge",
    )
    housing.visual(
        Box((0.023, 0.022, lip_thickness)),
        origin=Origin(xyz=(0.1285, slot_center_y, lip_center_z)),
        material=shell_frame,
        name="slot_right_bridge",
    )
    housing.visual(
        Box((0.280, 0.012, lip_thickness)),
        origin=Origin(xyz=(0.0, -0.134, lip_center_z)),
        material=shell_frame,
        name="slot_front_bridge",
    )
    housing.visual(
        Box((0.280, 0.022, lip_thickness)),
        origin=Origin(xyz=(0.0, -0.106, lip_center_z)),
        material=shell_frame,
        name="slot_rear_bridge",
    )

    housing.visual(
        Box((slot_total_width, 0.004, lip_thickness)),
        origin=Origin(xyz=(0.0, slot_center_y - 0.004, lip_center_z)),
        material=shell_frame,
        name="slot_front_lip",
    )
    housing.visual(
        Box((slot_total_width, 0.004, lip_thickness)),
        origin=Origin(xyz=(0.0, slot_center_y + 0.004, lip_center_z)),
        material=shell_frame,
        name="slot_rear_lip",
    )
    for index in range(9):
        x_center = slot_left + (slot_separator * 0.5) + (index * slot_pitch)
        housing.visual(
            Box((slot_separator, 0.012, lip_thickness)),
            origin=Origin(xyz=(x_center, slot_center_y, lip_center_z)),
            material=shell_frame,
        )

    housing.inertial = Inertial.from_geometry(
        Box((width, depth, base_thickness + wall_height)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, (base_thickness + wall_height) * 0.5)),
    )

    pad_cap_mesh = _lofted_rounded_rect_mesh(
        "pad_cap",
        lower_size=(0.0232, 0.0232),
        upper_size=(0.0218, 0.0218),
        height=0.006,
        lower_radius=0.0042,
        upper_radius=0.0036,
    )
    slider_cap_mesh = _lofted_rounded_rect_mesh(
        "slider_cap",
        lower_size=(0.013, 0.008),
        upper_size=(0.0115, 0.007),
        height=0.006,
        lower_radius=0.0024,
        upper_radius=0.0020,
    )

    for row in range(8):
        pad_y = pad_grid_bottom + pad_bar_thickness + (pad_opening * 0.5) + (row * pad_pitch)
        for col in range(8):
            pad_x = pad_grid_left + pad_bar_thickness + (pad_opening * 0.5) + (col * pad_pitch)
            pad = model.part(f"pad_r{row}_c{col}")
            pad.visual(
                pad_cap_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.002)),
                material=pad_rubber,
                name="pad_cap",
            )
            pad.visual(
                Box((0.010, 0.010, 0.009)),
                origin=Origin(xyz=(0.0, 0.0, -0.0025)),
                material=pad_rubber,
                name="pad_stem",
            )
            pad.visual(
                Box((0.026, 0.026, 0.002)),
                origin=Origin(xyz=(0.0, 0.0, -0.001)),
                material=pad_rubber,
                name="pad_retainer",
            )
            pad.inertial = Inertial.from_geometry(
                Box((0.026, 0.026, 0.010)),
                mass=0.018,
                origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            )
            model.articulation(
                f"housing_to_pad_r{row}_c{col}",
                ArticulationType.PRISMATIC,
                parent=housing,
                child=pad,
                origin=Origin(xyz=(pad_x, pad_y, lip_underside_z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=4.0,
                    velocity=0.060,
                    lower=0.0,
                    upper=0.003,
                ),
            )

    for index in range(8):
        fader_x = slot_left + slot_separator + (slot_length * 0.5) + (index * slot_pitch)
        fader = model.part(f"fader_{index}")
        fader.visual(
            slider_cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0005)),
            material=fader_cap,
            name="fader_cap",
        )
        fader.visual(
            Box((0.004, 0.0026, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=fader_body,
            name="fader_stem",
        )
        fader.visual(
            Box((0.010, 0.008, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=fader_body,
            name="fader_carriage",
        )
        fader.inertial = Inertial.from_geometry(
            Box((0.014, 0.008, 0.008)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
        )
        model.articulation(
            f"housing_to_fader_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(fader_x, slot_center_y, lip_underside_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.050,
                lower=-0.004,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")

    for row in range(8):
        for col in range(8):
            pad = object_model.get_part(f"pad_r{row}_c{col}")
            ctx.expect_contact(
                pad,
                housing,
                contact_tol=0.0005,
                name=f"pad_r{row}_c{col} is mounted in the lattice",
            )

    for index in range(8):
        fader = object_model.get_part(f"fader_{index}")
        ctx.expect_contact(
            fader,
            housing,
            contact_tol=0.0005,
            name=f"fader_{index} is captured in its slot",
        )

    pad_pitch_ok = True
    pad_row_positions = [ctx.part_world_position(object_model.get_part(f"pad_r0_c{col}")) for col in range(8)]
    pad_col_positions = [ctx.part_world_position(object_model.get_part(f"pad_r{row}_c0")) for row in range(8)]
    if any(pos is None for pos in pad_row_positions + pad_col_positions):
        pad_pitch_ok = False
    else:
        row_deltas = [pad_row_positions[i + 1][0] - pad_row_positions[i][0] for i in range(7)]
        col_deltas = [pad_col_positions[i + 1][1] - pad_col_positions[i][1] for i in range(7)]
        pad_pitch_ok = all(0.027 <= delta <= 0.029 for delta in row_deltas) and all(
            0.027 <= delta <= 0.029 for delta in col_deltas
        )
    ctx.check(
        "pad grid keeps an even 8x8 pitch",
        pad_pitch_ok,
        details=f"row_positions={pad_row_positions}, col_positions={pad_col_positions}",
    )

    fader_positions = [ctx.part_world_position(object_model.get_part(f"fader_{index}")) for index in range(8)]
    fader_spacing_ok = True
    if any(pos is None for pos in fader_positions):
        fader_spacing_ok = False
    else:
        x_deltas = [fader_positions[i + 1][0] - fader_positions[i][0] for i in range(7)]
        y_spread = max(pos[1] for pos in fader_positions) - min(pos[1] for pos in fader_positions)
        fader_spacing_ok = all(0.027 <= delta <= 0.029 for delta in x_deltas) and y_spread <= 0.001
    ctx.check(
        "faders form a straight bottom row",
        fader_spacing_ok,
        details=f"fader_positions={fader_positions}",
    )

    pad_joint = object_model.get_articulation("housing_to_pad_r0_c0")
    pad = object_model.get_part("pad_r0_c0")
    pad_rest = ctx.part_world_position(pad)
    with ctx.pose({pad_joint: 0.003}):
        pad_pressed = ctx.part_world_position(pad)
    ctx.check(
        "pad articulation compresses downward",
        pad_rest is not None
        and pad_pressed is not None
        and pad_pressed[2] < pad_rest[2] - 0.0025,
        details=f"rest={pad_rest}, pressed={pad_pressed}",
    )

    fader_joint = object_model.get_articulation("housing_to_fader_0")
    fader = object_model.get_part("fader_0")
    fader_rest = ctx.part_world_position(fader)
    with ctx.pose({fader_joint: 0.004}):
        fader_right = ctx.part_world_position(fader)
    ctx.check(
        "fader articulation slides horizontally",
        fader_rest is not None
        and fader_right is not None
        and fader_right[0] > fader_rest[0] + 0.0035
        and abs(fader_right[1] - fader_rest[1]) <= 0.0005
        and abs(fader_right[2] - fader_rest[2]) <= 0.0005,
        details=f"rest={fader_rest}, moved={fader_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
